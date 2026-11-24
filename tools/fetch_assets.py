#!/usr/bin/env python3
"""Fetch only a chosen Amazon Warehouse scene and its model dependencies.

Python 3.8+, standard library only. Does not install software or run downloaded code.
The repository tree is metadata; unrelated repository file contents are not fetched.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import sys
import subprocess
import tempfile
import time
from typing import Dict, List, Optional, Set
from urllib.error import HTTPError, URLError
from urllib.parse import quote, unquote, urlparse
from urllib.request import Request, urlopen
import xml.etree.ElementTree as ET

REPOSITORY = "JdeRobot/RoboticsInfrastructure"
COMMIT = "2855323b2e33fcfc42fcd7c7bb63fedd5cdb2ebb"
PACKAGE = "trsa_warehouse"
ROOT = Path(__file__).resolve().parents[1]
MODEL_URI = re.compile(r"model://([A-Za-z0-9_.-]+)")
RUNTIME_EXT = {
    ".sdf", ".config", ".dae", ".stl", ".obj", ".mtl", ".mesh",
    ".png", ".jpg", ".jpeg", ".tga", ".dds", ".bmp", ".tif", ".tiff",
    ".material", ".urdf", ".xacro", ".glb", ".gltf", ".bin", ".exr", ".hdr",
}
TEXT_EXT = {".world", ".sdf", ".config", ".dae", ".obj", ".mtl", ".material", ".urdf", ".xacro", ".gltf"}
LICENSE_NAME = re.compile(r"^(license|licence|copying|notice|copyright|attribution)([._-].*)?$", re.I)


def blob_sha(data: bytes) -> str:
    return hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()


def safe_path(path: str) -> str:
    p = PurePosixPath(path)
    if p.is_absolute() or ".." in p.parts or "\\" in path:
        raise ValueError("Unsafe repository path: " + path)
    return str(p)


def request_bytes(url: str) -> bytes:
    headers = {"User-Agent": "amazon-warehouse-standalone/1.0"}
    # A token is optional and is sent ONLY to GitHub's API host.
    if urlparse(url).hostname == "api.github.com":
        headers["Accept"] = "application/vnd.github+json"
        if os.environ.get("GITHUB_TOKEN"):
            headers["Authorization"] = "Bearer " + os.environ["GITHUB_TOKEN"]
    for attempt in range(4):
        try:
            with urlopen(Request(url, headers=headers), timeout=90) as response:
                return response.read()
        except HTTPError as exc:
            if exc.code in (403, 429):
                raise RuntimeError(
                    "GitHub denied/rate-limited this request. Retry later or provide "
                    "a read-only GITHUB_TOKEN in your environment. URL: " + url
                ) from exc
            if exc.code not in (500, 502, 503, 504) or attempt == 3:
                raise RuntimeError("HTTP %s fetching %s" % (exc.code, url)) from exc
        except (URLError, TimeoutError, OSError) as exc:
            if attempt == 3:
                raise RuntimeError("Cannot download %s: %s" % (url, exc)) from exc
        time.sleep(2 ** attempt)
    raise RuntimeError("Download retries exhausted: " + url)


class Source:
    def __init__(self, root: Path, local_repo: Optional[Path] = None):
        self.root = root
        self.local_repo = local_repo
        cache = root / ".download-cache"
        cache.mkdir(exist_ok=True)
        self.cache = cache
        tree_file = cache / (COMMIT + "-tree.json")
        if local_repo is not None:
            # Read the committed Git objects, NOT a potentially modified working tree.
            result = subprocess.run(
                ["git", "-C", str(local_repo), "ls-tree", "-r", "-l", "--full-tree", COMMIT],
                check=True, capture_output=True, text=True,
            )
            entries = []
            for line in result.stdout.splitlines():
                info, path = line.split("\t", 1)
                mode, kind, sha, size = info.split()
                entries.append({"path": path, "mode": mode, "type": kind,
                                "sha": sha, "size": int(size) if size != "-" else 0})
            tree = {"truncated": False, "tree": entries}
        elif tree_file.exists():
            tree = json.loads(tree_file.read_text())
        else:
            print("Reading pinned repository file index (metadata only)...", flush=True)
            url = "https://api.github.com/repos/%s/git/trees/%s?recursive=1" % (REPOSITORY, COMMIT)
            data = request_bytes(url)
            tree = json.loads(data)
            if not isinstance(tree.get("tree"), list) or tree.get("truncated"):
                raise RuntimeError("GitHub returned an incomplete file index; refusing a partial extraction.")
            tree_file.write_bytes(data)
        if tree.get("truncated") or not isinstance(tree.get("tree"), list):
            raise RuntimeError("Invalid cached file index. Remove .download-cache and retry.")
        self.files: Dict[str, dict] = {
            safe_path(e["path"]): e for e in tree["tree"] if e["type"] == "blob"
        }
        self.origins: Dict[str, dict] = {}

    def read(self, path: str) -> bytes:
        path = safe_path(path)
        entry = self.files.get(path)
        if entry is None:
            raise RuntimeError("File is absent from the pinned source: " + path)
        if entry.get("mode") == "120000":
            raise RuntimeError("A source symlink needs explicit handling: " + path)
        candidates = [self.root / "upstream_seed" / path, self.cache / "blobs" / entry["sha"]]
        data: Optional[bytes] = None
        for candidate in candidates:
            if candidate.is_file():
                b = candidate.read_bytes()
                if blob_sha(b) == entry["sha"]:
                    data = b
                    break
        if data is None and self.local_repo is not None:
            data = subprocess.run(
                ["git", "-C", str(self.local_repo), "cat-file", "blob", COMMIT + ":" + path],
                check=True, capture_output=True,
            ).stdout
            if blob_sha(data) != entry["sha"]:
                raise RuntimeError("Local Git object checksum mismatch: " + path)
        if data is None:
            url = "https://raw.githubusercontent.com/%s/%s/%s" % (REPOSITORY, COMMIT, quote(path, safe="/"))
            print("  " + path, flush=True)
            data = request_bytes(url)
            if blob_sha(data) != entry["sha"]:
                raise RuntimeError("Git blob checksum mismatch: " + path)
            candidates[1].parent.mkdir(parents=True, exist_ok=True)
            candidates[1].write_bytes(data)
        origin = {"repository_path": path, "git_blob_sha1": entry["sha"]}
        if data.startswith(b"version https://git-lfs.github.com/spec/v1"):
            pointer = data.decode("ascii")
            oid = re.search(r"^oid sha256:([0-9a-f]{64})$", pointer, re.M)
            size = re.search(r"^size ([0-9]+)$", pointer, re.M)
            if not oid or not size:
                raise RuntimeError("Malformed Git LFS pointer: " + path)
            lfs_file = self.cache / "lfs" / oid.group(1)
            if lfs_file.exists():
                data = lfs_file.read_bytes()
            elif self.local_repo is not None:
                common = subprocess.run(
                    ["git", "-C", str(self.local_repo), "rev-parse", "--git-common-dir"],
                    check=True, capture_output=True, text=True,
                ).stdout.strip()
                git_dir = Path(common)
                if not git_dir.is_absolute(): git_dir = self.local_repo / git_dir
                digest = oid.group(1)
                candidates = [git_dir / "lfs" / "objects" / digest[:2] / digest[2:4] / digest,
                              self.local_repo / path]
                data = None
                for candidate in candidates:
                    if candidate.is_file():
                        candidate_data = candidate.read_bytes()
                        if hashlib.sha256(candidate_data).hexdigest() == digest:
                            data = candidate_data
                            break
                if data is None:
                    raise RuntimeError("Local Git LFS object is missing for " + path +
                                       ". Fetch the pinned commit's LFS objects before exporting.")
            else:
                data = request_bytes(
                    "https://media.githubusercontent.com/media/%s/%s/%s" % (REPOSITORY, COMMIT, quote(path, safe="/"))
                )
            if hashlib.sha256(data).hexdigest() != oid.group(1) or len(data) != int(size.group(1)):
                raise RuntimeError("Git LFS checksum or size mismatch: " + path)
            lfs_file.parent.mkdir(parents=True, exist_ok=True)
            lfs_file.write_bytes(data)
            origin["lfs_sha256"] = oid.group(1)
        origin["upstream_content_sha256"] = hashlib.sha256(data).hexdigest()
        self.origins[path] = origin
        return data

    def model_root(self, name: str) -> str:
        # Prefer the exercise's own assets, then its shared AWS warehouse assets.
        candidates = [
            "CustomRobots/amazon_warehouse_harmonic/models/" + name,
            "CustomRobots/package_delivery/models/" + name,
            "CustomRobots/" + name + "/models/" + name,
        ]
        for candidate in candidates:
            if any(p.startswith(candidate + "/") for p in self.files):
                return candidate
        matches = sorted({
            str(PurePosixPath(p).parent)
            for p in self.files
            if p.startswith("CustomRobots/") and p.endswith("/" + name + "/model.config")
        })
        if len(matches) == 1:
            return matches[0]
        raise RuntimeError("Cannot resolve model '%s' unambiguously: %s" % (name, matches))


def included_models(data: bytes) -> Set[str]:
    return set(MODEL_URI.findall(data.decode("utf-8")))


def repair_dae_textures(folder: Path, changes: List[str]) -> None:
    """Repair stale authoring-machine paths ONLY when a unique local texture exists."""
    for dae in folder.rglob("*.dae"):
        tree = ET.parse(dae)
        changed = False
        for image in tree.getroot().iter():
            if image.tag.split("}")[-1] != "image":
                continue
            for node in image.iter():
                if node.tag.split("}")[-1] != "init_from" or not node.text:
                    continue
                ref = unquote(node.text.strip()).replace("\\", "/")
                if ref.startswith("model://"):
                    target = folder.parent / ref[len("model://"):]
                    if not target.is_file():
                        raise RuntimeError("Unresolved DAE model texture: " + ref)
                    continue
                ref = ref.removeprefix("file://") if hasattr(ref, "removeprefix") else (ref[7:] if ref.startswith("file://") else ref)
                target = dae.parent / ref
                if target.is_file() and not Path(ref).is_absolute():
                    continue
                basename = PurePosixPath(ref).name
                matches = [p for p in folder.rglob(basename) if p.is_file()] if basename else []
                if len(matches) != 1:
                    raise RuntimeError("Unresolved/ambiguous texture '%s' in %s" % (ref, dae))
                node.text = os.path.relpath(matches[0], dae.parent).replace(os.sep, "/")
                changes.append("Repaired texture path in %s: %s -> %s" % (dae.name, ref, node.text))
                changed = True
        if changed:
            tree.write(dae, encoding="utf-8", xml_declaration=True)


def validate_models(assets: Path) -> None:
    models = assets / "models"
    for file in assets.rglob("*"):
        if not file.is_file() or file.suffix.lower() not in TEXT_EXT:
            continue
        text = file.read_text(encoding="utf-8")
        for match in re.finditer(r"model://([^\s<>'\"]+)", text):
            rel = unquote(match.group(1))
            if not (models / safe_path(rel)).exists():
                raise RuntimeError("Unresolved resource in %s: model://%s" % (file, rel))
        if file.suffix.lower() in {".world", ".sdf", ".xacro", ".urdf"} or file.name == "model.config":
            root = ET.fromstring(text)
            if file.name == "model.config":
                for sdf in root.findall("sdf"):
                    if sdf.text and not (file.parent / safe_path(sdf.text.strip())).is_file():
                        raise RuntimeError("model.config references an absent SDF: " + str(file))
            for plugin in root.iter("plugin"):
                library = plugin.get("filename", "")
                if library and not library.startswith("gz-sim-"):
                    raise RuntimeError("Non-stock Gazebo plugin needs explicit handling: " + library)


def populate(source: Source, assets: Path, worlds: List[str], robot: str) -> dict:
    records: Dict[str, dict] = {}
    changes: List[str] = []
    pending: Set[str] = set()
    done: Dict[str, str] = {}

    def copy(src: str, rel: str) -> bytes:
        data = source.read(src)
        dest = assets / safe_path(rel)
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes(data)
        records[rel] = dict(source.origins[src])
        return data

    copy("LICENSE", "licenses/RoboticsInfrastructure-LICENSE")
    for number in worlds:
        data = copy("Scenes/warehouse%s.world" % number, "worlds/warehouse%s.world" % number)
        pending.update(included_models(data))
    if robot == "holonomic":
        pending.add("logistic_holonomic_robot")

    while pending:
        name = sorted(pending)[0]
        pending.remove(name)
        if name in done:
            continue
        src_root = source.model_root(name)
        done[name] = src_root
        print("Resolving model: " + name, flush=True)
        members = sorted(p for p in source.files if p.startswith(src_root + "/"))
        for p in members:
            leaf = PurePosixPath(p)
            if leaf.suffix.lower() not in RUNTIME_EXT and not LICENSE_NAME.match(leaf.name):
                continue
            rel = "models/" + name + "/" + p[len(src_root) + 1:]
            data = copy(p, rel)
            if leaf.suffix.lower() in TEXT_EXT:
                pending.update(included_models(data) - done.keys())
                if name == "logistic_holonomic_robot":
                    text = data.decode("utf-8")
                    patched = text.replace("$(find custom_robots)", "$(find %s)/assets" % PACKAGE)
                    patched = patched.replace("package://custom_robots/", "package://%s/assets/" % PACKAGE)
                    if patched != text:
                        (assets / rel).write_text(patched, encoding="utf-8")
                        changes.append("Rebased custom_robots paths in " + rel)
        # Preserve license/notice files at every relevant ancestor, not other exercises.
        parents = {str(p) for p in PurePosixPath(src_root).parents} | {src_root}
        for p in sorted(source.files):
            leaf = PurePosixPath(p)
            if str(leaf.parent) in parents and LICENSE_NAME.match(leaf.name):
                rel = "licenses/upstream/" + p
                if rel not in records:
                    copy(p, rel)
    for name in sorted(done):
        repair_dae_textures(assets / "models" / name, changes)
    validate_models(assets)
    for rel, record in records.items():
        data = (assets / rel).read_bytes()
        record["bytes"] = len(data)
        record["local_sha256"] = hashlib.sha256(data).hexdigest()
    manifest = {
        "schema_version": 2, "repository": REPOSITORY, "commit": COMMIT, "ros_package": PACKAGE,
        "worlds": worlds, "robot": robot, "target_gazebo": "Harmonic (gz-sim 8)",
        "model_sources": done, "changes": changes,
        "files": records, "total_asset_bytes": sum(r["bytes"] for r in records.values()),
        "runtime_tested": False,
    }
    (assets / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def verify(assets: Path) -> dict:
    manifest_file = assets / "manifest.json"
    if not manifest_file.is_file():
        raise RuntimeError("Assets are not ready. Run scripts/fetch_assets.py first.")
    manifest = json.loads(manifest_file.read_text())
    if manifest.get("commit") != COMMIT or manifest.get("repository") != REPOSITORY or manifest.get("ros_package") != PACKAGE:
        raise RuntimeError("Asset manifest targets a different source revision/package. Regenerate with this project's downloader.")
    for rel, expected in manifest["files"].items():
        p = assets / safe_path(rel)
        if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest() != expected["local_sha256"]:
            raise RuntimeError("Missing or locally modified asset: " + str(p))
    validate_models(assets)
    print("Verified %s asset files; commit %s." % (len(manifest["files"]), manifest["commit"]))
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", choices=["1", "2", "all"], default="1")
    parser.add_argument("--robot", choices=["none", "holonomic"], default="none")
    parser.add_argument("--local-repo", type=Path, help="Extract the pinned commit from an existing full Git clone")
    parser.add_argument("--output", type=Path, help="Destination asset directory (default: vendor/assets)")
    parser.add_argument("--verify", action="store_true", help="Offline checksum and resource-reference check; no downloads")
    parser.add_argument("--replace", action="store_true", help="Replace a previous managed asset selection, retaining a backup")
    args = parser.parse_args()
    target = args.output.resolve() if args.output else ROOT / "vendor" / "assets"
    target.parent.mkdir(parents=True, exist_ok=True)
    if args.verify:
        verify(target)
        return 0
    worlds = ["1", "2"] if args.world == "all" else [args.world]
    if target.exists() and not args.replace:
        manifest = verify(target)
        if manifest["worlds"] == worlds and manifest["robot"] == args.robot:
            print("Requested assets are already present. No download needed.")
            return 0
        raise RuntimeError("A different asset selection exists. Use --replace to retain it as a backup and fetch the new selection.")
    if target.exists() and not (target / "manifest.json").is_file():
        raise RuntimeError("Refusing to replace an unmanaged assets directory.")
    source = Source(ROOT, args.local_repo)
    with tempfile.TemporaryDirectory(prefix=".warehouse-staging-", dir=str(target.parent)) as temp:
        staged = Path(temp) / "assets"
        staged.mkdir()
        manifest = populate(source, staged, worlds, args.robot)
        if target.exists():
            backup = target.parent / ("assets.backup." + time.strftime("%Y%m%d-%H%M%S"))
            if backup.exists():
                raise RuntimeError("Backup path already exists: " + str(backup))
            target.rename(backup)
            print("Previous assets retained at " + str(backup))
        staged.rename(target)
    print("\nReady: %s model folders; %.1f MiB of assets." % (len(manifest["model_sources"]), manifest["total_asset_bytes"] / 1048576))
    print("Assets are ready for: docker compose build")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, OSError, subprocess.CalledProcessError, ET.ParseError, UnicodeError) as error:
        print("ERROR: " + str(error), file=sys.stderr)
        raise SystemExit(1)
