"""ROS-independent configuration validation shared by launch and tests."""
from pathlib import Path
import math
import yaml

NAV_NODES=('controller_server','smoother_server','planner_server','behavior_server',
           'bt_navigator','waypoint_follower','velocity_smoother','collision_monitor')
MODES=('slam','localization','off')


def config_file(share,value,default):
    p=Path(value).expanduser() if value else Path(share)/'config'/default
    if not p.is_file(): raise FileNotFoundError(f'Configuration file not found in this environment: {p}')
    data=yaml.safe_load(p.read_text())
    if not isinstance(data,dict): raise ValueError(f'Expected YAML mapping: {p}')
    return str(p.resolve())


def validate_map(value,world):
    if not value: raise ValueError('Localization requires an explicit map path; pass map:=/absolute/path/to/map.yaml to your localisation launch')
    p=Path(value).expanduser()
    if not p.is_file(): raise FileNotFoundError(f'Map YAML not found in this environment: {p}')
    data=yaml.safe_load(p.read_text())
    if not isinstance(data,dict) or not isinstance(data.get('image'),str):
        raise ValueError('Map YAML must contain an image filename')
    image=Path(data['image'])
    if not image.is_absolute(): image=p.parent/image
    if not image.is_file(): raise FileNotFoundError(f'Map image missing: {image}')
    resolution=data.get('resolution',0)
    origin=data.get('origin',[])
    if not isinstance(resolution,(int,float)) or not math.isfinite(resolution) or resolution<=0:
        raise ValueError('Map resolution must be finite and positive')
    if len(origin)!=3 or not all(isinstance(x,(int,float)) and math.isfinite(x) for x in origin):
        raise ValueError('Map origin must be [x, y, yaw] in metres/radians')
    metadata=p.with_suffix('.trsa.yaml')
    if metadata.exists():
        info=yaml.safe_load(metadata.read_text()) or {}
        if str(info.get('warehouse_world',world))!=str(world):
            raise ValueError('Map belongs to a different warehouse. Use a map for this warehouse.')
    return str(p.resolve())
