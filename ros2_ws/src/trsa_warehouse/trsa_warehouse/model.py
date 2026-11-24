"""Prepare the upstream logistics robot for a standard ROS 2 teaching interface.

The downloaded upstream sources stay unchanged. All changes are made in temporary
runtime XML. This module's structural functions also run without ROS or Gazebo.
"""
from __future__ import annotations
import atexit
import os
from collections.abc import Mapping
from pathlib import Path
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROBOT = 'logistic_robot'
WORLD = 'amazon_warehouse'

# Single-robot teaching scene: keep robot visuals visible in the GUI, but
# exclude them from its GPU lidar. Bit 4 avoids Gazebo's GUI/selectable bits.
ROBOT_VISIBILITY_FLAGS = 1 << 4                 # 16
LIDAR_VISIBILITY_MASK = 0xFFFFFFFF ^ ROBOT_VISIBILITY_FLAGS  # 4294967279


def resource_environment(package: Path, base: Mapping[str, str] | None = None) -> dict[str, str]:
    """Give both Gazebo Sim and the standalone SDFormat CLI the model root.

    ``gz sim`` resolves model:// resources through GZ_SIM_RESOURCE_PATH.
    ``gz sdf`` is an SDFormat command, not the simulator, and its findFile()
    implementation searches SDF_PATH instead. Keeping both in the child
    environment also covers recursive includes such as warehouse 2's rack sets.
    The caller's environment and the downloaded model files are not modified.
    """
    models = Path(package).resolve() / 'assets' / 'models'
    if not models.is_dir():
        raise FileNotFoundError(f'Model asset directory does not exist: {models}')
    env = dict(os.environ if base is None else base)
    for key in ('GZ_SIM_RESOURCE_PATH', 'SDF_PATH'):
        paths = [str(models)] + env.get(key, '').split(os.pathsep)
        env[key] = os.pathsep.join(dict.fromkeys(p for p in paths if p))
    return env

def sub(parent, tag, text=None, **attrs):
    e = ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})
    if text is not None:
        e.text = str(text)
    return e

def plugin(parent, library, name, params):
    p = sub(parent, 'plugin', filename=library, name=name)
    for key, value in params:
        sub(p, key, value)
    return p

def set_box_inertia(link, mass, size, center=(0, 0, 0)):
    inertial = link.find('inertial')
    if inertial is not None: link.remove(inertial)
    inertial = sub(link, 'inertial')
    sub(inertial, 'origin', xyz=' '.join(map(str, center)), rpy='0 0 0')
    sub(inertial, 'mass', value=mass)
    x, y, z = size
    sub(inertial, 'inertia', ixx=mass*(y*y+z*z)/12, iyy=mass*(x*x+z*z)/12,
        izz=mass*(x*x+y*y)/12, ixy=0, ixz=0, iyz=0)

def add_sensor_link(root, name, xyz, size):
    link = sub(root, 'link', name=name)
    set_box_inertia(link, 0.02, size)
    visual = sub(link, 'visual')
    sub(sub(visual, 'geometry'), 'box', size=' '.join(map(str, size)))
    material = sub(visual, 'material', name=name + '_material')
    sub(material, 'color', rgba='0.15 0.2 0.22 1')
    joint_name = name + '_joint'
    joint = sub(root, 'joint', name=joint_name, type='fixed')
    sub(joint, 'parent', link='base_link'); sub(joint, 'child', link=name)
    sub(joint, 'origin', xyz=xyz, rpy='0 0 0')
    sub(sub(root, 'gazebo', reference=joint_name), 'preserveFixedJoint', 'true')
    return sub(root, 'gazebo', reference=name)

def adapt_robot(root: ET.Element) -> ET.Element:
    """Mutate an expanded upstream URDF; no downloaded Python code is executed."""
    if root.tag != 'robot': raise ValueError('Expected an expanded URDF robot')
    root.set('name', ROBOT)
    # Replace all upstream simulation systems; no second odometry TF publisher.
    for g in list(root.findall('gazebo')):
        root.remove(g)
    # Upstream embeds some SDF fragments in <gz> inside URDF. They are not
    # portable URDF extensions. Recreate supported contact/friction tags below.
    for parent in root.iter():
        for child in list(parent):
            if child.tag == 'gz': parent.remove(child)
    for e in root.iter():
        if e.tag == 'link' and e.get('name') == 'base': e.set('name', 'base_link')
        if e.tag in ('parent', 'child') and e.get('link') == 'base': e.set('link', 'base_link')
    expected = {'base_footprint', 'base_link', 'left_wheel_link', 'right_wheel_link',
                'back_left_caster_link', 'back_right_caster_link', 'platform_link'}
    links = {l.get('name'): l for l in root.findall('link')}
    if not expected.issubset(links):
        raise ValueError('Upstream robot layout changed; missing links: ' + str(expected - links.keys()))
    base_joint = root.find("joint[@name='base_to_base_footprint']")
    if base_joint is None: raise ValueError('Missing upstream base joint')
    # Put the navigation frame at the drive axle, on the nominal ground plane.
    base_joint.find('origin').set('xyz', '-0.05 0 0.054')
    for name in ('back_left_caster_joint', 'back_right_caster_joint'):
        joint = root.find(f"joint[@name='{name}']")
        xyz = joint.find('origin').get('xyz').split()
        xyz[2] = '-0.029'
        joint.find('origin').set('xyz', ' '.join(xyz))
    set_box_inertia(links['base_link'], 1.5, (0.532, 0.532, 0.188), (0, 0, 0.07))
    set_box_inertia(links['platform_link'], 0.1, (0.55, 0.55, 0.04), (0, 0, 0.045))
    for name in ('left_wheel_link', 'right_wheel_link'):
        g = sub(root, 'gazebo', reference=name)
        sub(g, 'mu1', 1.0); sub(g, 'mu2', 1.0)
    for name in ('back_left_caster_link', 'back_right_caster_link'):
        g = sub(root, 'gazebo', reference=name)
        sub(g, 'mu1', 0.001); sub(g, 'mu2', 0.001)
    g = sub(root, 'gazebo', reference='platform_link')
    sub(g, 'mu1', 10.0); sub(g, 'mu2', 10.0)
    systems = sub(root, 'gazebo')
    plugin(systems, 'gz-sim-diff-drive-system', 'gz::sim::systems::DiffDrive', [
        ('left_joint', 'left_wheel_joint'), ('right_joint', 'right_wheel_joint'),
        ('wheel_separation', 0.576), ('wheel_radius', 0.1),
        ('topic', f'/model/{ROBOT}/cmd_vel'), ('odom_topic', f'/model/{ROBOT}/odometry'),
        ('tf_topic', f'/model/{ROBOT}/tf'), ('frame_id', 'odom'),
        ('child_frame_id', 'base_footprint'), ('odom_publish_frequency', 50),
        ('min_linear_velocity', -0.6), ('max_linear_velocity', 0.6),
        ('min_angular_velocity', -1.2), ('max_angular_velocity', 1.2),
        ('min_linear_acceleration', -1.5), ('max_linear_acceleration', 1.5),
        ('min_angular_acceleration', -3.0), ('max_angular_acceleration', 3.0)])
    plugin(systems, 'gz-sim-odometry-publisher-system', 'gz::sim::systems::OdometryPublisher', [
        ('odom_frame', 'world'), ('robot_base_frame', 'base_footprint'),
        ('odom_publish_frequency', 50), ('dimensions', 3),
        ('odom_topic', f'/model/{ROBOT}/ground_truth'),
        ('tf_topic', f'/model/{ROBOT}/ground_truth_tf_not_bridged')])
    # Position target in metres; actuate with bounded force, not ideal velocity.
    # gz-sim issue #2785 reports a similar lower-limit failure for a vertical
    # prismatic joint driven in velocity mode under DART. This is a targeted
    # workaround, not proof that the user's instance has that exact engine bug.
    # In force mode cmd_min/max are NEWTONS. Keeping the old +/-0.06 values
    # would not even support the 0.1 kg platform against gravity (~0.981 N).
    # Gains/feed-forward target the empty platform; payload transport still
    # requires separate testing and, potentially, retuning.
    plugin(systems, 'gz-sim-joint-position-controller-system', 'gz::sim::systems::JointPositionController', [
        ('joint_name', 'lift_joint'), ('topic', f'/model/{ROBOT}/lift/cmd_pos'),
        ('initial_position', 0.01), ('use_velocity_commands', 'false'),
        ('p_gain', 80.0), ('i_gain', 10.0), ('d_gain', 4.0),
        ('i_max', 5.0), ('i_min', -5.0), ('cmd_offset', 0.981),
        ('cmd_max', 20.0), ('cmd_min', -20.0)])
    plugin(systems, 'gz-sim-joint-state-publisher-system', 'gz::sim::systems::JointStatePublisher', [
        ('topic', f'/model/{ROBOT}/joint_state'),
        ('joint_name', 'left_wheel_joint'), ('joint_name', 'right_wheel_joint'),
        ('joint_name', 'lift_joint')])
    g = add_sensor_link(root, 'laser_frame', '0.30 0 0.25', (0.06, 0.06, 0.025))
    sensor = sub(g, 'sensor', name='laser', type='gpu_lidar')
    sub(sensor, 'always_on', 'true'); sub(sensor, 'update_rate', 10)
    sub(sensor, 'topic', f'/model/{ROBOT}/scan'); sub(sensor, 'gz_frame_id', 'laser_frame')
    sub(sensor, 'visualize', 'false')
    lidar = sub(sensor, 'lidar'); h = sub(sub(lidar, 'scan'), 'horizontal')
    sub(h, 'samples', 360); sub(h, 'resolution', 1)
    sub(h, 'min_angle', -3.141592653589793); sub(h, 'max_angle', 3.141592653589793)
    rng = sub(lidar, 'range'); sub(rng, 'min', 0.1); sub(rng, 'max', 20.0); sub(rng, 'resolution', 0.01)
    noise = sub(lidar, 'noise', type='gaussian'); sub(noise, 'mean', 0); sub(noise, 'stddev', 0.005)
    g = add_sensor_link(root, 'imu_link', '0 0 0.08', (0.015, 0.015, 0.015))
    sensor = sub(g, 'sensor', name='imu', type='imu')
    sub(sensor, 'always_on', 'true'); sub(sensor, 'update_rate', 50)
    sub(sensor, 'topic', f'/model/{ROBOT}/imu'); sub(sensor, 'gz_frame_id', 'imu_link')
    imu = sub(sensor, 'imu')
    for channel, sigma in [('angular_velocity', 0.0002), ('linear_acceleration', 0.01)]:
        channel_node = sub(imu, channel)
        for axis in ('x', 'y', 'z'):
            noise = sub(sub(channel_node, axis), 'noise', type='gaussian')
            sub(noise, 'mean', 0); sub(noise, 'stddev', sigma)
    return root

def exclude_robot_visuals_from_lidar(sdf_model: ET.Element) -> None:
    """Set rendering masks on the converted robot SDF, not on world objects.

    This is an idealised sensor model: rays see through the robot rather than
    returning NaN for an occluded direction. Collision geometry, joints and
    contacts are unchanged. Tables remain visible, including when carried.
    Apply after URDF conversion so these SDF-specific tags cannot be dropped.
    """
    if sdf_model.tag != 'model':
        raise ValueError('Expected the converted robot SDF model')
    visuals = sdf_model.findall('.//link/visual')
    lidars = []
    for sensor in sdf_model.findall('.//sensor'):
        if sensor.get('type') not in ('gpu_lidar', 'gpu_ray'):
            continue
        lidar = sensor.find('lidar')
        if lidar is None:
            lidar = sensor.find('ray')
        if lidar is None:
            raise RuntimeError('GPU lidar has no lidar/ray configuration')
        lidars.append(lidar)
    if not visuals or not lidars:
        raise RuntimeError('Converted robot is missing visuals or GPU lidar')
    for nodes, tag, value in ((visuals, 'visibility_flags', ROBOT_VISIBILITY_FLAGS),
                              (lidars, 'visibility_mask', LIDAR_VISIBILITY_MASK)):
        for parent in nodes:
            for old in list(parent.findall(tag)):
                parent.remove(old)
            sub(parent, tag, value)

def adapt_world(root: ET.Element, sdf_model: ET.Element, pose: str) -> ET.Element:
    world = root.find('world')
    if world is None: raise ValueError('No world in warehouse source')
    world.set('name', WORLD)
    physics = world.find('physics')
    if physics is not None:
        for tag, value in [('max_step_size', '0.002'), ('real_time_factor', '1.0')]:
            node = physics.find(tag)
            if node is None: node = sub(physics, tag)
            node.text = value
    # Supply explicit sensor systems: a bridge does not create sensors.
    names = {p.get('name') for p in world.findall('plugin')}
    if 'gz::sim::systems::Sensors' not in names:
        plugin(world, 'gz-sim-sensors-system', 'gz::sim::systems::Sensors', [('render_engine', 'ogre2')])
    if 'gz::sim::systems::Imu' not in names:
        plugin(world, 'gz-sim-imu-system', 'gz::sim::systems::Imu', [])
    for light in world.findall('light'):
        for node in list(light.findall('cast_shadows')): light.remove(node)
        sub(light, 'cast_shadows', 'false')
    for plane in world.findall("model[@name='ground_plane']/link/collision/geometry/plane"):
        if plane.find('size') is None: sub(plane, 'size', '100 100')
    sdf_model.set('name', ROBOT)
    p = sdf_model.find('pose')
    if p is None: p = sub(sdf_model, 'pose')
    p.text = pose
    world.append(sdf_model)
    return root

def rviz_compatible_urdf(robot: ET.Element, package: Path) -> str:
    """Return URDF with RViz-loadable mesh URIs, without changing Gazebo's SDF.

    The expanded upstream Xacro emits absolute filesystem paths. Gazebo accepts
    them, but RViz's resource_retriever expects a URI scheme. Prefer portable
    package:// URIs for assets in this package; handle external absolute assets
    with file:// URIs. Call AFTER gz sdf converts the original URDF.
    """
    package_path = Path(package).absolute()
    for mesh in robot.findall('.//mesh'):
        filename = mesh.get('filename', '')
        if not filename or '://' in filename:
            continue
        path = Path(filename)
        if not path.is_absolute():
            continue
        try:
            # Check the path as spelled first. This also handles assets folders
            # symlinked by colcon's --symlink-install.
            relative = path.relative_to(package_path)
        except ValueError:
            try:
                relative = path.resolve().relative_to(package_path.resolve())
            except ValueError:
                mesh.set('filename', path.as_uri())
                continue
        mesh.set('filename', 'package://trsa_warehouse/' + relative.as_posix())
    return ET.tostring(robot, encoding='unicode')


def prepare_files(package: Path, world_number='1', x=0.0, y=0.0, z=0.02, yaw=0.0,
                  output: Path | None = None, world_hook=None):
    """Expand Xacro, convert using installed sdformat, and embed robot in world.

    Embedding avoids a startup race between an asynchronous spawn process and
    Gazebo. robot_state_publisher receives the same robot kinematics with
    package:// mesh URIs suitable for RViz.
    """
    import xacro
    if str(world_number) not in ('1', '2'): raise ValueError('world must be 1 or 2')
    source = package / 'assets/models/logistic_holonomic_robot/logistic_holonomic_robot.urdf.xacro'
    doc = xacro.process_file(str(source), mappings={'namespace': ROBOT})
    robot = adapt_robot(ET.fromstring(doc.toxml()))
    urdf_text = ET.tostring(robot, encoding='unicode')
    if output is None:
        output = Path(tempfile.mkdtemp(prefix='trsa-warehouse-'))
        atexit.register(shutil.rmtree, output, ignore_errors=True)
    output.mkdir(parents=True, exist_ok=True)
    urdf_file = output / 'robot.urdf'; urdf_file.write_text(urdf_text)
    result = subprocess.run(['gz', 'sdf', '-p', str(urdf_file)], text=True,
                            capture_output=True, check=True, timeout=60,
                            env=resource_environment(package))
    try:
        sdf = ET.fromstring(result.stdout)
    except ET.ParseError as exc:
        raise RuntimeError('gz sdf did not produce usable XML:\n' + result.stderr) from exc
    model = sdf.find('model')
    if model is None: raise RuntimeError('URDF conversion produced no SDF model: ' + result.stderr)
    # The SDF has been generated from the original Gazebo-compatible URDF.
    # Publish and retain a portable URDF for RViz / robot_state_publisher.
    urdf_text = rviz_compatible_urdf(robot, package)
    urdf_file.write_text(urdf_text)
    exclude_robot_visuals_from_lidar(model)
    world = ET.parse(package / f'assets/worlds/warehouse{world_number}.world').getroot()
    adapt_world(world, model, f'{x} {y} {z} 0 0 {yaw}')
    if world_hook is not None:
        world_hook(world, package / 'assets')
    # Match the model SDF version to avoid embedding newer elements in an old world.
    world.set('version', sdf.get('version', '1.9'))
    world_file = output / 'warehouse.sdf'
    ET.indent(world)
    ET.ElementTree(world).write(world_file, encoding='utf-8', xml_declaration=True)
    return urdf_text, world_file
