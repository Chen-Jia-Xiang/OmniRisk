#!/usr/bin/env python3
"""Validate an isolated simulation without retaining process data."""
import argparse
from collections import deque
import json
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import tempfile
import threading
import time
import xmlrpc.client

import numpy as np
import rospy
from ruamel.yaml import YAML
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from m_detector.msg import TrackedObject, TrackedObjectArray
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[2]
yaml = YAML(typ='safe')
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--perception', choices=['detector', 'truth'], default='detector')
parser.add_argument('--scenario', choices=['none', 'head_on', 'side', 'multi'], default='head_on')
parser.add_argument('--speed', type=float, default=2)
parser.add_argument('--lead-time', type=float, default=2.5)
parser.add_argument('--duration', type=float, default=30)
parser.add_argument('--lidar-fps', type=float, default=20)
parser.add_argument('--port', type=int, default=11321)
parser.add_argument('--rviz', action='store_true',
                    help='open RViz with drone, dynamic balls, point cloud and selected trajectory')
args = parser.parse_args()
temporary = tempfile.TemporaryDirectory(prefix='omnirisk-simulation-')
out = Path(temporary.name)
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:%d' % args.port,
                  ROS_HOSTNAME='127.0.0.1', ROS_LOG_DIR=str(out / 'roslog'),
                  ROS_HOME=str(out / 'ros-home'), PYTHONUNBUFFERED='1',
                  OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1', MKL_NUM_THREADS='1')
os.environ.pop('ROS_IP', None)
state = dict(position=None, min_clearance=None, max_tracks=0, collision=None, bridge_errors=0)
children, logs = [], []
odom_buffer = deque(maxlen=100)
balls = {}
lock = threading.Lock()


def event(name, **values):
    entry = dict(event=name, time=time.time(), **values)
    print(json.dumps(entry), flush=True)


def start(name, command, cwd=ROOT):
    log = (out / (name + '.log')).open('w')
    logs.append(log)
    proc = subprocess.Popen(['bash', '-c', command], cwd=str(cwd), env=os.environ.copy(),
                            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    children.append((name, proc))


def healthy():
    for name, proc in children:
        if proc.poll() is not None:
            raise RuntimeError('%s exited with %s; see its log' % (name, proc.returncode))


def wait_for(predicate, timeout, label):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        healthy()
        if predicate():
            return
        time.sleep(.1)
    raise RuntimeError('Timeout waiting for ' + label)


def odom(msg):
    p, v, q = msg.pose.pose.position, msg.twist.twist.linear, msg.pose.pose.orientation
    row = [time.time(), msg.header.stamp.to_sec(), p.x, p.y, p.z, v.x, v.y, v.z]
    state['position'] = row[2:5]
    with lock:
        odom_buffer.append((msg.header.stamp.to_sec(), np.array(row[2:5]),
                            [q.x, q.y, q.z, q.w]))


def obstacles(msg):
    with lock:
        position = odom_buffer[-1][1] if odom_buffer else None
        for m in msg.markers:
            if m.type != Marker.SPHERE or m.action != Marker.ADD:
                continue
            t = m.header.stamp.to_sec()
            center = np.array([m.pose.position.x, m.pose.position.y, m.pose.position.z])
            old = balls.get(m.id)
            velocity = (center-old[1])/(t-old[0]) if old and t > old[0] else np.zeros(3)
            balls[m.id] = (t, center, velocity, m.scale.x/2)
            if position is not None:
                clearance = np.linalg.norm(center-position) - m.scale.x/2 - .2
                state['min_clearance'] = float(clearance) if state['min_clearance'] is None else min(state['min_clearance'], float(clearance))


def tracks(msg):
    state['max_tracks'] = max(state['max_tracks'], len(msg.tracks))


def collision(msg):
    state['collision'] = msg.text


def truth_bridge(msg):
    """Use simulator states to isolate planning/control from detection failures."""
    try:
        stamp = msg.header.stamp.to_sec()
        with lock:
            if not odom_buffer:
                return
            t, p, q = min(odom_buffer, key=lambda item: abs(item[0]-stamp))
            snapshot = list(balls.items())
        if abs(t-stamp) > .03:
            return
        offsets = {f.name: f.offset for f in msg.fields}
        dtype = np.dtype(dict(names=['x', 'y', 'z'], formats=['<f4']*3,
                              offsets=[offsets[k] for k in ['x', 'y', 'z']], itemsize=msg.point_step))
        raw = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                         strides=(msg.row_step, msg.point_step))
        xyz = np.column_stack([raw[k].reshape(-1) for k in ['x', 'y', 'z']])
        world = np.ascontiguousarray(xyz @ Rotation.from_quat(q).as_matrix().T + p, dtype=np.float32)
        cloud = PointCloud2()
        cloud.header.stamp, cloud.header.frame_id = msg.header.stamp, 'world'
        cloud.height, cloud.width = 1, len(world)
        cloud.fields = [PointField(name=k, offset=i*4, datatype=PointField.FLOAT32, count=1)
                        for i, k in enumerate(['x', 'y', 'z'])]
        cloud.point_step, cloud.row_step, cloud.is_dense = 12, len(world)*12, False
        cloud.data = world.tobytes()
        truth = TrackedObjectArray()
        truth.header = cloud.header
        for object_id, (measurement, center, velocity, radius) in snapshot:
            track = TrackedObject()
            track.id = track.track_uid = object_id
            track.confirmed = track.matched = True
            track.point_count = 10
            predicted = center + velocity*(stamp-measurement)
            track.position.x, track.position.y, track.position.z = predicted.tolist()
            track.velocity.x, track.velocity.y, track.velocity.z = velocity.tolist()
            track.dimensions.x = track.dimensions.y = track.dimensions.z = 2*radius
            truth.tracks.append(track)
        static_pub.publish(cloud)
        tracks_pub.publish(truth)
    except Exception as error:
        state['bridge_errors'] += 1
        rospy.logerr_throttle(2, 'Truth diagnostic input failed: %s', error)


try:
    master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
    try:
        master.getPid('/simulation_probe')
    except OSError:
        pass
    else:
        raise RuntimeError('ROS port already in use; choose another --port')
    start('master', 'exec roscore -p %d' % args.port)
    def master_ready():
        try:
            return master.getPid('/simulation_probe')[0] == 1
        except OSError:
            return False
    wait_for(master_ready, 15, 'ROS master')
    rospy.init_node('simulation_validation', anonymous=True, disable_signals=True)
    rospy.Subscriber('/sim/odom', Odometry, odom, queue_size=100)
    rospy.Subscriber('/sim/dynamic_obstacle_markers', MarkerArray, obstacles, queue_size=10)
    rospy.Subscriber('/sim/collision_counter', Marker, collision, queue_size=10)
    rospy.Subscriber('/m_detector/confirmed_tracks', TrackedObjectArray, tracks, queue_size=10)
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
    trigger_pub = rospy.Publisher('/clicked_point', PointStamped, queue_size=1)
    params = yaml.load((ROOT/'ros_ws/src/M-detector/config/mid360/mid360.yaml').read_text())['dyn_obj']
    overlay = yaml.load((ROOT/'ros_ws/src/M-detector/config/simulation.yaml').read_text())['dyn_obj']
    params['tracker'].update(overlay.pop('tracker'))
    params.update(overlay, timing_enabled=False)
    rospy.set_param('/dyn_obj', params)
    if args.perception == 'truth':
        static_pub = rospy.Publisher('/m_detector/std_points', PointCloud2, queue_size=1)
        tracks_pub = rospy.Publisher('/m_detector/confirmed_tracks', TrackedObjectArray, queue_size=1)
        rospy.Subscriber('/lidar_points', PointCloud2, truth_bridge, queue_size=1, buff_size=8*1024*1024)
    config = yaml.load((ROOT/'Simulator/src/config/config.yaml').read_text())
    config.update(maze_type=8, lidar_fps=args.lidar_fps)
    config['sim_scenario'].update(enabled=True, scenario='head_on' if args.scenario == 'none' else args.scenario)
    if args.scenario in ('head_on', 'side', 'none'):
        config['sim_scenario']['scenarios'][config['sim_scenario']['scenario']] = [dict(
            bearing_deg=90 if args.scenario == 'side' else 0, elev_deg=0,
            speed=args.speed, lead_time=args.lead_time, aim_up=0)]
    with (out/'scenario.yaml').open('w') as f:
        yaml.dump(config, f)
    start('dynamics', 'source Controller/devel/setup.bash\nexec roslaunch so3_quadrotor_simulator simulator_attitude_control.launch')
    wait_for(lambda: len(odom_buffer)>0, 20, 'odometry')
    start('sensor', 'source Simulator/devel/setup.bash\nexec rosrun sensor_simulator sensor_simulator_cuda ' +
          shlex.quote('_config_file:='+str(out/'scenario.yaml')))
    if args.perception == 'detector':
        start('detector', 'exec '+shlex.quote(str(ROOT/'ros_ws/devel/lib/m_detector/dynfilter')))
    start('policy', 'source ../ros_ws/devel/setup.bash\nexec ../omnirisk_env/bin/python -u omnirisk_node.py --trial 1 --epoch 60 --simulation --flight_mode nav', ROOT/'OmniRisk')
    wait_for(lambda: 'OmniRisk Node Ready!' in (out/'policy.log').read_text(), 50, 'policy')
    if args.rviz:
        start('rviz', 'source Controller/devel/setup.bash\nexec rviz -d ' +
              shlex.quote(str(ROOT/'experiments/simulation/rviz.yaml')) +
              ' __name:=simulation_rviz')
        def rviz_ready():
            subscribers = dict(master.getSystemState('/simulation_validation')[2][1])
            return '/simulation_rviz' in subscribers.get('/sim/dynamic_obstacle_markers', [])
        wait_for(rviz_ready, 30, 'RViz displays')
    wait_for(lambda: goal_pub.get_num_connections()>0 and trigger_pub.get_num_connections()>0, 10, 'subscribers')
    time.sleep(3)
    goal = PoseStamped()
    goal.header.stamp, goal.header.frame_id = rospy.Time.now(), 'world'
    goal.pose.position.x, goal.pose.position.z, goal.pose.orientation.w = 12, 2, 1
    event('goal', goal=[12, 0, 2])
    goal_pub.publish(goal)
    time.sleep(3)
    event('trigger', scenario=args.scenario, position=state['position'])
    if args.scenario != 'none':
        trigger = PointStamped()
        trigger.header.stamp, trigger.header.frame_id = rospy.Time.now(), 'world'
        trigger_pub.publish(trigger)
    deadline = time.monotonic()+args.duration
    while time.monotonic()<deadline:
        time.sleep(2)
        healthy()
    policy = (out/'policy.log').read_text()
    count = re.search(r'Collisions: (\d+)', state['collision'] or '')
    summary = dict(perception=args.perception, collision_count=int(count[1]) if count else None,
                   min_sampled_surface_clearance_m=state['min_clearance'],
                   final_position=state['position'],
                   final_goal_error_m=float(np.linalg.norm(np.array(state['position'])-[12,0,2])),
                   arrived='Arrive!' in policy.split('New Goal:')[-1],
                   dodge_activated='runtime mode=DODGE' in policy.split('New Goal:')[-1],
                   max_tracks=state['max_tracks'])
    dynamic_verified = args.scenario == 'none' or (summary['max_tracks'] > 0 and summary['dodge_activated'])
    summary['passed'] = summary['collision_count']==0 and summary['arrived'] and summary['final_goal_error_m']<2 and dynamic_verified and not state['bridge_errors']
    event('complete', **summary)
except BaseException as error:
    for name in ('policy', 'detector', 'sensor', 'rviz'):
        path = out/(name+'.log')
        if path.exists():
            print(name + ': ' + '\n'.join(path.read_text(errors='replace').splitlines()[-6:]), flush=True)
    raise
finally:
    for name, proc in reversed(children):
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
    for name, proc in reversed(children):
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=5)
    for log in logs:
        log.close()
    temporary.cleanup()

if not summary['passed']:
    raise SystemExit(1)
