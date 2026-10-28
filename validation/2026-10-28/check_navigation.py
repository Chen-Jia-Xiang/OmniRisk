import rospy,time,json,math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker,MarkerArray
from sensor_msgs.msg import PointCloud2
from roslib.message import get_message_class
rospy.init_node('omnirisk_validation',anonymous=True)
state={'counts':{},'collisions':[],'positions':[],'dynamic_markers':0,'tracks_max':0}
def count(k): state['counts'][k]=state['counts'].get(k,0)+1
def odom(m):
 count('odom');p=m.pose.pose.position
 state['positions'].append([time.time(),p.x,p.y,p.z])
def collision(m):
 if not state['collisions'] or state['collisions'][-1]!=m.text: state['collisions'].append(m.text)
def dyn(m): state['dynamic_markers']=max(state['dynamic_markers'],len(m.markers));count('dynamic')
def tracks(m): state['tracks_max']=max(state['tracks_max'],len(m.tracks));count('tracks')
rospy.Subscriber('/sim/odom',Odometry,odom,queue_size=100)
rospy.Subscriber('/sim/collision_counter',Marker,collision)
rospy.Subscriber('/sim/dynamic_obstacle_markers',MarkerArray,dyn)
for topic in ['/lidar_points','/m_detector/std_points']:
 rospy.Subscriber(topic,PointCloud2,lambda m,t=topic:count(t),queue_size=1)
from rostopic import get_topic_class
for topic in ['/so3_control/pos_cmd','/m_detector/confirmed_tracks']:
 cls,_,_=get_topic_class(topic,blocking=True)
 rospy.Subscriber(topic,cls,tracks if topic.endswith('tracks') else lambda m:count('control'),queue_size=10)
pub=rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1,latch=True)
time.sleep(2)
start=state['positions'][-1]
goal=PoseStamped();goal.header.frame_id='world';goal.header.stamp=rospy.Time.now();goal.pose.position.x=8;goal.pose.position.y=0;goal.pose.position.z=.6;goal.pose.orientation.w=1
pub.publish(goal)
print('START',start,'GOAL',[8,0,.6],flush=True)
begin=time.time()
while time.time()-begin<35:
 time.sleep(5)
 p=state['positions'][-1];print('POSITION',p[1:],'DISTANCE',math.hypot(p[1]-8,p[2]),'COLLISIONS',state['collisions'],flush=True)
state['goal']=[8,0,.6];state['elapsed']=time.time()-begin
with open('/tmp/omnirisk-nav-validation.json','w') as f:json.dump(state,f,indent=2)
print('COUNTS',state['counts'],'TRACKS_MAX',state['tracks_max'],flush=True)
