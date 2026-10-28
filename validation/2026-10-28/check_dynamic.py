import rospy,time,json,math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped,PointStamped
from visualization_msgs.msg import Marker,MarkerArray
from m_detector.msg import TrackedObjectArray
rospy.init_node('omnirisk_dynamic_validation',anonymous=True)
s={'positions':[],'collisions':[],'tracks_max':0,'min_surface_clearance_m':None,'markers':0}
latest=None
def odom(m):
 global latest
 p=m.pose.pose.position;latest=[p.x,p.y,p.z];s['positions'].append([time.time()]+latest)
def collision(m):
 if not s['collisions'] or s['collisions'][-1]!=m.text:s['collisions'].append(m.text)
def markers(m):
 if latest is None:return
 for a in m.markers:
  if a.action!=Marker.ADD or a.type!=Marker.SPHERE:continue
  s['markers']+=1;p=a.pose.position
  d=math.sqrt(sum((x-y)**2 for x,y in zip(latest,[p.x,p.y,p.z])))-a.scale.x/2-.2
  if s['min_surface_clearance_m'] is None or d<s['min_surface_clearance_m']:s['min_surface_clearance_m']=d

def tracks(m):s['tracks_max']=max(s['tracks_max'],len(m.tracks))
rospy.Subscriber('/sim/odom',Odometry,odom)
rospy.Subscriber('/sim/collision_counter',Marker,collision)
rospy.Subscriber('/sim/dynamic_obstacle_markers',MarkerArray,markers)
rospy.Subscriber('/m_detector/confirmed_tracks',TrackedObjectArray,tracks)
goalpub=rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1,latch=True)
trigger=rospy.Publisher('/clicked_point',PointStamped,queue_size=1)
time.sleep(2)
g=PoseStamped();g.header.frame_id='world';g.pose.position.x=20;g.pose.position.z=.6;g.pose.orientation.w=1;goalpub.publish(g)
time.sleep(3)
p=PointStamped();p.header.frame_id='world';p.header.stamp=rospy.Time.now();trigger.publish(p)
s['trigger_position']=latest[:];print('TRIGGER',latest,flush=True)
for i in range(5):
 time.sleep(3);print('POSITION',latest,'TRACKS_MAX',s['tracks_max'],'MIN_CLEARANCE',s['min_surface_clearance_m'],'COLLISIONS',s['collisions'],flush=True)
json.dump(s,open('/tmp/omnirisk-dynamic-validation.json','w'),indent=2)
