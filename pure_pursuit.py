#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path

max_vel = 2.0
global steer 
kp = 1.2
k = 0.1
wheelbase = 1.983
d_lookahead = 0.8
print ("start")
def callback_feedback(data):
  global x
  global y
  global yaw
  global vel
  
  x = data.pose.pose.position.x + 2
  y = data.pose.pose.position.y + 2
  siny = +2.0 * (data.pose.pose.orientation.w * data.pose.pose.orientation.z + data.pose.pose.orientation.x * data.pose.pose.orientation.y)
  cosy = +1.0 - 2.0 * (data.pose.pose.orientation.y * data.pose.pose.orientation.y + data.pose.pose.orientation.z * data.pose.pose.orientation.z) 
  yaw = math.atan2(siny, cosy)  
  # vel = data.twist.twist.linear.x * math.cos(yaw) + data.twist.twist.linear.y * math.sin(yaw)
  print 'x of car:',x
  print 'y of car:',y
  print 'angle of car:',yaw
  # print 'vel of car:', vel
  print 'c'
  
  
  # cmd.linear.x = 0.1
  # # cmd.angular.z = 0
  # steer_path = x_p.poses[cp].pose.orientation.z
  # steer_err = (bot_theta - steer_path)
  # cmd.angular.z = convert_steering_angle_to_trans_rot_vel(cmd.linear.x, kp * steer_err)
  # pub1.publish(cmd)

def convert_steering_angle_to_trans_rot_vel(v, steer):
  # if steer == 0 or v == 0:
  #   return 0
  # if steer == 0:
  # 	return 0
  if steer > 0:
  	print "on the right"
  	if steer < math.pi/2:
  		radius = wheelbase/(math.tan(steer))
  		omega = (v / radius)
  		return omega
  	if steer > math.pi/2:
  		radius = wheelbase/(math.tan(math.pi - steer))
  		omega = (v / radius)
  		return omega
  if steer < 0:
  	print "one the left"
  	if steer > -math.pi/2:
  		radius = wheelbase/(math.tan(steer))
  		omega = (v / radius)
  		return omega
  	if steer < -math.pi/2:
  		radius = wheelbase/(math.tan(math.pi - steer))
  		omega = (v / radius)
  		return omega

  # print "steer:", steer
  # radius = wheelbase/(math.tan(steer))
  # omega = (v / radius) 
  # print "radius:", radius


  # return omega

def dist(a,x,y):
  #print a.pose.position.y
  return (((a.pose.position.x - x)**2) + ((a.pose.position.y - y)**2))**0.5

def callback_path(data):
  global ep
  global cp
   
  x_p = data
  
  distances = []
  for i in range(len(x_p.poses)):
    a = x_p.poses[i]
    distances += [dist(a,x,y)]  
  ep = min(distances)
  cp = distances.index(ep)
  print "min distance:", ep
  print 'old index:',cp

  cmd = Twist()
  L = 0
  Lf = k * max_vel + d_lookahead

  while Lf > L and (cp + 1) < len(x_p.poses):
  	dx = data.poses[cp + 1].pose.position.x - data.poses[cp].pose.position.x
  	dy = data.poses[cp + 1].pose.position.y - data.poses[cp].pose.position.y
  	L += math.sqrt(dx ** 2 + dy ** 2)
  	cp = cp + 1
  print len(x_p.poses)
  print 'new index is:',cp


  goal_point = [x_p.poses[cp].pose.position.x, x_p.poses[cp].pose.position.y]
  print 'current goal is:',goal_point
  error = [goal_point[0] - x, goal_point[1] - y]
  print error
  steer_angle = pure_pursuit(goal_point)
  print "steer_angle:", steer_angle
  cmd.linear.x = max_vel
  cmd.angular.z = convert_steering_angle_to_trans_rot_vel(max_vel,steer_angle)
  cmd.linear.y = math.sqrt(error[0]**2 + error[1]**2)
  print 'omega:',cmd.angular.z
  pub1.publish(cmd)



  print "cmd published"

  #print (ep)
  print x_p.poses[cp].pose.orientation
  
# def publish():
#   #global pub
#   global max_vel
#   cmd = Twist()
#   cmd.linear.x = max_vel*math.cos(steer)*math.cos(steer)
#   cmd.angular.z = convert_steering_angle_to_trans_rot_vel(steer)
#   pub1.publish(cmd)
def pure_pursuit(goal_point):
  tx = goal_point[0]
  ty = goal_point[1]
  print 'yaw:', yaw
  slope = math.atan2(ty - y, tx - x) 
  print "slope:", slope
  if x > tx:
  	if slope > yaw:
  		alpha = slope - yaw
  	if slope < yaw:
  		alpha = yaw - slope
  if tx > x:
  	if slope > yaw:
  		alpha = yaw - slope
  	if slope < yaw:
  		alpha = slope - yaw
  print 'alpha:', alpha 
  Lf = k * max_vel + d_lookahead

  Delta = math.atan2(2.0 * wheelbase * math.sin(alpha)/Lf,1)
  print 'Delta:', Delta
  return Delta

def start(): 
  global pub1
  #publish() 
  rospy.init_node('path_tracking',anonymous = True)
  pub1 = rospy.Publisher('cmd_vel', Twist, queue_size=10)
  rospy.Subscriber("odom",Odometry, callback_feedback)
  rospy.Subscriber("astroid_path",Path, callback_path)

  rospy.spin()

if __name__ == '__main__':
	start()

