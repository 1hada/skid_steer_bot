#!/usr/bin/env python

# https://github.com/jsk-ros-pkg/jsk_visualization/blob/master/jsk_rviz_plugins/samples/polygon_array_sample.py

import rospy
#from jsk_recognition_msgs.msg import PolygonArray
from geometry_msgs.msg import Polygon, PolygonStamped, Point32, PoseStamped, PolygonStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header
from math import sin, cos, pi
import numpy as np

import actionlib
from boustrophedon_msgs.msg import PlanMowingPathGoal, PlanMowingPathAction, StripingPlan, StripingPoint,PlanMowingPathResult
from boustrophedon_msgs.srv import ConvertPlanToPath, ConvertPlanToPathRequest
m_pose_seen = False
m_pose_stamped = PoseStamped() 

def SquarePolygon(header):
    p = PolygonStamped()
    p.header = header
    p.polygon.points = [Point32(x=1.0, y=1.0),
                        Point32(x=-1.0, y=1.0),
                        Point32(x=-1.0, y=-1.0),
                        Point32(x=1.0, y=-1.0)]
    return p

def LargeSquarePolygon(header):
    p = PolygonStamped()
    p.header = header
    p.polygon.points = [Point32(x=-10.0, y=10.0),Point32(x=-6.0, y=10.0),Point32(x=-2.0, y=10.0),Point32(x=2.0, y=10.0),Point32(x=6.0, y=10.0),
                        Point32(x=10.0, y=10.0),Point32(x=10.0, y=6.0),Point32(x=10.0, y=2.0),Point32(x=10.0, y=-2.0),Point32(x=10.0, y=-6.0),
                        Point32(x=10.0, y=-10.0),Point32(x=6.0, y=-10.0),Point32(x=2.0, y=-10.0),Point32(x=-2.0, y=-10.0),Point32(x=-6.0, y=-10.0),
                        Point32(x=-10.0, y=-10.0),Point32(x=-10.0, y=-6.0),Point32(x=-10.0, y=-2.0),Point32(x=-10.0, y=2.0),Point32(x=-10.0, y=6.0)]
    return p
    
def RectanglePolygon(header):
    p = PolygonStamped()
    p.header = header
    p.polygon.points = [Point32(x=-1.0, y=1.0, z=1.0),
                        Point32(x=-2.0, y=1.0, z=1.0),
                        Point32(x=-2.0, y=-1.0, z=1.0),
                        Point32(x=-1.0, y=-1.0, z=1.0)]
    return p
def CirclePolygon(header):
    p = PolygonStamped()
    p.header = header
    for i in range(100):
        theta = i / 100.0 * 2.0 * pi
        x = 1.0 * cos(theta) + 3.0
        y = 1.0 * sin(theta)
        p.polygon.points.append(Point32(x=x, y=y))
    return p
    
    # star
def StarPolygon(header):
    p = PolygonStamped()
    p.header = header
    p.polygon.points = [Point32(x= .0000, y= 1.0000 + 3.0),
                        Point32(x= .2245, y= .3090 + 3.0),
                        Point32(x= .9511, y= .3090 + 3.0),
                        Point32(x= .3633, y= -.1180 + 3.0),
                        Point32(x= .5878, y= -.8090 + 3.0),
                        Point32(x= .0000, y= -.3820 + 3.0),
                        Point32(x= -.5878, y= -.8090 + 3.0),
                        Point32(x= -.3633, y= -.1180 + 3.0),
                        Point32(x= -.9511, y= .3090 + 3.0),
                        Point32(x= -.2245, y= .3090 + 3.0)]
    return p

 
def odom_cb(msg):
    global m_pose_seen, m_pose_stamped
    m_pose_stamped.header = msg.header
    m_pose_stamped.pose   = msg.pose.pose
    m_pose_seen = True
   
def do_action(polygon_stamped, pose_stamped):
    client = actionlib.SimpleActionClient('plan_path', PlanMowingPathAction)
    client.wait_for_server()
    goal = PlanMowingPathGoal()
    goal.property = polygon_stamped
    goal.robot_position = pose_stamped
    # Fill in the goal here
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    return client.get_result()

def convert_striping_plan_to_path_client(plan):
    # plan istype boustrophedon_msgs/StripingPlan
    rospy.wait_for_service('convert_striping_plan_to_path')
    try:
        svc_convert_striping_plan_to_path = rospy.ServiceProxy('convert_striping_plan_to_path', ConvertPlanToPath)
        resp1 = svc_convert_striping_plan_to_path(plan.plan)
        return resp1 # istype nav_msgs/Path
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
 
def main(pub_path):
    global m_pose_seen, m_pose_stamped
    r = rospy.Rate(2)

    m_polygon = PolygonStamped()
    header = Header()
    header.frame_id = "odom"
    header.stamp = rospy.Time.now()
    m_polygon.header = header
    m_polygon = LargeSquarePolygon(header)
    """
    m_polygon = [SquarePolygon(header),
	            RectanglePolygon(header),
	            CirclePolygon(header),
	            StarPolygon(header)][3]
    """
    while not m_pose_seen :
        rospy.loginfo("NOT pose seen.")
        r.sleep()

    rospy.loginfo("pose seen.")
    res = do_action(m_polygon, m_pose_stamped)
    rospy.loginfo("len(res.plan.points) == "+str(len(res.plan.points)))
    rospy.loginfo("do_action(..) seen.")

    mpath = convert_striping_plan_to_path_client(res)
    rospy.loginfo("convert_striping_plan_to_path_client(..) seen.")
    rospy.loginfo(str(mpath))
    while not rospy.is_shutdown():
        pub_path.publish(mpath.path)
        r.sleep()
"""
THROWS : AFTER     mpath = convert_striping_plan_to_path_client(res)

[ INFO] [1619578598.014387542]: Decomposing boundary polygon into sub-polygons...
[ INFO] [1619578598.014692191]: Broke the boundary up into 1 sub-polygons
inserted an extra boundary following path of size: 1

BECAUSE : the path is empty
  text: "Boustrophedon planner failed with a tf exception: \"map\" passed to lookupTransform\
  \ argument target_frame does not exist. "



"""

if __name__ == "__main__":
    rospy.init_node("polygon_array_sample")
    sub_odom    = rospy.Subscriber("/odom",Odometry,odom_cb)
    pub_polygon = rospy.Publisher("polygon_output", PolygonStamped,queue_size=1)
    pub_path = rospy.Publisher("path_planned_output", Path,queue_size=1)
    main(pub_path)


