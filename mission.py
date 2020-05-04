#!/usr/bin/python
# ROS specific imports
import roslib; roslib.load_manifest('floor_nav')
import rospy
from math import *
from task_manager_lib.TaskClient import *

rospy.init_node('task_client')
server_node = rospy.get_param("~server","/task_server")
default_period = rospy.get_param("~period",0.2)
tc = TaskClient(server_node,default_period)

tc.WaitForAuto()
try:

    tc.ActionMoveGoal(goal_x=0,goal_y=0,wait_completion=True,base_link="bubbleRob",resend_goal=True,reference_frame="Charger5",dist_threshold=0.3,angle_threshold=10,freq_resend=5.0)
    tc.Wait(duration=10.0)
    tc.ActionMoveGoal(goal_x=0,goal_y=0,wait_completion=True,base_link="bubbleRob",resend_goal=True,reference_frame="Waypoint1",dist_threshold=0.3,angle_threshold=10,freq_resend=5.0)
    tc.Wait(duration=5.0)
    tc.ActionMoveGoal(goal_x=0,goal_y=0,wait_completion=True,base_link="bubbleRob",resend_goal=True,reference_frame="Charger2",dist_threshold=0.3,angle_threshold=10,freq_resend=5.0)
    tc.Wait(duration=10.0)
    tc.ActionMoveGoal(goal_x=0,goal_y=0,wait_completion=True,base_link="bubbleRob",resend_goal=True,reference_frame="Waypoint2",dist_threshold=0.3,angle_threshold=10,freq_resend=5.0)
    tc.Wait(duration=5.0)
    tc.ActionMoveGoal(goal_x=0,goal_y=0,wait_completion=True,base_link="bubbleRob",resend_goal=True,reference_frame="Waypoint4",dist_threshold=0.3,angle_threshold=10,freq_resend=5.0)
    tc.Wait(duration=5.0)
    tc.ActionMoveGoal(goal_x=0,goal_y=0,wait_completion=True,base_link="bubbleRob",resend_goal=True,reference_frame="Charger1",dist_threshold=0.3,angle_threshold=10,freq_resend=5.0)
    tc.Wait(duration=10.0)
    tc.ActionMoveGoal(goal_x=0,goal_y=0,wait_completion=True,base_link="bubbleRob",resend_goal=True,reference_frame="Charger2",dist_threshold=0.3,angle_threshold=10,freq_resend=5.0)
    tc.Wait(duration=10.0)
    tc.ActionMoveGoal(goal_x=0,goal_y=0,wait_completion=True,base_link="bubbleRob",resend_goal=True,reference_frame="Waypoint3",dist_threshold=0.3,angle_threshold=10,freq_resend=5.0)
    tc.Wait(duration=5.0)
    tc.ActionMoveGoal(goal_x=0,goal_y=0,wait_completion=True,base_link="bubbleRob",resend_goal=True,reference_frame="Waypoint5",dist_threshold=0.3,angle_threshold=10,freq_resend=5.0)
    tc.Wait(duration=5.0)
    tc.ActionMoveGoal(goal_x=0,goal_y=0,wait_completion=True,base_link="bubbleRob",resend_goal=True,reference_frame="Charger3",dist_threshold=0.3,angle_threshold=10,freq_resend=5.0)
    tc.Wait(duration=10.0)
    tc.ActionMoveGoal(goal_x=0,goal_y=0,wait_completion=True,base_link="bubbleRob",resend_goal=True,reference_frame="Waypoint6",dist_threshold=0.3,angle_threshold=10,freq_resend=5.0)
    tc.Wait(duration=5.0)
    tc.ActionMoveGoal(goal_x=0,goal_y=0,wait_completion=True,base_link="bubbleRob",resend_goal=True,reference_frame="Charger4",dist_threshold=0.3,angle_threshold=10,freq_resend=5.0)
    tc.Wait(duration=10.0)
    tc.ActionMoveGoal(goal_x=0,goal_y=0,wait_completion=True,base_link="bubbleRob",resend_goal=True,reference_frame="Waypoint7",dist_threshold=0.3,angle_threshold=10,freq_resend=5.0)
    tc.Wait(duration=5.0)

except TaskException, e:
    rospy.logerr("Exception caught: " + str(e))

if not rospy.core.is_shutdown():
    tc.SetManual()


rospy.loginfo("Mission completed")
