import re

#Example of output
#Robot1(41)::<41: [MoveToC5()]>U<AllenInterval 41<56:{0,480} 57:{5,485}> [[0, 480], [5, 485]]>/JUSTIFIED --> 0

Action="Robot1"
taskMoveCharger="MoveToC"
taskMoveWaypoint="MoveTo"
taskWait="Wait"
taskSpin="Spin"

def TaskMoveCharger(i):
    return "    tc.ActionMoveGoal(goal_x=0,goal_y=0,wait_completion=True,base_link=\"bubbleRob\",resend_goal=True,reference_frame=\"Charger"+i+"\",dist_threshold=0.3,angle_threshold=10,freq_resend=5.0)\n"

def TaskMoveWaypoint(i):
    return "    tc.ActionMoveGoal(goal_x=0,goal_y=0,wait_completion=True,base_link=\"bubbleRob\",resend_goal=True,reference_frame=\"Waypoint"+i+"\",dist_threshold=0.3,angle_threshold=10,freq_resend=5.0)\n"

def TaskWait():
    return "    tc.Wait(duration=10.0)\n"

def TaskSpin():
    return "    tc.Wait(duration=5.0)\n"


fi=open("./test_planner/output/OutputTurtle.txt","r")
fo=open("mission.py","w")

header = '''#!/usr/bin/python
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

'''

ender='''
except TaskException, e:
    rospy.logerr("Exception caught: " + str(e))

if not rospy.core.is_shutdown():
    tc.SetManual()


rospy.loginfo("Mission completed")
'''

fo.write(header)

lines = fi.readlines()
for str in lines:
    if(str.split('(')[0]==Action):
        task=re.split('[\(\)\[\]]',str)[3]
        num=re.split('(\d)',task)[1]
        task=re.split('(\d)',task)[0]
        #print(task, num)
        if(task==taskMoveCharger):
             fo.write(TaskMoveCharger(num))
        elif(task==taskMoveWaypoint):
             fo.write(TaskMoveWaypoint(num))
        elif(task==taskWait):
             fo.write(TaskWait())
        elif(task==taskSpin):
             fo.write(TaskSpin())

        else:
             print("error")

fo.write(ender)
