#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import math
from std_msgs.msg import String
import random
from ar_week10_test.msg import square_size_param
from moveit_commander.conversions import pose_to_list


class MovePanda(): #create MovePanda object
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander() #create robot
        scene = moveit_commander.PlanningSceneInterface() #create scene
        group_name = "panda_arm" #create joint group name
        move_group = moveit_commander.MoveGroupCommander(group_name) #assign group move
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size=20) #create display trajectory publisher

	#create planning frame, eef_link, and group_names
        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()

	#create class variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names


    def go_to_home(self):
        move_group = self.move_group

        start_conf = [0, -math.pi/4, 0, -math.pi/2, 0, math.pi/3, 0] 
        joint_goal = move_group.get_current_joint_values() 
        joint_goal = start_conf #set joint goal to start_conf points
        move_group.go(joint_goal, wait=True) #move
        move_group.stop() #stop so there is no residual movement
        return

    def plan_cartesian_path(self, data):
        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose #get current robot pose
        wpose.position.x += data #forward in x
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y += data #forward in y
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x += -data #backwards in x
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y += -data #backwards in y
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)

        return plan, fraction

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()  # obtain display trajectory message
        display_trajectory.trajectory_start = robot.get_current_state() 
        display_trajectory.trajectory.append(plan) #assing cartesian plan to display
        # Publish
        display_trajectory_publisher.publish(display_trajectory);
    
    def execute_plan(self, plan):
        move_group = self.move_group

        move_group.execute(plan, wait=True) #move robot         

print("-------------------------------------------")
print("Move Panda - Initalizing") 
move = MovePanda()
print("-------------------------------------------")
print("-------------------------------------------")
print("Move Panda - Waiting for desired size of square trajectory")
print("-------------------------------------------")


def move_panda(data):
    print("-------------------------------------------")
    print"Recieved square size s =", data.l
    print("-------------------------------------------")
   
    print("-------------------------------------------")
    print("Move Panda - Going to start configuration")
    print("-------------------------------------------")
    move.go_to_home() #go to home configuration
    rospy.sleep(3.)

    print("-------------------------------------------")
    print("Move Panda - Planning motion trajectory")
    print("-------------------------------------------")
    cartesian_plan, fraction = move.plan_cartesian_path(data.l) #create cartesian plan
    rospy.sleep(5.)

    print("-------------------------------------------")
    print("Move Panda - Showing planned trajectory")
    print("-------------------------------------------")
    move.display_trajectory(cartesian_plan) #show cartesian plan
    rospy.sleep(5.)
    
    print("-------------------------------------------")
    print("Move Panda - Executing planned trajectory")
    print("-------------------------------------------")
    move.execute_plan(cartesian_plan) #move robot according to cartesian plan

    print("-------------------------------------------")
    print("Move Panda - Waiting for desired size of square trajectory")
    print("-------------------------------------------")


def listener():
    rospy.init_node('move_panda') #create move_panda node
    rospy.Subscriber('params', square_size_param, move_panda) #create subscrive
    rospy.spin()


if __name__ == '__main__':
    listener()
