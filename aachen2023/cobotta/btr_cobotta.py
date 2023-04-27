# Copyright (C) 2019  DENSO WAVE INCORPORATED
#
# -*- coding: utf-8 -*-
#
# usage: python ./packing_pose.py
#
#!/usr/bin/env python
import os
import sys
import rospy
import actionlib
import math
import moveit_commander
import rosservice
import geometry_msgs.msg
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion
from denso_cobotta_gripper.msg import GripperMoveAction, GripperMoveGoal
from denso_cobotta_driver.srv import GetMotorState

from std_srvs.srv import Empty, EmptyResponse

# NOTE: Before start this program, please launch denso_cobotta_bring.launch

joints_name = ["joint_1", "joint_2",
               "joint_3", "joint_4", "joint_5", "joint_6"]

#
# Poses
#
joints_grab_01 = [0.06125191496475896, -0.32464939841015966, 2.2169738618717036, 0.10941719117321055, 1.2436783485121323, 1.5506928224563084]
joints_grab_02 = [0.023708511487324654, 0.636281693027016, 1.072247128704676, 0.07258491467581482, 1.5366322484961243, 1.6949599441833756]
joints_grab_03 = [0.1058310113773345, 0.734590893346414, 1.4702046998243976, -0.00776509098648591, 1.035586186898004, 1.6193924835160065]
joints_grab_04 = [0.1058310113773345, 0.7346527953345616, 1.4702606110646617, -0.007287558653092391, 1.0356511818469727, 1.6192191636520905]
# gripper close
joints_grab_05 = [0.10589013484737769, 0.7345820502052501, 1.4701674256642216, -0.007806615537215781, 1.0355645219150147, 1.6193274885670381]
joints_grab_06 = [0.10589013484737769, 0.7345820502052501, 1.4702046998243976, -0.007827377812580716, 1.0356078518809937, 1.6192841586010591]
joints_grab_07 = [0.1058310113773345, 0.7345820502052501, 1.4702046998243976, -0.007848140087945652, 1.0356078518809937, 1.6192841586010591]
joints_grab_08 = [0.09965260875781971, 0.7295679891653024, 1.5415101682412036, 0.025724459177155195, 1.0025687528220093, 1.6400608772879877]
joints_grab_09 = [0.061311038434802165, 0.5398295523520429, 1.472236141553993, 0.03429927890287359, 1.1927223085208332, 1.7010911343694035]
joints_grab_10 = [0.060956297614542944, 0.5399533563283379, 1.4725716089955776, 0.03492214716382166, 1.1926356485888752, 1.7068323548616204]
joints_grab_11 = joints_grab_01

one_radian = math.pi / 180
joints_release_01 = joints_grab_01
joints_release_02 = [0.11449259973866383, 0.6079924844436072, 1.382125859328364, -0.05950468119590539, 1.1804599281487773, 1.6736416009217097]
joints_release_03 = [0.10855069099932187, 0.7401001702915418, 1.4309922833191808, -0.06041822131196256, 0.9812504095603432, 1.6966498128565564]
joints_release_04 = [0.10855069099932187 + 2*one_radian, 0.7401001702915418, 1.4309922833191808, -0.06041822131196256, 0.9812504095603432, 1.6966498128565564]
joints_release_05 = [0.10855069099932187 + 0.8*one_radian, 0.7401001702915418, 1.4309922833191808, -0.06041822131196256, 0.9812504095603432, 1.6966498128565564]


#
#
#PLAY EREA
#
#

def test_move(data):
    
    print "cobotta now moving..."
    
    move_group.go(joint_1,wait = True)


def grab_Arm(data):
    global gripper_client, gripper_parallel_open, gripper_parallel_close, gripper_parallel_speed, gripper_parallel_effort
    print "cobotta grabbing objects..."

    gripper_move(gripper_client,gripper_parallel_open,gripper_parallel_speed,gripper_parallel_effort)
    move_group.go(joints_grab_01, wait = True)
    move_group.go(joints_grab_02, wait = True)
    move_group.go(joints_grab_03, wait = True)
    move_group.go(joints_grab_04, wait = True)
    gripper_move(gripper_client,gripper_parallel_close,gripper_parallel_speed,gripper_parallel_effort)
    move_group.go(joints_grab_05, wait = True)
    move_group.go(joints_grab_06, wait = True)
    rospy.sleep(5)
    move_group.go(joints_grab_07, wait = True)
    move_group.go(joints_grab_08, wait = True)
    move_group.go(joints_grab_09, wait = True)
    move_group.go(joints_grab_10, wait = True)
    move_group.go(joints_grab_11, wait = True)

    print"robotino moving next position..."

    return EmptyResponse()


def release_Arm(data):
    global gripper_client, gripper_parallel_open, gripper_parallel, gripper_parallel_close, gripper_parallel_speed, gripper_parallel_effort  
    print "cobotta releacing objects..."
    print gripper_parallel_open, gripper_parallel_close
    move_group.go(joints_release_01,wait = True)
    move_group.go(joints_release_02,wait = True)
    #move_group.go(joints_release_03,wait = True)
    #gripper_move(gripper_client,gripper_parallel_open,gripper_parallel_speed,gripper_parallel_effort)
    #rospy.sleep(5)
    #move_group.go(joints_release_04,wait = True)
    move_group.go(joints_release_05,wait = True)
    #move_group.go(joints_release_03,wait = True)
    gripper_move(gripper_client,gripper_parallel_open,gripper_parallel_speed,gripper_parallel_effort)
    rospy.sleep(5)
    move_group.go(joints_release_02,wait = True)
    move_group.go(joints_release_01,wait = True)
    print"robotino moving next position..."

    return EmptyResponse()


#
#
#
#
#


#
# Parallel gripper
#
gripper_parallel_open   =  0.015
gripper_parallel_mid    =  0.0010
gripper_parallel_close  =  0.0001
gripper_parallel_speed  = 10.0
gripper_parallel_effort = 10.0

def gripper_move(gripper_client, width, speed, effort):
    goal = GripperMoveGoal()
    goal.target_position = width
    goal.speed = speed
    goal.effort = effort
    gripper_client.send_goal(goal)

def is_motor_running():
    rospy.wait_for_service('/cobotta/get_motor_state', 3.0)
    try:
        get_motor_state = rospy.ServiceProxy('/cobotta/get_motor_state',
                                             GetMotorState)
        res = get_motor_state()
        return res.state
    except rospy.ServiceException, e:
        print >> sys.stderr, "  Service call failed: %s" % e

def is_simulation():
    service_list = rosservice.get_service_list()
    if '/cobotta/get_motor_state' in service_list:
        return False
    return True

def arm_move_left():
    gripper_client = actionlib.SimpleActionClient('/cobotta/gripper_move',GripperMoveAction)
    gripper_width = 0.0

    arm = moveit_commander.MoveGroupCommander("arm")
    print("Reference frame: %s" % arm.get_planning_frame())
    print("Reference frame: %s" % arm.get_end_effector_link())

    target_pose_arm = arm.get_current_pose().pose
    print(target_pose_arm)

    gripper_width = gripper_parallel_open
    gripper_move(gripper_client, gripper_width,gripper_parallel_speed, gripper_parallel_effort)

#
#Test erea
#




if __name__ == '__main__':
    rospy.init_node("packing_pose")
    moveit_commander.roscpp_initialize(sys.argv)
    move_group = moveit_commander.MoveGroupCommander("arm")
    gripper_client=actionlib.SimpleActionClient('/cobotta/gripper_move',GripperMoveAction)
    
    print(os.path.basename(__file__) + " sets pose goal and moves COBOTTA.")
    print("start")
    src00 = rospy.Service('/btr/move_g',Empty,grab_Arm)
    src01 = rospy.Service('/btr/move_r',Empty,release_Arm)

    print"0: End"
    
    """while True:
        input = raw_input("  Select the value: ")
        if input.isdigit():
            input = int(input)
            if input == 0:
                break"""
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        rate.sleep()

        """ while True:
        input = raw_input("  Select the value: ")
        if input.isdigit():
            input = int(input)

        joints = []
        gripper_width = 0.0

        if input == 0:
            #joints = joints_packing_old

            arm_move_left()

            gripper_width = gripper_parallel_open

        elif input == 1:
            move_group.go(joint_1,wait = True)
            gripper_move(gripper_client, gripper_parallel_close,gripper_parallel_speed,gripper_parallel_effort)
            rospy.sleep(5)
            move_group.go(joint_2,wait = True)

            move_group.go(joint_3,wait = True)
            gripper_move(gripper_client, gripper_parallel_open, gripper_parallel_speed,gripper_parallel_effort)
            rospy.sleep(5)

        elif input == 2:
            #joints = joints_home
            gripper_move(gripper_client,gripper_parallel_close,gripper_parallel_speed,gripper_parallel_effort)
            rospy.sleep(5)

        elif input == 3:
            #joints = joints_demo
            gripper_move(gripper_client,gripper_parallel_open,gripper_parallel_speed,gripper_parallel_effort)
            rospy.sleep(5)

        elif input == 4:
            move_group.go(joint_first,wait = True)
            gripper_move(gripper_client,gripper_parallel_open,gripper_parallel_speed,gripper_parallel_effort)
            rospy.sleep(5)
            move_group.go(joint_top,wait = True)
            move_group.go(joint_grip,wait = True)
            gripper_move(gripper_client,gripper_parallel_close,gripper_parallel_speed,gripper_parallel_effort)
            rospy.sleep(5)
            move_group.go(joint_top,wait = True)
            rospy.sleep(1)
            move_group.go(joint_top2,wait = True)
            move_group.go(joint_release,wait = True)
            gripper_move(gripper_client,gripper_parallel_open,gripper_parallel_speed,gripper_parallel_effort)
            rospy.sleep(5)
            move_group.go(joint_top2,wait = True)
            move_group.go(joint_top,wait = True)
            move_group.go(joint_first,wait = True)
            
        elif input == 5
            
            print("True")
        else:
            break

        if not is_simulation() and is_motor_running() is not True:
            print >> sys.stderr, "  Please motor on."
            continue

        gripper_move(gripper_client, gripper_width,
                     gripper_parallel_speed, gripper_parallel_effort)
        #arm_move(move_group, joints)"""

    print("Bye...")
