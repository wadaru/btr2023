#!/usr/bin/python

import struct
import time
import math
import sys
import rospy
import numpy
from numpy import linalg
from scipy import interpolate
import quaternion
import tf
from geometry_msgs.msg import Vector3


from geometry_msgs.msg import Pose, Pose2D, PoseStamped, Point, Quaternion, Twist
from socket import socket, AF_INET, SOCK_DGRAM
from std_msgs.msg import Int8, Int16, UInt32, String, \
                         Float32, Float32MultiArray, \
                         Bool, Header
from std_srvs.srv import SetBool, SetBoolResponse, Empty, EmptyResponse
from nav_msgs.msg import Odometry
import rcll_ros_msgs
# import rcll_btr_msgs
from rcll_btr_msgs.msg import TagInfoResponse, TagLocationResponse
from rcll_btr_msgs.srv import SetOdometry, SetPosition, SetVelocity, \
                              SetDistance, TagInfo,     TagLocation
from robotino_msgs.srv import ResetOdometry

# linear max velocity is 0.1[m/s] and min is 0.01.
# angular max velocity is 1.0[rad/s?] and min is 0.01
min_mps_distance = 70
camera_offset = 100
turn_angle    = numpy.array([-999, -20, -10,   -5,   -2, -0.2, 0.1,     2,     5,    10,   20,  999])
turn_velocity = numpy.array([  5, 0.2, 0.1, 0.01, 0.01,    0,   0,- 0.01, -0.01, -0.1, -0.2, -5])

go_distance = numpy.array([-9999, -50,   -20,  -15, -10, 10,   15,   20,  50, 9999])
go_velocity = numpy.array([ -0.1,-0.1, -0.01,-0.01,   0,  0, 0.01, 0.01, 0.1,  0.1])

go_distance_fast = numpy.array([-9999, -20, -10,   -1, -0.9, 0,   1, 1.1,    5, 10,  20, 9999])
go_velocity_fast = numpy.array([ -0.1,-0.1,-0.1,-0.01,    0, 0,0.01,0.01,0.015,0.1, 0.1,  0.1])

move_distance = numpy.array([-99999, -1000, -500, -100,  -10, -9, 9,  10, 100, 500, 1000, 99999])
move_velocity = numpy.array([  -300,  -300,   -1,-0.05,-0.01,  0, 0,0.01,0.05,   1, 300,    300])

def quaternion_to_euler(quaternion):
    """Convert Quaternion to Euler Angles

    quarternion: geometry_msgs/Quaternion
    euler: geometry_msgs/Vector3
    """
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])

def tangent_angle(u, v):
    i = numpy.inner([u.y, u.x], [v.y, v.x])
    n = linalg.norm([u.y, u.x]) * linalg.norm([v.y, v.x])
    if (n == 0 ):
        return 90
    else:
        c = i / n
    return numpy.rad2deg(numpy.arccos(numpy.clip(c, -1.0, 1.0)))

def MPS_angle(u, v):
    # print(u, v)
    p0 = Point()
    p1 = Point()
    p0.x = 1.0
    p0.y = 0.0
    p1.x = v.x - u.x
    p1.y = v.y - u.y
    return tangent_angle(p1, p0)

class btr2023(object):
    def __init__(self):
        self.btrOdometry = Odometry()

        # rospy.init_node('btr2023')
        self.sub1 = rospy.Subscriber("/odom", Odometry, self.robotinoOdometry)
        self.sub3 = rospy.Subscriber("/btr/centerPoint", Point, self.centerPoint)
        self.sub4 = rospy.Subscriber("/btr/leftPoint", Point, self.leftPoint)
        self.sub5 = rospy.Subscriber("/btr/rightPoint", Point, self.rightPoint)
        self.rate = rospy.Rate(10)
        self.pub1 = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)

        # self.pose = Pose2D()
        # self.pose.x = -2500
        # self.pose.y = 500
        # self.pose.theta = 90
        # print("init", self.pose.x, self.pose.y, self.pose.theta)
        # self.setOdometry(self.pose)

        data = Pose2D()
        self.centerPoint = data
        self.leftPoint = data
        self.rightPoint = data

        self.startRpLidar()
        

    def startRpLidar(self):
        # setup for RPLidar
        rospy.wait_for_service('/btr/scan_start')
        scan_start = rospy.ServiceProxy('/btr/scan_start', Empty)
        resp = scan_start()

    def run(self):
        print("run")

    def w_resetOdometry(self, data):
        odometry = ResetOdometry()
        pose = Pose2D()
        rospy.wait_for_service('/reset_odometry')
        resetOdometry = rospy.ServiceProxy('/reset_odometry', ResetOdometry)
        # resp = resetOdometry(odometry.x, odometry.y, odometry.phi)
        resp = resetOdometry(data.x / 1000, data.y / 1000, data.theta / 180 * math.pi)

    def w_setVelocity(self, data):
        twist = Twist()
        twist.linear.x = data.x
        twist.linear.y = data.y
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = data.theta
        self.pub1.publish(twist)
        self.rate.sleep()

    def w_robotinoMove(self, x, y):
        global move_distance, move_velocity
        velocity1 = interpolate.interp1d(move_distance, move_velocity)
        while True:
            nowPoint = self.btrOdometry
            # print(nowAngle.pose.pose.position.z)
            if (nowPoint.header.seq != 0):
                break

        theta = nowPoint.pose.pose.position.z / 180 * math.pi
        print("theta", theta, nowPoint.pose.pose.position.z)
        target_x = x * math.cos(theta) - y * math.sin(theta) + nowPoint.pose.pose.position.x 
        target_y = x * math.sin(theta) + y * math.cos(theta) + nowPoint.pose.pose.position.y
        while True:
            diff_x1 = (target_x - self.btrOdometry.pose.pose.position.x)
            diff_y1 = (target_y - self.btrOdometry.pose.pose.position.y)
            diff_x = diff_x1 * math.cos(-theta) - diff_y1 * math.sin(-theta)
            diff_y = diff_x1 * math.sin(-theta) + diff_y1 * math.cos(-theta)
            v = Pose2D()
            if (math.isnan(diff_x) or math.isinf(diff_x)):
                v.x = 0
            else:
                v.x = velocity1(diff_x)
            if (math.isnan(diff_y) or math.isinf(diff_y)):
                v.y = 0
            else:
                v.y = velocity1(diff_y)
            v.theta = 0
            # if ((-0.001 < v.y) and (v.y < 0.001) and (-0.001 < v.x) and (v.x < 0.001)):
            #     v.y = 0
            #     v.x = 0
            # if (not(math.isnan(diff_y))) and (not(math.isnan(diff_x))):
            # print(diff_x, v.x, diff_y, v.y)
            print(v)
            self.w_setVelocity(v)
            if (v.x == 0) and (v.y == 0):
                break

    def w_goToInputVelt(self):    # 375mm from left side(= 25 + 50*7)
        # self.w_goToWall(min_mps_distance)
        self.w_goToMPSCenter()
        self.w_robotinoMove(0, 30)
        # self.w_goToWall(15)

    def w_goToOutputVelt(self):   # 325mm from left side (= 25 + 50*6)
        # self.w_goToWall(min_mps_distance)
        self.w_goToMPSCenter()
        self.w_robotinoMove(0, -30)
        # self.w_goToWall(15)

    def w_robotinoTurn(self, turnAngle):
        global turn_angle, turn_velocity
        velocity1 = interpolate.interp1d(turn_angle, turn_velocity)

        while True:
            print("turn")
            nowAngle = self.btrOdometry
            # print(nowAngle.pose.pose.position.z)
            if (nowAngle.header.seq != 0):
                break

        targetAngle = nowAngle.pose.pose.position.z + turnAngle
        if (targetAngle > 180):
            targetAngle -= 360
        if (targetAngle < -180):
            targetAngle += 360

        v = Pose2D()
        v.x = 0
        v.y = 0
        while True:
            diff = (targetAngle - self.btrOdometry.pose.pose.position.z)
            if (diff > 360):
                diff -= 360
            if (diff < -360):
                diff += 360
            v.theta = -velocity1(diff)
            self.w_setVelocity(v)
            # print(diff, v)
            if ((-3 < diff) and (diff < 3)):
                break
        v.theta = 0
        self.w_setVelocity(v)
        print("finish")

    def w_goToMPSCenter(self):
        global turn_angle, turn_velocity
        velocity1 = interpolate.interp1d(turn_angle, turn_velocity)
        rospy.wait_for_service('/btr_aruco/TagLocation')
        tagInfo = rospy.ServiceProxy('/btr_aruco/TagLocation', TagLocation)
        tag = tagInfo()
        print(tag)
        if (tag.ok == False):
            self.w_goToWall(min_mps_distance)
            tag = tagInfo()
            print(tag)
        if (tag.ok == True):
            degree = math.atan(tag.tag_location.y / tag.tag_location.x)
            if (tag.tag_location.y < 0):
                degree = -degree
            self.w_robotinoTurn(degree)
            print(tag.tag_location.y * 1000)
            self.w_robotinoMove(0, tag.tag_location.y * 1000)
            for i in range(2):
                # turn parallel for the face of MPS.
                self.w_parallelMPS()
                # goTo at the front of the MPS with 50cm.
                self.w_goToWall(min_mps_distance)
                # go to the front of the MPS.
                self.w_goToMPSCenterLRF()
            #self.w_goToWall(15)
            self.w_parallelMPS()

    def w_goToMPSCenterLRF(self):
        global go_distance, go_velocity
                
        velocityY = interpolate.interp1d(move_distance, move_velocity)
        while True:
            dist = self.leftPoint.y * 1000 + self.rightPoint.y * 1000
            v = Pose2D()
            v.x = 0
            if (math.isnan(dist) or math.isinf(dist)):
                v.y = 0
            else:
                v.y = velocityY(dist)
            v.theta = 0
            print("MPSCenter:", dist, v.y)
            # if ((-0.001 < v.y) and (v.y < 0.001)):
            #     v.y = 0
            # if (not(math.isnan(dist))):
            self.w_setVelocity(v)
            if (v.y == 0):
                break

    def w_goToWall(self, distance):
        global go_distance_fast, go_velocity_fast
        velocityX = interpolate.interp1d(go_distance_fast, go_velocity_fast)
        print("Wall ", distance)
        while True:
            sensor = self.centerPoint.x * 100
            v = Pose2D()
            if (math.isnan(sensor) or math.isinf(sensor)):
                if (distance < 17):
                    v.x = 0
                else:
                    v.x = -0.015
            else:
                v.x = velocityX(sensor - distance)
            v.y = 0
            v.theta = 0
            # print("Wall ", distance, "cm:", sensor, v.x)
            # if ((-0.001 < v.x) and (v.x < 0.001)):
            #     v.x = 0
            # if (not(math.isnan(sensor))):
            self.w_setVelocity(v)
            if (v.x == 0):
                break


    def w_parallelMPS(self):
        global turn_angle, turn_velocity
        velocity1 = interpolate.interp1d(turn_angle, turn_velocity)

        while True:
            angle1 = 90
            angle2 = 90
            angle3 = 90
            while ((angle1 == 90) or (angle2 == 90) or (angle3 == 90)):
                angle1 = MPS_angle(self.leftPoint, self.rightPoint)
                angle2 = MPS_angle(self.leftPoint, self.centerPoint)
                angle3 = MPS_angle(self.centerPoint, self.rightPoint)
                print(angle1)
            angle = (angle1 + angle2 + angle3) / 3.0
            # angle = (angle2 + angle3) / 2
            # angle = angle1
            print(angle, angle1, angle2, angle3)
            #
            v = Pose2D()
            v.x = 0
            v.y = 0
            if (math.isnan(angle)):
                angle = 90
            v.theta = velocity1(angle - 90)  

            print("parallelMPS:", angle, v.theta)
            # if ((-0.01 < v.theta) and (v.theta < 0.01)):
            #     v.theta = 0
            # if (not(math.isnan(angle))):
            self.w_setVelocity(v)
            if (v.theta == 0):
                break

    def w_turnClockwise(self):
        print("turnClockWise")
        self.w_goToWall(20)
        self.w_robotinoTurn(90)
        self.w_robotinoMove(700, 0)
        self.w_robotinoTurn(-90)
        self.w_robotinoMove(1200, 0)
        self.w_robotinoTurn(-90)
        self.w_robotinoMove(700, 0)
        self.w_robotinoTurn(-90)

    def w_turnCounterClockwise(self):
        print("turnCounterClockWise")
        self.w_goToWall(20)
        self.w_robotinoTurn(-90)
        self.w_robotinoMove(700, 0)
        self.w_robotinoTurn(90)
        self.w_robotinoMove(1200, 0)
        self.w_robotinoTurn(90)
        self.w_robotinoMove(700, 0)
        self.w_robotinoTurn(90)

    def w_getWork(self):
        rospy.wait_for_service('/btr/move_g')
        self.getWork = rospy.ServiceProxy('/btr/move_g', Empty)
        print("getWork")
        self.resp = self.getWork()
        print("finish")

    def w_putWork(self):
        rospy.wait_for_service('btr/move_r')
        self.putWork = rospy.ServiceProxy('/btr/move_r', Empty)
        print("putWork")
        self.resp = self.putWork()
        print("finish")

    def w_pick_rs(self):
        rospy.wait_for_service('/btr/pick_rs')
        self.pick_rs = rospy.ServiceProxy('/btr/pick_rs', Empty)
        print("pick rs")
        self.resp = self.pick_rs()
        print("finish")

    def w_put_rs(self):
        rospy.wait_for_service('/btr/put_rs')
        self.put_rs = rospy.ServiceProxy('/btr/put_rs', Empty)
        print("put rs")
        self.resp = self.put_rs()
        print("finish")

    def w_looking_for_C0(self):
        rospy.wait_for_service('/btr/looking_for_C0')
        self.looking_for_C0 = rospy.ServiceProxy('/btr/looking_for_C0', Empty)
        print("move")
        self.resp = self.looking_for_C0()
        print("finish")

    def w_move_g_C0(self):
        rospy.wait_for_service('/btr/move_g_C0')
        self.move_g_C0 = rospy.ServiceProxy('/btr/move_g_C0', Empty)
        print("grasp C0")
        self.resp = self.move_g_C0()
        print("finish")

    def w_move_scan(self):
        rospy.wait_for_service('/btr/move_s')
        self.move_g_C0 = rospy.ServiceProxy('/btr/move_s', Empty)
        print("scaning")
        self.resp = self.move_g_C0()
        print("finish")


    def robotinoOdometry(self, data):
        # global self.btrOdometry
        quat = quaternion_to_euler(Quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w))
        self.btrOdometry = data
        self.btrOdometry.pose.pose.position.x = data.pose.pose.position.x * 1000
        self.btrOdometry.pose.pose.position.y = data.pose.pose.position.y * 1000
        # self.btrOdometry.pose.pose.position.z = quat.z / math.pi * 180
        self.btrOdometry.pose.pose.position.z = data.pose.pose.position.z / math.pi * 180
        # print(self.btrOdometry)

    def centerPoint(self, data):
        self.centerPoint = data

    def leftPoint(self, data):
        self.leftPoint = data

    def rightPoint(self, data):
        self.rightPoint = data

    def w_getMPSLocation(self):
        rospy.wait_for_service('/btr_aruco/TagLocation')
        self.getTagLocation = rospy.ServiceProxy('/btr_aruco/TagLocation', TagLocation)
        print("getTagLocation")
        self.resp = self.getTagLocation()
        self.MPS_find = self.resp.ok
        if (self.resp.ok == False):
            return
        # 350 / 2 = the distance between MPS's tag and MPS's center.
        x = self.resp.tag_location.x * 1000 + camera_offset + 350 / 2
        y = self.resp.tag_location.y * 1000
        phi = self.resp.tag_location.theta
        # print("finish")
        # print(self.resp)

        while True:
            nowPoint = self.btrOdometry
            # print(nowAngle.pose.pose.position.z)
            if (nowPoint.header.seq != 0):
                break

        theta = nowPoint.pose.pose.position.z / 180 * math.pi
        target_x = x * math.cos(theta) - y * math.sin(theta) + nowPoint.pose.pose.position.x
        target_y = x * math.sin(theta) + y * math.cos(theta) + nowPoint.pose.pose.position.y
        self.MPS_id = self.resp.tag_id.data
        self.MPS_x = target_x
        self.MPS_y = target_y
        self.MPS_phi = -phi + nowPoint.pose.pose.position.z 
        if ((self.resp.tag_id.data % 2) == 1):
            self.MPS_phi += 180
        print(self.MPS_phi)
        self.MPS_phi = int((self.MPS_phi + 22.5) / 45) * 45
        self.MPS_phi = ((self.MPS_phi + 360) % 360)
        if self.MPS_x < 0:
            zone = "M"
        else:
            zone = "C"
        zone_x = int(abs(self.MPS_x) / 1000) + 1
        zone_y = int(abs(self.MPS_y) / 1000) + 1
        self.MPS_zone = zone + "_Z" + str(zone_x * 10 + zone_y)

#    robot.x = btrOdometry.pose.pose.position.x
#    robot.y = btrOdometry.pose.pose.position.y

# main
#
if __name__ == '__main__':
  args = sys.argv
  challenge = "test"
  if (len(args) == 2):
    challenge = args[1]

  print(challenge)
  challengeFlag = True

  agent = btr2023()
  # while True:
  while not rospy.is_shutdown():

    if (challenge == "test" and challengeFlag):
        agent.run()
        # agent.w_goToOutputVelt()
        w_turnClockwise()
        w_goToInputVelt()
        # w_turnCounterClockwise()
        challengeFlag = False

    agent.rate.sleep()


