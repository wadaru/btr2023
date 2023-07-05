#!/usr/bin/python
import struct
import time
import math
import sys
import rospy
import numpy
import cv2
# import robotino2022
import btr2023
from module_photographer import module_photographer
from module_work_detect import module_work_detect
from module_line_detect import module_line_detect

import quaternion
import tf
import rcll_ros_msgs
import rcll_btr_msgs
from geometry_msgs.msg import Pose, Pose2D, PoseStamped, PointStamped, Point, \
                              Quaternion, Vector3
from socket import socket, AF_INET, SOCK_DGRAM
from std_msgs.msg import Int8, Int16, UInt32, String, \
                         Float32, Float32MultiArray, \
                         Bool, Header
from std_srvs.srv import SetBool, SetBoolResponse, Empty, EmptyResponse
from nav_msgs.msg import Odometry
from rcll_btr_msgs.srv import SetOdometry, SetPosition, SetVelocity, \
                              SetDistance
from rcll_ros_msgs.msg import BeaconSignal, ExplorationInfo, \
                              ExplorationSignal, ExplorationZone, GameState, \
                              LightSpec, MachineInfo, Machine, \
                              MachineReportEntry, MachineReportEntryBTR, \
                              MachineReportInfo, OrderInfo, Order, \
                              ProductColor, RingInfo, Ring, Team, Time, \
                              NavigationRoutes, Route
from rcll_ros_msgs.srv import SendBeaconSignal, SendMachineReport, \
                              SendMachineReportBTR, SendPrepareMachine

TEAMNAME = "BabyTigers"

FIELDMINX = -5
FIELDMAXX = -1
FIELDMINY =  1
FIELDMAXY =  5
FIELDSIZEX = (FIELDMAXX - FIELDMINX) + 1
FIELDSIZEY = (FIELDMAXY - FIELDMINY) + 1
FIELDSIZE = FIELDSIZEX * FIELDSIZEY
MAXSTEP = 999

zoneX = { "S11" : -0.5, "S21" : -1.5, "S31" : -2.5, "S41" : -3.5, "S51" : -4.5,
          "S12" : -0.5, "S22" : -1.5, "S32" : -2.5, "S42" : -3.5, "S52" : -4.5,
          "S13" : -0.5, "S23" : -1.5, "S33" : -2.5, "S43" : -3.5, "S53" : -4.5,
          "S14" : -0.5, "S24" : -1.5, "S34" : -2.5, "S44" : -3.5, "S54" : -4.5,
          "S15" : -0.5, "S25" : -1.5, "S35" : -2.5, "S45" : -3.5, "S55" : -4.5 }
zoneY = { "S11" :  0.5, "S21" :  0.5, "S31" :  0.5, "S41" :  0.5, "S51" :  0.5,
          "S12" :  1.5, "S22" :  1.5, "S32" :  1.5, "S42" :  1.5, "S52" :  1.5,
          "S13" :  2.5, "S23" :  2.5, "S33" :  2.5, "S43" :  2.5, "S53" :  2.5,
          "S14" :  3.5, "S24" :  3.5, "S34" :  3.5, "S44" :  3.5, "S54" :  3.5,
          "S15" :  4.5, "S25" :  4.5, "S35" :  4.5, "S45" :  4.5, "S55" :  4.5 }
inputX = { 0: 1.0, 45: 0.5, 90:   0, 135: -0.5, 180: -1.0, 225: -0.5, 270:    0, 315:  0.5, 360: 1.0}
inputY = { 0:   0, 45: 0.5, 90: 1.0, 135:  0.5, 180:    0, 225: -0.5, 270: -1.0, 315: -0.5, 360:   0}
outputX = {  0: inputX[180],  45: inputX[225],  90: inputX[270], 135: inputX[315],
           180: inputX[  0], 225: inputX[ 45], 270: inputX[ 90], 315: inputX[135]}
outputY = {  0: inputY[180],  45: inputY[225],  90: inputY[270], 135: inputY[315],
           180: inputY[  0], 225: inputY[ 45], 270: inputY[ 90], 315: inputY[135]}
machineName = { 101 : "C-CS1-O", 102 : "C-CS1-I", 103 : "C-CS2-O", 104 : "C-CS2-I",
                201 : "M-CS1-O", 202 : "M-CS1-I", 203 : "M-CS2-O", 204 : "M-CS2-I",
                111 : "C-RS1-O", 112 : "C-RS1-I", 113 : "C-RS2-O", 114 : "C-RS2-I",
                211 : "M-RS1-O", 212 : "M-RS1-I", 213 : "M-RS2-O", 214 : "M-RS2-I",
                121 : "C-BS-O",  122 : "C-BS-I",  221 : "M-BS-O",  222 : "M-BS-I",
                131 : "C-DS-O",  132 : "C-DS-I",  231 : "M-DS-O",  232 : "M-DS-I",
                141 : "C-SS-O",  142 : "C-SS-I",  241 : "M-SS-O",  242 : "M-SS-I",
                301 : "UMPS-1",  302 : "UMPS-2" }

def quaternion_to_euler(quaternion):
    """Convert Quaternion to Euler Angles

    quarternion: geometry_msgs/Quaternion
    euler: geometry_msgs/Vector3
    """
    e = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])

def beaconSignal(data):
    global refboxBeaconSignal
    refboxBeaconSignal = data
    # print("BeaconSignal: ", data)

def explorationInfo(data):
    global refboxExplorationInfo
    refboxExplorationInfo = data
    # print("ExplorationInfo: ", data)

def gameState(data):
    global refboxTime, refboxGameState, refboxGamePhase, \
           refboxPointsMagenta, refboxTeamMagenta, \
           refboxPointCyan, refboxTeamCyan
    refboxTime = data.game_time
    refboxGameState = data.state
    refboxGamePhase = data.phase
    refboxPointsMagenta = data.points_magenta
    refboxTeamMagenta = data.team_magenta
    refboxPointsCyan = data.points_cyan
    refboxTeamCyan = data.team_cyan
    # print("GameState: ", data)
    sendBeacon()

def machineInfo(data):
    global refboxMachineInfo, refboxMachineInfoFlag
    refboxMachineInfo = data
    refboxMachineInfoFlag = True
    # print("MachineInfo: ", data)

def machineReportInfo(data):
    global refboxMachineReportInfo
    refboxMachineReportInfo = data
    # print("MachineReportInfo: ", data)

def orderInfo(data):
    global refboxOrderInfo
    refboxOrderInfo = data
    # print("OrderInfo: ", data)

def ringInfo(data):
    global refboxRingInfo
    refboxRingInfo = data
    # print("RingInfo: ", data)

def navigationRoutes(data):
   global refboxNavigationRoutes, refboxNavigationRoutesFlag
   refboxNavigationRoutes = data
   refboxNavigationRoutesFlag = True
   # print("NavigaionRoutes: ", data)

#
# send information to RefBox
#
def sendBeacon():
    global btrOdometry
    beacon = SendBeaconSignal()
    beacon.header = Header()

    # for poseStamped()
    beacon.pose = PoseStamped()
    beacon.pose.pose.position.x = btrOdometry.pose.pose.position.x # / 1000
    beacon.pose.pose.position.y = btrOdometry.pose.pose.position.y # / 1000
    beacon.pose.pose.position.z = 0
    beacon.pose.pose.orientation.x = btrOdometry.pose.pose.orientation.x
    beacon.pose.pose.orientation.y = btrOdometry.pose.pose.orientation.y
    beacon.pose.pose.orientation.z = btrOdometry.pose.pose.orientation.z
    beacon.pose.pose.orientation.w = btrOdometry.pose.pose.orientation.w
    beacon.header.seq = 1
    beacon.header.stamp = rospy.Time.now()
    beacon.header.frame_id = TEAMNAME
    beacon.pose.header.seq = 1
    beacon.pose.header.stamp = rospy.Time.now()
    beacon.pose.header.frame_id = "robot1"

    rospy.wait_for_service('/rcll/send_beacon')
    try:
        refboxSendBeacon = rospy.ServiceProxy('/rcll/send_beacon', SendBeaconSignal)
        resp1 = refboxSendBeacon(beacon.header, beacon.pose)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def sendMachineReport(report):
    sendReport = SendMachineReport()
    machineReport = MachineReportEntryBTR()
    machineReport.name = report.name
    machineReport.type = report.type
    machineReport.zone = report.zone
    machineReport.rotation = report.rotation
    if (refboxTeamCyan == TEAMNAME):
        sendReport.team_color = 1
    else:
        sendReport.team_color = 2
    machineReportEntryBTR = [machineReport]
    sendReport.machines = machineReportEntryBTR
    print("machineReport: ", machineReport)

    rospy.wait_for_service('/rcll/send_machine_report')
    try:
        refboxMachineReport = rospy.ServiceProxy('/rcll/send_machine_report', SendMachineReportBTR)
        resp1 = refboxMachineReport(sendReport.team_color, sendReport.machines)
        # print("resp: ", resp1)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def sendPrepareMachine(data):
    prepare = SendPrepareMachine()
    # prepare.machine = data.machine
    prepare.machine = data.machine
    prepare.bs_side = 0
    prepare.bs_base_color = 0
    prepare.ds_order_id = 0
    prepare.cs_operation = 0
    prepare.rs_ring_color =0
    
    machineType = prepare.machine[2:4]
    print(machineType)
    if (machineType == "BS"):
        prepare.bs_side = data.bs_side
        prepare.bs_base_color =data.bs_base_color
    if (machineType == "DS"):
        prepare.ds_order_id = data.ds_order_id
    if (machineType == "CS"):
        prepare.cs_operation = data.cs_operation
    if (machineType == "RS"):
        prepare.rs_ring_color = data.rs_ring_color
    prepare.wait = data.wait
    rospy.wait_for_service('/rcll/send_prepare_machine')
    try:
        refboxPrepareMachine = rospy.ServiceProxy('/rcll/send_prepare_machine', SendPrepareMachine)
        resp1 = refboxPrepareMachine(prepare.machine, prepare.wait, prepare.bs_side, prepare.bs_base_color, prepare.ds_order_id, prepare.rs_ring_color, prepare.cs_operation)
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def robotinoOdometry(data):
    global btrOdometry, btrBeaconCounter
    quat = quaternion_to_euler(Quaternion(data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w))
    btrOdometry = data
    ##btrOdometry.pose.pose.position.z = quat.z / math.pi * 180
    btrOdometry.pose.pose.position.z = btrOdometry.pose.pose.position.z # / math.pi * 180
    # btrOdometry = data
    btrBeaconCounter +=1
    if (btrBeaconCounter > 5):
      sendBeacon()
      btrBeaconCounter = 0

def w_findMPS():
    btrRobotino.w_getMPSLocation()
    if (btrRobotino.MPS_find == True):
        name = machineName[btrRobotino.MPS_id]
        print(name, btrRobotino.MPS_zone, btrRobotino.MPS_phi)
        machineReport.name = name[0: len(name) - 2]
        if (name[4 : 5] == "-"):
            machineReport.type = name[2 : 4]
        else:
            machineReport.type = name[2 : 5]
            zone = int(btrRobotino.MPS_zone[3 : 5])
        if (btrRobotino.MPS_zone[0: 1] == "M"):
            zone = -zone
        machineReport.zone = zone
        machineReport.rotation = btrRobotino.MPS_phi
        sendMachineReport(machineReport)
    return btrRobotino.MPS_find

def goToPoint(x, y, phi):
    btrRobotino.w_robotinoMove(0, 0)
    btrRobotino.w_waitOdometry()
    nowX = btrOdometry.pose.pose.position.x
    nowY = btrOdometry.pose.pose.position.y
    nowPhi = btrOdometry.pose.pose.position.z
    dist = ((x - nowX)**2 + (y - nowY)**2) **0.5
    # print(dist)
    if (dist > 0.30):
      turn = numpy.rad2deg(numpy.arctan2(y - nowY, x - nowX))
      print(nowX, nowY, nowPhi, "=>", x, y, phi)
      # print(turn, turn - nowPhi)
      btrRobotino.w_robotinoTurn(turn - nowPhi)
      btrRobotino.w_robotinoMove(dist, 0)
    else:
      moveX = x - nowX
      moveY = y - nowY
      # print(moveX, moveY, nowPhi)
      rad = math.radians(nowPhi)
      distX = moveX * math.cos(-rad) - moveY * math.sin(-rad)
      distY = moveX * math.sin(-rad) + moveY * math.cos(-rad)
      btrRobotino.w_robotinoMove(distX, distY)
    nowPhi = btrOdometry.pose.pose.position.z
    # print(phi, phi - nowPhi)
    btrRobotino.w_robotinoTurn(phi - nowPhi)
    # btrRobotino.w_robotinoMove(x, y)
    # btrRobotino.w_robotinoTurn(phi)
# 
# challenge program
#

def startGrasping():
    pg = module_photographer()
    for _ in range(3):
        challengeFlag = False
        btrRobotino.w_goToOutputVelt()
        # btrRobotino.w_robotinoMove(0, 25)
        btrRobotino.w_goToWall(0.35)
        btrRobotino.w_parallelMPS()

        name = "g_ref_img"
        a_previous = 0
        for i in range(10):
            pg.g_run()
            rospy.sleep(1)
            img = cv2.imread("{}.jpg".format(name), 0)
            wd = module_work_detect(img)
            ato_take = wd.detect()
            if ato_take == -1: # left
                btrRobotino.w_robotinoMove(0, -0.015)
            elif ato_take == 1: # right
                btrRobotino.w_robotinoMove(0, 0.015)
            elif ato_take == 2: # none detected
                btrRobotino.w_robotinoMove(0, -a_previous * 0.015)
            else:
                break
            a_previous = int((-ato_take) * (ato_take % 2))

        #btrRobotino.w_goToOutputVelt()
        btrRobotino.w_goToWall(0.15)
        btrRobotino.w_getWork()

        if (robotNum != 2):
            btrRobotino.w_turnClockwise()
        else:
            btrRobotino.w_turnCounterClockwise()

        btrRobotino.w_goToInputVelt()
        btrRobotino.w_goToWall(0.35)
        btrRobotino.w_parallelMPS()

        name = "r_ref_img"
        a_previous = 0
        for i in range(10):
            pg.r_run()
            rospy.sleep(1)
            img = cv2.imread("{}.jpg".format(name), 0)
            ld = module_line_detect(img)
            ato_take = ld.detect()
            if ato_take == -1: # left
                btrRobotino.w_robotinoMove(0, -0.15)
            elif ato_take == 1: # right
                btrRobotino.w_robotinoMove(0, 0.15)
            elif ato_take == 2: # none detected
                btrRobotino.w_robotinoMove(0, -a_previous * 0.15)
            else:
                break
            a_previous = int((-ato_take) * (ato_take % 2))
        btrRobotino.w_goToWall(0.15)

        btrRobotino.w_putWork()
        if (robotNum != 2):
            btrRobotino.w_turnCounterClockwise()
        else:
            btrRobotino.w_turnClockwise()

        print("{} / 3 finished!".format(_+1))


def initField():
    global btrField
    btrField = [[0 for y in range(FIELDSIZEY)] for x in range(FIELDSIZEX)]
    for zone in range(2):
        for x in range(1, 8):
            for y in range(1, 8):
                zoneName = str(zone * 100 + x) + str(y)
                zoneX[zoneName] = (x - 0.5) * (-zone * 2 + 1)
                zoneY[zoneName] = y - 0.5
                # print(zoneName, zoneX[zoneName], zoneY[zoneName])
    #
    # this field is [y][x]
    # but game field is from -5 to -1
    # so when you use this variable, please shift argment + 5.
    #   (-5, 5) => (0, 4)
    #   (-5 ,1) => (0, 0)

def setField(x, y, number):
    global FIELDMINX, FIELDMINY
    # print(x, FIELDMINX, y, FIELDMINX, number)
    btrField[y - FIELDMINY][x - FIELDMINX] = number

def getField(x, y):
    global FIELDMINX, FIELDMAXX, FIELDMINY, FIELDMAXY, MAXSTEP
    if (int(x) < FIELDMINX or FIELDMAXX < int(x) or int(y) < FIELDMINY or FIELDMAXY < int(y)):
        # print("getField range over: ", x, y)
        return MAXSTEP 
    return btrField[int(y) - FIELDMINY][int(x) - FIELDMINX]

def zoneToPose2D(zone):
    point = Pose2D()
    if zone < 0:        
        zone = zone + 255
    point.y = zone % 10
    point.x = (zone % 100) // 10
    if (zone > 100):
        point.x = -point.x
    return point

def setMPStoField():
    global btrField
    point = Pose2D()
    for machine in refboxMachineInfo.machines:
        point = zoneToPose2D(machine.zone)
        print("setMPS: ", machine.name, machine.zone, point.x, point.y)
        if (point.x == 0 and point.y == 0):
            print("received NULL data for MPS", machine.name)
        else:
            setField(point.x, point.y, MAXSTEP)

def getStep(x, y):
    global FIELDMINX, FIELDMAXX, FIELDMINY, FIELDMAXY, MAXSTEP
    if ((x < FIELDMINX or FIELDMAXX < x) or (y < FIELDMINY or FIELDMAXY < y)):
        return MAXSTEP

    step = getField(x, y)
    if (step == 0):
        step = MAXSTEP
    return step

def wallCheck(x, y, dx, dy):
    notWallFlag = True
    if ((x == -5 and y == 1) and (dx ==  0 and dy ==  1)):
        notWallFlag = False
    if ((x == -4 and y == 1) and (dx ==  0 and dy ==  1)):
        notWallFlag = False
    if ((x == -3 and y == 1) and (dx ==  1 and dy ==  0)):
        notWallFlag = False
    if ((x == -2 and y == 1) and (dx == -1 and dy ==  0)):
        notWallFlag = False
    if ((x == -5 and y == 2) and (dx ==  0 and dy == -1)):
        notWallFlag = False
    if ((x == -4 and y == 2) and (dx ==  0 and dy == -1)):
        notWallFlag = False
    return notWallFlag

def getNextDirection(x, y):
    minStep = getField(x, y)
    nextD = Pose2D()
    nextD.x = nextD.y = 0
    for dx, dy in zip([-1, 1, 0, 0], [0, 0, -1, 1]):
        notWallFlag = wallCheck(x, y, dx, dy)

        if ((minStep > getField(x + dx, y + dy)) and notWallFlag):
            minStep = getField(x + dx, y + dy)
            nextD.x = dx
            nextD.y = dy
    return nextD

def makeNextPoint(destination):
    global btrField, btrOdometry, FIELDMINX, FIELDMAXX, FIELDMINY, FIELDMAXY, MAXSTEP
    tmpField = btrField
    point = zoneToPose2D(destination)
    print("destination is ", destination, point.x, point.y)
    setField(point.x, point.y, 1)
    for i in range(FIELDSIZE):
        for x in range(FIELDMINX, FIELDMAXX + 1):
            for y in range(FIELDMINY, FIELDMAXY + 1):
                if (x == point.x and y == point.y):
                    setField(x, y, 1)
                elif (getField(x, y) != MAXSTEP):
                    setField(x, y, min(getStep(x - 1, y), getStep(x, y - 1), \
                                       getStep(x + 1, y), getStep(x, y + 1)) \
                                   + 1)
                    # wall information
                    if (x == -5 and y == 1): # M_Z51 = M_Z41 + 1
                        setField(x, y, getStep(x + 1, y) + 1)
                    if (x == -4 and y == 1): # M_Z41 = M_Z31 + 1
                        setField(x, y, getStep(x + 1, y) + 1)
                    if (x == -3 and y == 1): # M_Z31 = M_Z32 + 1
                        setField(x, y, getStep(x, y + 1) + 1)
                    if (x == -2 and y == 1): # M_Z21 <= min(M_Z22, MZ_11) + 1
                        setField(x, y, min(getStep(x, y + 1), getStep(x + 1, y)) + 1)

    # get optimized route
    if (False):
        for y in range(FIELDMAXY, FIELDMINY  - 1, -1):
            for x in range(FIELDMINX, FIELDMAXX + 1):
                if (getField(x,y) == MAXSTEP):
                    print("*",)
                else:
                    print(getField(x, y),)
            print()

    robotReal = Pose2D()
    robotZone = Pose2D()
    point = Pose2D()
    btrRobotino.w_waitOdometry()
    robotReal.x = btrOdometry.pose.pose.position.x
    robotReal.y = btrOdometry.pose.pose.position.y

    if (robotReal.x > 0):
        robotZone.x = int(robotReal.x) + 1
    else:
        robotZone.x = int(robotReal.x) - 1
    robotZone.y = int(robotReal.y) + 1

    x = int(robotZone.x)
    y = int(robotZone.y)
    # which direction?
    nextD = getNextDirection(x, y)
    # where is the turning point?
    point.x = robotZone.x + nextD.x
    point.y = robotZone.y + nextD.y
    # print("direction: ",nextD.x, nextD.y)
    for dx, dy, phi in zip([ 1, 0, -1, 0, 0], [0, 1, 0, -1, 0], [0, 90, 180, -90, 360]):
        if (nextD.x == dx and nextD.y == dy):
            theta = phi

    goToPoint(robotReal.x, robotReal.y, theta) # turn for the next point.

    while True:
        # print(point)
        if getField(point.x, point.y) == 0:
            print("Next Point is goal")
            break
        if (getField(point.x + nextD.x, point.y + nextD.y) == getField(point.x, point.y) - 1):
            notWallFlag = wallCheck(point.x, point.y, nextD.x, nextD.y)
            if (notWallFlag):
                point.x = point.x + nextD.x
                point.y = point.y + nextD.y
            else:
                # there is a wall.
                break
        else:
            print("Next Point is the turning point")
            break

    # for the nextStep
    nextDD = getNextDirection(point.x, point.y)
    theta = -360
    for dx, dy, phi in zip([ 1, 0, -1, 0, 0], [0, 1, 0, -1, 0], [0, 90, 180, -90, 360]): 
        if (nextDD.x == dx and nextDD.y == dy):
           theta = phi

    print("nextPosition: ", point.x, point.y, theta)
    if (point.x == robotZone.x and point.y == robotZone.y):
        theta = 360
    point.theta = theta
    return point

def getNextPoint(pointNumber):
    point = Pose2D()
    route = refboxNavigationRoutes.route
    # zone = route[pointNumber].zone
    zone = route[0].zone

    print(zone)
    point = makeNextPoint(zone)
    return point

def startNavigation():
    global btrField
    # initField()
    print("----")
    setMPStoField()
    print("====")
    oldTheta = 90
    for pointNumber in range(12):
        print(pointNumber)
        route = refboxNavigationRoutes.route
        if (len(route) == 0):
            print("finished")
        else:
            point = getNextPoint(pointNumber)
            robot = btrOdometry.pose.pose.position
            while True:
                point = getNextPoint(pointNumber)
                if (point.x > 0):
                    point.x = point.x * 1.0 - 0.5
                else:
                    point.x = point.x * 1.0 + 0.5
                point.y = point.y * 1.0 - 0.5
                if (point.theta == 360):
                    goToPoint(point.x, point.y, oldTheta)
                    break
                else:
                    goToPoint(point.x, point.y, point.theta)
                oldTheta = point.theta

            print("arrived #", pointNumber + 1, ": point")
            for i in range(4):
                sendBeacon()
                rospy.sleep(2)

    print("****")
    # print(refboxNavigationRoutes)
    # print(refboxMachineInfo)


# main
#
if __name__ == '__main__':
  args = sys.argv
  topicName = ""
  if (len(args) >= 2):
    challenge = args[1]
    if (len(args) >= 3):
      robotNum = int(args[2])
    else:
      robotNum = 1
    if challenge == "gazebo":
      topicName = "/robotino" + str(robotNum)

  # valiables for refbox
  refboxBeaconSignal = BeaconSignal()
  refboxExplorationInfo = ExplorationInfo()
  refboxExplorationSignal = ExplorationSignal()
  refboxExplorationZone = ExplorationZone()
  refboxGameState = Int8()
  refboxGamePhase = Int8()
  refboxPointsMagenta = UInt32()
  refboxTeamMagenta = String()
  refboxPointsCyan = UInt32()
  refboxTeamCyan = String()
  refboxLightSpec = LightSpec()
  refboxMachineInfo = MachineInfo()
  refboxMachine = Machine()
  refboxMachineReportEntry = MachineReportEntryBTR()
  refboxMachineReportInfo = MachineReportInfo()
  refboxOrderInfo = OrderInfo()
  refboxOrder = Order()
  refboxProductColor = ProductColor()
  refboxRingInfo = RingInfo()
  refboxRing = Ring()
  refboxTeam = Team()
  refboxTime = Time()
  refboxNavigationRoutes = NavigationRoutes()
  refboxMachineInfoFlag = False
  refboxNavigationRoutesFlag = False

  btrOdometry = Odometry()
  btrBeaconCounter = 0
  btrVelocity = Float32MultiArray()

  btrField = [[0 for y in range(5)] for x in range(5)]

  nodeName = "btr2023_" + str(robotNum)
  rospy.init_node(nodeName)
  rospy.Subscriber("rcll/beacon", BeaconSignal, beaconSignal)
  rospy.Subscriber("rcll/exploration_info", ExplorationInfo, explorationInfo)
  rospy.Subscriber("rcll/game_state", GameState, gameState)
  rospy.Subscriber("rcll/machine_info", MachineInfo, machineInfo)
  rospy.Subscriber("rcll/machine_report_info", MachineReportInfo, machineReportInfo)
  rospy.Subscriber("rcll/order_info", OrderInfo, orderInfo)
  rospy.Subscriber("rcll/ring_info", RingInfo, ringInfo)
  rospy.Subscriber(topicName + "/odom", Odometry, robotinoOdometry)
  rospy.Subscriber("rcll/routes_info", NavigationRoutes, navigationRoutes)
  rate = rospy.Rate(10)

  machineReport = MachineReportEntryBTR()
  prepareMachine = SendPrepareMachine() 

  print(topicName)
  btrRobotino = btr2023.btr2023(topicName)

  if (challenge == "reset"):
      goToPoint(-3.5,  1.5, 90)
      goToPoint(-3.5,  0.5, 90)
      goToPoint(pose.x, pose.y, pose.theta)
      exit()

  pose = Pose2D()
  pose.x = -1.0 * robotNum - 1.5
  pose.y = 0.5
  pose.theta = 90
  if (challenge == "grasping"):
      startX =     [ -0.5, -4.5, -0.5]
      startY =     [  0.5,  1.5,  4.5]
      startTheta = [   90,   90,  180]
      pose.x = startX[robotNum - 1]
      pose.y = startY[robotNum - 1]
      pose.theta = startTheta[robotNum - 1]
  if (challenge == "driving" or challenge == "positioning"):
      pose.x = zoneX["S31"]
      pose.y = zoneY["S31"]
      pose.theta = 90
  if (challenge == "gazebo" or challenge == "rcll" or challenge == "test"):
      pose.x = -(pose.x - 2.0)

  print(pose.x, pose.y, pose.theta)
  btrRobotino.w_resetOdometry(pose)
  # time.sleep(3)
  btrRobotino.w_waitOdometry()

  print(challenge)
  challengeFlag = True
  initField()
  # while True:
  while not rospy.is_shutdown():
    # sendBeacon()
    # print("sendBeacon")
    
    if (challenge == "nbr33" and challengeFlag):
        challengeFlag = False
        # goTo S33
        goToPoint(zoneX["S33"], zoneY["S33"], 90)
        for j in range(2):
            for i in range(9):
                # btrRobotino.w_findMPS()
                w_findMPS()
                btrRobotino.w_robotinoTurnAbs(45 * i)
            print(j)
            time.sleep(3)
        
        goToPoint(zoneX["S31"], zoneY["S31"], 90)
        break

    if (challenge == "gripping" and challengeFlag):
        slotNo = 3
        if (True):
            moveGo   = [-0.100, -0.220, -0.305]
            moveBack = [ 0.105,  0.220,  0.310]
            btrRobotino.w_goToInputVelt()
            btrRobotino.w_robotinoMove(0,  moveGo[  slotNo - 1])
            btrRobotino.w_getWork()
            # btrRobotino.w_goToInputVelt()
            btrRobotino.w_robotinoMove(0,  moveBack[slotNo - 1])
            btrRobotino.w_putWork()
        challengeFlag = False

    if (challenge == "graspingTest" and challengeFlag):
        startGrasping()
        challengeFlag = False
        break

    if (challenge == "driving" and challengeFlag):
        print("startDriving for JapanOpen2020")
        targetZone =  ["S31", "S35", "S15", "S13", "S33", "S31", "S31", "S31"]
        #                            Target1               Target2
        targetAngle = [   90,     0,   270,   180,   270,    90,    90,    90]
        sleepTime   = [    0,     5,     0,     5,     0,     5,     0,     1]
        for number in range(len(targetZone)):
            print(targetZone[number])
            x = zoneX[targetZone[number]]
            y = zoneY[targetZone[number]]
            theta = targetAngle[number]
            goToPoint(x, y, theta)
            time.sleep(sleepTime[number])

        challengeFlag = False
        break

    if (challenge == "positioning" and challengeFlag):
        print("startPositioning for JapanOpen2020")
        #
        # MPSZone, MPSAngle, firstSide, turn
        #
        MPSZone = "S34" # Input !!!
        MPSAngle = 180  # Input !!!
        firstSide = "input" # Check!!
        turn = "clock" # Check!!

        # goTo S322
        goToPoint(zoneX["S32"], zoneY["S32"], 90)

        if (firstSide == "input"):
            MPSx = zoneX[MPSZone] + inputX[MPSAngle]
            MPSy = zoneY[MPSZone] + inputY[MPSAngle]
            theta = MPSAngle + 180
        else:
            MPSx = zoneX[MPSZone] + outputX[MPSAngle]
            MPSy = zoneY[MPSZone] + outputY[MPSAngle]
            theta = MPSAngle
        
        goToPoint(MPSx, MPSy, theta)
        btrRobotino.w_goToMPSCenter()
        if (firstSide == "input"):
            print("wait")
            time.sleep(10)

        btrRobotino.w_goToWall(20)
        if (turn == "clock"):
            btrRobotino.w_turnClockwise()
        else:
            btrRobotino.w_turnCounterClockwise()
        btrRobotino.w_goToMPSCenter()
        print("wait")
        time.sleep(10)
        
        btrRobotino.w_goToWall(20)
        if (turn == "clock"):
            btrRobotino.w_turnCounterClockwise()
        else:
            btrRobotino.w_turnClockwise()
        if (firstSide == "output"):
            btrRobotino.w_goToMPSCenter() 
            print("wait")
            time.sleep(10)
        
        theta = 270
        goToPoint(MPSx, MPSy, theta)

        # goTo S32 & S31
        goToPoint(zoneX["S32"], zoneY["S32"], 270)
        goToPoint(zoneX["S31"], zoneY["S31"], 90)
        challengeFlag = False
        break

    if (refboxGamePhase == 30 and challenge == "grasping" and challengeFlag):
        startGrasping()
        challengeFlag = False
        break

    if (challenge == "navigationTest" and challengeFlag):
        startNavigation()
        challengeFlag = False
        break

    if (refboxGamePhase == 30 and challenge == "navigation" and challengeFlag):
        if (refboxMachineInfoFlag and refboxNavigationRoutesFlag):
            startNavigation()
            challengeFlag = False
            break

    # send machine prepare command
    if (refboxGamePhase == 30 and challenge == "" ):
        # make C0
        # which requires get base with cap from shelf at C-CS1, 
        #                Retrieve cap at C-CS1,
        #                bring base without cap to C-RS1,
        #                get base at C-BS,
        #                bring base to C-CS1,
        #                Mount cap at C-CS1,
        #                bring it to C-DS corresponded by order it.

        if (refboxTime.sec ==   5):
            prepareMachine.machine = "C-CS1"
            prepareMachine.cs_operation = 1 # CS_OP_RETRIEVE_CAP
            prepareMachine.wait = True
            sendPrepareMachine(prepareMachine)
        if (refboxTime.sec ==  30):
            prepareMachine.machine = "C-BS"
            prepareMachine.bs_side = 1  # INPUT or OUTPUT side
            prepareMachine.bs_base_color = 1 # BASE COLOR
            prepareMachine.wait = True
            sendPrepareMachine(prepareMachine)
        if (refboxTime.sec ==  60):
            prepareMachine.machine = "C-CS1"
            prepareMachine.cs_operation = 0 # CS_OP_MOUNT_CAP
            prepareMachine.wait = True
            sendPrepareMachine(prepareMachine)
        if (refboxTime.sec ==  90):
            prepareMachine.machine = "C-DS"
            prepareMachine.ds_order_id = 1 # ORDER ID
            prepareMachine.wait = True
            sendPrepareMachine(prepareMachine)
            
    if ( challenge == "gazebo"):
        sendBeacon()
        print(challenge)
        if (refboxGamePhase == 30 and challengeFlag):
            if (refboxTime.sec <= 180): # for exploration
                challengeFlag = False
                print("GoToExploration")
                goToPoint(zoneX["51"], zoneY["51"],  90)
                goToPoint(zoneX["52"], zoneY["52"],   0)
                # findMPS
                for i in range(-2, 2):
                    # btrRobotino.w_findMPS()
                    w_findMPS()
                    btrRobotino.w_robotinoTurnAbs(45 * i)


    if ( challenge == "test" and challengeFlag):
        challengeFlag = False
        sendBeacon()
        
        print("test1")
        goToPoint(zoneX["51"], zoneY["51"],  90)
        print("test2")
        goToPoint(zoneX["52"], zoneY["52"],   0)
        print("test3")

        # if (btrRobotino.w_findMPS() == True):
        if (w_findMPS() == True):
          print("test4")
          btrRobotino.w_goToOutputVelt()
          print("test5")
        # btrRobotino.w_goToWall(0.015 + 0.020)
        # btrRobotino.w_parallelMPS()
        # btrRobotino.w_findMPS()
        print("test finished")

    sendBeacon()
    rate.sleep()


