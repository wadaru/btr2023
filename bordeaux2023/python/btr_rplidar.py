#!/usr/bin/env python
#!/usr/bin/python

START_ANGLE = -90 # -90
END_ANGLE = 90 # 90
START_EDGE_ANGLE = -60 # -120
END_EDGE_ANGLE = 60 # -60
THRESHOLD_ANGLE = 5 

import rospy
import math
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyResponse

topicName = ""
#
def scanDistance(deg):
  global topicName
  if (topicName == ""):
    return scanData.ranges[int(len(scanData.ranges) / 360 * ((deg + 360) % 360))]
  else:
    # the range of gazebo's laser is from START_ANGLE to END_ANGLE?
    return scanData.ranges[int(len(scanData.ranges) / (END_ANGLE - START_ANGLE) * (deg - START_ANGLE))]

#
def polarToPoint(distance, angle):
  global topicName
  point = Point()
  if (topicName == ""):
    radian = math.radians(angle)
  else:
    radian = math.radians(angle)
  # point.x = distance * math.cos(radian)
  # point.y = distance * math.sin(radian)
  point.x = distance * math.cos(radian)
  point.y = distance * math.sin(radian)
  point.z = angle + 0
  return point

#
def calcAngle(pointA, pointB):
  point = Point()
  point.x = pointA.x - pointB.x
  point.y = pointA.y - pointB.y
  return math.degrees(math.atan2(point.y, point.x))

#
def findEdge(startAngle, angleStep):
  oldPoint = polarToPoint(scanDistance(startAngle - angleStep), startAngle - angleStep)
  i = startAngle
  oldAngle = -360

  while True:
    nowPoint = polarToPoint(scanDistance(i), i)
    if (math.isinf(scanDistance(i))):
      break
    dist = scanDistance(i - angleStep) - scanDistance(i)
    if (dist > 0.10):
      break
    i = i + angleStep
    if (i < -180 or i > 180):
      break
  
  # print("findEdge: ", i - angleStep, scanDistance(i - angleStep))
  return polarToPoint(scanDistance(i - angleStep), i - angleStep)

#
def calcPoint():
  global centerPoint, closePoint, leftPoint, rightPoint
  minDistance = scanDistance((START_ANGLE + END_ANGLE) / 2)
  minAngle = (START_ANGLE + END_ANGLE) / 2
  centerPoint = polarToPoint(minDistance, minAngle)
  # print(minDistance, minAngle)
  # print(len(scanData.ranges) / 360 , (((minAngle + 180 + 45) + 360) % 360))

  for i in range(START_EDGE_ANGLE, END_EDGE_ANGLE):
    if (minDistance > scanDistance(i)):
      minDistance = scanDistance(i)
      minAngle = i
  # print("minAngle:", minAngle, ", minDistance:", minDistance)
  closePoint = polarToPoint(minDistance, minAngle)

  # find the left edge and right edge
  # leftPoint = findEdge(minAngle - 1, -1)
  # rightPoint = findEdge(minAngle + 1, 1)
  leftPoint  = findEdge(minAngle - 1, -1)
  rightPoint = findEdge(minAngle + 1,  1)
  # print("centerAng:", minAngle, "left:", leftPoint.z, "right:", rightPoint.z)
  # print("dist", ((leftPoint.x - rightPoint.x) ** 2 + (leftPoint.y - rightPoint.y) **2) ** 0.5)
#
def laserScan(data):
  global scanData
  scanData = data
  # scanNumber = len(scanData.ranges)
  if (scanFlag == True):
    calcPoint()
  # else:
  #   print   "0:", scanDistance(  0), \
  #          "90:", scanDistance( 90), \
  #         "180:", scanDistance(180), \
  #         "270:", scanDistance(-90)

#
def btrScanStart(self):
  global scanFlag
  scanFlag = True
  print("start publishing")
  return EmptyResponse()
#
def btrScanStop(self):
  global scanFlag
  scanFlag = False
  print("stop publishing")
  return EmptyResponse()

# main
#
if __name__ == '__main__':
  args = sys.argv
  topicName = ""
  nodeName = "btr_scan"
  if (len(args) >= 2):
    if ( args[1] == "gazebo" or args[1] == "-g" or args[1] == "--gazebo"):
      topicName = "/robotino" + str(args[2]) 
      nodeName = "robotino_scan" + str(args[2])
  scanFlag = False
  centerPoint = Point()
  closePoint = Point()
  leftPoint = Point()
  rightPoint = Point()

  rospy.init_node(nodeName)
  sub01 = rospy.Subscriber(topicName + "/scan", LaserScan, laserScan)
  srv01 = rospy.Service(topicName + "/btr/scan_start", Empty, btrScanStart)
  srv02 = rospy.Service(topicName + "/btr/scan_stop", Empty, btrScanStop)
  pub00 = rospy.Publisher(topicName + "/btr/centerPoint", Point, queue_size = 10)
  pub01 = rospy.Publisher(topicName + "/btr/closePoint", Point, queue_size = 10)
  pub02 = rospy.Publisher(topicName + "/btr/leftPoint", Point, queue_size = 10)
  pub03 = rospy.Publisher(topicName + "/btr/rightPoint", Point, queue_size = 10)
  rate = rospy.Rate(10)

  # rospy.spin()

  scanData = LaserScan
  
  while not rospy.is_shutdown():
    if (scanFlag == True):
      pub00.publish(centerPoint)
      pub01.publish(closePoint)
      pub02.publish(leftPoint)
      pub03.publish(rightPoint)
    rate.sleep()

