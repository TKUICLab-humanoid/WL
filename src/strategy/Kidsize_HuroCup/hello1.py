#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np

from tku_msgs.msg import Interface,HeadPackage,SandHandSpeed,DrawImage,SingleMotorData,\
SensorSet,ObjectList,LabelModelObjectList,RobotPos,SetGoalPoint,SoccerDataList,SensorPackage
from std_msgs.msg import Int16,Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Sendmessage:     
    def __init__(self):
        self.walkingGait_pub = rospy.Publisher('SendBodyAuto_Topic', Interface, queue_size=100)
        self.head_motor_pub = rospy.Publisher("/package/HeadMotor",HeadPackage, queue_size=10)
        self.sector_pub = rospy.Publisher("/package/Sector",Int16, queue_size=100)
        self.hand_speed_pub = rospy.Publisher("/package/motorspeed",SandHandSpeed, queue_size=100)
        self.draw_image_pub = rospy.Publisher("/package/drawimage",DrawImage, queue_size=100)


        rospy.init_node('hello', anonymous=True)


    def sendBodyAuto(self,x,y,z,theta,mode,sensor):	#步態啟動
        walkdata = Interface()
        walkdata.x = x
        walkdata.y = y
        walkdata.z = z
        walkdata.theta = theta
        walkdata.walking_mode = mode
        walkdata.sensor_mode = sensor
        self.walkingGait_pub.publish(walkdata)

    def sendHeadMotor(self,ID,Position,Speed):	#頭部馬達 HorizontalID = 1, VerticalID = 2
        HeadData = HeadPackage()
        HeadData.ID = ID
        HeadData.Position = Position
        HeadData.Speed = Speed
        self.head_motor_pub.publish(HeadData)



if __name__ == '__main__':
    try:
        print('a')
        send = Sendmessage()
        print('a')
        send.sendHeadMotor(2,2000,40)
    except rospy.ROSInterruptException:
        pass
