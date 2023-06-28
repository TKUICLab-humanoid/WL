#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
import cv2
from Python_API import Sendmessage
import time

aaaa = rospy.init_node('talker', anonymous=True)

HEAD_MOTOR_START = 1433    # 初始位置1456
HEAD_MOTOR_FINISH = 1350    # 舉起前低頭 1263

PICK_ONE = 601
PICK_TWO = 602
PICK_THREE = 603
LIFT = 604

send = Sendmessage()

class WeightLift:
    def __init__(self):
        self.line = ObjectInfo('White', 'rise_line')
        self.bar = ObjectInfo('Red', 'weight_bar')
        
        self.init()

    def init(self):
        self.theta = 0
        self.ctrl_status = 'head_shake'
        self.body_auto = False
        self.third_line = False
        self.stop = True

    def walk_switch(self):
        send.sendBodyAuto(600, 0, 0, 0, 1, 0)
        if self.body_auto:
            self.body_auto = False
        else:
            self.body_auto = True

    def imu_fix(self):
        theta = 0
        if send.imu_value_Yaw > 5:
            theta = -2
        elif send.imu_value_Yaw < -5:
            theta = 2
        return theta
    
    def walking(self,yaw):
        if not self.body_auto:
            send.sendSensorReset(1, 1, yaw)
            send.sendWalkParameter(0, 1,-1,4.5, 29.5, 360, 0, 2, 0, 23.5, 0, False)
            time.sleep(0.03)
            self.walk_switch()
        self.theta = self.imu_fix()
        rospy.loginfo(f'theta : {self.theta}')
        send.sendContinuousValue(2000, 0, 0, self.theta, 0)

    def main(self):
        if send.is_start:#啟動電源與擺頭
            rospy.loginfo(f'ctrl_status : {self.ctrl_status}')
            rospy.loginfo(self.bar.edge_max.y )
            rospy.loginfo(self.line.edge_max.y )
            if self.ctrl_status == 'head_shake':
                send.sendHeadMotor(2, HEAD_MOTOR_START, 100)
                self.stop = False
                time.sleep(0.3)
                self.ctrl_status = 'start_line'
            self.bar.update()
            self.line.update()
            if self.ctrl_status == 'start_line':
                if self.bar.center.x > 170:
                    send.sendContinuousValue(1000, -600, 0, -3, 0)
                elif self.bar.center.x < 150 and self.bar.center.x > 0:
                    send.sendContinuousValue(1000, 600, 0, 3, 0)    
                else:
                    self.walking(1)
                rospy.loginfo(f"x = {self.bar.center.x}")
                if self.bar.center.y >= 198:
                    self.ctrl_status = 'turn_straight'
            elif self.ctrl_status == 'turn_straight':
                self.theta = self.imu_fix()
                send.sendContinuousValue(0, 0, 0, self.theta, 0)
                if self.theta == 0:
                    self.ctrl_status = 'pick_up'
            elif self.ctrl_status == 'pick_up':
                if self.body_auto:
                    self.walk_switch()
                time.sleep(2)
                send.sendBodySector(PICK_ONE)
                print("PICK_ONE")
                time.sleep(6)
                send.sendBodySector(PICK_TWO)
                print("PICK_TWO")
                time.sleep(5.5)
                send.sendBodySector(PICK_THREE)
                print("PICK_3")
                time.sleep(11)  
                self.ctrl_status = 'second_line'
            elif self.ctrl_status == 'second_line':
                self.walking(0)
                if self.line.edge_min.y < 80 and self.line.edge_min.y > 60:
                    self.third_line = True
                print(self.third_line)
                if self.line.edge_max.y >= 220 and self.third_line :
                    self.ctrl_status = 'rise_up'
                    time.sleep(2.5)
            elif self.ctrl_status == 'rise_up':
                if self.body_auto:
                    self.walk_switch()
                time.sleep(2)
                send.sendBodySector(LIFT)
                print("LIFT")
                time.sleep(17)
                self.ctrl_status = 'final'
            elif self.ctrl_status == 'final':
                self.walking(0)

        if not send.is_start:
            if self.body_auto:
                self.walk_switch()
            if not self.stop:
                send.sendHeadMotor(2, HEAD_MOTOR_FINISH, 100)
                self.init()
                rospy.loginfo(f'stop')
                




class Coordinate:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class ObjectInfo:
    color_dict = {'Orange': 0,
                  'Yellow': 1,
                  'Blue':   2,
                  'Green':  3,
                  'Black':  4,
                  'Red':    5,
                  'White':  6 }

    def __init__(self, color, object_type):
        self.color = self.color_dict[color]
        self.edge_max = Coordinate(0, 0)
        self.edge_min = Coordinate(0, 0)
        self.center = Coordinate(0, 0)
        self.get_target = False
        self.target_size = 0

        update_strategy = { 'rise_line': self.get_line_object,
                            'weight_bar': self.get_bar_object}
        self.find_object = update_strategy[object_type]

    def get_line_object(self):
        max_object_size = max(send.color_mask_subject_size[self.color])
        max_object_idx = send.color_mask_subject_size[self.color].index(max_object_size)

        return max_object_idx if max_object_size > 3000 else None

    def get_bar_object(self):
        max_object_size = max(send.color_mask_subject_size[self.color])
        max_object_idx = send.color_mask_subject_size[self.color].index(max_object_size)

        return max_object_idx if max_object_size > 200 else None

    def update(self):
        object_idx = self.find_object()

        if object_idx is not None:
            self.get_target = True
            self.edge_max.x = send.color_mask_subject_XMax[self.color][object_idx]
            self.edge_min.x = send.color_mask_subject_XMin[self.color][object_idx]
            self.edge_max.y = send.color_mask_subject_YMax[self.color][object_idx]
            self.edge_min.y = send.color_mask_subject_YMin[self.color][object_idx]
            self.center.x = send.color_mask_subject_X[self.color][object_idx]
            self.center.y = send.color_mask_subject_Y[self.color][object_idx]
            self.target_size = send.color_mask_subject_size[self.color][object_idx]

            
            rospy.logdebug(abs(abs(self.edge_max.x - self.edge_min.x) - abs(self.edge_max.y - self.edge_min.y)))
            send.drawImageFunction(1, 1, self.edge_min.x, self.edge_max.x, self.edge_min.y, self.edge_max.y, 0, 0, 255)
        else:
            self.edge_max.x = 0
            self.edge_min.x = 0
            self.edge_max.y = 0
            self.edge_min.y = 0
            self.center.x = 0
            self.center.y = 0
            self.target_size = 0
            self.get_target = False


if __name__ == '__main__':

    try:
        strategy = WeightLift()
        r = rospy.Rate(20)

        while not rospy.is_shutdown():
            strategy.main()
            r.sleep()
    except rospy.ROSInterruptException:
        pass
