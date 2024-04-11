#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
import cv2
import sys
sys.path.append('/home/iclab/Desktop/kid_hurocup/src/strategy')
from Python_API import Sendmessage
import time
import random

aaaa = rospy.init_node('WLstrategy', anonymous=True, log_level=rospy.DEBUG)

HEAD_MOTOR_START = 1433    # 初始位置1456
HEAD_MOTOR_FINISH = 1350    # 舉起前低頭 1263
# STAND_FIX = True

WIGHT= 8

if WIGHT==8:
    PICK_ONE = 801
    PICK_TWO = 802
    PICK_THREE = 803
    LIFT = 804

elif WIGHT==9:
    PICK_ONE = 901
    PICK_TWO = 902
    PICK_THREE = 903
    LIFT = 904
    
else:
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
        self.real_bar_center = 160
        self.speed = 0

    def walk_switch(self):
        time.sleep(0.5)
        send.sendBodyAuto(500, 0, 0, 0, 1, 0)
        if self.body_auto:
            self.body_auto = False
        else:
            self.body_auto = True

    def imu_fix(self):
        theta = 0
        if send.imu_value_Yaw > 2:
            theta = -2
        elif send.imu_value_Yaw < -2:
            theta = 2
        return theta

    def walk_parameter(self, yaw, Y_COM):
        send.sendSensorReset(1, 1, yaw)
        if WIGHT==8:
            send.sendWalkParameter('save'   , walk_mode = 1
                                            , com_y_shift = Y_COM
                                            , y_swing = 4.5
                                            , period_t = 360
                                            , t_dsp = 0.1
                                            , base_default_z = 1.5
                                            , right_z_shift = 0
                                            , base_lift_z = 3
                                            , com_height = 29.5
                                            , stand_height = 23.5
                                            , back_flag = False)
        else:
            send.sendWalkParameter('save'   , walk_mode = 1
                                            , com_y_shift = Y_COM
                                            , y_swing = 4.5
                                            , period_t = 360
                                            , t_dsp = 0.1
                                            , base_default_z = 2
                                            , right_z_shift = 0
                                            , base_lift_z = 3
                                            , com_height = 29.5
                                            , stand_height = 23.5
                                            , back_flag = False)    
    def walking(self, yaw, Y_COM):
        if not self.body_auto:
            self.walk_parameter(yaw, Y_COM)
            self.walk_switch()
        self.theta = self.imu_fix()
        rospy.loginfo(f'body_auto ========= {self.body_auto}')
        rospy.loginfo(f'theta ========= {self.theta}')
        rospy.loginfo(f'imu ========= {send.imu_value_Yaw}')
        if self.ctrl_status == 'final':
            if self.speed < 1800:
                self.speed += 200
            send.sendContinuousValue(self.speed , 200, 0, self.theta+1, 0)
            
        else:
            send.sendContinuousValue(1900, 200, 0, self.theta+1, 0) #90 1900

    def main(self):
        if send.is_start:#啟動電源與擺頭
            rospy.loginfo(f'ctrl_status : {self.ctrl_status}') #information
            # if self.ctrl_status == 'First':
            #     send.sendBodySector(29)             #基礎站姿磁區
                # while not send.execute:
                #     rospy.logdebug("站立姿勢")
                # send.execute = False
                # rospy.sleep(1) 
                # if STAND_FIX:
                #     send.sendBodySector(290)             #LC基礎站姿調整磁區
                #     while not send.execute:
                #         rospy.logdebug("站立姿勢調整")
                #     send.execute = False
                # rospy.sleep(1) 
                # self.ctrl_status = 'head_shake'
            # rospy.loginfo(self.bar.center.y )
            # rospy.loginfo(self.line.edge_min.y )
            # rospy.loginfo(self.line.edge_max.y )
            if self.ctrl_status == 'head_shake':
                send.sendBodySector(299)
                time.sleep(1)
                send.sendSensorReset(1,1,1)
                time.sleep(0.1)
                # send.sendBodySector(5559)  
                # time.sleep(1) 
                send.sendHeadMotor(2, HEAD_MOTOR_START, 100)
                self.stop = False
                time.sleep(0.5)
                # send.sendBodySector(99)
                # print("99")
                # time.sleep(1)
                self.ctrl_status = 'preturn'
            self.bar.update(1)
            self.line.update(2)
            if self.ctrl_status == 'preturn':
                rospy.loginfo(f'ctrl_status : {self.ctrl_status}')
                print(send.DIOValue)
                if not self.body_auto:
                    self.walk_parameter(1, 0)
                    self.walk_switch()
                    if send.DIOValue == 49:
                        send.sendHeadMotor(2, 1500, 100)
                        self.bar.update(1)
                        self.line.update(2)
                        while self.bar.center.x <= 163 or self.bar.center.x > 260:
                            self.bar.update(1)
                            self.line.update(2)
                            send.sendContinuousValue(1000, 1200, 0, 1, 0)
                            # if self.bar.center.x <=160:
                            #     send.sendContinuousValue(1000, 1200, 0, 1, 0)
                            # elif self.bar.center.x <= 163 or self.bar.center.x > 160:
                            #     send.sendContinuousValue(1000, 1200, 0, 0, 0)
                            rospy.loginfo(f"紅色preturn = {self.bar.center.x}")
                    if send.DIOValue == 51:
                        send.sendHeadMotor(2, 1500, 100)
                        self.bar.update(1)
                        self.line.update(2)
                        send.sendContinuousValue(1000, -1000, 0, 0, 0)
                        while self.bar.center.x >= 160 or self.bar.center.x <= 30 :
                            self.bar.update(1)
                            self.line.update(2)
                            send.sendContinuousValue(1000, -1100, 0, 0, 0)
                            rospy.loginfo(f"紅色preturn = {self.bar.center.x}")
                            if self.bar.center.x <= 168:
                                send.sendContinuousValue(1000, -1000, 0, -1, 0)

                self.ctrl_status = 'start_line'
                time.sleep(0.5)
                #self.ctrl_status = 'read'
            #if self.ctrl_status == 'read':
    
                #rospy.loginfo(f"aaaa = {self.line.edge_min.y}")
                #rospy.loginfo(f"bbbb = {self.line.edge_max.y}")
            send.sendHeadMotor(2, HEAD_MOTOR_START, 100)
            if self.ctrl_status == 'start_line':
                if self.bar.center.x > 180:
                    send.sendContinuousValue(1000, -400, 0, -1, 0)
                    rospy.loginfo(f"右轉")
                elif self.bar.center.x < 140 and self.bar.center.x > 0:
                    send.sendContinuousValue(1000, 400, 0, 1, 0)  
                    rospy.loginfo(f"左轉")  
                #change
                else:
                    self.walking(1, -1)
                rospy.loginfo(f"紅色 = {self.bar.center.x}")
                if self.bar.center.y >= 224 :
                    self.ctrl_status = 'turn_straight'
            # elif self.ctrl_status == 'turn_straight':
            #     self.theta = self.imu_fix()
            #     send.sendContinuousValue(0, 0, 0, self.theta, 0)
            #     if self.theta == 0:
            #         self.ctrl_status = 'pick_up'
            elif self.ctrl_status == 'turn_straight':
                self.theta = self.imu_fix()
                send.sendContinuousValue(0, 0, 0, self.theta, 0)
                if self.theta == 0:
                    self.ctrl_status = 'pick_up'
            # elif self.ctrl_status == 'smooth':
            #     if self.bar.center.x>=165:
            #         send.sendContinuousValue(0, -500, 0, 0, 0)
            #         rospy.loginfo(f"紅色 = {self.bar.center.x}")
            #     elif self.bar.center.x<=155:
            #         send.sendContinuousValue(0, 500, 0, 0, 0)
            #         rospy.loginfo(f"紅色 = {self.bar.center.x}")  
            #     if self.theta == 0:
            #         self.ctrl_status = 'pick_up'
            elif self.ctrl_status == 'pick_up':
                if self.body_auto:
                    self.walk_switch()
                time.sleep(2.5)
                send.sendHeadMotor(2, 1320, 100)
                send.sendBodySector(PICK_ONE) 
                print("PICK_ONE")
                time.sleep(6.5)
                send.sendBodySector(PICK_TWO)
                print("PICK_TWO")
                time.sleep(5)
                send.sendBodySector(PICK_THREE)
                print("PICK_3")
                time.sleep(5.5)  
                self.bar.update(1)
                send.sendHeadMotor(2, HEAD_MOTOR_START, 100)
                time.sleep(1)
                self.real_bar_center = self.bar.center.x 
                self.ctrl_status = 'second_line'
            elif self.ctrl_status == 'second_line':
                self.walking(0, -3)
                if self.line.edge_min.y < 90 and self.line.edge_min.y > 70:
                    self.third_line = True 
                print(self.third_line)
                rospy.loginfo(f"white_Y = {self.line.edge_max.y}")
                send.sendHeadMotor(2,1400, 100)
                if self.line.edge_max.y >= 230 and self.third_line :
                    self.ctrl_status = 'rise_up'
                    time.sleep(3.4)
            elif self.ctrl_status == 'rise_up':
                if self.body_auto:
                    self.walk_switch()
                time.sleep(2)
                send.sendBodySector(LIFT)
                print("LIFT")
                if WIGHT==9:
                    time.sleep(10)#90
                else:
                    time.sleep(18)#80
                print("x =============================== ",self.real_bar_center)
                if self.real_bar_center > 175 and self.real_bar_center < 210:
                    count = (self.real_bar_center - 165) // 7
                    count = min(count, 4)
                    print("count",count)
                    for i in range(count):
                        send.sendBodySector(605)
                    time.sleep(3.5) 
                elif self.real_bar_center < 155 and self.real_bar_center > 120:
                    count = (165 - self.real_bar_center) // 7
                    count = min(count, 4)
                    print("count",count)
                    for i in range(count):
                        print('a')
                        send.sendBodySector(606)       
                    time.sleep(3.5) 
                send.sendBodySector(3333)
                time.sleep(1)
                # send.sendBodySector(4444)1218-+
                # send.sendBodySector(5557)  
                # time.sleep(1) 
                #second open
                self.ctrl_status = 'final'
            elif self.ctrl_status == 'final':
                self.walking(0, -3)

        # elif not send.is_start:
        else:
            self.line.update(1)
            if self.body_auto:
                self.walk_switch()
            if not self.stop:
                send.sendHeadMotor(2, HEAD_MOTOR_FINISH, 100)
                self.init()
                rospy.loginfo(f'stop')
            # if self.ctrl_status != 'First':
            #     self.ctrl_status = 'First'


            
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
        if send.color_mask_subject_size[self.color] != []:
            max_object_size = max(send.color_mask_subject_size[self.color])
            max_object_idx = send.color_mask_subject_size[self.color].index(max_object_size)

            return max_object_idx if max_object_size > 3000 else None

    def get_bar_object(self):
        if send.color_mask_subject_size[self.color] != []:
            max_object_size = max(send.color_mask_subject_size[self.color])
            max_object_idx = send.color_mask_subject_size[self.color].index(max_object_size)

            return max_object_idx if max_object_size > 50 else None

    def update(self,ID):
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

            # rospy.loginfo(self.edge_max.y)
            # rospy.logdebug(abs(abs(self.edge_max.x - self.edge_min.x) - abs(self.edge_max.y - self.edge_min.y)))
            send.drawImageFunction( ID, 1, self.edge_min.x, self.edge_max.x, self.edge_min.y, self.edge_max.y, 0, 0, 255)
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
