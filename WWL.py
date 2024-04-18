#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
import cv2
from Python_API import Sendmessage
import time


# def ball_cv2():
#     ball_x , ball_y , ball_r= 0 ,0 ,0
#     img = send.rawimg.copy()
#     gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#     blur = cv2.GaussianBlur(gray,(5,5),0) 
#     # gray = cv2.bitwise_not(gray)
#     ret,thresh = cv2.threshold(blur,100,255,cv2.THRESH_BINARY) # 使用二值化閾值處理
#     circles = cv2.HoughCircles(thresh,cv2.HOUGH_GRADIENT,2,200,
#                             param1=33,param2=16,minRadius=3,maxRadius=15)        #thresh: 經過閾值處理的圖像
#                                                                                 # cv2.HOUGH_GRADIENT: Hough圓檢測的方法
#                                                                                 # 1: 圖像分辨率
#                                                                                 # 20: 圓之間的最小距離
#                                                                                 # param1: 檢測邊緣的閾值
#                                                                                 # param2: 檢測圓心的閾值
#                                                                                 # minRadius: 最小圓半徑
#                                                                                 # maxRadius: 最大圓半徑

#     if circles is not None:
#         circles = np.round(circles[0, :]).astype("int")
#         for (x, y, r) in circles:
#             ball_x = x
#             ball_y = y
#             ball_r = r

#     return ball_x,ball_y,ball_r
HEAD_MOTOR_START = 1433    # 初始位置1456
HEAD_MOTOR_FINISH = 1350    # 舉起前低頭 1263

PICK_ONE = 801
PICK_TWO = 802
PICK_THREE = 803
LIFT = 804

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
            send.saveWalkParameter(1, -1, 4.5, 360, 0.1, 2, 0, 0, False)
            time.sleep(0.03)
            self.walk_switch()
        self.theta = self.imu_fix()
        rospy.logdebug(f'theta : {self.theta}')
        send.sendContinuousValue(2000, 0, 0, self.theta, 0)

    def main(self):
        if send.Web:#啟動電源與擺頭
            rospy.loginfo(f'ctrl_status : {self.ctrl_status}')
            if self.ctrl_status == 'head_shake':
                send.sendHeadMotor(2, HEAD_MOTOR_START, 100)
                time.sleep(0.03)
                self.ctrl_status = 'start_line'
            self.bar.update()
            self.line.update()
            if self.ctrl_status == 'start_line':
                self.walking(1)
                if self.bar.edge_max.y >= 167:
                    self.ctrl_status = 'pick_up'
            elif self.ctrl_status == 'pick_up':
                if self.body_auto:
                    self.walk_switch()
                # send.sendBodySector(PICK_ONE)
                print("PICK_ONE")
                time.sleep(6)
                # send.sendBodySector(PICK_TWO)
                print("PICK_TWO")
                time.sleep(5.5)
                # send.sendBodySector(PICK_THREE)
                print("PICK_3")
                time.sleep(6)  
                self.ctrl_status = 'second_line'
            elif self.ctrl_status == 'second_line':
                self.walking(0)
                if self.line.edge_min.y < 80 and self.line.edge_min.y > 60:
                    self.third_line = True
                if self.line.edge_max.y >= 167 and self.third_line :
                    self.ctrl_status = 'rise_up'
            elif self.ctrl_status == 'rise_up':
                if self.body_auto:
                    self.walk_switch()
                # send.sendBodySector(LIFT)
                print("LIFT")
                time.sleep(19)
                self.ctrl_status = 'final'
            elif self.ctrl_status == 'final':
                self.walking(0)

        if not send.Web:
            if self.body_auto:
                send.sendHeadMotor(2, HEAD_MOTOR_FINISH, 100)
                self.walk_switch()
                self.init()




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

            rospy.loginfo(self.target_size)
            rospy.logdebug(abs(abs(self.edge_max.x - self.edge_min.x) - abs(self.edge_max.y - self.edge_min.y)))
            send.drawImageFunction(1, 1, self.edge_min.x, self.edge_max.x, self.edge_min.y, self.edge_max.y, 0, 0, 255)
        else:
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