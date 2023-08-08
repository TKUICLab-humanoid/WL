#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
import cv2
# # import sys
# sys.path.append('/home/iclab/Desktop/adult_hurocup/src/strategy')
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
HEAD_MOTOR_START    = 1233    # 初始位置1433
HEAD_MOTOR_FINISH   = 1330
HEAD_MOTOR_LIFT     = 1729

RED_X = 146
RED_Y = 163

'''
磁區
40片: 40 41 42
50片: 40 41 52(週期:420、雙支撐:0.3)
60片: 40 41 62(頭高:1729, SPEED = 3300, Period_T = 450, 雙支撐:0.3, BASE_Default_z = 3.5)
70片: 40 71 72(頭高:1729, Period_T = 450, 雙支撐:0.3, BASE_Default_z = 3.5)
'''

PICK_ONE = 40
PICK_TWO = 41
LIFT = 62

send = Sendmessage()
#send.use_new_color_mask = True
aaaa = rospy.init_node('talker', anonymous=True)

class WeightLift:
    def __init__(self):
        self.line = ObjectInfo('White', 'rise_line')
        self.bar = ObjectInfo('Red', 'weight_bar')
        self.init()

    def init(self):
        self.forward    = 0
        self.translate  = 0
        self.theta      = 0
        self.red_middle = 0
        self.number     = 0

        self.ctrl_status        = 'head_shake'
        self.forward_status     = '直走'
        self.translate_status   = '無'
        self.theta_status       = '無'
        
        self.body_auto  = False
        send.first_check = True
        self.reset      = True
        self.lift_get   = False

    def walk_switch(self):
        send.sendBodyAuto(600, 0, 0, 0, 1, 0)
        if self.body_auto:
            self.body_auto = False
        else:
            self.body_auto = True
    
    def forward_fix(self):
        if self.ctrl_status == 'start_line':
            forward = 3000
            self.forward_status = '大前進'
        elif self.ctrl_status == 'first_line':
            forward = -1500
            self.forward_status = '大後退'
            
        if self.bar.edge_max.y != 0:
            if self.bar.edge_max.y < (RED_Y - 100):
                forward = 3500
                self.forward_status = '大前進'
            elif ((RED_Y - 100) <= self.bar.edge_max.y) and (self.bar.edge_max.y < (RED_Y - 20)):
                forward = 2500
                self.forward_status = '前進'
            elif ((RED_Y - 20) <= self.bar.edge_max.y) and (self.bar.edge_max.y < RED_Y):
                forward = 200
                self.forward_status = '小前進'
            elif RED_Y <= self.bar.edge_max.y:
                forward = -1500
                self.forward_status = '後退'
        return forward

    def translate_fix(self):
        translate = 0
        self.translate_status = '無'
        middle = (self.bar.edge_max.x + self.bar.edge_min.x)/2
        if middle != 0:
            if middle > (RED_X + 20):
                translate = -2800 + 100
                self.translate_status = '大 - 右平移'
            elif ((RED_X + 5) < middle) and (middle <= (RED_X + 20)):
                translate = -1500 + 100
                self.translate_status = '中 - 右平移'
            elif (RED_X < middle) and (middle <= (RED_X + 5)):
                translate = -1200 + 100
                self.translate_status = '小 - 右平移'
            elif ((RED_X - 5) < middle) and (middle <= RED_X):
                translate = 900 - 100
                self.translate_status = '小 - 左平移'
            elif ((RED_X - 20) < middle) and (middle <= (RED_X - 5)):
                translate = 1200 - 100
                self.translate_status = '中 - 左平移'
            elif middle <= (RED_X - 20):
                translate = 2500 - 100
                self.translate_status = '大 - 左平移'
        return translate
    
    def imu_fix(self):
        theta = 0
        self.theta_status = '無'
        if send.imu_value_Yaw > 4:
            theta = -3
            self.theta_status = '右旋'
        elif send.imu_value_Yaw < -4:
            theta = 3
            self.theta_status = '左旋'
        return theta

    def walking(self,yaw):
        if not self.body_auto:
            send.sendSensorReset(1, 1, yaw)
            end = -9999
            start = time.time()
            while(end - start < 1):
                end = time.time() 
                send.sendBodySector(9999)
                time.sleep(0.05)
                if self.ctrl_status == "final":
                    #send.sendWalkParameter('save', walk_mode = 1, y_swing = 8, t_dsp = 0.2)
                    send.sendWalkParameter(0, 1, -1, 8, 50, 450, 0.3, 3.5, 0, 47.3, 0, False)
                    time.sleep(0.5)
                    send.sendWalkParameter(1, 1, -1, 8, 50, 450, 0.3, 3.5, 0, 47.3, 0, False)
                    time.sleep(0.5)
                else:
                    #send.sendWalkParameter('save', walk_mode = 1, y_swing = 8, t_dsp = 0)
                    send.sendWalkParameter(0, 1, 0, 8, 50, 420, 0, 4, 0, 47.3, 0, False)
                    time.sleep(0.5)
                    send.sendWalkParameter(0, 1, 0, 8, 50, 420, 0, 4, 0, 47.3, 0, True)
                    time.sleep(0.5)
            time.sleep(0.03)
            self.walk_switch()
        
        if (self.ctrl_status == 'start_line') or (self.ctrl_status == 'first_line'):
            self.final_forward = 0
            self.forward = self.forward_fix()
            self.translate = self.translate_fix()
        elif self.ctrl_status == 'second_line' and self.reset:
            self.final_forward = 4000
            self.forward = 0
            self.translate = 0
            self.forward_status = '前進'
            self.translate_status = '無'
            self.reset = False
        elif self.ctrl_status == 'final' and self.reset:
            self.final_forward = 3000
            self.forward = 0
            self.translate = 0
            self.reset = False

        self.theta = self.imu_fix()
        if self.forward < self.final_forward:
            self.forward += 500
        rospy.loginfo(f'forward : {self.forward}, translate : {self.translate}, theta : {self.theta}')
        rospy.loginfo(f'狀態 : {self.forward_status}, {self.translate_status}, {self.theta_status}')
        send.sendContinuousValue(self.forward, self.translate, 0, self.theta, 0)

    def main(self):
        if send.is_start:#啟動電源與擺頭
            # send.data_check = False
            rospy.loginfo(f'ctrl_status : {self.ctrl_status}')
            if self.ctrl_status == 'head_shake':
                send.sendHeadMotor(2, HEAD_MOTOR_FINISH, 100)
                time.sleep(0.03)
                self.ctrl_status = 'start_line'

            if send.data_check:
                self.line.update()
                self.bar.update()
                send.data_check = False
                send.first_check = False

            if self.ctrl_status == 'start_line':
                send.sendHeadMotor(2, HEAD_MOTOR_FINISH, 100)
                self.walking(1)
                if self.bar.edge_max.y > (RED_Y - 100):
                    self.ctrl_status = 'first_line'
            
            elif self.ctrl_status == 'first_line':
                self.walking(1)
                rospy.loginfo(f'self.red_middle = {self.bar.center.x}')
                if (self.bar.edge_max.y >= RED_Y) and (self.bar.edge_max.y <= RED_Y+5)\
                    and (self.bar.center.x >= RED_X-5) and (self.bar.center.x <= RED_X + 5)\
                    and (send.imu_value_Yaw <= 8) and (send.imu_value_Yaw >= -8):
                        self.ctrl_status = 'pick_up'

            elif self.ctrl_status == 'pick_up':
                if self.body_auto:
                    self.walk_switch()
                time.sleep(2)
                send.sendBodySector(29)
                time.sleep(0.5)
                send.sendBodySector(PICK_ONE)
                time.sleep(6.8)
                # self.number = int(abs(self.bar.center.x - RED_X) / 3)
                # if self.number > 3:
                #     self.number = 3
                # left = 0
                # right = 0
                # if self.bar.center.x > RED_X:
                #     while(self.number != 0):
                #         send.sendBodySector(222)
                #         self.number -= 1
                #         left += 1
                #         time.sleep(0.1)
                # elif self.bar.center.x < RED_X :
                #     while(self.number != 0):
                #         send.sendBodySector(666)
                #         self.number -= 1
                #         right += 1
                #         time.sleep(0.1)
                send.sendBodySector(PICK_TWO)
                time.sleep(14.5)
                # while(right != 0):
                #     send.sendBodySector(222)
                #     right -= 1
                #     time.sleep(0.1)
                # while(left != 0):
                #     send.sendBodySector(666)
                #     left -= 1
                #     time.sleep(0.1)
                    
                # send.sendBodySector(200)
                # time.sleep(1)
                
                self.ctrl_status = 'second_line'

            elif self.ctrl_status == 'second_line':
                self.walking(0)
                rospy.loginfo(self.lift_get)
                rospy.loginfo(f'while : {self.line.edge_max.y}')
                if not self.lift_get:
                    if self.line.edge_min.y >= 60 and self.line.edge_min.y <= 70:
                        send.sendHeadMotor(2, 1300, 100)
                        self.lift_get = True
                        
                else:
                    if self.line.edge_max.y >= 200:
                        #print('while : ', self.line.edge_max.y)
                        self.ctrl_status = 'rise_up'

            elif self.ctrl_status == 'rise_up':
                end = -9999
                start = time.time()
                while (end - start < 1):
                    end = time.time()
                    rospy.loginfo(f'delay : {end - start}')
                if self.body_auto:
                    self.walk_switch()
                time.sleep(3)
                send.sendHeadMotor(2, HEAD_MOTOR_LIFT, 100)
                time.sleep(0.03)
                send.sendBodySector(LIFT)
                time.sleep(8.8)
                # send.sendBodySector(300)
                # time.sleep(1)
                self.ctrl_status = 'final'
                self.reset = True

            elif self.ctrl_status == 'final':
                self.walking(0)

        else:
            if self.body_auto:
                self.walk_switch()
            
            # if self.reset:
            #     send.sendBodySector(29)
            #     send.sendBodySector(999)
            #     self.reset = False
            #if send.data_check == True:
            # self.line.update()
            # if send.data_check:
            #     self.line.update()
            #self.bar.update()
            #     send.data_check = False
            # print(int(abs(self.bar.center.x - RED_X)))
                #send.data_check = False
            # rospy.loginfo(f'red_middle_x : {(self.bar.edge_max.x + self.bar.edge_min.x) / 2}')
            # rospy.loginfo(f'red_line_y :   {self.bar.edge_max.y}')
            #rospy.loginfo(f'while_line :   {self.line.edge_max.y}')
            #rospy.loginfo(f"while size :   {self.line.target_size}")
            #rospy.loginfo(f"red size :     {self.bar.target_size}")
            
            send.sendHeadMotor(2, HEAD_MOTOR_START, 100)
            self.init()
            for i in range(0,1,1):
                print('start')

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
            return max_object_idx if max_object_size > 1000 else None

    def get_bar_object(self):
        if send.color_mask_subject_size[self.color] != []:
            max_object_size = max(send.color_mask_subject_size[self.color])
            max_object_idx = send.color_mask_subject_size[self.color].index(max_object_size)

            return max_object_idx if max_object_size > 50 else None

    def update(self):
        object_idx = self.find_object()

        if object_idx is not None:
            self.get_target = True
            self.edge_max.x = np.array(send.color_mask_subject_XMax[self.color])[object_idx]
            self.edge_min.x = np.array(send.color_mask_subject_XMin[self.color])[object_idx]
            self.edge_max.y = np.array(send.color_mask_subject_YMax[self.color])[object_idx]
            self.edge_min.y = np.array(send.color_mask_subject_YMin[self.color])[object_idx]
            self.center.x = np.array(send.color_mask_subject_X[self.color])[object_idx]
            self.center.y = np.array(send.color_mask_subject_Y[self.color])[object_idx]
            self.target_size = np.array(send.color_mask_subject_size[self.color])[object_idx]

            rospy.logdebug(self.target_size)
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