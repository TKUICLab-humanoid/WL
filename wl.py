#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
import cv2
# # import sys
# sys.path.append('/home/iclab/Desktop/adult_hurocup/src/strategy')
from Python_API import Sendmessage
import time

HEAD_MOTOR_START    = 1233    # 初始位置1433
HEAD_MOTOR_FINISH   = 1330
HEAD_MOTOR_LIFT     = 1729

#中心點
RED_X = 157
RED_Y = 183

#原地步態調整
SPEED_X = 500
SPEED_Y = -200
SPEED_Z = 0

#磁區動作串
START = 123
PICK_ONE = 401
PICK_TWO = 402
PICK_THREE = 412
LIFT = 403          #4032:30片1218值38/40片1218值52

send = Sendmessage()
#send.use_new_color_mask = True
aaaa = rospy.init_node('talker', anonymous=True)

class WeightLift:
    def __init__(self):
        self.line = ObjectInfo('White', 'rise_line')
        self.bar = ObjectInfo('Red', 'weight_bar')
        self.strategy_start = False
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

    def walk_switch(self):      #啟動停止步態
        send.sendBodyAuto(600, 0, 0, 0, 1, 0)
        if self.body_auto:
            self.body_auto = False
        else:
            self.body_auto = True
    
    def forward_fix(self):      #前後步態微調
        if self.ctrl_status == 'start_line':
            forward = SPEED_X + 3500
            self.forward_status = '大前進'
        elif self.ctrl_status == 'first_line':
            forward = SPEED_X - 1000
            self.forward_status = '大後退'
            
        if self.bar.edge_max.y != 0:
            if self.bar.edge_max.y < (RED_Y - 100):
                forward = SPEED_X + 3500
                self.forward_status = '大前進'
            elif ((RED_Y - 100) <= self.bar.edge_max.y) and (self.bar.edge_max.y < (RED_Y - 20)):
                forward = SPEED_X + 2500
                self.forward_status = '前進'
            elif ((RED_Y - 20) <= self.bar.edge_max.y) and (self.bar.edge_max.y < RED_Y):
                forward = SPEED_X + 200
                self.forward_status = '小前進'
            elif RED_Y <= self.bar.edge_max.y:
                forward = SPEED_X - 1500
                self.forward_status = '後退'
        return forward

    def translate_fix(self):    #左右步態微調
        translate = SPEED_Y
        self.translate_status = '無'
        middle = (self.bar.edge_max.x + self.bar.edge_min.x)/2
        if middle != 0:
            if middle > (RED_X + 20):
                translate = SPEED_Y - 2500
                self.translate_status = '大 - 右平移'
            elif ((RED_X + 5) < middle) and (middle <= (RED_X + 20)):
                translate = SPEED_Y - 1500
                self.translate_status = '中 - 右平移'
            elif (RED_X < middle) and (middle <= (RED_X + 5)):
                translate = SPEED_Y - 1200
                self.translate_status = '小 - 右平移'
            elif middle <= (RED_X - 20):
                translate = SPEED_Y + 2000
                self.translate_status = '大 - 左平移'
            elif ((RED_X - 20) < middle) and (middle <= (RED_X - 5)):
                translate = SPEED_Y + 1200
                self.translate_status = '中 - 左平移'
            elif ((RED_X - 5) < middle) and (middle <= RED_X):
                translate = SPEED_Y + 900
                self.translate_status = '小 - 左平移'
        return translate
    
    def imu_fix(self):  #旋轉量修正
        theta = SPEED_Z
        self.theta_status = '無'
        if self.ctrl_status == 'final':
            if send.imu_value_Yaw > 4:
                theta = SPEED_Z - 1
                self.theta_status = '右旋'
            elif send.imu_value_Yaw < -4:
                theta = SPEED_Z + 1
                self.theta_status = '左旋'
        else:
            if send.imu_value_Yaw > 4:
                theta = SPEED_Z - 2
                self.theta_status = '右旋'
            elif send.imu_value_Yaw < -4:
                theta = SPEED_Z + 2
                self.theta_status = '左旋'
        return theta

    def walking(self,yaw):  #是否更新yaw值&步態
        if not self.body_auto:
            send.sendSensorReset(1, 1, yaw)
            end = -9999
            start = time.time()
            while(end - start < 1):
                end = time.time() 
                send.sendBodySector(9999)
                time.sleep(0.05)
                if self.ctrl_status == "final":
                    #back_flag要一個True一個False
                    send.sendWalkParameter('save', walk_mode = 1, com_y_shift = 0, y_swing = 8, com_height = 62, period_t = 420, \
                                            t_dsp = 0.3, base_default_z = 3.5, right_z_shift = 0, stand_height = 47.3, base_lift_z = 0, back_flag = True)
                    time.sleep(1)
                    send.sendWalkParameter('save', walk_mode = 1, com_y_shift = 0, y_swing = 8, com_height = 62, period_t = 420, \
                                            t_dsp = 0.3, base_default_z = 3.5, right_z_shift = 0, stand_height = 47.3, base_lift_z = 0, back_flag = False)
                    time.sleep(1)
                # else:
                    # send.sendWalkParameter('save', walk_mode = 1, com_y_shift = 0, y_swing = 8, com_height = 62, period_t = 450, \
                    #                        t_dsp = 0.1, base_default_z = 3.5, right_z_shift = 0, stand_height = 47.3, base_lift_z = 0, back_flag = True)
                    # time.sleep(2)
                    # send.sendWalkParameter('save', walk_mode = 1, com_y_shift = 0, y_swing = 8, com_height = 62, period_t = 450, \
                    #                        t_dsp = 0.1, base_default_z = 3.5, right_z_shift = 0, stand_height = 47.3, base_lift_z = 0, back_flag = False)
                    # time.sleep(2)
            time.sleep(0.03)
            self.walk_switch()
        
        if (self.ctrl_status == 'start_line') or (self.ctrl_status == 'first_line'):
            self.final_forward = SPEED_X
            self.forward = self.forward_fix()
            self.translate = self.translate_fix()
        elif self.ctrl_status == 'second_line' and self.reset:
            self.final_forward = SPEED_X + 4100
            self.forward = SPEED_X
            self.translate = SPEED_Y
            self.forward_status = '前進'
            self.translate_status = '無'
            self.reset = False
        elif self.ctrl_status == 'final' and self.reset:
            self.final_forward = SPEED_X + 3000
            self.forward = SPEED_X
            self.translate = SPEED_Y
            self.reset = False

        self.theta = self.imu_fix()
        if self.forward <= self.final_forward:
            self.forward += 100
        rospy.loginfo(f'forward : {self.forward}, translate : {self.translate}, theta : {self.theta}')
        rospy.loginfo(f'狀態 : {self.forward_status}, {self.translate_status}, {self.theta_status}')
        send.sendContinuousValue(self.forward, self.translate, 0, self.theta, 0)

    def main(self):
        if send.is_start:
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
                # send.sendBodySector(29)
                # time.sleep(0.5)
                send.sendBodySector(PICK_ONE)
                time.sleep(5.3)
                send.sendBodySector(PICK_TWO)
                time.sleep(8.5)
                send.sendBodySector(PICK_THREE)
                time.sleep(8.5)
                self.ctrl_status = 'second_line'

            elif self.ctrl_status == 'second_line':
                self.walking(0)
                rospy.loginfo(f'while : {self.line.edge_max.y}')
                if not self.lift_get:
                    if self.line.edge_min.y >= 70 and self.line.edge_min.y <= 80:
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
                time.sleep(7)
                self.ctrl_status = 'final'
                self.reset = True

            elif self.ctrl_status == 'final':
                self.walking(0)

        else:
            if self.body_auto:
                self.walk_switch()
            
            # send.sendWalkParameter('save', walk_mode = 1, com_y_shift = 0, y_swing = 8, com_height = 62, period_t = 450, \
            #                         t_dsp = 0.1, base_default_z = 3.5, right_z_shift = 0, stand_height = 47.3, base_lift_z = 0, back_flag = True)
            # time.sleep(2)
            # send.sendWalkParameter('save', walk_mode = 1, com_y_shift = 0, y_swing = 8, com_height = 62, period_t = 450, \
            #                         t_dsp = 0.1, base_default_z = 3.5, right_z_shift = 0, stand_height = 47.3, base_lift_z = 0, back_flag = False)
            # time.sleep(2)
            send.sendHeadMotor(2, HEAD_MOTOR_START, 100)
            self.init()
            time.sleep(1)
            if not self.strategy_start:
                send.sendBodySector(START)
                time.sleep(3.5)
                print('start')
                self.strategy_start = True

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