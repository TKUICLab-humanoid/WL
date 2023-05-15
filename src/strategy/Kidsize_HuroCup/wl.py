#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
from Python_API import Sendmessage
import time
# send.sendBodySector(3)
# send.sendBodySector(4)
aaaa = rospy.init_node('talker', anonymous=True, log_level=rospy.DEBUG)

imgdata = [[None for high in range(240)]for width in range(320)]
send = Sendmessage()

WHITE_SLOPE = 205

# 原地步態數值
X_ORIGIN = -150
Y_ORIGIN = 0
THETA_ORIGIN = 0

# 理想中間值，用於 "correct==true" 區域，與上方 xl, yl, ..., xr, yr, ...等等做搭配
RED_LEFT = 152
RED_RIGHT = 158

# 理想中間值，當機器人抓到槓鈴，此變數用於判斷是否執行磁區 31or32 進行微調
RED_MIDDLE_IDEAL = 155

# 停下/判斷距離設定，用於 拾起線 距離區間停下判斷
PICK_DIS_ONE = 162 
PICK_DIS_TWO = 167  # 停下數值改這個.126

# 判斷距離設定，用於 舉起線 距離區間停下判斷
LIFT_DIS_MIN = 60   # 此數值應小於 liftup_distance2 ; 看第3條線
LIFT_DIS_MAX = 120  # 這兩個數值進行第一階段判斷，判斷成功後(lift_line=True)進行第二階段 ; 看第3條線

LIFT_STOP_MIN = 15    # 距離在此區間便停下；此數值應小於 liftup_distance4 ; 看第4條線
LIFT_STOP_MAX = 75  # 看第4條線

# 頭部馬達角度設定
HEAD_MOTOR_STAND = 1433    # 初始位置1456
HEAD_MOTOR_PICK = 1337    # 最一開始移動後的位置
HEAD_MOTOR_LIFT = 1270    # 拾起槓鈴後的位置
HEAD_MOTOR_FINISH = 1275    # 舉起前低頭 1263

# 磁區
PICK_ONE = 601
PICK_TWO = 602
PICK_THREE = 603
LIFT = 604

class WeightLift():
    def __init__(self):
          self.pick_bar = False
          self.lift_bar = False
          self.body_auto = False
          self.arrive = False
          self.arrive_two = False
          self.imu_reset = False
          self.lift_line = False
          self.correct = False
          self.yaw = 0

          self.x_fix = 2000
          self.y_fix = 0
          self.theta_fix = 0

          self.red_xmax = 0
          self.red_xmin = 0
          self.red_ymax = 0
          self.red_ymin = 0
          self.red_size = 0
          
          self.white_xmax = 0
          self.white_xmin = 0
          self.white_ymax = 0
          self.white_ymin = 0
          self.white_size = 0
          # imu
          self.x = X_ORIGIN + self.x_fix
          self.y = Y_ORIGIN + self.y_fix
          self.theta = THETA_ORIGIN + self.theta_fix

          self.white_width_x = self.white_xmax - self.white_xmin
          self.white_width_y = self.white_ymax - self.white_ymin

    def horizontal(self):
        rospy.loginfo(f'開始微調')
        self.red_line_value()
        self.red_middle = float(self.red_xmax + self.red_xmin) / 2
        rospy.loginfo(f'red_middle = {self.red_middle}')
        if self.red_middle < RED_LEFT:
          send.sendContinuousValue(X_ORIGIN - 0, Y_ORIGIN + 1000, 0, THETA_ORIGIN + 1, 0)
          rospy.loginfo(f'左左左左左左左左左左左左左左左左左左左')
        elif self.red_middle > RED_RIGHT:
          send.sendContinuousValue(X_ORIGIN - 50, Y_ORIGIN - 1100, 0, THETA_ORIGIN - 1, 0)
          rospy.loginfo(f'右右右右右右右右右右右右右右右右右右右')
        else:
            self.arrive = True
            self.correct = False
            rospy.loginfo(f'微調結束')

    def imu(self):
      self.red_line()
      self.red_xmiddle = (self.red_xmax + self.red_xmin) / 2
      self.red_ymiddle = (self.red_ymax + self.red_ymin) / 2

      if self.imu_reset: #imu
        if self.red_xmiddle > 160:
          self.fix = -1
        elif self.red_xmiddle < 160:
          self.fix = 1
        else:
          self.fix = 0
        self.yaw = send.imu_value_Yaw
        rospy.loginfo(f'yaw = {self.yaw}')
        rospy.loginfo(f'red_xmiddle = {self.red_xmiddle}')

        if self.red_ymax > PICK_DIS_ONE and self.red_ymax < PICK_DIS_TWO:
          self.x_fix = 1500

        if self.yaw > 2:
          self.y_fix = -600
          self.theta_fix = -1 + self.fix
          self.theta = THETA_ORIGIN + self.theta_fix 
          send.sendContinuousValue(self.x, self.y, 0, self.theta, 0)
          rospy.loginfo(f'111111111111111右轉1111111111111111111')
          
        elif self.yaw < -2:
          self.y_fix = 600
          self.theta_fix = 1 + self.fix
          self.theta = THETA_ORIGIN + self.theta_fix 
          send.sendContinuousValue(self.x, self.y, 0, self.theta, 0)
          rospy.loginfo(f'11111111111111左轉1111111111111111')

        elif -2 <= self.yaw and self.yaw <= 2:
          self.y_fix = 0
          self.theta_fix = 0 + self.fix
          self.theta = THETA_ORIGIN + self.theta_fix 
          send.sendContinuousValue(self.x, self.y, 0, self.theta, 0)
          rospy.loginfo(f'111111111111直走1111111111111111111')

      elif self.lift_line or self.pick_bar: #imu2
            self.white_line()  
            self.x = 2000
            self.yaw = send.imu_value_Yaw
            rospy.loginfo(f'yaw = {self.yaw}')
            # send.sendContinuousValue(self.x, self.y, 0, self.theta, 0)
            if self.white_width_x - self.white_width_y < WHITE_SLOPE:
              if self.yaw > 2:
                  self.y_fix = -500
                  self.theta_fix = -2
                  self.theta = THETA_ORIGIN + self.theta_fix 
                  send.sendContinuousValue(self.x, self.y - 300, 0, self.theta, 0)
                  rospy.loginfo(f'22222222222222222右轉22222222222222222222')

              elif self.yaw < -2:
                  self.y_fix = 100
                  self.theta_fix = 2
                  self.theta = THETA_ORIGIN + self.theta_fix 
                  send.sendContinuousValue(self.x, self.y + 100, 0, self.theta, 0)
                  rospy.loginfo(f'222222222222222222左轉2222222222222222222')

            else:
                self.y_fix = -200
                self.theta_fix = 0
                self.theta = THETA_ORIGIN + self.theta_fix 
                send.sendContinuousValue(self.x, self.y, 0, self.theta, 0)
                rospy.loginfo(f'2222222222222直走22222222222222222222222')
        
      elif self.lift_bar: #imu_3
            self.yaw = send.imu_value_Yaw
            rospy.loginfo(f'yaw = {self.yaw}')

            self.x = 2000

            if self.yaw > 3:
              self.y_fix = -500
              self.theta_fix = -2
              self.theta = THETA_ORIGIN + self.theta_fix 
              send.sendContinuousValue(self.x, self.y, 0, self.theta, 0)
              rospy.loginfo(f'3333333333333333右轉3333333333333333')
            elif self.yaw < -3:
              self.y_fix = 500
              self.theta_fix = 2 
              self.theta = THETA_ORIGIN + self.theta_fix 
              send.sendContinuousValue(self.x, self.y, 0, self.theta, 0)
              rospy.loginfo(f'333333333333333333左轉33333333333333333')
            elif -3 <= self.yaw and self.yaw <= 3:
              self.y_fix = -500
              self.theta_fix = 0
              self.theta = THETA_ORIGIN + self.theta_fix 
              send.sendContinuousValue(self.x, self.y, 0, self.theta, 0)
              rospy.loginfo(f'33333333333333333333直走33333333333333333333333')
    
    def initial(self):
        self.red_xmax = 0
        self.red_xmin = 0
        self.red_ymax = 0
        self.red_ymin = 0
        self.red_size = 0
        
        self.white_xmax = 0
        self.white_xmin = 0
        self.white_ymax = 0
        self.white_ymin = 0
        self.white_size = 0
  
    def turn_on(self):
        rospy.loginfo(f'OOOOO = {self.body_auto}')
        if not self.body_auto:
          time.sleep(0.7)
          send.sendBodyAuto(X_ORIGIN + 600, Y_ORIGIN, 0, THETA_ORIGIN, 1, 0)
          self.body_auto = True

    def turn_off(self):
        rospy.loginfo(f'FFFFF = {self.body_auto}')
        if self.body_auto:
          send.sendBodyAuto(X_ORIGIN + 600, Y_ORIGIN, 0, THETA_ORIGIN, 1, 0)
          self.body_auto = False

    def red_line(self):
        for self.red_cnt in range(send.color_mask_subject_cnts[5]):    
          if send.color_mask_subject_size[5][self.red_cnt] > 300:
            self.red_xmax = send.color_mask_subject_XMax[5][self.red_cnt]
            self.red_xmin = send.color_mask_subject_XMin[5][self.red_cnt]
            self.red_ymax = send.color_mask_subject_YMax[5][self.red_cnt]
            self.red_ymin = send.color_mask_subject_YMin[5][self.red_cnt]
            self.red_size = send.color_mask_subject_size[5][self.red_cnt]
          rospy.loginfo(f'red_ymax = {self.red_ymax}')
          rospy.loginfo(f'red_size = {self.red_size}')
          send.drawImageFunction(1, 0, self.red_xmax, self.red_xmax, self.red_ymin, self.red_ymax, 0, 0, 0)
          send.drawImageFunction(2, 0, self.red_xmin, self.red_xmin, self.red_ymin, self.red_ymax, 0, 0, 0)
          send.drawImageFunction(3, 0, self.red_xmin, self.red_xmax, self.red_ymin, self.red_ymin, 0, 0, 0)
          send.drawImageFunction(4, 0, self.red_xmin, self.red_xmax, self.red_ymax, self.red_ymax, 0, 0, 0)

    def white_line(self):
      for self.white_cnt in range(send.color_mask_subject_cnts[6]): 
        self.white_line_wide = send.color_mask_subject_XMax[6][self.white_cnt] - send.color_mask_subject_XMin[6][self.white_cnt]  
        if send.color_mask_subject_size[6][self.white_cnt] > 500:
          self.white_xmax = send.color_mask_subject_XMax[6][self.white_cnt]
          self.white_xmin = send.color_mask_subject_XMin[6][self.white_cnt]
          self.white_ymax = send.color_mask_subject_YMax[6][self.white_cnt]
          self.white_ymin = send.color_mask_subject_YMin[6][self.white_cnt]
          self.white_size = send.color_mask_subject_size[6][self.white_cnt]
          rospy.loginfo(f'white_size = {self.white_size}')
        send.drawImageFunction(1, 0, self.white_xmax, self.white_xmax, self.white_ymin, self.white_ymax, 0, 0, 0)
        send.drawImageFunction(2, 0, self.white_xmin, self.white_xmin, self.white_ymin, self.white_ymax, 0, 0, 0)
        send.drawImageFunction(3, 0, self.white_xmin, self.white_xmax, self.white_ymin, self.white_ymin, 0, 0, 0)
        send.drawImageFunction(4, 0, self.white_xmin, self.white_xmax, self.white_ymax, self.white_ymax, 0, 0, 0)
      
    def red_line_value(self):
        self.red_line()
        self.red_middle = float(self.red_xmax + self.red_xmin) / 2
        rospy.logdebug(f'red_ymax = {self.red_ymax}')
        rospy.logdebug(f'red_ymin = {self.red_ymin}')
        rospy.logdebug(f'red_xmax = {self.red_xmax}')
        rospy.logdebug(f'red_xmin = {self.red_xmin}')
        rospy.logdebug(f'red_middle = {self.red_middle}')

    def white_line_value(self):
        self.white_line()
        rospy.logdebug(f'white_ymax = {self.white_ymax}')
        rospy.logdebug(f'white_ymin = {self.white_ymin}')
        rospy.logdebug(f'white_xmax = {self.white_xmax}')
        rospy.logdebug(f'white_xmin = {self.white_xmin}')

    def main(self):
        send.sendSensorReset()
        send.sendHeadMotor(2, HEAD_MOTOR_STAND, 50)
        r = rospy.Rate(5) #5hz
        while not rospy.is_shutdown():
          if send.is_start:
            rospy.loginfo(f'self.body_auto = {self.body_auto}')
            if not self.lift_bar: 
              if not self.lift_line: 
                  if not self.pick_bar:
                    if not self.arrive_two:
                      if not self.arrive:
                        if not self.correct:
                          if not self.imu_reset:
                              send.sendSensorReset()
                              # send.sendBodySector(3)
                              # time.sleep(1)
                              self.initial()
                              self.imu_reset = True
                          if self.imu_reset:
                            send.sendHeadMotor(2, HEAD_MOTOR_PICK, 50)
                            rospy.loginfo(f'前前前前前前前前前進進進進進進進進進')
                            self.turn_on()
                            self.imu()
                            self.red_line_value()
                            if self.red_ymax >= PICK_DIS_TWO:
                              self.correct = True
                              self. imu_reset = False
                        if self.correct:
                          self.horizontal()
                      if self.arrive:                                     
                        self.turn_off()
                        time.sleep(3.5)
                        rospy.loginfo(f'down')
                        rospy.loginfo(f'我要抬手囉!!!!!!')
                        send.sendBodySector(29)
                        time.sleep(1)
                        send.sendBodySector(PICK_ONE)
                        time.sleep(4.5)
                        self.red_line_value()
                        self.red_middle = round((self.red_xmax + self.red_xmin) / 2)                        
                        self.distance2 = round(RED_MIDDLE_IDEAL - self.red_middle)
                        if self.distance2 > 32:
                          self.distance = 10
                        elif self.distance2 < -32:
                          self.distance = -10
                        else:
                          self.distance = self.distance2
                          rospy.logdebug(f'修正 = {self.distance}')
                        if self.distance > 0:
                          for self.d in range(self.distance):
                            send.sendBodySector(32)
                            time.sleep(0.2)
                        if self.distance < 0:  
                          for self.d in range(self.distance):
                            send.sendBodySector(31)
                            time.sleep(0.2)
                        self.arrive_two = True
                        self.arrive = False
                        rospy.loginfo(f'我要撿起來囉!!!!!!!!!')
                    if self.arrive_two:  
                      rospy.loginfo(f'我撿撿撿撿撿撿撿撿撿撿')
                      time.sleep(0.5)
                      send.sendBodySector(PICK_TWO)
                      time.sleep(5.5)
                      rospy.logdebug(f'修正 = {self.distance}')
                      if self.distance > 0:
                        for self.d in range(self.distance):
                          send.sendBodySector(31)
                          time.sleep(0.2)
                      if self.distance < 0:  
                        for d in range(self.distance):
                          send.sendBodySector(32)
                          time.sleep(0.2)
                      time.sleep(0.4) 
                      send.sendBodySector(PICK_THREE)
                      time.sleep(6)  
                      rospy.logdebug(f'yaw = {self.yaw}') 
                      send.sendHeadMotor(2,HEAD_MOTOR_LIFT,50) 
                      time.sleep(0.5)
                      self.pick_bar = True 
                      self.arrive_two = False    
                      rospy.loginfo(f'舉起線在哪!?')
                  if self.pick_bar:
                    # send.sendBodySector(3)
                    # time.sleep(1)
                    self.turn_on()
                    rospy.logdebug(f'##################################################')
                    rospy.loginfo(f'舉起線我來了!!!')
                    self.imu()                                 #imu2
                    rospy.logdebug(f'white_ymax = {self.white_ymax}')
                    time.sleep(3)
                    if self.white_ymax > LIFT_DIS_MIN and self.white_ymax < LIFT_DIS_MAX:
                      rospy.loginfo(f'舉起線要到了')
                      send.sendHeadMotor(2, HEAD_MOTOR_FINISH, 50) 
                      self.lift_line = True
                      self.pick_bar = False
                    else:
                      rospy.loginfo(f'imu微調')
                      self.imu()
              if self.lift_line:
                self.white_line()
                rospy.logdebug(f'distance 2 = {self.white_ymax}')             
                if self.white_ymax > LIFT_STOP_MIN and self.white_ymax < LIFT_STOP_MAX:
                  rospy.loginfo(f'======停下，舉起======')
                  self.turn_off()
                  time.sleep(1.5)
                  # send.sendBodySector(4)
                  # time.sleep(1)
                  send.sendBodySector(LIFT)
                  time.sleep(19)
                  send.sendBodySector(666)
                  time.sleep(1.5)
                  self.lift_bar = True
                  self.lift_line = False
                  rospy.logdebug(f'imu_value_Pitch = {send.imu_value_Pitch}')
                else:           
                  rospy.loginfo(f'imu微調')    
                  self.imu()                      
            elif self.lift_bar:
              rospy.loginfo(f'##################################################')
              rospy.loginfo(f'======終點我來了======')
              self.turn_on()
              time.sleep(0.5)
              self.imu()                 #imu3
              rospy.loginfo(f'耶~~~~~到了~~~~~')
          if not send.is_start:
            self.turn_off()
            rospy.loginfo(f'草尼馬,Ready to go!')
            self.white_line_value()
            rospy.logdebug(f'==================================')
            rospy.logdebug(f'==================================')
            self.red_line_value()            
          r.sleep()

if __name__ == '__main__':
    try:
        strategy = WeightLift()
        r = rospy.Rate(20)

        while not rospy.is_shutdown():
          strategy.main()
          r.sleep()  
            
    except rospy.ROSInterruptException:
        pass

