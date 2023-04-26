#!/usr/bin/env python
#coding=utf-8
from xml.etree.ElementTree import XML
import rospy
import numpy as np
from Python_API import Sendmessage
import time

send = Sendmessage()

'''
rospy.logdebug()
rospy.loginfo()
rospy.logwarn()
rospy.logerr()
rospy.logfatal()
'''

# -----------------  平移基準量  -------------------
# 直走(START -> PICK)
FORWARD_X               = 1500
FORWARD_Y               = -300
FORWARD_THETA           = 1

# 直走(PICK -> LIFT)
FORWARD_TO_LIFT_X       = 4000
FORWARD_TO_LIFT_Y       = 0
FORWARD_TO_LIFT_THETA   = 0

# 直走(LIFT -> END)
FORWARD_TO_END_X        = 3500
FORWARD_TO_END_Y        = 0  
FORWARD_TO_END_THETA    = 0

# 左平移參數
TRANSLATE_LEFT_X        = -1400
TRANSLATE_LEFT_Y        = 1400
TRANSLATE_LEFT_THETA    = 0

# 右平移參數
TRANSLATE_RIGHT_X       = -1400
TRANSLATE_RIGHT_Y       = -2000
TRANSLATE_RIGHT_THETA   = 0

# --------------------------------------------------
# forward_fix 修正大小
FORWARD_FIX_BIG         = 1200
FORWARD_FIX_NOR         = 700

# 平移修改量
TRANSLATE_FIX_BIG       = 1400  #大修
TRANSLATE_FIX_NOR       = 900   #中修
TRANSLATE_FIX_MIN       = 400   #小修

# 平移標準參數
TRANSLATE_STANDARD_BIG  = 20
TRANSLATE_STANDARD_MIN  = 2

# 紅色左右基準
TARGET_RED_X_LEFT       = 157
TARGET_RED_X_RIGHT      = 160

# 紅線(桿子)停止基準
RED_Y_STANDARD          = 146

# 距離紅線(桿子)遠近
RED_Y_DISTANCE_FAR      = 100
RED_Y_DISTANCE_NEAR     = 50

# yaw 基準值
YAW_STANDARD_TO_PICK    = 8
YAW_STANDAND_TO_LIFT    = 2
YAW_STANDARD_TO_END     = 3

# 影像中點
IMAGE_MIDDLE_X          = 160
IMAGE_MIDDLE_Y          = 120

#-------------    磁區    ----------------
DOWN_60    = 60     # 60片蹲下
DOWN_70    = 70     # 70片蹲下

PICK_60    = 61     # 60片拿起
PICK_70    = 71     # 70片拿起

LIFT_60    = 62     # 60片舉起
LIFT_70    = 72     # 70片舉起


class WeightLifting:
    
    # 初始設定
    def __init__(self):
        self.body_auto      = False     #是否開始踏步
        self.start_tem      = True      #起始是否需要左右平移
        self.to_lift_tem    = True
        self.standing_tem   = True

        self.imu_reset      = False     #起點 imu reset
        self.pick_bar       = False     #是否抵達 pick_bar (第一條線)
        self.lift_bar       = False     #是否抵達 lift_bar (第二條線)
        self.middle_pole    = False     #是否抵達桿子的中間
        self.get_down       = False     #蹲下
        self.stand_up       = False     #站起
        
        self.forward        = 0
        self.translate      = 0
        self.theta          = 0
        self.now_state      = ["前進", "平移", "旋轉"]
        
        self.translate_fix  = 0
        self.yaw            = 0
        self.imu_fix        = 0
        self.number         = 0

        self.target_xmax = 0
        self.target_xmin = 0
        self.target_ymax = 0
        self.target_ymin = 0

        self.white_xmax = 0
        self.white_xmin = 0
        self.white_ymax = 0
        self.white_ymin = 0

    # 開始踏步
    def turn_on(self):

        rospy.logerr("-------------------")
        rospy.logerr("START!!")
        rospy.logerr("-------------------")
        if self.body_auto:
            pass
        else:
            send.sendBodyAuto(200, 0, 0, 0, 1, 0)
            #self.start_forward()
            self.body_auto = True

    # 停止踏步
    def turn_off(self):
        rospy.logerr("-------------------")
        rospy.logerr("STOP!!")
        rospy.logerr("-------------------")

        if self.body_auto:
            send.sendBodyAuto(200, 0, 0, 0, 1, 0)
            self.body_auto = False
        else:
            pass
     
    # imu修正
    def imu(self):

        if self.lift_bar:
            self.yaw = send.imu_value_Yaw

            if self.yaw > YAW_STANDARD_TO_END: 
                self.theta += -2
                self.now_state[2] = "右旋_END"
            
            elif self.yaw < -YAW_STANDARD_TO_END:
                self.theta += 1
                self.now_state[2] = "左旋_END"

            elif (-YAW_STANDARD_TO_END <= self.yaw) and (self.yaw <= YAW_STANDARD_TO_END):
                self.now_state[2] = "直走_END"

        elif self.stand_up:
            self.yaw = send.imu_value_Yaw

            if self.yaw > YAW_STANDAND_TO_LIFT: 
                self.theta += -2
                self.now_state[2] = "右旋_LIFT"
            
            elif self.yaw < -YAW_STANDAND_TO_LIFT:
                self.theta += 2
                self.now_state[2] = "左旋_LIFT"

            elif (-YAW_STANDAND_TO_LIFT <= self.yaw) and (self.yaw <= YAW_STANDAND_TO_LIFT):
                self.now_state[2] = "直走_LIFT"

        elif self.imu_reset:
            self.red_line()
            target_x_middle = (self.target_xmax + self.target_xmin) / 2

            if target_x_middle > IMAGE_MIDDLE_X:
                self.imu_fix = 1
            if target_x_middle < IMAGE_MIDDLE_X:
                self.imu_fix = -3
            else:
                self.imu_fix = 0

            self.yaw = send.imu_value_Yaw

            if self.yaw > YAW_STANDARD_TO_PICK: 
                self.theta += self.imu_fix - 2
                self.now_state[2] = "右旋"

            elif self.yaw < -YAW_STANDARD_TO_PICK:
                self.theta += self.imu_fix - 1
                self.now_state[2] = "左旋"

            elif (YAW_STANDARD_TO_PICK <= self.yaw) and (self.yaw <= YAW_STANDARD_TO_PICK):
                self.now_state[2] = "直走"

    # 紅線判斷
    def red_line(self):
        self.target_xmax = 0
        self.target_xmin = 0
        self.target_ymax = 0
        self.target_ymin = 0
        target_size = 0
        for red_cnt in range(send.color_mask_subject_cnts[5]):  #  send.color_mask_subject_cnts[5] is value about red range
            if send.color_mask_subject_size[5][red_cnt]>200:
                self.target_xmax = send.color_mask_subject_XMax[5][red_cnt]
                self.target_xmin = send.color_mask_subject_XMin[5][red_cnt]
                self.target_ymax = send.color_mask_subject_YMax[5][red_cnt]
                self.target_ymin = send.color_mask_subject_YMin[5][red_cnt]
                target_size = send.color_mask_subject_size[5][red_cnt]
        
        send.drawImageFunction(1, 0, self.target_xmax, self.target_xmax, self.target_ymin, self.target_ymax, 0, 0, 0)
        send.drawImageFunction(2, 0, self.target_xmin, self.target_xmin, self.target_ymin, self.target_ymax, 0, 0, 0)
        send.drawImageFunction(3, 0, self.target_xmin, self.target_xmax, self.target_ymin, self.target_ymin, 0, 0, 0)
        send.drawImageFunction(4, 0, self.target_xmin, self.target_xmax, self.target_ymax, self.target_ymax, 0, 0, 0)

    # 白線判斷
    def white_line(self):
        self.white_xmax = 0
        self.white_xmin = 0
        self.white_ymax = 0
        self.white_ymin = 0
        white_size = 0
        for white_cnt in range(send.color_mask_subject_cnts[6]): 
            white_line_wide=send.color_mask_subject_XMax[6][white_cnt]-send.color_mask_subject_XMin[6][white_cnt]  
            if send.color_mask_subject_size[6][white_cnt]>500:
                self.white_xmax = send.color_mask_subject_XMax[6][white_cnt]
                self.white_xmin = send.color_mask_subject_XMin[6][white_cnt]
                self.white_ymax = send.color_mask_subject_YMax[6][white_cnt]
                self.white_ymin = send.color_mask_subject_YMin[6][white_cnt]
                white_size = send.color_mask_subject_size[6][white_cnt]

        send.drawImageFunction(1, 0, self.white_xmax, self.white_xmax, self.white_ymin, self.white_ymax, 0, 0, 0)
        send.drawImageFunction(2, 0, self.white_xmin, self.white_xmin, self.white_ymin, self.white_ymax, 0, 0, 0)
        send.drawImageFunction(3, 0, self.white_xmin, self.white_xmax, self.white_ymin, self.white_ymin, 0, 0, 0)
        send.drawImageFunction(4, 0, self.white_xmin, self.white_xmax, self.white_ymax, self.white_ymax, 0, 0, 0)

    # 紅色的左右平移
    def correct_target(self):
        self.red_line()
        red_middle = float(self.target_xmax + self.target_xmin) / 2
        rospy.logdebug(f'red_middle = {red_middle}')
        #---------------------------------------  前進 + 平移  -------------------------------------------
        if self.target_ymax < RED_Y_STANDARD:
            if red_middle != 0:
                if red_middle > (TARGET_RED_X_RIGHT + TRANSLATE_STANDARD_BIG):
                    self.translate_fix = -TRANSLATE_FIX_BIG
                    self.now_state[1] = 'big right'

                elif (TARGET_RED_X_RIGHT + TRANSLATE_STANDARD_MIN) < red_middle and red_middle <= (TARGET_RED_X_RIGHT + TRANSLATE_STANDARD_BIG):
                    self.translate_fix = -TRANSLATE_FIX_NOR
                    self.now_state[1] = 'normal right'

                elif (TARGET_RED_X_RIGHT + TRANSLATE_STANDARD_MIN) <= red_middle:
                    self.translate_fix = -TRANSLATE_FIX_MIN
                    self.now_state[1] = 'small right'
                #=====================================================================================
                elif (TARGET_RED_X_LEFT - TRANSLATE_STANDARD_BIG) < red_middle:
                    self.translate_fix = TRANSLATE_FIX_BIG
                    self.now_state[1] = 'big left'

                elif (TARGET_RED_X_LEFT - TRANSLATE_STANDARD_MIN) > red_middle and red_middle >= (TARGET_RED_X_LEFT - TRANSLATE_STANDARD_BIG):
                    self.translate_fix = TRANSLATE_FIX_NOR
                    self.now_state[1] = 'normal left'

                elif (TARGET_RED_X_LEFT - TRANSLATE_STANDARD_MIN) >= red_middle:
                    self.translate_fix = TRANSLATE_FIX_MIN
                    self.now_state[1] = 'small left'
                #======================================================================================
                time.sleep(0.5)
            
            else:
                pass
        else:
           pass

    # 前進量
    def start_forward(self):

        if self.lift_bar:
            self.forward    = 1000 
            self.translate  = FORWARD_TO_END_Y
            self.theta      = FORWARD_TO_END_THETA
            self.imu()
            if FORWARD_TO_END_X > self.forward:
              self.forward += 1000
            send.sendContinuousValue(self.forward, self.translate, 0, self.theta, 0)
            rospy.logwarn(f'{self.now_state}')
            

        elif self.stand_up:
            self.forward    = FORWARD_TO_LIFT_X
            self.translate  = FORWARD_TO_LIFT_Y
            self.theta      = FORWARD_TO_LIFT_THETA
            self.imu()
            send.sendContinuousValue(self.forward, self.translate, 0, self.theta, 0)
            rospy.logwarn(f'{self.now_state}')

        elif self.imu_reset:
            self.forward    = FORWARD_X
            self.translate  = FORWARD_Y
            self.theta      = FORWARD_THETA

            # 依照距離桿子遠近變更前進速度
            if not self.start_tem:
                self.red_line()

                if (self.target_ymax - 20) < RED_Y_STANDARD:
                    if (RED_Y_STANDARD - self.target_ymax) > RED_Y_DISTANCE_FAR:
                        self.forward += FORWARD_FIX_BIG
                        self.now_state[0] = "大前進"
                    elif (RED_Y_STANDARD - self.target_ymax) > RED_Y_DISTANCE_NEAR:
                        self.forward += FORWARD_FIX_NOR
                        self.now_state[0] = "前進"
                    else:
                        pass
                else:
                    self.forward -= 500
                    self.now_state[0] = "小前進"
                self.imu()
                self.correct_target()

            if (self.target_ymax - 20) < RED_Y_STANDARD:
                send.sendContinuousValue(self.forward, self.translate + self.translate_fix, 0, self.theta, 0)
                start_delay = time.time()
                end_delay = -9999
                while (end_delay - start_delay) < 0.7:
                    end_delay = time.time()
                    rospy.logwarn(f'{self.now_state}')
            else:
                pass
    
    def main(self):
      if send.is_start:         #跑策略
            if not self.lift_bar:
                if not self.stand_up:
                    if not self.get_down:
                        if not self.middle_pole:
                            if not self.pick_bar:
                                if not self.imu_reset:
                                    send.sendSensorReset()
                                    self.imu_reset = True
                                    rospy.logerr("----------------------")
                                    rospy.logerr("READY!!")
                                    rospy.logerr("----------------------")
                                    rospy.logerr(f"imu_reset: {self.imu_reset}")
                                elif self.imu_reset:
                                    send.sendHeadMotor(2, 2746, 50)

                                    if self.start_tem:
                                        self.turn_on()
                                        self.start_forward()
                                        # 右移
                                        if send.DIOValue == 25 or send.DIOValue == 29 or send.DIOValue == 33 or send.DIOValue == 37:
                                            send.sendContinuousValue(self.forward, self.translate - 2000, 0, self.theta, 0)
                                            # delay 7 sec
                                            start_delay = time.time()
                                            end_delay = -9999
                                            while (end_delay - start_delay) < 7:
                                                end_delay = time.time()
                                                rospy.loginfo("=======")
                                                rospy.loginfo(f"delay: {end_delay - start_delay}")
                                                rospy.loginfo("=======")
                                        
                                        #左移
                                        elif send.DIOValue == 26 or send.DIOValue == 30 or send.DIOValue == 34 or send.DIOValue == 38:
                                            send.sendContinuousValue(self.forward, self.translate + 2000, 0, self.theta, 0)
                                            # delay 7 sec
                                            start_delay = time.time()
                                            end_delay = -9999
                                            while (end_delay - start_delay) < 7:
                                                end_delay = time.time()
                                                rospy.loginfo("=======")
                                                rospy.loginfo(f"delay: {end_delay - start_delay}")
                                                rospy.loginfo("=======")
                                        self.start_tem = False
                                    
                                    self.start_forward()

                                    self.red_line()
                                    #rospy.logdebug(f'red_middle = {red_middle}')
                                    self.start_forward()

                                    if self.target_ymax >= RED_Y_STANDARD:
                                        self.pick_bar = True
                                        rospy.logerr("==================")
                                        rospy.logerr(f"pick_bar = {self.pick_bar}")
                            elif self.pick_bar:
                                self.red_line()
                                red_middle = float(self.target_xmax + self.target_xmin) / 2
                                rospy.logdebug(f'red_middle = {red_middle}')
                                
                                if red_middle < TARGET_RED_X_LEFT:
                                    send.sendContinuousValue(TRANSLATE_LEFT_X, TRANSLATE_LEFT_Y, 0, TRANSLATE_LEFT_THETA, 0)
                                    self.now_state[1] = 'move left'
                                    self.number += 1
                                    
                                elif red_middle > TARGET_RED_X_RIGHT:
                                    send.sendContinuousValue(TRANSLATE_RIGHT_X, TRANSLATE_RIGHT_Y, 0, TRANSLATE_RIGHT_THETA, 0)
                                    self.now_state[1] = 'move right'
                                    self.number += 1
                                else:
                                    self.middle_pole  = True
                                    self.now_state[0] = "gogo"
                                    self.now_state[1] = "無"
                                    rospy.logerr("================")
                                    rospy.logerr(f'middle_pole = {self.middle_pole}')
                                
                                # start_delay = time.time()
                                # end_delay = -9999
                                # while (end_delay - start_delay) < 0.5:
                                #     end_delay = time.time()
                                rospy.logwarn(f'{self.now_state[1]}')
                                
                                # 避免一直左右調整，直接到下一階段
                                if self.number == 30:
                                    self.middle_pole = True
                                    rospy.logerr("================")
                                    rospy.logerr(f'middle_pole = {self.middle_pole}')

                                # self.red_line()
                                # red_middle = float(self.target_xmax + self.target_xmin) / 2
                                # if (red_middle >= TARGET_RED_X_LEFT) and (red_middle <= TARGET_RED_X_RIGHT):
                                #     self.middle_pole  = True
                                #     self.now_state[0] = "no"
                                #     self.now_state[1] = "無"
                                #     rospy.logerr("================")
                                #     rospy.logerr(f'middle_pole = {self.middle_pole}')

                        elif self.middle_pole:
                            self.turn_off()
                            time.sleep(1)
                            rospy.logerr("----蹲蹲蹲----")
                            time.sleep(2)
                            # 60片蹲下
                            if send.DIOValue == 24 or send.DIOValue == 25 or send.DIOValue == 26:
                                send.sendBodySector(DOWN_60)
                            # 70片蹲下
                            elif send.DIOValue == 28 or send.DIOValue == 29 or send.DIOValue == 30:
                                send.sendBodySector(DOWN_70)
                            
                            time.sleep(9.5)
                            self.get_down = True
                            rospy.logerr("==================")
                            rospy.logerr(f'get_down = {self.get_down}')
                            
                    elif self.get_down:
                        rospy.logerr("----拿起----")
                        time.sleep(1)
                        # 60片拿起
                        if send.DIOValue == 24 or send.DIOValue == 25 or send.DIOValue == 26:  #60
                            send.sendBodySector(PICK_60)
                            time.sleep(17)
                        # 70片拿起
                        elif send.DIOValue == 28 or send.DIOValue == 29 or send.DIOValue == 30:  #70
                            send.sendBodySector(PICK_70)
                            time.sleep(19)
                            #send.sendBodySector(26)  # 伸右腳的磁區，maybe可以拿掉
                            #time.sleep(0.5)
                        

                        send.sendHeadMotor(2, 2802, 50)
                        time.sleep(1)
                        self.stand_up = True
                        rospy.logerr("==================")
                        rospy.logerr(f'stand_up = {self.stand_up}')

                elif self.stand_up:
                    self.turn_on()
                    rospy.logerr("------moving to liftline------")
                    self.start_forward()
                    time.sleep(0.5)
                    self.start_forward()

                    if  self.to_lift_tem:
                        self.start_forward()
                        # delay 5 sec
                        start_delay = time.time()
                        end_delay = -9999
                        while (end_delay - start_delay) < 5:
                            end_delay = time.time()
                            rospy.loginfo("=======")
                            rospy.loginfo(f"delay: {end_delay - start_delay}")
                            rospy.loginfo("=======")
                        self.to_lift_tem = False

                    self.white_line()
                    time.sleep(0.35)
                    self.start_forward()
                    rospy.logerr(f"while_ymax = {self.white_ymax}")
                    
                    if self.white_ymax > 180 and self.white_ymax < 240:
                        rospy.logerr("-----Find While_line-----")
                        rospy.logerr("1")
                        rospy.logerr("2")
                        time.sleep(1)
                        rospy.logerr("3")

                        rospy.logerr('-----stop and lift-----')
                        self.turn_off()
                        time.sleep(1.5)
                        self.turn_off()
                        time.sleep(0.5)
                        rospy.logerr("----舉起----")

                        # 60片
                        if send.DIOValue == 24 or send.DIOValue == 25 or send.DIOValue == 26: 
                            send.sendBodySector(LIFT_60)
                            time.sleep(12)
                        # 70片
                        elif send.DIOValue == 28 or send.DIOValue == 29 or send.DIOValue == 30:
                            send.sendBodySector(LIFT_70)
                            time.sleep(12)
                        
                        self.lift_bar = True
                        rospy.logerr("==================")
                        rospy.logerr(f'lift_bar = {self.lift_bar}')
                        time.sleep(1.5)
                    else:
                        self.start_forward()
                        rospy.logerr("keep going")
            elif self.lift_bar:
                rospy.logerr('-----keep going to endline----')
                self.turn_on()
                time.sleep(0.5)
                self.start_forward()

      elif not send.is_start:
          self.turn_off()
          if self.standing_tem: 
              send.sendBodySector(29)
              time.sleep(0.5)
              send.sendBodySector(27)
              self.standing_tem = False
          
          self.white_line()
          rospy.logdebug(f'w_ymax = {self.white_ymax}')
          
          rospy.logdebug(f'red y_max = {send.color_mask_subject_YMax[6][0]}')
          rospy.logdebug(f'red y_min = {send.color_mask_subject_YMin[6][0]}')

          self.red_line()
          red_middle = float(self.target_xmax + self.target_xmin) / 2
          rospy.logdebug(f'red_middle = {red_middle}')

          send.sendHeadMotor(2, 2746, 50)
          #rospy.logdebug()
          rospy.logdebug(f'white x_max = {send.color_mask_subject_XMax[5][0]}')
          rospy.logdebug(f'white x_min = {send.color_mask_subject_XMin[5][0]}')
          #rospy.logdebug()

imgdata = [[None for high in range(240)]for width in range(320)]

if __name__ == "__main__":

    wl = WeightLifting()

    try:
        send.sendSensorReset()
        send.sendHeadMotor(2,2646,50)
        r = rospy.Rate(5) #5hz

        while not rospy.is_shutdown():
            wl.main()
            r.sleep()

    except rospy.ROSInterruptException:
        pass