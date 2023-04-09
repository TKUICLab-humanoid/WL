#!/usr/bin/env python
#coding=utf-8
from xml.etree.ElementTree import XML
import rospy
import numpy as np
from Python_API import Sendmessage
import time
find_white_line         = False
pick_bar                = False #是否夾起槓桿
lift_bar                = False #是否舉起槓桿
Body_Auto               = False
arrive                  = False #是否站在槓桿中間
arrive2                 = False #是否蹲下
imu_reset               = False
lift_line               = False #是否抵達 lift line
correct                 = False #是否站在賽道中間
start_gogo              = True #初始偏移的調整
tem                     = True # go to lift 的暫停
red_ymax_revise         = False
imu_yaw_revise          = False
stand = True

yaw     = 0

BIG_X = 1000
NOR_X = 700
THEAT = 0
X = 0

# 直走
x       = 1500
y       = -300
z       = 0
theta   = 1

# imu2_直走
x2      = 4000
y2      = 0
z2      = 0
theta2  = 0

# imu3_直走
x3      = 3500
y3      = 0
z3      = 0  
theta3  = 0

# 左平移參數
xl      = -1200
yl      = 1400
zl      = 0
tl      = 0

# 右平移參數
xr      = -1200
yr      = -2000
zr      = 0
tr      = 0

# 紅色左右基準
target_left   = 157
target_right  = 160

# 右平移標準參數
big_right_Standard_1     = 20
small_right_Standard_1   = 2

# 左平移標準參數
big_left_Standard_1      = -20
small_left_Standard_1    = -2

#紅線停止基準
Red_Standard_min         = 154

#red_middle2=166.5

#-------------    磁區    ----------------
pick1    = 60     # 蹲下
pick2    = 70

pick3    = 61     # 60片拿起
pick4    = 71     # 70片拿起

lift     = 62     # 60片舉起
lift2    = 72     # 70片舉起


def turn_on():
    global Body_Auto
    # time.sleep(0.8)
    print("-------------------")
    print("START!!")
    print("-------------------")
    if Body_Auto == True:
      pass
    elif Body_Auto == False:
      time.sleep(0.7)
      #send.sendSensorReset() ==========================================
      send.sendBodyAuto(200, 0, 0, 0, 1, 0)
      Body_Auto = True

def turn_off():
    global Body_Auto
    print("-------------------")
    print("STOP!!")
    print("-------------------")
    if Body_Auto == True:
      send.sendBodyAuto(200, 0, 0, 0, 1, 0)
      Body_Auto = False
    elif Body_Auto == False:
      pass

def red_line():
    target_xmax = 0
    target_xmin = 0
    target_ymax = 0
    target_ymin = 0
    target_size = 0
    for red_cnt in range(send.color_mask_subject_cnts[5]):  #  send.color_mask_subject_cnts[5] is value about red range
      if send.color_mask_subject_size[5][red_cnt]>200:
        target_xmax = send.color_mask_subject_XMax[5][red_cnt]
        target_xmin = send.color_mask_subject_XMin[5][red_cnt]
        target_ymax = send.color_mask_subject_YMax[5][red_cnt]
        target_ymin = send.color_mask_subject_YMin[5][red_cnt]
        target_size = send.color_mask_subject_size[5][red_cnt]
        #print(target_xmax,target_xmin)
      # print(target_ymax)
      # print(target_size)
      send.drawImageFunction(1,0,target_xmax,target_xmax,target_ymin,target_ymax,0,0,0)
      send.drawImageFunction(2,0,target_xmin,target_xmin,target_ymin,target_ymax,0,0,0)
      send.drawImageFunction(3,0,target_xmin,target_xmax,target_ymin,target_ymin,0,0,0)
      send.drawImageFunction(4,0,target_xmin,target_xmax,target_ymax,target_ymax,0,0,0)
    return target_ymax ,target_ymin ,target_xmax ,target_xmin

def white_line():
    white_xmax = 0
    white_xmin = 0
    white_ymax = 0
    white_ymin = 0
    white_size = 0
    for white_cnt in range(send.color_mask_subject_cnts[6]): 
      white_line_wide=send.color_mask_subject_XMax[6][white_cnt]-send.color_mask_subject_XMin[6][white_cnt]  
      if send.color_mask_subject_size[6][white_cnt]>500:
        white_xmax = send.color_mask_subject_XMax[6][white_cnt]
        white_xmin = send.color_mask_subject_XMin[6][white_cnt]
        white_ymax = send.color_mask_subject_YMax[6][white_cnt]
        white_ymin = send.color_mask_subject_YMin[6][white_cnt]
        white_size = send.color_mask_subject_size[6][white_cnt]
     #print(target_xmax,target_xmin)
     #print(target_ymax,target_ymin)
     # print(white_size)
      send.drawImageFunction(1,0,white_xmax,white_xmax,white_ymin,white_ymax,0,0,0)
      send.drawImageFunction(2,0,white_xmin,white_xmin,white_ymin,white_ymax,0,0,0)
      send.drawImageFunction(3,0,white_xmin,white_xmax,white_ymin,white_ymin,0,0,0)
      send.drawImageFunction(4,0,white_xmin,white_xmax,white_ymax,white_ymax,0,0,0)
    return white_ymax,white_ymin

def imu():
    target_ymax,target_ymin,target_xmax,target_xmin=red_line()
    target_middle=(target_xmax+target_xmin)/2
    if target_middle>160:
      fix=1
    if target_middle<160:
      fix=-2
    else :
      fix=0
    yaw_1=send.imu_value_Yaw
    #print("yaw =", yaw_1)

    if (Red_Standard_min - target_ymax) > 100:
      X = x + BIG_X
    elif (Red_Standard_min - target_ymax) >50:
      X = x + NOR_X
    else:
      X = x

    if yaw_1>8: 
      THETA = theta+fix-1
      print("turn right")
      
    elif yaw_1<-8:
      THETA = theta+fix-1
      print("turn left")

    elif -8<=yaw_1 and yaw_1<=8:
      THETA = theta
      print("walk straight")

    send.sendContinuousValue(X,y,z,THETA,0)

def imu1_5():
    target_ymax,target_ymin,target_xmax,target_xmin=red_line()
    target_middle=(target_xmax+target_xmin)/2
    if target_middle>160:
      fix=2
    if target_middle<160:
      fix=-3
    else :
      fix=0
    yaw_1=send.imu_value_Yaw
    #print("yaw =", yaw_1)

    if yaw_1>8: 
      send.sendContinuousValue(x,y,z,theta+fix-2,0)
      print("turn right")
      
    elif yaw_1<-8:
      send.sendContinuousValue(x,y,z,theta+fix-1,0)
      print("turn left")

    elif -8<=yaw_1 and yaw_1<=8:
      send.sendContinuousValue(x,y,z,theta,0)
      print("walk straight")

def imu_2():
    yaw_1=send.imu_value_Yaw
    #print("cccccc",yaw_1) 
    #time.sleep(0.5)
    yaw_1+=yaw
    #print("yaw =", yaw_1)

    if yaw_1>2: 
      send.sendContinuousValue(x2,y,z,theta2-2,0)
      print("turn right 2")
      
    elif yaw_1<-2 :
      send.sendContinuousValue(x2,y,z,theta2+2,0)
      print("turn left 2")

    elif -2<=yaw_1 and yaw_1<=2 :
      send.sendContinuousValue(x2,y,z,theta2,0)
      print("walk straight 2")
    #print("gggggg",theta)  

def imu_3():
    yaw_1=send.imu_value_Yaw
    yaw_1+=yaw
    #print("yaw =", yaw_1)

    if yaw_1>3: 
      send.sendContinuousValue(x3,y3,z3,theta3-2,0)
      print("turn right 3")
      
    elif yaw_1<-3 :
      send.sendContinuousValue(x3,y3,z3,theta3+1,0)
      print("turn left 3")

    elif -3<=yaw_1 and yaw_1<=3 :
      send.sendContinuousValue(x3,y3,z3,theta3,0)
      print("walk straight 3")

def afterbar():
    print('revise')
    yaw=send.imu_value_Yaw
    time.sleep(0.35)
    send.sendSensorReset()
    return yaw

def correct_target():
    global correct
    target_ymax,target_ymin,target_xmax,target_xmin=red_line()
    red_middle=float(target_xmax+target_xmin)/2
    print('red_middle =',red_middle)
    #---------------------------------------  前進 + 平移  -------------------------------------------
    #if correct == False:
    if target_ymax < Red_Standard_min:
        if red_middle != 0:
          if red_middle > (target_right + big_right_Standard_1): 
            send.sendContinuousValue(x, y-1700, z, theta-2, 0)
            print('big right')

          elif (target_right + small_right_Standard_1) < red_middle and red_middle <= (target_right + big_right_Standard_1):
            send.sendContinuousValue(x, y-1300, z, theta-2, 0)
            print('normal right')

          elif (target_right + small_right_Standard_1) <= red_middle:
            send.sendContinuousValue(x, y-800, z, theta-2, 0)
            print('small right')
            #=====================================================================================
          elif (target_left + big_left_Standard_1) < red_middle:
            send.sendContinuousValue(x, y+1800, z, theta, 0)
            print('big left')

          elif (target_left + small_left_Standard_1) > red_middle and red_middle >= (target_left + big_left_Standard_1):
            send.sendContinuousValue(x, y+1400, z, theta, 0)
            print('normal left')

          elif (target_left + small_left_Standard_1) >= red_middle:
            send.sendContinuousValue(x, y+900, z, theta, 0)
            print('small left')
          #======================================================================================
          time.sleep(0.5)
          
        else:
          imu()
          print("gogo")

    # else:
    #   pass
    #----------------------------------------  紅線前平移  -------------------------------------------------
    # else:
    #   if red_middle >= target_left and red_middle <= target_right:
    #     if red_middle != 0:
    #       if red_middle > (target_right + big_right_Standard_1):
    #         send.sendContinuousValue(xr, yr-1000, zr, tr, 0)
    #         print('右！右！右！大平移')

    #     elif (target_right + small_right_Standard_1) < red_middle and red_middle <= (target_right + big_right_Standard_1):
    #       send.sendContinuousValue(xr, yr-500, zr, tr, 0)
    #       print('右！右！右！平移')

    #     elif (target_right + small_right_Standard_1) <= red_middle:
    #       send.sendContinuousValue(xr, yr+200, zr, tr, 0)
    #       print('右！右！右！小平移')
    #     #=====================================================================================
    #     elif (target_left + big_left_Standard_1) < red_middle:
    #       send.sendContinuousValue(xl, yl+1000, zl, tl, 0)
    #       print('左！左！左！大平移')
 
    #     elif (target_left + small_left_Standard_1) > red_middle and red_middle >= (target_left + big_left_Standard_1):
    #       send.sendContinuousValue(xl, yl+500, zl, tl, 0)
    #       print('左！左！左！平移')

    #     elif (target_left + small_left_Standard_1) >= red_middle:
    #       send.sendContinuousValue(xl, yl-200, zl, tl, 0)
    #       print('左！左！左！小平移')
    #     #======================================================================================
    #     time.sleep(0.3)      
      # else:
      #   pass
    return red_middle
    # if red_middle>157 and red_middle<172:
    #   arrive=True
    # return arrive
  
  


imgdata= [[None for high in range(240)]for width in range(320)]
if __name__ == '__main__':
    send = Sendmessage()

    try:
        #find_white_line=True
        #lift_bar=True
        #pick_bar = True
        time.sleep(0.35)
        send.sendSensorReset()
        send.sendHeadMotor(2,2646,50)
        r = rospy.Rate(5) #5hz
        while not rospy.is_shutdown():
          if send.is_start == True:
            #print(Body_Auto)
            if lift_bar == False: 
              # if lift_line == False: 
                #if find_white_line == False:
                  if pick_bar == False:
                    if arrive2 == False:
                      if arrive == False:
                        if correct == False:

                          if imu_reset == False:
                            #send.sendBodySector(666) #起始動作
                            time.sleep(0.5)
                            send.sendSensorReset()
                            imu_reset = True
                            print("----------------------")
                            print("READY!!")
                            print("----------------------")
                            print("imu_reset:", imu_reset)

                          if imu_reset == True:
                            send.sendHeadMotor(2,2746,50)
                            #注意平移量！！
                            if start_gogo == True:
                              turn_on()
                              imu()
                              imu()
                              if send.DIOValue == 25 or send.DIOValue == 29 or send.DIOValue == 33 or send.DIOValue == 37: #右移
                                imu()
                                send.sendContinuousValue(x, y-2000, z, theta, 0)
                                time.sleep(7)
                                start_gogo = False
                              elif send.DIOValue == 26 or send.DIOValue == 30 or send.DIOValue == 34 or send.DIOValue == 38: #左移
                                imu1_5()
                                send.sendContinuousValue(x, y+2000, z, theta, 0)
                                time.sleep(7)
                                start_gogo = False

                            imu()
                            correct_target()
                            #print("red_middle =", correct_target())

                            target_ymax,target_ymin,target_xmax,target_xmin=red_line()
                            print("red_ymax =", target_ymax)
                            
                            if target_ymax>115 and target_ymax<128:
                              imu()
                              #time.sleep(0.7)
                              print("time:", time.sleep(0.7))
                              target_ymax ,target_ymin ,target_xmax ,target_xmin = red_line()
                              #print("target_ymax_2:", target_ymax)
                            if target_ymax >= Red_Standard_min:
                              correct = True
                              print("==================")
                              print("correct =", correct)

                        if correct == True:
                          target_ymax, target_ymin, target_xmax, target_xmin = red_line()
                          red_middle = float(target_xmax+target_xmin) / 2
                          print('red_middle =', red_middle)
                          #imu1_5()
                          if red_middle<target_left:
                            send.sendContinuousValue(xl,yl,zl,tl,0)
                            print('move left')
                          if red_middle>target_right:
                            send.sendContinuousValue(xr,yr,zr,tr,0)
                            print('move right')
                          time.sleep(0.5)

                          #correct_target()
                          #red_middle = correct_target()
                          print("red_middle =", red_middle)
                          
                          if red_middle >= target_left and red_middle <= target_right:
                            arrive = True
                            print("================")
                            print("arrive =", arrive)

                      if arrive == True:
                        #print("stop")                                     
                        turn_off()
                        time.sleep(1)
                        print("----蹲蹲蹲----")
                        time.sleep(2)
                        if send.DIOValue == 24 or send.DIOValue == 25 or send.DIOValue == 26:  #60
                          send.sendBodySector(pick1)  #手打開 and 蹲下(左：負正正[1:2:1]  右：正負負[1:2:1])

                        elif send.DIOValue == 28 or send.DIOValue == 29 or send.DIOValue == 30:  #70
                          send.sendBodySector(pick2)

                        time.sleep(9.5)

                        # target_ymax,target_ymin,target_xmax,target_xmin=red_line()
                        # red_middle=round((target_xmax+target_xmin)/2)                        
                        # distance2=round(red_middle2-red_middle) #練習紅中 - 實際紅中
                        # if distance2>32:
                        #   distance=32
                        # elif distance2<-32:
                        #   distance=-32
                        # else :
                        #   distance=distance2
                        # time.sleep(0.5)
                        # print('fix=',distance)
                        # time.sleep(0.5)
                        '''
                        # 預測：調整腰 --> 左移 or 右移
                        if distance>0:
                          for d in range(distance):
                            #send.sendBodySector(32) #右移
                            time.sleep(0.2)
                            #print("蹲:左移")
                        if distance<0:  
                          for d in range(distance):
                            #send.sendBodySector(31) #左移
                            time.sleep(0.2)
                            #print("蹲:右移")
                         '''         
                        #time.sleep(0.4)    
                        arrive2 = True
                        print("==================")
                        print("arrive2 =", arrive2)

                    if arrive2 == True:  
                      print("----拿起----")
                      time.sleep(1)
                      if send.DIOValue == 24 or send.DIOValue == 25 or send.DIOValue == 26:  #60
                        send.sendBodySector(pick3) #手夾起 and 站起來(左:正負負[1:2:1]  右:負正正[1:2:1])
                        time.sleep(17)

                      elif send.DIOValue == 28 or send.DIOValue == 29 or send.DIOValue == 30:  #70
                        send.sendBodySector(pick4)
                        time.sleep(19)
                        send.sendBodySector(26)
                        time.sleep(0.5)
                      
                      
                      # print('fix=',distance)
                      # time.sleep(0.35)
                      '''
                      #相呼對應的左移 or 右移
                      if distance>0:
                        for d in range(distance):
                          #send.sendBodySector(31)
                          time.sleep(0.2)
                          #print("站:右移")
                      if distance<0:  
                        for d in range(distance):
                          #send.sendBodySector(32)
                          #time.sleep(0.2)
                          #print("站:左移")
                      '''
                      #time.sleep(0.4) 
                      #send.sendBodySector(pick3) #??
                      #time.sleep(2.2)  
                      #print("111", yaw) 
                      #yaw = afterbar() #reset imu
                      #print("222", yaw) 
                      #time.sleep(1.5)
                      send.sendHeadMotor(2, 2802, 50) 
                      
                      time.sleep(1)
                      pick_bar = True
                      print("====================")
                      print("pick_bar =", pick_bar)

                  if pick_bar == True:
                    turn_on()
                    #time.sleep(1)
                    print("------moving to liftline------")
                    imu_2()
                    time.sleep(0.5)
                    imu_2()

                    if tem == True:
                      imu_2()
                      time.sleep(5)
                      tem = False

                    white_ymax,white_ymin = white_line()
                    time.sleep(0.35)
                    imu_2()
                    print('while_ymax =', white_ymax)
                    

                    if white_ymax>180 and white_ymax<240:
                      print("-----Find While_line-----")
                      print("1")
                      print("2")
                      time.sleep(1)
                      print("3") 
                      # lift_line=True
                      #print("lift_line =", lift_line)

                      print('-----stop and lift-----')
                      turn_off()
                      time.sleep(1.5)
                      turn_off()
                      time.sleep(0.5)
                      print("----舉起----")
                      if send.DIOValue == 24 or send.DIOValue == 25 or send.DIOValue == 26:  #60 
                        send.sendBodySector(lift) #舉起
                        time.sleep(12)
                      elif send.DIOValue == 28 or send.DIOValue == 29 or send.DIOValue == 30:  #70
                        send.sendBodySector(lift2)
                        time.sleep(12)
                        
          
                      #yaw=afterbar() #reset yaw
                      #time.sleep(1.5)

                      lift_bar=True
                      print("====================")
                      print("lift_bar =", lift_bar)

                      print("imu_value_pitch :", send.imu_value_Pitch)
                      time.sleep(1.5)
                    else:
                      imu_2()
                      print("imu_2")

              # if lift_line==True:
              #   white_ymax,white_ymin=white_line()
              #   time.sleep(0.35)
              #   print('distance 2=',white_ymax)

              #   if white_ymax>180 and white_ymax<240:
              #     print('stop and lift')
              #     turn_off()
              #     time.sleep(2)
              #     turn_off()

              #     send.sendBodySector(lift) #舉起
              #     time.sleep(5.5)

              #     yaw=afterbar() #reset yaw
              #     time.sleep(1.5)

              #     lift_bar=True
              #     print("lift_bar =", lift_bar)

              #     print("imu_value_pitch :", send.imu_value_Pitch)
              #     time.sleep(1.5)
              #   else :                 
              #     imu_2()
            elif lift_bar == True:
              print('-----keep going to endline----')
              turn_on()
              time.sleep(0.5)
              imu_3()      
              #print('end')

          if send.is_start == False:
            turn_off()
            if stand: 
              send.sendBodySector(999)
              stand = False
            #print("AA")
            white_ymax,white_ymin = white_line()
            print('w=', white_ymax)
            
            print("red y_max =", send.color_mask_subject_YMax[6][0])
            print("red y_min =", send.color_mask_subject_YMin[6][0])
            target_ymax,target_ymin,target_xmax,target_xmin = red_line()
            red_middle = float(target_xmax+target_xmin)/2
            print('r=', red_middle)

            send.sendHeadMotor(2, 2746, 50)
            print()
            print("white x_max =", send.color_mask_subject_XMax[5][0])
            print("white x_min =", send.color_mask_subject_XMin[5][0])
            print()
            
          r.sleep()  
            
                     
    except rospy.ROSInterruptException:
        pass