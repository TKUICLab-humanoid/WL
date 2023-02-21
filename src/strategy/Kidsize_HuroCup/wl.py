#!/usr/bin/env python
#coding=utf-8
from xml.etree.ElementTree import XML
import rospy
import numpy as np
from Python_API import Sendmessage
import time
find_white_line = False
pick_bar =False #是否夾起槓桿
lift_bar = False #是否舉起槓桿
Body_Auto = False
arrive = False #是否站在槓桿中間
arrive2 = False #是否蹲下
imu_reset = False
lift_line = False #是否抵達 lift line
correct = False #是否站在賽道中間
yaw = 0

x=1000
y=0
z=0
theta=1

x2=3000
y2=0
z2=0
theta2=0

x3=3000
y3=0
z3=0  
theta3=0

xl=-1000
yl=700  
zl=0
tl=3

xr=-1000
yr=-500
zr=0
tr=1

target_left=162
target_right=165

red_middle2=166.5

pick1=110
pick2=111
pick3=8413

lift=112


def turn_on():
    global Body_Auto
    # time.sleep(0.8)
    print("o===",Body_Auto)
    if Body_Auto==True:
      pass
    elif Body_Auto==False:
      time.sleep(0.7)
      #send.sendSensorReset() ==========================================
      send.sendBodyAuto(200,0,0,0,1,0)
      Body_Auto=True

def turn_off():
    global Body_Auto
    # time.sleep(0.8)
    print("F===",Body_Auto)
    if Body_Auto==True:
      send.sendBodyAuto(200,0,0,0,1,0)
      Body_Auto=False
    elif Body_Auto==False:
      pass

def red_line():
    target_xmax = 0
    target_xmin = 0
    target_ymax = 0
    target_ymin = 0
    target_size = 0
    for red_cnt in range(send.color_mask_subject_cnts[2]):  #  send.color_mask_subject_cnts[5] is value about red range
      if send.color_mask_subject_size[2][red_cnt]>300:
        target_xmax = send.color_mask_subject_XMax[2][red_cnt]
        target_xmin = send.color_mask_subject_XMin[2][red_cnt]
        target_ymax = send.color_mask_subject_YMax[2][red_cnt]
        target_ymin = send.color_mask_subject_YMin[2][red_cnt]
        target_size = send.color_mask_subject_size[2][red_cnt]
        #print(target_xmax,target_xmin)
      print(target_ymax)
      print(target_size)
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
      if send.color_mask_subject_size[6][white_cnt]>500 and send.color_mask_subject_YMax[6][white_cnt]<160:
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
      fix=-1
    else :
      fix=0
    yaw_1=send.imu_value_Yaw
    print(yaw_1)
    if yaw_1>2: 
      send.sendContinuousValue(x,y,z,theta+fix,0)
      print("turn right")
      
    elif yaw_1<-2:
      send.sendContinuousValue(x,y,z,theta+fix,0)
      print("turn left")

    elif -2<=yaw_1 and yaw_1<=2:
      send.sendContinuousValue(x,y,z,theta+fix,0)
      print("walk straight")

def imu1_5():
    target_ymax,target_ymin,target_xmax,target_xmin=red_line()
    target_middle=(target_xmax+target_xmin)/2
    if target_middle>160:
      fix=1
    if target_middle<160:
      fix=-1
    else :
      fix=0
    yaw_1=send.imu_value_Yaw
    print(yaw_1)
    if yaw_1>2: 
      send.sendContinuousValue(x,y,z,theta+fix,0)
      print("turn right")
      
    elif yaw_1<-2:
      send.sendContinuousValue(x,y,z,theta+fix,0)
      print("turn left")

    elif -2<=yaw_1 and yaw_1<=2:
      send.sendContinuousValue(x,y,z,theta+fix,0)
      print("walk straight")

def imu_2():
    yaw_1=send.imu_value_Yaw
    #print("cccccc",yaw_1) 
    #time.sleep(0.5)
    yaw_1+=yaw
    print(yaw_1)
    if yaw_1>2: 
      send.sendContinuousValue(x2,y,z,theta2,0)
      print("turn right 2")
      
    elif yaw_1<-2 :
      send.sendContinuousValue(x2,y,z,theta2,0)
      print("turn left 2")

    elif -2<=yaw_1 and yaw_1<=2 :
      send.sendContinuousValue(x2,y,z,theta2,0)
      print("walk straight 2")
    #print("gggggg",theta)  

def imu_3():
    yaw_1=send.imu_value_Yaw
    yaw_1+=yaw
    print(yaw_1)
    if yaw_1>3: 
      send.sendContinuousValue(x2,y3,z3,theta3,0)
      print("turn right 3")
      
    elif yaw_1<-3 :
      send.sendContinuousValue(x2,y3,z3,theta3,0)
      print("turn left 3")

    elif -3<=yaw_1 and yaw_1<=3 :
      send.sendContinuousValue(x2,y3,z3,theta3,0)
      print("walk straight 3")


def afterbar():
    print('revise')
    yaw=send.imu_value_Yaw
    time.sleep(0.35)
    send.sendSensorReset()
    return yaw

def correct_target():
    target_ymax,target_ymin,target_xmax,target_xmin=red_line()
    red_middle=float(target_xmax+target_xmin)/2
    print('middle=',red_middle)
    if red_middle>172:
      send.sendContinuousValue(0,-500,0,0,0)
      print('move right')
    if red_middle<157:
      send.sendContinuousValue(0,500,0,0,0)
      print('move left')
    if red_middle>157 and red_middle<172:
      arrive=True
    return arrive
  
  


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
            print(Body_Auto)
            if lift_bar==False: 
              if lift_line==False: 
                #if find_white_line==False:
                  if pick_bar==False:
                    if arrive2==False:
                      if arrive==False:
                        if correct==False:

                          if imu_reset==False:
                            #send.sendBodySector(666) #起始動作
                            time.sleep(2)
                            send.sendSensorReset()
                            imu_reset=True
                            print("imu_reset:", imu_reset)

                          if imu_reset==True:
                            send.sendHeadMotor(2,2746,50)
                            print('move')
                            turn_on()
                            imu()
                            target_ymax,target_ymin,target_xmax,target_xmin=red_line()
                            print("target_ymax_1:", target_ymax)
                            if target_ymax>115 and target_ymax<128:
                              imu1_5()
                              target_ymax ,target_ymin ,target_xmax ,target_xmin=red_line()
                              print("target_ymax_2:", target_ymax)
                            if target_ymax>=190:
                              correct=True
                              print("correct:", correct)

                        if correct==True:
                          target_ymax,target_ymin,target_xmax,target_xmin=red_line()
                          red_middle=float(target_xmax+target_xmin)/2
                          print('middle=',red_middle)
                          if red_middle<target_left:
                            send.sendContinuousValue(xl,yl,zl,tl,0)
                            print('move left')
                          if red_middle>target_right:
                            send.sendContinuousValue(xr,yr,zr,tr,0)
                            print('move right')
                          if red_middle>target_left and red_middle<target_right:
                            arrive=True
                            print("arrive:", arrive)

                      if arrive==True:
                        print("stop")                                     
                        turn_off()
                        time.sleep(0.5)
                        print('down')
                        time.sleep(1)
                        send.sendBodySector(pick1)  #手打開 and 蹲下(左：負正正[1:2:1]  右：正負負[1:2:1])
                        time.sleep(20)

                        target_ymax,target_ymin,target_xmax,target_xmin=red_line()
                        red_middle=round((target_xmax+target_xmin)/2)                        
                        distance2=round(red_middle2-red_middle) #練習紅中 - 實際紅中
                        if distance2>32:
                          distance=32
                        elif distance2<-32:
                          distance=-32
                        else :
                          distance=distance2
                        time.sleep(0.5)
                        print('fix=',distance)
                        time.sleep(0.5)

                        # 預測：調整腰 --> 左移 or 右移
                        if distance>0:
                          for d in range(distance):
                            #send.sendBodySector(32) #右移
                            time.sleep(0.2)
                            print("蹲:左移")
                        if distance<0:  
                          for d in range(distance):
                            #send.sendBodySector(31) #左移
                            time.sleep(0.2)
                            print("蹲:右移")
                        
                        time.sleep(0.4)    
                        arrive2=True
                        print("arrive2 =", arrive2)

                    if arrive2==True:  
                      print('pick up')
                      time.sleep(1.5)
                      send.sendBodySector(pick2) #手夾起 and 站起來(左:正負負[1:2:1]  右:負正正[1:2:1])
                      time.sleep(13)
                      print('fix=',distance)
                      time.sleep(0.35)

                      #相呼對應的左移 or 右移
                      if distance>0:
                        for d in range(distance):
                          #send.sendBodySector(31)
                          time.sleep(0.2)
                          print("站:右移")
                      if distance<0:  
                        for d in range(distance):
                          #send.sendBodySector(32)
                          time.sleep(0.2)
                          print("站:左移")

                      time.sleep(0.4) 
                      #send.sendBodySector(pick3) #??
                      time.sleep(2.2)  
                      print("111",yaw) 
                      yaw = afterbar() #reset imu
                      print("222",yaw) 
                      time.sleep(1.5)
                      send.sendHeadMotor(2,2704,50) 
                      
                      time.sleep(1)
                      pick_bar = True
                      print("pick_bar =", pick_bar)

                  if pick_bar==True:
                    turn_on()
                    time.sleep(1)
                    print("moving to liftline")

                    white_ymax,white_ymin=white_line()
                    time.sleep(0.35)
                    imu_2()
                    print('find',white_ymax)

                    if white_ymax>70 and white_ymax<120:
                      print("1")
                      print("2")
                      time.sleep(1)
                      print("3") 
                      lift_line=True
                      print("lift_line =", lift_line)
                    else:
                      imu_2()
                      print("imu_2")

              if lift_line==True:
                white_ymax,white_ymin=white_line()
                print('distance 2=',white_ymax)

                if white_ymax>5 and white_ymax<40:
                  print('stop and lift')
                  turn_off()
                  time.sleep(2.5)
                  turn_off()

                  send.sendBodySector(lift) #舉起
                  time.sleep(10)

                  yaw=afterbar() #reset yaw
                  time.sleep(1.5)

                  lift_bar=True
                  print("lift_bar =", lift_bar)

                  print("imu_value_pitch :", send.imu_value_Pitch)
                  time.sleep(1.5)
                else :                 
                  imu_2()
            elif lift_bar==True:
              print('keep going to endline')
              turn_on()
              time.sleep(0.5)
              imu_3()      
              print('end')

          if send.is_start == False:
            turn_off()
            send.sendBodySector(666)
            print("AA")
            white_ymax,white_ymin=white_line()
            print('w=',white_ymax)

            print("y_max =", send.color_mask_subject_YMax[2][0])
            target_ymax,target_ymin,target_xmax,target_xmin = red_line()
            red_middle = float(target_xmax+target_xmin)/2
            print('r=', red_middle)
          r.sleep()  
            
                     
    except rospy.ROSInterruptException:
        pass
