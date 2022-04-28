#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
from Python_API import Sendmessage
import time
find_white_line = False
pick_bar =False
lift_bar = False
Body_Auto = False
arrive = False
yaw = 0

x=1500
y=100
z=0
theta=-2

x2=1000
y2=100
z2=0
theta2=-2

x3=1000
y3=-0
z3=0
theta3=0

def turn_on():
    global Body_Auto
    if Body_Auto==True:
      pass
    elif Body_Auto==False:
      send.sendBodyAuto(0,0,0,0,1,0)
      Body_Auto=True

def turn_off():
    global Body_Auto
    if Body_Auto==True:
      send.sendBodyAuto(0,0,0,0,1,0)
      Body_Auto=False
    elif Body_Auto==False:
      pass

def red_line():
    target_xmax = 0
    target_xmin = 0
    target_ymax = 0
    target_ymin = 0
    target_size = 0
    for red_cnt in range(send.color_mask_subject_cnts[5]):    
      if send.color_mask_subject_size[5][red_cnt]>120:
        target_xmax = send.color_mask_subject_XMax[5][red_cnt]
        target_xmin = send.color_mask_subject_XMin[5][red_cnt]
        target_ymax = send.color_mask_subject_YMax[5][red_cnt]
        target_ymin = send.color_mask_subject_YMin[5][red_cnt]
        target_size = send.color_mask_subject_size[5][red_cnt]
        #print(target_xmax,target_xmin)
      print(target_ymax)
     #print(target_size)
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
      if send.color_mask_subject_size[6][white_cnt]>200 and white_line_wide<300 and send.color_mask_subject_YMax[6][white_cnt]>70:
        white_xmax = send.color_mask_subject_XMax[6][white_cnt]
        white_xmin = send.color_mask_subject_XMin[6][white_cnt]
        white_ymax = send.color_mask_subject_YMax[6][white_cnt]
        white_ymin = send.color_mask_subject_YMin[6][white_cnt]
        white_size = send.color_mask_subject_size[6][white_cnt]
     #print(target_xmax,target_xmin)
     #print(target_ymax,target_ymin)
     #print(target_size)
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
      send.sendContinuousValue(x,y,z,theta-4+fix,0)
      print("turn right")
      
    elif yaw_1<-2:
      send.sendContinuousValue(x,y,z,theta+4+fix,0)
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
      send.sendContinuousValue(x,y,z,theta-4+fix,0)
      print("turn right")
      
    elif yaw_1<-2:
      send.sendContinuousValue(x,y,z,theta+4+fix,0)
      print("turn left")

    elif -2<=yaw_1 and yaw_1<=2:
      send.sendContinuousValue(x,y,z,theta+fix,0)
      print("walk straight")

def imu_2():
    yaw_1=send.imu_value_Yaw
    yaw_1+=yaw
    print(yaw_1)
    if yaw_1>3: 
      send.sendContinuousValue(x2,y2,z2,theta2-5,0)
      print("turn right 2")
      
    elif yaw_1<-3 :
      send.sendContinuousValue(x2,y2,z2,theta2+5,0)
      print("turn left 2")

    elif -3<=yaw_1 and yaw_1<=3 :
      send.sendContinuousValue(x2,y2,z2,theta2,0)
      print("walk straight 2")

def imu_3():
    yaw_1=send.imu_value_Yaw
    yaw_1+=yaw
    print(yaw_1)
    if yaw_1>3: 
      send.sendContinuousValue(x3,y3,z3,theta3-5,0)
      print("turn right 2")
      
    elif yaw_1<-3 :
      send.sendContinuousValue(x3,y3,z3,theta3+5,0)
      print("turn left 2")

    elif -3<=yaw_1 and yaw_1<=3 :
      send.sendContinuousValue(x3,y3,z3,theta3,0)
      print("walk straight 2")


def afterbar():
    print('revise')
    yaw=send.imu_value_Yaw
    time.sleep(0.35)
    send.sendSensorReset()
    return yaw



  
  


imgdata= [[None for high in range(240)]for width in range(320)]
if __name__ == '__main__':
    send = Sendmessage()

    try:
        #find_white_line=True
        #lift=True
        #pick_bar = True
        time.sleep(0.35)
        send.sendSensorReset()
        while not rospy.is_shutdown():
          send.sendHeadMotor(2,1350,50)
          if send.is_start == True:
            if lift_bar==False:  
              if find_white_line==False:
                if pick_bar==False:
                  if arrive==False:
                    send.sendHeadMotor(2,1350,50)
                    print('move')
                    turn_on()
                    imu()
                    target_ymax,target_ymin,target_xmax,target_xmin=red_line()
                    print(target_ymax)
                    if target_ymax>80 and target_ymax<90:
                      imu1_5()
                      target_ymax ,target_ymin ,target_xmax ,target_xmin=red_line()
                      print(target_ymax)
                    if target_ymax>=90:
                      arrive=True
                  if arrive==True:
                      print("stop")                                     
                      turn_off()
                      print('pick up')
                      time.sleep(3)
                      send.sendBodySector(123)
                      time.sleep(8) 
                      yaw=afterbar()
                      time.sleep(1.5)
                      send.sendHeadMotor(2,1250,50) 
                      time.sleep(1)
                      pick_bar=True
                      
                if pick_bar==True:
                  send.sendHeadMotor(2,1150,50)
                  time.sleep(1) 
                  turn_on()
                  print("moving to liftline")
                  imu_2()
                  white_ymax,white_ymin=white_line()
                  print('aaabbb',white_ymin)
                  if white_ymin >180:
                    print("1")
                    send.sendHeadMotor(2,1250,50)
                    print("2")
                    time.sleep(1)
                    print("3") 
                    find_white_line=True
                  else:
                    imu_2()
              if find_white_line==True: 
                imu_2()
                white_ymax,white_ymin=white_line()
                print('aaaaaa',white_ymin)
                if white_ymin<210:
                  print('bbbbbbbb',white_ymin)
                  white_ymax,white_ymin=white_line()
                  print("moving to liftline 2.0")
                  imu_2()
                  #time.sleep(1) 
                else :                 
                  print('stop and lift')
                  turn_off()
                  time.sleep(2.5)
                  send.sendBodySector(456)
                  time.sleep(3)
                  yaw=afterbar()
                  time.sleep(2)
                  lift_bar=True
                  print(send.imu_value_Pitch)
                  time.sleep(2)
            elif lift_bar==True:
              print('keep going to endline')
              turn_on()
              imu_3()      
              #time.sleep(1)
              print('end')
          if send.is_start == False:
            turn_off()
            
                     
    except rospy.ROSInterruptException:
        pass
