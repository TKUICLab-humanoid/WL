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
yaw = 0
arrive = False

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
    return target_ymax ,target_xmin

def white_line():
    white_xmax = 0
    white_xmin = 0
    white_ymax = 0
    white_ymin = 0
    white_size = 0
    for white_cnt in range(send.color_mask_subject_cnts[6]): 
      white_line_wide=send.color_mask_subject_XMax[6][white_cnt]-send.color_mask_subject_XMin[6][white_cnt]  
      if send.color_mask_subject_size[6][white_cnt]>500 and white_line_wide<300 and send.color_mask_subject_YMin[6][white_cnt]<220:
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
    x=send.imu_value_Yaw
    print(x)
    if x>2: 
      send.sendContinuousValue(500,-450,0,-10,0)
      print("turn right")
      
    elif x<-2:
      send.sendContinuousValue(500,-450,0,-2,0)
      print("turn left")

    elif -2<=x and x<=2:
      send.sendContinuousValue(800,-450,0,-6,0)
      print("walk straight")

def imu1_5():
    x=send.imu_value_Yaw
    print(x)
    if x>2: 
      send.sendContinuousValue(500,-450,0,-10,0)
      print("turn right")
      
    elif x<-2 :
      send.sendContinuousValue(500,-450,0,-2,0)
      print("turn left")

    elif -2<=x and x<=2 :
      send.sendContinuousValue(500,0,0,-6,0)
      print("walk straight")

def imu_2():
    x=send.imu_value_Yaw
    x+=yaw
    print(x)
    if x>3: 
      send.sendContinuousValue(500,-450,0,-11,0)
      print("turn right 2")
      
    elif x<-3 :
      send.sendContinuousValue(500,-450,0,-1,0)
      print("turn left 2")

    elif -3<=x and x<=3 :
      send.sendContinuousValue(800,-450,0,-6,0)
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
          if send.is_start == True:
            if lift_bar==False:  
              if find_white_line==False:
                if pick_bar==False:
                  if arrive==False:
                    send.sendHeadMotor(2,1350,50)
                    print('move')
                    turn_on()
                    imu()
                    target_ymax,target_xmin=red_line()
                    print(target_ymax)
                    if target_ymax>90 and target_ymax<95:
                      imu1_5()
                      target_ymax,target_xmin=red_line()
                      print(target_ymax)
                    if target_ymax>95:
                      arrive=True
                  if arrive==True:
                    #target_ymax,target_xmin=red_line()
                    #if target_xmin>131:
                    #  send.sendContinuousValue(40,-500,0,0,0)
                    #  print("right")
                    #elif target_xmin<129:
                    #  send.sendContinuousValue(40,500,0,0,0)
                    #  print("left")
                    #else :
                      print("stop")                                     
                      turn_off()
                      print('pick up')
                      time.sleep(3)
                      send.sendBodySector(123)
                      time.sleep(27) 
                      print("aaaaaaaaaaaaaaaaaaaaa")
                      yaw=afterbar()
                      time.sleep(1.5) 
                      pick_bar=True
                if pick_bar==True:
                  #time.sleep(3)
                  #if white_ymax==0:
                  turn_on()
                  print("moving to liftline")
                  imu_2()
                  white_ymax,white_ymin=white_line()
                  print(white_ymax)
                  if 0<white_ymax and white_ymax<50:
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
                  print('ccccccc',white_ymin)
                  print("moving to liftline 2.0")
                  imu_2()
                  #time.sleep(1) 
                else :                 
                  print('stop and lift')
                  turn_off()
                  time.sleep(2.5)
                  send.sendBodySector(18)
                  time.sleep(2.5)
                  yaw=afterbar()
                  time.sleep(2)
                  lift_bar=True
            elif lift_bar==True:
              print('keep going to endline')
              turn_on()
              imu_2()      
              #time.sleep(1)
              print('end')
          if send.is_start == False:
            turn_off()
            
                     
    except rospy.ROSInterruptException:
        pass



