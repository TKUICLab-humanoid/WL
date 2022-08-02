#!/usr/bin/env python
#coding=utf-8
from xml.etree.ElementTree import XML
import rospy
import numpy as np
from Python_API import Sendmessage
import time
find_white_line = False
pick_bar =False
lift_bar = False
Body_Auto = False
arrive = False
arrive2 = False
imu_reset = False
lift_line = False
correct = False
yaw = 0

x=1500
y=-100
z=0
theta=4

x2=1500
y2=-100
z2=0
theta2=3

x3=1500  
y3=-200
z3=0  
theta3=3

xl=-200
yl=600  
zl=0
tl=5

xr=-300
yr=-800
zr=0
tr=4

target_left=158
target_right=161

red_middle2=159.5

pick1=6411
pick2=6412
pick3=6413

lift=642


def turn_on():
    global Body_Auto
    # time.sleep(0.8)
    print("o===",Body_Auto)
    if Body_Auto==True:
      pass
    elif Body_Auto==False:
      time.sleep(0.7)
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
    #print("cccccc",yaw_1) 
    #time.sleep(0.5)
    yaw_1+=yaw
    print(yaw_1)
    if yaw_1>2: 
      send.sendContinuousValue(x,y,z,theta2-3,0)
      print("turn right 2")
      
    elif yaw_1<-2 :
      send.sendContinuousValue(x,y,z,theta2+3,0)
      print("turn left 2")

    elif -2<=yaw_1 and yaw_1<=2 :
      send.sendContinuousValue(x,y,z,theta2,0)
      print("walk straight 2")
    #print("gggggg",theta)  

def imu_3():
    yaw_1=send.imu_value_Yaw
    yaw_1+=yaw
    print(yaw_1)
    if yaw_1>3: 
      send.sendContinuousValue(x3,y3,z3,theta3-4,0)
      print("turn right 3")
      
    elif yaw_1<-3 :
      send.sendContinuousValue(x3,y3,z3,theta3+4,0)
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
        send.sendHeadMotor(2,1450,50)
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
                            send.sendSensorReset()
                            imu_reset=True
                          if imu_reset==True:
                            send.sendHeadMotor(2,1350,50)
                            print('move')
                            turn_on()
                            imu()
                            target_ymax,target_ymin,target_xmax,target_xmin=red_line()
                            print(target_ymax)
                            if target_ymax>115 and target_ymax<123:
                              imu1_5()
                              target_ymax ,target_ymin ,target_xmax ,target_xmin=red_line()
                              print(target_ymax)
                            if target_ymax>=123:
                              correct=True
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
                      if arrive==True:
                       print("stop")                                     
                       turn_off()
                       time.sleep(0.5)
                       print('down')
                       time.sleep(0.35)
                       send.sendBodySector(pick1)
                       time.sleep(4)
                       target_ymax,target_ymin,target_xmax,target_xmin=red_line()
                       red_middle=round((target_xmax+target_xmin)/2)                        
                       distance2=round(red_middle2-red_middle)
                       if distance2>32:
                         distance=32
                       else :
                         distance=distance2
                       time.sleep(0.5)
                       print('fix=',distance)
                       time.sleep(0.5)
                       if distance>0:
                         for d in range(distance):
                           send.sendBodySector(32)
                           time.sleep(0.2)
                       if distance<0:  
                         for d in range(distance):
                           send.sendBodySector(31)
                           time.sleep(0.2)
                       time.sleep(0.4)    
                       arrive2=True
                    if arrive2==True:  
                      print('pick up')
                      time.sleep(1.5)
                      send.sendBodySector(pick2)
                      time.sleep(3)
                      print('fix=',distance)
                      time.sleep(0.35)
                      if distance>0:
                        for d in range(distance):
                          send.sendBodySector(31)
                          time.sleep(0.2)
                      if distance<0:  
                        for d in range(distance):
                          send.sendBodySector(32)
                          time.sleep(0.2)
                      time.sleep(0.4) 
                      send.sendBodySector(pick3)
                      time.sleep(2.2)  
                      print("111",yaw) 
                      yaw=afterbar()
                      print("222",yaw) 
                      time.sleep(1.5)
                      send.sendHeadMotor(2,1300,50) 
                      time.sleep(1)
                      pick_bar=True                      
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
                    else:
                      imu_2()
              if lift_line==True:
                white_ymax,white_ymin=white_line()
                print('distance 2=',white_ymax)
                if white_ymax>5 and white_ymax<40:
                  print('stop and lift')
                  turn_off()
                  time.sleep(2.5)
                  turn_off()
                  send.sendBodySector(lift)
                  time.sleep(8.3)
                  yaw=afterbar()
                  time.sleep(1.5)
                  lift_bar=True
                  print(send.imu_value_Pitch)
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
            print("AA")
            white_ymax,white_ymin=white_line()
            print('w=',white_ymax)
            target_ymax,target_ymin,target_xmax,target_xmin=red_line()
            red_middle=float(target_xmax+target_xmin)/2
            print('r=',red_middle)
          r.sleep()  
            
                     
    except rospy.ROSInterruptException:
        pass
