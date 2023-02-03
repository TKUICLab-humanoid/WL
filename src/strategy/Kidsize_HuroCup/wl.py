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
# imu   imu1_5   imu_2
x=1500
y=-100
z=0
theta=0

# /////Only use theta2, used at imu_2/////
x2=1500
y2=-600
z2=0
theta2=0

# imu_3
x3=1500
y3=-100
z3=0  
theta3=0
# 以下變數用於機器人在 "拾起線" 時的左右移動，用於 "correct==true" 
xl=-250
yl=300  #100
zl=0
tl=4

xr=-250
yr=-500 #-500
zr=0
tr=-4
# 理想中間值，用於 "correct==true" 區域，與上方 xl, yl, ..., xr, yr, ...等等做搭配
target_left=161
target_right=165
# 理想中間值，當機器人抓到槓鈴，此變數用於判斷是否執行磁區 31or32 進行微調
red_middle2=162.5

# 停下/判斷距離設定，用於 拾起線 距離區間停下判斷
pickup_distance1=149  # 此數值應小於 pickup_distance2
pickup_distance2=160  # 停下數值改這個.126

# 判斷距離設定，用於 舉起線 距離區間停下判斷
liftup_distance1=70   # 此數值應小於 liftup_distance2
liftup_distance2=120  # 這兩個數值進行第一階段判斷，判斷成功後(lift_line=True)進行第二階段

liftup_distance3=5    # 距離在此區間便停下；此數值應小於 liftup_distance4
liftup_distance4=40   # 

# 頭部馬達角度設定
head_motor_angle1=1456    # 初始位置
head_motor_angle2=1337    # 最一開始移動後的位置
head_motor_angle3=1377    # 拾起槓鈴後的位置
# 磁區變數設定
pick1=7411
pick2=7412
pick3=7413

lift=742



def turn_on():
    global Body_Auto
    # time.sleep(0.8)
    print("o===",Body_Auto)
    if Body_Auto==True:
      pass
    elif Body_Auto==False:
      time.sleep(0.7)
      send.sendSensorReset()
      send.sendBodyAuto(200,0,0,0,1,0)
      Body_Auto=True

def turn_off():
    global Body_Auto
    # time.sleep(0.8)
    print("F===",Body_Auto)
    if Body_Auto==True:
      send.sendSensorReset()
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
      if send.color_mask_subject_size[5][red_cnt]>300:
        target_xmax = send.color_mask_subject_XMax[5][red_cnt]
        target_xmin = send.color_mask_subject_XMin[5][red_cnt]
        target_ymax = send.color_mask_subject_YMax[5][red_cnt]
        target_ymin = send.color_mask_subject_YMin[5][red_cnt]
        target_size = send.color_mask_subject_size[5][red_cnt]
        #print(target_xmax,target_xmin)
      print("target_ymax= ",target_ymax)
      print("target_size= ",target_size)
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
    print("yaw_1= ",yaw_1)
    if yaw_1>2: 
      send.sendContinuousValue(x,y,z,theta-3+fix,0) #theta-2
      print("turn right")
      
    elif yaw_1<-2:
      send.sendContinuousValue(x,y,z,theta+3+fix,0) #theta+2
      print("turn left")

    elif -2<=yaw_1 and yaw_1<=2:
      send.sendContinuousValue(x,y,z,theta+fix,0)   #theta+1 or-1
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
    print("yaw_1= ",yaw_1)
    if yaw_1>2: 
      send.sendContinuousValue(x,y,z,theta-3+fix,0)
      print("turn right")
      
    elif yaw_1<-2:
      send.sendContinuousValue(x,y,z,theta+3+fix,0)
      print("turn left")

    elif -2<=yaw_1 and yaw_1<=2:
      send.sendContinuousValue(x,y,z,theta+fix,0)
      print("walk straight")

def imu_2():
    yaw_1=send.imu_value_Yaw
    #print("cccccc",yaw_1) 
    #time.sleep(0.5)
    yaw_1+=yaw
    print("yaw_1= ",yaw_1)
    if yaw_1>2: 
      send.sendContinuousValue(x,y,z,theta2-1,0)
      print("turn right 2")
      
    elif yaw_1<-2 :
      send.sendContinuousValue(x,y,z,theta2+1,0)
      print("turn left 2")

    elif -2<=yaw_1 and yaw_1<=2 :
      send.sendContinuousValue(x,y,z,theta2,0)
      print("walk straight 2")
    #print("gggggg",theta)  

def imu_3():
    yaw_1=send.imu_value_Yaw
    yaw_1+=yaw
    print("yaw_1= ",yaw_1)
    if yaw_1>3: 
      send.sendContinuousValue(x3,y3,z3,theta3-1,0)
      print("turn right 3")

def afterbar():
    print('revise')
    yaw=send.imu_value_Yaw
    time.sleep(0.35)
    send.sendSensorReset()
    return yaw
      
# /////No use/////
    return arrive
  
def print_section():
    print("=========================")
    print("New Stage")
    print("=========================")
    
imgdata= [[None for high in range(240)]for width in range(320)]
if __name__ == '__main__':
    send = Sendmessage()

    try:
        time.sleep(0.35)
        send.sendSensorReset()
        send.sendHeadMotor(2,head_motor_angle1,50)
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
                            # send.sendBodySector(666)
                            # time.sleep(2)
                            send.sendSensorReset()
                            imu_reset=True
                          if imu_reset==True:
                            send.sendHeadMotor(2,head_motor_angle2,50)
                            print('前前前前前前前前前進進進進進進進進進')
                            send.sendSensorReset()
                            # turn_on()
                            imu()
                            target_ymax,target_ymin,target_xmax,target_xmin=red_line()
                            print('第262行')
                            print("target_xmax= ",target_xmax)
                            print("target_xmin= ",target_xmin)
                            print("target_ymax= ",target_ymax)
                            print("target_ymin= ",target_ymin,end='\n')
                            if target_ymax>pickup_distance1 and target_ymax<pickup_distance2:
                              imu1_5()
                              target_ymax ,target_ymin ,target_xmax ,target_xmin=red_line()
                              print('第270行')
                              print("target_xmax= ",target_xmax)
                              print("target_xmin= ",target_xmin)
                              print("target_ymax= ",target_ymax)
                              print("target_ymin= ",target_ymin,end='\n')
                            if target_ymax>=pickup_distance2:
                              correct=True
                        if correct==True:
                          print('開始微調',end='\n')
                          target_ymax,target_ymin,target_xmax,target_xmin=red_line()
                          print('第280行')
                          print("target_xmax= ",target_xmax)
                          print("target_xmin= ",target_xmin)
                          print("target_ymax= ",target_ymax)
                          print("target_ymin= ",target_ymin,end='\n')
                          red_middle=float(target_xmax+target_xmin)/2
                          print('紅色中心點= ',red_middle)
                          if red_middle<target_left:
                            send.sendContinuousValue(xl,yl,zl,tl,0)
                            print('左左左左左左左左左左左左左左左左左左左')
                          if red_middle>target_right:
                            send.sendContinuousValue(xr,yr,zr,tr,0)
                            print('右右右右右右右右右右右右右右右右右右右')
                          if red_middle>target_left and red_middle<target_right:
                            arrive=True
                            print('微調結束')
                      if arrive==True:                                     
                      #  turn_off()
                       time.sleep(0.5)
                       print('down')
                       time.sleep(0.5)
                       print('我要抬手囉!!!!!!')
                      #  send.sendBodySector(pick1)
                       time.sleep(6)
                       target_ymax,target_ymin,target_xmax,target_xmin=red_line()
                       print('第305行')
                       print("target_xmax= ",target_xmax)
                       print("target_xmin= ",target_xmin)
                       print("target_ymax= ",target_ymax)
                       print("target_ymin= ",target_ymin,end='\n')
                       red_middle=round((target_xmax+target_xmin)/2)                        
                       distance2=round(red_middle2-red_middle)
                       if distance2>32:
                         distance=32
                       elif distance2<-32:
                         distance=-32
                       else :
                         distance=distance2
                       time.sleep(0.5)
                       print('修正= ',distance)
                       time.sleep(0.5)
                       if distance>0:
                         for d in range(distance):
                          #  send.sendBodySector(32)
                           time.sleep(0.2)
                       if distance<0:  
                         for d in range(distance):
                          #  send.sendBodySector(31)
                           time.sleep(0.2)
                       time.sleep(0.4)    
                       arrive2=True
                       print('我要撿起來囉!!!!!!!!!')
                    if arrive2==True:  
                      print('我撿撿撿撿撿撿撿撿撿撿')
                      time.sleep(1.5)
                      # send.sendBodySector(pick2)
                      time.sleep(5.5)
                      print('修正= ',distance)
                      time.sleep(0.35)
                      if distance>0:
                        for d in range(distance):
                          # send.sendBodySector(31)
                          time.sleep(0.2)
                      if distance<0:  
                        for d in range(distance):
                          # send.sendBodySector(32)
                          time.sleep(0.2)
                      time.sleep(0.4) 
                      # send.sendBodySector(pick3)
                      time.sleep(3.5)  
                      print("111 yaw before afterbar= ",yaw) 
                      yaw=afterbar()
                      print("222 yaw after afterbar= ",yaw)
                      time.sleep(1.5)
                      send.sendHeadMotor(2,head_motor_angle3,50) 
                      time.sleep(1)
                      pick_bar=True     
                      print('舉起線在哪!?')                 
                  if pick_bar==True:
                    send.sendSensorReset()
                    # turn_on()
                    print_section()
                    time.sleep(1)
                    print("舉起線我來了!!!")
                    white_ymax,white_ymin=white_line()
                    time.sleep(0.35)
                    imu_2()
                    print('find white_ymax= ',white_ymax)
                    if white_ymax>liftup_distance1 and white_ymax<liftup_distance2:
                      time.sleep(1)
                      print("舉起線要到了") 
                      lift_line=True
                    else:
                      print('imu_2微調')
                      imu_2()
              if lift_line==True:
                white_ymax,white_ymin=white_line()
                print('distance 2=',white_ymax)
                if white_ymax>liftup_distance3 and white_ymax<liftup_distance4:
                  print('======停下，舉起======')
                  # turn_off()
                  time.sleep(2.5)
                  # turn_off()
                  # send.sendBodySector(lift)
                  time.sleep(6)
                  yaw=afterbar()
                  time.sleep(1.5)
                  lift_bar=True
                  print("imu_value_Pitch= ",send.imu_value_Pitch)
                  time.sleep(1.5)
                else :          
                  print('imu_2微調')       
                  imu_2()
            elif lift_bar==True:
              print_section()
              print('======終點我來了======')
              send.sendSensorReset()
              # turn_on()
              time.sleep(0.5)
              imu_3()      
              print('耶~~~~~到了~~~~~')
          if send.is_start == False:
            # turn_off()
            print("草尼馬,Ready to go!")
            white_ymax,white_ymin=white_line()
            print("white_ymax= ",white_ymax)
            target_ymax,target_ymin,target_xmax,target_xmin=red_line()
            red_middle=float(target_xmax+target_xmin)/2
            print("red_middle= ",red_middle)
          r.sleep()  
            
                     
    except rospy.ROSInterruptException:
        pass
