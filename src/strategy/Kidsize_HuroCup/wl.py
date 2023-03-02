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
arrive2 = False
imu_reset = False
lift_line = False
correct = False
yaw = 0

# 原地步態數值
X_origin=0
Y_origin=-100
Theta_origin=-1


Theta_fix=0         # 用於imu修正，進入判斷式才給值

Theta_LfixValue=4   # 用於imu修正，進入判斷式將此值丟給Theta_fix
Theta_RfixValue=-1  # 用於imu修正，進入判斷式將此值丟給Theta_fix

# 前進，左右踏，左右旋調整   imu
X_forward=1800
X_backward=-800
Y_left=500
Y_right=-500

Theta_trunleft=4    # 只用於原地左轉
Theta_trunright=-4  # 只用於原地右轉

# 原地踏步左右轉數值
xStand=X_origin
xBack=X_origin+X_backward
yStand=Y_origin
LeftStand=Theta_origin+Theta_trunleft
RightStand=Theta_origin+Theta_trunright

# imu   imu1_5   imu_2
x=X_origin+X_forward
y=Y_origin
theta=Theta_origin

# /////Only use theta2, used at imu_2/////
x2=X_origin+X_forward
y2=Y_origin
theta2=Theta_origin

# imu_3
x3=X_origin+X_forward
y3=Y_origin
theta3=Theta_origin


# 以下變數用於機器人在 "拾起線" 時的左右移動，用於 "correct==true" 
xl=X_origin
yl=Y_origin+Y_left
tl=Theta_origin

xr=X_origin
yr=Y_origin+Y_right
tr=Theta_origin


# 理想中間值，用於 "correct==true" 區域，與上方 xl, yl, ..., xr, yr, ...等等做搭配
target_left=161
target_right=165
# 理想中間值，當機器人抓到槓鈴，此變數用於判斷是否執行磁區 31or32 進行微調
red_middle2=162.5

# 停下/判斷距離設定，用於 拾起線 距離區間停下判斷
pickup_distance1=179  # 此數值應小於 pickup_distance2
pickup_distance2=180  # 停下數值改這個.126

pickup_distance3=210  # 
pickup_distance4=230  # 太近數值判定，用於第二階段原地左右旋轉

# 判斷距離設定，用於 舉起線 距離區間停下判斷
liftup_distance1=85   # 此數值應小於 liftup_distance2
liftup_distance2=190  # 這兩個數值進行第一階段判斷，判斷成功後(lift_line=True)進行第二階段

liftup_distance3=205    # 距離在此區間便停下；此數值應小於 liftup_distance4
liftup_distance4=215  # 

# 頭部馬達角度設定
head_motor_angle1=1456    # 初始位置1456
head_motor_angle2=1337    # 最一開始移動後的位置
head_motor_angle3=1270    # 拾起槓鈴後的位置
head_motor_angle4=1233    # 舉起前低頭
# 磁區變數設定
pick1=7411
pick2=7412
pick3=7413

#pick4=7414

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
    # whiteYMin_one=0
    # whiteYMax_one=0
    # whiteYMin_two=0
    # whiteYMax_two=0
    # if send.color_mask_subject_size[6][0]>500:
    #   if send.color_mask_subject_cnts[6] == 1:
    #     print("我只看到一一一一個啦啦啦")
    #     whiteYMin_one = send.color_mask_subject_XMin[6][0]
    #     whiteYMax_one = send.color_mask_subject_XMax[6][0]
    #   elif send.color_mask_subject_cnts[6] == 2:
    #     print("我看到兩兩兩兩個囉")
    #     whiteYMin_one = send.color_mask_subject_XMin[6][0]
    #     whiteYMax_one = send.color_mask_subject_XMax[6][0]
    #     whiteYMin_two = send.color_mask_subject_XMin[6][1]
    #     whiteYMax_two = send.color_mask_subject_XMax[6][1]
    white_xmax = 0
    white_xmin = 0
    white_ymax = 0
    white_ymin = 0
    white_size = 0
    for white_cnt in range(send.color_mask_subject_cnts[6]): 
      white_line_wide=send.color_mask_subject_XMax[6][white_cnt]-send.color_mask_subject_XMin[6][white_cnt]  
      if send.color_mask_subject_size[6][white_cnt]>500: #and send.color_mask_subject_YMax[6][white_cnt]<200
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
    return white_xmax,white_xmin,white_ymax,white_ymin

def imu():  #一開始走路在用的，還不在紅模範圍
    target_ymax,target_ymin,target_xmax,target_xmin=red_line()
    target_xmiddle=(target_xmax+target_xmin)/2
    target_ymiddle=(target_ymax+target_ymin)/2

    if target_xmiddle>160:
      fix=1
    if target_xmiddle<160:
      fix=-1
    else:
      fix=0
    yaw_1=send.imu_value_Yaw
    print("yaw_1= ",yaw_1)
    print("target_xmiddle=",target_xmiddle)

    if yaw_1>2 : 
      Theta_fix=Theta_RfixValue
      send.sendContinuousValue(x,y,0,theta+Theta_fix+fix,0) #theta-2
      print(" ONE 右轉")
      
    elif yaw_1<-2 :
      Theta_fix=Theta_LfixValue
      send.sendContinuousValue(x,y,0,theta+Theta_fix+fix,0) #theta+2
      print(" ONE 左轉")

    elif -2<=yaw_1 and yaw_1<=2:
      send.sendContinuousValue(x,y,0,theta+fix,0)   #theta+1 or-1
      print(" ONE 直走")

def imu1_5(): #已經靠近紅模，還沒到停下的程度
    target_ymax,target_ymin,target_xmax,target_xmin=red_line()
    target_xmiddle=(target_xmax+target_xmin)/2
    target_ymiddle=(target_ymax+target_ymin)/2
    if target_xmiddle>160:
      fix=1
    if target_xmiddle<160:
      fix=-1
    else :
      fix=0
    yaw_1=send.imu_value_Yaw
    print("yaw_1= ",yaw_1)
    if yaw_1>2 : 
      Theta_fix=Theta_RfixValue
      send.sendContinuousValue(x,y,0,theta+Theta_fix+fix,0)
      print(" ONE FIVE 右轉")
      
    elif yaw_1<-2 :
      Theta_fix=Theta_LfixValue
      send.sendContinuousValue(x,y,0,theta+Theta_fix+fix,0)
      print(" ONE FIVE 左轉")

    elif -2<=yaw_1 and yaw_1<=2:
      send.sendContinuousValue(x,y,0,theta+fix,0)
      print(" ONE FIVE 直走")


def imu_2():  #拾起線到舉起線在用的
    white_xmax ,white_xmin,white_ymax,white_ymin=white_line()
    # white_ymiddle=(white_ymax+whiteYMin_one)/2

    yaw_1=send.imu_value_Yaw
    print("yaw_1= ",yaw_1)
    if yaw_1>2: 
      Theta_fix=Theta_RfixValue
      send.sendContinuousValue(x+100,y,0,theta2+Theta_fix-1,0)
      print(" TWO 右轉")

    elif yaw_1<-2:
      Theta_fix=Theta_LfixValue
      send.sendContinuousValue(x+100,y+400,0,theta2+Theta_fix+1,0)
      print(" TWO 左轉")
    
    elif -2<=yaw_1 and yaw_1<=2 :
      send.sendContinuousValue(x+200,y,0,theta2,0)
      print(" TWO 直走")
    #print("gggggg",theta)  

def imu_3():  #舉起線到終點線在用的
    yaw_1=send.imu_value_Yaw
    # yaw_1+=yaw
    print("yaw_1= ",yaw_1)
    if yaw_1>3: 
      Theta_fix=-1
      send.sendContinuousValue(x3+500,y3-200,0,theta3+Theta_fix-2,0)
      print(" THREE 右轉")
    if yaw_1<-3: 
      Theta_fix=1
      send.sendContinuousValue(x3+500,y3+400,0,theta3+Theta_fix+4,0)
      print(" THREE 左轉")
    elif -3<=yaw_1 and yaw_1<=3: 
      send.sendContinuousValue(x3+500,y3,0,theta3+1,0)
      print(" THREE 直走")
def afterbar():
    print('revise')
    yaw=send.imu_value_Yaw
    time.sleep(0.35)
    send.sendSensorReset()
    return yaw
  
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
                            # send.sendSensorReset()
                            turn_on()
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
                            send.sendContinuousValue(xl-200,yl+600,0,tl,0)
                            print('左左左左左左左左左左左左左左左左左左左')
                          elif red_middle>target_right:
                            send.sendContinuousValue(xr-200,yr-400,0,tr,0)
                            print('右右右右右右右右右右右右右右右右右右右')

                          elif red_middle>target_left and red_middle<target_right:
                            print("左右踏步結束，原地旋轉/微調")
                            if target_ymax>pickup_distance4 and target_ymin>pickup_distance4:
                              print("太靠近了！！！後退！！！！！！")
                              send.sendContinuousValue(xBack,yStand,0,Theta_origin,0)
                            # if target_ymin<pickup_distance3:
                            #   print("桿子右下左上，右旋！！！！！")
                            #   send.sendContinuousValue(xStand,yStand,0,RightStand,0)
                            # elif target_ymax<pickup_distance3:
                            #   print("桿子左下右上，左旋！！！")
                            #   send.sendContinuousValue(xStand,yStand,0,LeftStand,0)
                            arrive=True
                            print('微調結束')
                      if arrive==True:                                     
                       turn_off()
                       time.sleep(3.5)
                       print('down')
                       time.sleep(0.5)
                       print('我要抬手囉!!!!!!')
                       send.sendBodySector(pick1)
                       time.sleep(4.5)
                       target_ymax,target_ymin,target_xmax,target_xmin=red_line()
                       print('第305行')
                       print("target_xmax= ",target_xmax)
                       print("target_xmin= ",target_xmin)
                       print("target_ymax= ",target_ymax)
                       print("target_ymin= ",target_ymin,end='\n')
                       red_middle=round((target_xmax+target_xmin)/2)                        
                       distance2=round(red_middle2-red_middle)
                       if distance2>32:
                         distance=10
                       elif distance2<-32:
                         distance=-10
                       else :
                         distance=distance2
                       time.sleep(0.5)
                       print('修正= ',distance)
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
                       print('我要撿起來囉!!!!!!!!!')
                    if arrive2==True:  
                      print('我撿撿撿撿撿撿撿撿撿撿')
                      time.sleep(1.5)
                      send.sendBodySector(pick2)
                      time.sleep(5.5)
                      print('修正= ',distance)
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
                      time.sleep(6)  
                      print("111 yaw before afterbar= ",yaw) 
                      # yaw=afterbar()
                      print("222 yaw after afterbar= ",yaw)
                      # time.sleep(1.5)
                      send.sendHeadMotor(2,head_motor_angle3,50) 
                      time.sleep(1)
                      
                      # print("調整起步磁區")
                      # send.sendBodySector(pick4)
                      # time.sleep(5)
                      # print("起步磁區調整完畢，準備出發")

                      pick_bar=True     
                      print('舉起線在哪!?')
                      # send.sendSensorReset()
                  if pick_bar==True:
                    # send.sendSensorReset()
                    time.sleep(0.1)
                    turn_on()
                    print_section()
                    time.sleep(1)
                    print("舉起線我來了!!!")
                    white_xmax ,white_xmin,white_ymax,white_ymin=white_line()
                    time.sleep(0.35)
                    imu_2()
                    print('find whiteYMax_one= ',white_ymax)
                    if white_ymax>liftup_distance1 and white_ymax<liftup_distance2:
                      # time.sleep(1)
                      print("舉起線要到了") 
                      send.sendHeadMotor(2,head_motor_angle4,50) 
                      time.sleep(0.2)
                      lift_line=True
                      time.sleep(5)
                    else:
                      print('imu_2微調')
                      imu_2()
              if lift_line==True:
                white_xmax ,white_xmin,white_ymax,white_ymin=white_line()
                print('distance 2=',white_ymax)
                if white_ymax>liftup_distance3 and white_ymax<liftup_distance4:
                  print('======停下，舉起======')
                  turn_off()
                  time.sleep(2.5)
                  turn_off()
                  send.sendBodySector(lift)
                  time.sleep(8.8)
                  # yaw=afterbar()
                  # time.sleep(1.5)
                  lift_bar=True
                  print("imu_value_Pitch= ",send.imu_value_Pitch)
                  time.sleep(1.5)
                  # send.sendSensorReset()
                else :          
                  print('imu_2微調')       
                  imu_2()
            elif lift_bar==True:
              print_section()
              print('======終點我來了======')
              # send.sendSensorReset()
              turn_on()
              time.sleep(0.5)
              imu_3()      
              print('耶~~~~~到了~~~~~')
          if send.is_start == False:
            turn_off()
            print("草尼馬,Ready to go!")
            white_xmax ,white_xmin,white_ymax,white_ymin=white_line()
            print("whiteYMax_one= ",white_ymax)
            target_ymax,target_ymin,target_xmax,target_xmin=red_line()
            red_middle=float(target_xmax+target_xmin)/2
            print("red_middle= ",red_middle)
          r.sleep()  
            
                     
    except rospy.ROSInterruptException:
        pass