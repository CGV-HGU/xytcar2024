#!/usr/bin/env python
# -*- coding: utf-8 -*-
####################################################################
# 프로그램명 : hough_drive_c1.py
# 작 성 자 : (주)자이트론, S2Y (2022 대상)
# 생 성 일 : 2020년 07월 23일
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
####################################################################

import rospy, rospkg, time
import numpy as np
import cv2, math
from sensor_msgs.msg import Image #usb카메라
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor #모터
from std_msgs.msg import Int32MultiArray #초음파센서
from math import *
import signal
import sys
import os

def signal_handler(sig, frame):#close exit kill switch
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0])
bridge = CvBridge()
motor = None
Width = 320
Height = 240
Offset = 140 # ROI_top_line , 상한선
gapHeight = 70 # heigth_length offset+gap <=240 상한선부터 세로로 gap만큼의 크기
gapWidth = 320 # gapWidth <=320
sideCut = int((Width-gapWidth)/2)
Cnt = 0
# Define the HSV range for the color orange
lower_orange = np.array([40, 100, 100])
upper_orange = np.array([100, 200, 200])

cam_debug = False

def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    
def ultra_callback(data):
    global ultra_msg
    ultra_msg = data.data

# publish xycar_motor msg
def drive(Angle, Speed): 
    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = Angle
    motor_msg.speed = Speed

    motor.publish(motor_msg)

# draw lines
def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), (0, 255, 0), 2)
    return img


# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (int(lpos - 2), int(7 + offset)),
                       (int(lpos + 2), int(12 + offset)),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (int(rpos - 2), int(7 + offset)),
                       (int(rpos + 2), int(12 + offset)),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (int(center - 2), int(7 + offset)),
                       (int(center + 2), int(12 + offset)),
                       (0, 255, 0), 2)    
    cv2.rectangle(img, (int(157), int(7 + offset)),
                       (int(162), int(12 + offset)),
                       (0, 0, 255), 2)
    return img


# left lines, right lines
def divide_left_right(lines):
    global Width

    low_slope_threshold = 0
    high_slope_threshold = 20

    # calculate slope & filtering with threshold
    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]

        if x2 - x1 == 0:
            slope = 0
        else:
            slope = float(y2-y1) / float(x2-x1)
        
        if low_slope_threshold < abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])

    # divide lines left to right
    left_lines = []
    right_lines = []
    th = -20 #sonbayaham 

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line

        if (slope < 0) and (x2 < Width/2 - th):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width/2 + th):
            right_lines.append([Line.tolist()])

    return left_lines, right_lines

# get average m, b of line, sum of x, y, mget lpos, rpos
def get_slope(llines, rlines):
    lm_sum=0.0
    rm_sum=0.0
    
    lsize = len(llines)
    rsize = len(rlines)
    
    for line in llines:
            xl1, yl1, xl2, yl2 = line[0]
            lm_sum += float(yl2 - yl1) / float(xl2 - xl1)
    
    for line in rlines:
            xr1, yr1, xr2, yr2 = line[0]
            rm_sum += float(yr2 - yr1) / float(xr2 - xr1)
    
    if (lsize==0):
        lsize=1
    if (rsize==0):
        rsize=1
    if (lm_sum==0):
        lm_sum=100
    if(rm_sum==0):
        rm_sum=100
    
    lm_av = lm_sum / lsize
    rm_av = rm_sum / rsize
    
    if (lsize==0) and (rsize==0):
        result_x=1
        return result_x
    else:
        return (abs(lm_av)-abs(rm_av))
    
def get_line_pos(img, lines, left=False, right=False):
    global Width, Height
    global Offset, gapHeight, cam_debug

    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0

    size = len(lines)
    
    m = 0
    b = 0

    if size != 0:
        for line in lines:
            x1, y1, x2, y2 = line[0]

            x_sum += x1 + x2
            y_sum += y1 + y2
            m_sum += float(y2 - y1) / float(x2 - x1)

        x_avg = x_sum / (size * 2) #2:03
        y_avg = y_sum / (size * 2)

        m = m_sum / size
        b = y_avg - m * x_avg

    if m == 0 and b == 0:
        if left:
            pos = 0
        elif right:
            pos = Width
    else:
        y = gapHeight / 2

        pos = (y - b) / m

        if cam_debug:
            b += Offset
            xs = (Height - b) / float(m)
            xe = ((Height/2) - b) / float(m)

            cv2.line(img, (int(xs), int(Height)), (int(xe), int(Height / 2)), (255, 0, 0), 3)


    return img, int(pos)


# show image and return lpos, rpos
def process_image(frame):
    global Width
    global Offset, gapHeight
    global cam_debug
    global lower_orange, upper_orange

    # graysls
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    roi = gray[Offset : Offset+gapHeight, sideCut : gapWidth + sideCut ]

    # blur
    kernel_size = 5
    standard_deviation_x = 3     #Kernel standard deviation along X-axis
    blur_gray = cv2.GaussianBlur(roi, (kernel_size, kernel_size), standard_deviation_x)

    # canny edge
    low_threshold = 120
    high_threshold = 255
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold, kernel_size)
    # cv2.imshow('img', edge_img)
    
    # HoughLinesP
    #all_lines = cv2.HoughLinesP(edge_img, 0.7, math.pi/180, 8, 30, 2)
    all_lines = cv2.HoughLinesP(edge_img, 0.7, math.pi/180, 20, 20, 7)
    # divide left, right lines
    if all_lines is None:
        return (Width)/2, (Width)/2, 0 ,False
    left_lines, right_lines = divide_left_right(all_lines)

    # get center of lines
    frame, lpos = get_line_pos(frame, left_lines, left=True)
    frame, rpos = get_line_pos(frame, right_lines, right=True)
    
    if cam_debug:
        # draw lines
        frame = draw_rectangle(frame, lpos, rpos, offset=Offset)
        frame = cv2.rectangle(frame, (0, Offset), (Width, Offset+gapHeight), (0, 255, 0), 2)


    #stopline_detect
    low_threshold = 40
    high_threshold = 255
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold, kernel_size)
    # cv2.imshow('img2', edge_img)

    # HoughLinesP
    center_lines = cv2.HoughLinesP(edge_img, 0.7, math.pi/180, 10, 50, 5)
    center_line_count = 0
    if center_lines is not None:
        for line in center_lines:
            x1,y1,x2,y2 = line[0]
            angle = np.abs(np.arctan2(y2-y1,x2-x1)*180/np.pi)

            if angle < 10:
                center_line_count = center_line_count + 1


    return lpos, rpos, len(all_lines), center_line_count


def pid_angle(ITerm, error, b_error):
    angle = 0
    Kp = 1.025 #
    Ki = 0.0002 #0.0001 good #0.0002
    Kd = 1.1 #1.0 good #2.0
    dt = 1

    PTerm = Kp * error
    ITerm += Ki * error * dt
    derror = error - b_error
    DTerm = Kd * (derror / dt)
    angle = PTerm + ITerm + DTerm

    return angle, ITerm

def avoid_car(speed):
    global image
    print("avoiding~")
    setMove(-99, speed, 0.35)
    setMove(99, speed, 0.25)
    setMove(0, speed, 0.6)
    setMove(99, speed, 0.8)
    setMove(-99, speed, 0.15)
    
    
def setMove(angle,speed,delay):
    currTime = time.time()
    while time.time() - currTime < delay:
        drive(angle,speed)
    

def start():
    global motor
    global image
    global ultra_msg
    global Width, Height

    rospy.init_node('auto_drive')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    
    rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, ultra_callback)

    rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    print ("---------- Xycar C1 HD v1.0 ----------")
    time.sleep(1)#3
    sq = rospy.Rate(60)
    timeCheck = time.time()
    startedTimeCheck = timeCheck
    frameNum = 0
    Laps = 0
    current_line_hit = 0
    current_ultra_hit = 0

    #PID variables
    ITerm = 0
    b_error = 0

    angle = 0
    speed = 50

    avoided = False
    
    while not rospy.is_shutdown():
        while not image.size == (Width*Height*3):
            continue
        if time.time() > startedTimeCheck + 4 : #after 5 sec
            if not avoided:
                if ultra_msg == None:
                    continue
                if  (ultra_msg[2] < 75) and (ultra_msg[2] > 30):
                    current_ultra_hit += 1
                    print("current_ultra_hit",current_ultra_hit)
                    if current_ultra_hit > 3 :
                        avoided = True
                        avoid_car(speed)

                else :
                    current_ultra_hit = 0
    
        #check_frame per sec
        frameNum  += 1
        if (time.time() - timeCheck) > 1:
            print("fps : ", frameNum )
            timeCheck = time.time()
            frameNum  = 0
        
        draw_img = image.copy()
        
        lpos, rpos, len_all_lines, center_line_count = process_image(draw_img)
       
        
        if time.time() > startedTimeCheck + 4 : #after 3 sec
            if not avoided:
                speed = 25
            else:
                speed = 50
            if center_line_count > 13:
                print("center_line_count: ", center_line_count)
                current_line_hit += 1
                if current_line_hit > 1:
                    Laps = Laps + 1
                    print("Laps : ",Laps)
                    startedTimeCheck = time.time()
                    current_line_hit = 0
            else :
                current_line_hit = 0
        
        # 라인 카운트 2개시 정지 
        if (Laps>=2):
            setMove(0, 30, 0.5)
            setMove(0, -5, 0.1)
            break
  
        #PID control
        center = ((lpos + rpos) / 2)
        #print("center : " ,center)
        dis = gapWidth/2
        error = (center - dis)
        #print("error : " ,error)
        angle, ITerm = pid_angle(ITerm, error, b_error)
        if angle >99 :
            angle = 99
        elif angle <-99 :
            angle = -99
        b_error = error
        #print("angle : " ,angle)
        drive(angle, speed)

        
        sq.sleep()
        

if __name__ == '__main__':
    start()

