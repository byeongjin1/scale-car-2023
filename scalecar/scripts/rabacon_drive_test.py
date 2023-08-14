#!/usr/bin/python3
#-*- coding: utf-8 -*-

import rospy
import math
from time import sleep
from std_msgs.msg import String
# from obstacle_detector.msg import Obstacles
from std_msgs.msg import Int32, Float32,String
from info_object.msg import ObjectInfo

class ClusterLidar :

    def __init__(self) :
        rospy.Subscriber("/object_info", ObjectInfo, self.rabacon)
        self.rabacon_pub = rospy.Publisher("rabacon_drive", Float32, queue_size = 5)
        self.direction_pub = rospy.Publisher("direction_dynamic", String, queue_size = 5)
        self.angle = 0.0 

    def distance(self, x1, y1, x2, y2) :
        return (x1-x2)**2 +(y1-y2)**2
        

    # select close rabacon
    def rabacon(self, _data)  :
        arr = []
        left_cnt = 0
        right_cnt = 0
        direction = None

        for i in range(_data.objectCounts) :
            arr.append((_data.centerX[i], _data.centerY[i]))
        
        arr.sort(reverse=True)
        x_arr = []
        
        left_rabacon = []
        right_rabacon = []

        close_right_x = 0.0
        close_right_y = 0.0
        close_left_x = 0.0
        close_left_y = 0.0

        
        right_dist = 999999
        left_dist = 9999999



        for x,y in arr :
            if -1.8 < x < 0.4 and x!= 0.0 :
                x_arr.append(x)
                if 0.1 < y < 1.0 : # 1 
                    right_rabacon.append([x,y])
                    if self.distance(x,y,0,0) < right_dist and x > 0:
                        right_dist = self.distance(x,y,0,0)
                        close_right_x = x
                        close_right_y = y
                    if right_cnt + left_cnt < 3 :
                        right_cnt += 1
                elif -1.0 < y < -0.1 : # -1
                    left_rabacon.append([x,y])
                    if self.distance(x,y,0,0) < right_dist and x > 0 :
                        left_dist = self.distance(x,y,0,0)
                        close_left_x = x
                        close_left_y = y
                    if right_cnt + left_cnt < 3 :
                        left_cnt += 1
        second_right_x = close_right_x
        second_right_y = close_right_y
        second_left_x = close_left_x
        second_left_y = close_left_y

        second_dist_right = 999999
        second_dist_left = 99999

        for x, y in right_rabacon :
            if self.distance(close_right_x, close_right_y, x, y) < second_dist_right  and close_right_x != x and close_right_y != y:
                second_dist_right =  self.distance(close_right_x, close_right_y, x, y)
                second_right_x = x
                second_right_y = y  

        for x, y in left_rabacon :
            if self.distance(close_left_x, close_left_y, x, y) < second_dist_left and close_left_x != x and close_left_y != y:
                second_dist_left =  self.distance(close_left_x, close_left_y, x, y)
                second_left_x = x
                second_left_y = y  



        if right_cnt + left_cnt == 3 :
            if right_cnt > left_cnt :
                direction = "RIGHT"
            elif left_cnt > right_cnt :
                direction = "LEFT"
            else :
                direction = None
        else :
            direction = None
        self.direction_pub.publish(direction)
            
        # print(x_arr)
        x_arr = sorted(x_arr)
        if 2 <= len(x_arr) <= 5 and abs(x_arr[0] - x_arr[-1]) <= 0.4 :
            self.rabacon_pub.publish(500.0)   
            print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
        elif len(left_rabacon) >= 1 and len(right_rabacon) >= 1 :
            # left_close_rabacon = sorted(left_rabacon, reverse=True)
            # right_close_rabacon = sorted(right_rabacon, reverse=True)
            # raba = (left_close_rabacon[0][1] + right_close_rabacon[0][1])
            raba = second_left_y + second_right_y
            # if len(left_rabacon) > len(right_rabacon) +1:
            #     raba -= 0.1
            # if len(left_rabacon) +1 < len(right_rabacon) :
            #     raba += 0.1
            self.rabacon_pub.publish(raba*2)
        else :
            self.rabacon_pub.publish(1000.0)

    # def cal_angle(self, left_rabacons, right_rabacons) :
    #     left_angle = abs(math.atan(left_rabacons[0].center.y/left_rabacons[0].center.x))
    #     right_angle =  abs(math.atan(right_rabacons[0].center.y/right_rabacons[0].center.x))
    #     rospy.loginfo("left angle = {} right angle = {}".format(left_angle, right_angle))
    #     return (left_angle - right_angle)*0.5


    # def sel_close_rabacon(self, rabacons) :
    #     left_rabacon = []
    #     right_rabacon = []
    #     raba = 0
    #     for circle in rabacons :
    #         if circle.center.y < 0 :
    #             right_rabacon.append(circle)
    #         else :
    #             left_rabacon.append(circle)
    #     left_rabacon = sorted(left_rabacon, key = lambda x : -x.center.x)
    #     right_rabacon = sorted(right_rabacon, key = lambda x : -x.center.x)
    #     if len(left_rabacon) != 0 and len(right_rabacon) != 0 :
    #         raba = (left_rabacon[0].center.y + right_rabacon[0].center.y)/2
    #     return raba

def run() :
    rospy.init_node("rabacon_drive")
    cluster = ClusterLidar()
    rospy.spin()

if __name__=='__main__' :
    run()
