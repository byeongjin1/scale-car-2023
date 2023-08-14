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

        # for x, _ in arr : 
        #     x_arr.append(x)
        # x_arr = sorted(x_arr)
        # if 3 <= len(x_arr) <= 4 and abs(x_arr[0] - x_arr[1]) <= 0.1 and abs(x_arr[1] - x_arr[2]) <= 0.1 and abs(x_arr[0] - x_arr[2]) <= 0.1:
        #     self.rabacon_pub.publish(1000.0)    
        left_raba = 0
        right_raba = 0
        
        for x,y in arr :
            if -2.0 < x < -0.1 and x != 0.0 :
                x_arr.append(x)
                if 0 < y < 1.2:
                    right_raba += 1
                elif -1.2 < y < 0.0 :
                    left_raba += 1
                if 0.1 < y < 1.0 : # 1 
                    left_rabacon.append([x,y])
                    if right_cnt + left_cnt < 3 :
                        right_cnt += 1
                elif -1.0 < y < -0.1 : # -1
                    right_rabacon.append([x,y])
                    if right_cnt + left_cnt < 3 :
                        left_cnt += 1

        
        # for x,y in arr :
        #     if -2.0 < x < -0.01 :
        #         x_arr.append(x)
        #         if 0.07 < y <1.0 : # 1 
        #             left_rabacon.append([x,y])
        #             if right_cnt + left_cnt < 3 :
        #                 right_cnt += 1
        #         elif -1.0 < y < -0.07 : # -1
        #             right_rabacon.append([x,y])
        #             if right_cnt + left_cnt < 3 :
        #                 left_cnt += 1
        # print("left_rabacon", left_rabacon)
        # print("right_rabacon", right_rabacon)

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
        if 2 <= len(x_arr)  and abs(x_arr[0] - x_arr[-1]) <= 0.4:
            self.rabacon_pub.publish(500.0)   
            print('@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@')
        elif len(left_rabacon) >= 1 and len(right_rabacon) >= 1 and len(x_arr) >= 4:
            left_close_rabacon = sorted(left_rabacon, reverse=True)
            right_close_rabacon = sorted(right_rabacon, reverse=True)
            raba = (left_close_rabacon[0][1] + right_close_rabacon[0][1])
            if left_raba - right_raba > 1 :
                raba -= 0.1*(left_raba - right_raba)
            elif left_raba - right_raba > 1 :
                raba += 0.1*(right_raba - left_raba)
            self.rabacon_pub.publish(raba*2.0)
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
