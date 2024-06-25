#!/usr/bin/env python
# -- coding: utf-8 --

# basic package
from datetime import date
import rospy
import time
from math import pi
import math
import numpy as np

# message file
from macaron_6.msg import erp_read
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
from ublox_msgs.msg import NavPVT
from math import atan2 ,pi, sin, cos


L = 1.04
W = 0.985
r = 0.265
add = 0

class Dead_Reckoning():
    def __init__(self, init_col=[[0,0]]):
        #pub sub
        self.erp_sub = rospy.Subscriber('erp_read', erp_read, self.erp_callback, queue_size=1)
        self.pose_sub = rospy.Subscriber('current_pose', Point, self.pose_callback, queue_size = 1)
        self.sub_imu = rospy.Subscriber('/imu', Imu, self.imu_callback, queue_size=1)
        self.sub_heading = rospy.Subscriber('/ublox_gps/navpvt', NavPVT, self.heading_callback, queue_size=1)
        self.prev_enc = -9999
        self.Sl = 0
        self.Sm = 0
        self.enc = 0
        self.yaw = 0
        self.gps_heading = 0
        self.heading_offset = 0
        self.heading_offset_flag = False
        self.steer = 0
        self.pose = [0,0]
        self.offset = 0
        self.init_col = [[0,0]]
        #     init_col = [self.pose]
        # self.foot_print = np.array(init_col)
        self.en = []
        self.a = np.array([])
        self.pose_mem = [0,0]
        self.gps_heading2 = 0
        self.prev_time = None

    def pose_callback(self, data):
        if np.hypot(self.pose_mem[0]-data.x, self.pose_mem[1]-data.y) > 0.3:
            if not self.pose_mem[0] == 0:
                self.a = np.append(self.a, [atan2(self.pose_mem[1]-data.y, self.pose_mem[0]-data.x)], axis=0)
            self.pose_mem = [data.x,data.y]
        if len(self.a) == 6:
            self.a = np.delete(self.a, (0), axis = 0)
            self.gps_heading2 = np.mean(self.a, axis = 0)+pi

        self.pose = [data.x, data.y]
        self.heading = data.z


    def erp_callback(self, data):
        self.steer = (data.read_steer)/71
        self.enc = data.read_ENC

    def imu_callback(self, imu):
        self.yaw = self.q_to_yaw(imu.orientation)
            # 시간 추출
        current_time = imu.header.stamp

        # 이전 시간이 초기화되어 있지 않다면, 현재 시간을 이전 시간으로 설정하고 리턴
        if self.prev_time is None:
            self.prev_time = current_time
            return
        # if abs(imu.angular_velocity.y) < 0.001:
        #     return
        
        dt = (current_time - self.prev_time).to_sec()
        
        self.prev_time = current_time

        if self.heading_offset_flag:
            self.yaw2 += imu.angular_velocity.y*dt


    def heading_callback(self, head):
        sub_gps_heading = float(head.heading)
        self.gps_heading = self.tf_heading_to_rad(sub_gps_heading)
        print("------------------------")
        print("gps : ",self.gps_heading)
        print("------------------------")
        print('yaw : ',self.yaw)
        print("heading_offset :",self.heading_offset)
        print("offset_yaw : ",self.yaw+self.heading_offset)

    def tf_heading_to_rad(self, head):
        heading = 5*pi/2 - np.deg2rad(float(head / 100000))
        if heading > 2*pi:
            heading -= 2*pi

        return heading

    def q_to_yaw(self, imu):

        yaw = (-1) * imu.x * pi / 180

        if yaw < 0:
            yaw = pi + (pi + yaw)

        if yaw > 2 * pi:
            yaw = yaw - (2 * pi)
        elif yaw <= 0:
            yaw = yaw + (2 *pi)

        return yaw

    def calc_position(self):
        global add
        now_enc = self.enc

        if self.init_col == [[0,0]]:
            self.init_col = [self.pose]
            self.foot_print = np.array(self.init_col)

        if self.yaw != 0 and self.gps_heading2 != 0 and self.heading_offset_flag == False :
            self.heading_offset = self.gps_heading2 - self.yaw
            # self.yaw2 = self.gps_heading2
            self.yaw2 = self.heading
            self.init_col = [self.pose]
            self.foot_print = np.array(self.init_col)
            self.heading_offset_flag = True

        if not self.heading_offset_flag:
            return

        if self.prev_enc == -9999 and now_enc != 0 and self.heading_offset_flag:
            self.prev_enc = now_enc
            self.offset = self.heading - self.yaw


        else:
            try:
                R = abs(L/sin((self.steer*math.pi)/180)) #최소값 -> 2
                OCl = R*abs(cos(self.steer*math.pi/180))+W/2
                OCr = R*abs(cos(self.steer*math.pi/180))-W/2
            except:
                pass

            if now_enc - self.prev_enc != 0:
                interval = now_enc - self.prev_enc
                print("now_enc - self.prev_enc : ",interval)
                add += interval
                #self.en.append(self.enc)
                Sl = (now_enc-self.prev_enc)*(2*pi/100)*r
                if self.steer < 0:
                    Sm = Sl*OCl/R*0.975
                elif self.steer > 0:
                    Sm = Sl*OCr/R*0.995
                else:
                    Sm = Sl
                #Sm = Sl
                #dx = Sm*np.cos(self.yaw+self.heading_offset)
                #dy = Sm*np.sin(self.yaw+self.heading_offset)
                # dx = Sm*np.cos(self.gps_heading)
                # dy = Sm*np.sin(self.gps_heading)
                dx = Sm*np.cos(self.yaw2)
                dy = Sm*np.sin(self.yaw2)

                x = self.foot_print[-1][0] + dx
                y = self.foot_print[-1][1] + dy

                self.foot_print = np.append(self.foot_print ,[[x,y]], axis=0)

                self.prev_enc = now_enc

                self.pose = [x,y]
            else:
                pass

def main():
    rospy.init_node("dead_reckoning", anonymous=True)

    DR = Dead_Reckoning()

    gps = np.empty((1,2))

    while not rospy.is_shutdown():
        if DR.pose != [0,0]:
            gps = np.append(gps, [DR.pose], axis=0)
        DR.calc_position()
        # print(DR.yaw)
        # print(DR.enc)
    gps = np.delete(gps, (0), axis=0)
    print(DR.foot_print)
    print("d:",math.sqrt(DR.foot_print[-1][0]**2+DR.foot_print[-1][1]**2))
    print("add:",add)
    print(DR.en)
    plt.plot(gps[1:,0],gps[1:,1],'red', DR.foot_print[:,0],DR.foot_print[:,1],'blue')

    plt.show()

if __name__ == '__main__':
    main()