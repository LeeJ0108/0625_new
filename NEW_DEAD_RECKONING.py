import numpy as np
from pyquaternion import Quaternion
import rospy
import numpy as np

import time
import matplotlib.pyplot as plt

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
from ublox_msgs.msg import NavPVT,NavVELNED
from math import atan2 ,pi, sin, cos
from sensor_msgs.msg import NavSatFix


from tf.transformations import euler_from_quaternion

from pyproj import Proj, Transformer, CRS

import time
import csv

proj_UTMK = CRS(init='epsg:5179')
proj_WGS84 = CRS(init='epsg:4326')


class Position:
    def __init__(self):
        self.sub_imu = rospy.Subscriber('/imu/data', Imu, self.imu_callback, queue_size=1)
        self.sub_gps = rospy.Subscriber("ublox_gps/fix", NavSatFix, self.gps_callback,queue_size=1)
        # self.sub_gps_heading = rospy.Subscriber("ublox_gps/navpvt", NavPVT, self.gps_heading_callback,queue_size=1)
        self.sub_gps_vel = rospy.Subscriber("ublox_gps/navpvt",NavPVT,self.gps_data_callback,queue_size=1)
        self.erp_sub = rospy.Subscriber('erp_read', erp_read, self.erp_callback, queue_size=1)

        self.GPS_INPUT_TIME = time.time()
        self.GPS_LAST_TIME = time.time()
        self.GPS_VELOCITY_COUNT = 0
        self.GPS_VELOCITY_dT = 0
        self.GPS_VELOCITY_VEL = 0 #속도
        self.GPS_VELOCITY_LAST_VEL = 0 #이전 속도
        self.GPS_VECOCITY_accel = 0
        self.GPS_VECOCITY_Heading = 0

        self.pos = [0,0]
        self.last_pos = [0,0]

        self.GPS_VELOCITY_COUNT = 0
        self.GPS_DATA_dT = 0
        self.GPS_DATA_LAST_TIME = time.time()
        self.GPS_DATA_accel = 0
        self.GPS_DATA_velocity_kmh도 = 0 #속도
        self.GPS_DATA_LAST_velocity_kmh = 0 #이전 속도
        self.GPS_DATA_heading_degrees = 0
        self.GPS_DATA_pos_heading_radians = 0

        self.ENC_VELOCITY_COUNT = 0
        self.ENC = 0
        self.ENC_LAST_TIME = time.time()
        self.ENC_NOW_TIME = time.time()
        self.ENC_LAST_VELOCITY_VEL = 0
        self.ENC_ACCEL = 0
        self.ENC_VELOCITY_dT = 0
        self.ENC_VELOCITY_VEL = 0

        self.IMU_VELOCITY_COUNT = 0
        self.IMU_accel_x = 0
        self.IMU_accel_y = 0
        self.yaw = 0
        self.IMU_LAST_accel_x = 0
        self.IMU_LAST_accel_y = 0
        self.IMU_INPUT_TIME = time.time()
        self.IMU_LAST_TIME = time.time()
        self.IMU_VELOCITY_VEL = 0
        self.IMU_ACCEL_X_avg = 0
        self.IMU_VELOCITY_dT = 0
        
    def gps_callback(self, gps_msg):
        lon = gps_msg.longitude
        lat = gps_msg.latitude
        transformer = Transformer.from_crs(proj_WGS84, proj_UTMK)
        x,y = transformer.transform(lon,lat)
        self.pos = [x, y]
        self.GPS_INPUT_TIME = time.time()

    def GPS_POS(self):
        return self.pos    

    def GPS_VELOCITY(self):
        if self.GPS_VELOCITY_COUNT == 0:
            while self.pos == [0,0]:
                print("WAITING")
            self.GPS_LAST_TIME = self.GPS_INPUT_TIME
            self.last_pos = self.pos
            self.GPS_VELOCITY_COUNT = 1

        elif self.GPS_VELOCITY_COUNT == 1:
            self.GPS_VELOCITY_dT = abs(self.GPS_INPUT_TIME-self.GPS_LAST_TIME)

            if self.GPS_VELOCITY_dT > 0.1:
                Distance = np.hypot(self.pos[0] - self.last_pos[0],self.pos[1] - self.last_pos[1])
                self.GPS_VECOCITY_Heading = np.arctan2(self.pos[1] - self.last_pos[1],self.pos[0] - self.last_pos[0])

                if self.GPS_VELOCITY_dT != 0:
                    self.GPS_VECOCITY_accel = (self.GPS_VELOCITY_LAST_VEL - self.GPS_VELOCITY_VEL) / self.GPS_VELOCITY_dT /3.6

                if self.GPS_VECOCITY_Heading > 2*pi:
                    self.GPS_VECOCITY_Heading -= 2*pi

                self.GPS_VELOCITY_VEL = Distance / self.GPS_VELOCITY_dT * 3.6

                self.last_pos = self.pos
                self.GPS_LAST_TIME = self.GPS_INPUT_TIME
                self.GPS_VELOCITY_LAST_VEL = self.GPS_VELOCITY_VEL

        return self.GPS_VELOCITY_VEL, self.GPS_VECOCITY_accel, self.GPS_VECOCITY_Heading, self.GPS_VELOCITY_dT


    def gps_data_callback(self,nav_msg):
        self.GPS_DATA_dT = abs(self.GPS_INPUT_TIME - self.GPS_DATA_LAST_TIME)
        velocity_ms = nav_msg.gSpeed / 1000
        self.GPS_DATA_velocity_kmh = velocity_ms * 3.6

        if self.GPS_DATA_dT != 0:
            self.GPS_DATA_accel = (self.GPS_DATA_velocity_kmh-self.GPS_DATA_LAST_velocity_kmh) / self.GPS_DATA_dT / 3.6

        pos_heading_degrees = nav_msg.heading * 1e5
        self.GPS_DATA_pos_heading_radians = np.radians(pos_heading_degrees)

        if self.GPS_DATA_pos_heading_radians > 2*np.pi:
            self.GPS_DATA_pos_heading_radians  -= 2*np.pi

        heading_degrees = nav_msg.headVeh * 1e5
        self.GPS_DATA_heading_degrees = np.radians(heading_degrees)

        if self.GPS_DATA_heading_degrees > 2*np.pi:
            self.GPS_DATA_heading_degrees  -= 2*np.pi

        accuracy_velocity = nav_msg.sAcc /1000
        accuracy_heading = nav_msg.headAcc * 1e5

        print("-----속도 정확도-----",accuracy_velocity,"-----헤딩 정확도-----",accuracy_heading)

        self.GPS_DATA_LAST_TIME = self.GPS_INPUT_TIME 
        self.GPS_DATA_LAST_velocity_kmh = self.GPS_DATA_velocity_kmh

    def GPS_DATA_VELOCITY(self):
        return self.GPS_DATA_velocity_kmh, self.GPS_DATA_accel, self.GPS_DATA_heading_degrees,self.GPS_DATA_dT

    def erp_callback(self, data):
        self.steer = (data.read_steer)/71
        self.ENC = data.read_ENC
        self.ERP_INPUT_TIME = time.time()

    def ENC_VELOCITY(self):
        if self.ENC_VELOCITY_COUNT == 0:
            self.LAST_ENC = self.ENC
            self.ENC_VELOCITY_COUNT = 1

        elif self.ENC_VELOCITY_COUNT == 1:
            if self.LAST_ENC != self.ENC:
                self.ENC_NOW_TIME = time.time()
                self.ENC_VELOCITY_dT = abs( self.ENC_NOW_TIME -  self.ENC_LAST_TIME)

                if self.ENC_VELOCITY_dT != 0:
                    self.ENC_VELOCITY_VEL = 0.5*abs(self.ENC - self.LAST_ENC)*(2*pi/100)/2/self.ENC_VELOCITY_dT * 3.6  # 바퀴 지름 50cm
                    self.ENC_ACCEL = (self.ENC_VELOCITY_VEL - self.ENC_LAST_VELOCITY_VEL) / self.ENC_VELOCITY_dT / 3.6

                self.ENC_LAST_TIME = self.ENC_NOW_TIME
                self.LAST_ENC = self.ENC
                self.ENC_LAST_VELOCITY_VEL = self.ENC_VELOCITY_VEL


        return self.ENC_VELOCITY_VEL,self.ENC_ACCEL,self.ENC_VELOCITY_dT        
    
    def imu_callback(self,imu):
        self.IMU_RAW_accel_x = imu.linear_acceleration.x
        self.IMU_RAW_accel_y = imu.linear_acceleration.y
        self.IMU_RAW_accel_z = imu.linear_acceleration.z

        quaternion = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
        roll, self.pitch, self.yaw = euler_from_quaternion(quaternion)

        if self.yaw  > 2*np.pi:
            self.yaw  -= 2*np.pi

        measurement =[[self.IMU_RAW_accel_x],[self.IMU_RAW_accel_y],[self.IMU_RAW_accel_z]]
        angular = self.pitch
        angular_array = [[np.cos(angular),0,np.sin(angular)],[0,1,0],[-np.sin(angular),0,np.cos(angular)]]
        
        gravity_vector = np.array([[0],[0],[9.80665]])
        result = measurement - np.dot(np.transpose(angular_array) , gravity_vector)

        a = 8
        self.IMU_accel_x = -int(result[0][0] * 10**a) / 10**a
        self.IMU_accel_y = int(result[1][0] * 10**a) / 10**a
        self.IMU_accel_z = int(result[2][0] * 10**a) / 10**a    

        self.IMU_INPUT_TIME = time.time()

    def IMU_VELOCITY(self):
        if self.IMU_VELOCITY_COUNT == 0:
            self.IMU_LAST_accel_x = self.IMU_accel_x
            self.IMU_LAST_accel_y = self.IMU_accel_y
            self.IMU_LAST_TIME = time.time()
            self.IMU_VELOCITY_COUNT = 1

        elif self.IMU_VELOCITY_COUNT == 1:
            self.IMU_VELOCITY_dT = abs(self.IMU_INPUT_TIME - self.IMU_LAST_TIME)    

            if self.IMU_VELOCITY_dT > 0.1:
                self.IMU_ACCEL_X_avg = self.IMU_accel_x + self.IMU_LAST_accel_x
                self.IMU_ACCEL_Y_avg = self.IMU_accel_y + self.IMU_LAST_accel_y
                self.IMU_VELOCITY_X = (self.IMU_ACCEL_X_avg) * self.IMU_VELOCITY_dT 
                self.IMU_VELOCITY_Y = (self.IMU_ACCEL_Y_avg) * self.IMU_VELOCITY_dT 

                self.IMU_VELOCITY_VEL += self.IMU_VELOCITY_X *3.6

                if self.IMU_VELOCITY_VEL < 0:
                    self.IMU_VELOCITY_VEL = 0

                self.IMU_LAST_accel_x = self.IMU_accel_x
                self.IMU_LAST_accel_y = self.IMU_accel_y
                self.IMU_LAST_VELOCITY_VEL = self.IMU_VELOCITY_VEL
                self.IMU_LAST_TIME = self.IMU_INPUT_TIME

            
        return self.IMU_VELOCITY_VEL,self.IMU_ACCEL_X_avg,self.yaw,self.IMU_VELOCITY_dT
    
class KalmanFilter_ENC:
    def __init__(self, process_variance, measurement_variance, estimation_error, initial_value):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimation_error = estimation_error
        self.value = initial_value
        self.kalman_gain = 0

    def update(self, measurement, update_measurement_variance):
        self.measurement_variance = update_measurement_variance

        # Prediction update
        self.estimation_error += self.process_variance

        # Measurement update
        self.kalman_gain = self.estimation_error / (self.estimation_error + self.measurement_variance)
        self.value += self.kalman_gain * (measurement - self.value)
        self.estimation_error *= (1 - self.kalman_gain)

        return self.value
        
def main():
    rospy.init_node('mapping', anonymous=True)
    p = Position()
    rate = rospy.Rate(10)

    GPS_pos_velocity,GPS_pos_accel,GPS_pos_heading,GPS_pos_dt = 0,0,0,0
    GPS_data_velocity,GPS_data_accel,GPS_data_heading,GPS_data_dt = 0,0,0,0
    IMU_velocity,IMU_accel,IMU_heading,IMU_dt = 0,0,0,0
    ENC_velocity,ENC_accel,ENC_dt = 0,0,0

    GPS_pos_velocity_plot,GPS_pos_accel_plot,GPS_pos_heading_plot = [],[],[]
    GPS_data_velocity_plot,GPS_data_accel_plot,GPS_data_heading_plot = [],[],[]
    IMU_velocity_plot,IMU_accel_plot,IMU_heading_plot = [],[],[]
    ENC_velocity_plot,ENC_accel_plot = [],[]

    count,count_plot = 0,[]

    ###############################################################################

    process_variance = 1e-5  # 프로세스 노이즈 공분산
    measurement_variance = 0.000375  # 측정 노이즈 공분산 // 노이즈가 클수록 더 크게
    estimation_error = 1.0  # 초기 추정 오차
    initial_value = 0 # 초기 값게

    kf_ENC_VELOCITY = KalmanFilter_ENC(process_variance, measurement_variance, estimation_error, initial_value)

    FILTERED_ENC_VELOCITY = 0
    FILTERED_ENC_VELOCITY_plot = []

    while not rospy.is_shutdown():  
        GPS_pos_velocity,GPS_pos_accel,GPS_pos_heading,GPS_pos_dt = p.GPS_VELOCITY()
        GPS_data_velocity,GPS_data_accel,GPS_data_heading,GPS_data_dt = p.GPS_DATA_VELOCITY()
        IMU_velocity,IMU_accel,IMU_heading,IMU_dt = p.IMU_VELOCITY()
        ENC_velocity,ENC_accel,ENC_dt = p.ENC_VELOCITY()

        GPS_pos_velocity_plot.append(GPS_pos_velocity)
        GPS_data_velocity_plot.append(GPS_data_velocity)
        IMU_velocity_plot.append(IMU_velocity)
        ENC_velocity_plot.append(ENC_velocity)

        GPS_pos_accel_plot.append(GPS_pos_accel)
        GPS_data_accel_plot.append(GPS_data_accel)
        IMU_accel_plot.append(IMU_accel)
        ENC_accel_plot.append(ENC_accel)

        GPS_pos_heading_plot.append(GPS_pos_heading)
        GPS_data_heading_plot.append(GPS_data_heading)
        IMU_heading_plot.append(IMU_heading)

        count += 1
        count_plot.append(count)

        ####################################################################################
        start = 100
        if count >start:
            ENC_VELOCITY_array = []
            GPS_data_VELOCITY_array = []
            ENC_VELOCITY_array = np.array(ENC_velocity_plot)
            GPS_data_VELOCITY_array = np.array(GPS_data_velocity_plot)
            covariance_matrix = np.cov(ENC_VELOCITY_array[start:], GPS_data_VELOCITY_array[start:],ddof=1)
            update_measurement_variance = covariance_matrix[0, 1]
            print("공분산",update_measurement_variance)
        else:
            update_measurement_variance = 0.000375 #초기값값    

        FILTERED_ENC_VELOCITY =kf_ENC_VELOCITY.update(ENC_velocity,update_measurement_variance)
        FILTERED_ENC_VELOCITY_plot.append(FILTERED_ENC_VELOCITY)
        if count > 500:
            break
        rate.sleep()

    start = 5    
    plt.figure(1)
    plt.plot(count_plot[start:], GPS_pos_velocity_plot[start:], label='GPS', color='red')
    # plt.plot(count_plot[start:], IMU_velocity_plot[start:], label='IMU', color='blue')
    plt.plot(count_plot[start:], ENC_velocity_plot[start:], label='ENC', color='green')
    plt.plot(count_plot[start:], GPS_data_velocity_plot[start:], label='GPS_DATA', color='orange')
    plt.plot(count_plot[start:], FILTERED_ENC_VELOCITY_plot[start:], label='ENC_filter', color='black')
    plt.xlabel('count')
    plt.ylabel('velocity')
    plt.title('Velocity')
    plt.show()


if __name__ == "__main__":
    main()                  