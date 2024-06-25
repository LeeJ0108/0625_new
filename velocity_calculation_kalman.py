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
from ublox_msgs.msg import NavPVT
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
        self.erp_sub = rospy.Subscriber('erp_read', erp_read, self.erp_callback, queue_size=1)

        self.GPS_INPUT_TIME = time.time()
        self.GPS_LAST_TIME = time.time()
        self.pos = [0,0]
        self.last_pos = [0,0]
        self.GPS_init_Count = 0

        self.velocity = 0
        
        self.IMU_init_Count = 0
        self.IMU_last_accel_x = 0
        self.linear_x = 0
        self.linear_y = 0
        self.linear_z= 0
        self.IMU_last_Time  = time.time()
        self.IMU_input_Time = time.time()
        self.IMU_vel_total= 0
        self.last_velocity = 0


        self.timer = 0.3
        self.count = 0
        self.yaw  = 0
        self.pitch = 0
        self.linear_x2  = 0
        self.linear_y2= 0
        self.linear_z2 = 0

        self.erp_init_Count= 0
        self.last_enc = 0
        self.enc = 0 
        self.erp_velocity2  = 0
        self.erp_INPUT_TIME = time.time()
        self.erp_LAST_TIME = time.time()


    def erp_callback(self, data):
        self.steer = (data.read_steer)/71
        self.enc = data.read_ENC
        self.erp_INPUT_TIME = time.time()


    def erp_Velocity(self):
        if self.erp_init_Count == 0:
           
            self.erp_LAST_TIME = self.erp_INPUT_TIME
            self.last_enc = self.enc  
            self.erp_count = 0
            self.erp_init_Count = 1    
      
        elif self.erp_init_Count == 1:    
            # print(self.erp_count)
            
            if self.last_enc !=  self.enc:
                dt = abs(self.erp_INPUT_TIME-self.erp_LAST_TIME)
                self.erp_velocity2 = 0.5*abs(self.enc - self.last_enc)*(2*np.pi/100)/2/dt * 3.6  # 바퀴 지름 50cm
                if self.erp_velocity2 >50:
                    self.erp_velocity2 = 0
                self.erp_LAST_TIME = self.erp_INPUT_TIME
                self.last_enc = self.enc 
                self.erp_count = 0
                # print(self.erp_velocity2)


        return self.erp_velocity2 
    
    def gps_callback(self, gps_msg):
        
        lon = gps_msg.longitude
        lat = gps_msg.latitude
        transformer = Transformer.from_crs(proj_WGS84, proj_UTMK)
        x,y = transformer.transform(lon,lat)
        self.pos = [x, y]
        self.GPS_INPUT_TIME = time.time()
        if self.count < 5:
            self.count += 1
        
    def GPS_VELOCITY(self):
        if self.GPS_init_Count == 0:
           
            self.GPS_LAST_TIME = self.GPS_INPUT_TIME
            self.last_pos = self.pos  
            if self.count > 4:
                # print(self.count)
                self.GPS_init_Count = 1
              
        elif self.GPS_init_Count == 1:    
            Time_interval = abs(self.GPS_INPUT_TIME- self.GPS_LAST_TIME)
            if Time_interval > self.timer:
                distance = np.hypot(self.pos[0] - self.last_pos[0],self.pos[1] - self.last_pos[1])
                self.velocity = round(distance/Time_interval *3.6,1)
                
                self.last_pos = self.pos
                self.GPS_LAST_TIME = self.GPS_INPUT_TIME
                self.last_velocity = self.velocity

        return self.velocity

    def GPS_POS(self):
        return self.pos       
        
    def imu_callback(self, imu):
        self.linear_x = imu.linear_acceleration.x
        self.linear_y = imu.linear_acceleration.y 
        self.linear_z = imu.linear_acceleration.z 
        self.angular_z = imu.angular_velocity.z 
        self.magnetic_field_x = imu.orientation.x
        self.magnetic_field_y = imu.orientation.y
        self.magnetic_field_z = imu.orientation.z
        self.IMU_input_Time = time.time()
        
    
        quaternion = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
        roll, self.pitch, self.yaw = euler_from_quaternion(quaternion)
        if self.yaw  < 0:
            self.yaw  += 2*np.pi
 

        measurement =[[self.linear_x],[self.linear_y],[self.linear_z]]
        angular = self.pitch
        angular_array = [[np.cos(angular),0,np.sin(angular)],[0,1,0],[-np.sin(angular),0,np.cos(angular)]]
        
        # 업데이트 단계
        # updated_state = kf.update(measurement)
        gravity_vector = np.array([[0],[0],[9.80665]])
        

        result = measurement - np.dot(np.transpose(angular_array) , gravity_vector)
        # print(np.array(result))
        
        # self.linear_x2  = result[0][0]
        a = 8
        self.linear_x2 = int(result[0][0] * 10**a) / 10**a
        self.linear_y2 = int(result[1][0] * 10**a) / 10**a
        self.linear_z2 = int(result[2][0] * 10**a) / 10**a
        # print( self.linear_x2, self.linear_y2, self.linear_z2,self.IMU_vel_total )

        # self.linear_y2 = 0
        # print(self.linear_x2,self.linear_y2,self.yaw,self.IMU_vel_total)
        

    def IMU_HEADING(self):
        return self.yaw        
            
    def IMU_velocity_calc(self):
        if self.IMU_init_Count == 0:
            
            self.IMU_last_accel_x = self.linear_x2
            self.IMU_last_accel_y = self.linear_y2
            self.IMU_last_Time  = time.time()
            self.IMU_init_Count = 1
        
            
        elif self.IMU_init_Count == 1:
            IMU_time_intervel = self.IMU_input_Time -self.IMU_last_Time 
            if IMU_time_intervel >= 0.001:  
                self.IMU_vel_x = (self.linear_x2+self.IMU_last_accel_x)*IMU_time_intervel/2 
                self.IMU_vel_y = (self.linear_y2+self.IMU_last_accel_y) * IMU_time_intervel/2 
   
                i = 1
                if self.IMU_vel_x > 0:
                    i = 1
                elif self.IMU_vel_x == 0:
                    i = 0
                else:
                    i = -1    
                self.IMU_vel_total += i*np.sqrt(self.IMU_vel_x**2+self.IMU_vel_y**2) * 3.6
                # self.IMU_vel_total += self.IMU_vel_x 
                
                if self.IMU_vel_total == 0:
                    self.vel_error = 0
                if self.IMU_vel_total<0:
                    self.IMU_vel_total = 0  
                    self.vel_error = 0

                # print(self.IMU_vel_total)

                self.IMU_last_Time = self.IMU_input_Time
                self.IMU_last_accel_x  = self.linear_x2
                self.IMU_last_accel_y  = self.linear_y2
             
        return self.IMU_vel_total
    

class kalman:
    def __init__(self):
        # 초기 상태 (예: 초기 속도 추정값)
        self.x = np.array([0, 0, 0])  # 세 가지 방법의 초기 속도 추정값

        # 관측 행렬
        self.H = np.eye(3)

        # 초기 오차 공분산
        self.P = np.eye(3) * 1000

        # 상태 전이 행렬
        self.A = np.eye(3)

        # 프로세스 노이즈 공분산
        self.Q = np.eye(3)

        # 관측 노이즈 공분산 (각 방법의 노이즈 레벨에 따라 설정)

        sigma1 = 1
        sigma2 = 1
        sigma3 = 1

        self.R = np.array([
            [sigma1**2, 0, 0],
            [0, sigma2**2, 0],
            [0, 0, sigma3**2]
        ])

        self.best_velocity = 0


   
    def select_min_value(self,val1, val2, val3):
        # 1단계: 첫 번째 값을 최솟값으로 초기 설정
        count = 1
        min_value = val1
        
        # 2단계: 두 번째 값과 비교
        if val2 < min_value:
            min_value = val2
            count = 2
            
        # 3단계: 세 번째 값과 비교
        if val3 < min_value:
            min_value = val3
            count = 3
        
        # 4단계: 최솟값 반환
        return min_value,count

    def kalman_filter(self, measurements, R):
        self.R = R
        for z in measurements:
            
            self.x = np.dot(self.A, self.x)
            self.P = np.dot(self.A, np.dot(self.P, self.A.T)) + self.Q

            # 관측 단계
            y = z - np.dot(self.H, self.x)
            S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
            K = np.dot( self.P, np.dot(self.H.T, np.linalg.inv(S)))
            self.x = self.x + np.dot(K, y)
            self.P =  self.P - np.dot(K, np.dot(self.H,  self.P))

            # print("------------",self.P )
            min_value,count = self.select_min_value(self.P[0][0],self.P[1][1],self.P[2][2])
            if count == 1:
                self.best_velocity = self.x[0]

            elif count == 2:
                self.best_velocity = self.x[1]

            else:
                self.best_velocity = self.x[2] 


        return self.best_velocity,min_value,count

        
    

def main():
    rospy.init_node('mapping', anonymous=True)
    p = Position()
    K = kalman()
    rate = rospy.Rate(10)

    X = []
    Y = []
    Z = []
    E = []
    F = []


    count = 0
    init_pos = [0,0]
    while init_pos == [0,0]:
        init_pos = p.GPS_POS()


    while not rospy.is_shutdown():  
        GPS_vel = p.GPS_VELOCITY()
        IMU_Vel = p.IMU_velocity_calc()
        ERP_vel = p.erp_Velocity()
        count +=1

    
        measurements = [GPS_vel, IMU_Vel, ERP_vel]  # 첫 번째 측정
        
        X.append(count)
        Y.append(GPS_vel)
        Z.append(IMU_Vel) 
        E.append(ERP_vel)

        sigma1 = np.std(Y)  # 방법 1의 표준편차
        # sigma2 = np.std(Z)  # 방법 2의 표준편차
        sigma2 = 10000
        sigma3 = np.std(E)  # 방법 3의 표준편차

        # 관측 노이즈 공분산 행렬 구성
        R = np.array([
            [sigma1**2, 0, 0],
            [0, sigma2**2, 0],
            [0, 0, sigma3**2]
        ])

        # 필터 적용
        filtered_speed,min_P,num = K.kalman_filter(measurements,R )
        print("Filtered speed:", filtered_speed,"공분산",min_P,"숫자",num)

        F.append(filtered_speed)



        if count > 100000000:
            break
        
        rate.sleep()
        
 
    start = 5    
    plt.figure(1)
    plt.plot(X[start:], Y[start:], label='GPS VEL', color='blue')
    plt.plot(X[start:], Z[start:], label='IMU VEL', color='red')
    plt.plot(X[start:], E[start:], label='ENC VEL', color='green')
    plt.plot(X[start:], F[start:], label='KALMAN VEL', color='black')

    plt.xlabel('count')
    plt.ylabel('velocity')
    plt.title('Velocity')
    plt.legend()  

    plt.show()

if __name__ == "__main__":
    main()                           