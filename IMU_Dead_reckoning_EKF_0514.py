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
        self.accel =0


        self.timer = 0.3
        self.count = 0
        self.yaw  = 0
        self.pitch = 0
        self.linear_x2  = 0
        self.linear_y2= 0
        self.linear_z2 = 0


    def erp_callback(self, data):
        self.steer = (data.read_steer)/71
        self.enc = data.read_ENC


    def gps_callback(self, gps_msg):
        
        lon = gps_msg.longitude
        lat = gps_msg.latitude
        transformer = Transformer.from_crs(proj_WGS84, proj_UTMK)
        x,y = transformer.transform(lon,lat)
        self.pos = [x, y]
        self.GPS_INPUT_TIME = time.time()
        if self.count < 5:
            self.count += 1
        
    def GPS_VELOCITY(self,flag):
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
                
                ###########################################################################
                if flag ==0:
                    if self.velocity == 0:
                        self.IMU_vel_total =0

                    error = 5
                    if (self.velocity >= self.last_velocity and self.linear_x >= 0) or (self.velocity <= self.last_velocity and self.linear_x <= 0):
                        if (self.velocity >= self.last_velocity - error and self.velocity <= self.last_velocity + error):
                            self.IMU_vel_total = self.velocity
                            print("------------GPS-------------")
                 
                # print(self.velocity)

                else:
                    self.velocity = 0
                ##########################################################################

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
            
    def IMU_velocity_calc(self,flag):
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

                ###########################################################################
                if self.IMU_vel_total < 10 and flag == 0:
                    self.timer = 0.1
                    self.IMU_vel_x = 0
                else:
                    self.timer = 0.5
                ###########################################################################    
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

                self.accel =np.sqrt(((self.linear_x2+self.IMU_last_accel_x)/2)**2 + ((self.linear_y2+self.IMU_last_accel_y)/2)**2 )
        return self.accel,self.IMU_vel_total

    def IMU_HEADING(self):
        return self.yaw        
    
    def IMU_HEADING_degree(self):
        return round(np.rad2deg(self.yaw),1)     
      
    def IMU_GET_ACCEL(self):
        return [self.linear_x2,self.linear_y]
    

class ExtendedKalmanFilter_IMU_ENCODER:
    def __init__(self, initial_state, initial_covariance, process_noise_cov, measurement_function, measurement_noise_cov):
        self.state = initial_state
        self.covariance = initial_covariance
        self.process_noise_cov = process_noise_cov
        self.measurement_function = measurement_function
        self.measurement_noise_cov = measurement_noise_cov

    def predict(self, dt, dynamics_function, dynamics_jacobian):
        # Prediction step
        # Update state estimate
        predicted_state = dynamics_function(self.state, dt)
        # Update state covariance
        F = dynamics_jacobian(self.state, dt)
        # print(F)
        # print(F.shape)
        # print(self.covariance)
        # print(self.covariance.shape)
        # print(F.T)
        # print(F.T.shape)
        # print(self.process_noise_cov)
        # print(self.process_noise_cov.shape)

        self.covariance = F @ self.covariance @ F.T + self.process_noise_cov
        self.state = predicted_state

    def update(self, measurement):         #가속도 속도 순으로 넣을 예정정
        # Update step
        # Calculate measurement Jacobian
        H = self._calculate_measurement_jacobian(self.state)
        # Calculate innovation covariance

        print(H)
        print(H.shape)
        print(self.covariance)
        print(self.covariance.shape)
        print(H.T)
        print(H.T.shape)
        print(self.process_noise_cov)
        print(self.process_noise_cov.shape)

        S = H @ self.covariance @ H.T + self.measurement_noise_cov
        # Calculate Kalman gain
        K = self.covariance @ H.T @ np.linalg.inv(S)
        # Update state estimate
        innovation = measurement - self.measurement_function(self.state)
        self.state = self.state + K @ innovation
        # Update state covariance
        self.covariance = (np.eye(len(self.state)) - K @ H) @ self.covariance

    def _calculate_measurement_jacobian(self, state):
        # Example: Assuming linear measurement function for demonstration
        # Replace with actual measurement function's Jacobian
        return np.eye(len(state))
        # return [[state[0]]]

# Example usage:
# Define the state dynamics function
def dynamics_function(state, dt):
    # Example: Linear dynamics
    # A = np.array([[1, dt],
    #             [0, 1]])
    A = np.array([[0],[1]])
    return np.dot(A, [state])

# Define the dynamics Jacobian
def dynamics_jacobian(state, dt):
    # Example: Jacobian of linear dynamics
    # return np.array([[1, dt],
    #             [0, 1]])

    return np.array([[1,0]])

# Define measurement function
def measurement_function(state):
    # Example: Linear measurement
    return np.array([state])    

        

    
def main():
    rospy.init_node('mapping', anonymous=True)
    p = Position()
    rate = rospy.Rate(10)
    count = 0

    X = []
    Y = []
    Z = []

   
    
    last_time = time.time()
   
    P_Gx = []
    P_Gy = []
    P_Ix = []
    P_Iy = []

    last_dis_x,last_dis_y = 0,0


    yaw= p.IMU_HEADING()
    while yaw == 0:
        print("Waiting",yaw)
        yaw= p.IMU_HEADING()

    set_heading = yaw


    print(set_heading)


    init_pos = [0,0]
    # while init_pos == [0,0]:
    #     init_pos = p.GPS_POS()

    pos = init_pos    


    plus_X = 0
    plus_Y = 0

    flag = 0

    initial_state = np.array([0, 0])  # 초기 속도
    initial_covariance = np.array([[1, 0],
                                   [0, 0]])
    process_noise_cov = np.array([[1e-4], 
                                  [0]])
    measurement_noise_cov = np.array([0.1])

    # 확장 칼만 필터 인스턴스 생성
    ekf = ExtendedKalmanFilter_IMU_ENCODER(initial_state, initial_covariance, process_noise_cov,measurement_function, measurement_noise_cov)

    while not rospy.is_shutdown():  
        GPS_vel = p.GPS_VELOCITY(flag)
        IMU_accel,IMU_Vel = p.IMU_velocity_calc(flag)
        # print(IMU_Vel)
        count += 1
        X.append(count)
        Y.append(IMU_Vel)
        Z.append(GPS_vel)

        yaw = p.IMU_HEADING()
        print(np.rad2deg(yaw))
        
        dt = 0.1 ## 나중에 수정정
        ekf.predict(dt, dynamics_function, dynamics_jacobian)
        ekf.update([IMU_accel,IMU_Vel])

        dt = 0.1 ##사용 안함
        # ekf.predict(dt, dynamics_function, dynamics_jacobian)
        # ekf.update([엔코더 가속도, 엔코더 속도])




        IMU_vel_ms = IMU_Vel /3.6
        dis_x = IMU_vel_ms *np.cos(yaw)
        dis_y = IMU_vel_ms *np.sin(yaw)

        init_time = time.time()
        time_interval = init_time -last_time 
        
        if abs(time_interval) > 0.001:


            plus_X += (dis_x+last_dis_x) * time_interval/2 
            plus_Y += (dis_y+last_dis_y) * time_interval/2 


            init_pos[0] += plus_X
            init_pos[1] += plus_Y
            # init_pos[0] += (dis_x+last_dis_x) * time_interval/2 
            # init_pos[1] += (dis_y+last_dis_y) * time_interval/2 

            predict_pos = [init_pos[0],init_pos[1]]
            last_time = init_time
            last_dis_x,last_dis_y = dis_x ,dis_y


            pos = p.GPS_POS()
            P_Gx.append(pos[0])
            P_Gy.append(pos[1])
            P_Ix.append(predict_pos[0])
            P_Iy.append(predict_pos[1])
 


    start = 5    
    plt.figure(1)
    plt.plot(X[start:], Y[start:], label='GPS+IMU VEL', color='blue')
    plt.plot(X[start:], Z[start:], label='GPS VEL', color='red')

    plt.xlabel('count')
    plt.ylabel('velocity')
    plt.title('Velocity')
    plt.legend()  

    plt.figure(2)
    plt.plot(P_Gx[start:], P_Gy[start:], label='GPS', color='blue')
    plt.plot(P_Ix[start:], P_Iy[start:], label='IMU', color='red')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('POSITION')
    plt.legend()  
    plt.show()

if __name__ == "__main__":
    main()                           