from collections import namedtuple
from phoenix6.hardware import Pigeon2
from phoenix6.configs import Pigeon2Configuration

from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpilib import Timer

import numpy as np
from numpy.linalg import norm

imuConfig = namedtuple('imuConfig', 
                       'CAN_id')

class IMU:
    """ IMU Class"""
    
    imuConfig: imuConfig
    
    timer:Timer|None
    
    beta:float
    zeta:float
    prev_time = 0
    current_pose = Pose2d(0, 0, 0)
    
    def __init__(self, beta=1, zeta=1):
        self.beta = beta
        self.zeta = zeta
        
        imu_config = Pigeon2Configuration()
        imu_config.mount_pose.mount_pose_yaw = 0
        imu_config.mount_pose.mount_pose_pitch = 0
        imu_config.mount_pose.mount_pose_roll = 90
        self.IMU.ConfigAllSettings(imu_config)
        
        self.__initTimer__()
        self.__initMadgwick__()

        
    def __initTimer__(self):
        self.timer = Timer()
        self.timer.start()
        self.current_time = self.timer.get()
        
    def __initMadgwick__(self):
        current_time = self.timer.get()
        self.dt = current_time - self.prev_time
        self.prev_time = current_time
        return
    
    def __updateMadgwick__(self):
        # https://github.com/morgil/madgwick_py/blob/master/madgwickahrs.py
        gx = self.IMU.get_accum_gyro_x()
        gy = self.IMU.get_accum_gyro_y()
        gz = self.IMU.get_accum_gyro_z()
        g = np.array([gx, gy, gz], dtype=float).flatten()
        
        ax = self.IMU.get_acceleration_x()
        ay = self.IMU.get_acceleration_y()
        az = self.IMU.get_acceleration_z()
        a = np.array([ax, ay, az], dtype=float).flatten()
        a /= norm(a) # Normalise accelerometer measurement
        
        mx = self.IMU.get_magnetic_field_x()
        my = self.IMU.get_magnetic_field_y()
        mz = self.IMU.get_magnetic_field_z()
        m = np.array([mx, my, mz], dtype=float).flatten()
        m /= norm(m) # Normalise magnetometer measurement
        return
    
    def __getTimeSinceUpdate__(self):
        # self.current_time 
        self.IMU.get_up_time()
        return
    
    def execute(self):
        self.__madgwickUpdate__()
    