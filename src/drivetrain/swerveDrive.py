
from collections import namedtuple
from .swerveModule import SwerveModule

import wpilib
from phoenix6.hardware import Pigeon2 
from phoenix6.configs import Pigeon2Configuration 
from phoenix6.canbus import CANBus
import math

from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState, SwerveModulePosition
from wpimath.estimator import SwerveDrive4PoseEstimator

from wpilib import DriverStation

from pathplannerlib.auto import AutoBuilder 
from pathplannerlib.controller import PPHolonomicDriveController
import numpy as np 
import os
# from pathplannerlib.config import config, PIDConstants

SwerveConfig = namedtuple('config',
                          ['fl_CAN',
                           'fl_zoffset',
                           'fl_loc',
                          'fr_CAN',
                          'fr_zoffset',
                          'fr_loc',
                          'rl_CAN',
                          'rl_zoffset',
                          'rl_loc',
                          'rr_CAN',
                          'rr_zoffset',
                          'rr_loc',
                          'wheel_diameter',
                          'CAN_id_imu',
                          'max_driving_speed',
                          'max_angular_speed',
                          'chassis_length',
                          'chassis_width',
                          'drive_wheel_diameter'])


class SwerveDrive:
    """SwerveDrive Class"""

    config: SwerveConfig

    fl_mod: SwerveModule
    fr_mod: SwerveModule
    rl_mod: SwerveModule
    rr_mod: SwerveModule
    
    
    __frontLeftAngle__: float = 0
    __frontLeftSpeed__: float = 0
    
    __rearLeftAngle__: float = 0
    __rearleftSpeed__: float = 0
    
    __rearRightAngle__: float = 0
    __rearRightSpeed__: float = 0
    
    __frontRightAngle__: float = 0
    __frontRightSpeed__: float = 0
    #
    
    move_changed: bool = False
    distance_changed: bool = False
    target_chassis_speeds:ChassisSpeeds = ChassisSpeeds(0, 0, 0)
    current_pose: Pose2d = Pose2d(0, 0, 0)

    LxSlewRateLimiter = SlewRateLimiter(0.5)
    LySlewRateLimiter = SlewRateLimiter(0.5)
    RxSlewRateLimiter = SlewRateLimiter(1)
    
    def __init__(self, config: SwerveConfig) -> None:
        # TODO: update robotpy config in main file
        self.config = config
        
        self.fl_mod = SwerveModule(config.fl_CAN, config.fl_zoffset, config.wheel_diameter)
        self.fr_mod = SwerveModule(config.fr_CAN, config.fr_zoffset, config.wheel_diameter)
        self.rl_mod = SwerveModule(config.rl_CAN, config.rl_zoffset, config.wheel_diameter)
        self.rr_mod = SwerveModule(config.rr_CAN, config.rr_zoffset, config.wheel_diameter)

    def clearFaults(self):
        """SwerveDrive.clearFaults()
        Clears faults on all of the SparkMax modules
        """
        self.fl_mod.clearFaults()
        self.fr_mod.clearFaults()
        
        self.rl_mod.clearFaults()
        self.rr_mod.clearFaults()
        
        return False
    
    def move(self, Lx: float, Ly: float, Rx: float): 
        """SwerveDrive.move(Lx: float, Ly: float, Rx: float)

        ::params::
        Lx: Magnitude to move in the x direction in range, negative is forward [-1, 1]
        Ly: Magnitude to move in the y direction in range, positive is left [-1, 1]
        Rx: Magnitude of rotational speed where counter-clockwise is positive [-1, 1]
        """
        # Rx = self.RxSlewRateLimiter.calculate(Rx)
        # Check negatives and positives here for Lx, Ly, and Rx
        Vx0 = -Lx*self.config.max_driving_speed*self.config.drive_wheel_diameter*math.pi
        Vy0 = Ly*self.config.max_driving_speed*self.config.drive_wheel_diameter*math.pi
        w0 = Rx*self.config.max_angular_speed*math.pi
        
        # Calculate component vectors for the swerve modeule
        Vxp = Vx0 + w0*self.config.chassis_length
        Vxn = Vx0 - w0*self.config.chassis_length
        Vyp = Vy0 - w0*self.config.chassis_width
        Vyn = Vy0 + w0*self.config.chassis_width

        # Calculate the speed and angle for each swerve motor
        self.__frontLeftAngle__ = math.atan2(Vyp, Vxp) + math.pi/2
        self.__frontLeftSpeed__ = math.hypot(Vyp, Vxp)/(math.pi*self.config.drive_wheel_diameter)

        self.__rearLeftAngle__ = math.atan2(Vyp, Vxn) + math.pi/2
        self.__rearLeftSpeed__ = math.hypot(Vyp, Vxn)/(math.pi*self.config.drive_wheel_diameter)

        self.__rearRightAngle__ = math.atan2(Vyn, Vxn) + math.pi/2
        self.__rearRightSpeed__ = math.hypot(Vyn, Vxn)/(math.pi*self.config.drive_wheel_diameter)

        self.__frontRightAngle__ = math.atan2(Vyn, Vxp) + math.pi/2
        self.__frontRightSpeed__ = math.hypot(Vyn, Vxp)/(math.pi*self.config.drive_wheel_diameter)
        
        # maxSpeed = max([self.__frontLeftSpeed__, self.__frontRightSpeed__,
        #         self.__rearLeftSpeed__, self.__rearRightSpeed__])

        # if maxSpeed > 1.:
        #     scalar = 1./maxSpeed
        #     self.__frontLeftSpeed__ *= scalar
        #     self.__frontRightSpeed__ *= scalar
        #     self.__rearRightSpeed__ *= scalar
        #     self.__rearLeftSpeed__ *= scalar
        
        # print(f"\nRR-Transformed-Angle {self.__frontRightAngle__}\n")
        # Enable the move to be changed when "execute()" is run
        self.move_changed = True
        
        return False  
    
    def __compensate__(ang:float):
        if ang < -math.pi:
            return ang + 2*math.pi
        
        elif ang > math.pi:
            return 2*math.pi - ang
        
        else:
            return ang

    def resetEncoders(self):
        self.fl_mod.resetEncoder()
        self.rr_mod.resetEncoder()
        self.fr_mod.resetEncoder()
        self.rl_mod.resetEncoder()

    def atDistance(self):
        """SwerveDrive.atDistance()

        Checks if each wheel has traveled a specified distance"""
        FL = self.fl_mod.atDistance()
        FR = self.fr_mod.atDistance()
        RL = self.rl_mod.atDistance()
        RR = self.rr_mod.atDistance()

        if FL and FR and RL and RR:
            return True

        return False
    
    def goDistance(self, Rx0: float, Ry0: float, r0: float):
        """SwerveDrive.gotDistance(target_distance: float, target_angle: float, target_rotations: float)
        
        Calculates the angle and speed for a robot to move autonomusly a certain distance and at a target angle. 

        ::params::
        target_distance: The target distance the robot should travel autonomously range[0, 2*math.pi]
        target_angle: the target angle the robot should be at once the autonomous movement ends
        target_rotations: How many time should the robot spin before the autonomous movement ends.

        """
        Rx0 *= self.config.drive_wheel_diameter*math.pi
        Ry0 *= self.config.drive_wheel_diameter*math.pi
        r0  *= -math.pi
        
        # Calculate component vectors for the swerve modeule
        Xp = Rx0 + r0*self.config.chassis_length
        Xn = Rx0 - r0*self.config.chassis_length
        Yp = Ry0 - r0*self.config.chassis_width
        Yn = Ry0 + r0*self.config.chassis_width        
        
        self.__frontLeftAngle__ = math.atan2(Yp, Xp)+math.pi/2
        self.__frontLeftDistance__ = math.hypot(Yp, Xp)/(math.pi*self.config.drive_wheel_diameter)

        self.__rearLeftAngle__ = math.atan2(Yp, Xn)+math.pi/2
        self.__rearLeftDistance__ = math.hypot(Yp, Xn)/(math.pi*self.config.drive_wheel_diameter)

        self.__rearRightAngle__ = math.atan2(Yn, Xn)+math.pi/2
        self.__rearRightDistance__ = math.hypot(Yn, Xn)/(math.pi*self.config.drive_wheel_diameter)

        self.__frontRightAngle__ = math.atan2(Yn, Xp)+math.pi/2
        self.__frontRightDistance__ = math.hypot(Yn, Xp)/(math.pi*self.config.drive_wheel_diameter)
                
        self.distance_changed = True
    
    def updateDrive(self):
        self.fl_mod.setAngle(self.__frontLeftAngle__)
        self.fl_mod.setSpeed(self.__frontLeftSpeed__)
        
        self.fr_mod.setAngle(self.__frontRightAngle__)
        self.fr_mod.setSpeed(self.__frontRightSpeed__)
        
        self.rr_mod.setAngle(self.__rearRightAngle__)
        self.rr_mod.setSpeed(self.__rearRightSpeed__)
        
        self.rl_mod.setAngle(self.__rearLeftAngle__)
        self.rl_mod.setSpeed(self.__frontLeftSpeed__)
        
    def updateDistance(self) -> None:
        
        """Updates the distances and angles of the wheels during auto
        
        @params
        None
        
        """
        
        self.fl_mod.setDistance(self.__frontLeftDistance__) 
        self.fl_mod.setAngle(self.__frontLeftAngle__)
        
        self.fr_mod.setDistance(self.__frontRightDistance__) 
        self.fr_mod.setAngle(self.__frontRightAngle__)
        
        self.rl_mod.setDistance(self.__rearLeftDistance__) 
        self.rl_mod.setAngle(self.__rearLeftAngle__)
        
        self.rr_mod.setDistance(self.__rearRightDistance__) 
        self.rr_mod.setAngle(self.__rearRightAngle__)
        
        print(self.__rearRightDistance__)
       
        self.distance_changed = False
        
    def execute(self):
        """SwerveDrive.execute()
        Updates the postion of the absolute encoders and the speed of each swerve module
        """
        if self.move_changed: 
            
            self.updateDrive()
            
        if self.distance_changed:
            
            self.updateDistance()
            
            
        
        # print(self.fl_mod.getAngle(), self.fl_mod.getSpeed(), self.target_chassis_speeds)
        

        # self.updatePose()
