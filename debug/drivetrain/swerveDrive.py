from collections import namedtuple
from swerveModule import SwerveModule
# from config import RobotConfig

from phoenix6.hardware import Pigeon2
from phoenix6.configs import Pigeon2Configuration 

from wpimath.filter import SlewRateLimiter
from wpimath.units import radians
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState, SwerveModulePosition
from wpimath.estimator import SwerveDrive4PoseEstimator

from wpilib import DriverStation

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants

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
                          'CAN_id_imu'])


class SwerveDrive:
    """SwerveDrive Class"""

    # RobotConfig: RobotConfig    
    config: SwerveConfig

    fl_mod: SwerveModule
    fr_mod: SwerveModule
    rl_mod: SwerveModule
    rr_mod: SwerveModule
    
    move_changed: bool = False
    distance_changed: bool = False
    target_chassis_speeds:ChassisSpeeds = ChassisSpeeds(0, 0, 0)
    current_pose:Pose2d = Pose2d(0, 0, 0)

    LxSlewRateLimiter = SlewRateLimiter(0.5)
    LySlewRateLimiter = SlewRateLimiter(0.5)
    RxSlewRateLimiter = SlewRateLimiter(1)
    
    def __init__(self) -> None:
        
        # TODO: update robotpy config in main file
        self.__initIMU__()
        self.fr_mod = SwerveModule(self.config.fl_CAN, self.config.fl_zoffset, self.config.wheel_diameter)
        self.fl_mod = SwerveModule(self.config.fr_CAN, self.config.fr_zoffset, self.config.wheel_diameter)
        self.rl_mod = SwerveModule(self.config.rl_CAN, self.config.rl_zoffset, self.config.wheel_diameter)
        self.rr_mod = SwerveModule(self.config.rr_CAN, self.config.rr_zoffset, self.config.wheel_diameter)
        self.IMU = Pigeon2(self.config.CAN_id_imu)
        
        # Setup kinematics module
        frontLeftLocation = Translation2d(self.config.fl_loc[0], self.config.fl_loc[1]) # TODO: update me
        frontRightLocation = Translation2d(self.config.fr_loc[0], self.config.fr_loc[1]) # TODO: update me
        rearLeftLocation = Translation2d(self.config.rl_loc[0], self.config.rl_loc[1]) # TODO: update me
        rearRightLocation = Translation2d(self.config.rr_loc[0], self.config.rr_loc[1]) # TODO: update me
        
        self.kinematics = SwerveDrive4Kinematics(
            frontLeftLocation, 
            frontRightLocation, 
            rearLeftLocation, 
            rearRightLocation
            )
        
        # setup pose estimator
        self.poseEstimator = SwerveDrive4PoseEstimator(self.kinematics, 
                                                       Rotation2d(self.IMU.get_acceleration_x().value), # TODO: Check me
                                                       [self.flSwervePos,
                                                        self.frSwervePos,
                                                        self.rlSwervePos,
                                                        self.rrSwervePos],
                                                       Pose2d(0, 0, 0) # TODO: Update me--will need to change for each auto configuration
                                                       )
        
        
        config = RobotConfig.fromGUISettings()
        # https://github.com/mjansen4857/pathplanner/blob/main/examples/java/src/main/java/frc/robot/subsystems/SwerveSubsystem.java
        AutoBuilder.configure(
            self.getPose,
            self.resetPose,
            self.getRobotRelativeSpeeds,
            lambda speeds, feedforwards: self.driveRobotRelative(speeds),
            PPHolonomicDriveController(
                PIDConstants(5.0, 0.0, 0.0),
                PIDConstants(5.0, 0.0, 0.0)
                ),
            config,
            self.shouldFlipPath,
            self
        )  
        
    def __initIMU__(self):
        imu_config = Pigeon2Configuration()
        imu_config.mount_pose.mount_pose_yaw = 0
        imu_config.mount_pose.mount_pose_pitch = 0
        imu_config.mount_pose.mount_pose_roll = 90
        self.IMU.configurator.apply(imu_config)
        
    def __initSwerveModule__(self, CAN_id_turn, CAN_id_direction, z_offset_turn):
        return
        
    def getPose(self):
        return self.poseEstimator.getEstimatedPosition()
        
    def resetPose(self):
        self.poseEstimator.resetPose()
    
    @property   
    def flSwervePos(self):
        # ang = self.frontLeftAngleMotor.getAbsPosition()
        # dist = self.frontLeftSpeedMotor.getDistance()
        ang = self.fl_mod.getAngle()
        dist = self.fl_mod.getSpeed()
        return SwerveModulePosition(dist, Rotation2d(ang))
    
    @property
    def frSwervePos(self):
        ang = self.fr_mod.getAngle()
        dist = self.fr_mod.getSpeed()
        return SwerveModulePosition(dist, Rotation2d(ang))
    
    @property
    def rlSwervePos(self):
        ang = self.rl_mod.getAngle()
        dist = self.rl_mod.getSpeed()
        return SwerveModulePosition(dist, Rotation2d(ang))
    
    @property
    def rrSwervePos(self):
        ang = self.rr_mod.getAngle()
        dist = self.rr_mod.getSpeed()
        return SwerveModulePosition(dist, Rotation2d(ang))
    
    def updatePose(self):
        self.poseEstimator.update(Rotation2d(self.IMU.get_accum_gyro_x()), # TODO: Check me
                                    [self.__flSwervePos__(),
                                    self.__frSwervePos__(),
                                    self.__rlSwervePos__(),
                                    self.__rrSwervePos__()]
        )
        
        
    def shouldFlipPath(self):
        return DriverStation.getgetAlliance() == DriverStation.Alliance.kRed
    
    def driveRobotRelativeSpeeds(self, chassisSpeeds:ChassisSpeeds):
        fl, fr, rl, rr = self.kinematics.toSwerveModuleStates(chassisSpeeds)
        
        self.frontLeftAngleMotor.setAbsPosition(fl.angle)
        self.frontLeftSpeedMotor.setSpeed(fl.speed)
        
        self.frontRightAngleMotor.setAbsPosition(fr.angle)
        self.frontRightSpeedMotor.setSpeed(fr.speed)

        self.rearLeftAngleMotor.setAbsPosition(rl.angle)
        self.rearLeftSpeedMotor.setSpeed(rl.speed)

        self.rearRightAngleMotor.setAbsPosition(rr.angle)
        self.rearRightSpeedMotor.setSpeed(rr.speed)
        
        
    def getRobotRelativeSpeeds(self):
        fl = SwerveModuleState(self.frontLeftSpeedMotor.getSpeed, self.frontLeftAngleMotor.getAbsPosition)
        fr = SwerveModuleState(self.frontRightSpeedMotor.getSpeed, self.frontRightAngleMotor.getAbsPosition)
        rl = SwerveModuleState(self.rearLeftSpeedMotor.getSpeed, self.rearLeftAngleMotor.getAbsPosition)
        rr = SwerveModuleState(self.rearRightSpeedMotor.getSpeed, self.rearRightAngleMotor.getAbsPosition)        
        return self.kinematics.toChassisSpeeds([fl, fr, rl, rr])

    def clearFaults(self):
        """SwerveDrive.clearFaults()
        Clears faults on all of the SparkMax modules
        """
        self.frontLeftAngleMotor.clearFaults()
        self.frontLeftSpeedMotor.clearFaults()
        self.rearLeftAngleMotor.clearFaults()
        self.rearLeftSpeedMotor.clearFaults()
        self.rearRightAngleMotor.clearFaults()
        self.rearRightSpeedMotor.clearFaults()
        self.frontRightAngleMotor.clearFaults()
        self.frontRightSpeedMotor.clearFaults()
        return False
    
    def move(self, Lx: float, Ly: float, Rx: float):
        # https://pathplanner.dev/pplib-getting-started.html#-b8ykko_12
        # https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
        
        speeds = ChassisSpeeds(Lx, Ly, Rx)
        fl, fr, rl, rr = self.kinematics.toSwerveModuleStates(speeds)
        
        # Rotation optimization + Cos compensation
        fl_angle = Rotation2d(self.frontLeftAngleMotor.getAbsPosition())
        fl = SwerveModuleState.optimize(fl, fl_angle)
        fl.speed *= (fl.angle - fl_angle).cos()
        
        fr_angle = Rotation2d(self.frontRightAngleMotor.getAbsPosition())
        fr = SwerveModuleState.optimize(fr, fr_angle)
        fr.speed *= (fr.angle - fl_angle).cos()
        
        rl_angle = Rotation2d(self.rearLeftAngleMotor.getAbsPosition())
        rl = SwerveModuleState.optimize(rl, rl_angle)
        rl.speed *= (rl.angle - rl_angle).cos()
        
        rr_angle = Rotation2d(self.rearRightAngleMotor.getAbsPosition())
        rr = SwerveModuleState.optimize(rr, rr_angle)
        rr.speed *= (rr.angle - rr_angle).cos()
        
        self.target_chassis_speeds = self.kinematics.toChassisSpeeds([fl, fr, rl, rr])
        
        self.move_changed = True        

    def resetEncoders(self):
        self.rearLeftSpeedMotor.resetEncoder()
        self.rearRightSpeedMotor.resetEncoder()
        self.frontLeftSpeedMotor.resetEncoder()
        self.frontRightSpeedMotor.resetEncoder()

    def atDistance(self):
        """SwerveDrive.atDistance()

        Checks if each wheel has traveled a specified distance"""
        FL = self.frontLeftSpeedMotor.atDistance()
        FR = self.frontRightSpeedMotor.atDistance()
        RL = self.rearLeftSpeedMotor.atDistance()
        RR = self.rearRightSpeedMotor.atDistance()

        if FL and FR and RL and RR:
            return True

        return False

    def execute(self):
        """SwerveDrive.execute()
        Updates the postion of the absolute encoders and the speed of each swerve module
        """

        if self.move_changed:
            self.driveRobotRelativeSpeeds(self.target_chassis_speeds)
            self.move_changed = False
            
        self.updatePose()
