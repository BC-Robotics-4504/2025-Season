from .sparkmaxDrive import SparkMaxDriving
from .sparkmaxTurning import SparkMaxTurning
# from config import RobotConfig

from phoenix6.hardware import Pigeon2
from phoenix6.configs import Pigeon2Configuration

from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState, SwerveModulePosition
from wpimath.estimator import SwerveDrive4PoseEstimator

from wpilib import DriverStation

from pathplannerlib.auto import AutoBuilder
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants


class SwerveDrive:
    """SwerveDrive Class"""

    # RobotConfig: RobotConfig
    
    IMU: Pigeon2

    frontLeftAngleMotor: SparkMaxTurning
    frontLeftSpeedMotor: SparkMaxDriving

    rearLeftAngleMotor: SparkMaxTurning
    rearLeftSpeedMotor: SparkMaxDriving

    rearRightAngleMotor: SparkMaxTurning
    rearRightSpeedMotor: SparkMaxDriving

    frontRightAngleMotor: SparkMaxTurning
    frontRightSpeedMotor: SparkMaxDriving
    
    move_changed: bool = False
    distance_changed: bool = False
    target_chassis_speeds:ChassisSpeeds = ChassisSpeeds(0, 0, 0)
    current_pose: Pose2d(0, 0, 0)

    LxSlewRateLimiter = SlewRateLimiter(0.5)
    LySlewRateLimiter = SlewRateLimiter(0.5)
    RxSlewRateLimiter = SlewRateLimiter(1)
    
    def __init__(self):

        # TODO: update robotpy config in main file
        imu_config = Pigeon2Configuration()
        imu_config.mount_pose.mount_pose_yaw = 0
        imu_config.mount_pose.mount_pose_pitch = 0
        imu_config.mount_pose.mount_pose_roll = 90
        self.IMU.ConfigAllSettings(imu_config)

        # Setup kinematics module
        frontLeftLocation = Translation2d(0.381, 0.381) # TODO: update me
        frontRightLocation = Translation2d(0.381, -0.381) # TODO: update me
        rearLeftLocation = Translation2d(-0.381, 0.381) # TODO: update me
        rearRightLocation = Translation2d(-0.381, -0.381) # TODO: update me
        
        self.kinematics = SwerveDrive4Kinematics(
            frontLeftLocation, 
            frontRightLocation, 
            rearLeftLocation, 
            rearRightLocation
            )
        
        # setup pose estimator
        self.poseEstimator = SwerveDrive4PoseEstimator(self.kinematics, 
                                                       Rotation2d(self.IMU.get_accum_gyro_x()), # TODO: Check me
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
        
    def getPose(self):
        return self.poseEstimator.getEstimatedPosition()
        
    def resetPose(self):
        self.poseEstimator.resetPose()
    
    @property   
    def flSwervePos(self):
        ang = self.frontLeftAngleMotor.getAbsPosition()
        dist = self.frontLeftSpeedMotor.getDistance()
        return SwerveModulePosition(dist, Rotation2d(ang))
    
    @property
    def frSwervePos(self):
        ang = self.frontRightAngleMotor.getAbsPosition()
        dist = self.frontRightSpeedMotor.getDistance()
        return SwerveModulePosition(dist, Rotation2d(ang))
    
    @property
    def rlSwervePos(self):
        ang = self.rearLeftAngleMotor.getAbsPosition()
        dist = self.rearLeftSpeedMotor.getDistance()
        return SwerveModulePosition(dist, Rotation2d(ang))
    
    @property
    def rrSwervePos(self):
        ang = self.rearRightAngleMotor.getAbsPosition()
        dist = self.rearRightSpeedMotor.getDistance()
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
