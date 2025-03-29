import wpilib
from magicbot import MagicRobot

from hmi import HMI, HMIConfig

from intake import Intake, IntakeConfig

from drivetrain import SwerveDrive, SwerveConfig

from pathplannerlib.config import RobotConfig

from vision import Vision, VisionConfig


class MyRobot(MagicRobot):

    # Swerve Drive Component Code
    
    swerve: SwerveDrive
    swerve_config = SwerveConfig(
        fl_CAN=(1, 2),  # (drive_id, turn_id)
        fl_zoffset=0.1104242,  # rad
        fl_loc=tuple(RobotConfig.fromGUISettings().moduleLocations[0]),  # m
        fr_CAN=(3, 4),  # (drive_id, turn_id)
        fr_zoffset= 0.5566983,  # rad
        fr_loc=tuple(RobotConfig.fromGUISettings().moduleLocations[1]),  # m
        rl_CAN=(5, 6),  # (drive_id, turn_id)
        rl_zoffset=0.8517382,  # rad
        rl_loc=tuple(RobotConfig.fromGUISettings().moduleLocations[2]),  # m
        rr_CAN=(7, 8),  # (drive_id, turn_id)
        rr_zoffset=0.2010291,  # rad
        rr_loc=tuple(RobotConfig.fromGUISettings().moduleLocations[3]),  # m
        wheel_diameter=RobotConfig.fromGUISettings().moduleConfig.wheelRadiusMeters * 2,
        CAN_id_imu=11,  # IMU_id
    ) 

    Intake: Intake
    Intake_config = IntakeConfig(
        CAN_ids=(20, 21),  
        pivot_zoffset=.4,
        pivot_gear_ratio=1,
        up_angle=0,
        down_angle=24,
        spinner_speed=0,
    )

    # Controller Component Code
    HMI: HMI
    HMI_config = HMIConfig(port_id=0)

    # Vision Code
    # vision: Vision
    vision_config = VisionConfig(
        camera_angle=0,
        camera_mount_height=0.25,
        apriltag_target_height=1.25,
        max_target_range=3.37,
        min_target_range=2.60,
    )

    def createObjects(self):
        """MyRobot.createObjects() -> None

        Create motors and other hardware components here."""
        pass

    def disabledPeriodic(self):
        pass

    def teleopInit(self):
        """MyRobot.teleopInit() -> None

        Called once each time the robot enters teleoperated mode.
        """
        # self.IntakeController.ground()
        self.swerve.clearFaults()
        # self.intakeController.defaultGround()

        pass

    def teleopPeriodic(self):
        """MyRobot.teleopPeriodic() -> None

        Called repeatedly during teleoperated mode."""
        # if self.Vision.getTargetDistance() is not None:
        #     print(self.Vision.getTargetDistance())
        # Move drivetrain based on Left X/Y and Right X/Y controller inputs
        
        Lx, Ly, Rx, _ = self.HMI.getAnalogSticks()
        
        #  # Actuate Launcher
        if self.HMI.getA():
            self.Intake.setUp()
            

        if self.HMI.getB():
            self.Intake.setDown()
            
        else:
            self.Intake.setUp()
            
            
        # if self.HMI.getY():
            
            
        # if self.HMI.getRT() > 0.35:
            
            
        # if self.HMI.getLT() > 0.35:
            
        
        # # #3.) Actuate Climber
        # if self.HMI.getDpadUp():
        #     self.Climber.raiseClimber()
        
        # elif self.HMI.getDpadDown():
        #     self.Climber.lowerClimber()
            
        # elif self.HMI.getStart():
        #     self.Climber.lockClimber()
        
        #Runs LAUNCHER state machine
       
        self.swerve.move(Lx, Ly, Rx)
        

        # """
        # SmartDashboard Setup
        # """

        # # Add stuff to SmartDashboard
        # wpilib.SmartDashboard.putNumber(
        #     "LF Speed", self.SwerveDrive.FrontLeftSpeedMotor.getSpeed()
        # )
        # wpilib.SmartDashboard.putNumber(
        #     "LF Angle",
        #     self.SwerveDrive.FrontLeftAngleMotor.getAbsPosition(),
        # )
        # wpilib.SmartDashboard.putNumber(
        #     "RF Speed", self.SwerveDrive.FrontRightSpeedMotor.getSpeed()
        # )
        # wpilib.SmartDashboard.putNumber(
        #     "RF Angle",
        #     self.SwerveDrive.FrontRightAngleMotor.getAbsPosition(),
        # )
        # wpilib.SmartDashboard.putNumber(
        #     "LR Speed", self.SwerveDrive.RearLeftSpeedMotor.getSpeed()
        # )
        # wpilib.SmartDashboard.putNumber(
        #     "LR Angle", self.SwerveDrive.RearLeftAngleMotor.getAbsPosition()
        # )
        # wpilib.SmartDashboard.putNumber(
        #     "RR Speed", self.SwerveDrive.RearRightSpeedMotor.getSpeed()
        # )
        # wpilib.SmartDashboard.putNumber(
        #     "RR Angle",
        #     self.SwerveDrive.RearRightAngleMotor.getAbsPosition(),
        # )


if __name__ == "__main__":
    wpilib.run(MyRobot)
