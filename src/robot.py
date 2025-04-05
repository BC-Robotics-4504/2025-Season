import wpilib
import math

from magicbot import MagicRobot
import wpilib.shuffleboard

from hmi import HMI, HMIConfig

from intake import Intake, IntakeConfig

from wench import Wench, WenchConfig

from drivetrain import SwerveDrive, SwerveConfig

from pathplannerlib.config import RobotConfig

from vision import Vision, VisionConfig


class MyRobot(MagicRobot):

    
    # Swerve Drive Component Code
    
    
    
    swerve: SwerveDrive
    swerve_config = SwerveConfig(
        fl_CAN=(1, 2),  # (drive_id, turn_id)
        fl_zoffset=3.23,  # rad
        fl_loc=tuple(RobotConfig.fromGUISettings().moduleLocations[0]),  # m
        fr_CAN=(3, 4),  # (drive_id, turn_id)
        fr_zoffset=5.73,
        fr_loc=tuple(RobotConfig.fromGUISettings().moduleLocations[1]),  # m
        rl_CAN=(5, 6),  # (drive_id, turn_id)
        rl_zoffset=4.096,  # rad
        rl_loc=tuple(RobotConfig.fromGUISettings().moduleLocations[2]),  # m
        rr_CAN=(7, 8),  # (drive_id, turn_id)
        rr_zoffset=4.096,  # rad
        rr_loc=tuple(RobotConfig.fromGUISettings().moduleLocations[3]),  # m
        wheel_diameter=RobotConfig.fromGUISettings().moduleConfig.wheelRadiusMeters * 2,
        CAN_id_imu=11,  # IMU_id
        max_driving_speed=3500,
        max_angular_speed= 600,
        chassis_length=0.762,
        chassis_width=0.762,
        drive_wheel_diameter=0.114
    )

    Intake: Intake
    Intake_config = IntakeConfig(
        CAN_ids=(20, 21),
        pivot_zoffset=0,
        pivot_gear_ratio=64,
        up_angle=4.,
        down_angle=2.,
        spinner_speed=0.17
    )

    Wench: Wench
    Wench_config = WenchConfig(CAN_id=23, 
                               up_angle=1, 
                               down_angle=5, 
                               gear_ratio=1, 
                               zoffset=0)

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

        pass

    def teleopPeriodic(self):
        """MyRobot.teleopPeriodic() -> None

        Called repeatedly during teleoperated mode."""
        # if self.Vision.getTargetDistance() is not None:
        #     print(self.Vision.getTargetDistance())
        # Move drivetrain based on Left X/Y and Right X/Y controller inputsW

        Lx, Ly, Rx, _ = self.HMI.getAnalogSticks()


        # deadband = 0.05
        # if abs(Lx) < deadband:
        #     Lx = 0
            
        # if abs(Ly) < deadband:
        #     Ly = 0
            
        # if abs(Rx) < deadband:
        #     Rx = 0

        #  # Actuate Launcher

        # if self.HMI.getB():
        #     self.Intake.setDown()
        #     self.Intake.setSpin()

        # else:
        #     self.Intake.setUp()
            
        if self.HMI.getA():
            self.Intake.setSpin()
            
        elif self.HMI.getY():
            self.Intake.setInverseSpin()
            
        else:
            self.Intake.resetSpin()

      
        self.swerve.move(-Lx, Ly, Rx)
               
        # print(f"RR: Angle {self.swerve.rr_mod.getAngle()},\n RAW STICKS: {self.HMI.getAnalogSticks()} ")
        
        if self.HMI.getDpadUp():
            self.Wench.setUp()
        
        if self.HMI.getDpadDown():
            self.Wench.setDown()
            
        """
        SmartDashboard Setup
        """
        
        wpilib.SmartDashboard.putData
        
   

if __name__ == "__main__":
    wpilib.run(MyRobot)
