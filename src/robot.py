import wpilib
import math

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
        CAN_id_imu=11,  # IMU_id #NOTE Irrelevant since IMU has been removed 
        max_driving_speed=3500, # RPM
        max_angular_speed=600, # RPM
        chassis_length=0.762, # m 
        chassis_width=0.762, # m
        drive_wheel_diameter=0.114,# m 
    )

    Intake: Intake
    Intake_config = IntakeConfig(
        CAN_ids=(20, 21),
        pivot_zoffset=0,
        pivot_gear_ratio=64,
        up_angle=4.0,
        down_angle=2.0,
        spinner_speed=0.45,
        spinner_high_speed=0.58,
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
        
        # Swerve Statistics

        wpilib.SmartDashboard.putNumber("FR Speed", self.swerve.fr_mod.getSpeed())
        wpilib.SmartDashboard.putNumber("FR Angle", self.swerve.fr_mod.getAngle())

        wpilib.SmartDashboard.putNumber("FL Speed", self.swerve.fl_mod.getSpeed())
        wpilib.SmartDashboard.putNumber("FL Angle", self.swerve.fl_mod.getAngle())

        wpilib.SmartDashboard.putNumber("RR Speed", self.swerve.rr_mod.getSpeed())
        wpilib.SmartDashboard.putNumber("RR Angle", self.swerve.rr_mod.getAngle())

        wpilib.SmartDashboard.putNumber("RL Speed", self.swerve.rl_mod.getSpeed())
        wpilib.SmartDashboard.putNumber("RL Angle", self.swerve.rl_mod.getAngle())
        
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

        # Move drivetrain based on Left X/Y and Right X/Y controller inputs

        Lx, Ly, Rx, _ = self.HMI.getAnalogSticks()

        # Compensate for Stick Drift

        # deadband = 0.05
        # if abs(Lx) < deadband:
        #     Lx = 0

        # if abs(Ly) < deadband:
        #     Ly = 0

        # if abs(Rx) < deadband:
        #     Rx = 0

        # Actuate Launcher

        if self.HMI.getLB():
            self.Intake.setDown()

        else:
            self.Intake.setUp()

        # Spin Launcher

        if self.HMI.getA():
            self.Intake.setSpin()

        elif self.HMI.getY():
            self.Intake.setHighInverse()

        elif self.HMI.getRB():
            self.Intake.setHighSpin()

        else:
            self.Intake.resetSpin()

        self.swerve.move(-Lx, Ly, Rx)

        """
        SmartDashboard Setup
        """

        # Spinner Motor Statistics

        wpilib.SmartDashboard.putNumber(
            "Spinner Motor Current", self.Intake.readSpinnerCurrent()
        )
        wpilib.SmartDashboard.putNumber(
            "Spinner Motor Temp.", self.Intake.readSpinnerTemp()
        )

        # Pivot Motor Statistics

        wpilib.SmartDashboard.putNumber(
            "Pivot Motor Current", self.Intake.readPivotCurrent()
        )
        wpilib.SmartDashboard.putNumber(
            "Pivot Motor Temp.", self.Intake.readPivotTemp()
        )

        # Swerve Statistics

        wpilib.SmartDashboard.putNumber("FR Speed", self.swerve.fr_mod.getSpeed())
        wpilib.SmartDashboard.putNumber("FR Angle", self.swerve.fr_mod.getAngle())

        wpilib.SmartDashboard.putNumber("FL Speed", self.swerve.fl_mod.getSpeed())
        wpilib.SmartDashboard.putNumber("FL Angle", self.swerve.fl_mod.getAngle())

        wpilib.SmartDashboard.putNumber("RR Speed", self.swerve.rr_mod.getSpeed())
        wpilib.SmartDashboard.putNumber("RR Angle", self.swerve.rr_mod.getAngle())

        wpilib.SmartDashboard.putNumber("RL Speed", self.swerve.rl_mod.getSpeed())
        wpilib.SmartDashboard.putNumber("RL Angle", self.swerve.rl_mod.getAngle())


if __name__ == "__main__":
    wpilib.run(MyRobot)
