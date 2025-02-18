import math

import wpilib
from magicbot import MagicRobot

from config import RobotConfig

from hmi import HMI
from intake import IntakeController, Intake

from intake import IntakeController, SparkMaxDualSpinner, SparkMaxPivot
from drivetrain import SwerveDrive, SparkMaxDriving, SparkMaxTurning

from vision import Vision
from vision import Limelight


class MyRobot(MagicRobot):

    # Configuration Class
    RobotConfig = RobotConfig()

    # Swerve Drive Component Code
    SwerveDrive: SwerveDrive
    
    Intake: Intake
    IntakeController: IntakeController

    # Controller Component Code
    HMI: HMI

    # Vision Code
    Limelight: Limelight
    

    controlGain: float = 1

    def createObjects(self):
        """MyRobot.createObjects() -> None

        Create motors and other hardware components here."""
        # Swerve Drive Hardware Config
        self.SwerveDrive_FrontLeftAngleMotor = SparkMaxTurning(
            6,
            inverted=False,
            gear_ratio=1,
            wheel_diameter=1,
            absolute_encoder=True,
            z_offset=4.2,
        )
        self.SwerveDrive_FrontLeftSpeedMotor = SparkMaxDriving(
            5, inverted=False, wheel_diameter=0.1143
        )

        self.SwerveDrive_RearLeftAngleMotor = SparkMaxTurning(
            8, inverted=False, wheel_diameter=1, absolute_encoder=True, z_offset=3.9
        )
        self.SwerveDrive_RearLeftSpeedMotor = SparkMaxDriving(
            7, inverted=False, wheel_diameter=0.1143
        )

        self.SwerveDrive_RearRightAngleMotor = SparkMaxTurning(
            2, inverted=False, wheel_diameter=1, absolute_encoder=True, z_offset=4.1
        )
        self.SwerveDrive_RearRightSpeedMotor = SparkMaxDriving(
            1, inverted=False, wheel_diameter=0.1143
        )

        self.SwerveDrive_FrontRightAngleMotor = SparkMaxTurning(
            4, inverted=False, wheel_diameter=1, absolute_encoder=True, z_offset=0.75
        )
        self.SwerveDrive_FrontRightSpeedMotor = SparkMaxDriving(
            3, inverted=False, wheel_diameter=0.1143
        )
        
        # Intake Hardware Config
        self.Intake_OutputSpinner = SparkMaxDualSpinner(10, inverted=True)

        self.Intake_IntakePivot = SparkMaxPivot(9, inverted=False, gear_ratio=4, 
                                                  follower_canID=15)
        
        self.Intake_LimitSwitch = wpilib.DigitalInput(0)
        
        # HMI Hardware Config
        self.HMI_xbox = wpilib.XboxController(0)

        # Vision Hardware Config
        self.Vision_Limelight = Limelight(name="limelight_front")

    def disabledPeriodic(self):
        pass

    def teleopInit(self):
        """MyRobot.teleopInit() -> None

        Called once each time the robot enters teleoperated mode.
        """
        self.IntakeController.ground()
        self.SwerveDrive.clearFaults()
        pass

    def teleopPeriodic(self):
        """MyRobot.teleopPeriodic() -> None

        Called repeatedly during teleoperated mode."""
        # if self.Vision.getTargetDistance() is not None:
        #     print(self.Vision.getTargetDistance())
        # Move drivetrain based on Left X/Y and Right X/Y controller inputs
        Lx, Ly, Rx, _ = self.HMI.getAnalogSticks()
        # print(Lx, Ly, Rx)
        # print(self.SwerveDrive_FrontLeftAngleMotor.getAbsPosition())
        # Rx *= self.controlGain
        Lx *= self.controlGain
        Ly *= self.controlGain

        self.SwerveDrive.move(Lx, Ly, Rx)

        # """
        # SmartDashboard Setup
        # """

        # Add stuff to SmartDashboard
        wpilib.SmartDashboard.putNumber(
            "LF Speed", self.SwerveDrive.SwerveDrive_FrontLeftSpeedMotor.getSpeed()
        )
        wpilib.SmartDashboard.putNumber(
            "LF Angle",
            self.SwerveDrive.SwerveDrive_FrontLeftAngleMotor.getAbsPosition(),
        )
        wpilib.SmartDashboard.putNumber(
            "RF Speed", self.SwerveDrive.SwerveDrive_FrontRightSpeedMotor.getSpeed()
        )
        wpilib.SmartDashboard.putNumber(
            "RF Angle",
            self.SwerveDrive.SwerveDrive_FrontRightAngleMotor.getAbsPosition(),
        )
        wpilib.SmartDashboard.putNumber(
            "LR Speed", self.SwerveDrive.SwerveDrive_RearLeftSpeedMotor.getSpeed()
        )
        wpilib.SmartDashboard.putNumber(
            "LR Angle", self.SwerveDrive.SwerveDrive_RearLeftAngleMotor.getAbsPosition()
        )
        wpilib.SmartDashboard.putNumber(
            "RR Speed", self.SwerveDrive.SwerveDrive_RearRightSpeedMotor.getSpeed()
        )
        wpilib.SmartDashboard.putNumber(
            "RR Angle",
            self.SwerveDrive.SwerveDrive_RearRightAngleMotor.getAbsPosition(),
        )


if __name__ == "__main__":
    wpilib.run(MyRobot)
