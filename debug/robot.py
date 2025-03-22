import wpilib
from magicbot import MagicRobot

# from config import RobotConfig

from phoenix6.hardware import Pigeon2

from hmi import HMI, HMIConfig
from intake import IntakeController, Intake

from intake import IntakeController, SparkMaxDualSpinner, SparkMaxPivot
from drivetrain import SwerveDrive, SwerveConfig

from vision import Vision
from vision import Limelight


class MyRobot(MagicRobot):

    # Configuration Class
    # RobotConfig = RobotConfig()

    # Swerve Drive Component Code
    SwerveDrive: SwerveDrive
    Swerve_config = SwerveConfig(fl_CAN=(1,2),           # (drive_id, turn_id)
                                 fl_zoffset=0.0,          # rad
                                 fl_loc=(0.381, 0.381),   # m
                                 fr_CAN=(3,4),            # (drive_id, turn_id)
                                 fr_zoffset=0.0,          # rad
                                 fr_loc=(0.381, -0.381),  # m
                                 rl_CAN=(5,6),            # (drive_id, turn_id)
                                 rl_zoffset=0.0,          # rad
                                 rl_loc=(-0.381, -0.381), # m
                                 rr_CAN=(7,8),            # (drive_id, turn_id)
                                 rr_zoffset=0.0,          # rad
                                 rr_loc=(-0.381, -0.381), # m
                                 wheel_diameter=4./0.0254,
                                 CAN_id_imu=11)           # IMU_id

    # Intake: Intake
    # IntakeController: IntakeController

    # Controller Component Code
    HMI: HMI
    HMI_config = HMIConfig(port_id=0)

    # Vision Code
    Vision: Vision

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
        self.SwerveDrive.clearFaults()
        pass

    def teleopPeriodic(self):
        """MyRobot.teleopPeriodic() -> None

        Called repeatedly during teleoperated mode."""
        # if self.Vision.getTargetDistance() is not None:
        #     print(self.Vision.getTargetDistance())
        # Move drivetrain based on Left X/Y and Right X/Y controller inputs
        
        Lx, Ly, Rx, _ = self.HMI.getAnalogSticks()
        self.SwerveDrive.move(Lx, Ly, Rx)

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
