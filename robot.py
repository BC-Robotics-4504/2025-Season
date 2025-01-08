import wpilib
from magicbot import MagicRobot

from config import RobotConfig

from hmi import HMI
from drivetrain import SwerveDrive


class MyRobot(MagicRobot):

    # Configuration Class
    RobotConfig = RobotConfig()
    
    # Swerve Drive Component Code
    SwerveDrive: SwerveDrive
    
    # Controller Component Code
    HMI: HMI

    controlGain: float = 1
    
    def createObjects(self):
        """MyRobot.createObjects() -> None
        
        Create motors and other hardware components here."""
        # Swerve Drive Hardware Config
        
        # HMI Hardware Config
        self.HMI_xbox = wpilib.XboxController(0)  
        pass

    # def disabledPeriodic(self):
    #     pass

    def teleopInit(self):
        """MyRobot.teleopInit() -> None
        
        Called once each time the robot enters teleoperated mode.
        """
        self.SwerveDrive.clearFaults()
        pass

    def teleopPeriodic(self):
        """MyRobot.teleopPeriodic() -> None
        
        Called repeatedly during teleoperated mode."""
        # if self.Vision.getTargetDistance() is not None:
        #     print(self.Vision.getTargetDistance())
        # Move drivetrain based on Left X/Y and Right X/Y controller inputs
        Lx, Ly, Rx, _ = self.HMI.getAnalogSticks()
        
        # Rx *= self.controlGain
        Lx *= self.controlGain
        Ly *= self.controlGain

        self.SwerveDrive.move(Lx, Ly, Rx)

        '''
        SmartDashboard Setup
        '''

        # Add stuff to SmartDashboard
        wpilib.SmartDashboard.putNumber('LF Speed', self.SwerveDrive.SwerveDrive_FrontLeftSpeedMotor.getSpeed())
        wpilib.SmartDashboard.putNumber('LF Angle', self.SwerveDrive.SwerveDrive_FrontLeftAngleMotor.getAbsPosition())
        wpilib.SmartDashboard.putNumber('RF Speed', self.SwerveDrive.SwerveDrive_FrontRightSpeedMotor.getSpeed())
        wpilib.SmartDashboard.putNumber('RF Angle', self.SwerveDrive.SwerveDrive_FrontRightAngleMotor.getAbsPosition())
        wpilib.SmartDashboard.putNumber('LR Speed', self.SwerveDrive.SwerveDrive_RearLeftSpeedMotor.getSpeed())
        wpilib.SmartDashboard.putNumber('LR Angle', self.SwerveDrive.SwerveDrive_RearLeftAngleMotor.getAbsPosition())
        wpilib.SmartDashboard.putNumber('RR Speed', self.SwerveDrive.SwerveDrive_RearRightSpeedMotor.getSpeed())
        wpilib.SmartDashboard.putNumber('RR Angle', self.SwerveDrive.SwerveDrive_RearRightAngleMotor.getAbsPosition())

if __name__ == "__main__":
    wpilib.run(MyRobot)
