import math

import wpilib
import rev

from config import RobotConfig
    
class SparkMaxDualSpinner:
    """Swerve Drive SparkMax Class
    Custom class for configuring SparkMaxes used in Swerve Drive Drivetrain
    """
    
    # PID coefficients
    kP = 0.32
    kI = 1e-4
    kD = 1
    kIz = 0.30
    kFF = 0
    kMaxOutput = 1
    kMinOutput = -1
    maxRPM = 5700

    # Smart Motion Coefficients
    maxVel = 2000  # rpm
    maxAcc = 1000
    minVel = 0
    allowedErr = 0

    smartMotionSlot=0

    def __init__(
        self,
        canID,
        inverted=False,
        gear_ratio=1,
        wheel_diameter=1,
        absolute_encoder=False,
        z_offset = 0
    ):
        self.canID = canID
        self.gear_ratio = gear_ratio
        self.inverted = inverted
        self.absolute = absolute_encoder
        self.gear_ratio = gear_ratio
        self.wheel_diameter = wheel_diameter
        self.zOffset = z_offset

        self.motor = rev.CANSparkMax(self.canID, rev.CANSparkMax.MotorType.kBrushless)  
        self.motor.restoreFactoryDefaults()
        self.motor.setInverted(not inverted)
        self.motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.motor.setSmartCurrentLimit(25)

        self.SMcontroller = self.motor.getPIDController()
        self.encoder = self.motor.getEncoder()
        # self.encoder.setInverted(inverted)
        self.encoder.setVelocityConversionFactor(.104719755119659771)
        
        #self.controller.burnFlash()    
        self.clearFaults()
    
    def clearFaults(self):
        """SparkMaxDualSpinner.clearFaults() -> None
        
        Clear the faults on the motor controller."""
        self.motor.clearFaults()
    
    def setSpeed(self, speed):
        """SparkMaxDualSpinner.setSpeed(speed: float) -> None
        
        Set the speed of the motor controller.
        
        ::params:
        speed: float : The speed to set the motor controller to."""
    
        self.target_speed = speed
        self.motor.set(speed)
        return False
    
    def getSpeed(self):
        """SparkMaxDualSpinner.getSpeed() -> float
        
        Gets the current speed of the motor controller."""
        return self.encoder.getVelocity()
    
    def atSpeed(self, tolerance=0.02):
        """SparkMaxDualSpinner.atSpeed(tolerance: float) -> bool
        
        Check if the motor controller is at speed."""
        err = self.target_speed - self.getSpeed()
        if abs(err) <= tolerance:
            return True
        return False
    
