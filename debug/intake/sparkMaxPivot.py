from config import RobotConfig
import rev
import math

class SparkMaxPivot:
    """Swerve Drive SparkMax Class
    Custom class for configuring SparkMaxes used in Swerve Drive Drivetrain
    """
    
    # PID coefficients
    kP = 0.9
    kI = 0
    kD = 0
    kIz = 0.
    kFF = 0
    kMaxOutput = 1
    kMinOutput = -1
    maxRPM = 5700

    # Smart Motion Coefficients
    maxVel = 2000 # rpm
    maxAcc = 1500
    minVel = 0
    allowedErr = 0.01
    smartMotionSlot = 0

    target_position = 0

    def __init__(
        self,
        canID,
        inverted=False,
        gear_ratio=1,
        wheel_diameter=1,
        absolute_encoder=False,
        z_offset = 0,
        follower_canID = None,
    ):
        self.canID = canID
        self.follower_canID = follower_canID
        self.gear_ratio = gear_ratio
        self.inverted = inverted
        self.absolute = absolute_encoder
        self.gear_ratio = gear_ratio
        self.wheel_diameter = wheel_diameter
        self.zOffset = z_offset

        # Encoder parameters 
        # https://docs.reduxrobotics.com/canandcoder/spark-max
        # https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/subsystems/MAXSwerveModule.java

        self.motor = rev.SparkMax(self.canID, rev.SparkMax.MotorType.kBrushless)  
        # self.motor.restoreFactoryDefaults()
        self.motor.setInverted(inverted)
        # self.motor.setIdleMode(rev.SparkMax.IdleMode.kBrake)
        self.motor.setSmartCurrentLimit(40)

        self.SMcontroller = self.motor.getPIDController()
        self.encoder = self.motor.getAbsoluteEncoder(rev.SparkMaxAlternateEncoder..kDutyCycle)
        self.encoder.setInverted(inverted)
        self.encoder.setPositionConversionFactor(2*math.pi/self.gear_ratio)
        self.encoder.setVelocityConversionFactor(.104719755119659771)
        
        self.SMcontroller.setFeedbackDevice(self.encoder)
        self.SMcontroller.setPositionPIDWrappingEnabled(False) #TODO: does this need to be removed?
        self.SMcontroller.setPositionPIDWrappingMinInput(0) #TODO: does this need to be removed?
        self.SMcontroller.setPositionPIDWrappingMaxInput(2*math.pi/self.gear_ratio) #TODO: does this need to be removed?
        
        # self.SMcontroller.setSmartMotionMaxVelocity(self.maxVel, self.smartMotionSlot)
        # self.SMcontroller.setSmartMotionMinOutputVelocity(self.minVel, self.smartMotionSlot)
        # self.SMcontroller.setSmartMotionMaxAccel(self.maxAcc, self.smartMotionSlot)
        # self.SMcontroller.setSmartMotionAllowedClosedLoopError(self.allowedErr, self.smartMotionSlot)
        
        # PID parameters
        self.SMcontroller.setP(self.kP)
        self.SMcontroller.setI(self.kI)
        self.SMcontroller.setD(self.kD)
        self.SMcontroller.setIZone(self.kIz)
        self.SMcontroller.setFF(self.kFF)
        self.SMcontroller.setOutputRange(self.kMinOutput, self.kMaxOutput)
        
        # Setup follower
        if follower_canID is not None:
            follower_motor = rev.SparkMax(self.follower_canID, rev.SparkMax.MotorType.kBrushless) 
            # follower_motor.restoreFactoryDefaults()
            # follower_motor.setIdleMode(rev.SparkMax.IdleMode.kCoast)
            # follower_motor.setSmartCurrentLimit(40)
            follower_motor.follow(self.motor, invert=True)
            
            self.follower_motor = follower_motor
            
        else:
            self.follower_motor = None
    
    def clearFaults(self):
        """SparkMaxPivot.clearFaults() -> None
        
        Clear the faults on the motor controller."""
        self.motor.clearFaults()
    
    def setPosition(self, position):
        """SparkMaxPivot.setPosition(position: float) -> None
        
        Set the position of the motor controller.
        
        ::params:
        position: float : The position to set the motor controller to."""
        self.target_position = position-self.zOffset
        self.SMcontroller.setReference(self.target_position, rev.SparkMax.ControlType.kPosition)
        return False
    
    def getPosition(self):
        """SparkMaxPivot.getPosition() -> float
        
        Get the position of the motor controller."""
        rotation = self.encoder.getPosition()
        return rotation
    
    def atPosition(self, tolerance=0.05):
        """SparkMaxPivot.atPosition(tolerance: float) -> bool
        
        Check if the motor controller is at the target position."""
        err = self.target_position - self.getPosition()
        if abs(err) <= tolerance:
            return True
        return False
        