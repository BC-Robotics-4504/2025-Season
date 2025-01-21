import wpilib
import math
import rev



class SparkMaxTurning:
    """Swerve Drive SparkMax Class
    Custom class for configuring SparkMaxes used in Swerve Drive Drivetrain
    """

    # PID coefficients
    kP = 0.25
    kI = 0
    kD = 0
    kIz = 0
    kFF = 0
    kMaxOutput = 2 * math.pi
    kMinOutput = -2 * math.pi
    maxRPM = 5700

    # Smart Motion Coefficients
    maxVel = 2000  # rpm
    maxAcc = 1000
    minVel = 0
    allowedErr = 0

    smartMotionSlot = 0

    def __init__(
        self,
        canID,
        inverted=False,
        gear_ratio=1,
        wheel_diameter=1,
        absolute_encoder=False,
        z_offset=0,
    ):
        self.canID = canID
        self.gear_ratio = gear_ratio
        self.inverted = inverted
        self.absolute = absolute_encoder
        self.gear_ratio = gear_ratio
        self.wheel_diameter = wheel_diameter
        self.zOffset = z_offset
        
        self.motor = rev.SparkMax(self.canID, rev.SparkMax.MotorType.kBrushless)
        self.config = rev.SparkMaxConfig()
        
        self.config.inverted(not inverted)
        self.config.smartCurrentLimit(40)
        
        self.config.absoluteEncoder.inverted(inverted)
        self.config.absoluteEncoder.positionConversionFactor(2*math.pi)
        self.config.absoluteEncoder.velocityConversionFactor(0.104719755119659771)
        # self.config.absoluteEncoder.zeroOffset(z_offset) #!FIXME Causes code to crash with Invalid Parameter Runtime error
        
        self.config.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
        self.config.closedLoop.positionWrappingEnabled(True)
        self.config.closedLoop.positionWrappingMinInput(0)
        self.config.closedLoop.positionWrappingMaxInput(2*math.pi)
        self.config.closedLoop.pidf(self.kP, self.kI, self.kD, self.kFF)
        self.config.closedLoop.IZone(self.kIz) 
        self.config.closedLoop.outputRange(self.kMinOutput, self.kMaxOutput)
        print("==========================================")
        self.motor.configure(self.config, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
        print("xxxxxxxxxxxxxxxxxxxxxxxxAfterxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")

        self.clearFaults()
    def clearFaults(self):
        """SparkMaxTurning.clearFaults()

        Clears the faults of the turning SparkMax
        """
        self.motor.clearFaults()

    def setAbsPosition(self, position: float):
        """SparkMaxTurning.setAbsPosition()

        Sets the absoulute positon of the encoder"""
        # self.encoder.(position, rev.SparkMax.ControlType.kPosition)
        self.motor.getEncoder().setPosition(position)
        return False

    def getAbsPosition(self):
        """SparkMaxTurning.getAbsPosition()

        Gets the absolute positon of the encoder
        """
        rotation = self.motor.getEncoder().getPosition()
        return rotation
