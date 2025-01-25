import wpilib
import math
import rev



class SparkMaxTurning:
    """Swerve Drive SparkMax Class
    Custom class for configuring SparkMaxes used in Swerve Drive Drivetrain
    """

    # PID coefficients
    kP = 0
    kI = 0
    kD = 0
    kIz = 0
    kFF = 1.
    kMaxOutput = math.pi
    kMinOutput = -math.pi
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
        
        # self.config.inverted(not inverted)
        self.config.smartCurrentLimit(40)
        
        self.config.absoluteEncoder.setSparkMaxDataPortConfig()
        self.config.absoluteEncoder.inverted(not inverted)
        self.config.absoluteEncoder.endPulseUs(4096)
        self.config.absoluteEncoder.startPulseUs(1)
        self.config.absoluteEncoder.positionConversionFactor(1.0)
        self.config.absoluteEncoder.velocityConversionFactor(0.104719755119659771)
        self.encoder = self.motor.getAbsoluteEncoder()

        self.config.IdleMode(rev.SparkMax.IdleMode.kBrake)
   
        # self.config.absoluteEncoder.zeroOffset(z_offset) #!FIXME Causes code to crash with Invalid Parameter Runtime error
        #TODO: Configure Feedback Sensor dataport
        self.config.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
        # self.config.closedLoop.positionWrappingEnabled(True)
        self.config.closedLoop.positionWrappingMinInput(-math.pi)
        self.config.closedLoop.positionWrappingMaxInput(math.pi)
        self.config.closedLoop.pidf(self.kP, self.kI, self.kD, self.kFF)
        self.config.closedLoop.IZone(self.kIz) 
        self.config.closedLoop.outputRange(self.kMinOutput, self.kMaxOutput)
        self.motor.configure(self.config, rev.SparkMax.ResetMode.kResetSafeParameters, rev.SparkMax.PersistMode.kPersistParameters)
        self.controller = self.motor.getClosedLoopController()
        self.clearFaults()
        
    def clearFaults(self):
        """SparkMaxTurning.clearFaults()

        Clears the faults of the turning SparkMax
        """
        self.motor.clearFaults()

    def setAbsPosition(self, position: float):
        """SparkMaxTurning.setAbsPosition()

        Sets the absoulute positon of the encoder"""
        self.controller.setReference(position, rev._rev.SparkLowLevel.ControlType.kPosition)
        return False

    def getAbsPosition(self):
        """SparkMaxTurning.getAbsPosition()

        Gets the absolute positon of the encoder
        """
        rotation = self.encoder.getPosition()
        return rotation
