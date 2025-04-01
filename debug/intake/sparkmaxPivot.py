import math
import rev

class SparkMaxPivot:
    """Swerve Drive SparkMax Class
    Custom class for configuring SparkMaxes used in Swerve Drive Drivetrain
    """
    kP = 0.35
    kI = 0
    kD = 0
    kIz = 0
    kFF = 0.0
    kMaxOutput = 2 * math.pi
    kMinOutput = -2 * math.pi
    maxRPM = 5700

    # Smart Motion Coefficients
    maxVel = 1000  # rpm
    maxAcc = 2000
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

        self.config.smartCurrentLimit(60)

        self.config.absoluteEncoder.setSparkMaxDataPortConfig()
        # self.config.absoluteEncoder.inverted(inverted)
        self.config.absoluteEncoder.endPulseUs(1024)
        self.config.absoluteEncoder.startPulseUs(1)
        self.config.absoluteEncoder.positionConversionFactor(2*math.pi/self.gear_ratio)

        # self.config.IdleMode(rev.SparkMax.IdleMode.kBrake)
        # z_offset /= 2.0 * math.pi
        # if z_offset < 0.0:
        #     z_offset += 1.0

        # self.config.absoluteEncoder.zeroOffset(
        #     z_offset
        
        self.config.setIdleMode(self.config.IdleMode.kBrake)
        # )  #!FIXME Causes code to crash with Invalid Parameter Runtime error
        # #TODO: Configure Feedback Sensor dataport
        self.config.closedLoop.setFeedbackSensor(
            rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder
        )
        self.config.closedLoop.positionWrappingEnabled(True)
        self.config.closedLoop.positionWrappingMinInput(0)
        self.config.closedLoop.positionWrappingMaxInput(2 * math.pi)
        self.config.closedLoop.pidf(self.kP, self.kI, self.kD, self.kFF)
        self.config.closedLoop.outputRange(
            self.kMinOutput, 
            self.kMaxOutput, 
            rev.ClosedLoopSlot.kSlot0
        )
        
        self.encoder = self.motor.getAbsoluteEncoder()
        self.controller = self.motor.getClosedLoopController()
        
        self.motor.configure(
            self.config,
            rev.SparkMax.ResetMode.kResetSafeParameters,
            rev.SparkMax.PersistMode.kPersistParameters,
        )
        
        self.clearFaults()

    def clearFaults(self):
        """SparkMaxTurning.clearFaults()

        Clears the faults of the turning SparkMax
        """
        self.motor.clearFaults()

    def setPosition(self, position: float):
        """SparkMaxTurning.setAbsPosition()

        Sets the absoulute positon of the encoder"""
        self.controller.setReference(
            position, 
            rev._rev.SparkLowLevel.ControlType.kPosition, 
            rev.ClosedLoopSlot.kSlot0
        )
        return False

    def getPosition(self):
        """SparkMaxTurning.getAbsPosition()

        Gets the absolute positon of the encoder
        """
        return self.encoder.getPosition()
