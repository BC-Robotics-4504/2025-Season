import math
import rev

class SparkMaxPivot:
    """Swerve Drive SparkMax Class
    Custom class for configuring SparkMaxes used in Swerve Drive Drivetrain
    """

    # PID coefficients
    kP = 0.9
    kI = 0
    kD = 0
    kIz = 0.0
    kFF = 0
    kMaxOutput = 1
    kMinOutput = -1
    maxRPM = 5700

    # Smart Motion Coefficients
    maxVel = 2000  # rpm
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
        z_offset=0,
    ):
        self.canID = canID
        self.gear_ratio = gear_ratio
        self.inverted = inverted
        self.gear_ratio = gear_ratio
        self.wheel_diameter = wheel_diameter
        self.zOffset = z_offset

        self.motor = rev.SparkMax(self.canID, rev.SparkMax.MotorType.kBrushless)
        self.config = rev.SparkMaxConfig()

        self.config.inverted(inverted)
        self.config.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        self.config.smartCurrentLimit(40)
        
        # Limit Switch
        self.config.limitSwitch.setSparkMaxDataPortConfig()

        self.config.absoluteEncoder.setSparkMaxDataPortConfig()
        self.config.absoluteEncoder.inverted(not inverted)
        self.config.absoluteEncoder.endPulseUs(1024)
        self.config.absoluteEncoder.startPulseUs(1)
        self.config.absoluteEncoder.positionConversionFactor(2 * math.pi)

        # self.config.IdleMode(rev.SparkMax.IdleMode.kBrake)
        z_offset /= 2.0 * math.pi
        if z_offset < 0.0:
            z_offset += 1.0

        self.config.absoluteEncoder.zeroOffset(
            z_offset
        )  #!FIXME Causes code to crash with Invalid Parameter Runtime error

        self.config.absoluteEncoder.velocityConversionFactor(0.104719755119659771)

        self.config.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
        self.config.closedLoop.positionWrappingEnabled(False)
        self.config.closedLoop.positionWrappingMinInput(0)
        self.config.closedLoop.positionWrappingMaxInput(2 * math.pi / self.gear_ratio)

        # PID parameters
        self.config.closedLoop.pidf(self.kP, self.kI, self.kD, self.kFF)
        self.config.closedLoop.IZone(self.kIz)

        self.config.closedLoop.outputRange(
            self.kMinOutput, self.kMaxOutput, rev.ClosedLoopSlot.kSlot0
        )

        self.encoder = self.motor.getAbsoluteEncoder()
        self.controller = self.motor.getClosedLoopController()
        
        self.motor.configure(
            self.config,
            rev.SparkMax.ResetMode.kResetSafeParameters,
            rev.SparkMax.PersistMode.kPersistParameters,
        )

    def clearFaults(self):
        """SparkMaxPivot.clearFaults() -> None

        Clear the faults on the motor controller."""
        self.motor.clearFaults()

    def setPosition(self, position):
        """SparkMaxPivot.setPosition(position: float) -> None

        Set the position of the motor controller.

        ::params:
        position: float : The position to set the motor controller to."""
        self.target_position = position - self.zOffset
        self.controller.setReference(
            self.target_position, rev.SparkMax.ControlType.kPosition
        )
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