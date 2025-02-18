import rev
import wpilib

from config import RobotConfig

import math


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
        absolute_encoder=False,
        z_offset=0,
        follower_canID=None,
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
        self.config = rev.SparkMaxConfig()

        self.config.inverted(inverted)
        self.config.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        self.config.smartCurrentLimit(40)
        
        self.config.limitSwitch.setSparkMaxDataPortConfig()
        

        self.encoder = self.motor.getAbsoluteEncoder()
        self.controller = self.motor.getClosedLoopController()

        self.config.absoluteEncoder.inverted(inverted)
        self.config.absoluteEncoder.positionConversionFactor(
            2 * math.pi / self.gear_ratio
        )

        self.config.absoluteEncoder.velocityConversionFactor(0.104719755119659771)

        self.config.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
        self.config.closedLoop.positionWrappingEnabled(False)
        self.config.closedLoop.positionWrappingMinInput(0)
        self.config.closedLoop.positionWrappingMaxInput(2 * math.pi / self.gear_ratio)

        # self.SMcontroller.setSmartMotionMaxVelocity(self.maxVel, self.smartMotionSlot)
        # self.SMcontroller.setSmartMotionMinOutputVelocity(self.minVel, self.smartMotionSlot)
        # self.SMcontroller.setSmartMotionMaxAccel(self.maxAcc, self.smartMotionSlot)
        # self.SMcontroller.setSmartMotionAllowedClosedLoopError(self.allowedErr, self.smartMotionSlot)

        # PID parameters
        self.config.closedLoop.pidf(self.kP, self.kI, self.kD, self.kFF)
        self.config.closedLoop.IZone(self.kIz)

        self.config.closedLoop.outputRange(
            self.kMinOutput, self.kMaxOutput, rev.ClosedLoopSlot.kSlot0
        )

        self.motor.configure(
            self.config,
            rev.SparkMax.ResetMode.kResetSafeParameters,
            rev.SparkMax.PersistMode.kPersistParameters,
        )

        # Setup follower
        if follower_canID is not None:
            follower_motor = rev.SparkMax(
                self.follower_canID, rev.SparkMax.MotorType.kBrushless
            )
            self.followerConfig = rev.SparkMaxConfig()
            self.followerConfig.follow(self.motor, invert=True)
            self.motor.configure(
                self.followerConfig,
                rev.SparkMax.ResetMode.kResetSafeParameters,
                rev.SparkMax.PersistMode.kPersistParameters,
            )

            # follower_motor.setIdleMode(rev.CANSparkMax.IdleMode.kCoast)
            # follower_motor.setSmartCurrentLimit(40)

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
    
    def getLimitSwitch(self):
        return self.motor.getForwardLimitSwitch().get()

    def atPosition(self, tolerance=0.05):
        """SparkMaxPivot.atPosition(tolerance: float) -> bool

        Check if the motor controller is at the target position."""
        err = self.target_position - self.getPosition()
        if abs(err) <= tolerance:
            return True
        return False
