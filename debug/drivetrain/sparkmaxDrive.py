import rev
import math
from config import RobotConfig


class SparkMaxDriving:
    """Swerve Drive SparkMax Class
    Custom class for configuring SparkMaxes used in Swerve Drive Drivetrain
    """

    # PID coefficients
    kP0 = 6e-5
    kI0 = 0
    kD0 = 0
    kIz0 = 0
    kFF0 = 0.00015
    kMaxOutput0 = 3_500
    kMinOutput0 = -3_500

    kP1 = 1e-2
    kI1 = 1e-5
    kD1 = 0
    kIz1 = 0
    kFF1 = 0
    kMaxOutput1 = 2_000
    kMinOutput1 = -2_200

    maxRPM = 5700

    # Smart Motion Coefficients
    maxVel = 3500  # rpm
    maxAcc = 1000
    minVel = 0
    allowedErr = 0

    driveFactor = (RobotConfig.drive_wheel_diameter * math.pi) / (
        RobotConfig.drivingMotorReduction
    )

    targetDistance = 5.5
    tolerance = 0.1

    def __init__(
        self,
        canID,
        inverted=False,
        gear_ratio=1,
        wheel_diameter=1,
        absolute_encoder=False,
    ):
        self.canID = canID
        self.gear_ratio = gear_ratio
        self.inverted = inverted
        self.absolute = absolute_encoder
        self.gear_ratio = gear_ratio
        self.wheel_diameter = wheel_diameter

        # Encoder parameters
        # https://docs.reduxrobotics.com/canandcoder/spark-max
        # https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/subsystems/MAXSwerveModule.java

        self.motor = rev.SparkMax(self.canID, rev.SparkMax.MotorType.kBrushless)
        self.config = rev.SparkMaxConfig()

        self.config.smartCurrentLimit(60)
        self.config.absoluteEncoder.positionConversionFactor(0.05077956125529683)

        self.config.absoluteEncoder.velocityConversionFactor(self.driveFactor)
        self.motor.getEncoder().setPosition(0)

        self.config.closedLoop.setFeedbackSensor(
            rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
        )
        self.config.closedLoop.pidf(
            self.kP0, self.kI0, self.kD0, self.kFF0, rev.ClosedLoopSlot.kSlot0
        )
        self.config.closedLoop.pidf(
            self.kP1, self.kI1, self.kD1, self.kFF1, rev.ClosedLoopSlot.kSlot1
        )

        self.motor.configure(
            self.config,
            rev.SparkMax.ResetMode.kResetSafeParameters,
            rev.SparkMax.PersistMode.kPersistParameters,
        )
        self.clearFaults()

    def clearFaults(self):
        """SparkMaxDriving.clearFaults()

        Clears the faults of the speed SparkMax"""
        self.motor.clearFaults()

    def getSpeed(self):
        """SparkMaxDriving.getSpeed()

        Gets the current speed of the swerve modules
        """
        vel = -self.motor.getEncoder().getVelocity  # rpm
        return vel

    def setSpeed(self, speed):
        """
        SparkMaxDriving.setSpeed()

        Sets the speed of the swerve modules"""
        # self.motor.set(speed)
        self.motor.getClosedLoopController().setReference(
            speed, rev.SparkMax.ControlType.kVelocity, rev.ClosedLoopSlot.kSlot0
        )  # NOTE: Changed this.
        return None

    def atDistance(self):
        """SparkMaxDriving.atDistance()

        Checks if the robot has travlled to the specfied distance"""
        currentDistance = self.motor.getEncoder().getPosition()
        # print(currentDistance)
        if abs(currentDistance - self.targetDistance) <= self.tolerance:
            return True

        return False

    def setDistance(self, targetDistance: float):
        """SparkMaxDriving.setDistance()

        Sets a distance for the robot to travel

        ::params:
        targetDistance: Distance for the robot to travel
        """
        self.targetDistance = targetDistance
        self.motor.getClosedLoopController().setReference(
            targetDistance,
            rev.SparkMax.ControlType.kPosition,
            rev.ClosedLoopSlot.kSlot1,
        )
        return False

    def resetEncoder(self):
        self.motor.getEncoder().setPosition(0)
        return 0
