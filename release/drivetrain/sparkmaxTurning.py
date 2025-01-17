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

        # Encoder parameters
        # https://docs.reduxrobotics.com/canandcoder/spark-max
        # https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/subsystems/MAXSwerveModule.java

        self.motor = rev.CANSparkMax(self.canID, rev.CANSparkMax.MotorType.kBrushless)
        self.motor.restoreFactoryDefaults()
        self.motor.setInverted(not inverted)
        self.motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.motor.setSmartCurrentLimit(40)

        self.SMcontroller = self.motor.getPIDController()
        self.encoder = self.motor.getAbsoluteEncoder(
            rev.SparkMaxAbsoluteEncoder.Type.kDutyCycle
        )
        self.encoder.setInverted(inverted)
        self.encoder.setPositionConversionFactor(2 * math.pi)
        self.encoder.setVelocityConversionFactor(0.104719755119659771)
        self.encoder.setZeroOffset(z_offset)

        self.SMcontroller.setFeedbackDevice(self.encoder)
        self.SMcontroller.setPositionPIDWrappingEnabled(
            True
        )  # TODO: does this need to be removed?
        self.SMcontroller.setPositionPIDWrappingMinInput(
            0
        )  # TODO: does this need to be removed?
        self.SMcontroller.setPositionPIDWrappingMaxInput(
            2 * math.pi
        )  # TODO: does this need to be removed?

        # PID parameters
        self.SMcontroller.setP(self.kP)
        self.SMcontroller.setI(self.kI)
        self.SMcontroller.setD(self.kD)
        self.SMcontroller.setIZone(self.kIz)
        self.SMcontroller.setFF(self.kFF)
        self.SMcontroller.setOutputRange(self.kMinOutput, self.kMaxOutput)

        # self.controller.burnFlash()
        self.clearFaults()

    def clearFaults(self):
        """SparkMaxTurning.clearFaults()

        Clears the faults of the turning SparkMax
        """
        self.motor.clearFaults()

    def setAbsPosition(self, position):
        """SparkMaxTurning.setAbsPosition()

        Sets the absoulute positon of the encoder"""
        self.SMcontroller.setReference(position, rev.CANSparkMax.ControlType.kPosition)
        return False

    def getAbsPosition(self):
        """SparkMaxTurning.getAbsPosition()

        Gets the absolute positon of the encoder
        """
        rotation = self.encoder.getPosition()
        return rotation
