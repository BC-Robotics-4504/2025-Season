import rev


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

        self.motor = rev.CANSparkMax(canID, rev.CANSparkMax.MotorType.kBrushless)
        # self.motor.restoreFactoryDefaults()

        self.controller = self.motor.getPIDController()
        self.encoder = self.motor.getEncoder()

        self.encoder.setPositionConversionFactor(1)
        self.encoder.setVelocityConversionFactor(1)
        self.encoder.setPosition(0)

        # PID parameters
        self.controller.setP(self.kP0, slotID=0)
        self.controller.setI(self.kI0, slotID=0)
        self.controller.setD(self.kD0, slotID=0)
        self.controller.setFF(self.kFF0, slotID=0)
        self.controller.setOutputRange(self.kMinOutput0, self.kMaxOutput0, slotID=0)

        self.controller.setP(self.kP1, slotID=1)
        self.controller.setI(self.kI1, slotID=1)
        self.controller.setD(self.kD1, slotID=1)
        self.controller.setFF(self.kFF1, slotID=1)
        self.controller.setOutputRange(self.kMinOutput1, self.kMaxOutput1, slotID=1)

        # self.motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.motor.setSmartCurrentLimit(60)

        self.clearFaults()

    def clearFaults(self):
        """SparkMaxDriving.clearFaults()

        Clears the faults of the speed SparkMax"""
        self.motor.clearFaults()

    def getSpeed(self):
        """SparkMaxDriving.getSpeed()

        Gets the current speed of the swerve modules
        """
        vel = -self.encoder.getVelocity()  # rpm
        return vel

    def setSpeed(self, speed):
        """
        SparkMaxDriving.setSpeed()

        Sets the speed of the swerve modules"""
        # self.motor.set(speed)
        self.controller.setReference(
            speed, rev.CANSparkMax.ControlType.kVelocity, pidSlot=0
        )  # NOTE: Changed this.
        return None

    def atDistance(self):
        """SparkMaxDriving.atDistance()

        Checks if the robot has travlled to the specfied distance"""
        currentDistance = self.encoder.getPosition()
        print(currentDistance)
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
        self.controller.setReference(
            targetDistance, rev.CANSparkMax.ControlType.kPosition, pidSlot=1
        )
        return False

    def resetEncoder(self):
        self.encoder.setPosition(0)
        return 0
