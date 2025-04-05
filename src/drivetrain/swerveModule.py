import rev
import math

class SwerveModule:
    turnMotor: rev.SparkMax
    driveMotor: rev.SparkMax
    
    def __init__(self, CAN_ids:tuple[int, int], turn_zoffset, drive_wheel_diameter):
        self.driveMotor = SparkMaxDriving(
            CAN_ids[0], 
            inverted=False, 
            wheel_diameter=drive_wheel_diameter
        )
        
        self.turnMotor = SparkMaxTurning(
            CAN_ids[1], 
            inverted=False, 
            wheel_diameter=drive_wheel_diameter, 
            absolute_encoder=True, 
            z_offset=turn_zoffset
        )
        
    def setAngle(self, position: float):
        self.turnMotor.setAbsPosition(position)
        return False

    def getAngle(self):
        return self.turnMotor.getAbsPosition()
    
    def atDistance(self):
        return self.driveMotor.atDistance()
    
    def setDistance(self, distance):
        return self.driveMotor.setDistance(distance)
    
    def getSpeed(self) -> int:
        return self.driveMotor.getSpeed()

    def setSpeed(self, speed:float):
        self.driveMotor.setSpeed(speed)
        return None
    
    def clearFaults(self):
        self.driveMotor.clearFaults()
        self.turnMotor.clearFaults()
        
        return None
    
    def resetEncoder(self):
        """Reset Swerve Module Encoders
        
        returns None"""
        self.driveMotor.getEncoder().setPosition(0)
        self.turnMotor.getEncoder().setPosition(0)



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

        self.motor = rev.SparkMax(self.canID, rev.SparkMax.MotorType.kBrushless)
        self.config = rev.SparkMaxConfig()
        
        self.config.smartCurrentLimit(60)
        self.config.absoluteEncoder.positionConversionFactor(0.05077956125529683)

        self.config.absoluteEncoder.velocityConversionFactor(0.0008463260209216138) #TODO: update me!!!!
        self.motor.getEncoder().setPosition(0)

        self.config.closedLoop.setFeedbackSensor(
            rev.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder
        )
        
        self.config.closedLoop.pidf(
            self.kP0, self.kI0, self.kD0, self.kFF0, rev.ClosedLoopSlot.kSlot0
        )

        self.motor.configure(
            self.config,
            rev.SparkMax.ResetMode.kResetSafeParameters,
            rev.SparkMax.PersistMode.kNoPersistParameters,
        )
        
        self.encoder = self.motor.getAbsoluteEncoder()
        self.controller = self.motor.getClosedLoopController()
        self.clearFaults()

    def clearFaults(self):
        """SparkMaxDriving.clearFaults()

        Clears the faults of the speed SparkMax"""
        self.motor.clearFaults()

    def getSpeed(self):
        """SparkMaxDriving.getSpeed()

        Gets the current speed of the swerve modules
        """
        vel = -self.motor.getEncoder().getVelocity()  # rpm
        return vel

    def setSpeed(self, speed):
        """
        SparkMaxDriving.setSpeed()

        Sets the speed of the swerve modules"""
        # self.motor.set(speed)
        self.controller.setReference(
            speed, 
            rev.SparkMax.ControlType.kVelocity, 
            rev.ClosedLoopSlot.kSlot0
        )  # NOTE: Changed this.
        return None
    
    def getEncoder(self):
        return self.motor.getEncoder()
    
    def getDistance(self):
        return self.encoder.getPosition()

    def atDistance(self) -> None:
        """SparkMaxDriving.atDistance()

        Checks if the robot has travlled to the specfied distance"""
        currentDistance = self.encoder.getPosition()
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
        self.controller.setReference(
            targetDistance,
            rev.SparkMax.ControlType.kPosition,
            rev.ClosedLoopSlot.kSlot1,
        )
        return False

    def resetEncoder(self):
        self.motor.getEncoder().setPosition(0)
        return 0

class SparkMaxTurning:
    """Swerve Drive SparkMax Class
    Custom class for configuring SparkMaxes used in Swerve Drive Drivetrain
    """

    # PID coefficients
    kP = 0.25
    kI = 0
    kD = 0
    kIz = 0
    kFF = 0.0
    kMaxOutput = math.pi
    kMinOutput = -math.pi
    maxRPM = 5700

    # Smart Motion Coefficients
    maxVel = 3000  # rpm
    maxAcc = 3000
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
        self.motor.setInverted(True)
        self.config = rev.SparkMaxConfig()

        self.config.smartCurrentLimit(40)

        self.config.absoluteEncoder.setSparkMaxDataPortConfig()
        self.config.absoluteEncoder.inverted(True)
        # self.config.absoluteEncoder.endPulseUs(1024)
        # self.config.absoluteEncoder.startPulseUs(1)
        
        self.config.absoluteEncoder.positionConversionFactor(2*math.pi)
        self.config.absoluteEncoder.velocityConversionFactor(.104719755119659771)

        self.config.IdleMode(rev.SparkMax.IdleMode.kBrake)
        z_offset /= 2.0 * math.pi
        if z_offset < 0.0:
            z_offset += 1.0
        
        self.config.absoluteEncoder.zeroOffset(z_offset)  
        self.config.absoluteEncoder.zeroCentered(True)
        self.config.closedLoop.setFeedbackSensor(
            rev.ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder
        )
        self.config.closedLoop.positionWrappingEnabled(True)
        self.config.closedLoop.positionWrappingMinInput(0)
        self.config.closedLoop.positionWrappingMaxInput(2*math.pi)
        self.config.closedLoop.pidf(self.kP, 
                                    self.kI, 
                                    self.kD, 
                                    self.kFF,
                                    rev.ClosedLoopSlot.kSlot0)
        self.config.closedLoop.outputRange(
            self.kMinOutput, 
            self.kMaxOutput, 
            rev.ClosedLoopSlot.kSlot0
        )

        self.motor.configure(
            self.config,
            rev.SparkMax.ResetMode.kResetSafeParameters,
            rev.SparkMax.PersistMode.kNoPersistParameters
        )
        self.encoder = self.motor.getAbsoluteEncoder()
        self.controller = self.motor.getClosedLoopController()
        self.clearFaults()

    def clearFaults(self):
        """SparkMaxTurning.clearFaults()

        Clears the faults of the turning SparkMax
        """
        self.motor.clearFaults()
    
    def getEncoder(self):
        return self.motor.getEncoder()

    def setAbsPosition(self, position: float):
        """SparkMaxTurning.setAbsPosition()

        Sets the absoulute positon of the encoder"""
        self.controller.setReference(
            position, 
            rev.SparkLowLevel.ControlType.kPosition
        )
        return False

    def getAbsPosition(self):
        """SparkMaxTurning.getAbsPosition()

        Gets the absolute positon of the encoder
        """
        
        return self.encoder.getPosition()
