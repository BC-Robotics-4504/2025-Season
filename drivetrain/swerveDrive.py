import math
import rev
from drivetrain import SparkMaxDriving, SparkMaxTurning
from components.config import RobotConfig

from wpimath.filter import SlewRateLimiter


class SwerveDrive:
    """ SwerveDrive Class 

    """
    RobotConfig: RobotConfig

    FrontLeftAngleMotor: SparkMaxTurning
    __frontLeftAngle__: float = 0
    
    FrontLeftSpeedMotor: SparkMaxDriving 
    __frontLeftSpeed__: float = 0
    __frontLeftDistance__: float = 0
    
    RearLeftAngleMotor: SparkMaxTurning
    __rearLeftAngle__: float = 0
    
    RearLeftSpeedMotor: SparkMaxDriving
    __rearleftSpeed__: float = 0
    __rearLeftDistance__: float = 0
    
    RearRightAngleMotor: SparkMaxTurning
    __rearRightAngle__: float = 0

    RearRightSpeedMotor: SparkMaxDriving
    __rearRightSpeed__: float = 0
    __rearRightDistance__: float = 0 
    
    FrontRightAngleMotor: SparkMaxTurning
    __frontRightAngle__: float = 0

    FrontRightSpeedMotor: SparkMaxDriving
    __frontRightSpeed__: float = 0
    __frontRightDistance__: float = 0
    move_changed: bool = False
    distance_changed: bool = False

    LxSlewRateLimiter = SlewRateLimiter(0.5)
    LySlewRateLimiter = SlewRateLimiter(0.5)
    RxSlewRateLimiter = SlewRateLimiter(1)
    
    def clearFaults(self):
        """SwerveDrive.clearFaults()
        Clears faults on all of the SparkMax modules
        """
        self.FrontLeftAngleMotor.clearFaults()
        self.FrontLeftSpeedMotor.clearFaults()
        self.RearLeftAngleMotor.clearFaults()
        self.RearLeftSpeedMotor.clearFaults()
        self.RearRightAngleMotor.clearFaults()
        self.RearRightSpeedMotor.clearFaults()
        self.FrontRightAngleMotor.clearFaults()
        self.FrontRightSpeedMotor.clearFaults()
        return False

    def move(self, Lx: float, Ly: float, Rx: float): 
        """SwerveDrive.move(Lx: float, Ly: float, Rx: float)

        ::params::
        Lx: Magnitude to move in the x direction in range, negative is forward [-1, 1]
        Ly: Magnitude to move in the y direction in range, positive is left [-1, 1]
        Rx: Magnitude of rotational speed where counter-clockwise is positive [-1, 1]
        """
        # Rx = self.RxSlewRateLimiter.calculate(Rx)
        # Check negatives and positives here for Lx, Ly, and Rx
        Vx0 = -Lx*self.RobotConfig.max_driving_speed*self.RobotConfig.drive_wheel_diameter*math.pi
        Vy0 = Ly*self.RobotConfig.max_driving_speed*self.RobotConfig.drive_wheel_diameter*math.pi
        w0 = -Rx*RobotConfig.max_angular_speed*math.pi
        
        # Calculate component vectors for the swerve modeule
        Vxp = Vx0 + w0*self.RobotConfig.chassis_length
        Vxn = Vx0 - w0*self.RobotConfig.chassis_length
        Vyp = Vy0 + w0*self.RobotConfig.chassis_width
        Vyn = Vy0 - w0*self.RobotConfig.chassis_width

        # Calculate the speed and angle for each swerve motor
        self.__frontLeftAngle__ = math.atan2(Vyp, Vxp)
        self.__frontLeftSpeed__ = math.hypot(Vyp, Vxp)/(math.pi*self.RobotConfig.drive_wheel_diameter)

        self.__rearLeftAngle__ = math.atan2(Vyp, Vxn)
        self.__rearLeftSpeed__ = math.hypot(Vyp, Vxn)/(math.pi*self.RobotConfig.drive_wheel_diameter)

        self.__rearRightAngle__ = math.atan2(Vyn, Vxn)
        self.__rearRightSpeed__ = math.hypot(Vyn, Vxn)/(math.pi*self.RobotConfig.drive_wheel_diameter)

        self.__frontRightAngle__ = math.atan2(Vyn, Vxp)
        self.__frontRightSpeed__ = math.hypot(Vyn, Vxp)/(math.pi*self.RobotConfig.drive_wheel_diameter)

        # Normalizes speed to maximum allowable speed for each wheel
        self.normalizeSpeeds()

        # Enable the move to be changed when "execute()" is run
        self.move_changed = True
        
        return False
    
    def goDistance(self, Rx0: float, Ry0: float, r0: float):
        """SwerveDrive.gotDistance(target_distance: float, target_angle: float, target_rotations: float)
        
        Calculates the angle and speed for a robot to move autonomusly a certain distance and at a target angle. 

        ::params::
        target_distance: The target distance the robot should travel autonomously range[0, 2*math.pi]
        target_angle: the target angle the robot should be at once the autonomous movement ends
        target_rotations: How many time should the robot spin before the autonomous movement ends.

        """
        Rx0 *= self.RobotConfig.drive_wheel_diameter*math.pi
        Ry0 *= self.RobotConfig.drive_wheel_diameter*math.pi
        r0 *= -math.pi
        
        # Calculate component vectors for the swerve modeule
        Xp = Rx0 + r0*self.RobotConfig.chassis_length
        Xn = Rx0 - r0*self.RobotConfig.chassis_length
        Yp = Ry0 + r0*self.RobotConfig.chassis_width
        Yn = Ry0 - r0*self.RobotConfig.chassis_width        
        
        self.__frontLeftAngle__ = math.atan2(Yp, Xp)+math.pi/2
        self.__frontLeftDistance__ = math.hypot(Yp, Xp)/(math.pi*self.RobotConfig.drive_wheel_diameter)

        self.__rearLeftAngle__ = math.atan2(Yp, Xn)+math.pi/2
        self.__rearLeftDistance__ = math.hypot(Yp, Xn)/(math.pi*self.RobotConfig.drive_wheel_diameter)

        self.__rearRightAngle__ = math.atan2(Yn, Xn)+math.pi/2
        self.__rearRightDistance__ = math.hypot(Yn, Xn)/(math.pi*self.RobotConfig.drive_wheel_diameter)

        self.__frontRightAngle__ = math.atan2(Yn, Xp)+math.pi/2
        self.__frontRightDistance__ = math.hypot(Yn, Xp)/(math.pi*self.RobotConfig.drive_wheel_diameter)
                
        self.distance_changed = True
        
    def resetEncoders(self):
        self.RearLeftSpeedMotor.resetEncoder()
        self.RearRightSpeedMotor.resetEncoder()
        self.FrontLeftSpeedMotor.resetEncoder()
        self.FrontRightSpeedMotor.resetEncoder()
        
    def clampSpeed(self):
        """SwerveDrive.clampSpeed()
        
        Clamps the speed of the swerve drive"""
        self.__frontLeftSpeed__ *= self.RobotConfig.speed_clamp
        self.__frontRightSpeed__ *= self.RobotConfig.speed_clamp
        self.__rearLeftSpeed__ *= self.RobotConfig.speed_clamp
        self.__rearRightSpeed__ *= self.RobotConfig.speed_clamp
        return None
    
    def normalizeSpeeds(self):
        """SwerveDrive.normalizeSpeeds()

        Normalizes the speed vectors for each wheel"""
        maxSpeed = max([self.__frontLeftSpeed__, self.__frontRightSpeed__,
                        self.__rearLeftSpeed__, self.__rearRightSpeed__])
        if maxSpeed > self.RobotConfig.max_driving_speed:
            scalar = self.RobotConfig.max_driving_speed/maxSpeed
            self.__frontLeftSpeed__ *= scalar
            self.__frontRightSpeed__ *= scalar
            self.__rearRightSpeed__ *= scalar
            self.__rearLeftSpeed__ *= scalar
            return True
        
        return False

    def atDistance(self):
        """SwerveDrive.atDistance()

        Checks if each wheel has traveled a specified distance"""
        FL = self.FrontLeftSpeedMotor.atDistance()
        FR = self.FrontRightSpeedMotor.atDistance()
        RL = self.RearLeftSpeedMotor.atDistance()
        RR = self.RearRightSpeedMotor.atDistance()

        if FL and FR and RL and RR:
            return True
        
        return False
    
    def closestAngle(current_angle: float, setpoint: float):
        """SwerveDrive.closestAngle()

        Calculates the shortest direction to turn from a current angle to a desired setpoint angle.
        
        ::params: 
        current_angle: The current angle of the swerve module.
        setpoint: The desired angle for the swerve module to move to.
        """
        #get direction
        two_pi = math.pi*2
        dir = setpoint%(two_pi) - current_angle%(two_pi)

        # convert from -2*pi to 2*pi to -pi to pi
        if abs(dir) > math.pi:
            dir = -math.copysign(two_pi, dir) + dir
        return dir
        
    def execute(self):

        """SwerveDrive.execute()
        Updates the postion of the absolute encoders and the speed of each swerve module"""

        if self.move_changed:

            # self.clampSpeed()

            self.FrontLeftAngleMotor.setAbsPosition(self.__frontLeftAngle__)
            self.FrontLeftSpeedMotor.setSpeed(self.__frontLeftSpeed__) 

            self.RearLeftAngleMotor.setAbsPosition(self.__rearLeftAngle__)
            self.RearLeftSpeedMotor.setSpeed(self.__rearLeftSpeed__) 

            self.RearRightAngleMotor.setAbsPosition(self.__rearRightAngle__)
            self.RearRightSpeedMotor.setSpeed(self.__rearRightSpeed__) 
            
            self.FrontRightAngleMotor.setAbsPosition(self.__frontRightAngle__)
            self.FrontRightSpeedMotor.setSpeed(self.__frontRightSpeed__) 

            self.move_changed = False
        
        if self.distance_changed:
            
            self.FrontLeftSpeedMotor.setDistance(self.__frontLeftDistance__) 
            self.FrontLeftAngleMotor.setAbsPosition(self.__frontLeftAngle__)

            self.RearLeftSpeedMotor.setDistance(self.__rearLeftDistance__) 
            self.RearLeftAngleMotor.setAbsPosition(self.__rearLeftAngle__)

            self.RearRightSpeedMotor.setDistance(self.__rearRightDistance__) 
            self.RearRightAngleMotor.setAbsPosition(self.__rearRightAngle__)

            self.FrontRightSpeedMotor.setDistance(self.__frontRightDistance__) 
            self.FrontRightAngleMotor.setAbsPosition(self.__frontRightAngle__)

            self.distance_changed = False