from dataclasses import dataclass

class RobotConfig(dataclass):
    
    # Robot Geometry Parameters
    chassis_length: float = 0.7366  # 29 in to m
    chassis_width: float = 0.7366  # 29 in to m
    
    # Drive Motor
    drivingMotorReduction: float = 8.14

