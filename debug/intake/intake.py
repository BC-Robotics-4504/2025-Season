from enum import Enum

from collections import namedtuple
from .sparkmaxPivot import SparkMaxPivot
from .sparkmaxSpinner import SparkMaxSpinner

IntakeConfig = namedtuple('config',
                          ['CAN_ids',
                           'pivot_zoffset',
                           'pivot_gear_ratio',
                           'up_angle',
                           'down_angle',
                           'spinner_speed'])

class PivotPosition(Enum):
    UP = 1
    DOWN = 2
    
class SpinnerSpeed(Enum):
    ON = 1
    OFF = 2

class Intake:
    
    config: IntakeConfig
    spinnerMotor: SparkMaxSpinner
    pivotMotor: SparkMaxPivot
    
    current_position: PivotPosition = PivotPosition.UP
    current_speed: SpinnerSpeed = SpinnerSpeed.OFF
    
    def __init__(self, config: IntakeConfig) -> None:
        self.config = config
        
        
        self.pivotMotor = SparkMaxPivot(
            self.config.CAN_ids[0], 
            inverted=False, 
            z_offset=self.config.pivot_zoffset,
            gear_ratio=self.config.pivot_gear_ratio
        )
    
        self.spinnerMotor = SparkMaxSpinner(
            self.config.CAN_ids[1],
            inverted=False
        )
        
    def setDown(self):
        if self.current_position != PivotPosition.DOWN:
            self.__setAngle__(self.config.down_angle)
            self.current_position = PivotPosition.DOWN
        
        if self.current_speed != SpinnerSpeed.ON:
            self.__setSpeed__(self.config.spinner_speed)
            self.current_speed = SpinnerSpeed.ON
    
    def setUp(self):
        if self.current_position != PivotPosition.UP:
            self.__setAngle__(self.config.up_angle)
            self.current_position = PivotPosition.UP
        
        if self.current_speed != SpinnerSpeed.OFF:
            self.__setSpeed__(0)
            self.current_speed = SpinnerSpeed.OFF
    
    def setSpin(self):
        if self.current_speed != SpinnerSpeed.ON:
            self.__setSpeed__(self.config.spinner_speed)
            self.current_speed = SpinnerSpeed.ON
    
    def resetSpin(self):
        if self.current_speed != SpinnerSpeed.OFF:
            self.__setSpeed__(0)
            self.current_speed = SpinnerSpeed.OFF
        
    def __setAngle__(self, angle: float):
        self.pivotMotor.setPosition(angle*self.config.pivot_gear_ratio)
        return False
    
    def __setSpeed__(self, speed: float):
        self.spinnerMotor.setSpeed(speed)
    
    def clearFaults(self):
        self.spinnerMotor.clearFaults()
        self.pivotMotor.clearFaults()
        return None
    
    def resetEncoder(self):
        self.spinnerMotor.getEncoder().Position(0)
        
    def execute(self):
        return
        
