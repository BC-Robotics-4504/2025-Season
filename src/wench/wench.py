from enum import Enum

from collections import namedtuple
from .sparkmaxWench import SparkMaxWench

WenchConfig = namedtuple('config',
                          ['CAN_id',
                           'zoffset',
                           'gear_ratio',
                           'up_angle',
                           'down_angle'])

class WenchPosition(Enum):
    UP = 1
    DOWN = 2

class Wench:
    config: WenchConfig
    wenchMotor: SparkMaxWench
    
    current_position: WenchPosition = WenchPosition.UP
    
    def __init__(self, config: WenchConfig) -> None:
        self.config = config
        
        
        self.wenchMotor = SparkMaxWench(
            self.config.CAN_id, 
            inverted=False, 
            absolute_encoder=True, 
            gear_ratio=self.config.gear_ratio
        )
        
    def setDown(self):
        if self.current_position != WenchPosition.DOWN:
            self.__setAngle__(self.config.down_angle)
            self.current_position = WenchPosition.DOWN
        return
    
    def setUp(self):
        if self.current_position != WenchPosition.UP:
            self.__setAngle__(self.config.up_angle)
            self.current_position = WenchPosition.UP
        return
        
    def __setAngle__(self, angle: float):
        self.wenchMotor.setPosition(angle)
        return False
    
    def clearFaults(self):
        self.wenchMotor.clearFaults()
        return None

    def execute(self):
        pass
