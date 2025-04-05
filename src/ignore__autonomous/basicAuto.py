import math
from magicbot import AutonomousStateMachine, timed_state, state
from time import time
from drivetrain.swerveDrive import SwerveDrive
from wpilib import Timer
# this is one of your components
def distance_to_rotation(distance, wheel_diamter=0.1016):
    return distance / (math.pi*wheel_diamter)
    
DISTANCE1 = 1 # m

class DefaultAuto(AutonomousStateMachine):
     
    swerve : SwerveDrive
    MODE_NAME = "Default Auto"
    DEFAULT = True
    timer = Timer()
    timer.start()

    # Injected from the definition in robot.py

    @state(first=True)
    def start(self):
       self.engage()
       self.next_state('__setup__')
       
    @state(must_finish=True)
    def __setup__(self):
       self.swerve.resetEncoders()
       self.timer.reset()
       self.next_state('__driveBackwards1__')
        
    @state(must_finish= True)
    def __driveBackwards1__(self):
        self.swerve.goDistance(distance_to_rotation(DISTANCE1),0,0)
        self.swerve.execute()
        
        if self.swerve.atDistance(): #or self.timer.hasElapsed(2.0):
            
            print("I made it here! =======================================")

            self.next_state('__stop__') 
    
    @state()
    def __stop__(self):
        pass
        print(f"[{time()}] ================================= Finished. =======================================")

        
        
            
    
        
    # # @state()
    # # def __shoot__(self):
    # #     self.LauncherController.runLauncher()
    # #     print("IMHERE++++++++=======================")
    # #     if not self.LauncherController.currentlyShooting():
    # #         self.swerve.resetEncoders()
    # #         self.next_state('__driveBackwards2__')
    
   
    # @state(must_finish= True)
    # def __driveBackwards2__(self):
    #     self.swerve.resetEncoders()
    #     self.swerve.goDistance(distance_to_rotation(DISTANCE2), 0, 0)
    #     self.swerve.execute()
    #     self.timer.restart()
    #     self.next_state('__ismoving2__')
    #     # print(f"[{time.time()}] ================================= I am moving =======================================")

    #     return False
               
    # @state(must_finish=True)   
    # def __ismoving2__(self):
    #     self.swerve.execute()
    #     if self.timer.hasElapsed(5.0):
    #         self.next_state('__stop__')
