from magicbot import StateMachine, state
from enum import Enum
from wpilib import Timer

from .intake import Intake

from config import RobotConfig


class IntakeActions(Enum):
    GROUND = 1
    RAISE_CORAL = 2
    SCORE_CORAL = 3
    SCORE_ALGAE = 4
    INTAKE_ALGAE = 5
    SPINDOWN = 6
    WAIT = 7
    


class IntakeController(StateMachine):
    MODE_NAME = "Intake Controller"
    DEFAULT = False

    target_action = IntakeActions.WAIT

    Kp = -0.1

    RobotConfig: RobotConfig
    Intake: Intake

    isEngaged = False
    isShooting = False

    timer = Timer()
    timer.start()

    def ground(self):
        self.target_action = IntakeActions.GROUND
    
    def scoreCoral(self):
        self.target_action = IntakeActions.SCORE_CORAL
        self.isShooting = True
    
    def scoreAlgae(self):
        self.target_action = IntakeActions.SCORE_ALGAE
        self.isShooting = True
        
    def spindownIntake(self):
        self.target_action = IntakeActions.SPINDOWN
        
    def intakeAlgae(self):
        self.target_action = IntakeActions.INTAKE_ALGAE

    def raiseCoral(self):
        self.target_action = IntakeActions.RAISE_CORAL


    def runLauncher(self):
        self.engage()

    def currentlyShooting(self):
        return self.isShooting

    """
    STATE MACHINE DEFINITIONS ===================================
    """

    @state(first=True)
    def __wait__(self):

        if self.target_action == IntakeActions.GROUND:
            self.next_state("__setGround__")
        
        if self.target_action == IntakeActions.SPINDOWN:
            self.next_state("__spindownIntake__")
            
        if self.target_action == IntakeActions.INTAKE_ALGAE:
            self.next_state("__intakeAlgae__")
            
        if self.target_action == IntakeActions.SCORE_ALGAE:
            self.next_state("__scoreAlgae__")
            self.isShooting == True
                       
        if self.target_action == IntakeActions.RAISE_CORAL:
            self.timer.restart()
            self.next_state("__raiseCoral__")

        if self.target_action == IntakeActions.SCORE_CORAL:
            self.next_state("__scoreCoral__")
            self.isShooting == True 
        
        self.target_action = IntakeActions.WAIT

    @state()
    def __setGround__(self):
        self.Intake.defaultGround()
        self.next_state("__wait__")

    @state()
    def __intakeAlgae__(self):
        self.Intake.grabAlgae()
        if self.Intake.isCoralAtSpeed() or self.timer.hasElapsed(
            self.RobotConfig.shooting_abort_delay
        ):
            self.timer.restart()
            self.next_state_now("__wait__")

    @state()
    def __scoreAlgae__(self):
        self.Intake.scoreAlgae()
        if self.timer.hasElapsed(self.RobotConfig.intake_feed_delay):
            self.timer.stop()
            self.isShooting = False
            self.next_state("__spindownIntake__")
            
    @state()
    def __spindownIntake__(self):
        self.Intake.spindownIntake()
        self.next_state("__wait__")
    
    @state 
    def scoreCoral(self):
        self.Intake.scoreCoral() 
        if self.timer.hasElapsed(self.coral_abort_time):
            self.timer.stop
            self.isShooting = False
            self.next_state("__spindownIntake__")
            
    @state()
    def __raiseCoral__(self):
        self.Intake.raiseCoral()
        self.Intake.spindownIntake()
        self.next_state("__wait__")

   