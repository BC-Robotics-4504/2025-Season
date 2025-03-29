from .intake import Intake
from wpilib import Timer
from enum import Enum
from magicbot import StateMachine, state
from collections import namedtuple

IntakeConfig = namedtuple(
    "config",
    [
        "CAN_ids",
        "tolerance",
        "intake_ground_position",
        "shooting_flywheel_threshold_speed",
        "intake_reverse_rolling_speed",
        "intake_forward_rolling_speed",
        "intake_raised_position",
        "coral_score_speed",
        "z_offset",
    ],
)


class IntakeController:

    config: IntakeConfig

    intake: Intake

    current_intake_position = None
    target_intake_position = None

    current_spinner_speed = 0

    def __init__(self, config=IntakeConfig):
        self.config = config

        self.intake = Intake(self.config.CAN_ids)

        pass

    def isPositionedIntake(self):
        """Intake.isPositionedIntake() -> bool

        Check if the intake is in the correct postition."""
        if self.target_intake_position is None:
            return False

        err = self.current_intake_position - self.target_intake_position
        if abs(err) < self.config.tolerance:
            return True
        return False

    def coralRaised(self):
        """Intake.isNoteInIntake() -> bool

        Check if a note is in the intake."""
        raised = not self.intake.getLimitSwitch()
        return raised

    def defaultGround(self):
        """Intake.lowerIntake() -> None

        Set the intake to default ground position."""

        self.intake.setPosition(self.config.intake_ground_position)
        self.current_intake_position = self.config.intake_ground_position
        
        return None

    def raiseCoral(self):
        """Intake.raiseIntake() -> None

        Raise the intake."""
        self.intake.setPosition(self.config.intake_raised_position)
        
        self.current_intake_position = self.config.intake_raised_position
    
        return None

    def getSpinnerSpeed(self):
        return self.intake.spinMotor.get()
    
    def isCoralAtSpeed(self):
        """Intake.isIntakeAtSpeed() -> bool

        Check if the Intake is at speed."""
        err = self.getSpinnerSpeed() > self.config.shooting_flywheel_threshold_speed

        if err:
            return True
        return False

    def spindownIntake(self):
        """Intake.spindownIntake() -> None

        Spin the intake down."""
        self.intake.setSpeed(0.0)
        return None

    def grabAlgae(self):
        """Intake.spinIntakeIn() -> None

        Spin the intake in to grab Algae."""
        if self.current_intake_position != self.config.intake_ground_position:
            self.intake.setPosition(self.config.intake_ground_position)
            
        self.intake.setSpeed(self.config.intake_reverse_rolling_speed)
        return None

    def scoreAlgae(self):
        """Intake.spinIntakeIn() -> None

        Spin the intake in to grab Algae."""
        self.intake.setSpeed(self.config.intake_forward_rolling_speed)
        return None

    def scoreCoral(self):
        if self.current_intake_position != self.config.intake_raised_position:
            self.intake.setPosition(self.config.intake_raised_position)
        self.intake.setSpeed(self.config.coral_score_speed)

    def execute(self):
        """Intake.execute() -> None

        Updates position and speed of the Intake."""
        self.current_intake_position = self.intake.getPosition()
        self.current_spinner_speed = self.intake.getSpeed()
        pass


# class IntakeActions(Enum):
#     GROUND_INTAKE = 1
#     RAISE_INTAKE = 2
#     SCORE_CORAL = 3
#     SCORE_ALGAE = 4
#     PICKUP_ALGAE = 5
#     WAIT = 6



# class IntakeController(StateMachine):
#     MODE_NAME = "Intake Controller"
#     DEFAULT = False
    
#     target_action = IntakeActions.WAIT

#     Kp = -0.1
    
#     def __init__(self, config=IntakeConfig):
#         super().__init__()
#         self.config = config

#     IntakeConfig : IntakeConfig
#     Intake : Intake
    
#     isEngaged = False
#     isShooting = False
    
#     timer = Timer()
#     timer.start()

#     def groundIntake(self):
#         self.target_action = IntakeActions.GROUND_INTAKE

#     def raiseIntake(self):
#         self.target_action = IntakeActions.RAISE_INTAKE
        
#     def scoreCoral(self):
#         self.target_action = IntakeActions.SCORE_CORAL
#         self.isShooting = True

#     def shootSpeaker(self):
#         self.target_action = IntakeActions.
#         self.isShooting = True

#     def runIntake(self):
#         self.engage()

#     def currentlyShooting(self):
#         return self.isShooting

#     '''
#     STATE MACHINE DEFINITIONS ===================================
#     '''

#     @state(first=True)
#     def __wait__(self):       
       
#         if self.target_action == IntakeActions.RAISE_INTAKE:
#             self.next_state('__raiseIntake__')

#         if self.target_action == IntakeActions.SCORE_CORAL:
#             self.next_state('__scoreCoral__')
            
#         if self.target_action == IntakeActions.SCORE_ALGAE:
#             self.timer.restart()
#             self.next_state('__shootAlgae__')
            
#         if self.target_action == IntakeActions.PICKUP_ALGAE:
#             self.next_state('__pickupAlgae__')
            
#         self.target_action = IntakeActions.WAIT
           
#     @state()
#     def __lowerIntake__(self):
#         self.Intake.lowerIntake()
#         self.Intake.spinIntakeIn()
#         self.next_state('__wait__')
            
#     @state()
#     def __spinupIntake__(self):
#         self.Intake.spinupShooter()
#         if self.Intake.isIntakeAtSpeed() or self.timer.hasElapsed(self.RobotConfig.shooting_abort_delay):
#             self.timer.restart()
#             self.next_state_now('__launchNoteSpeaker__')
            
    
#     @state()
#     def __launchNoteSpeaker__(self):
#         self.Intake.feedShooterSpeaker()
#         if self.timer.hasElapsed(self.RobotConfig.intake_feed_delay):
#             self.timer.stop()
#             self.isShooting = False
#             self.next_state('__spindownIntake__')
            
#     @state()
#     def __spindownIntake__(self):
#         self.Intake.spindownIntake()
#         self.Intake.spindownIntake()
#         self.next_state('__wait__')
            
#     @state()
#     def __raiseIntake__(self):
#         self.Intake.raiseIntake()
#         self.Intake.spindownIntake()
#         self.next_state('__wait__')
    
#     @state()
#     def __ampIntake__(self):
#         self.Intake.ampIntake()
#         if self.Intake.isPositionedIntake():
#             self.timer.restart()
#             self.next_state('__launchNoteAmp__')
        
#     @state()
#     def __launchNoteAmp__(self):
#         self.Intake.feedShooterAmp()
#         if self.timer.hasElapsed(self.RobotConfig.intake_feed_delay):
#             self.timer.stop()
#             self.isShooting = False
#             self.next_state('__raiseIntake__')            
        
