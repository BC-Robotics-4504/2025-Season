
from .intake import Intake
from collections import namedtuple

IntakeConfig = namedtuple(
    'intakeConfig',
    [   'CAN_ids',
        'tolerance',
        'intake_ground_position',
        'shooting_flywheel_threshold_speed',
        'intake_reverse_rolling_speed',
        'intake_forward_rolling_speed',
        'intake_raised_position',
        'coral_score_speed',
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
        """Launcher.isPositionedIntake() -> bool

        Check if the intake is in the correct postition."""
        if self.target_intake_position is None:
            return False

        err = self.current_intake_position - self.target_intake_position
        if abs(err) < self.config.tolerance:
            return True
        return False

    def CoralRaised(self):
        """Launcher.isNoteInIntake() -> bool

        Check if a note is in the intake."""
        raised = not self.intake.getLimitSwitch()
        return raised

    def defaultGround(self):
        """Launcher.lowerIntake() -> None

        Set the intake to default ground position."""
        self.target_intake_position = self.config.intake_ground_position
        self.intake.setPosition(self.config.intake_ground_position)
        return None

    def raiseCoral(self):
        """Launcher.raiseIntake() -> None

        Raise the intake."""
        self.target_intake_position = self.config.intake_raised_position
        self.intake.setPosition(self.config.intake_raised_position)
        return None

    def isCoralAtSpeed(self):
        """Launcher.isLauncherAtSpeed() -> bool

        Check if the launcher is at speed."""
        err = self.current_spinner_speed > self.config.shooting_flywheel_threshold_speed

        if err:
            return True
        return False

    def spindownIntake(self):
        """Launcher.spindownIntake() -> None

        Spin the intake down."""
        self.intake.setSpeed(0.0)
        return None

    def grabAlgae(self):
        """Launcher.spinIntakeIn() -> None

        Spin the intake in to grab Algae."""
        self.intake.setSpeed(self.config.intake_reverse_rolling_speed)
        return None

    def scoreAlgae(self):
        """Launcher.spinIntakeIn() -> None

        Spin the intake in to grab Algae."""
        self.intake.setSpeed(self.config.intake_forward_rolling_speed)
        return None

    def scoreCoral(self):
        self.intake.setSpeed(self.config.coral_score_speed)

    def execute(self):
        """Launcher.execute() -> None

        Updates position and speed of the Launcher."""
        self.current_intake_position = self.intake.getPosition()
        self.current_spinner_speed = self.intake.getSpeed()
        pass
