import rev
import wpilib

from config import RobotConfig

from .spinner import SparkMaxDualSpinner
from .pivot import SparkMaxPivot

import math


class Intake:

    RobotConfig: RobotConfig

    IntakePivot: SparkMaxPivot

    OutputSpinner: SparkMaxDualSpinner

    current_intake_position = None
    target_intake_position = None

    current_spinner_speed = 0

    def __init__(self):
        pass

    def isPositionedIntake(self):
        """Launcher.isPositionedIntake() -> bool

        Check if the intake is in the correct postition."""
        if self.target_intake_position is None:
            return False

        err = self.current_intake_position - self.target_intake_position
        if abs(err) < self.RobotConfig.intake_tolerance:
            return True
        return False

    def CoralRaised(self):
        """Launcher.isNoteInIntake() -> bool

        Check if a note is in the intake."""
        raised = not self.IntakePivot.getLimitSwitch()
        return raised

    def defaultGround(self):
        """Launcher.lowerIntake() -> None

        Set the intake to default ground position."""
        self.target_intake_position = self.RobotConfig.intake_ground_position
        self.IntakePivot.setPosition(self.RobotConfig.intake_ground_position)
        return None

    def raiseCoral(self):
        """Launcher.raiseIntake() -> None

        Raise the intake."""
        self.target_intake_position = self.RobotConfig.intake_raised_position
        self.IntakePivot.setPosition(self.RobotConfig.intake_raised_position)
        return None

    def isCoralAtSpeed(self):
        """Launcher.isLauncherAtSpeed() -> bool

        Check if the launcher is at speed."""
        err = (
            self.current_spinner_speed
            > self.RobotConfig.shooting_flywheel_threshold_speed
        )

        if err:
            return True
        return False

    def spindownIntake(self):
        """Launcher.spindownIntake() -> None

        Spin the intake down."""
        self.OutputSpinner.setSpeed(0.0)
        return None

    def grabAlgae(self):
        """Launcher.spinIntakeIn() -> None

        Spin the intake in to grab Algae."""
        self.OutputSpinner.setSpeed(self.RobotConfig.intake_reverse_rolling_speed)
        return None

    def scoreAlgae(self):
        """Launcher.spinIntakeIn() -> None

        Spin the intake in to grab Algae."""
        self.OutputSpinner.setSpeed(self.RobotConfig.intake_forward_rolling_speed)
        return None
    
    def scoreCoral(self):
        self.OutputSpinner.setSpeed(self.RobotConfig.coral_score_speed)

    def execute(self):
        """Launcher.execute() -> None

        Updates position and speed of the Launcher."""
        self.current_intake_position = self.IntakePivot.getPosition()
        self.current_spinner_speed = self.LauncherSpinnerL.getSpeed()
        self.currentR_launcher_speed = self.LauncherSpinnerR.getSpeed()
        pass
