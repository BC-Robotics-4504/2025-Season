import rev
import wpilib

from config import RobotConfig

from spinner import SparkMaxDualSpinner
from pivot import SparkMaxPivot

import math


class Launcher:
    RobotConfig: RobotConfig

    IntakePivot: SparkMaxPivot

    IntakeSpinnerL: SparkMaxDualSpinner
    IntakeSpinnerR: SparkMaxDualSpinner

    LauncherSpinnerL: SparkMaxDualSpinner
    LauncherSpinnerR: SparkMaxDualSpinner

    LimitSwitch: wpilib.DigitalInput

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

    def intakeRaised(self):
        """Launcher.isNoteInIntake() -> bool

        Check if a note is in the intake."""
        raised = not self.LimitSwitch.get()
        return raised

    def lowerIntake(self):
        """Launcher.lowerIntake() -> None

        Lower the intake."""
        self.target_intake_position = self.RobotConfig.intake_lowered_position
        self.IntakePivot.setPosition(self.RobotConfig.intake_lowered_position)
        return None

    def raiseIntake(self):
        """Launcher.raiseIntake() -> None

        Raise the intake."""
        self.target_intake_position = self.RobotConfig.intake_raised_position
        self.IntakePivot.setPosition(self.RobotConfig.intake_raised_position)
        return None

    def isLauncherAtSpeed(self):
        """Launcher.isLauncherAtSpeed() -> bool

        Check if the launcher is at speed."""
        err= (
            self.current_spinner_speed
            > self.RobotConfig.shooting_flywheel_threshold_speed
        )
        
        if err:
            return True
        return False

    def spinupShooter(self):
        """Launcher.spinupShooter() -> None

        Spin the launcher up."""
        self.LauncherSpinnerL.setSpeed(self.RobotConfig.shooting_flywheel_speed)
        self.LauncherSpinnerR.setSpeed(self.RobotConfig.shooting_flywheel_speed)
        return None

    def spindownLauncher(self):
        """Launcher.spindownLauncher() -> None

        Spin the launcher down."""
        self.LauncherSpinnerL.setSpeed(0.0)
        self.LauncherSpinnerR.setSpeed(0.0)
        return None

    def feedShooterSpeaker(self):
        """Launcher.feedShooterSpeaker() -> None

        Feed the shooter when shooter is spinning up to shoot speaker."""
        self.IntakeSpinnerL.setSpeed(self.RobotConfig.intake_feed_speaker_speed)
        self.IntakeSpinnerR.setSpeed(self.RobotConfig.intake_feed_speaker_speed)
        return None

    def spindownIntake(self):
        """Launcher.spindownIntake() -> None

        Spin the intake down."""
        self.IntakeSpinnerL.setSpeed(0.0)
        self.IntakeSpinnerR.setSpeed(0.0)
        return None

    def spinIntakeIn(self):
        """Launcher.spinIntakeIn() -> None

        Spin the intake in."""
        self.IntakeSpinnerL.setSpeed(self.RobotConfig.intake_reverse_rolling_speed)
        self.IntakeSpinnerR.setSpeed(self.RobotConfig.intake_reverse_rolling_speed)
        return None

    def ampIntake(self):
        """Launcher.ampIntake() -> None

        Raise the intake to the amp position."""
        self.target_intake_position = self.RobotConfig.intake_amp_position
        self.IntakePivot.setPosition(self.RobotConfig.intake_amp_position)
        return None

    def feedShooterAmp(self):
        """Launcher.feedShooterAmp() -> None

        Feed the shooter when shooter is spinning up to shoot amp."""
        self.IntakeSpinnerL.setSpeed(self.RobotConfig.intake_amp_shooting_speed + 0.02)
        self.IntakeSpinnerR.setSpeed(self.RobotConfig.intake_amp_shooting_speed)
        return None

    def execute(self):
        """Launcher.execute() -> None

        Updates position and speed of the Launcher."""
        self.current_intake_position = self.IntakePivot.getPosition()
        self.current_spinner_speed = self.LauncherSpinnerL.getSpeed()
        self.currentR_launcher_speed = self.LauncherSpinnerR.getSpeed()
        pass
