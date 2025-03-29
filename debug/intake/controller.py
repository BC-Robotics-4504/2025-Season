from .intake import Intake
from wpilib import Timer
from enum import Enum
from magicbot import StateMachine, state
from collections import namedtuple

IntakeConfig = namedtuple(
    "config",
    [
        "CAN_ids",
        "pivot_tolerance",
        "intake_ground_position",
        "intake_raised_position",
        "coral_score_speed",
        "z_offset",
        "coral_eject_time",
        "algae_pickup_time",
        "algae_pickup_speed",
        "algae_score_time",
        "algae_score_speed",
    ],
)


class IntakeController:

    config: IntakeConfig

    intake: Intake

    current_intake_position = None
    target_intake_position = None

    current_spinner_speed = 0

    timer = Timer()
    timer.start()

    def __init__(self, config=IntakeConfig):
        self.config = config

        self.intake = Intake(self.config.CAN_ids)

        pass

    def raiseIntake(self):

        self.intake.setPosition(self.config.intake_raised_position)
        self.current_intake_position = self.config.intake_raised_position

    def groundIntake(self):

        self.intake.setPosition(self.config.intake_ground_position)
        self.current_intake_position = self.config.intake_ground_position

    def scoreCoral(self):
        if (
            self.intake.getPosition() - self.config.intake_raised_position
            >= self.config.pivot_tolerance
        ):
            self.raiseIntake()

        self.timer.reset()
        self.intake.setSpeed(self.config.coral_score_speed)
        if self.timer.hasElapsed(self.config.coral_eject_time):
            self.intake.setSpeed(0)

    def scoreAlgae(self):
        if (
            self.intake.getPosition() - self.config.intake_ground_position
            >= self.config.pivot_tolerance
        ):
            self.groundIntake()
        self.timer.reset()
        self.intake.setSpeed(self.config.algae_score_speed)
        if self.timer.hasElapsed(self.config.algae_score_time):
            self.intake.setSpeed(0)

    def pickupAlgae(self):

        if (
            self.intake.getPosition() - self.config.intake_ground_position
            >= self.config.pivot_tolerance
        ):
            self.groundIntake()

        self.timer.reset()
        self.intake.setSpeed(self.config.algae_pickup_speed)
        if self.timer.hasElapsed(self.config.algae_pickup_time):
            self.intake.setSpeed(0)
