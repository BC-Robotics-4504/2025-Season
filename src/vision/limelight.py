''' LimeLight
Repo: https://github.com/FRC703/Robotpy-Limelight/tree/master (https://pypi.org/project/robotpy-limelight)
Author: Josh Bacon, FRC703
'''
import ntcore
from enum import Enum
from typing import Tuple


class LEDState(Enum):
    """
    |LEDState|Sets limelight's LED state|
    |-|-|
    |0|use the LED Mode set in the current pipeline|
    |1|force off|
    |2|force blink|
    |3|force on|
    """

    MATCH_PIPELINE = 0
    OFF = 1
    BLINK = 2
    ON = 3


class CamMode(Enum):
    """
    |CamMode|Sets limelight's operation mode|
    |-|-|
    |PROCESSED|Vision processor|
    |DRIVER|Driver camera (increases exposure, disables vision processing)|
    """

    PROCESSED = 0
    DRIVER = 1


class StreamMode(Enum):
    """
    |StreamMode|Sets limelight's streaming mode|
    |-|-|
    |0|Standard - Side-by-side streams if a webcam is attached to Limelight|
    |1|PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream|
    |2|PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream|
    """

    STANDARD = 0
    PIP_MAIN = 1
    PIP_SECONDARY = 2


class SnapshotMode(Enum):
    """
    |SnapshotMode|Allows users to take snapshots during a match|
    |-|-|
    |0|Stop taking snapshots|
    |1|Take two snapshots per second|
    """

    NONE = 0
    TAKE_2_PS = 1


class Limelight:
    _enabled = 1
    _light = LEDState.OFF
    _stream_mode = 0
    _snapshots = 0
    __nt = None
    _active_pipeline = 0

    def __init__(self, name='limelight', nt=None, camera=False, light=False):
        """
        Creates an instance of a limelight

        Args:
            nt: Pass in a custom networktables table if your limelight is running
                on a table other than `limelight`

            camera: Processed pipeline or driver control mode.

            light: Default state of the light to set when camera is connected to the code
        """
        if nt is not None:
            self.__nt = nt
        else:
            inst = ntcore.NetworkTableInstance.getDefault()
            self.__nt = inst.getTable(name)
            
        self._enabled = camera
        self._light = light

    @property
    def valid_targets(self) -> bool:
        """
        Whether the camera has found a valid target

        Returns:
            Any valid targets?
        """
        return bool(self.__nt.getNumber("tv", 0))

    @property
    def horizontal_offset(self) -> float:
        """
        Gives the horizontal offset from the crosshair to the target
        LL1: -27° - 27°
        LL2: -29.8° - 29.8°

        Returns:
            The horizontal offest from the crosshair to the target.

        """
        return self.__nt.getNumber("tx", 0)

    @property
    def vertical_offset(self) -> float:
        """
        Gives the vertical offset from the crosshair to the target
        LL1: -20.5° - 20.5°
        LL2: -24.85° - 24.85°

        Returns:
            The vertical offset from the crosshair to the target
        """
        return self.__nt.getNumber("ty", 0)

    @property
    def target_area(self) -> float:
        """
        How much of the image is being filled by the target

        Returns:
            0% - 100% of image
        """
        return self.__nt.getNumber("ta", 0)

    @property
    def skew(self) -> float:
        """
        How much the target is skewed

        Returns:
            -90° - 0°
        """
        return self.__nt.getNumber("ts", 0)

    @property
    def latency(self) -> float:
        """
        How much the pipeline contributes to the latency. Adds at least 11ms for image capture

        Returns:
            Latency contribution
        """
        return self.__nt.getNumber("tl", 0)

    @property
    def bb_short(self) -> float:
        """
        Sidelength of the shortest side of the fitted bouding box (pixels)

        Returns:
            Shortest sidelength
        """
        return self.__nt.getNumber("tshort", 0)

    @property
    def bb_long(self) -> float:
        """
        Sidelength of the longest side of the fitted bouding box (pixels)

        Returns:
            Longest sidelength
        """
        return self.__nt.getNumber("tlong", 0)

    @property
    def bb_horizontal(self) -> float:
        """
        Horizontal sidelength of the rough bounding box (0 - 320 px)

        Returns:
            The horizontal sidelength
        """
        return self.__nt.getNumber("thor", 0)

    @property
    def bb_vertical(self) -> float:
        """
        Vertical sidelength of the rough bounding box (0 - 320 px)

        Returns:
            The vertical sidelength
        """
        return self.__nt.getNumber("tvert", 0)

    @property
    def bounding_box(self) -> Tuple[float, float]:
        return (self.bb_horizontal, self.bb_vertical)

    def camtran(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        """
        Results of a 3D solution position, 6 numbers: Translation(x,y,z) Rotation(pitch, yaw, roll)
        Honestly I have no clue what this does yet without some testing.
        """
        return self.__nt.getNumber("camtran", 0)

    @property
    def crosshair_ax(self) -> float:
        """
        Get crosshair A's X position
        """
        return self.__nt.getNumber("cx0", 0)

    @property
    def crosshair_ay(self) -> float:
        """
        Get crosshair A's Y position
        """
        return self.__nt.getNumber("cy0", 0)

    @property
    def crosshair_bx(self) -> float:
        """
        Get crosshair B's X position
        """
        return self.__nt.getNumber("cx1", 0)

    @property
    def crosshair_by(self) -> float:
        """
        Get crosshair B's Y position
        """
        return self.__nt.getNumber("cy1", 0)

    def camera(self, camMode: CamMode) -> None:
        """
        Set the camera mode. You can set this to be driver operated or to run on a pipeline.

        Args:
            camMode: The camera mode to set
        """
        self._enabled = camMode
        self.__nt.putNumber(
            "camMode", camMode.value if isinstance(camMode, CamMode) else camMode
        )

    def light(self, status: LEDState) -> None:
        """
        Set the status of the limelight lights

        Args:
            status: The status to set the light to
        """
        self._light = status
        self.__nt.putNumber(
            "ledMode", status.value if isinstance(status, LEDState) else status
        )

    def pipeline(self, pipeline: int):
        """
        Sets the currently active pipeline

        Args:
            pipeline: The pipeline id to set to be active
        """
        self._active_pipeline = 0
        self.__nt.putNumber("pipeline", pipeline)

    def snapshot(self, snapshotMode: SnapshotMode):
        
        """
        Allow users to take snapshots during a match

        Args:
            snapshotMode: The state to put the camera in
        """
        self._snapshots = snapshotMode
        self.__nt.putNumber(
            "snapshot",
            snapshotMode.value
            if isinstance(snapshotMode, SnapshotMode)
            else snapshotMode,
        )
    def execute(self):
        pass