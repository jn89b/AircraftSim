from typing import List
from dataclasses import dataclass
# import numpy as np
# from aircraftsim.utils.conversions import mps_to_ktas, meters_to_feet
from ..utils.conversions import mps_to_ktas, meters_to_feet
import numpy as np

# from src.conversions import mps_to_ktas, meters_to_feet


@dataclass
class AircraftStateLimits():
    """
    This defines the limits of the aircraft state variables
    meters or radians
    """

    def __init__(self,
                 x_bounds: List[float],
                 y_bounds: List[float],
                 z_bounds: List[float],
                 roll_bounds: List[float],
                 pitch_bounds: List[float],
                 yaw_bounds: List[float],
                 airspeed_bounds: List[float]) -> None:

        self.x_bounds: List[float] = x_bounds
        self.y_bounds: List[float] = y_bounds
        self.z_bounds: List[float] = z_bounds
        self.roll_bounds: List[float] = roll_bounds
        self.pitch_bounds: List[float] = pitch_bounds
        self.yaw_bounds: List[float] = yaw_bounds
        self.airspeed_bounds: List[float] = airspeed_bounds


@dataclass
class HighLevelControlLimits():
    """
    High level control limits for
    rad/s or m/s 
    Ctrl type 0: attitude control
    Ctrl type 1: dz_heading_airspeed control

    """

    def __init__(self,
                 roll_rate: List[float],
                 pitch_rate: List[float],
                 yaw_rate: List[float],
                 vel_cmd: List[float]) -> None:
        self.roll_rate: List[float] = roll_rate
        self.pitch_rate: List[float] = pitch_rate
        self.yaw_rate: List[float] = yaw_rate
        self.vel_cmd: List[float] = vel_cmd


@dataclass
class LowLevelControlLimits():
    def __init__(self) -> None:
        pass


@dataclass
class HighControlInputs():
    """
    High level control inputs
    User must specify the control type and the corresponding control inputs
    0: attitude control
    1: dz_heading_airspeed control

    0: roll_rate, pitch_rate, yaw_rate, vel_cmd
    1: alt_ref_ft, heading_ref_deg, airspeed_ref_kts, vel_cmd

    If incorrect control type is specified, an error will be raised

    """

    def __init__(self,
                 ctrl_idx: int = 1,
                 alt_ref_m: float = None,
                 heading_ref_deg: float = None,
                 roll: float = None,
                 pitch: float = None,
                 yaw: float = None,
                 vel_cmd: float = None) -> None:
        self.ctrl_list: List[str] = ["attitude",
                                     "dz_heading_airspeed", "target_tracking"]
        self.ctrl_idx: int = ctrl_idx
        self.ctrl_type: str = self.ctrl_list[self.ctrl_idx]
        self.alt_ref_m: float = alt_ref_m
        if self.alt_ref_m is not None:
            self.alt_ref_ft: float = meters_to_feet(alt_ref_m)
        self.heading_ref_deg: float = heading_ref_deg
        self.vel_cmd: float = vel_cmd
        # self.airspeed_ref_m:float = airspeed_ref_m
        self.airspeed_ref_kts: float = mps_to_ktas(vel_cmd)
        self.roll: float = roll
        self.pitch: float = pitch
        self.yaw: float = yaw

        self.check_correct()

    def check_correct(self) -> None:
        """
        """
        if self.ctrl_idx == 0:
            if self.roll is None:
                raise ValueError("Roll rate is Non")
            if self.pitch is None:
                raise ValueError("Pitch rate is None")
            if self.yaw is None:
                raise ValueError("Yaw rate is None")
            return

        if self.ctrl_idx == 1:
            if self.alt_ref_ft is None:
                raise ValueError("Altitude reference is None")
            if self.heading_ref_deg is None:
                raise ValueError("Heading reference is None")
            if self.airspeed_ref_kts is None:
                raise ValueError("Airspeed reference is None")
            if self.vel_cmd is None:
                raise ValueError("Velocity command is None")
            return

# @dataclass
# class LowLevelControlInputs():
#     def __init__(self) -> None:
#         pass


@dataclass
class AircraftState():
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    airspeed: float
    # def __init__() -> None:
    # self.x:float = x
    # self.y:float = y
    # self.z:float = z
    # self.roll:float = roll
    # self.pitch:float = pitch
    # self.yaw:float = yaw
    # self.airspeed:float = airspeed


@dataclass
class AircraftIC():
    """
    Might need to add more initial conditions
    """
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float
    airspeed_m: float

    def return_as_array(self) -> np.ndarray:
        states = np.array([
            self.x,
            self.y,
            self.z,
            self.roll,
            self.pitch,
            self.yaw], dtype=float)

        return states
