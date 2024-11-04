"""
User defines

Handle the JSBSIM first

"""
from typing import Optional
import numpy as np

from aircraftsim.utils.data_containers import AircraftStateLimits, \
    HighLevelControlLimits, LowLevelControlLimits, AircraftState, \
    AircraftIC, HighControlInputs

from aircraftsim.jsbsim_aircraft.aircraft import x8
from aircraftsim.jsbsim_aircraft.simulator import FlightDynamics
from aircraftsim.guidance_control.autopilot import X8Autopilot
from aircraftsim.guidance_control.navigation import WindEstimation
from aircraftsim.utils.conversions import mps_to_ktas, meters_to_feet, local_to_global_position
from aircraftsim.utils.report_diagrams import ReportGraphs, SimResults


class AircraftSim():
    fidelity_list = ['JSBSIM', 'KINEMATICS']
    integration_list = ['EULER', 'RK4']

    def __init__(self, aircraft_name: str = None,
                 fidelity_type: Optional[int] = 0,
                 integration_type: Optional[int] = 1,
                 sim_freq: Optional[int] = 200,
                 is_high_lvl_ctrl: Optional[bool] = True,
                 init_cond: AircraftIC = None,
                 low_control_lim: LowLevelControlLimits = None) -> None:

        self.aircraft = None
        self.sim = None

        self.aircraft_name: str = aircraft_name
        self.autopilot = None

        if self.aircraft_name == "x8":
            self.aircraft = x8

        self.fidelity_type: str = self.fidelity_list[fidelity_type]
        self.integration_type: str = self.integration_list[integration_type]
        self.sim_freq: int = sim_freq
        self.dt: float = 1/self.sim_freq
        self.is_high_lvl_ctrl: bool = is_high_lvl_ctrl
        self.init_cond: AircraftIC = self.convert_ic_to_dict(init_cond)

        # self.low_control_lim:LowLevelControlLimits = low_control_lim
        self.__check_filled()
        self.__set_fidelity()
        # self.__set_integration()
        self.report: SimResults = self.__set_report()
        self.wind_estimation: WindEstimation = WindEstimation(self.sim)

    def convert_ic_to_dict(self, init_cond: AircraftIC) -> dict:

        if init_cond is None:
            raise ValueError("init cond is None")

        lon_lat_alt = local_to_global_position(
            [init_cond.x, init_cond.y, init_cond.z]
        )
        lon = lon_lat_alt[0]
        lat = lon_lat_alt[1]
        # alt = lon_lat_alt[2]

        init_cond_dict = {
            "ic/u-fps": mps_to_ktas(init_cond.airspeed_m),
            "ic/v-fps": 0.0,
            "ic/w-fps": 0.0,
            "ic/p-rad_sec": 0.0,
            "ic/q-rad_sec": 0.0,
            "ic/r-rad_sec": 0.0,
            "ic/h-sl-ft": meters_to_feet(init_cond.z),
            "ic/long-gc-deg": lon,
            "ic/lat-gc-deg": lat,
            "ic/psi-true-deg": np.rad2deg(init_cond.yaw),
            "ic/theta-deg": np.rad2deg(init_cond.pitch),
            "ic/phi-deg": np.rad2deg(init_cond.roll),
            "ic/alpha-deg": 0.0,
            "ic/beta-deg": 0.0,
            "ic/num_engines": 1,
        }

        return init_cond_dict

    def __check_filled(self) -> None:
        # if self.state_lim is None:
        #     raise ValueError("State limits not defined")
        # if self.high_control_lim is None:
        #     raise ValueError("High level control limits not defined")
        # if self.low_control_lim is None:
        #     raise ValueError("Low level control limits not defined")
        if self.aircraft_name is None:
            raise ValueError("Aircraft name not defined")
        if self.fidelity_type is None:
            raise ValueError("Fidelity type not defined")
        if self.integration_type is None:
            raise ValueError("Integration type not defined")
        if self.sim_freq is None:
            raise ValueError("Simulation frequency not defined")
        if self.init_cond is None:
            raise ValueError("Initial Conditions are not defined")

        return

    def __set_fidelity(self) -> None:
        """
        Depending on what the user sets will
        set fidelity of simulator between using JSBSIM or kinematics
        """
        if self.fidelity_type == "JSBSIM":
            self.sim = FlightDynamics(
                sim_frequency_hz=self.sim_freq,
                aircraft=self.aircraft,
                init_conditions=self.init_cond,
                return_metric_units=True,
                debug_level=0)
            self.sim.init_fdm()
            self.sim.start_engines()
            self.sim.set_throttle_mixture_controls(0.3, 0)
            self.autopilot = X8Autopilot(
                sim=self.sim)
        elif self.fidelity_type == "KINEMATICS":
            print("Using Kinematics")
        else:
            raise ValueError("Fidelity type not defined \
                you must specify JSBSIM or KINEMATICS")

    def __set_integration(self) -> None:
        # TODO: this is used for kinematics
        if self.integration_type == "EULER":
            print("Using Euler Integration")
        elif self.integration_type == "RK4":
            pass
        else:
            raise ValueError("Integration type not defined \
                you must specify EULER or RK4")

    def __set_report(self) -> SimResults:
        if self.fidelity_type == "JSBSIM":
            return SimResults(self.sim)

    def __set_autopilot_ref(self, high_lvl_ctrl: HighControlInputs) -> None:
        """
        TODO: refactor the ctrl type checker to be more efficient
        """
        if high_lvl_ctrl.ctrl_type == "attitude":
            self.autopilot.airspeed_hold_w_throttle(
                mps_to_ktas(high_lvl_ctrl.vel_cmd))
            self.autopilot.roll_hold(high_lvl_ctrl.roll)
            # self.autopilot.pitch_hold(high_lvl_ctrl.pitch)
            self.autopilot.altitude_hold(high_lvl_ctrl.alt_ref_ft)
            # self.autopilot.heading_hold(high_lvl_ctrl.yaw)

        elif high_lvl_ctrl.ctrl_type == "dz_heading_airspeed":
            # print("alt ref: ", high_lvl_ctrl.alt_ref_ft)
            # print("alt m: ", high_lvl_ctrl.alt_ref_m)
            self.autopilot.altitude_hold(high_lvl_ctrl.alt_ref_ft)
            self.autopilot.heading_hold(high_lvl_ctrl.heading_ref_deg)
            self.autopilot.airspeed_hold_w_throttle(
                high_lvl_ctrl.airspeed_ref_kts)
            # self.autopilot.airspeed_hold_w_throttle(
            #     meters_to_feet(high_lvl_ctrl.vel_cmd))
        elif high_lvl_ctrl.ctrl_type == "target_tracking":
            true_x = 100.0
            true_y = 0.0
            true_z = 50.0
            dx = true_x - self.sim.get_local_position()[0]
            dy = true_y - self.sim.get_local_position()[1]
            dz = true_z - self.sim.get_local_position()[2]
            self.autopilot.track_to_target(
                target_northing=dx,
                target_easting=dy,
                target_alt=dz
            )

        return

    def step(self, high_lvl_ctrl: HighControlInputs) -> None:
        """
        #TODO: Add low level control

        For JSBSIM: the high level controls are:
            altitude, airspeed, heading
        """

        if high_lvl_ctrl.check_correct() == False:
            raise ValueError("High level control inputs are incorrect\
                please check the control type which was set to: {}".format(
                high_lvl_ctrl.ctrl_type))

        self.__set_autopilot_ref(high_lvl_ctrl)
        if self.fidelity_type == "JSBSIM":
            self.sim.run()
            aircraft_state: AircraftState = self.sim.get_states()
            self.report.collect_data(aircraft_state)

    def close_sim(self) -> None:
        if self.fidelity_type == "JSBSIM":
            self.sim.close()

    def get_states(self) -> AircraftState:
        if self.fidelity_type == "JSBSIM":
            return self.sim.get_states()

    def reset_sim(self, init_cond: AircraftIC) -> None:
        if self.fidelity_type == "JSBSIM":
            self.init_cond = self.convert_ic_to_dict(init_cond)
            self.sim.reinitialise(self.init_cond)

            # self.sim = FlightDynamics(
            #     sim_frequency_hz=self.sim_freq,
            #     aircraft=self.aircraft,
            #     init_conditions=self.init_cond,
            #     return_metric_units=True,
            #     debug_level=0)
            self.sim.start_engines()
            self.sim.set_throttle_mixture_controls(0.3, 0)
            self.autopilot = X8Autopilot(
                sim=self.sim)
