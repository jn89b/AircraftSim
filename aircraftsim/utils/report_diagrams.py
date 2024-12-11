import matplotlib
# matplotlib.rcParams['text.usetex'] = True
from typing import List, Tuple
# rc('font',**{'family':'sans-serif','sans-serif':['Helvetica']})
# for Palatino and other serif fonts use:
# rc('font', **{'family': 'serif', 'serif': ['Computer Modern']})
# rc('text', usetex=True)
import matplotlib.pyplot as plt
# import jsbsim_properties as prp
# import jsbsim_backend.properties as prp
# import aircraftsim.jsbsim_aircraft.properties as prp
import aircraftsim.jsbsim_aircraft.properties as prp

import aircraftsim.guidance_control.navigation as navigation
from aircraftsim.jsbsim_aircraft.simulator import FlightDynamics
from aircraftsim.utils.data_containers import AircraftState
from aircraftsim.utils.conversions import feet_to_meters
# import guidance_control.navigation as navigation
import math
import numpy as np


class ReportGraphs:
    def __init__(self, sim):
        self.sim = sim
        self.nav = navigation.LocalNavigation(sim)

        self.time = []

        self.lat_m = []
        self.long_m = []
        self.alt = []

        self.yaw = []
        self.pitch = []
        self.roll = []

        self.aileron_cmd = []
        self.elevator_cmd = []
        self.throttle_cmd = []
        self.rudder_cmd = []

        self.aileron_combined = []
        self.elevator = []
        self.throttle = []

        self.p = []
        self.q = []
        self.r = []

        self.airspeed = []
        self.vs = []

    def get_time_data(self):
        self.time.append(self.sim.get_time())

    def get_pos_data(self):
        self.lat_m.append(self.nav.get_local_pos()[0])
        self.long_m.append(self.nav.get_local_pos()[1])
        self.alt.append(self.sim.get_local_position()[2])

    def get_attitude_data(self):
        self.pitch.append(self.sim.get_local_orientation()
                          [0] * (180 / math.pi))
        self.roll.append(self.sim.get_local_orientation()[1] * (180 / math.pi))
        self.yaw.append(self.sim.get_local_orientation()[2] * (180 / math.pi))

    def get_rate_data(self):
        self.p.append(self.sim[prp.p_radps])
        self.q.append(self.sim[prp.q_radps])
        self.r.append(self.sim[prp.r_radps])

    def get_airspeed(self):
        self.airspeed.append(self.sim[prp.airspeed] * 0.5925)

    def get_control_command(self):
        self.aileron_combined.append(
            self.sim[prp.aileron_combined_rad] * (180.0 / math.pi))
        self.elevator.append(self.sim[prp.elevator] * (180.0 / math.pi))

    def get_graph_info(self):
        self.get_time_data()
        self.get_pos_data()
        self.get_attitude_data()
        self.get_rate_data()
        self.get_airspeed()
        self.get_control_command()

    def trace_plot(self, desired_points):
        fig, ax = plt.subplots()
        ax.set_title(r'\textbf{Plan View Track}')
        ax.set_xlabel(r'x position [$m$]')
        ax.set_ylabel(r'y position [$m$]')
        plt.grid(True)
        points, = ax.plot([i[0] for i in desired_points], [i[1] for i in desired_points], marker='^',
                          color='#FF7F11', linestyle='None')
        line, = ax.plot(self.lat_m, self.long_m,
                        linestyle='--', color='#0077B6')
        line.set_label(r'track made good')
        points.set_label(r'commanded fly-by waypoints')
        ax.legend()
        ax.set_aspect('equal')
        plt.savefig('trace_plot.eps')
        plt.show()

    def control_response(self, start_time, stop_time, update_frequency):
        start = start_time * update_frequency
        stop = stop_time * update_frequency
        orange = '#FF7F11'
        blue = '#0077B6'
        ax1 = plt.subplot(211)
        # ax1.plot(self.time[start:stop], self.q[start:stop], linestyle='--', color=orange)
        ax1.set_title(r'\textbf{Roll Response}')
        ax1.set_xlabel(r'time[s]')
        ax1.set_ylabel(r'q')
        ax3 = ax1.twinx()
        ax3.plot(self.time[start:stop],
                 self.elevator[start:stop], linestyle='-', color=blue)

        ax2 = plt.subplot(212)
        # ax2.plot(self.time[start:stop], self.p[start:stop], linestyle='--', color=orange)
        ax2.set_title(r'\textbf{Pitch Response}')
        ax2.set_xlabel(r'time [s]')
        ax4 = ax2.twinx()
        ax4.plot(self.time[start:stop],
                 self.aileron_combined[start:stop], linestyle='-', color=blue)

        # ax5 = plt.subplot(212)
        # plt.plot(self.time[start:stop], self.yaw[start:stop], linestyle='--', color=orange)
        # ax5.set_title(r'\textbf{Pitch Response}')
        # ax5.set_xlabel(r'time [s]')
        # ax6 = ax2.twinx()
        # ax6.plot(self.time[start:stop], self.aileron_combined[start:stop], linestyle='-', color=blue)

        plt.show()

    def three_d_plot(self, start_time, stop_time, update_frequency):
        start = start_time * update_frequency
        stop = stop_time * update_frequency
        orange = '#FF7F11'
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.set_title(r'\textbf{3D Track}')
        ax.set_xlabel(r'x position [$m$]')
        ax.set_ylabel(r'y position [$m$]')
        ax.set_zlabel(r'Altitude [$m$]')
        xline = self.lat_m[start:stop]
        yline = self.long_m[start:stop]
        zline = [x / 3.28 for x in self.alt[start:stop]]
        ax.plot3D(xline, yline, zline, linestyle='--', color=orange)
        ax.set_box_aspect((np.ptp(xline), np.ptp(yline), 11*np.ptp(zline)))
        plt.savefig('box.eps')
        plt.show()


class SimResults():
    """
    Collects simulation results for plotting
    """

    def __init__(self,
                 sim: FlightDynamics) -> None:
        self.sim: FlightDynamics = sim
        self.time: List[float] = []
        self.lat_m: List[float] = []
        self.long_m: List[float] = []
        self.alt: List[float] = []

        self.yaw_dg: List[float] = []
        self.pitch_dg: List[float] = []
        self.roll_dg: List[float] = []

        self.aileron_cmd: List[float] = []
        self.elevator_cmd: List[float] = []
        self.throttle_cmd: List[float] = []
        self.rudder_cmd: List[float] = []

        self.aileron_combined: List[float] = []
        self.elevator: List[float] = []
        self.throttle: List[float] = []

        self.p: List[float] = []
        self.q: List[float] = []
        self.r: List[float] = []

        self.airspeed: List[float] = []
        self.vs: List[float] = []
        self.airspeed_ref: List[float] = []

        self.x: List[float] = []
        self.y: List[float] = []
        self.z: List[float] = []

        self.vx: List[float] = []
        self.vy: List[float] = []
        self.vz: List[float] = []

    def collect_data(self, aircraft_state: AircraftState) -> None:
        self.time.append(self.sim.get_time())
        self.lat_m.append(self.sim.get_local_position()[0])
        self.long_m.append(self.sim.get_local_position()[1])
        self.alt.append(self.sim.get_local_position()[2])

        self.roll_dg.append(self.sim.get_local_orientation()[
                            0] * (180 / math.pi))
        self.pitch_dg.append(self.sim.get_local_orientation()[
                             1] * (180 / math.pi))
        self.yaw_dg.append(self.sim.get_local_orientation()
                           [2] * (180 / math.pi))

        self.p.append(self.sim[prp.p_radps])
        self.q.append(self.sim[prp.q_radps])
        self.r.append(self.sim[prp.r_radps])

        self.airspeed.append(feet_to_meters(self.sim[prp.airspeed]))

        self.aileron_combined.append(
            self.sim[prp.aileron_combined_rad] * (180.0 / math.pi))
        self.elevator.append(self.sim[prp.elevator] * (180.0 / math.pi))

        self.x.append(aircraft_state.x)
        self.y.append(aircraft_state.y)
        self.z.append(aircraft_state.z)
        # self.vx.append(aircraft_state.vx)
        # self.vy.append(aircraft_state.vy)
        # self.vz.append(aircraft_state.vz)

        return None


class DataVisualizer():
    def __init__(self, sim_results: SimResults) -> None:
        self.report: SimResults = sim_results
        # self.report_graphs:ReportGraphs = ReportGraphs(sim)

    def plot_3d_trajectory(self, start_time: float = None,
                           end_time: float = None,
                           color: str = 'blue',
                           swap_xy: bool = True) -> Tuple[plt.Figure, plt.Axes]:

        if start_time is None:
            start_time = 0
        if end_time is None:
            end_time = self.report.time[-1]

        if swap_xy:
            x = self.report.y
            y = self.report.x

            self.report.x = x
            self.report.y = y

        # fig, ax = plt.subplots()
        # ax = fig.add_subplot(111, projection='3d')
        fig, ax = plt.subplots(subplot_kw={'projection': '3d'})

        # get the indices of the time
        ax.plot(self.report.x, self.report.y, self.report.z, label='trajectory',
                linewidth=2, color=color)
        # plot start and end points
        ax.scatter(self.report.x[0], self.report.y[0],
                   self.report.z[0], color='green', label='start')
        ax.scatter(self.report.x[-1], self.report.y[-1],
                   self.report.z[-1], color='red', label='end')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_zlabel('z [m]')
        ax.legend()

        return fig, ax

    def plot_attitudes(self, start_time: float = None,
                       end_time: float = None) -> Tuple[plt.Figure, plt.Axes]:
        if start_time is None:
            start_time = 0
        if end_time is None:
            end_time = self.report.time[-1]

        labels = ['roll', 'pitch', 'yaw']

        # create subplots for roll, pitch, yaw
        fig, ax = plt.subplots(3, 1, figsize=(10, 10))
        ax[0].plot(self.report.time, self.report.roll_dg,
                   label='roll', color='blue')
        ax[1].plot(self.report.time, self.report.pitch_dg,
                   label='pitch', color='blue')
        ax[2].plot(self.report.time, self.report.yaw_dg,
                   label='yaw', color='blue')

        # plot the reference values
        # TODO: Add reference values

        for a in ax:
            a.set_xlabel('Time [s]')
            a.set_ylabel('Angle [deg]')
            a.legend()

        return fig, ax

    def plot_airspeed(self, start_time: float = None,
                      end_time: float = None) -> Tuple[plt.Figure, plt.Axes]:
        if start_time is None:
            start_time = 0

        if end_time is None:
            end_time = self.report.time[-1]

        fig, ax = plt.subplots()
        ax.plot(self.report.time, self.report.airspeed,
                label='airspeed', color='blue')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Airspeed [kts]')
        ax.legend()

        return fig, ax
