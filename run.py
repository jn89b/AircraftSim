from aircraftsim import (
    X8Autopilot,
    C172Autopilot,
    SimInterface
)


from aircraftsim import (
    FlightDynamics
)

from aircraftsim import (
    LocalNavigation,
    WindEstimation
)

from aircraftsim import (
    AircraftStateLimits,
    HighLevelControlLimits,
    LowLevelControlLimits,
    AircraftState,
    AircraftIC,
    HighControlInputs,
    DataVisualizer
)

import matplotlib.pyplot as plt
import matplotlib
import numpy as np
matplotlib.use("TkAgg")

"""
To do write the equations of motion right here and see if it matches
with the actual trajectory after running the simulation
"""


state_limits = AircraftStateLimits(
    x_bounds=[-100, 100],
    y_bounds=[-100, 100],
    z_bounds=[-100, 100],
    roll_bounds=[-np.deg2rad(45), np.deg2rad(45)],
    pitch_bounds=[-np.deg2rad(45), np.deg2rad(45)],
    yaw_bounds=[-np.deg2rad(180), np.deg2rad(180)],
    airspeed_bounds=[15, 30])

hl_ctrl_limits = HighLevelControlLimits(
    roll_rate=[-np.deg2rad(45), np.deg2rad(45)],
    pitch_rate=[-np.deg2rad(45), np.deg2rad(45)],
    yaw_rate=[-np.deg2rad(45), np.deg2rad(45)],
    vel_cmd=[15, 30])

init_cond = AircraftIC(
    x=0, y=0, z=50,
    roll=np.deg2rad(0),
    pitch=np.deg2rad(0),
    yaw=np.deg2rad(0),
    airspeed_m=20)

sim = SimInterface(
    aircraft_name='x8',
    init_cond=init_cond,
    sim_freq=200
)

N = 8000
high_lvl_ctrl = HighControlInputs(
    ctrl_idx=1,
    alt_ref_m=50,
    heading_ref_deg=20,
    vel_cmd=20,
)

high_lvl_ctrl = HighControlInputs(
    ctrl_idx=0,
    roll=0,
    pitch=0,
    yaw=0,
    vel_cmd=20
)

# create a sinusoid for heading ref deg
heading_ref_deg = 40*np.sin(np.linspace(0, 2*np.pi, N))
total_time = N/sim.sim_freq
print("Total time: ", total_time)


def compute_los(goal_x: float, goal_y: float, sim: SimInterface):
    current_state: AircraftState = sim.get_states()
    dy = goal_x - current_state.x
    dx = goal_y - current_state.y
    los = np.arctan2(dy, dx)

    return los


goal_x = -50
goal_y = 150

for i in range(N):
    goal_x += 5

    los = compute_los(goal_x, goal_y, sim)
    # los = np.pi/2 - los
    # los_dg = np.rad2deg(los)
    # high_lvl_ctrl = HighControlInputs(
    #     ctrl_idx=1,
    #     alt_ref_m=50,
    #     heading_ref_deg=los_dg,
    #     vel_cmd=15,
    # )
    # can I map the los to the roll?
    current_yaw = sim.get_states().yaw
    error = los - current_yaw
    print("Error: ", np.rad2deg(error))
    # wrap the error to -pi to pi
    if error > np.pi:
        error = error - 2*np.pi
    elif error < -np.pi:
        error = error + 2*np.pi
    roll = np.arctan2(error, -9.81)
    roll = np.clip(roll, -np.deg2rad(35), np.deg2rad(35))

    # print("Roll: ", np.rad2deg(roll))
    # high_lvl_ctrl = HighControlInputs(
    #     ctrl_idx=0,
    #     roll=roll,
    #     pitch=np.deg2rad(-10.0),
    #     alt_ref_m=50,
    #     yaw=0,
    #     vel_cmd=20
    # )

    sim.step(high_lvl_ctrl=high_lvl_ctrl)

report = sim.report
data_vis = DataVisualizer(report)
fig, ax = data_vis.plot_3d_trajectory()
ax.scatter(goal_x, goal_y, 50)
fig1, ax1 = data_vis.plot_attitudes()
fig2, ax2 = data_vis.plot_airspeed()
# save figure
plt.show()
