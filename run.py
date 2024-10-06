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

state_limits = AircraftStateLimits(
    x_bounds=[-100,100],
    y_bounds=[-100,100],
    z_bounds=[-100,100],
    roll_bounds=[-np.deg2rad(45), np.deg2rad(45)],
    pitch_bounds=[-np.deg2rad(45), np.deg2rad(45)],
    yaw_bounds = [-np.deg2rad(180), np.deg2rad(180)],
    airspeed_bounds=[15, 30])

hl_ctrl_limits = HighLevelControlLimits(
    roll_rate=[-np.deg2rad(45), np.deg2rad(45)],
    pitch_rate=[-np.deg2rad(45), np.deg2rad(45)],
    yaw_rate=[-np.deg2rad(45), np.deg2rad(45)],
    vel_cmd=[15,30])

init_cond = AircraftIC(
    x=0, y=0, z= 50, 
    roll=np.deg2rad(0),
    pitch=np.deg2rad(0),
    yaw=np.deg2rad(0),
    airspeed_m=20)

sim = SimInterface(
    aircraft_name='x8',
    init_cond=init_cond,
    high_control_lim=hl_ctrl_limits,
    state_lim=state_limits,
    sim_freq=200
)

N = 2000
high_lvl_ctrl = HighControlInputs(
    ctrl_idx=1,
    alt_ref_m=50,
    heading_ref_deg=-20,
    vel_cmd=20,
)


high_lvl_ctrl = HighControlInputs(
    ctrl_idx=1,
    alt_ref_m=50,
    heading_ref_deg=-20,
    vel_cmd=20,
)

for i in range(N):
    sim.step(high_lvl_ctrl=high_lvl_ctrl)

report = sim.report
data_vis = DataVisualizer(report)
fig, ax = data_vis.plot_3d_trajectory()
fig1, ax1 = data_vis.plot_attitudes()
#save figure
plt.show()