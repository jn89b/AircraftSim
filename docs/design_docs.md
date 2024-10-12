- I want to be able to pull this library and use it:
    - Library called Aircraft Sim
    - Define limits and capabilities of aircraft
    - Define fidelity of the simulator:
        - Kinematics or JSBSIM
- When you call out the step function:
    - Aircraft will give you positions and various information of the aircraft




To create aircraft sim must define the following:
- Aircraft name
- Fidelity Type:optional [JSBSIM, KINEMATICS]
- Integration Type:optional [EULER, RK4]
- sim_freq:optional
- is_high_lvl_ctrl -> used if you want to control r,p,y,vel_cmd
- init_cond: InitialCondition 
- state_limit: Declare the limits of your aircraft
- high_control_limit: limit the control bounds of your aircraft
- low_level_control_limit

User creates the aircraft sim
- Calls out the step and inputs controls whether high level or low level
- Get's results 
- Keeps looping
 

## Note
- JSBSIM is in NED Coordinate frame:
- X = North
- Y = East
- Z = Down


## Heading Control Notes
- The heading control orientation is as follows:
    - 0 degrees is north in the y direction
    - 90 degrees is east in the x direction