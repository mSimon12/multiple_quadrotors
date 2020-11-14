# multiple_quadrotors
This is an implementation of multiple quadrotor capable of mapping the environment and collision avoidance.

To run it first launch the system with:
roslaunch full_system_launcher system.launch

To set a position for any of the UAVs use:
rosrun quadrotor_control_system move_drone.py drone_name X Y Z THETA

- drone_name = (UAV_1, UAV_2, ...);
- X = quadrotor x desired position;
- Y = quadrotor y desired position;
- Z = quadrotor z desired position;
- THETA = quadrotor desired orientation.
