# BlueROV2 - Pipeline Inspection Mission (Pipe Tracking)

This repository contains a complete ROS 2 project for simulating a subsea pipeline inspection mission with a BlueROV2-type AUV. The project demonstrates a full autonomy stack, including perception, state estimation, and advanced control.

The system is based on a finite state machine (FSM) that manages the mission logic, a UKF for pose estimation, and a custom H-2 controller for trajectory tracking.

## Key Features

-   **Gazebo Simulation**: Underwater environment with a pipeline to follow.
-   **Pipeline Detection**: Computer vision node to detect the pipeline's dark line in an image stream.
-   **State Estimation**: Utilizes the `robot_localization` package with an **Unscented Kalman Filter (UKF)** to fuse simulated sensor data (IMU, DVL, MLAT).
-   **Advanced Control**: Implements a custom **H-2 controller** (`h2_int`) integrated with the `auv_control` framework for low-level control.
-   **Finite State Machine (FSM)**: A main node (`pipe_tracker`) manages the mission logic through several states:
    1.  **DIVING**: Dives to the operational depth.
    2.  **SEARCHING**: Actively searches for the pipeline.
    3.  **TRACKING**: Tracks the pipeline once detected.
    4.  **RETURN TO HOME**: Executes a manual return sequence (surfacing and returning to the start point).
-   **Manual Supervision**: Allows the operator to end the mission and initiate the return sequence via a ROS 2 service.

## System Architecture

The data flow in the system is as follows:
1.  **Gazebo** simulates the environment, vehicle physics, and generates ground truth odometry.
2.  **Simulated Sensor Nodes** (`odom_to_dvl`, `odom_to_mlat`) and Gazebo bridges produce realistic sensor topics from ground truth data.
3.  The **UKF node** (`robot_localization`) fuses sensor data to produce a robust, filtered odometry estimate (`/bluerov2/ukf/odom`).
4.  The **Tracker node** (`pipe_tracker`) uses the filtered odometry and camera data to execute its state machine logic, publishing pose (`cmd_pose`) and velocity (`cmd_vel`) setpoints.
5.  The **Controller node** (`h2_controller`) receives the setpoints and current state, and calculates the required wrench (forces and torques).
6.  The **Thruster Manager** receives the wrench command and converts it into individual thrust commands for the simulated thrusters.

## Dependencies

This package depends on several other ROS 2 packages:

-   **`bluerov2_description`**: For the robot's URDF model.
-   **`auv_control`**: For the base controller library.
-   **`thruster_manager`**: For thrust allocation.
-   **`simple_launch`**: For simplified management of launch files.
-   **`robot_localization`**: For the UKF filter.

To install missing dependencies that are not indicized by rosdep, from your workspace `src` folder, run:
```bash
git clone https://github.com/CentraleNantesROV/bluerov2.git
git clone https://github.com/CentraleNantesROV/auv_control.git
git clone https://github.com/CentraleNantesROV/thruster_manager.git
```

## Installation and Build

1.  Clone the repository into your ROS 2 workspace:
    ```bash
    cd ~/your_ros2_ws/src
    git clone https://github.com/Sandro-tzo/bluerov2_pipe_track.git
    ```
2.  Return to the workspace root and build the packages:
    ```bash
    cd ~/your_ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    colcon build
    ```
3.  Source the workspace to make the packages available:
    ```bash
    source install/setup.bash
    ```

## Usage

1.  **Launch the Full Simulation**:
    To launch the Gazebo environment and all required nodes, run the main launch files. It's recommended to run each in a separate terminal to see the specific logs:
    ```bash
    # Terminal 1: Launch the world and robot model
    ros2 launch bluerov2_pipe_track world_pipe_launch.py
    
    # Terminal 2: Launch the estimation (UKF) and sensor simulator nodes
    ros2 launch bluerov2_pipe_track ukf_launch.py
    
    # Terminal 3: Launch the mission logic and controller
    ros2 launch bluerov2_pipe_track pipe_tracker_launch.py
    ```

2.  **Command Return to Home**:
    While the simulation is running, to manually trigger the return sequence, open a new terminal (after sourcing the workspace) and call the service:
    ```bash
    ros2 service call /bluerov2/autonomous_tracker/trigger_return_home std_srvs/srv/Trigger '{}'
    ```

2.  **Simulate an Ocean Current (Optional)**:
    To simulate an example ocean current affecting the vehicle, you can publish a single message to the `/current` topic. This is useful for testing the controller's robustness against external disturbances. In a new terminal, run:
    ```bash
    ros2 topic pub --once /current geometry_msgs/msg/Vector3 '{x: 0.2, y: 0.6, z: 0.05}'
    ```


## Node Configuration

-   **UKF**: Filter parameters (covariance matrices, sensor configuration, etc.) are defined in `config/ukf_bluerov.yaml`.
-   **H-2 Controller**: The gain matrix `K` is hard-coded in the source file `src/h2_int.cpp`.
-   **Tracker**: Mission parameters (depth, speed, vision thresholds) are set in the launch file `launch/pipe_tracker_launch.py`.