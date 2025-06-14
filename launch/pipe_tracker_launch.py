from simple_launch import SimpleLauncher

def generate_launch_description():
    """
    Avvia i nodi di controllo per la missione di pipe tracking.

    Per far rientrare il robot, usare il servizio:
    ros2 service call /bluerov2/autonomous_tracker/trigger_return_home std_srvs/srv/Trigger '{}'
    """
    sl = SimpleLauncher(use_sim_time=True)

    # --- Argomenti di Lancio ---
    sl.declare_arg('namespace', default_value='bluerov2', description='Namespace per i nodi del robot.')
    sl.declare_arg('rviz', default_value=True, description='Avvia RViz per la visualizzazione.')

    # --- Gruppo di Nodi Principali ---
    with sl.group(ns=sl.arg('namespace')):

        # Nodo Tracker Autonomo (Macchina a Stati)
        sl.node(
            package='bluerov2_pipe_track',
            executable='pipe_tracker',
            name='autonomous_tracker',
            output='screen',
            parameters=[{
                # Topic e Frame
                'image_topic': 'image',
                'odom_topic': '/bluerov2/ukf/odom',
                'velocity_setpoint_topic': 'cmd_vel',
                'pose_setpoint_topic': 'cmd_pose',
                'robot_frame_id': 'bluerov2/base_link',
                'world_frame_id': 'world', 
                
                # Parametri di Visione
                'hsv_v_low': 0,
                'hsv_v_high': 50,
                'min_area': 5000,

                # Parametri di Missione e Controllo
                'depth_setpoint': -8.0,
                'depth_tolerance': 0.5,
                'constant_forward_speed': 0.5,
                'yaw_correction_gain': 0.005,
                'search_yaw_velocity': 0.2,
                'surface_depth': 0.0,
                'surface_depth_tolerance': 0.5,
                'home_position_tolerance': 1.0,
                'return_home_step_size_m': 1.5,
            }]
        )

        # Nodo H2 Controller
        sl.node(
            package='bluerov2_pipe_track',
            executable='h2_int',
            name='h2_controller',
            output='screen',
            remappings=[
                # Input: Odometria dal filtro UKF
                ('odom', '/bluerov2/ukf/odom'),
                
                # Input: Setpoint di posa dal tracker
                ('~/pose_setpoint', 'cmd_pose'),

                # Input: Setpoint di velocità dal tracker
                ('~/velocity_setpoint', 'cmd_vel'),
                
                # Output: Comando di sforzo (wrench) al gestore dei propulsori
                ('~/wrench_command', 'thruster_manager/input_wrench')
            ]
        )

    # Avvio di RViz 
    with sl.group(if_arg='rviz'):
        sl.include('bluerov2_control', 'rviz_launch.py',
                   launch_arguments={'namespace': sl.arg('namespace'), 'use_sim_time': sl.sim_time})

    return sl.launch_description()