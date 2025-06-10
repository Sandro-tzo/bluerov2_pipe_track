# launch/autonomous_tracker.launch.py

from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher(use_sim_time=True)

    sl.declare_arg('namespace', default_value='bluerov2')
    sl.declare_arg('rviz', default_value=True)

    with sl.group(ns=sl.arg('namespace')):

        # Nodo Odom->TF: ora sa che il frame genitore è 'world'
        sl.node(
            package='bluerov2_pipe_track',  # <<< USA IL NOME DEL TUO PACCHETTO
            executable='odom_to_tf',
            name='odom_to_tf_publisher',
            parameters=[
                {'odom_topic': 'odom'},
                # Forziamo il frame corretto letto dal simulatore
                {'fallback_parent_frame': 'world'} 
            ]
        )
        
        # Nodo Tracker Autonomo: ora sa che deve pubblicare setpoint nel frame 'world'
        sl.node(
            package='bluerov2_pipe_track',  # <<< USA IL NOME DEL TUO PACCHETTO
            executable='pipe_tracker',
            name='autonomous_tracker',
            output='screen',
            parameters=[
                {'image_topic': 'image'},
                {'odom_topic': 'odom'},
                {'velocity_setpoint_topic': 'cmd_vel'},
                {'pose_setpoint_topic': 'cmd_pose'},
                {'robot_frame_id': 'bluerov2/base_link'},
                # CORREZIONE FONDAMENTALE QUI
                {'world_frame_id': 'world'}, 
                {'depth_setpoint': -8.0},
                # ... etc ...
            ]
        )

        # Nodo Hinf Controller (nessuna modifica necessaria qui, i remapping sono generici)
        sl.node(
            package='bluerov2_pipe_track',
            executable='hinf_int',
            name='hinf_controller',
            output='screen',
            remappings=[
                ('~/odom', 'odom'),
                ('~/pose_setpoint', '/bluerov2/cmd_pose'),
                ('~/velocity_setpoint', '/bluerov2/cmd_vel'),
                ('~/wrench_command', 'thruster_manager/input_wrench')
            ]
        )

    # RVIZ
    with sl.group(if_arg='rviz'):
        sl.include('bluerov2_control', 'rviz_launch.py',
                   launch_arguments={'namespace': sl.arg('namespace'), 'use_sim_time': sl.sim_time})

    return sl.launch_description()

    #ros2 service call /bluerov2/autonomous_tracker/trigger_return_home std_srvs/srv/Trigger '{}'