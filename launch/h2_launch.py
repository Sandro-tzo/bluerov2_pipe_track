from simple_launch import SimpleLauncher

def generate_launch_description():

    sl = SimpleLauncher(use_sim_time = True)
    sl.declare_arg('namespace', default_value='bluerov2')
    sl.declare_arg('sliders', default_value=True) #sliders
    sl.declare_arg('rviz', default_value=True)    #rviz per la visualizzazione

    with sl.group(ns=sl.arg('namespace')):

        sl.node(
            package='bluerov2_pipe_track',  
            executable='h2_int', 
            name='h2_controller',    
            output='screen',
            remappings=[
                # Input Odometria per H2Controller:
                ('odom', '/bluerov2/ukf/odom'),

                # Input Setpoint di Posa per H2Controller:
                ('~/pose_setpoint', '/bluerov2/pose_control/pose_setpoint'),

                # Output Comando di Wrench da H2Controller:
                ('~/wrench_command', '/bluerov2/thruster_manager/input_wrench')
            ]
        )
        

        # sliders
        with sl.group(if_arg='sliders'):
            # Questo slider pubblica i setpoint di posa.
            sl.node('slider_publisher', 'slider_publisher', name='pose_control',
                    arguments=[sl.find('auv_control', 'pose_setpoint.yaml')])

    # rviz per la visualizzazione
    with sl.group(if_arg='rviz'):
        sl.include('bluerov2_control','rviz_launch.py',
                   launch_arguments={'namespace': sl.arg('namespace'), 'use_sim_time': sl.sim_time})

    return sl.launch_description()