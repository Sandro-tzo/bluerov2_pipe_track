from simple_launch import SimpleLauncher, GazeboBridge


def generate_launch_description():
    
    sl = SimpleLauncher()
    sl.declare_arg('gui', default_value=True)
    sl.declare_arg('spawn', default_value=True)

    with sl.group(if_arg='gui'):
        sl.gz_launch(sl.find('bluerov2_pipe_track', 'world_sine_pipe.sdf'), "-r")
        
    with sl.group(unless_arg='gui'):
        sl.gz_launch(sl.find('bluerov2_pipe_track', 'world_sine_pipe.sdf'), "-r -s")

    bridges = [GazeboBridge.clock(),
               GazeboBridge('/ocean_current', '/current', 'geometry_msgs/Vector3',
                            GazeboBridge.ros2gz)]
        
    sl.create_gz_bridge(bridges)

    with sl.group(if_arg='spawn'):
        sl.include('bluerov2_description', 'upload_bluerov2_launch.py')

    with sl.group(ns='bluerov2'):

        sl.node(
            package='bluerov2_pipe_track',
            executable='odom_to_dvl',
            name='odom_to_dvl_node',
            parameters=[{
                'noise_std_dev': 0.05
            }]
        )
        
        # ++ SINTASSI CORRETTA ++
        sl.node(
            # ... (package, executable, name sono invariati)
            package='bluerov2_pipe_track', 
            executable='ukf',
            name='ukf_filter_node',
            
            remappings={
                # SINTASSI CORRETTA: 'nome_interno_al_nodo': 'nome_reale_nel_sistema'
                'imu/data': '/bluerov2/mpu',  # NOTA: Ho usato /mpu come nel tuo messaggio di errore e rimosso lo / finale
                'dvl/twist': '/bluerov2/dvl/twist'
            }
        )
        
    return sl.launch_description()

# Corrente di circa 0.3 m/s con una leggera deviazione sull'asse Y
#ros2 topic pub --once /ocean_current geometry_msgs/msg/Vector3 '{x: 0.25, y: 0.15, z: 0.0}'