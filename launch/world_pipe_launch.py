# File di lancio per avviare la simulazione del BlueROV2 con il nodo UKF per la stima dell'odometria.

from simple_launch import SimpleLauncher, GazeboBridge

def generate_launch_description():
    """
    Genera la descrizione del lancio per la simulazione e il nodo UKF.
    """
    sl = SimpleLauncher()

    sl.declare_arg('gui', default_value=True, description='Avvia Gazebo con GUI se True')
    sl.declare_arg('spawn', default_value=True, description='Spawna il modello del BlueROV2 se True')

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

    # # Gruppo di nodi per il BlueROV2
    # with sl.group(ns='bluerov2'):

    #     # Nodo che simula un DVL. Pubblica su /bluerov2/dvl/twist.
    #     sl.node(
    #         package='bluerov2_pipe_track',
    #         executable='odom_to_dvl',
    #         name='odom_to_dvl_node',
    #         remappings={
    #             'odom': '/bluerov2/odom'
    #         }
    #         # Nessun remapping per l'output, è già corretto
    #     )
        
    #     # Nodo che simula una misura di posizione (MLAT). Pubblica su /bluerov2/mlat.
    #     sl.node(
    #         package='bluerov2_pipe_track',
    #         executable='mlat_node',
    #         name='mlat_node',
    #         output='screen',
    #         parameters=[{
    #             'noise_std_dev': 0.15 
    #         }]
    #         # Nessun remapping necessario
    #     )
        
    #     # --- NODO UKF PER LA STIMA DELL'ODOMETRIA ---
    #     sl.node(
    #         package='bluerov2_pipe_track',
    #         executable='ukf_node',
    #         name='ukf_filter_node',
    #         output='screen',
    #         parameters=[{
    #             'gravity': 9.81,
    #             'noise_proc': {
    #                 'pos': 0.1,
    #                 'quat': 1e-4,
    #                 'nu': 0.005,
    #                 'bgyro': 7e-7,
    #                 'bacc': 1e-4,
    #             },
    #             'noise_meas': {
    #                 'dvl_vel_var': 0.000025,
    #                 'mlat_pos_var': 0.0225
    #             }
    #         }],
    #         remappings={
    #             # Remapping dei topic di input del filtro ai topic sorgente globali
    #             'imu/data': '/bluerov2/mpu',
    #             'dvl/twist': '/bluerov2/dvl/twist',  # <-- MODIFICATO E SEMPLIFICATO
    #             'mlat/pose': '/bluerov2/mlat'       # <-- MODIFICATO E SEMPLIFICATO
    #         }
    #    )

    return sl.launch_description()