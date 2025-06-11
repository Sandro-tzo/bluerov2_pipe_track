import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Definisci il nome del tuo pacchetto
    pkg_name = 'bluerov2_pipe_track' # <-- SOSTITUISCI SE NECESSARIO
    
    # Percorso al file di configurazione YAML per il filtro UKF
    config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'ukf_bluerov.yaml'
    )

    # --- Nodi Simulatori ---
    # Questi sono i nodi che hai creato per simulare i sensori DVL e MLAT
    
    # Nodo che simula un DVL a partire dall'odometria di Gazebo
    # Pubblica su /bluerov2/dvl/twist
    odom_to_dvl_node = Node(
        package=pkg_name,
        executable='odom_to_dvl',
        name='odom_to_dvl_node',
        remappings=[
            ('odom', '/bluerov2/odom') # Input
        ]
    )
    
    # Nodo che simula una misura di posizione (MLAT)
    # Pubblica su /bluerov2/mlat
    mlat_node = Node(
        package=pkg_name,
        executable='mlat_node',
        name='mlat_node',
        parameters=[{
            'noise_std_dev': 0.15 
        }]
    )

    # --- Nodo di Stima dal Pacchetto robot_localization ---
    # Questo è il nodo UKF ufficiale che useremo.
    # L'input viene gestito dal file YAML, noi rimappiamo solo l'output.
    ukf_filter_node = Node(
        package='robot_localization',
        executable='ukf_node',
        name='ukf_filter_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/odometry/filtered', '/bluerov2/odom_estim') # Rinomina l'output
        ]
    )

    return LaunchDescription([
        odom_to_dvl_node,
        mlat_node,
        ukf_filter_node
    ])