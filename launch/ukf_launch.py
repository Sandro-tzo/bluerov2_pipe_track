import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_name = 'bluerov2_pipe_track'
    
    # Percorso al tuo file di configurazione YAML.
    # Assicurati che questo file usi 'bluerov2/base_link' come base_link_frame.
    config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'ukf_bluerov.yaml'
    )

    # Parametro comune per usare il tempo di simulazione
    use_sim_time_param = {'use_sim_time': True}

    # --- Nodi Simulatori C++ ---
    # Questi nodi vengono lanciati INSIEME al filtro.
    
    # Il tuo nodo che simula un DVL
    odom_to_dvl_node = Node(
        package=pkg_name,
        executable='odom_to_dvl',
        name='odom_to_dvl_node',
        parameters=[use_sim_time_param],
        remappings=[
            ('odom', '/bluerov2/odom')
        ]
    )
    
    # Il tuo nodo che simula una misura di posizione (MLAT)
    mlat_node = Node(
        package=pkg_name,
        executable='mlat_node',
        name='mlat_node',
        parameters=[
            use_sim_time_param,
            {'noise_std_dev': 0.15}
        ]
    )

    # --- Nodo di Stima dal Pacchetto robot_localization ---
    ukf_filter_node = Node(
        package='robot_localization',
        executable='ukf_node',
        name='ukf_filter_node',
        output='screen',
        parameters=[config_file, use_sim_time_param],
        remappings=[
            ('/odometry/filtered', 'odom_estim')
        ]
    )

    # ====================================================================================
    # --- MODIFICA: Aggiunta dei Publisher per le Trasformazioni Statiche (TF) ---
    # Questi nodi risolvono il problema dei frame_id degli IMU.
    # Assumiamo che i sensori siano montati al centro del base_link (offset x,y,z = 0,0,0).
    # Se così non fosse, aggiungi gli argomenti --x, --y, --z, etc.
    # ====================================================================================

    # Trasformazione per il primo IMU (MPU)
    static_tf_mpu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_mpu',
        parameters=[use_sim_time_param],
        arguments=['--frame-id', 'bluerov2/base_link', '--child-frame-id', 'bluerov2/bluerov2/base_link/mpu_imu']
    )

    # Trasformazione per il secondo IMU (LSM)
    static_tf_lsm_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_lsm',
        parameters=[use_sim_time_param],
        arguments=['--frame-id', 'bluerov2/base_link', '--child-frame-id', 'bluerov2/bluerov2/base_link/lsm_imu']
    )


    return LaunchDescription([
        # Avviamo i due publisher TF, i tuoi simulatori e il filtro.
        static_tf_mpu_node,
        static_tf_lsm_node,
        odom_to_dvl_node,
        mlat_node,
        ukf_filter_node
    ])