import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml 

def generate_launch_description():
    
    pkg_name = 'bluerov2_pipe_track'
    common_namespace = 'bluerov2'
    
    # Percorso al file di configurazione YAML
    config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'ukf_bluerov.yaml'
    )

    # --- FASE 1: Carichiamo il file YAML e prepariamo un unico dizionario di parametri ---
    with open(config_file, 'r') as f:
        # Carica la struttura YAML e prendi solo la sezione dei parametri per il nostro nodo
        params = yaml.safe_load(f)['ukf_node']['ros__parameters']
    
    # --- FASE 2: Aggiungiamo il parametro 'use_sim_time' direttamente al dizionario ---
    params['use_sim_time'] = True

    # --- Nodi Simulatori  ---
    dvl_node = Node(
        package=pkg_name,
        executable='odom_to_dvl',
        name='dvl_node',
        namespace=common_namespace,
        parameters=[{'use_sim_time': True}],
        remappings=[('odom', '/bluerov2/odom'), ('twist', '/bluerov2/dvl/twist')]
    )
    
    mlat_node = Node(
        package=pkg_name,
        executable='odom_to_mlat',
        name='mlat_node',
        namespace=common_namespace,
        parameters=[{'use_sim_time': True}, {'noise_std_dev': 0.15}],
        remappings=[('odom', '/bluerov2/odom'), ('pose', '/bluerov2/mlat/pose')]
    )

    # --- Nodo di Stima UKF  ---
    ukf_node = Node(
        package='robot_localization',
        executable='ukf_node',
        name='ukf_node',
        namespace=common_namespace,
        output='screen',
        parameters=[params],
        remappings=[
            ('/bluerov2/odometry/filtered', '/bluerov2/ukf/odom')
        ]
    )

    # ... definizioni dei nodi static_tf  ...
    static_tf_mpu_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_mpu',
        parameters=[{'use_sim_time': True}],
        arguments=['--frame-id', 'bluerov2/base_link', '--child-frame-id', 'bluerov2/bluerov2/base_link/mpu_imu']
    )
    static_tf_lsm_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_lsm',
        parameters=[{'use_sim_time': True}],
        arguments=['--frame-id', 'bluerov2/base_link', '--child-frame-id', 'bluerov2/bluerov2/base_link/lsm_imu']
    )

    return LaunchDescription([
        static_tf_mpu_node,
        static_tf_lsm_node,
        dvl_node,
        mlat_node,
        ukf_node
    ])