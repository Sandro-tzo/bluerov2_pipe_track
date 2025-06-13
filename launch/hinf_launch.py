from simple_launch import SimpleLauncher
# Non sono necessarie altre importazioni specifiche per le modifiche di base,
# SimpleLauncher gestisce la maggior parte dei dettagli.

def generate_launch_description():

    sl = SimpleLauncher(use_sim_time = True)

    # Queste dichiarazioni di argomenti rimangono invariate
    sl.declare_arg('namespace', default_value='bluerov2')
    sl.declare_arg('sliders', default_value=True) # Manteniamo i sliders, userai 'pose_control' per i setpoint
    sl.declare_arg('rviz', default_value=True)    # Manteniamo rviz per la visualizzazione

    # Il tuo HinfController e gli slider saranno in questo gruppo di namespace (es. /bluerov2)
    with sl.group(ns=sl.arg('namespace')):

        sl.node(
            package='bluerov2_pipe_track',  # <<< PACCHETTO del tuo HinfController
            executable='hinf_int', # <<< ESEGUIBILE del tuo HinfController
            name='hinf_controller',      # Nome del nodo ROS (quello che hai usato in ControllerIO("hinf_controller"))
            output='screen',
            remappings=[
                # Input Odometria per HinfController:
                # ControllerIO si aspetta '~/odom'. Se il nodo è '/bluerov2/hinf_controller',
                # questo topic completo è '/bluerov2/hinf_controller/odom'.
                # Lo mappiamo al topic di odometria pubblicato da Gazebo per il BlueROV2.
                ('odom', '/odom_estim'),

                # Input Setpoint di Posa per HinfController:
                # ControllerIO si aspetta '~/pose_setpoint' (cioè '/bluerov2/hinf_controller/pose_setpoint').
                # Lo slider 'pose_control' (definito sotto) pubblica su '/bluerov2/pose_control/pose_setpoint'.
                # Quindi, rimappiamo la sottoscrizione del HinfController per ascoltare l'output dello slider.
                ('~/pose_setpoint', '/bluerov2/pose_control/pose_setpoint'),

                # Output Comando di Wrench da HinfController:
                # ControllerIO pubblica su '~/wrench_command' (cioè '/bluerov2/hinf_controller/wrench_command').
                # Questo deve essere mappato all'input del thruster_manager.
                ('~/wrench_command', '/bluerov2/thruster_manager/input_wrench')
            ]
        )
        # --- FINE MODIFICA ---

        # Manteniamo i sliders. Il tuo HinfController userà 'pose_control' per ricevere i setpoint.
        with sl.group(if_arg='sliders'):
            # Questo slider pubblica i setpoint di posa.
            # Il suo output di default (dato il nome del nodo 'pose_control' e il namespace 'bluerov2')
            # sarà '/bluerov2/pose_control/pose_setpoint'.
            # Il tuo HinfController è stato rimappato per ascoltare questo topic.
            sl.node('slider_publisher', 'slider_publisher', name='pose_control',
                    arguments=[sl.find('auv_control', 'pose_setpoint.yaml')])

            # Questo slider è per il tilt. Il tuo HinfController, nella configurazione attuale,
            # non usa direttamente il suo output. Puoi mantenerlo o commentarlo se non ti serve.
            # sl.node('slider_publisher', 'slider_publisher', name='tilt_control',
                    # arguments=[sl.find('bluerov2_control', 'tilt.yaml')])

    # Manteniamo rviz per la visualizzazione
    with sl.group(if_arg='rviz'):
        sl.include('bluerov2_control','rviz_launch.py',
                   launch_arguments={'namespace': sl.arg('namespace'), 'use_sim_time': sl.sim_time})

    return sl.launch_description()