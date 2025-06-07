from simple_launch import SimpleLauncher

def generate_launch_description():
    """
    Launch file per avviare e configurare il nodo di rilevamento del cilindro/tubo.
    """
    sl = SimpleLauncher()

    # --- Argomenti di Configurazione ---
    # Namespace per il nodo, utile per organizzare i nodi ROS
    sl.declare_arg('namespace', default_value='bluerov2')

    # Argomento per il topic dell'immagine
    sl.declare_arg('image_topic', default_value='/bluerov2/image',
                   description='Il topic dell\'immagine a cui sottoscriversi')

    # NUOVI: Argomenti per la calibrazione del detector
    sl.declare_arg('hsv_v_low', default_value='0',
                   description='Valore minimo di "Value" (luminosità) per il nero in HSV')
                   
    sl.declare_arg('hsv_v_high', default_value='50',
                   description='Valore massimo di "Value" (luminosità) per il nero in HSV')
                   
    sl.declare_arg('min_area', default_value='1000',
                   description='Area minima in pixel per considerare valido un rilevamento')

    # --- Definizione del Nodo ---
    with sl.group(ns=sl.arg('namespace')):
        sl.node(
            package='bluerov2_pipe_track',
            # Assicurati che questo nome corrisponda all'eseguibile nel CMakeLists.txt
            executable='pipe_detector',
            name='pipe_detector',
            output='screen',
            parameters=[{
                # Passa tutti i parametri al nodo C++
                'image_topic': sl.arg('image_topic'),
                'hsv_v_low': sl.arg('hsv_v_low'),
                'hsv_v_high': sl.arg('hsv_v_high'),
                'min_area': sl.arg('min_area'),
            }]
        )

    return sl.launch_description()