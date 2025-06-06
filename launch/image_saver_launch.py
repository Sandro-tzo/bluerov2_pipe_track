from simple_launch import SimpleLauncher

def generate_launch_description():
    """
    Launch file minimale per avviare solo il nodo image_saver.
    """

    # 1. Inizializza SimpleLauncher
    sl = SimpleLauncher()

    # 2. Dichiara gli argomenti che vuoi poter configurare da riga di comando
    sl.declare_arg('image_topic', default_value='/bluerov2/image',
                   description='Il topic dell\'immagine a cui sottoscriversi')

    sl.declare_arg('output_path', default_value='~/rov_saved_images',
                   description='La cartella dove salvare le immagini')

    sl.declare_arg('save_every_n_frames', default_value='30',
                   description='Salva un\'immagine ogni N fotogrammi')

    sl.declare_arg('file_format', default_value='jpg',
                   description='Formato del file immagine (es. jpg, png)')

    # 3. Definisce il nodo da lanciare, passando gli argomenti come parametri
    sl.node(
        package='bluerov2_pipe_track',
        executable='image_saver',
        name='image_saver',
        output='screen',
        parameters={
            'image_topic': sl.arg('image_topic'),
            'output_path': sl.arg('output_path'),
            'save_every_n_frames': sl.arg('save_every_n_frames'),
            'file_format': sl.arg('file_format'),
        }
    )

    # 4. Ritorna la descrizione del lancio
    return sl.launch_description()