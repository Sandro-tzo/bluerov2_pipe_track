# BlueROV2 - Missione di Ispezione Pipeline (Pipe Tracking)

Questo repository contiene un progetto completo in ROS 2 per la simulazione di una missione di ispezione di una pipeline sottomarina con un AUV di tipo BlueROV2. Il progetto dimostra uno stack di autonomia completo che include percezione, stima dello stato e controllo avanzato.

Il sistema è basato su una macchina a stati finiti che gestisce la logica della missione, un filtro UKF per la stima della posa e un controllore H-infinity custom per il tracciamento della traiettoria.

## Key Features

-   **Simulazione in Gazebo**: Ambiente sottomarino con una pipeline da seguire.
-   **Rilevamento della Pipeline**: Nodo di visione computerizzata per rilevare la linea scura della pipeline in un'immagine.
-   **Stima dello Stato**: Utilizzo del pacchetto `robot_localization` con un **Unscented Kalman Filter (UKF)** per fondere i dati dei sensori simulati (IMU, DVL, MLAT).
-   **Controllo Avanzato**: Implementazione di un **controllore H-infinity** (`h2_int`) che si integra con il framework `auv_control` per il controllo di basso livello.
-   **Macchina a Stati (FSM)**: Un nodo principale (`pipe_tracker`) che gestisce la logica della missione:
    1.  **DIVING**: Immersione alla quota operativa.
    2.  **SEARCHING**: Ricerca attiva della pipeline.
    3.  **TRACKING**: Inseguimento della pipeline una volta rilevata.
    4.  **RETURN TO HOME**: Sequenza di rientro manuale (emersione e ritorno al punto di partenza).
-   **Supervisione Manuale**: Possibilità per l'operatore di terminare la missione e avviare la sequenza di rientro tramite un servizio ROS 2.

## System Architecture

Il flusso dei dati nel sistema è il seguente:
1.  **Gazebo** simula l'ambiente, la fisica del BlueROV2 e genera dati di odometria "ground truth".
2.  I nodi **sensori simulati** (`odom_to_dvl`, `odom_to_mlat`) e i bridge di Gazebo producono topic di sensori realistici (IMU, DVL, MLAT) a partire dai dati di Gazebo.
3.  Il nodo **UKF** (`robot_localization`) fonde questi dati per produrre una stima dell'odometria filtrata e robusta (`/bluerov2/ukf/odom`).
4.  Il nodo **Tracker** (`pipe_tracker`) utilizza l'**odometria filtrata** e i dati della **telecamera** per eseguire la sua logica. In base allo stato corrente (es. `TRACKING`), calcola e pubblica i setpoint di posa (`cmd_pose`) e velocità (`cmd_vel`).
5.  Il nodo **Controllore** (`h2_controller`) riceve i setpoint e lo stato attuale, e calcola lo sforzo richiesto (`wrench`) per raggiungere l'obiettivo.
6.  Il **Thruster Manager** riceve il comando di `wrench` e lo converte in comandi di spinta per i singoli propulsori del BlueROV2 in Gazebo.

## Dependencies

Questo pacchetto dipende da diversi altri pacchetti ROS 2:

-   **`robot_localization`**: Per il filtro UKF.
-   **`auv_control`**: Per la libreria di base del controllore.
-   **`thruster_manager`**: Per l'allocazione della spinta.
-   **`bluerov2_description`**: Per il modello URDF del robot.
-   **`simple_launch`**: Per la gestione semplificata dei file di lancio.

Per installare le dipendenze mancanti, esegui dal tuo workspace:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Installation and Build

1.  Clona questo repository nella cartella `src` del tuo workspace ROS 2:
    ```bash
    cd ~/your_ros2_ws/src
    git clone https://github.com/Sandro-tzo/bluerov2_pipe_track.git
    ```
2.  Torna alla radice del workspace e compila:
    ```bash
    cd ~/your_ros2_ws
    colcon build
    ```
3.  Esegui il source del workspace per rendere i pacchetti disponibili:
    ```bash
    source install/setup.bash
    ```

## Usage

1.  **Avviare la Simulazione Completa**:
    Per avviare l'ambiente Gazebo, i nodi di controllo, stima e il tracker, lancia il file di lancio principale:
    ```bash
    ros2 launch bluerov2_pipe_track world_pipe_launch.py
    ros2 launch bluerov2_pipe_track ukf_launch.py
    ros2 launch bluerov2_pipe_track pipe_tracker_launch.py
    ```

2.  **Comandare il Rientro a Casa**:
    Mentre la simulazione è in esecuzione, per terminare la missione e far tornare il robot al punto di partenza, apri un nuovo terminale (eseguendo prima il `source`) e chiama il servizio dedicato:
    ```bash
    ros2 service call /bluerov2/autonomous_tracker/trigger_return_home std_srvs/srv/Trigger '{}'
    ```

## Node Configuration

-   **UKF**: I parametri del filtro (matrici di covarianza, sensori da usare, etc.) sono definiti in `config/ukf_bluerov.yaml`.
-   **Controller H-2**: La matrice di guadagno `K` è definita direttamente nel codice sorgente `src/h2_int.cpp`.
-   **Tracker**: I parametri della missione (profondità, velocità, soglie di visione) sono configurati direttamente nel file di lancio.