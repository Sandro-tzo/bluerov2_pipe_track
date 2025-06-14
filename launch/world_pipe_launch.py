
from simple_launch import SimpleLauncher, GazeboBridge

def generate_launch_description():
    
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

    return sl.launch_description()


#Il comando per pubblicare una corrente oceanica di esempio
#ros2 topic pub --once /current geometry_msgs/msg/Vector3 '{x: 0.2, y: 0.6, z: 0.05}'