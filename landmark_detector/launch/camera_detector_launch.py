from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Camera + Foxglove bridge (from robot_vision_camera) + YOLO detector + range estimate
# + AprilTag 36h11 node (HW3 uses tags 0-3 next to the four objects).
#
#   ros2 launch landmark_detector camera_detector_launch.py
#   ros2 launch landmark_detector camera_detector_launch.py model:=yolo11n_416.onnx
#   ros2 launch landmark_detector camera_detector_launch.py tag_size:=0.16
#
# Only the landmarks placed in the field are kept: the fan and mug classes fire on
# dark background. A white wastebasket is detected as toilet paper, so keep it out
# of the field.

LANDMARKS = ['water_bottle', 'toilet_paper', 'cereal_box', 'dog_plushie']


def generate_launch_description():
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('robot_vision_camera'), 'launch', 'robot_vision_camera.launch.py'])))

    detector = Node(
        package='landmark_detector',
        executable='detector_node',
        name='landmark_detector',
        output='screen',
        parameters=[{
            'model': LaunchConfiguration('model'),
            'conf': LaunchConfiguration('conf'),
            'threads': LaunchConfiguration('threads'),
            'max_fps': LaunchConfiguration('max_fps'),
            'classes': LANDMARKS,
        }],
    )

    ranges = Node(
        package='landmark_detector',
        executable='range_node',
        name='landmark_range',
        output='screen',
    )

    tags = Node(
        package='landmark_detector',
        executable='tag_node',
        name='tag_node',
        output='screen',
        parameters=[{'size': LaunchConfiguration('tag_size')}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='landmarks_round2_416.onnx'),
        DeclareLaunchArgument('conf', default_value='0.4'),
        DeclareLaunchArgument('threads', default_value='4'),
        DeclareLaunchArgument('max_fps', default_value='5.0'),
        # Effective tag size: printed tags measure 16 cm, but ranges read 5.7% long at 1 and 2 m
        # (tape from the lens), so 0.16 / 1.057 = 0.151 m makes them match the tape.
        DeclareLaunchArgument('tag_size', default_value='0.151'),
        camera,
        detector,
        ranges,
        tags,
    ])
