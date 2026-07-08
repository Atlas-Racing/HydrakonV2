import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

CONE_CLASS_IDS = range(5)  # class_000..class_004, see cone_detection.yaml


def launch_setup(context, *args, **kwargs):
    zed_wrapper_launch_dir = get_package_share_directory('zed_wrapper')
    hydrakon_perception_share_dir = get_package_share_directory('hydrakon_perception')

    custom_od_config_path = os.path.join(
        hydrakon_perception_share_dir, 'config', 'cone_detection.yaml'
    )
    onnx_path = os.path.join(hydrakon_perception_share_dir, 'models', 'best.onnx')

    cone_is_static = LaunchConfiguration('cone_is_static').perform(context) == 'true'

    # cone_detection.yaml ships with an empty 'custom_onnx_file' since the
    # absolute install path can't be known ahead of time. Fill it in here,
    # along with enabling object detection in CUSTOM_YOLOLIKE_BOX_OBJECTS mode,
    # via a small override file loaded last so it wins over the defaults.
    # 'is_static' is also overridden here: True gives smoother/more stable
    # positions for cones that never move (a real race track), but heavily
    # damps position updates, which reads as latency if you're hand-carrying
    # a cone around during testing. Toggle with 'cone_is_static:=false'.
    object_detection_params = {
        'od_enabled': True,
        'detection_model': 'CUSTOM_YOLOLIKE_BOX_OBJECTS',
        'custom_onnx_file': onnx_path,
    }
    for class_id in CONE_CLASS_IDS:
        object_detection_params[f'class_{class_id:03d}'] = {'is_static': cone_is_static}

    od_enable_override = {
        '/**': {
            'ros__parameters': {
                'object_detection': object_detection_params
            }
        }
    }
    override_file = tempfile.NamedTemporaryFile(
        mode='w', suffix='.yaml', prefix='zed_cone_od_override_', delete=False
    )
    yaml.safe_dump(od_enable_override, override_file)
    override_file.close()

    zed_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_launch_dir, 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'zedx',
            'custom_object_detection_config_path': custom_od_config_path,
            'ros_params_override_path': override_file.name,
        }.items()
    )

    return [zed_camera_launch]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'cone_is_static',
            default_value='true',
            description=(
                "Whether the ZED SDK should treat detected cones as static objects. "
                "'true' (default) gives stable tracking for a real, stationary track. "
                "'false' makes markers respond immediately to movement, useful when "
                "testing by hand-carrying a cone."
            ),
            choices=['true', 'false'],
        ),
        OpaqueFunction(function=launch_setup),
    ])
