# Copyright 2026 nevlife
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

share_dir = get_package_share_directory('realsense2_camera')

# Streams and filters that every D435i of this launch uses.
FIXED = {
    'enable_sync': 'true',
    'enable_rgbd': 'false',
    'spatial_filter.enable': 'false',
    'temporal_filter.enable': 'false',
    'hole_filling_filter.enable': 'false',
}


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    rs_args = dict(FIXED)
    rs_args.update({
        'camera_name': arg('camera_name'),
        'camera_namespace': arg('camera_namespace'),
        'serial_no': arg('serial_no'),
        'enable_color': arg('color'),
        'enable_depth': arg('depth'),
        'enable_infra1': arg('infra'),
        'enable_infra2': arg('infra'),
        'pointcloud.enable': arg('pointcloud'),
        'align_depth.enable': arg('align_depth'),
        'enable_gyro': arg('imu'),
        'enable_accel': arg('imu'),
        'unite_imu_method': '2' if arg('imu') == 'true' else '0',
    })
    # rs_launch.py warns about every launch configuration it does not know, so it sees only its own.
    return [
        GroupAction(
            [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share_dir, 'launch', 'rs_launch.py')),
                launch_arguments=rs_args.items())],
            forwarding=False)
    ]


def generate_launch_description():
    def declare(name, default, description):
        return DeclareLaunchArgument(name, default_value=default, description=description)

    return LaunchDescription(
        [
            declare('camera_name', 'd435i', 'The node name.'),
            declare('camera_namespace', 'camera', 'The node namespace.'),
            declare('serial_no', "''", "The serial number to open, as '_<serial>'. Empty opens the first camera found."),
            declare('color', 'true', 'Enable the color stream.'),
            declare('depth', 'true', 'Enable the depth stream.'),
            declare('infra', 'false', 'Enable the two infrared streams.'),
            declare('pointcloud', 'true', 'Publish the point cloud.'),
            declare('align_depth', 'true', 'Publish the depth aligned to the color image.'),
            declare('imu', 'true', 'Enable the gyro and the accelerometer, united on one IMU topic.'),
            OpaqueFunction(function=launch_setup),
        ]
    )
