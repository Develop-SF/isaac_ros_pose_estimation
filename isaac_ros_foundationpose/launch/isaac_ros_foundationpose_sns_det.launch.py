# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# Number of Realsense messages to be dropped in 1 second
HAWK_EXPECT_FREQ = 28
# Expected number of Realsense messages in 1 second
INPUT_IMAGES_DROP_FREQ = 30

YOLOV8_MODEL_INPUT_SIZE = 640  # RT-DETR models expect 640x640 encoded image size
YOLOV8_MODEL_NUM_CHANNELS = 3  # RT-DETR models expect 3 image channels

REALSENSE_IMAGE_WIDTH = 1280
REALSENSE_IMAGE_HEIGHT = 720

VISUALIZATION_DOWNSCALING_FACTOR = 10

REALSENSE_TO_YOLO_RATIO = REALSENSE_IMAGE_WIDTH / YOLOV8_MODEL_INPUT_SIZE

isaac_ros_assets_path = '/workspaces/isaac_ros-dev/isaac_ros_assets'

MESH_FILE_PATH = os.path.join(isaac_ros_assets_path, 'isaac_ros_foundationpose/grape_juice/AR-Code-Object-Capture-app-1718350160.obj')
TEXTURE_PATH = os.path.join(isaac_ros_assets_path, 'isaac_ros_foundationpose/grape_juice/baked_mesh_tex0.png')

REFINE_MODEL_PATH = os.path.join(isaac_ros_assets_path, 'models/foundationpose/refine_model.onnx')
REFINE_ENGINE_PATH = os.path.join(isaac_ros_assets_path, 'models/foundationpose/refine_trt_engine.plan')
SCORE_MODEL_PATH = os.path.join(isaac_ros_assets_path, 'models/foundationpose/score_model.onnx')
SCORE_ENGINE_PATH = os.path.join(isaac_ros_assets_path, 'models/foundationpose/score_trt_engine.plan')
YOLO_MODEL_PATH = os.path.join(isaac_ros_assets_path, 'models/yolov8/best_grape_juice.onnx')
YOLO_ENGINE_PATH = os.path.join(isaac_ros_assets_path, 'models/yolov8/best_grape_juice.plan')

def generate_launch_description():
    """Generate launch description for testing relevant nodes."""
    rviz_config_path = os.path.join(
        get_package_share_directory('isaac_ros_foundationpose'),
        'rviz', 'sns_foundationpose.rviz')
    
    yolov8_dir = get_package_share_directory('isaac_ros_yolov8')

    launch_args = [
        DeclareLaunchArgument(
            'hawk_expect_freq',
            default_value=str(HAWK_EXPECT_FREQ),
            description='Number of Realsense messages to be dropped in 1 second'),

        DeclareLaunchArgument(
            'input_images_drop_freq',
            default_value=str(INPUT_IMAGES_DROP_FREQ),
            description='Expected number of Realsense messages in 1 second'),

        DeclareLaunchArgument(
            'mesh_file_path',
            default_value=MESH_FILE_PATH,
            description='The absolute file path to the mesh file'),

        DeclareLaunchArgument(
            'texture_path',
            default_value=TEXTURE_PATH,
            description='The absolute file path to the texture map'),

        DeclareLaunchArgument(
            'refine_model_file_path',
            default_value=REFINE_MODEL_PATH,
            description='The absolute file path to the refine model'),

        DeclareLaunchArgument(
            'refine_engine_file_path',
            default_value=REFINE_ENGINE_PATH,
            description='The absolute file path to the refine trt engine'),

        DeclareLaunchArgument(
            'score_model_file_path',
            default_value=SCORE_MODEL_PATH,
            description='The absolute file path to the score model'),

        DeclareLaunchArgument(
            'score_engine_file_path',
            default_value=SCORE_ENGINE_PATH,
            description='The absolute file path to the score trt engine'),

        DeclareLaunchArgument(
            'yolov8_model_file_path',
            default_value=YOLO_MODEL_PATH,
            description='The absolute file path to the YOLOV8 ONNX file'),

        DeclareLaunchArgument(
            'yolov8_engine_file_path',
            default_value=YOLO_ENGINE_PATH,
            description='The absolute file path to the YOLOV8 TensorRT engine file'),

        DeclareLaunchArgument(
            'launch_rviz',
            default_value='False',
            description='Flag to enable Rviz2 launch'),

        DeclareLaunchArgument(
            'container_name',
            default_value='foundationpose_container',
            description='Name for ComposableNodeContainer'),

        DeclareLaunchArgument(
            'container_name',
            default_value='foundationpose_container',
            description='Name for ComposableNodeContainer'),

        DeclareLaunchArgument(
            'image_input_topic',
            default_value='/color/image_raw',
            description='The input topic for color images'),

        DeclareLaunchArgument(
            'camera_info_input_topic',
            default_value='/color/camera_info',
            description='The input topic for camera information'),

        DeclareLaunchArgument(
            'depth_input_topic',
            default_value='/camera/aligned_depth_to_color/image_raw',
            description='The input topic for aligned depth images')
    ]

    hawk_expect_freq = LaunchConfiguration('hawk_expect_freq')
    input_images_drop_freq = LaunchConfiguration('input_images_drop_freq')
    mesh_file_path = LaunchConfiguration('mesh_file_path')
    texture_path = LaunchConfiguration('texture_path')
    refine_model_file_path = LaunchConfiguration('refine_model_file_path')
    refine_engine_file_path = LaunchConfiguration('refine_engine_file_path')
    score_model_file_path = LaunchConfiguration('score_model_file_path')
    score_engine_file_path = LaunchConfiguration('score_engine_file_path')
    yolov8_model_file_path = LaunchConfiguration('yolov8_model_file_path')
    yolov8_engine_file_path = LaunchConfiguration('yolov8_engine_file_path')
    launch_rviz = LaunchConfiguration('launch_rviz')
    container_name = LaunchConfiguration('container_name')
    image_input_topic = LaunchConfiguration('image_input_topic')
    camera_info_input_topic = LaunchConfiguration('camera_info_input_topic')
    depth_input_topic = LaunchConfiguration('depth_input_topic')

    yolov8_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            yolov8_dir, 'launch', 'yolov8_tensor_rt.launch.py')]
        ),
        launch_arguments={
            'model_file_path': yolov8_model_file_path,
            'engine_file_path': yolov8_engine_file_path,
            'input_tensor_name': '["input_tensor"]',
            'input_binding_names': '["images"]',
            'output_tensor_names': '["output_tensor"]',
            'output_binding_names': '["output0"]',
            'verbose': 'False',
            'force_engine_update': 'False',
            'confidence_threshold': '0.25',
            'nms_threshold': '0.45',
            'input_image_width': str(REALSENSE_IMAGE_WIDTH),
            'input_image_height': str(REALSENSE_IMAGE_HEIGHT),
            'network_image_width': str(YOLOV8_MODEL_INPUT_SIZE),
            'network_image_height': str(YOLOV8_MODEL_INPUT_SIZE),
            'image_mean': '[0.0, 0.0, 0.0]',
            'image_stddev': '[1.0, 1.0, 1.0]',
            'image_input_topic': image_input_topic,
            'camera_info_input_topic': camera_info_input_topic
        }.items(),
    )

    # Drops hawk_expect_freq out of input_images_drop_freq RealSense messages
    drop_node = ComposableNode(
        name='drop_node',
        package='isaac_ros_nitros_topic_tools',
        plugin='nvidia::isaac_ros::nitros::NitrosCameraDropNode',
        parameters=[{
            'X': hawk_expect_freq,
            'Y': input_images_drop_freq,
            'mode': 'mono+depth',
            'depth_format_string': 'nitros_image_mono16',
            'input_qos': 'SENSOR_DATA'
        }],
        remappings=[
            ('image_1', image_input_topic),
            ('camera_info_1', camera_info_input_topic),
            ('depth_1', depth_input_topic),
            ('image_1_drop', 'rgb/image_rect_color'),
            ('camera_info_1_drop', 'rgb/camera_info'),
            ('depth_1_drop', 'depth_uint16')
        ]
    )

    # Realsense depth is in uint16 and millimeters. Convert to float32 and meters
    convert_metric_node = ComposableNode(
        package='isaac_ros_depth_image_proc',
        plugin='nvidia::isaac_ros::depth_image_proc::ConvertMetricNode',
        remappings=[
            ('image_raw', 'depth_uint16'),
            ('image', 'depth_image')
        ]
    )

    # Create a binary segmentation mask from a Detection2DArray published by YOLOV8.
    # The segmentation mask is of size int(REALSENSE_IMAGE_WIDTH/REALSENSE_TO_YOLO_RATIO) x
    # int(REALSENSE_IMAGE_HEIGHT/REALSENSE_TO_YOLO_RATIO)
    detection2_d_to_mask_node = ComposableNode(
        name='detection2_d_to_mask',
        package='isaac_ros_foundationpose',
        plugin='nvidia::isaac_ros::foundationpose::Detection2DToMask',
        parameters=[{
            'mask_width': REALSENSE_IMAGE_WIDTH,
            'mask_height': REALSENSE_IMAGE_HEIGHT,
            'target_class_id': 0
        }],
        remappings=[('detection2_d_array', 'detections_output'),
                    ('segmentation', 'yolo_segmentation')])

    # Resize segmentation mask to ESS model image size so it can be used by FoundationPose
    # FoundationPose requires depth, rgb image and segmentation mask to be of the same size
    # Resize from int(REALSENSE_IMAGE_WIDTH/REALSENSE_TO_YOLO_RATIO) x
    # int(REALSENSE_IMAGE_HEIGHT/REALSENSE_TO_YOLO_RATIO) to
    # ESS_MODEL_IMAGE_WIDTH x ESS_MODEL_IMAGE_HEIGHT
    # output height constraint is used since keep_aspect_ratio is False
    # and the image is padded
    resize_mask_node = ComposableNode(
        name='resize_mask_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        parameters=[{
            'input_width': int(REALSENSE_IMAGE_WIDTH/REALSENSE_TO_YOLO_RATIO),
            'input_height': int(REALSENSE_IMAGE_HEIGHT/REALSENSE_TO_YOLO_RATIO),
            'output_width': REALSENSE_IMAGE_WIDTH,
            'output_height': REALSENSE_IMAGE_HEIGHT,
            'keep_aspect_ratio': True,
            'disable_padding': False
        }],
        remappings=[
            ('image', 'yolo_segmentation'),
            ('camera_info', '/yolov8_encoder/resize/camera_info'),
            ('resize/image', 'segmentation'),
            ('resize/camera_info', 'camera_info_segmentation')
        ]
    )

    resize_left_viz = ComposableNode(
        name='resize_left_viz',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ResizeNode',
        parameters=[{
            'input_width': REALSENSE_IMAGE_WIDTH,
            'input_height': REALSENSE_IMAGE_HEIGHT,
            'output_width': int(REALSENSE_IMAGE_WIDTH/VISUALIZATION_DOWNSCALING_FACTOR),
            'output_height': int(REALSENSE_IMAGE_HEIGHT/VISUALIZATION_DOWNSCALING_FACTOR),
            'keep_aspect_ratio': False,
            'encoding_desired': 'rgb8',
            'disable_padding': False
        }],
        remappings=[
            ('image', 'rgb/image_rect_color'),
            ('camera_info', 'rgb/camera_info'),
            ('resize/image', 'rgb/image_rect_color_viz'),
            ('resize/camera_info', 'rgb/camera_info_viz')
        ]
    )

    foundationpose_node = ComposableNode(
        name='foundationpose_node',
        package='isaac_ros_foundationpose',
        plugin='nvidia::isaac_ros::foundationpose::FoundationPoseNode',
        parameters=[{
            'mesh_file_path': mesh_file_path,
            'texture_path': texture_path,

            'refine_model_file_path': refine_model_file_path,
            'refine_engine_file_path': refine_engine_file_path,
            'refine_input_tensor_names': ['input_tensor1', 'input_tensor2'],
            'refine_input_binding_names': ['input1', 'input2'],
            'refine_output_tensor_names': ['output_tensor1', 'output_tensor2'],
            'refine_output_binding_names': ['output1', 'output2'],

            'score_model_file_path': score_model_file_path,
            'score_engine_file_path': score_engine_file_path,
            'score_input_tensor_names': ['input_tensor1', 'input_tensor2'],
            'score_input_binding_names': ['input1', 'input2'],
            'score_output_tensor_names': ['output_tensor'],
            'score_output_binding_names': ['output1'],
        }],
        remappings=[
            ('pose_estimation/depth_image', 'depth_image'),
            ('pose_estimation/image', 'rgb/image_rect_color'),
            ('pose_estimation/camera_info', 'rgb/camera_info'),
            ('pose_estimation/segmentation', 'segmentation'),
            ('pose_estimation/output', 'output')])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(launch_rviz))

    nodes = [
        drop_node,
        convert_metric_node,
        detection2_d_to_mask_node,
        resize_mask_node,
        foundationpose_node,
        resize_left_viz
        # realsense_node
    ]

    foundationpose_container = ComposableNodeContainer(
        name=container_name,
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=nodes,
        output='screen'
    )

    return launch.LaunchDescription(launch_args + [foundationpose_container, rviz_node, yolov8_launch])
