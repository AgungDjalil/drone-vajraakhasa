from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ------ Argumen yang bisa kamu override saat launch ------
    cam_x  = LaunchConfiguration('cam_x')
    cam_y  = LaunchConfiguration('cam_y')
    cam_z  = LaunchConfiguration('cam_z')
    cam_qx = LaunchConfiguration('cam_qx')
    cam_qy = LaunchConfiguration('cam_qy')
    cam_qz = LaunchConfiguration('cam_qz')
    cam_qw = LaunchConfiguration('cam_qw')

    # Nama frame
    base_link_frame      = LaunchConfiguration('base_link_frame')
    camera_frame         = LaunchConfiguration('camera_frame')
    camera_optical_frame = LaunchConfiguration('camera_optical_frame')

    # Parameter untuk node kinematika (PX4 → TF odom->base_link)
    odom_frame     = LaunchConfiguration('odom_frame')
    veh_odom_topic = LaunchConfiguration('veh_odom_topic')

    return LaunchDescription([
        # ====== Defaults (ubah saat launch bila perlu) ======
        DeclareLaunchArgument('cam_x',  default_value='0.12'),
        DeclareLaunchArgument('cam_y',  default_value='0.0'),
        DeclareLaunchArgument('cam_z',  default_value='-0.05'),
        # contoh pitch +15° → qy ≈ 0.130526 ; kalau lurus pakai 0 0 0 1
        DeclareLaunchArgument('cam_qx', default_value='0.0'),
        DeclareLaunchArgument('cam_qy', default_value='0.130526'),
        DeclareLaunchArgument('cam_qz', default_value='0.0'),
        DeclareLaunchArgument('cam_qw', default_value='0.991445'),

        DeclareLaunchArgument('base_link_frame',      default_value='base_link'),
        DeclareLaunchArgument('camera_frame',         default_value='camera_link'),
        DeclareLaunchArgument('camera_optical_frame', default_value='camera_optical_frame'),

        DeclareLaunchArgument('odom_frame',     default_value='odom'),
        DeclareLaunchArgument('veh_odom_topic', default_value='/fmu/out/vehicle_odometry'),

        # ====== TF statik: base_link -> camera_frame ======
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_cam',
            arguments=[
                cam_x, cam_y, cam_z,
                cam_qx, cam_qy, cam_qz, cam_qw,
                base_link_frame, camera_frame
            ]
        ),

        # ====== TF statik: camera_frame -> camera_optical_frame (standar ROS) ======
        # rpy = (-90°, 0°, -90°) → quat ≈ [-0.5, -0.5, 0.5, 0.5]
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_cam_to_optical',
            arguments=[
                '0', '0', '0',
                '-0.5', '-0.5', '0.5', '0.5',
                camera_frame, camera_optical_frame
            ]
        ),

        # ====== Node kinematika: publish TF odom -> base_link dari PX4 VehicleOdometry ======
        Node(
            package='offboard_control',
            executable='drone_kinematics',
            name='drone_kinematics',
            parameters=[{
                'odom_frame':      odom_frame,
                'base_link_frame': base_link_frame,
                'veh_odom_topic':  veh_odom_topic,
            }],
            output='screen'
        ),
    ])
