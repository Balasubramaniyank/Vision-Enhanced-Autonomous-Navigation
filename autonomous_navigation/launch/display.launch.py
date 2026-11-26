from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess ,TimerAction,SetEnvironmentVariable
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
import xacro


def generate_launch_description():

   
    urdf_file = os.path.join(
        get_package_share_directory('agv_description'),
        'urdf',
        'tortoisebot.urdf'
    )       
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("agv_description"), "urdf", "agv.urdf.xacro"]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

  
    robot_path = '/home/arz-1028/Desktop/Balasubramaniyan/ros2_ws/src'
    world_pkg = get_package_share_directory('aws_robomaker_hospital_world')
    world_paths = os.path.join(world_pkg, 'models')

    world_path = os.path.join(
        os.getenv('HOME'),
        'Desktop',
        'Balasubramaniyan',
        'ros2_ws',
        'src',
        'aws_robomaker_hospital_world',
        'worlds',
        'myworld.world'
    )

    gazebo_launch = ExecuteProcess(
            cmd=['gz', 'sim','-r',world_path],
            output='screen'
        )

    gz_resource_path = f"{robot_path}:{world_paths}"


    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path
    )
    
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        arguments=[urdf_file],
        parameters=[{'use_sim_time':True}],
        output="screen",    
    )
      
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'agv', '-x', '-2', '-y', '0', '-z', '0.4'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    rviz2= Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]

    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Maps ROS 2 /cmd_vel (Twist) to Gazebo's internal topic (Twist)
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            # '/rgbd_camera_1/image@sensor_msgs/msg/Image@gz.msgs.Image',
            # '/rgbd_camera_1/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            # '/rgbd_camera_1/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            # '/rgbd_camera_2/image@sensor_msgs/msg/Image@gz.msgs.Image',
            # '/rgbd_camera_2/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
             '/depth_camera_1/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            # '/depth_camera_1/image@sensor_msgs/msg/Image@gz.msgs.Image',

           

        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )


    return LaunchDescription([
            
   
    # robot_state_publisher_node,
      
    gazebo_resource_path,
    gazebo_launch,
    node_robot_state_publisher,
    spawn_entity,
    bridge,
    rviz2,


    ]
    )