import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 模型路径（相对于 install 后的 share 目录）
    pkg_share = get_package_share_directory("vision_detector")
    default_weapon_model = os.path.join(pkg_share, "model", "weapon_pickup.onnx")
    default_pole_model = os.path.join(pkg_share, "model", "pole_detect.onnx")

    # ==================== 武器_长杆节点参数 ====================
    weapon_model_arg = DeclareLaunchArgument(
        "weapon_model", default_value=default_weapon_model,
        description="武器检测模型路径")
    pole_model_arg = DeclareLaunchArgument(
        "pole_model", default_value=default_pole_model,
        description="杆子检测模型路径")
    weapon_camera_arg = DeclareLaunchArgument(
            "weapon_camera", default_value="0",
            description="武器摄像头索引")
    pole_camera_arg = DeclareLaunchArgument(
            "pole_camera", default_value="2",
            description="武器摄像头索引")
    enable_weapon_arg = DeclareLaunchArgument(
        "enable_weapon_pole", default_value="true",
        description="是否启动武器检测节点")
    weapon_conf_arg = DeclareLaunchArgument(
        "weapon_pole_conf", default_value="0.25",
        description="武器检测置信度阈值")
    weapon_show_arg = DeclareLaunchArgument(
        "weapon_pole_show_window", default_value="true",
        description="武器节点显示窗口")


    # # ==================== 武器节点参数 ====================
    # weapon_model_arg = DeclareLaunchArgument(
    #     "weapon_model", default_value=default_weapon_model,
    #     description="武器检测模型路径")
    # weapon_camera_arg = DeclareLaunchArgument(
    #     "weapon_camera", default_value="2",
    #     description="武器摄像头索引")
    # enable_weapon_arg = DeclareLaunchArgument(
    #     "enable_weapon", default_value="true",
    #     description="是否启动武器检测节点")
    # weapon_conf_arg = DeclareLaunchArgument(
    #     "weapon_conf", default_value="0.25",
    #     description="武器检测置信度阈值")
    # weapon_serial_arg = DeclareLaunchArgument(
    #     "weapon_serial_port", default_value="/dev/ttyUSB0",
    #     description="武器串口路径")
    # weapon_show_arg = DeclareLaunchArgument(
    #     "weapon_show_window", default_value="true",
    #     description="武器节点显示窗口")
    # weapon_use_serial_arg = DeclareLaunchArgument(
    #     "weapon_use_serial", default_value="true",
    #     description="武器节点是否启用串口")

    # # ==================== 杆子节点参数 ====================
    # pole_model_arg = DeclareLaunchArgument(
    #     "pole_model", default_value=default_pole_model,
    #     description="杆子检测模型路径")
    # pole_camera_arg = DeclareLaunchArgument(
    #     "pole_camera", default_value="0",
    #     description="杆子摄像头索引")
    # enable_pole_arg = DeclareLaunchArgument(
    #     "enable_pole", default_value="false",
    #     description="是否启动杆子检测节点")
    # pole_conf_arg = DeclareLaunchArgument(
    #     "pole_conf", default_value="0.25",
    #     description="杆子检测置信度阈值")
    # pole_serial_arg = DeclareLaunchArgument(
    #     "pole_serial_port", default_value="/dev/ttyUSB1",
    #     description="杆子串口路径")
    # pole_show_arg = DeclareLaunchArgument(
    #     "pole_show_window", default_value="false",
    #     description="杆子节点显示窗口")
    # pole_use_serial_arg = DeclareLaunchArgument(
    #     "pole_use_serial", default_value="true",
    #     description="杆子节点是否启用串口")

    # ==================== 节点 ====================
    weapon_pole = Node(
            package="vision_detector",
            executable="weapon_pole",
            name="weapon_pole",
            parameters=[{
                "weapon_model_path": LaunchConfiguration("weapon_model"),
                "pole_model_path": LaunchConfiguration("pole_model"),
                "weapon_camera_index": LaunchConfiguration("weapon_camera"),
                "pole_camera_index": LaunchConfiguration("pole_camera"),
                "conf_thres": LaunchConfiguration("weapon_pole_conf"),
                "show_window": LaunchConfiguration("weapon_pole_show_window"),
            }],
            output="screen",
            emulate_tty=True,
            condition=IfCondition(LaunchConfiguration("enable_weapon_pole")),
        )

    # weapon_node = Node(
    #     package="vision_detector",
    #     executable="weapon_node",
    #     name="weapon_node",
    #     parameters=[{
    #         "model_path": LaunchConfiguration("weapon_model"),
    #         "camera_index": LaunchConfiguration("weapon_camera"),
    #         "conf_thres": LaunchConfiguration("weapon_conf"),
    #         "serial_port": LaunchConfiguration("weapon_serial_port"),
    #         "show_window": LaunchConfiguration("weapon_show_window"),
    #         "use_serial": LaunchConfiguration("weapon_use_serial"),
    #     }],
    #     output="screen",
    #     emulate_tty=True,
    #     condition=IfCondition(LaunchConfiguration("enable_weapon")),
    # )

    # pole_node = Node(
    #     package="vision_detector",
    #     executable="pole_node",
    #     name="pole_node",
    #     parameters=[{
    #         "model_path": LaunchConfiguration("pole_model"),
    #         "camera_index": LaunchConfiguration("pole_camera"),
    #         "conf_thres": LaunchConfiguration("pole_conf"),
    #         "serial_port": LaunchConfiguration("pole_serial_port"),
    #         "show_window": LaunchConfiguration("pole_show_window"),
    #         "use_serial": LaunchConfiguration("pole_use_serial"),
    #     }],
    #     output="screen",
    #     emulate_tty=True,
    #     condition=IfCondition(LaunchConfiguration("enable_pole")),
    # )

    return LaunchDescription([
        # 武器参数
        weapon_model_arg,
        pole_model_arg,
        weapon_camera_arg,
        pole_camera_arg,
        enable_weapon_arg,
        weapon_conf_arg,
        weapon_show_arg,
        # 武器参数
        weapon_model_arg,
        # weapon_camera_arg,
        enable_weapon_arg,
        weapon_conf_arg,
        # weapon_serial_arg,
        weapon_show_arg,
        # weapon_use_serial_arg,
        # 杆子参数
        pole_model_arg,
        # pole_camera_arg,
        # enable_pole_arg,
        # pole_conf_arg,
        # pole_serial_arg,
        # pole_show_arg,
        # pole_use_serial_arg,
        # 节点
        # weapon_node,
        # pole_node,
        weapon_pole
    ])
