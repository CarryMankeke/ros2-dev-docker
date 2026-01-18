#!/usr/bin/env python3
# Autor: Camilo Soto Villegas
# Contacto: camilo.soto.v@usach.cl
# Proyecto: clean_v2

from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class RvizVisualDescriptions(Node):
    def __init__(self) -> None:
        super().__init__('rviz_visual_descriptions')

        self.declare_parameter('prefix', 'mm1_')
        self.declare_parameter('namespace', 'mm1')
        self.declare_parameter('robot_description_base_visual', '')
        self.declare_parameter('robot_description_arm_visual', '')
        self.declare_parameter('arm_x', 0.0)
        self.declare_parameter('arm_y', 0.0)
        self.declare_parameter('arm_z', 0.02)
        self.declare_parameter('arm_roll', 0.0)
        self.declare_parameter('arm_pitch', 0.0)
        self.declare_parameter('arm_yaw', 0.0)
        self.declare_parameter('enable_lidar', True)
        self.declare_parameter('lidar_x', 0.20)
        self.declare_parameter('lidar_y', 0.0)
        self.declare_parameter('lidar_z', 0.125)
        self.declare_parameter('enable_ee_imu', True)
        self.declare_parameter('ee_imu_x', 0.03)
        self.declare_parameter('ee_imu_y', 0.0)
        self.declare_parameter('ee_imu_z', 0.02)
        self.declare_parameter('ee_imu_roll', 0.0)
        self.declare_parameter('ee_imu_pitch', 0.0)
        self.declare_parameter('ee_imu_yaw', 0.0)

        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.base_pub = self.create_publisher(
            String,
            '~/robot_description_base_visual',
            qos,
        )
        self.arm_pub = self.create_publisher(
            String,
            '~/robot_description_arm_visual',
            qos,
        )

        self.base_urdf = self._render_xacro('base_only')
        self.arm_urdf = self._render_xacro('arm_only')

        self._publish_descriptions()
        self.create_timer(5.0, self._publish_descriptions)

    def _render_xacro(self, visual_mode: str) -> str:
        prefix = self.get_parameter('prefix').value
        namespace = self.get_parameter('namespace').value
        arm_x = self.get_parameter('arm_x').value
        arm_y = self.get_parameter('arm_y').value
        arm_z = self.get_parameter('arm_z').value
        arm_roll = self.get_parameter('arm_roll').value
        arm_pitch = self.get_parameter('arm_pitch').value
        arm_yaw = self.get_parameter('arm_yaw').value
        enable_lidar = self.get_parameter('enable_lidar').value
        lidar_x = self.get_parameter('lidar_x').value
        lidar_y = self.get_parameter('lidar_y').value
        lidar_z = self.get_parameter('lidar_z').value
        enable_ee_imu = self.get_parameter('enable_ee_imu').value
        ee_imu_x = self.get_parameter('ee_imu_x').value
        ee_imu_y = self.get_parameter('ee_imu_y').value
        ee_imu_z = self.get_parameter('ee_imu_z').value
        ee_imu_roll = self.get_parameter('ee_imu_roll').value
        ee_imu_pitch = self.get_parameter('ee_imu_pitch').value
        ee_imu_yaw = self.get_parameter('ee_imu_yaw').value

        xacro_path = Path(
            get_package_share_directory('mm_robot_description')
        ) / 'urdf' / 'mm_robot.urdf.xacro'

        cmd = [
            'xacro',
            str(xacro_path),
            f'prefix:={prefix}',
            f'namespace:={namespace}',
            f'arm_x:={arm_x}',
            f'arm_y:={arm_y}',
            f'arm_z:={arm_z}',
            f'arm_roll:={arm_roll}',
            f'arm_pitch:={arm_pitch}',
            f'arm_yaw:={arm_yaw}',
            f'enable_lidar:={str(enable_lidar).lower()}',
            f'lidar_x:={lidar_x}',
            f'lidar_y:={lidar_y}',
            f'lidar_z:={lidar_z}',
            f'enable_ee_imu:={str(enable_ee_imu).lower()}',
            f'ee_imu_x:={ee_imu_x}',
            f'ee_imu_y:={ee_imu_y}',
            f'ee_imu_z:={ee_imu_z}',
            f'ee_imu_roll:={ee_imu_roll}',
            f'ee_imu_pitch:={ee_imu_pitch}',
            f'ee_imu_yaw:={ee_imu_yaw}',
            f'visual_mode:={visual_mode}',
            'enable_ros2_control:=false',
        ]

        result = subprocess.run(
            cmd,
            check=False,
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            self.get_logger().error(
                'xacro failed for visual_mode=%s: %s',
                visual_mode,
                result.stderr.strip(),
            )
            return ''
        return result.stdout

    def _publish_descriptions(self) -> None:
        if self.base_urdf:
            self.set_parameters([
                Parameter('robot_description_base_visual', Parameter.Type.STRING, self.base_urdf),
            ])
            msg = String()
            msg.data = self.base_urdf
            self.base_pub.publish(msg)

        if self.arm_urdf:
            self.set_parameters([
                Parameter('robot_description_arm_visual', Parameter.Type.STRING, self.arm_urdf),
            ])
            msg = String()
            msg.data = self.arm_urdf
            self.arm_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = RvizVisualDescriptions()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
