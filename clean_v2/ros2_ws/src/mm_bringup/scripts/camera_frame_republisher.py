#!/usr/bin/env python3

import functools

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class CameraFrameRepublisher(Node):
    """Re-publica Image/CameraInfo con frame_id alineado al link de la camara.

    Uso minimo:
      ros2 run mm_bringup camera_frame_republisher.py --ros-args -p prefix:=mm1_
    """

    def __init__(self):
        super().__init__('camera_frame_republisher')
        self.declare_parameter('prefix', '')
        prefix = self.get_parameter('prefix').get_parameter_value().string_value
        namespace = self.get_namespace().strip('/')
        ns_prefix = f'/{namespace}' if namespace else ''

        self._image_pubs = {}
        self._info_pubs = {}
        for name in ('front', 'left', 'right', 'rear', 'ee'):
            if name == 'ee':
                frame_id = f'{prefix}ee_cam_link'
            else:
                frame_id = f'{prefix}cam_{name}_link'

            src_image = f'{ns_prefix}/camera/{name}/image_raw'
            src_info = f'{ns_prefix}/camera/{name}/camera_info_raw'
            dst_image = f'{ns_prefix}/camera/{name}/image'
            dst_info = f'{ns_prefix}/camera/{name}/camera_info'

            self._image_pubs[name] = self.create_publisher(Image, dst_image, qos_profile_sensor_data)
            self._info_pubs[name] = self.create_publisher(CameraInfo, dst_info, qos_profile_sensor_data)

            self.create_subscription(
                Image,
                src_image,
                functools.partial(self._republish_image, name=name, frame_id=frame_id),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                CameraInfo,
                src_info,
                functools.partial(self._republish_info, name=name, frame_id=frame_id),
                qos_profile_sensor_data,
            )

        self.get_logger().info('Camera frame republisher activo.')

    def _republish_image(self, msg, name, frame_id):
        msg.header.frame_id = frame_id
        self._image_pubs[name].publish(msg)

    def _republish_info(self, msg, name, frame_id):
        msg.header.frame_id = frame_id
        self._info_pubs[name].publish(msg)


def main():
    rclpy.init()
    node = CameraFrameRepublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
