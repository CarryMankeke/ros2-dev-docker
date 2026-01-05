#!/usr/bin/env python3
# Autor: Camilo Soto Villegas
# Contacto: camilo.soto.v@usach.cl
# Proyecto: clean_v2

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class CamerasChecker(Node):
    def __init__(self):
        super().__init__('smoke_cameras')
        self.declare_parameter('namespace', 'mm1')
        self.declare_parameter('prefix', 'mm1_')
        self.declare_parameter('timeout_sec', 5.0)

        self._images = {}
        self._infos = {}

        namespace = self.get_parameter('namespace').value.strip('/')
        self._ns_prefix = f'/{namespace}' if namespace else ''
        self._prefix = self.get_parameter('prefix').value

        self._cameras = ['front', 'left', 'right', 'rear', 'ee']
        for name in self._cameras:
            image_topic = f'{self._ns_prefix}/camera/{name}/image'
            info_topic = f'{self._ns_prefix}/camera/{name}/camera_info'

            self.create_subscription(
                Image,
                image_topic,
                lambda msg, cam=name: self._on_image(cam, msg),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                CameraInfo,
                info_topic,
                lambda msg, cam=name: self._on_info(cam, msg),
                qos_profile_sensor_data,
            )

    def _on_image(self, cam, msg):
        if cam not in self._images:
            self._images[cam] = msg

    def _on_info(self, cam, msg):
        if cam not in self._infos:
            self._infos[cam] = msg

    def _expected_frame_id(self, cam):
        if cam == 'ee':
            return f'{self._prefix}ee_cam_link'
        return f'{self._prefix}cam_{cam}_link'

    def _check_qos(self, topic):
        infos = self.get_publishers_info_by_topic(topic)
        if not infos:
            return False
        for info in infos:
            if info.qos_profile.reliability == QoSReliabilityPolicy.BEST_EFFORT:
                return True
        return False

    def run_check(self):
        timeout = float(self.get_parameter('timeout_sec').value)
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout:
            if len(self._images) == len(self._cameras) and len(self._infos) == len(self._cameras):
                break
            rclpy.spin_once(self, timeout_sec=0.1)

        missing = [cam for cam in self._cameras if cam not in self._images or cam not in self._infos]
        if missing:
            print(f'FAIL: camaras sin mensajes: {sorted(missing)}')
            return 1

        bad_frames = []
        for cam in self._cameras:
            expected = self._expected_frame_id(cam)
            if self._images[cam].header.frame_id != expected:
                bad_frames.append((cam, 'image', self._images[cam].header.frame_id, expected))
            if self._infos[cam].header.frame_id != expected:
                bad_frames.append((cam, 'camera_info', self._infos[cam].header.frame_id, expected))

        if bad_frames:
            print('FAIL: frame_id incorrecto en camaras')
            for cam, msg_type, current, expected in bad_frames:
                print(f'- {cam} {msg_type}: {current} != {expected}')
            return 1

        qos_fail = []
        for cam in self._cameras:
            image_topic = f'{self._ns_prefix}/camera/{cam}/image'
            info_topic = f'{self._ns_prefix}/camera/{cam}/camera_info'
            if not self._check_qos(image_topic):
                qos_fail.append(image_topic)
            if not self._check_qos(info_topic):
                qos_fail.append(info_topic)

        if qos_fail:
            print(f'FAIL: QoS no es best_effort en: {sorted(set(qos_fail))}')
            return 1

        print('PASS: camaras publican con frame_id y QoS esperado')
        return 0


def main():
    rclpy.init()
    node = CamerasChecker()
    try:
        code = node.run_check()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
