#!/usr/bin/env python3
import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _get_list_param(node: Node, name: str, default: List[int]) -> List[int]:
    node.declare_parameter(name, default)
    value = node.get_parameter(name).value
    return [int(v) for v in value]


def _get_int_param(node: Node, name: str, default: int) -> int:
    node.declare_parameter(name, default)
    return int(node.get_parameter(name).value)


def _get_float_param(node: Node, name: str, default: float) -> float:
    node.declare_parameter(name, default)
    return float(node.get_parameter(name).value)


def _get_str_param(node: Node, name: str, default: str) -> str:
    node.declare_parameter(name, default)
    return str(node.get_parameter(name).value)


class JoyTeleop(Node):
    def __init__(self) -> None:
        super().__init__('joy_teleop')

        self._mode = _get_str_param(self, 'mode', 'base').lower()
        self._mode_cycle = ['base', 'arm', 'hybrid']

        self._joy_topic = _get_str_param(self, 'joy_topic', '/joy')
        self._cmd_vel_topic = _get_str_param(
            self, 'cmd_vel_topic', '/mm_base/mecanum_drive_controller/cmd_vel_unstamped'
        )
        self._cmd_vel_alt_topic = _get_str_param(self, 'cmd_vel_alt_topic', '/mm_base/cmd_vel_unstamped')
        self._arm_twist_topic = _get_str_param(
            self, 'arm_twist_topic', '/mm_arm/servo_node/delta_twist_cmds'
        )
        self._arm_traj_topic = _get_str_param(
            self, 'arm_traj_topic', '/mm_arm/arm_trajectory_controller/joint_trajectory'
        )
        self._gripper_topic = _get_str_param(self, 'gripper_topic', '/mm_arm/gripper_command')
        self._arm_frame = _get_str_param(self, 'arm_frame', 'mm_arm_tool0')
        self._arm_control_mode = _get_str_param(self, 'arm_control_mode', 'trajectory').lower()
        self._arm_joint_step = _get_float_param(self, 'arm_joint_step', 0.08)

        self._publish_rate = _get_float_param(self, 'publish_rate', 30.0)
        self._joy_timeout = _get_float_param(self, 'joy_timeout', 0.5)

        self._scale_linear = _get_float_param(self, 'scale_linear', 0.6)
        self._scale_angular = _get_float_param(self, 'scale_angular', 1.2)
        self._arm_linear_scale = _get_float_param(self, 'arm_linear_scale', 0.08)
        self._arm_vertical_scale = _get_float_param(self, 'arm_vertical_scale', 0.06)
        self._home_duration = _get_float_param(self, 'home_duration', 2.0)

        self._axis_linear_x = _get_int_param(self, 'axis_linear_x', 1)
        self._axis_linear_y = _get_int_param(self, 'axis_linear_y', 0)
        self._axis_angular_z = _get_int_param(self, 'axis_angular_z', -1)
        self._invert_linear_x = bool(_get_int_param(self, 'invert_linear_x', 1))
        self._invert_linear_y = bool(_get_int_param(self, 'invert_linear_y', 0))

        self._dpad_axis_x = _get_int_param(self, 'dpad_axis_x', -1)
        self._dpad_axis_y = _get_int_param(self, 'dpad_axis_y', -1)
        self._dpad_axis_deadband = _get_float_param(self, 'dpad_axis_deadband', 0.5)
        self._mode_axis = _get_int_param(self, 'mode_axis', -1)

        self._button_rot_left = _get_int_param(self, 'button_rot_left', 2)
        self._button_rot_right = _get_int_param(self, 'button_rot_right', 3)
        self._arm_up_button = _get_int_param(self, 'arm_up_button', 2)
        self._arm_down_button = _get_int_param(self, 'arm_down_button', 3)

        self._dpad_up = _get_int_param(self, 'dpad_up', 9)
        self._dpad_down = _get_int_param(self, 'dpad_down', 10)
        self._dpad_left = _get_int_param(self, 'dpad_left', 11)
        self._dpad_right = _get_int_param(self, 'dpad_right', 12)

        self._deadman_buttons = _get_list_param(self, 'deadman_buttons', [4, 5])
        self._home_button = _get_int_param(self, 'home_button', 6)
        self._start_button = _get_int_param(self, 'start_button', 7)
        self._select_button = _get_int_param(self, 'select_button', 8)
        self._gripper_open_button = _get_int_param(self, 'gripper_open_button', 0)
        self._gripper_close_button = _get_int_param(self, 'gripper_close_button', 1)

        self.declare_parameter(
            'arm_joint_names',
            [
                'mm_arm_shoulder_pan_joint',
                'mm_arm_shoulder_lift_joint',
                'mm_arm_elbow_joint',
                'mm_arm_wrist_1_joint',
                'mm_arm_wrist_2_joint',
                'mm_arm_wrist_3_joint',
            ],
        )
        self._arm_joint_names = list(self.get_parameter('arm_joint_names').value)

        self._last_buttons: List[int] = []
        self._last_joy: Optional[Joy] = None
        self._last_joy_time: Optional[Time] = None

        self._home_positions = {}
        self._arm_positions = {}
        self._home_ready = False

        self._cmd_vel_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._cmd_vel_alt_pub = None
        if self._cmd_vel_alt_topic and self._cmd_vel_alt_topic != self._cmd_vel_topic:
            self._cmd_vel_alt_pub = self.create_publisher(Twist, self._cmd_vel_alt_topic, 10)

        self._arm_twist_pub = self.create_publisher(TwistStamped, self._arm_twist_topic, 10)
        self._arm_traj_pub = self.create_publisher(JointTrajectory, self._arm_traj_topic, 10)
        self._gripper_pub = self.create_publisher(String, self._gripper_topic, 10)

        self.create_subscription(Joy, self._joy_topic, self._on_joy, 10)
        self.create_subscription(JointState, '/mm_arm/joint_states', self._on_arm_joint_state, 10)

        self._last_base_active = False
        self._last_arm_active = False

        period = 1.0 / max(self._publish_rate, 1.0)
        self.create_timer(period, self._on_timer)
        self.get_logger().info(f'Joy teleop started in mode={self._mode}')

    def _on_arm_joint_state(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return
        name_to_pos = dict(zip(msg.name, msg.position))
        if all(name in name_to_pos for name in self._arm_joint_names):
            self._arm_positions = {name: name_to_pos[name] for name in self._arm_joint_names}
            if not self._home_ready:
                self._home_positions = dict(self._arm_positions)
                self._home_ready = True
                self.get_logger().info('Captured arm home pose from joint_states.')

    def _on_joy(self, msg: Joy) -> None:
        self._last_joy = msg
        self._last_joy_time = self.get_clock().now()

        axes = list(msg.axes)
        use_mode_axis = 0 <= self._mode_axis < len(axes)
        if use_mode_axis:
            mode_val = axes[self._mode_axis]
            mode_idx = int(round(mode_val))
            mode_idx = max(0, min(2, mode_idx))
            self._mode = self._mode_cycle[mode_idx]

        buttons = list(msg.buttons)
        if not self._last_buttons:
            self._last_buttons = [0] * len(buttons)

        if not use_mode_axis:
            if self._edge_pressed(buttons, self._start_button):
                self._cycle_mode(1)
            if self._edge_pressed(buttons, self._select_button):
                self._cycle_mode(-1)

        if self._edge_pressed(buttons, self._gripper_open_button):
            self._publish_gripper('open')
        if self._edge_pressed(buttons, self._gripper_close_button):
            self._publish_gripper('close')

        if self._edge_pressed(buttons, self._home_button):
            self._publish_home()

        self._last_buttons = buttons

    def _edge_pressed(self, buttons: List[int], index: int) -> bool:
        if index < 0 or index >= len(buttons):
            return False
        prev = self._last_buttons[index] if index < len(self._last_buttons) else 0
        return buttons[index] == 1 and prev == 0

    def _cycle_mode(self, direction: int) -> None:
        if self._mode not in self._mode_cycle:
            self._mode = self._mode_cycle[0]
        idx = self._mode_cycle.index(self._mode)
        self._mode = self._mode_cycle[(idx + direction) % len(self._mode_cycle)]
        self.get_logger().info(f'Mode changed to {self._mode}')

    def _button(self, buttons: List[int], index: int) -> int:
        if index < 0 or index >= len(buttons):
            return 0
        return int(buttons[index])

    def _axis(self, axes: List[float], index: int) -> float:
        if index < 0 or index >= len(axes):
            return 0.0
        return float(axes[index])

    def _axis_to_dir(self, axes: List[float], index: int) -> int:
        if index < 0 or index >= len(axes):
            return 0
        value = float(axes[index])
        if value > self._dpad_axis_deadband:
            return 1
        if value < -self._dpad_axis_deadband:
            return -1
        return 0

    def _deadman_pressed(self, buttons: List[int]) -> bool:
        return any(self._button(buttons, idx) == 1 for idx in self._deadman_buttons)

    def _publish_gripper(self, command: str) -> None:
        msg = String()
        msg.data = command
        self._gripper_pub.publish(msg)

    def _publish_home(self) -> None:
        if not self._home_ready:
            self.get_logger().warn('Home pose not available yet (no joint_states).')
            return
        traj = JointTrajectory()
        traj.joint_names = self._arm_joint_names
        point = JointTrajectoryPoint()
        point.positions = [self._home_positions[name] for name in self._arm_joint_names]
        point.time_from_start.sec = int(self._home_duration)
        point.time_from_start.nanosec = int((self._home_duration % 1.0) * 1e9)
        traj.points = [point]
        self._arm_traj_pub.publish(traj)
        self.get_logger().info('Sent arm HOME trajectory.')

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        if self._last_joy is None or self._last_joy_time is None:
            self._publish_zero_cmds()
            return

        if (now - self._last_joy_time).nanoseconds / 1e9 > self._joy_timeout:
            self._publish_zero_cmds()
            return

        buttons = list(self._last_joy.buttons)
        axes = list(self._last_joy.axes)
        deadman = self._deadman_pressed(buttons)

        base_active = self._mode in ('base', 'hybrid') and deadman
        arm_active = self._mode in ('arm', 'hybrid') and deadman

        if base_active:
            self._publish_base_cmd(axes, buttons)
        else:
            self._publish_zero_base()

        if arm_active:
            if self._arm_control_mode == 'twist':
                self._publish_arm_twist(buttons, axes, allow_z=self._mode == 'arm')
            else:
                self._publish_arm_trajectory(buttons, axes, allow_z=self._mode == 'arm')
        else:
            self._publish_zero_arm()

    def _publish_base_cmd(self, axes: List[float], buttons: List[int]) -> None:
        lin_x = self._axis(axes, self._axis_linear_x)
        lin_y = self._axis(axes, self._axis_linear_y)
        if self._invert_linear_x:
            lin_x = -lin_x
        if self._invert_linear_y:
            lin_y = -lin_y

        if self._axis_angular_z >= 0:
            ang_z = self._axis(axes, self._axis_angular_z)
        elif self._mode == 'hybrid' and (self._dpad_axis_x >= 0 or self._dpad_axis_y >= 0):
            ang_z = 0.0
        else:
            ang_z = float(self._button(buttons, self._button_rot_right) - self._button(buttons, self._button_rot_left))

        twist = Twist()
        twist.linear.x = lin_x * self._scale_linear
        twist.linear.y = lin_y * self._scale_linear
        twist.angular.z = ang_z * self._scale_angular

        self._cmd_vel_pub.publish(twist)
        if self._cmd_vel_alt_pub is not None:
            self._cmd_vel_alt_pub.publish(twist)
        self._last_base_active = True

    def _publish_arm_twist(self, buttons: List[int], axes: List[float], allow_z: bool) -> None:
        if self._dpad_axis_x >= 0 or self._dpad_axis_y >= 0:
            dx = float(self._axis_to_dir(axes, self._dpad_axis_y))
            dy = float(self._axis_to_dir(axes, self._dpad_axis_x))
        else:
            dx = float(self._button(buttons, self._dpad_up) - self._button(buttons, self._dpad_down))
            dy = float(self._button(buttons, self._dpad_right) - self._button(buttons, self._dpad_left))
        dz = 0.0
        if allow_z:
            dz = float(self._button(buttons, self._arm_up_button) - self._button(buttons, self._arm_down_button))

        if math.isclose(dx, 0.0) and math.isclose(dy, 0.0) and math.isclose(dz, 0.0):
            self._publish_zero_arm()
            return

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._arm_frame
        msg.twist.linear.x = dx * self._arm_linear_scale
        msg.twist.linear.y = dy * self._arm_linear_scale
        msg.twist.linear.z = dz * self._arm_vertical_scale
        self._arm_twist_pub.publish(msg)
        self._last_arm_active = True

    def _publish_arm_trajectory(self, buttons: List[int], axes: List[float], allow_z: bool) -> None:
        if not self._arm_positions:
            return

        if self._dpad_axis_x >= 0 or self._dpad_axis_y >= 0:
            dx = self._axis_to_dir(axes, self._dpad_axis_y)
            dy = self._axis_to_dir(axes, self._dpad_axis_x)
        else:
            dx = self._button(buttons, self._dpad_up) - self._button(buttons, self._dpad_down)
            dy = self._button(buttons, self._dpad_right) - self._button(buttons, self._dpad_left)

        dz = 0
        if allow_z:
            dz = self._button(buttons, self._arm_up_button) - self._button(buttons, self._arm_down_button)

        if dx == 0 and dy == 0 and dz == 0:
            self._publish_zero_arm()
            return

        targets = dict(self._arm_positions)
        if len(self._arm_joint_names) >= 2:
            targets[self._arm_joint_names[0]] += dx * self._arm_joint_step
            targets[self._arm_joint_names[1]] += dy * self._arm_joint_step
        if allow_z and len(self._arm_joint_names) >= 3:
            targets[self._arm_joint_names[2]] += dz * self._arm_joint_step

        traj = JointTrajectory()
        traj.joint_names = list(self._arm_joint_names)
        point = JointTrajectoryPoint()
        point.positions = [targets[name] for name in self._arm_joint_names]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 200000000
        traj.points = [point]
        self._arm_traj_pub.publish(traj)
        self._last_arm_active = True

    def _publish_zero_cmds(self) -> None:
        self._publish_zero_base()
        self._publish_zero_arm()

    def _publish_zero_base(self) -> None:
        if not self._last_base_active:
            return
        self._cmd_vel_pub.publish(Twist())
        if self._cmd_vel_alt_pub is not None:
            self._cmd_vel_alt_pub.publish(Twist())
        self._last_base_active = False

    def _publish_zero_arm(self) -> None:
        if not self._last_arm_active:
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._arm_frame
        self._arm_twist_pub.publish(msg)
        self._last_arm_active = False


def main() -> None:
    rclpy.init()
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
