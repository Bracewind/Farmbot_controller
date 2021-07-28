from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped


def linear_interpolation(mini, maxi, nb_repetition, current_time):
    current_time2 = current_time % 10
    return mini + (maxi - mini) * current_time2 / nb_repetition


class StatePublisher(Node):
    def __init__(self):
        rclpy.init()
        super().__init__("state_publisher")

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, "joint_states", qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        loop_rate = self.create_rate(30)

        self.joint_names = [
            "joint_tracks_gantry",
            "joint_gantry_crossslide",
            "joint_crossslide_zaxis",
        ]

    def send_wanted_position(
        self, gantry_move: float, crossslide_move: float, zaxis_move: float
    ):
        joint_state = JointState()
        # update joint_state
        now = self.get_clock().now()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = self.joint_names
        joint_state.position = [gantry_move, crossslide_move, zaxis_move]

        """
                # update transform
                # (moving in a circle with radius=2)
                odom_trans.header.stamp = now.to_msg()
                odom_trans.transform.translation.x = cos(angle) * 2
                odom_trans.transform.translation.y = sin(angle) * 2
                odom_trans.transform.translation.z = 0.7
                odom_trans.transform.rotation = euler_to_quaternion(
                    0, 0, angle + pi / 2
                )  # roll,pitch,yaw
                """

        odom_trans = TransformStamped()
        odom_trans.header.frame_id = "odom"
        odom_trans.child_frame_id = "world"
        odom_trans.header.stamp = now.to_msg()
        odom_trans.transform.translation.x = 0.0
        odom_trans.transform.translation.y = 0.0
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = euler_to_quaternion(0, 0, 0)

        # send the joint state and transform
        self.joint_pub.publish(joint_state)
        self.broadcaster.sendTransform(odom_trans)


def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(
        pitch / 2
    ) * sin(yaw / 2)
    qy = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(
        pitch / 2
    ) * sin(yaw / 2)
    qz = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(
        pitch / 2
    ) * cos(yaw / 2)
    qw = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(
        pitch / 2
    ) * sin(yaw / 2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main():
    node = StatePublisher()


if __name__ == "__main__":
    main()