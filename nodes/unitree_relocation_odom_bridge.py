#!/usr/bin/env python3
"""
Unitree SLAM relocation odom bridge (DDS domain 0 -> ROS 2 domain 1).

Subscribes: rt/unitree/slam_relocation/odom  (nav_msgs::msg::dds_::Odometry_)
Publishes:  /unitree/odom + TF (frames from message, typically map -> base_link)
"""
import os

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_ as OdometryDds

ChannelFactoryInitialize(0, 'eth0')
os.environ.setdefault('ROS_DOMAIN_ID', '1')

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

UNITREE_ODOM_TOPIC = 'rt/unitree/slam_relocation/odom'
ROS_ODOM_TOPIC = '/unitree/odom'
DEFAULT_PARENT_FRAME = 'odom'
DEFAULT_CHILD_FRAME = 'base_link'


class UnitreeRelocationOdomBridge(Node):
    def __init__(self):
        super().__init__('unitree_relocation_odom_bridge')
        self.odom_pub = self.create_publisher(Odometry, ROS_ODOM_TOPIC, 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self._warned_frames = False
        self.sub = ChannelSubscriber(UNITREE_ODOM_TOPIC, OdometryDds)
        self.sub.Init(self._odom_callback, 1)
        self.get_logger().info(f'Subscribed to {UNITREE_ODOM_TOPIC} (Unitree DDS domain 0)')

    def _odom_callback(self, msg: OdometryDds):
        try:
            odom = self._dds_to_ros_odometry(msg)
            self.odom_pub.publish(odom)
            self._publish_tf(odom)
        except Exception as e:
            self.get_logger().error(f'Callback error: {e}')

    def _dds_to_ros_odometry(self, msg: OdometryDds) -> Odometry:
        odom = Odometry()
        parent_frame = msg.header.frame_id or DEFAULT_PARENT_FRAME
        child_frame = msg.child_frame_id or DEFAULT_CHILD_FRAME

        if not self._warned_frames and (
            parent_frame != DEFAULT_PARENT_FRAME or child_frame != DEFAULT_CHILD_FRAME
        ):
            self.get_logger().warn(
                f'Unitree odom frames [{parent_frame} -> {child_frame}] '
                f'(Nav2 launch uses static map->odom identity)',
                throttle_duration_sec=10.0,
            )
            self._warned_frames = True

        odom.header.stamp.sec = int(msg.header.stamp.sec)
        odom.header.stamp.nanosec = int(msg.header.stamp.nanosec)
        if odom.header.stamp.sec == 0 and odom.header.stamp.nanosec == 0:
            odom.header.stamp = self.get_clock().now().to_msg()

        odom.header.frame_id = parent_frame
        odom.child_frame_id = child_frame
        odom.pose.pose.position.x = float(msg.pose.pose.position.x)
        odom.pose.pose.position.y = float(msg.pose.pose.position.y)
        odom.pose.pose.position.z = float(msg.pose.pose.position.z)
        odom.pose.pose.orientation.x = float(msg.pose.pose.orientation.x)
        odom.pose.pose.orientation.y = float(msg.pose.pose.orientation.y)
        odom.pose.pose.orientation.z = float(msg.pose.pose.orientation.z)
        odom.pose.pose.orientation.w = float(msg.pose.pose.orientation.w)
        odom.pose.covariance = [float(v) for v in msg.pose.covariance]
        odom.twist.twist.linear.x = float(msg.twist.twist.linear.x)
        odom.twist.twist.linear.y = float(msg.twist.twist.linear.y)
        odom.twist.twist.linear.z = float(msg.twist.twist.linear.z)
        odom.twist.twist.angular.x = float(msg.twist.twist.angular.x)
        odom.twist.twist.angular.y = float(msg.twist.twist.angular.y)
        odom.twist.twist.angular.z = float(msg.twist.twist.angular.z)
        odom.twist.covariance = [float(v) for v in msg.twist.covariance]
        return odom

    def _publish_tf(self, odom: Odometry):
        tf = TransformStamped()
        tf.header.stamp = odom.header.stamp
        tf.header.frame_id = odom.header.frame_id
        tf.child_frame_id = odom.child_frame_id
        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.translation.z = odom.pose.pose.position.z
        tf.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = UnitreeRelocationOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
