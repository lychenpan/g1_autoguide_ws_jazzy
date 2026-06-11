#!/usr/bin/env python3
"""
Unitree pointcloud bridge (DDS domain 0 -> ROS 2 domain 1).

"""
import os

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_ as PointCloud2Dds

# Initialize DDS domain 0 participant on interface eth0
ChannelFactoryInitialize(0, 'eth0')
os.environ.setdefault('ROS_DOMAIN_ID', '1')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

UNITREE_PCL_TOPIC = 'rt/utlidar/cloud_livox_mid360'
ROS_PCL_TOPIC = '/utlidar/pcl2'
DEFAULT_FRAME = 'livox_frame'


class UtlidarPCLBridge(Node):
    def __init__(self):
        super().__init__('utlidar_pcl_bridge')
        self.pcl_pub = self.create_publisher(PointCloud2, ROS_PCL_TOPIC, 10)
        # receive counters for diagnostics
        self.recv_count = 0
        self.recv_count_alt = 0
        # main topic (no leading slash)
        try:
            self.sub = ChannelSubscriber(UNITREE_PCL_TOPIC, PointCloud2Dds)
            self.sub.Init(self._pcl_callback, 1)
            self.get_logger().info(f'Subscribed to {UNITREE_PCL_TOPIC} (Unitree DDS domain 0)')
        except Exception as e:
            self.sub = None
            self.get_logger().error(f'Failed to init main subscriber: {e}')
        
        self.get_logger().info(f'Bridge initialized: DDS({UNITREE_PCL_TOPIC}) -> ROS2({ROS_PCL_TOPIC})')
        # periodic status log to show if any messages were received
        # self.create_timer(5.0, self._status_timer)

    def _pcl_callback(self, msg: PointCloud2Dds):
        try:
            self.recv_count += 1
            data_len = len(getattr(msg, 'data', b'')) if getattr(msg, 'data', None) else 0
            # self.get_logger().info(f'recv #{self.recv_count} main: width={getattr(msg, "width", 0)} data_len={data_len}')
            pcl = self._dds_to_ros_pointcloud(msg)
            self.pcl_pub.publish(pcl)
        except Exception as e:
            self.get_logger().error(f'Callback error: {e}')

    def _status_timer(self):
        self.get_logger().info(f'PCL recv counts: main={self.recv_count} alt={self.recv_count_alt}')

    def _dds_to_ros_pointcloud(self, msg: PointCloud2Dds) -> PointCloud2:
        out = PointCloud2()
        # Match unitree_relocation_odom_bridge: use domain-1 ROS clock so TF and
        # costmap message filters share the same timeline as /unitree/odom.
        out.header.stamp = self.get_clock().now().to_msg()

        # frame
        out.header.frame_id = getattr(msg.header, 'frame_id', DEFAULT_FRAME) or DEFAULT_FRAME

        # dimensions
        out.height = int(getattr(msg, 'height', 1))
        out.width = int(getattr(msg, 'width', 0))

        # fields
        out.fields = []
        try:
            for f in getattr(msg, 'fields', []):
                pf = PointField()
                pf.name = str(getattr(f, 'name', ''))
                pf.offset = int(getattr(f, 'offset', 0))
                pf.datatype = int(getattr(f, 'datatype', 0))
                pf.count = int(getattr(f, 'count', 1))
                out.fields.append(pf)
        except Exception:
            out.fields = []

        out.is_bigendian = bool(getattr(msg, 'is_bigendian', False))
        out.point_step = int(getattr(msg, 'point_step', 0))
        out.row_step = int(getattr(msg, 'row_step', 0))
        out.is_dense = bool(getattr(msg, 'is_dense', False))

        # data
        try:
            data = getattr(msg, 'data', None)
            if data is None:
                out.data = b''
            else:
                if isinstance(data, (bytes, bytearray, memoryview)):
                    out.data = bytes(data)
                else:
                    out.data = bytes(list(data))
        except Exception:
            out.data = b''

        return out


def main(args=None):
    rclpy.init(args=args)
    node = UtlidarPCLBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
