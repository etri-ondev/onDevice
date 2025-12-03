# Copyright 2025 Electronics and Telecommunications Research Institute
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os, json, socket, select
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from radar_detection_msgs.msg import RadarDetection


class Udp2Ros(Node):
    def __init__(self):
        super().__init__('udp2ros_bridge')

        bt_if   = os.environ.get('BT_IF', 'bt0')
        port    = int(os.environ.get('BT_PORT', '9999'))
        ifidx   = socket.if_nametoindex(bt_if)

        # ROS publisher (BEST_EFFORT)
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub  = self.create_publisher(RadarDetection, 'radar/table', qos)
        self.stat = self.create_publisher(Float32,        'radar/udp_rx_rate', 5)

        self.sock = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
        self.sock.setsockopt(1, 25, (bt_if + '\0').encode())  # SO_BINDTODEVICE
        self.sock.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 1)
        self.sock.bind(('::', port, 0, ifidx))
        self.sock.setblocking(False)
        self.get_logger().info(f'UDP6 listen on [{bt_if}] :{port}')

    def _rx_once(self):
        r, _, _ = select.select([self.sock], [], [], 0)
        if not r:
            return

        try:
            data, addr = self.sock.recvfrom(4096)
        except BlockingIOError:
            return

        self.get_logger().info(f'UDP rx {len(data)}B from {addr}')
        self.get_logger().info(f'head: {data[:120]!r}')

        try:
            txt = data.decode('utf-8', errors='replace').strip()
            obj = json.loads(txt)
            if not isinstance(obj, dict):
                self.get_logger().warn('payload is not a JSON object')
                return
        except Exception as e:
            self.get_logger().warn(f'JSON decode failed: {e}')
            return
