import os
import sys
import time
import json
import random
import pathlib
import threading
import socket
import subprocess

import numpy as np
import pandas as pd

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32
from radar_detection_msgs.msg import RadarDetection

COLUMN_MAP = {
    'SourceDeviceCode'     : 'source_device_code',
    'DestinationDeviceCode': 'destination_device_code',
    'MessageCode'          : 'message_code',
    'DataSize'             : 'data_size',
    'DetectionCount'       : 'detection_count',
    'object_id'            : 'full_id',
    'ViewID'               : 'view_id',
    'time_step'            : 'time',
    'Range'                : 'range',
    'Azimuth'              : 'azimuth',
    'Elevation'            : 'elevation',
    'Altitude'             : 'altitude',
    'RelativeVelocity'     : 'relative_velocity',
    'AbsoluteVelocity'     : 'absolute_velocity',
    'TrackedTime'          : 'tracked_time',
    'TrackedDistance'      : 'tracked_distance',
    'SpotlightMode'        : 'spotlight_mode',
    'current_alert_level'  : 'alert_level',
}

INT_FIELDS = {
    'source_device_code', 'destination_device_code', 'message_code',
    'data_size', 'detection_count', 'spotlight_mode',
    'alert_level', 'alert_level_sub'
}
FLT_FIELDS = {
    'range', 'azimuth', 'elevation', 'altitude',
    'relative_velocity', 'absolute_velocity', 'tracked_time', 'tracked_distance'
}
STR_FIELDS = {
    'full_id', 'view_id', 'time'
}

class CsvPublisher(Node):
    def __init__(self, csv_path: str):
        super().__init__('excel_radar_publisher')

        self.declare_parameter('rate_hz', float(os.environ.get('RATE_HZ', '8.0')))
        self.declare_parameter('verbose', True)
        self.declare_parameter('eth_iface', os.environ.get('ETH_IF', 'enP8p1s0'))  
        self.declare_parameter('bt_iface',  os.environ.get('BT_IF',  'bt0'))

        self.rate_hz   = float(self.get_parameter('rate_hz').value)
        self.verbose   = bool (self.get_parameter('verbose').value)
        self.eth_iface = str(self.get_parameter('eth_iface').value)
        self.bt_iface  = str(self.get_parameter('bt_iface').value)

        self.b_ll    = os.environ.get('B_LL', '').strip()  
        self.bt_port = int(os.environ.get('BT_PORT', '9999'))

        csv = pathlib.Path(csv_path)
        if not csv.exists():
            raise FileNotFoundError(f'CSV not found: {csv}')
