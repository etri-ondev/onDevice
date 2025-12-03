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


def _read(path: str) -> str:
    try:
        with open(path, 'r') as f:
            return f.read().strip()
    except Exception:
        return ""


def is_iface_link_up(iface: str) -> bool:
    carrier = _read(f"/sys/class/net/{iface}/carrier")
    oper    = _read(f"/sys/class/net/{iface}/operstate")
    if carrier == '1' and oper == 'up':
        return True
    return False


def has_ipv4_addr(iface: str) -> bool:
    try:
        out = subprocess.run(["ip", "-4", "-o", "addr", "show", "dev", iface],
                             text=True, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
        return bool(out.stdout.strip())
    except Exception:
        return False


class IfaceWatcher:
    def __init__(self, iface: str, on_up, on_down, interval=0.5, debounce_cnt=3):
        self.iface = iface
        self.on_up = on_up
        self.on_down = on_down
        self.interval = interval
        self.debounce_cnt = debounce_cnt
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)
        self._state = None  # True=UP, False=DOWN

    def start(self):
        self._t.start()

    def stop(self):
        self._stop.set()
        self._t.join(timeout=1.0)

    def _run(self):
        last_seen = None
        same_cnt = 0
        while not self._stop.is_set():
            up = is_iface_link_up(self.iface)
            if up and not has_ipv4_addr(self.iface):
                up = False

            if up == last_seen:
                same_cnt += 1
            else:
                same_cnt = 1
                last_seen = up

            if same_cnt >= self.debounce_cnt and up != self._state:
                self._state = up
                try:
                    if up:
                        self.on_up()
                    else:
                        self.on_down()
                except Exception:
                    pass
            time.sleep(self.interval)


def _run(cmd, check=False):
    return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=check)


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
        
        raw = pd.read_csv(csv)
        present = [c for c in COLUMN_MAP.keys() if c in raw.columns]
        df = raw[present].rename(columns=COLUMN_MAP).copy()

        if 'alert_level_sub' not in df.columns:
            df['alert_level_sub'] = 0 

        msg_fields = RadarDetection.get_fields_and_field_types().keys()
        for f in msg_fields:
            if f in df.columns:
                continue
            if f in INT_FIELDS:
                df[f] = 0
            elif f in FLT_FIELDS:
                df[f] = 0.0
            elif f in STR_FIELDS:
                df[f] = ''
            else:
                df[f] = '' if f.endswith('_id') or f.endswith('_name') else 0

        for f in df.columns:
            try:
                if f in INT_FIELDS:
                    df[f] = df[f].fillna(0).astype(int)
                elif f in FLT_FIELDS:
                    df[f] = df[f].fillna(0.0).astype(float)
                elif f in STR_FIELDS:
                    df[f] = df[f].fillna('').astype(str)
                else:
                    df[f] = df[f].astype(str)
            except Exception:
                if f in INT_FIELDS:
                    df[f] = 0
                elif f in FLT_FIELDS:
                    df[f] = 0.0
                else:
                    df[f] = ''

        self.df   = df.reset_index(drop=True)
        self.rows = len(self.df)
        self.idx  = 0

        # ── ROS2 Publisher ──────────────────────────────────────────
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.pub  = self.create_publisher(RadarDetection, 'radar/table', qos_profile=qos)
        self.stat = self.create_publisher(Float32,        'radar/pub_stats', 5)

        # ── 타이밍 ──────────────────────────────────────────────────
        self.base     = max(0.01, 1.0 / self.rate_hz)
        self.next     = time.time()
        self.sent_sec = 0
        self.sec_mark = int(time.time())
    
    def _row_to_msg(self, row: pd.Series) -> RadarDetection:
        d = {}
        for f in RadarDetection.get_fields_and_field_types().keys():
            if f in row:
                v = row[f]
            else:
                if f in INT_FIELDS:   v = 0
                elif f in FLT_FIELDS: v = 0.0
                elif f in STR_FIELDS: v = ''
                else:                 v = 0
            try:
                if f in INT_FIELDS:   v = int(v)
                elif f in FLT_FIELDS: v = float(v)
                elif f in STR_FIELDS: v = str(v)
            except Exception:
                if f in INT_FIELDS:   v = 0
                elif f in FLT_FIELDS: v = 0.0
                elif f in STR_FIELDS: v = ''
            d[f] = v

        if 'time' in d:
            d['time'] = time.strftime('%Y-%m-%dT%H:%M:%S.%fZ', time.gmtime())
        return RadarDetection(**d)
        