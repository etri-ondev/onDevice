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

        # ── UDP 폴백 준비 ───────────────────────────────────────────
        self._fallback = False     
        self._udp = None
        self._udp_dst = None

        if self.b_ll:
            try:
                ifidx = socket.if_nametoindex(self.bt_iface) 
                s = socket.socket(socket.AF_INET6, socket.SOCK_DGRAM)
                try:
                    s.setsockopt(1, 25, (self.bt_iface + '\0').encode())
                except Exception:
                    pass
                s.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 1)
                self._udp = s
                self._udp_dst = (self.b_ll, self.bt_port, 0, ifidx)
                self.get_logger().info(f"UDP fallback ready → {self._udp_dst}")
            except Exception as e:
                self.get_logger().warn(f"UDP fallback init failed: {e}")
                self._udp = None
                self._udp_dst = None
        else:
            self.get_logger().warn("UDP fallback disabled (env B_LL not set)")

        threading.Thread(target=self._watch_eth_and_switch,
                         args=(self.eth_iface,),
                         daemon=True).start()

        self.create_timer(0.01, self._tick)

        self.get_logger().info(
            f"Loaded {self.rows} rows • rate≈{self.rate_hz} Hz • "
            f"ETH={self.eth_iface} • BT={self.bt_iface} • UDP_Fallback={'ON' if self._udp_dst else 'OFF'}"
        )
    
    def _is_link_really_up(self, iface: str) -> bool:
        def _read(p):
            try:
                with open(p, 'r') as f:
                    return f.read().strip()
            except Exception:
                return ''

        carrier   = _read(f"/sys/class/net/{iface}/carrier") == '1'
        oper_up   = _read(f"/sys/class/net/{iface}/operstate") == 'up'
        has_ipv4  = False
        try:
            out = subprocess.run(["ip", "-4", "-o", "addr", "show", "dev", iface],
                                 text=True, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
            has_ipv4 = bool(out.stdout.strip())
        except Exception:
            pass
        return carrier and oper_up and has_ipv4

    def _watch_eth_and_switch(self, iface: str):
        last_state = None
        same = 0
        while rclpy.ok():
            up = self._is_link_really_up(iface)
            if up == last_state:
                same += 1
            else:
                same = 1
                last_state = up

            if same >= 3:  # debounce
                if up and self._fallback:
                    self._fallback = False
                    self.get_logger().info(f"{iface} link UP → UDP fallback DISABLED")
                elif (not up) and (not self._fallback):
                    self._fallback = True
                    self.get_logger().warn(f"{iface} link DOWN → UDP fallback ENABLED (bt0)")
            time.sleep(0.5)
    
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
    
    def _tick(self):
        now = time.time()
        if now < self.next:
            return

        row = self.df.iloc[self.idx]
        msg = self._row_to_msg(row)
        self.pub.publish(msg)

        if self._fallback and self._udp and self._udp_dst:
            try:
                payload = {k: getattr(msg, k) for k in msg.get_fields_and_field_types().keys()}
                self._udp.sendto(json.dumps(payload).encode('utf-8'), self._udp_dst)
                if self.verbose:
                    self.get_logger().info(f"▸ pub {self.idx+1}/{self.rows} via UDP+ROS2")
            except Exception as e:
                self.get_logger().warn(f"UDP fallback send failed: {e}")
        else:
            if self.verbose:
                self.get_logger().info(f"▸ pub {self.idx+1}/{self.rows} via ROS2")

        self.idx  = (self.idx + 1) % self.rows
        self.next = now + self.base + self.base * 0.3 * random.uniform(-1, 1)

        sec = int(now)
        self.sent_sec += 1
        if sec != self.sec_mark:
            try:
                self.stat.publish(Float32(data=float(self.sent_sec)))
            except Exception:
                pass
            self.sent_sec = 0
            self.sec_mark = sec


def main():
    print("Olla")
    csv_env = os.environ.get('CSV_PATH')
    if csv_env:
        csv_path = pathlib.Path(csv_env)
    else:
        csv_path = pathlib.Path.home() / 'Project' / 'ROS2_FS' / 'data' / 'RadarDetectionTable.csv'

    rclpy.init(args=sys.argv)
    node = CsvPublisher(str(csv_path))

    from rclpy.executors import SingleThreadedExecutor
    exe = SingleThreadedExecutor()
    exe.add_node(node)
    try:
        exe.spin()
    finally:
        exe.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
