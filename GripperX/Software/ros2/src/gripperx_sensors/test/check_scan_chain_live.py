#!/usr/bin/env python3
"""Measure /scan_raw -> /scan for message loss and latency on a RUNNING system.

Written during the 2026-08-24 audit of scan_range_filter and kept because the
question it answers cannot be settled on the laptop. On this machine, under a
load average of 18 on 8 cores with the twin at RTF 0.38, the chain lost a
contiguous burst of 8 messages out of 171 and showed latency spikes to 38 ms
against a 1.9 ms median. The mechanism is understood - a single-threaded rclpy
executor starved of CPU lets the subscriber's KEEP_LAST queue evict unread
samples - but whether the Pi does the same under its own concurrent load of
SLAM, Nav2, ros2_control and this filter has NOT been measured.

RUN THIS EARLY on the robot, before trusting SLAM output:

    source /opt/ros/jazzy/setup.bash
    source /home/ubuntu/ws/Software/ros2/install/setup.bash
    export ROS_DOMAIN_ID=20
    export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/fastdds_udp_only.xml
    python3 check_scan_chain_live.py

Read it as: loss near zero and latency well under one scan period (100 ms at
10 Hz) means the chain is not costing scans. Sustained loss, or latency above a
period, means the filter is being starved and SLAM is seeing gaps - in that case
the fallback is to drop the filter from the chain (driver back onto /scan) and
accept the self-returns for the session rather than lose scans.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
import time

DURATION = 25.0

class Probe(Node):
    def __init__(self):
        super().__init__('loss_latency_probe')
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.raw = {}  # stamp_key -> wall arrival time
        self.filt = {}
        self.create_subscription(LaserScan, '/scan_raw', self.on_raw, qos)
        self.create_subscription(LaserScan, '/scan', self.on_filt, qos)

    def key(self, msg):
        return (msg.header.stamp.sec, msg.header.stamp.nanosec)

    def on_raw(self, msg):
        self.raw[self.key(msg)] = time.monotonic()

    def on_filt(self, msg):
        self.filt[self.key(msg)] = time.monotonic()


def main():
    rclpy.init()
    node = Probe()
    start = time.monotonic()
    try:
        while time.monotonic() - start < DURATION:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        pass

    raw_keys = set(node.raw.keys())
    filt_keys = set(node.filt.keys())
    lost = raw_keys - filt_keys
    extra = filt_keys - raw_keys
    common = raw_keys & filt_keys
    latencies = sorted((node.filt[k] - node.raw[k]) * 1000.0 for k in common)

    print(f"window: {DURATION}s")
    print(f"raw messages received:      {len(raw_keys)}")
    print(f"filtered messages received: {len(filt_keys)}")
    print(f"raw stamps with NO matching filtered stamp (lost by filter, or filter msg not yet arrived): {len(lost)}")
    print(f"filtered stamps with NO matching raw stamp (should be 0, filter invents nothing): {len(extra)}")
    if latencies:
        n = len(latencies)
        print(f"latency (arrival filtered - arrival raw), n={n}")
        print(f"  min:    {latencies[0]:.2f} ms")
        print(f"  median: {latencies[n//2]:.2f} ms")
        print(f"  p90:    {latencies[int(n*0.9)]:.2f} ms")
        print(f"  max:    {latencies[-1]:.2f} ms")
    if lost:
        print("lost stamps (sec,nanosec):", sorted(lost)[:20])
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
