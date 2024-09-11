#!/usr/bin/env python3
import rosbags
import matplotlib.pyplot as plt
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import numpy as np
import os
from scipy.signal import argrelextrema, find_peaks
import argparse


def read_bag(bag_dir, verbose=False):
    
    uwb_raw, rot_raw, scan_meta, scan_ranges = [], [], [], []

    with Reader(bag_dir) as reader:
        # topic and msgtype information is available on .connections list
        if verbose: 
            for connection in reader.connections:
                print(connection.topic, connection.msgtype)

        # grab the raw data
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/uwb/range':
                msg = deserialize_cdr(rawdata, connection.msgtype)
                uwb_raw.append([timestamp, float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec*10**-9), msg.range])
            if connection.topic == '/lidar_rotations':
                msg = deserialize_cdr(rawdata, connection.msgtype)
                rot_raw.append([timestamp, int(msg.data)])
            if connection.topic == '/scan':
                msg = deserialize_cdr(rawdata, connection.msgtype)
                scan_meta.append([timestamp, msg.header.stamp.sec, msg.header.stamp.nanosec, msg.angle_min, msg.angle_max, \
                       msg.angle_increment, msg.time_increment, msg.scan_time, msg.range_min, msg.range_max])
                scan_ranges.append(msg.ranges)

    uwb_raw = np.array(uwb_raw)
    rot_raw = np.array(rot_raw)
    scan_meta = np.array(scan_meta)
    scan_ranges = np.array(scan_ranges)
    return uwb_raw, rot_raw, scan_meta, scan_ranges
                
def export_data(out_dir, scan_meta, scan_ranges, uwb, rot):
    np.savetxt(out_dir + '/SCM.csv', scan_meta ,delimiter=',', fmt='%.14f')
    np.savetxt(out_dir + '/SC.csv', scan_ranges ,delimiter=',', fmt='%.14f')
    np.savetxt(out_dir + '/UWB' +'.csv', uwb, delimiter=',', fmt='%.14f')

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--order', type=str,required=False)
    parser.add_argument('--bag_dir', type=str, required=False)
    parser.add_argument('--out_dir', type=str, required=False)
    parser.add_argument('--graph_dir', type=str, required=False)
    args = parser.parse_args()
    bag_dir = args.bag_dir
    out_dir = args.out_dir
    # graph_dir = args.graph_dir
    
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname)
    #bag_dir = '/home/mvm1/blade/v3/dep/bags/rosbag2_2023_05_19-11_48_15_0_0' #testing
    uwb, rot, scan_meta, scan_ranges = read_bag(bag_dir)
    export_data(os.getcwd() + '/bag_dumps', scan_meta, scan_ranges, uwb, rot)
    
