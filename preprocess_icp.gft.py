#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from copy import deepcopy
import collections
import argparse 
import json
import rosbags
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import os
import time
from collections import defaultdict

IMG_EXTENSION = ".png"

ROT_INPUT = 5880
section = 'lead'
orientation = 0
dz = 0.14 / 10
n_scans = 1
search_ratio = 30.0
n_samples = 2000000
z_off = 0.25
slide_res = 0.025
n_icp_iters = 50
threshold = 2
cm_depth = 0.05

def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def trans_init(section='None', cm=[0, 0], z=0):
    ref = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
    rot = [0, 0, 0]
    trans = [0, 0, z]
    tf = np.eye(4)
    if orientation == 0:
        if section == 'lead':
            rot = [0, 0, np.radians(-160)]
        elif section == 'web':
            rot = [0, 0, np.radians(160)]
        elif section == 'trail':
            rot = [0, 0, np.radians(145)]
    else:
        if section == 'lead':
            rot = [0, 0, np.radians(-35)]
        elif section == 'web':
            rot = [0, 0, np.radians(0)]
        elif section == 'trail':
            rot = [0, 0, np.radians(0)]

    trans = [cm[0], cm[1], z]
    tf[:3, :3] = ref.get_rotation_matrix_from_xyz(rot)
    tf[:3, 3] = trans
    return tf

def read_bag(bag_dir, verbose=False):
    uwb_raw, rot_raw, scan_meta, scan_ranges = [], [], [], []
    with Reader(bag_dir) as reader:
        if verbose:
            for connection in reader.connections:
                print(connection.topic, connection.msgtype)

        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == '/uwb/range':
                msg = deserialize_cdr(rawdata, connection.msgtype)
                uwb_raw.append([timestamp, float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec * 10 ** -9), msg.range])
            if connection.topic == '/lidar_rotations':
                msg = deserialize_cdr(rawdata, connection.msgtype)
                rot_raw.append([timestamp, int(msg.data)])
            if connection.topic == '/scan':
                msg = deserialize_cdr(rawdata, connection.msgtype)
                scan_meta.append([timestamp, msg.header.stamp.sec, msg.header.stamp.nanosec, msg.angle_min, msg.angle_max,
                                  msg.angle_increment, msg.time_increment, msg.scan_time, msg.range_min, msg.range_max])
                scan_ranges.append(msg.ranges)

    return np.array(uwb_raw), np.array(rot_raw), np.array(scan_meta), np.array(scan_ranges)

class Slice:
    def __init__(self, pcl, z, z_bounds, scan_index=None, pcl_type='model'):
        self.pcl = pcl if isinstance(pcl, type(o3d.geometry.PointCloud())) else o3d.geometry.PointCloud()
        if not isinstance(pcl, type(o3d.geometry.PointCloud())):
            self.pcl.points = o3d.utility.Vector3dVector(pcl)
        self.pcl.estimate_normals()
        self.z = z
        self.z_bounds = z_bounds
        self.scan_index = scan_index
        self.type = pcl_type
        self.n_pts = len(self.pcl.points)
        self.cm = self.calc_cm(np.asarray(self.pcl.points)[:, :2])
        if self.type == 'scan':
            self.pcl.paint_uniform_color([1, 0.706, 0])
            cm_pcl = deepcopy(self.pcl)
            cm_pcl.transform(trans_init(section))
            self.cm = self.calc_cm(np.asarray(cm_pcl.points)[:, :2])
        elif self.type == 'model':
            self.pcl.paint_uniform_color([0, 0.651, 0.929])

    def calc_cm(self, pts):
        return np.mean(pts, axis=0) if len(pts) >= 0 else None

def read_data(models_dir, section, sc, scm, uwb):
    stl_path = os.path.join(models_dir, f'{section}-75.STL')
    print(f'Reading {section} STL from path {stl_path}...')
    bmesh = o3d.io.read_triangle_mesh(stl_path)
    bmesh.scale(1 / 1000, center=(0, 0, 0))
    bmesh.compute_vertex_normals()
    bpcl = bmesh.sample_points_uniformly(1000000)
    bpcl.estimate_normals()
    bpcl.paint_uniform_color([0, 0.651, 0.929])
    return [sc, scm, uwb], bmesh

def rot2z(scm, uwb, ts):
    scan_ind = (np.abs(scm[:, 0] - ts)).argmin()
    diff = np.abs(uwb[:, 0] - ts)
    z = uwb[np.where(diff == diff.min())[0][0], -1]
    return z, scan_ind

def pol2cart(pol_pts, A_MIN, A_INC, z):
    theta = A_MIN + np.arange(len(pol_pts)) * A_INC
    mask = ~np.isinf(pol_pts)
    pol_pts = pol_pts[mask]
    theta = theta[mask]
    return np.column_stack((pol_pts * np.cos(theta), pol_pts * np.sin(theta), np.full_like(pol_pts, z)))

def rot2slice(run_data, ts, z_off, dz, n):
    sc, scm, uwb = run_data
    print('Matching rotation count to Z estimate...')
    z, ind = rot2z(scm, uwb, ts)
    z += z_off
    print('Building the scan slice...')
    z_vec = np.linspace(z - dz * n, z + dz * n, n * 2 + 1)
    z_bounds = (z_vec[0], z_vec[-1])
    scans_pol = sc[ind - n:ind + n + 1]
    scans_xyz = [pol2cart(scans_pol[i], scm[ind, 3], scm[ind, 5], _z) for i, _z in enumerate(z_vec)]
    return Slice(np.concatenate(scans_xyz), z, z_bounds, ind, pcl_type='scan')

def register(source, target, tf_init):
    reg = o3d.pipelines.registration.registration_icp(source, target, threshold, tf_init,
                                                      o3d.pipelines.registration.TransformationEstimationPointToPoint())
    return reg.transformation, reg.fitness, reg.inlier_rmse

def slice_mdl(bmesh, scan_slice, search_ratio, n_samples):
    print('Cropping the model slice...')
    z_min = scan_slice.z + search_ratio * (scan_slice.z_bounds[0] - scan_slice.z)
    z_max = scan_slice.z + search_ratio * (scan_slice.z_bounds[1] - scan_slice.z)
    bpcl = bmesh.sample_points_uniformly(n_samples)
    pts = [pt for pt in np.asarray(bpcl.points) if z_min <= pt[2] <= z_max]
    return Slice(pts, scan_slice.z, (z_min, z_max))

def slide_icp(scan_slice, model_slice, vis=False):
    z_min, z_max = model_slice.z_bounds
    z = scan_slice.z
    z_uwb = z - z_off
    z_init_vec = np.linspace(z_min, z_max, int((z_max - z_min) / slide_res))
    results = []
    for z_init in z_init_vec:
        _z = z_init - z
        tf, fitness, rmse = register(scan_slice.pcl, model_slice.pcl, trans_init(section, model_slice.cm, _z))
        if fitness > 0.0:
            results.append([tf, fitness, rmse, z_init])
        else:
            print(f'Did not converge for z_init: {z_init}')

    if results:
        results = sorted(results, key=lambda x: x[2])
        pos = np.matmul(results[0][0], np.array([0, 0, scan_slice.z, 1]).T)
        print(f'Best result with rmse = {results[0][2]}')
        print(f'Estimated z of {pos[2]}m at z_init: {results[0][3]}m and z uwb: {z_uwb}m')
        return True, (pos, results[0][0], z_uwb, results[0][1], results[0][2])
    return False, None

def estimate_poses_icp(ts_list, models_dir, section, sc, scm, uwb):
    run_data, bmesh = read_data(models_dir, section, sc, scm, uwb)
    ts_poses_dict = collections.defaultdict(dict)

    time_scan_slice = 0
    time_model_slice = 0
    time_icp = 0
    
    for ts_row in ts_list:
        z, frame, ts = ts_row
        ts = ts * (10 ** 9)
        try:
            print("Matching rotation count to Z estimate...")
            scan_slice_start = time.time()
            scan_slice = rot2slice(run_data, ts, z_off, dz, n_scans)
            time_scan_slice += time.time() - scan_slice_start
            print("Building the scan slice...")

            print("Cropping the model slice...")
            model_slice_start = time.time()
            model_slice = slice_mdl(bmesh, scan_slice, search_ratio, n_samples)
            time_model_slice += time.time() - model_slice_start

            print("Running ICP registration...")
            icp_start = time.time()
            success, res = slide_icp(scan_slice, model_slice, vis=False)
            time_icp += time.time() - icp_start

            if success and res is not None:
                ts_poses_dict[int(frame)]['z'] = int(z)
                ts_poses_dict[int(frame)]['timestamp'] = int(ts)
                ts_poses_dict[int(frame)]['position'] = res[0].tolist() if res[0] is not None else None
                ts_poses_dict[int(frame)]['transform'] = res[1].tolist() if res[1] is not None else None
            else:
                print(f"Did not converge for frame: {frame}")
                ts_poses_dict[int(frame)]['z'] = int(z)
                ts_poses_dict[int(frame)]['timestamp'] = int(ts)
                ts_poses_dict[int(frame)]['position'] = ""
                ts_poses_dict[int(frame)]['transform'] = ""

        except Exception as e:
            print(f"Error processing frame {frame}: {e}")
            ts_poses_dict[int(frame)]['z'] = int(z)
            ts_poses_dict[int(frame)]['timestamp'] = int(ts)
            ts_poses_dict[int(frame)]['position'] = ""
            ts_poses_dict[int(frame)]['transform'] = ""

    print(f'Scan Slice Time: {time_scan_slice}')
    print(f'Model Slice Time: {time_model_slice}')
    print(f'ICP Time: {time_icp}')

    return ts_poses_dict


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--section', type=str)
    parser.add_argument('--frame_list_path', type=str)
    parser.add_argument('--json_out_dir', type=str)
    parser.add_argument('--bag_dir', type=str)
    parser.add_argument('--video_ident', type=str)
    parser.add_argument('--models_dir', type=str)
    parser.add_argument('--enable_icp', type=str, default="False")

    args = parser.parse_args()
    section = args.section
    frame_list_path = args.frame_list_path
    json_out_dir = args.json_out_dir
    bag_dir = args.bag_dir
    video_ident = args.video_ident
    models_dir = args.models_dir
    enable_icp = str2bool(args.enable_icp)

    frame_list = np.genfromtxt(frame_list_path, delimiter=',', dtype=None, skip_header=1)

    if enable_icp:
        list_of_bags = os.listdir(bag_dir)
        assert len(list_of_bags) == 1
        uwb, rot_raw, scm, sc = read_bag(os.path.join(bag_dir, list_of_bags[0]))
        ts_poses_dict = estimate_poses_icp(frame_list, models_dir, section, sc, scm, uwb)
    else:
        ts_poses_dict = defaultdict(empty_poses_entry)

    image_meta_data = {
        "image_ts": "",
        "defect_severity": "n_a",
        "defect_location": "",
        "defect_size": "0.0",
        "defect_desc": "",
    }
    for row in frame_list:
        frame_number = row[1]
        z_dist = row[0]
        ts_poses_dict[int(frame_number)]["sect"] = section
        ts_poses_dict[int(frame_number)]["eqHeight"] = 2688
        ts_poses_dict[int(frame_number)]["eqWidth"] = 5376

        json_filename = str(video_ident) + "img" + str(frame_number) + ".json"
        json_path = os.path.join(json_out_dir, json_filename)

        image_meta_data["frame"] = ts_poses_dict[int(frame_number)]
        image_meta_data["image_id"] = int(frame_number)
        image_meta_data["image_distance"] = float(z_dist)
        image_meta_data["image_path"] = os.path.splitext(json_path)[0] + IMG_EXTENSION

        image_json = json.dumps(image_meta_data, indent=4)
        with open(json_path, "w") as outfile:
            outfile.write(image_json)
