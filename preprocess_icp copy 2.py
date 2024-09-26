#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from pathlib import Path
from copy import deepcopy
import collections
import argparse
import json
import rosbags
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import os
from scipy.signal import argrelextrema, find_peaks
import time
from collections import defaultdict

IMG_EXTENSION = ".png"

# Configuration variables
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

def trans_init(section='None', cm=[0,0], z=0):
    print(f"Initializing transformation with section: {section}, cm: {cm}, z: {z}")
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
    print("Transformation initialized.")
    return tf

def read_bag(bag_dir, verbose=False):
    print(f"Reading bag from directory: {bag_dir}")
    uwb_raw, rot_raw, scan_meta, scan_ranges = [], [], [], []

    with Reader(bag_dir) as reader:
        if verbose:
            for connection in reader.connections:
                print(f"Connection topic: {connection.topic}, msgtype: {connection.msgtype}")

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
    print("Bag read successfully.")
    return uwb_raw, rot_raw, scan_meta, scan_ranges

class Slice:
    def __init__(self, pcl, z, z_bounds, scan_index=None, pcl_type='model'):
        print(f"Initializing Slice with z: {z}, z_bounds: {z_bounds}, type: {pcl_type}")
        if isinstance(pcl, type(o3d.geometry.PointCloud())):
            self.pcl = pcl
        else:
            self.pcl = o3d.geometry.PointCloud()
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
            self.cm = self.calc_cm(np.asarray(self.pcl.points)[:, :2])
        print("Slice initialized.")

    def calc_cm(self, pts):
        print("Calculating center of mass for points.")
        return np.mean(pts, axis=0) if len(pts) > 0 else None

def read_data(models_dir, section, sc, scm, uwb):
    stl_path = os.path.join(models_dir, f'{section}-75.STL')
    print(f"Reading STL file from {stl_path}...")

    try:
        bmesh = o3d.io.read_triangle_mesh(stl_path)
        print("STL file successfully read.")
    except Exception as e:
        print(f"Error reading STL file: {e}")
        raise

    try:
        print("Scaling the mesh...")
        bmesh.scale(1 / 1000, center=(0, 0, 0))
        print("Scaling successful.")
        
        print("Translating the mesh...")
        bmesh.translate((0, 0, 0))  # Adjust if needed
        print("Translation successful.")
        
        print("Computing vertex normals...")
        bmesh.compute_vertex_normals()
        print("Vertex normals computed.")
    except Exception as e:
        print(f"Error during mesh processing: {e}")
        raise

    try:
        print("Sampling points from the mesh...")
        bpcl = bmesh.sample_points_uniformly(1000000)
        print(f"Sampling successful. Sampled {len(bpcl.points)} points.")
        
        print("Estimating normals for the point cloud...")
        bpcl.estimate_normals()
        print("Normals estimation successful.")
        
        print("Painting the point cloud...")
        bpcl.paint_uniform_color([0, 0.651, 0.929])
        print("Painting successful.")
    except Exception as e:
        print(f"Error during point cloud processing: {e}")
        raise

    return [sc, scm, uwb], bmesh

def rot2z(scm, uwb, ts):
    print(f"Matching rotation count to Z estimate for timestamp {ts}...")
    scan_ind = (np.abs(scm[:,0] - ts)).argmin()
    diff = np.abs(uwb[:,0] - ts)
    z = uwb[np.where(diff == diff.min())[0][0], -1]
    print(f"Z estimate matched: {z}")
    return z, scan_ind

def pol2cart(pol_pts, A_MIN, A_INC, z):
    print("Converting polar coordinates to Cartesian...")
    scan_xy = []
    for i, pt in enumerate(pol_pts):
        if np.isinf(pt):
            continue
        theta = A_MIN + i * A_INC
        scan_xy.append([pt * np.cos(theta), pt * np.sin(theta), z])
    return scan_xy

def rot2slice(run_data, ts, z_off, dz, n):
    print("Building the scan slice...")
    sc, scm, uwb = run_data
    z, ind = rot2z(scm, uwb, ts)
    z += z_off
    z_vec = np.linspace(z - dz * n, z + dz * n, n * 2 + 1)
    z_bounds = (z_vec[0], z_vec[-1])
    scans_pol = sc[ind - n: ind + n + 1]
    scans_xyz = []
    for i, _z in enumerate(z_vec):
        scans_xyz.append(pol2cart(scans_pol[i], scm[ind, 3], scm[ind, 5], _z))
    print("Scan slice built.")
    return Slice(np.concatenate(scans_xyz), z, z_bounds, ind, pcl_type='scan')

def register(source, target, tf_init):
    print("Running ICP registration...")
    reg = o3d.pipelines.registration.registration_icp(source, target, threshold, tf_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint())
    print(f"ICP registration completed with fitness: {reg.fitness} and RMSE: {reg.inlier_rmse}")
    return reg.transformation, reg.fitness, reg.inlier_rmse

def slice_mdl(bmesh, scan_slice, search_ratio, n_samples):
    print("Cropping the model slice...")
    z = scan_slice.z
    n = int(search_ratio * scan_slice.n_pts)
    pts = []
    z_min = z + search_ratio * (scan_slice.z_bounds[0] - z)
    z_max = z + search_ratio * (scan_slice.z_bounds[1] - z)

    bpcl = bmesh.sample_points_uniformly(n_samples)
    for pt in np.asarray(bpcl.points):
        if z_min <= pt[2] <= z_max:
            pts.append(pt)
    
    print(f"Model slice cropped. Points cropped: {len(pts)}")
    return Slice(pts, z, (z_min, z_max))

def slide_icp(scan_slice, model_slice, vis=False):
    print("Starting ICP registration process...")
    source = scan_slice.pcl
    target = model_slice.pcl
    cm = [model_slice.cm[0] - scan_slice.cm[0], model_slice.cm[1] - scan_slice.cm[1]]
    z_min, z_max = model_slice.z_bounds
    z = scan_slice.z
    z_uwb = z - z_off
    z_init_vec = np.linspace(z_min, z_max, int((z_max - z_min) / slide_res))
    results = []
    print(f"ICP registration will run on {len(z_init_vec)} initial values.")
    
    for z_init in z_init_vec:
        print(f"Running ICP for z_init = {z_init}...")
        try:
            tf, fitness, rmse = register(scan_slice.pcl, model_slice.pcl, trans_init(section, cm, z_init - z))
            print(f"ICP completed for z_init = {z_init} with fitness {fitness} and rmse {rmse}.")
            
            if fitness > 0.0:
                results.append([tf, fitness, rmse, z_init])
            else:
                print(f"ICP did not converge for z_init: {z_init}")
        except Exception as e:
            print(f"Error during ICP at z_init {z_init}: {e}")
            raise

    if results:
        results = sorted(results, key=lambda x: x[2])
        pos = np.matmul(results[0][0], np.array([0, 0, scan_slice.z, 1]).T)
        out_rot = np.matmul(results[0][0][:3, :3], np.array([0, 0, 1]).T)
        out_rot = out_rot / np.linalg.norm(out_rot)
        print(f"Best result found with rmse = {results[0][2]}, z_init = {results[0][3]}")
        return True, (pos, results[0][0], z_uwb, results[0][1], results[0][2])
    else:
        print("No successful ICP results.")
        return False, None

def visualize(source, target, tf=np.eye(4)):
    print("Visualizing source and target point clouds...")
    s = deepcopy(source)
    t = deepcopy(target)
    s.transform(tf)
    ref = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1)
    o3d.visualization.draw_geometries([ref, s, t],
                                      zoom=0.065,
                                      front=[0.42, 0.7, -0.56],
                                      lookat=[0.05, -0.33, 16.5],
                                      up=[0.9, -0.23, 0.37],
                                      width=3140, height=1920)

def estimate_poses_icp(ts_list, models_dir, section, sc, scm, uwb):
    print("Starting ICP estimation...")
    run_data, bmesh = read_data(models_dir, section, sc, scm, uwb)
    ts_poses_dict = collections.defaultdict(dict)
    time_scan_slice = 0
    time_model_slice = 0
    time_icp = 0
    for ts_row in ts_list:
        z, frame, ts = ts_row
        ts = ts * (10**9)
        try:
            scan_slice_start = time.time()
            scan_slice = rot2slice(run_data, ts, z_off, dz, n_scans)
            time_scan_slice += time.time() - scan_slice_start

            model_slice_start = time.time()
            model_slice = slice_mdl(bmesh, scan_slice, search_ratio, n_samples)
            time_model_slice += time.time() - model_slice_start

            icp_start = time.time()
            success, res = slide_icp(scan_slice, model_slice, vis=False)
            time_icp += time.time() - icp_start
            
            ts_poses_dict[int(frame)]['z'] = int(z)
            ts_poses_dict[int(frame)]['timestamp'] = int(ts)
            ts_poses_dict[int(frame)]['position'] = res[0].tolist()
            ts_poses_dict[int(frame)]['transform'] = res[1].tolist()
        except Exception as e:
            ts_poses_dict[int(frame)]['z'] = int(z)
            ts_poses_dict[int(frame)]['timestamp'] = int(ts)
            ts_poses_dict[int(frame)]['position'] = ""
            ts_poses_dict[int(frame)]['transform'] = ""
            print(f"Error during ICP pose estimation: {e}")
    print(f'Scan Slice Time: {time_scan_slice}')
    print(f'Model Slice Time: {time_model_slice}')
    print(f'ICP Time: {time_icp}')
    return ts_poses_dict

def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

def empty_poses_entry():
    return {}

if __name__ == '__main__':
    print("Starting the script...")

    # Argument parsing
    print("Parsing arguments...")
    parser = argparse.ArgumentParser()
    parser.add_argument('--section', type=str, required=False)
    parser.add_argument('--frame_list_path', type=str, required=False)
    parser.add_argument('--json_out_dir', type=str, required=False)
    parser.add_argument('--bag_dir', type=str, required=False)
    parser.add_argument('--video_ident', type=str, required=False)
    parser.add_argument('--models_dir', type=str, required=False)
    parser.add_argument('--enable_icp', type=str, required=False, default="False")
    args = parser.parse_args()

    print("Initializing variables from arguments...")
    section = args.section
    frame_list_path = args.frame_list_path
    json_out_dir = args.json_out_dir
    bag_dir = args.bag_dir
    video_ident = args.video_ident
    models_dir = args.models_dir
    enable_icp = str2bool(args.enable_icp)

    # Load timestamp list
    print(f"Loading frame list from {frame_list_path}...")
    try:
        frame_list = np.genfromtxt(frame_list_path, delimiter=',', dtype=None, skip_header=1)
        print(f"Loaded {len(frame_list)} frames.")
    except Exception as e:
        print(f"Error loading frame list: {e}")
        raise

    if enable_icp:
        print(f"Bag directory: {bag_dir}")
        list_of_bags = os.listdir(bag_dir)
        print(f"Files in bag directory: {list_of_bags}")

        assert len(list_of_bags) == 1, f"Expected 1 bag file, but found {len(list_of_bags)}"
        selected_bag_file = os.path.join(bag_dir, list_of_bags[0])
        print(f"Selected bag file: {selected_bag_file}")

        try:
            uwb, rot_raw, scm, sc = read_bag(selected_bag_file)
            print(f"UWB raw data: {uwb[:10] if uwb.size else 'No data'}")
            print(f"Rotation raw data: {rot_raw[:10] if rot_raw.size else 'No data'}")
            print(f"Scan metadata: {scm[:10] if scm.size else 'No data'}")
            print(f"Scan ranges: {sc[:10] if sc.size else 'No data'}")
        except Exception as e:
            print(f"Error occurred while reading the bag file: {e}")
            raise

        # Call ICP
        print("Calling ICP estimation...")
        try:
            ts_poses_dict = estimate_poses_icp(frame_list, models_dir, section, sc, scm, uwb)
            print("ICP estimation completed.")
        except Exception as e:
            print(f"Error during ICP estimation: {e}")
            raise

    else:
        ts_poses_dict = defaultdict(empty_poses_entry)

    print("Script finished.")
