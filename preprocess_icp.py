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
import matplotlib
import matplotlib.pyplot as plt
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import numpy as np
import os
from scipy.signal import argrelextrema, find_peaks
import argparse

import time

from collections import defaultdict
from importlib.metadata import version, PackageNotFoundError


# Print versions of the imported libraries
print(f"matplotlib version: {matplotlib.__version__}")
print(f"numpy version: {np.__version__}")
print(f"open3d version: {o3d.__version__}")
print(f"pathlib version: {Path.__module__}")
# print(f"collections version: {collections.__version__}")
print(f"argparse version: {argparse.__version__}")
print(f"json version: {json.__version__}")
# print(f"rosbags version: {rosbags.__version__}")



try:
    rosbags_version = version('rosbags')
except PackageNotFoundError:
    rosbags_version = 'not installed'
print(f"rosbags version: {rosbags_version}")

# print(f"scipy version: {signal.__version__}")
print(f"os module: {os.__name__}")
print(f"time module: {time.__name__}")


IMG_EXTENSION = ".png"

#ex bounds: lead 486-4602, web: 5968-9838, trail: 11046-12306
ROT_INPUT =  5880 #11500 #1500 #7500 
section = 'lead'
orientation = 0
dz = 0.14 / 10 # speed in m/s over scan rate
n_scans = 1 #scans to populate in each direction (fw/back)
search_ratio = 30.0 # 20.0
n_samples = 2000000
z_off = 0.25 #approximate distance from the ranging sensor to the lidar
slide_res = 0.025
n_icp_iters = 50
threshold = 2
cm_depth = 0.05
# path = Path(__file__).parent

def trans_init(section='None', cm=[0,0], z=0):
    ref = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 1)
    rot = [0,0,0]
    trans = [0,0,z]
    tf = np.eye(4)
    if orientation == 0:
        if section == 'lead':
            rot = [0,0,np.radians(-160)]
        elif section == 'web':
            rot = [0,0,np.radians(160)]
        elif section == 'trail':
            rot = [0,0,np.radians(145)]
    else:
        if section == 'lead':
            rot = [0,0,np.radians(-35)]
        elif section == 'web':
            rot = [0,0,np.radians(0)]
        elif section == 'trail':
            rot = [0,0,np.radians(0)]

    trans = [cm[0],cm[1], z] 
    tf[:3, :3] = ref.get_rotation_matrix_from_xyz(rot)
    tf[:3, 3] = trans
    return tf

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

class Slice:
    def __init__(self, pcl, z, z_bounds, scan_index=None, pcl_type='model'):
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

        self.cm = self.calc_cm(np.asarray(self.pcl.points)[:,:2])
        if self.type == 'scan':
            self.pcl.paint_uniform_color([1, 0.706, 0])
            cm_pcl = deepcopy(self.pcl)
            cm_pcl.transform(trans_init(section))
            self.cm = self.calc_cm(np.asarray(cm_pcl.points)[:,:2])
        elif self.type == 'model':
            self.pcl.paint_uniform_color([0, 0.651, 0.929])
            self.cm = self.calc_cm(np.asarray(self.pcl.points)[:,:2])

    def calc_cm(self, pts):
        if len(pts) >= 0 :
            return np.mean(pts, axis=0)
        else:
            return None
        


def read_data(models_dir, section, sc, scm, uwb):
    
    stl_path = os.path.join(models_dir, f'{section}-75.STL')    
    print(f'Reading {section} STL from path {stl_path}...')
    bmesh = o3d.io.read_triangle_mesh(stl_path)
    bmesh.scale(1/1000, center=(0,0,0))
    bmesh.translate((0,0,0)) #+1.86z to z align ASW
    bmesh.compute_vertex_normals()

    # Sample 2,000,000 points from the mesh (added by H.)
    bpcl = bmesh.sample_points_uniformly(n_samples)

    # Pre-slice the points into 1000 lists based on their Z values (added by H.)
    z_slices = defaultdict(list)
    z_min_total = np.min(np.asarray(bpcl.points)[:, 2])
    z_max_total = np.max(np.asarray(bpcl.points)[:, 2])
    z_step = (z_max_total - z_min_total) / 1000.0

    # Sort points into slices based on their Z value (added by H.)
    for pt in np.asarray(bpcl.points):
        z_index = int((pt[2] - z_min_total) // z_step)
        z_slices[z_index].append(pt)

    # ref = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 1)
    # bpcl = bmesh.sample_points_uniformly(1000000)
    # bpcl.estimate_normals()
    # bpcl.paint_uniform_color([0, 0.651, 0.929]) #blue, 75
    # o3d.visualization.draw_geometries([ref, bpcl])
    return [sc, scm, uwb], bmesh, z_slices, z_min_total, z_max_total

def rot2z(scm, uwb, ts):
    scan_ind = (np.abs(scm[:,0] - ts)).argmin()
    # scan_ind = np.where(np.abs(scm[:,0] - ts) < 10000)[0][0]
    diff = np.abs(uwb[:,0] - ts)
    z = uwb[np.where(diff == diff.min())[0][0], -1]
    return z, scan_ind

def pol2cart(pol_pts, A_MIN, A_INC, z):
    scan_xy = []
    for i, pt in enumerate(pol_pts):

        if np.isinf(pt):
            continue
        # if i<= 50 or i >= 300:
        #     continue
        theta = A_MIN + i*A_INC
        scan_xy.append([pt * np.cos(theta), pt * np.sin(theta), z])

    return scan_xy

def rot2slice(run_data, ts, z_off, dz, n): 
    sc, scm, uwb = run_data
    
    #match rotation to z estimate and offset
    print('Matching rotation count to Z estimate...')
    z, ind = rot2z(scm, uwb, ts)
    z += z_off

    #build scan pointcloud with z, speed estimate
    print('Building the scan slice...')
    z_vec = np.linspace(z - dz*n, z + dz*n, n*2 + 1)
    z_bounds = (z_vec[0], z_vec[-1])
    scans_pol = sc[ind-n:ind+n+1]
    scans_xyz = []
    for i, _z in enumerate(z_vec):
        scans_xyz.append(pol2cart(scans_pol[i], scm[ind,3], scm[ind,5], _z))

    return Slice(np.concatenate(scans_xyz), z, z_bounds, ind, pcl_type='scan')

def register(source, target, tf_init):

    reg = o3d.pipelines.registration.registration_icp(source, target, threshold, tf_init,
    o3d.pipelines.registration.TransformationEstimationPointToPoint())

    return reg.transformation, reg.fitness, reg.inlier_rmse

def slice_mdl(bmesh, scan_slice, search_ratio, z_slices, z_min_total, z_max_total):
    print('Cropping the model slice...')
    scan_slice_internal_start = time.time()      

    z = scan_slice.z
    n = int(search_ratio * scan_slice.n_pts)
    pts = []

    # Define z_min and z_max for the current slice
    z_min = z + search_ratio * (scan_slice.z_bounds[0] - z)
    z_max = z + search_ratio * (scan_slice.z_bounds[1] - z)
    
    z_step = (z_max_total - z_min_total) / 1000.0   # (added by H.)
    z_min_index = int((z_min - z_min_total) // z_step)
    z_max_index = int((z_max - z_min_total) // z_step)

    # Loop through relevant slices and add points (added by H.)
    for z_index in range(z_min_index, z_max_index + 1):
        # If the current slice is fully within z_min and z_max, append all points directly
        if z_index > z_min_index and z_index < z_max_index:
            pts.extend(z_slices[z_index])
        else:
            # For the boundary slices, check individual points
            for pt in z_slices[z_index]:
                if z_min <= pt[2] <= z_max:
                    pts.append(pt)


    scan_slice_duration = time.time() - scan_slice_internal_start
    print ("scan_slice_duration = ", scan_slice_duration)
    
    return Slice(pts, z, (z_min, z_max))



def slide_icp(scan_slice, model_slice, vis=False):
    source = scan_slice.pcl
    target = model_slice.pcl
    cm = [model_slice.cm[0] - scan_slice.cm[0], model_slice.cm[1] - scan_slice.cm[1]]
    z_min, z_max = model_slice.z_bounds
    # if vis: 
    #     print('Raw, uninitiated visualization')
    #     visualize(source, target)
    
    z = scan_slice.z
    z_uwb = z - z_off
    z_init_vec = np.linspace(z_min,z_max, int((z_max-z_min)/slide_res))
    results = []

    for z_init in z_init_vec:
        _z = z_init-z 
        # print(f'Initialization for z = {z_init}')
        # visualize(source, target, trans_init(section, cm, _z))

        tf, fitness, rmse = register(scan_slice.pcl, model_slice.pcl, trans_init(section, cm, _z))
        
        #print(f'Registration for z = {z_init}')
        #visualize(source, target, tf)
        if fitness > 0.0:
            #print(f'z = {z_init} registered with fitness {fitness} and rmse = {rmse}')        
            results.append([tf, fitness, rmse, z_init])
        else:
            print(f'Did not converge for z_init: {z_init}')
                
    if results:
        results = sorted(results, key = lambda x: x[2])
        pos = np.matmul(results[0][0],np.array([0, 0, scan_slice.z, 1]).T)
        out_rot = np.matmul(results[0][0][:3,:3],np.array([0, 0, 1]).T)
        out_rot = out_rot / np.linalg.norm(out_rot)
        print(f'Best result with rmse = {results[0][2]}')
        print(f'Estimated z of {pos[2]}m at z_init: {results[0][3]}m and z uwb: {z_uwb}m')
        # out_pt[0] = 0.4
        # print(np.linalg.norm(out_pt[:3]))
        if vis:
            rmse_out = [x[2] for x in results]
            z_out = [x[3] for x in results]
            visualize(source, target, results[0][0])
            plt.plot(z_out, rmse_out, marker='.',linestyle='None')
            plt.title(f'Z Initial Registration and ICP RMSE: Z_UWB of {z_uwb}m')
            plt.xlabel('z (m)')
            plt.ylabel('Error')
            plt.show()
        return True, (pos, results[0][0], z_uwb, fitness, rmse)
    else:
        return False, None

def visualize(source, target, tf=np.eye(4)):
    s = deepcopy(source)
    t = deepcopy(target)
    s.transform(tf)
    ref = o3d.geometry.TriangleMesh.create_coordinate_frame(size = 1)
    o3d.visualization.draw_geometries([ref, s, t],
                                zoom = 0.065,
                                front = [0.42, 0.7, -0.56],
                                lookat = [0.05, -0.33, 16.5],
                                up = [0.9, -0.23, 0.37],
                                width = 3140, height = 1920)

def estimate_poses_icp(ts_list, models_dir, section, sc, scm, uwb):
    run_data, bmesh ,z_slices, z_min_total, z_max_total = read_data(models_dir, section, sc, scm, uwb)
    ts_poses_dict = collections.defaultdict(dict)
    time_scan_slice = 0
    time_model_slice = 0
    time_icp = 0
    for ts_row in ts_list:
        z, frame, ts = ts_row
        ts = ts * (10**9)
        try:
            scan_slice_start = time.time()
            scan_slice =  rot2slice(run_data, ts, z_off, dz, n_scans)
            time_scan_slice += time.time() - scan_slice_start

            model_slice_start = time.time()
            model_slice = slice_mdl(bmesh, scan_slice, search_ratio, z_slices, z_min_total, z_max_total)
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
            print(e)
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
    parser = argparse.ArgumentParser()
    parser.add_argument('--section', type=str, required=False)
    parser.add_argument('--frame_list_path', type=str, required=False)
    parser.add_argument('--json_out_dir', type=str, required=False)
    parser.add_argument('--bag_dir', type=str, required=False)
    parser.add_argument('--video_ident', type=str, required=False)
    parser.add_argument('--models_dir', type=str, required=False)
    parser.add_argument('--enable_icp', type=str, required=False, default="False" )

    args = parser.parse_args()
    section = args.section
    frame_list_path = args.frame_list_path
    json_out_dir = args.json_out_dir
    bag_dir = args.bag_dir
    video_ident = args.video_ident
    models_dir = args.models_dir
    enable_icp = str2bool(args.enable_icp)


    # Load timestamp list
    frame_list = np.genfromtxt(frame_list_path, delimiter=',',dtype=None, skip_header=1)
    
    if enable_icp:
        list_of_bags = os.listdir(bag_dir)
        assert(len(list_of_bags) == 1)
        uwb, rot_raw, scm, sc = read_bag(os.path.join(bag_dir, list_of_bags[0]))
        

        # get the augmented pose list
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
        
        # Path.joinpath(Path(json_out_dir), Path(f'{section}.json')).write_text(json.dumps(ts_poses_dict, indent=4) + '\n')