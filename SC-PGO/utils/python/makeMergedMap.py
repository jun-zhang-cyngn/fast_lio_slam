import os 
import sys
import time 
import copy 
from io import StringIO
from scipy.spatial.transform import Rotation as R

import pypcd # for the install, use this command: python3.x (use your python ver) -m pip install --user git+https://github.com/DanielPollithy/pypcd.git
from pypcd import pypcd

import numpy as np
from numpy import linalg as LA

import open3d as o3d

from pypcdMyUtils import * 

jet_table = np.load('jet_table.npy')
bone_table = np.load('bone_table.npy')

color_table = jet_table
color_table_len = color_table.shape[0]


##########################
# User only consider this block
##########################

data_dir = "/home/ubuntu/github/catkin_fastlio/data/2023-07-25-21-05-17_0814_surf_0.1/" # should end with / 
node_skip = 1

num_points_in_a_scan = 150000 # for reservation (save faster) // e.g., use 150000 for 128 ray lidars, 100000 for 64 ray lidars, 30000 for 16 ray lidars, if error occured, use the larger value.

is_live_vis = False # recommend to use false 
is_o3d_vis = False
intensity_color_max = 200

is_near_removal = True
thres_near_removal = 1 # meter (to remove platform-myself structure ghost points)

##########################


def load_json_hba_json_pose(data_dir):
    f = open(data_dir+ "/pose.json", 'r')
    poses = []
    while True:
        line = f.readline()
        if not line: break
        tx, ty, tz, qw, qx, qy, qz = [float(i) for i in line.split()]
        pose_SE3 = np.identity(4)
        # import pdb; pdb.set_trace()
        pose_SE3[0:3,0:3] = R.from_quat([qx, qy, qz, qw]).as_matrix()
        pose_SE3[:, -1][0] = tx
        pose_SE3[:, -1][1] = ty
        pose_SE3[:, -1][2] = tz
        poses.append(pose_SE3)
        # import pdb; pdb.set_trace()
    f.close()
    return poses


def load_fasfio_slam_pose(data_dir, fname):
    f = open(data_dir+ "/" + fname, 'r')
    poses = []
    while True:
        line = f.readline()
        if not line: break
        pose_SE3 = np.asarray([float(i) for i in line.split()])
        pose_SE3 = np.vstack( (np.reshape(pose_SE3, (3, 4)), np.asarray([0,0,0,1])) )
        poses.append(pose_SE3)
    f.close()
    return poses

def covert_fastfio2_to_hba_poses(fastfio_poses, json_pose_path):
    poses_vector7 = []
    for pose_SE3 in fastfio_poses:
        dcm_so3 = pose_SE3[0:3,0:3]
        tx, ty, tz = pose_SE3[:, -1][0:3].tolist()
        scipy_r = R.from_matrix(dcm_so3)
        qx, qy, qz, qw = scipy_r.as_quat() # x, y, z, w
        poses_vector7.append(np.array([tx, ty, tz, qw, qx, qy, qz]))
    poses_vector7 = np.array(poses_vector7)
    np.savetxt(json_pose_path, poses_vector7, delimiter=" ")
    return poses_vector7

scan_dir = data_dir + "/pcd"
scan_files = os.listdir(scan_dir) 
scan_files.sort()
scan_idx_range_to_stack = [0, len(scan_files)] # if you want a whole map, use [0, len(scan_files)]

# poses = load_json_hba_json_pose(data_dir)

poses = load_fasfio_slam_pose(data_dir, 'odom_poses.txt')
covert_fastfio2_to_hba_poses(poses, data_dir + '/odom_pose.json')
poses = load_fasfio_slam_pose(data_dir, 'optimized_poses.txt')
covert_fastfio2_to_hba_poses(poses, data_dir + '/optimized_poses.json')
poses = load_fasfio_slam_pose(data_dir, 'optimized_poses_gnc.txt')
covert_fastfio2_to_hba_poses(poses, data_dir + '/optimized_poses_gnc.json')

import pdb; pdb.set_trace()

assert (scan_idx_range_to_stack[1] > scan_idx_range_to_stack[0])
print("Merging scans from", scan_idx_range_to_stack[0], "to", scan_idx_range_to_stack[1])


if(is_live_vis):
    vis = o3d.visualization.Visualizer() 
    vis.create_window('Map', visible = True) 

nodes_count = 0
pcd_combined_for_vis = o3d.geometry.PointCloud()
pcd_combined_for_save = None

# The scans from 000000.pcd should be prepared if it is not used (because below code indexing is designed in a naive way)

# manually reserve memory for fast write  
num_all_points_expected = int(num_points_in_a_scan * np.round((scan_idx_range_to_stack[1] - scan_idx_range_to_stack[0])/node_skip))

np_xyz_all = np.empty([num_all_points_expected, 3])
np_intensity_all = np.empty([num_all_points_expected, 1])
curr_count = 0

for node_idx in range(len(scan_files)):
    if(node_idx < scan_idx_range_to_stack[0] or node_idx >= scan_idx_range_to_stack[1]):
        continue

    nodes_count = nodes_count + 1
    if( nodes_count % node_skip is not 0): 
        if(node_idx is not scan_idx_range_to_stack[0]): # to ensure the vis init 
            continue

    print("read keyframe scan idx", node_idx)

    scan_pose = poses[node_idx]

    scan_path = os.path.join(scan_dir, scan_files[node_idx])
    # import pdb; pdb.set_trace()
    scan_pcd = o3d.io.read_point_cloud(scan_path)
    scan_xyz_local = copy.deepcopy(np.asarray(scan_pcd.points))
    # import pdb; pdb.set_trace()
    scan_pypcd_with_intensity = pypcd.PointCloud.from_path(scan_path)
    scan_intensity = scan_pypcd_with_intensity.pc_data['intensity']
    scan_intensity_colors_idx = np.round( (color_table_len-1) * np.minimum( 1, np.maximum(0, scan_intensity / intensity_color_max) ) )
    scan_intensity_colors = color_table[scan_intensity_colors_idx.astype(int)]

    scan_pcd_global = scan_pcd.transform(scan_pose) # global coord, note that this is not deepcopy
    scan_pcd_global.colors = o3d.utility.Vector3dVector(scan_intensity_colors)
    scan_xyz = np.asarray(scan_pcd_global.points)

    scan_intensity = np.expand_dims(scan_intensity, axis=1) 
    scan_ranges = LA.norm(scan_xyz_local, axis=1)

    if(is_near_removal):
        eff_idxes = np.where (scan_ranges > thres_near_removal)
        scan_xyz = scan_xyz[eff_idxes[0], :]
        scan_intensity = scan_intensity[eff_idxes[0], :]

        scan_pcd_global = scan_pcd_global.select_by_index(eff_idxes[0])

    if(is_o3d_vis):
        pcd_combined_for_vis += scan_pcd_global # open3d pointcloud class append is fast 

    if is_live_vis:
        if(node_idx is scan_idx_range_to_stack[0]): # to ensure the vis init 
            vis.add_geometry(pcd_combined_for_vis) 

        vis.update_geometry(pcd_combined_for_vis)
        vis.poll_events()
        vis.update_renderer()

    # save 
    np_xyz_all[curr_count:curr_count + scan_xyz.shape[0], :] = scan_xyz
    np_intensity_all[curr_count:curr_count + scan_xyz.shape[0], :] = scan_intensity

    curr_count = curr_count + scan_xyz.shape[0]
    print(curr_count)
 
#
if(is_o3d_vis):
    print("draw the merged map.")
    o3d.visualization.draw_geometries([pcd_combined_for_vis])


# save ply having intensity
np_xyz_all = np_xyz_all[0:curr_count, :]
np_intensity_all = np_intensity_all[0:curr_count, :]

np_xyzi_all = np.hstack( (np_xyz_all, np_intensity_all) )
xyzi = make_xyzi_point_cloud(np_xyzi_all)

map_name = data_dir + "map_" + str(scan_idx_range_to_stack[0]) + "_to_" + str(scan_idx_range_to_stack[1]) + "_with_intensity.pcd"
xyzi.save_pcd(map_name, compression='binary_compressed')
print("intensity map is save (path:", map_name, ")")

# save rgb colored points 
map_name = data_dir + "map_" + str(scan_idx_range_to_stack[0]) + "_to_" + str(scan_idx_range_to_stack[1]) + ".pcd"
o3d.io.write_point_cloud(map_name, pcd_combined_for_vis)
print("the map is save (path:", map_name, ")")


