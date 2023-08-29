# -*-coding:utf-8-*-
import numpy as np
from scipy.spatial.transform import Rotation
import rosbag
import sensor_msgs.point_cloud2 as pc2
import argparse
import open3d as o3d
import glob
import os
from tqdm import tqdm

def mergemap(args, base_to_lidar, bag_file):
    groud_truth_path = bag_file.split(".")[0] + '_gt.txt'

    bag = rosbag.Bag(bag_file, "r")
    bag_data = bag.read_messages(args.lidartopic)

    sampled_pcs = []
    lidar_times = []
    count = 0
    for topic, msg, t in bag_data:
        if count % 10 == 0:
            time = float('%6f' % (msg.header.stamp.to_sec()))
            lidar = pc2.read_points(msg, skip_nans=True, field_names=('x', 'y', 'z', 'intensity'))
            points = np.array(list(lidar))
            sampled_pcs.append(points)
            lidar_times.append(time)
        count += 1

    # load groudtruth
    gt_times, gt_poses = load_poses(groud_truth_path)
    sample_gt_poses = []
    for t in lidar_times:
        index = np.abs(np.array(gt_times) - t).argmin()
        sample_gt_poses.append(gt_poses[index])

    # merge points in World cood
    num_all_points_expected = int(args.num_points_in_a_scan *
                                  np.round((args.scan_idx_range_to_stack[1] - args.scan_idx_range_to_stack[
                                      0]) / args.node_skip))
    np_xyz_all = np.empty([num_all_points_expected, 3])
    np_intensity_all = np.empty([num_all_points_expected, 1])
    curr_count = 0
    sampled_gt_pose = []

    for idx, points in enumerate(sampled_pcs):
        temp_gt_pose = sample_gt_poses[idx]
        sampled_gt_pose.append(temp_gt_pose)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        pcd_bs = pcd.transform(base_to_lidar)
        pcd_global = pcd_bs.transform(temp_gt_pose)
        xyz_global = np.asarray(pcd_global.points)
        scan_intensity = np.asarray(points[:, 3]).reshape(-1, 1)

        np_xyz_all[curr_count:curr_count + xyz_global.shape[0], :] = xyz_global
        np_intensity_all[curr_count:curr_count + xyz_global.shape[0], :] = scan_intensity
        curr_count = curr_count + xyz_global.shape[0]

    # down sample and save pcd
    np_xyz_all = np_xyz_all[0:curr_count, :]
    pcd_all = o3d.geometry.PointCloud()
    pcd_all.points = o3d.utility.Vector3dVector(np_xyz_all)
    pcd_downsampled = pcd_all.voxel_down_sample(voxel_size=args.down_voxel_size)

    return pcd_downsampled, sampled_gt_pose

def load_poses(poses_path):
    poses = []
    gt_times =[]
    with open(poses_path, 'r') as f:
        lines = f.readlines()
        for i, line in enumerate(lines):
            if i ==0:
                continue
            time = np.fromstring(line, sep=' ')[0]
            temp = np.fromstring(line, dtype=np.float32, sep=' ')
            xyz = temp[1:4].reshape(3, 1)
            quaternion = temp[4:8]
            r = Rotation.from_quat(quaternion)
            rotation_matrix = r.as_matrix()
            pose = np.hstack((rotation_matrix, xyz))
            pose = np.vstack((pose, [0, 0, 0, 1]))
            poses.append(pose)
            gt_times.append(time)
    return gt_times, poses


def read_calib(calib_path):
    with open(calib_path, 'r') as f:
        lines = f.readlines()
        for i, line in enumerate(lines):
            if i==0:
                xyz = np.fromstring(line, sep=' ').reshape(3, 1)
            if i==1:
                quaternion = np.fromstring(line, sep=' ')
                r = Rotation.from_quat(quaternion)
                rotation_matrix = r.as_matrix()
        calib = np.hstack((rotation_matrix, xyz))
        calib = np.vstack((calib, [0, 0, 0, 1]))
    return calib


def load_trans(trans_path):
    with open(trans_path, 'r') as f:
        lines = f.readlines()
        pose = []
        for i, line in enumerate(lines):
            if i==0:
                db_id = int(((line.split(' ')[0]).split('.')[0]).split(':')[-1])
                query_id = int(((line.split(' ')[1]).split('.')[0]).split(':')[-1])
            else:
                pose_i = np.fromstring(line, dtype=np.float32, sep=' ')
                pose.append(pose_i)
        pose = np.asarray(pose)
    return db_id, query_id, pose


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='turn list of rosbags into ColoRadar dasatet format')
    parser.add_argument('-datapath', '--datapath', type=str, default='/media/cyw/CYW-ZX2/coloradar/data/',
                        help='bag files path')
    parser.add_argument('--savepath', type=str, default='/media/cyw/CYW-ZX2/coloradar/mergemap/',
                        help='merged pcd path')
    parser.add_argument('--lidartopic', type=str, default='/os1_cloud_node/points',
                        help='lidartopic')
    parser.add_argument('--bs_to_lidar_path', type=str,
                        default='/media/cyw/CYW-ZX2/coloradar/calib/transforms/base_to_lidar.txt')
    parser.add_argument('--db_to_query_path', type=str,
                        default='/media/cyw/CYW-ZX2/coloradar/transform/')
    parser.add_argument('--data_tra_name', type=str, default='edgar_classroom_run')
    parser.add_argument('--num_points_in_a_scan', type=int, default=150000)
    parser.add_argument('--scan_idx_range_to_stack', type=list, default=[0, 200])
    parser.add_argument('--node_skip', type=int, default=1)
    parser.add_argument('--down_voxel_size', type=float, default=0.5)
    args = parser.parse_args()

    # load calib
    base_to_lidar = read_calib(args.bs_to_lidar_path)

    if not os.path.exists(args.savepath):
        os.makedirs(args.savepath)

    database_gt = []
    query_gt = []

    # Get all .bag files from datapath
    bag_files = glob.glob(args.datapath + "*.bag")
    for bag_file in tqdm(bag_files):
        # merge pointclouds
        seq_name = (bag_file.split("/")[-1]).split(".")[0]
        pcd_downsampled, sampled_gt_pose = mergemap(args, base_to_lidar, bag_file)
        # Save sampled_gt_pose to txt file
        with open(args.savepath + seq_name + 'sampled_gt.txt', 'w') as f:
            for pose in sampled_gt_pose:
                pose_str = ' '.join(str(x) for x in pose.flatten())
                f.write(pose_str + '\n')

        # save merged pointcloud
        save_pcd_path = args.savepath + seq_name + ".pcd"
        o3d.io.write_point_cloud(save_pcd_path, pcd_downsampled)

        # Wb_2_Wq
        if "0" in seq_name:
            database_gt = sampled_gt_pose
            continue
        query_gt = sampled_gt_pose
        trans_path = args.db_to_query_path + seq_name + '_trans.txt'
        db_id, query_id, db_to_query = load_trans(trans_path)

        W_database = database_gt[db_id]
        W_query = query_gt[query_id]
        W_query_inv = np.linalg.inv(query_gt[query_id])
        lidar_to_base = np.linalg.inv(base_to_lidar)

        Wb_2_Wq = database_gt[db_id] @ base_to_lidar @ db_to_query @ np.linalg.inv(base_to_lidar) @ np.linalg.inv(query_gt[query_id])
        pcd_in_Wb = pcd_downsampled.transform(Wb_2_Wq)

        # save transformed query pcd
        save_pcd_Ws_path = args.savepath + seq_name + "_Ws.pcd"
        o3d.io.write_point_cloud(save_pcd_Ws_path, pcd_in_Wb)












