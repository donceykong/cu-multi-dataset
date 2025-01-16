#!/usr/bin/env python3

# External
import open3d as o3d
import numpy as np
import os
import re
from tqdm import tqdm
from scipy.spatial.transform import Rotation as R, Slerp

# Internal
from utils.projection import *

def quaternion_pose_to_4x4(trans, quat):
    rotation_matrix = R.from_quat(quat).as_matrix()

    # Create the 4x4 transformation matrix
    transformation_matrix = np.eye(4)  # Start with an identity matrix
    transformation_matrix[:3, :3] = rotation_matrix  # Set the rotation part
    transformation_matrix[:3, 3] = trans  # Set the translation part

    return transformation_matrix


def pose4x4_to_quat(matrix):
    position = matrix[:3, 3]
    rotation_matrix = matrix[:3, :3]
    quaternion = R.from_matrix(rotation_matrix).as_quat()
    return position, quaternion


def read_quat_poses(poses_path, pose_ts_path):
    # Read timestamps into a list
    with open(pose_ts_path, "r") as ts_file:
        timestamps = [line.strip() for line in ts_file]

    poses_xyz = {}
    with open(poses_path, "r") as file:
        for idx, line in enumerate(file):
            elements = line.strip().split()
            trans = np.array(elements[:3], dtype=float)
            quat = np.array(elements[3:], dtype=float)

            if len(trans) > 0:
                transformation_matrix = quaternion_pose_to_4x4(trans, quat)
                timestamp = timestamps[idx]
                poses_xyz[timestamp] = transformation_matrix
            else:
                timestamp = timestamps[idx]
                poses_xyz[timestamp] = np.eye(4)
    return poses_xyz


def interpolate_pose(velodyne_poses, target_timestamp):
    timestamps = [timestamp for timestamp in velodyne_poses.keys()]
    positions = [pose4x4_to_quat(pose)[0] for pose in velodyne_poses.values()]
    quaternions = [pose4x4_to_quat(pose)[1] for pose in velodyne_poses.values()]

    # Find the interval for interpolation
    idx = np.searchsorted(timestamps, target_timestamp) - 1
    t0, t1 = timestamps[idx], timestamps[idx + 1]
    p0, p1 = positions[idx], positions[idx + 1]
    q0, q1 = quaternions[idx], quaternions[idx + 1]

    target_timestamp = np.float128(target_timestamp)
    t0 = np.double(t0)
    t1 = np.double(t1)
    p0 = np.double(p0)
    p1 = np.double(p1)
    q0 = np.double(q0)
    q1 = np.double(q1)

    # Perform linear interpolation for position
    ratio = (target_timestamp - t0) / (t1 - t0)
    interp_position = (1 - ratio) * p0 + ratio * p1

    # Perform SLERP for orientation
    rotations = R.from_quat([q0, q1])
    # print(f"t0: {t0}, t1: {t1}")
    slerp = Slerp([t0, t1], rotations)
    interp_orientation = slerp(target_timestamp).as_quat()

    return interp_position, interp_orientation


def get_frame_numbers(directory_path) -> list:
    """
    Count the total number of files in the directory
    """
    frame_numbers = []
    all_files = os.listdir(directory_path)

    # Filter out files ending with ".bin" and remove the filetype
    filenames = [
        int(re.search(r'\d+', os.path.splitext(file)[0]).group())
        for file in all_files if file.endswith(".bin") and re.search(r'\d+', os.path.splitext(file)[0])
    ]

    for filename in filenames:
        frame_numbers.append(int(filename))

    return sorted(frame_numbers)


# Function to read GPS data from a file
def read_gps_data(file_path):
    points = []
    with open(file_path, "r") as file:
        for line in file:
            lat, lon, alt = map(float, line.strip().split())
            points.append([lat, lon, alt])
    return np.array(points)


def save_lidar_world_poses(root_dir, yaw_offset):
    lidar_ts_path = os.path.join(root_dir, "lidar/timestamps.txt")
    label_path = os.path.join(root_dir, "lidar/labels/gt_labels")
    poses_path = os.path.join(root_dir, "poses/groundtruth_odom.txt")
    pose_ts_path = os.path.join(root_dir, "poses/odom_timestamps.txt")

    # Read lidar timestamps into a list
    with open(lidar_ts_path, "r") as ts_file:
        lidar_timestamps = [line.strip() for line in ts_file]

    # dict with ts as key and pose as value
    lidar_poses = read_quat_poses(poses_path, pose_ts_path)
    labelled_frames = get_frame_numbers(label_path)

    lidar_world_poses = []
    for frame_idx in tqdm(range(0, len(labelled_frames)-1, 1)):
    # for frame_idx in tqdm(range(0, 2000, 1)):
        frame_number = labelled_frames[frame_idx]
        # points_transformed = get_transformed_point_cloud(pc_xyz, velodyne_poses, frame_number)

        # if using groundtruth_odom
        interp_trans, interp_quat = interpolate_pose(lidar_poses, lidar_timestamps[frame_number])
        transformation_matrix = quaternion_pose_to_4x4(interp_trans, interp_quat)
        
        # Move to gps reciever location to be used when TFing to latlon
        # transformation_matrix[:3, 3] += np.array([-0.757, 0.16, 0]) 

        world_lidar_tf = yaw_offset @ transformation_matrix

        # Shift points from lidar to gnss receiver using lidar's relative xy plane
        # Doesnt need to change as only the first gps datapoint is used for all succeeding lidar scans
        world_lidar_tf[:3, 3] += np.array([-0.757, 0.16, 0]) 

        world_position, world_quat = pose4x4_to_quat(world_lidar_tf)

        lidar_world_pose = np.concatenate((world_position, world_quat), axis=0)

        # print(lidar_world_pose)
        lidar_world_poses.append(lidar_world_pose)

    lidar_world_poses_np = np.array(lidar_world_poses)
    lidar_poses_path = os.path.join(root_dir, "lidar/poses_world.txt")
    print(lidar_poses_path)
    np.savetxt(lidar_poses_path, lidar_world_poses_np)


if __name__ == "__main__":
    env = "main_campus"
    robot_name = "robot4"
    env_path = f"/media/donceykong/donceys_data_ssd/datasets/CU_MULTI/data/{env}"

    root_dir = os.path.join(env_path, robot_name)

    if env == "kittredge_loop":
        theta = np.deg2rad(25)
        print(f"theta: {theta}")
    elif env == "main_campus":
        theta = np.deg2rad(-4.5)

    print(f"theta: {theta}")

    yaw_offset = np.array([
        [np.cos(theta), -np.sin(theta), 0, 0],
        [np.sin(theta),  np.cos(theta), 0, 0],
        [0,              0,             1, 0],
        [0,              0,             0, 1]
    ])

    save_lidar_world_poses(root_dir, yaw_offset)