import os
import re

def get_indices_in_range(pointcloud_timestamps_path, reference_timestamps_path):
    """
    Find the indices of the first and last timestamps from the point cloud's timestamps
    that fall within the range of the reference timestamps.

    Args:
        pointcloud_timestamps_path (str): Path to the point cloud's timestamps.txt file.
        reference_timestamps_path (str): Path to the reference timestamps file.

    Returns:
        tuple: Indices of the first and last timestamps in the point cloud's timestamps.
    """
    # Read the reference timestamps
    with open(reference_timestamps_path, "r") as file:
        reference_timestamps = [float(line.strip()) for line in file]

    reference_start = reference_timestamps[0]
    reference_end = reference_timestamps[-1]
    print(f"Reference timestamp range: {reference_start} to {reference_end}")

    # Read the point cloud timestamps
    with open(pointcloud_timestamps_path, "r") as file:
        pointcloud_timestamps = [float(line.strip()) for line in file]

    print(f"len(pointcloud_timestamps)-1: {len(pointcloud_timestamps)-1}")
    # Find the first and last indices within the range
    first_index = next(
        (i for i, ts in enumerate(pointcloud_timestamps) if ts >= reference_start), None
    )
    last_index = next(
        (i for i, ts in reversed(list(enumerate(pointcloud_timestamps))) if ts <= reference_end),
        None,
    )

    if first_index is None or last_index is None:
        raise ValueError("No timestamps fall within the reference range.")

    print(f"First index in range: {first_index}, Last index in range: {last_index}")
    return first_index, last_index, len(pointcloud_timestamps)

def reindex_pointclouds(directory, prefix="lidar_pointcloud_", extension=".bin", delete_start=3, delete_end=3):
    """
    Reindex files in a directory to be 0-indexed, ensuring numerical sorting and removing files from both ends.

    Args:
        directory (str): Path to the directory containing the pointcloud files.
        prefix (str): The prefix of the files to rename.
        extension (str): The file extension of the pointcloud files.
        delete_start (int): Number of files to delete from the start.
        delete_end (int): Number of files to delete from the end.
    """
    # List all files with the given prefix and extension
    files = [f for f in os.listdir(directory) if f.startswith(prefix) and f.endswith(extension)]

    # Extract numerical part and sort numerically
    files = sorted(files, key=lambda x: int(re.search(r'\d+', x).group()))

    # Delete the first `delete_start` files
    for old_name in files[:delete_start]:
        old_path = os.path.join(directory, old_name)
        os.remove(old_path)
        print(f"Deleted: {old_name} (start)")

    # Delete the last `delete_end` files
    for old_name in files[-delete_end:]:
        old_path = os.path.join(directory, old_name)
        os.remove(old_path)
        print(f"Deleted: {old_name} (end)")

    # Remaining files after deletion
    remaining_files = files[delete_start:len(files) - delete_end]

    # Reindex the remaining files
    for new_index, old_name in enumerate(remaining_files):
        old_path = os.path.join(directory, old_name)
        new_name = f"{prefix}{new_index}{extension}"
        new_path = os.path.join(directory, new_name)
        
        # Rename the file
        os.rename(old_path, new_path)
        print(f"Renamed: {old_name} -> {new_name}")


def update_timestamps(timestamps_path, delete_start=3, delete_end=3):
    """
    Update timestamps.txt by removing the corresponding lines from both ends.

    Args:
        timestamps_path (str): Path to the timestamps.txt file.
        delete_start (int): Number of lines to delete from the start.
        delete_end (int): Number of lines to delete from the end.
    """
    if os.path.exists(timestamps_path):
        with open(timestamps_path, "r") as file:
            lines = file.readlines()
        
        # Remove the first `delete_start` lines and last `delete_end` lines
        updated_lines = lines[delete_start:len(lines) - delete_end]

        with open(timestamps_path, "w") as file:
            file.writelines(updated_lines)
        
        print(f"Updated timestamps.txt by removing {delete_start} lines from the start and {delete_end} lines from the end.")
    else:
        print(f"timestamps.txt not found at {timestamps_path}.")

# Usage
env = "main_campus"
robot_name = "robot4"

pointcloud_timestamps_path = f"/media/donceykong/donceys_data_ssd/datasets/CU_MULTI/data/{env}/{robot_name}/lidar/timestamps.txt"
reference_timestamps_path = f"/media/donceykong/donceys_data_ssd/datasets/CU_MULTI/data/{env}/{robot_name}/poses/odom_timestamps.txt"

first_index, last_index, pc_timestamps_len = get_indices_in_range(pointcloud_timestamps_path, reference_timestamps_path)
print(f"First index: {first_index}, Last index: {last_index}")

if first_index > 0:
    first_n_files = first_index + 1
else:
    first_n_files = 0
last_n_files = pc_timestamps_len -1 - last_index

print(f"first_n_files: {first_n_files}, last_n_files: {last_n_files}")
# pointclouds_directory = f"/media/donceykong/donceys_data_ssd/datasets/CU_MULTI/data/{env}/{robot_name}/lidar/pointclouds"
# labels_directory = f"/media/donceykong/donceys_data_ssd/datasets/CU_MULTI/data/{env}/{robot_name}/lidar/labels/gt_labels"

# # Process pointclouds
# reindex_pointclouds(pointclouds_directory, delete_start=first_n_files, delete_end=last_n_files)

# # Process labels
# reindex_pointclouds(labels_directory, delete_start=first_n_files, delete_end=last_n_files)

# # Update timestamps once
# update_timestamps(pointcloud_timestamps_path, delete_start=first_n_files, delete_end=last_n_files)