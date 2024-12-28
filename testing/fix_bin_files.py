import os
import numpy as np

# Directory containing the .bin files
input_dir = "/home/donceykong/Desktop/datasets/KITTI-360/data_3d_semantics/2013_05_28_drive_0000_sync/labels"

output_dir = "/home/donceykong/Desktop/datasets/KITTI-360/data_3d_semantics/2013_05_28_drive_0000_sync/labels_int32"

# Function to convert and save the .bin files
def convert_bin_files(input_dir):
    # Iterate through all files in the directory
    for filename in os.listdir(input_dir):
        if filename.endswith(".bin"):
            filepath = os.path.join(input_dir, filename)
            
            # Load the data as np.int16
            data = np.fromfile(filepath, dtype=np.int16)
            
            # Convert the data to np.int32
            data = data.astype(np.int32)
            
            # Save the data back as np.int32
            filepath = os.path.join(output_dir, filename)
            data.tofile(filepath)
            
            print(f"Converted {filename} from np.int16 to np.int32")

# Run the conversion
convert_bin_files(input_dir)

