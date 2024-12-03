import json
import os.path

import numpy as np
from scipy.spatial.transform import Rotation as Rot


def euler_to_rotation_matrix(pitch, yaw, roll):
    """
    Convert Euler angles (in degrees) to a rotation matrix.

    Args:
    roll: Rotation around the x-axis (in degrees)
    pitch: Rotation around the y-axis (in degrees)
    yaw: Rotation around the z-axis (in degrees)

    Returns:
    A 3x3 numpy array representing the rotation matrix.
    """
    # Convert degrees to radians
    roll_rad = np.deg2rad(roll)
    pitch_rad = np.deg2rad(pitch)
    yaw_rad = np.deg2rad(yaw)

    # Create a rotation object using the euler angles
    rotation = Rot.from_euler('xyz', [pitch_rad, yaw_rad, roll_rad])

    # Return the rotation matrix
    return rotation.as_matrix()


def save_matrices(folder, matrices, output_file, extra=None):
    """
    Save matrices to a file.

    Args:
    matrices: List of matrices.
    output_file: Path to the output file.
    """
    with open(os.path.join(folder, output_file), 'w') as file:
        for matrix in matrices:
            # Convert the numpy array to a flattened string
            matrix_str = ' '.join(map(str, matrix.flatten()))
            file.write(f'{matrix_str}\n')
        #if extra is not None:
        #    file.write()


# if __name__ == "__main__":
#     folder = "nerf_data/test_normal"
#     input_json = 'camera_poses.txt'  # Replace with your input JSON file path
#     rotation_output_txt = 'rotation_output.txt'  # Replace with your desired rotation matrices output file path
#     transform_output_txt = 'transform_output.txt'  # Replace with your desired homogeneous matrices output file path
#
#     # Convert poses to rotation matrices and homogeneous matrices
#     rotation_matrices, homogeneous_matrices = convert_poses_to_colmap(folder, input_json)
#
#     # Save rotation matrices to a file
#     save_matrices(folder, rotation_matrices, rotation_output_txt)
#
#     # Save homogeneous transformation matrices to a file
#     save_matrices(folder, homogeneous_matrices, transform_output_txt)
#
#     print(f"Rotation matrices saved to {rotation_output_txt}")
#     print(f"Homogeneous transformation matrices saved to {transform_output_txt}")


# Define the camera parameters
w, h = 1850, 1000

cameraFOV = 110.0
f = w /(2 * np.tan(cameraFOV * np.pi / 360))
#fl_y = h /(2 * np.tan(cameraFOV * np.pi / 360))
camera_params = {
    "w": w,
    "h": h,
    "fl_x": f,
    "fl_y": f,
    "cx": w/2,
    "cy": h/2,
    "k1": 0.0,
    "k2": 0.0,
    "p1": 0.0,
    "p2": 0.0,
    "camera_model": "OPENCV"
}


def create_homogeneous_matrix(pos, rot_matrix):
    """
    Create a homogeneous transformation matrix.

    Args:
    pos: A dictionary with x, y, z coordinates.
    rot_matrix: A 3x3 rotation matrix.

    Returns:
    A 4x4 numpy array representing the homogeneous transformation matrix.
    """
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rot_matrix
    transform_matrix[:3, 3] = [pos["x"], pos["y"], pos["z"]]

    return transform_matrix


def convert_poses_to_colmap(folder, json_file):
    """
    Converts JSON file with poses to COLMAP-compatible rotation matrices and homogeneous transformation matrices.

    Args:
    json_file: Path to the input JSON file.

    Returns:
    A list of dictionaries with file_path, transform_matrix, and colmap_im_id.
    """
    with open(os.path.join(folder, json_file), 'r') as file:
        data = json.load(file)

    frames = []

    for pose in data["poses"]:
        idx = pose["id"]
        pos = pose["pos"]
        roll = pose["rot"]["roll"]
        pitch = pose["rot"]["pitch"]
        yaw = pose["rot"]["yaw"]
        normal = pose["normal"]
        mtx = pose["matrix"]

        # Get the rotation matrix
        #rotation_matrix = euler_to_rotation_matrix(roll, pitch, yaw)

        # Get the homogeneous transformation matrix
        #homogeneous_matrix = create_homogeneous_matrix(pos, rotation_matrix)

        # Convert matrix to nested lists for JSON serialization
        transform_matrix_list = mtx #homogeneous_matrix.tolist()

        # Create frame dictionary
        frame = {
            "file_path": f"images/pose_{idx:05d}.png",  # Adjust the file path format as needed
            "transform_matrix": transform_matrix_list,
            "colmap_im_id": idx
        }

        frames.append(frame)

    return frames


def save_to_json(folder, output_file, frames):
    """
    Save frames and camera parameters to a JSON file.

    Args:
    output_file: Path to the output JSON file.
    frames: List of frame dictionaries.
    """
    output_data = camera_params.copy()  # Use camera parameters as base
    output_data["frames"] = frames

    with open(os.path.join(folder, output_file), 'w') as file:
        json.dump(output_data, file, indent=4)


if __name__ == "__main__":
    print(f"folder: {os.getcwd()}")
    folder = "../nerf_data/pergola"
    input_json = 'camera_poses.json'  # Replace with your input JSON file path


    output_json = 'transforms.json'  # Replace with your desired output JSON file path

    # Convert poses to the required output format
    frames = convert_poses_to_colmap(folder, input_json)

    # Save the final output to JSON file
    save_to_json(folder, output_json, frames)

    print(f"Output saved to {output_json}")
