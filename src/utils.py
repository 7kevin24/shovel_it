"""
Utility functions for robot control system
"""
import numpy as np
import pybullet as p
import json
import os
import time
import uuid
import meshcat.transformations as tf
from lerobot.common.utils.geometry import quat_diff_as_angle_axis
from config import JOINT_OFFSETS, INVERT_JOINTS, R_HEADSET_TO_WORLD, SCALE_FACTOR


# ============= ID Generation =============
def gen_id():
    """Generate a unique ID"""
    return str(uuid.uuid4())


# ============= Kinematic Conversion Functions =============
def motors_angle_to_joint_angles(motors_angle):
    """Convert motor angles to joint angles for simulation"""
    motors_angle = motors_angle.copy()
    motors_angle[1] = -motors_angle[1]  # Invert the second joint angle
    motors_angle[4] = -motors_angle[4]  # Invert the fourth joint angle
    return motors_angle + JOINT_OFFSETS * np.pi / 180  # Convert to radians


def joint_angles_to_motors_angle(joint_angles):
    """Convert joint angles to motor angles for robot commands"""
    motors_angle = joint_angles - JOINT_OFFSETS * np.pi / 180  # Convert to radians
    motors_angle[1] = -motors_angle[1]  # Invert the second joint angle
    motors_angle[4] = -motors_angle[4]  # Invert the fourth joint angle
    return motors_angle * 180 / np.pi  # Convert to degrees


# ============= Pose Transformation Functions =============
def pose_array_to_transform(pose_array):
    """Convert a pose array [x, y, z, qx, qy, qz, qw] to a transformation matrix of SE(3)"""
    position = pose_array[:3]
    quaternion = pose_array[3:7]
    rotation_matrix = p.getMatrixFromQuaternion(quaternion)
    matrix = np.eye(4)
    matrix[:3, :3] = np.array(rotation_matrix).reshape(3, 3)
    matrix[:3, 3] = position
    return matrix


def get_quaternion_from_rotation_matrix(R):
    """Convert rotation matrix to quaternion"""
    q = np.zeros(4)
    q[0] = np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
    q[1] = (R[2, 1] - R[1, 2]) / (4 * q[0])
    q[2] = (R[0, 2] - R[2, 0]) / (4 * q[0])
    q[3] = (R[1, 0] - R[0, 1]) / (4 * q[0])
    return q


def get_euler_from_quaternion(quaternion):
    """Convert quaternion to euler angles"""
    qx, qy, qz, qw = quaternion
    # Calculate euler angles (roll, pitch, yaw)
    roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))
    pitch = np.arcsin(2 * (qw * qy - qz * qx))
    yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))
    return np.array([roll, pitch, yaw]) * 180 / np.pi  # Convert to degrees


def get_rotation_matrix_from_euler(euler):
    """Convert euler angles [roll, pitch, yaw] to a rotation matrix"""
    roll, pitch, yaw = np.radians(euler)  # Convert to radians
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])
    
    return R_z @ R_y @ R_x  # Return rotation matrix


def transform_to_pose_array(T):
    """Convert a transformation matrix of SE(3) to a pose array [x, y, z, qx, qy, qz, qw]"""
    position = T[:3, 3]
    rotation_matrix = T[:3, :3]  # Extract rotation matrix from transformation matrix
    quaternion = get_quaternion_from_rotation_matrix(rotation_matrix)
    return np.concatenate((position, quaternion))


def cube_to_robot_xyz(cube_xyz):
    """Convert cube xyz to real robot xyz"""
    x, y, z = cube_xyz
    real_x = -y
    real_y = -x
    real_z = z
    return np.array([real_x, real_y, real_z])


# ============= XR Processing Functions =============
def process_xr_pose(ref, xr_pose, src_name):
    """Process XR controller pose data"""
    ref_controller_xyz = R_HEADSET_TO_WORLD @ ref[:3]
    controller_xyz = R_HEADSET_TO_WORLD @ np.array(xr_pose[:3])

    R_transform = np.eye(4)
    R_transform[:3, :3] = R_HEADSET_TO_WORLD
    R_quat = tf.quaternion_from_matrix(R_transform)

    ref_quat_raw = [ref[6], ref[3], ref[4], ref[5]]  # [w, x, y, z]
    curr_quat_raw = [xr_pose[6], xr_pose[3], xr_pose[4], xr_pose[5]]

    ref_controller_quat = tf.quaternion_multiply(
        tf.quaternion_multiply(R_quat, ref_quat_raw),
        tf.quaternion_conjugate(R_quat),
    )
    controller_quat = tf.quaternion_multiply(
        tf.quaternion_multiply(R_quat, curr_quat_raw),
        tf.quaternion_conjugate(R_quat),
    )

    delta_xyz = (controller_xyz - ref_controller_xyz) * SCALE_FACTOR
    delta_rot = quat_diff_as_angle_axis(ref_controller_quat, controller_quat)
    return delta_xyz, delta_rot


# ============= File I/O Functions =============
def read_json_file(file_path):
    """Read and parse JSON file"""
    try:
        with open(file_path, 'r') as file:
            data = json.load(file)
        return data
    except FileNotFoundError:
        print(f"File {file_path} not found.")
        return {}
    except json.JSONDecodeError:
        print(f"File {file_path} parse error, please check file format.")
        return {}


def write_json_file(file_path, data):
    """Write data to JSON file"""
    try:
        with open(file_path, 'w') as f:
            json.dump(data, f, indent=4)
    except Exception as e:
        print(f"Error writing to {file_path}: {e}")


# ============= Command Parsing Functions =============
def parse_string_to_dict(command_str):
    """Parse command string to dictionary"""
    command_str = command_str.replace(" ", "")  # Remove spaces
    command_list = command_str.split(",")  # Split by comma

    command_dict = {}
    
    for item in command_list:
        print(f"Parsing item: {item}")
        key, value = item.split(":")  # Split by colon
        # Convert boolean values
        if value == "True":
            value = True
        elif value == "False":
            value = False
        # Convert numeric types
        elif value.isdigit():  # If it's a numeric string, convert to number
            value = int(value)
        # Handle special "running_state" key
        if key == "running":
            key = "running_state"
        # Add to dictionary
        command_dict[key] = value

    return command_dict