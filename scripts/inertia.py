import numpy as np
from scipy.spatial.transform import Rotation

# Original joint position and inertia
original_joint_position_larm = np.array([0, 0.145, 0.370296])
original_joint_position_rarm = np.array([0, -0.145, 0.370296])
original_inertia_diagonal_larm = np.array([0.00243831, 0.00205563, 0.0014035])
original_inertia_diagonal_rarm = np.array([0.00243831, 0.00205563, 0.0014035])

# New joint position
new_joint_position_larm = np.array([0, 0.145, 1.170296])
new_joint_position_rarm = np.array([0, -0.145, 1.170296])

# Definition of quaternion and inertia frame
rotation_quaternion_larm = np.array([0.801199, -0.06689, -0.00387916, 0.593381])
rotation_quaternion_rarm = np.array([0.593381, -0.0387916, -0.06689, 0.801199])
inertia_frame_position_larm = np.array([-0.0226388, -0.00521383, 0.8156081])
inertia_frame_position_rarm = np.array([-0.00226388, 0.00521383, 0.0156081])

# Convert quaternion to rotation matrix
rotation_matrix_larm = Rotation.from_quat(rotation_quaternion_larm).as_matrix()
rotation_matrix_rarm = Rotation.from_quat(rotation_quaternion_rarm).as_matrix()
new_rotation_matrix_larm = np.block([[rotation_matrix_larm, new_joint_position_larm[:3][:, None]],
                                     [np.zeros((1, 3)), 1]])
new_rotation_matrix_rarm = np.block([[rotation_matrix_rarm, new_joint_position_rarm[:3][:, None]],
                                     [np.zeros((1, 3)), 1]])
new_quaternion_larm = Rotation.from_matrix(new_rotation_matrix_larm[:3, :3]).as_quat()
new_quaternion_rarm = Rotation.from_matrix(new_rotation_matrix_rarm[:3, :3]).as_quat()

# Convert coordinate of original inertia frame
original_inertia_frame_position_larm = np.dot(rotation_matrix_larm, inertia_frame_position_larm)
original_inertia_frame_position_rarm = np.dot(rotation_matrix_rarm, inertia_frame_position_rarm)
# Compute position of new inertia frame
new_inertia_frame_position_larm = new_joint_position_larm[:3] + np.dot(rotation_matrix_larm, inertia_frame_position_larm)
new_inertia_frame_position_rarm = new_joint_position_rarm[:3] + np.dot(rotation_matrix_rarm, inertia_frame_position_rarm)

# Compute inertia tensor in new coordinate system
new_inertia_diagonal_larm = np.dot(rotation_matrix_larm.T, np.dot(np.diag(original_inertia_diagonal_larm), rotation_matrix_larm))
new_inertia_diagonal_rarm = np.dot(rotation_matrix_rarm.T, np.dot(np.diag(original_inertia_diagonal_rarm), rotation_matrix_rarm))

print("New quaternion:", new_quaternion_larm)
print("New inertia frame position:", new_inertia_frame_position_larm)
print("New inertia diagonal:", new_inertia_diagonal_larm)
print("New quaternion:", new_quaternion_rarm)
print("New inertia frame position:", new_inertia_frame_position_rarm)
print("New inertia diagonal:", new_inertia_diagonal_rarm)
