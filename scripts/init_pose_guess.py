""" Visualize the initial pose of object
"""
import os
import numpy as np
import open3d as o3d
import json
import cv2
from scipy.spatial.transform import Rotation as R


def get_points(color, depth, intrinsic, depth_scale=0.001, depth_trunc=1.0):
    """Get the 3D points from color and depth images"""
    # Get the point cloud from color and depth images
    h, w = depth.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    depth = depth * depth_scale
    depth[depth > depth_trunc] = 0
    z = depth
    x = (u - intrinsic[0, 2]) * z / intrinsic[0, 0]
    y = (v - intrinsic[1, 2]) * z / intrinsic[1, 1]
    points = np.stack([x, y, z], axis=-1).reshape(-1, 3)
    colors = color.reshape(-1, 3)
    return points, colors


def print_pose(pose):
    """Print the pose"""
    pose_str = ",".join([f"{p:.6f}" for p in pose.flatten()])
    print(pose_str)


if __name__ == "__main__":
    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    tracker_name_list = ["plug", "socket"]
    model_dir = os.path.join(root_dir, "test_data", "model")
    sequence_dir = os.path.join(root_dir, "test_data", "sequence")
    sequence_id = 0
    export_dir = os.path.join(root_dir, "test_data", "config")

    # Init pose
    init_pose_0 = np.eye(4, dtype=np.float32)
    init_pose_0[:3, 3] = np.array([-0.2, 0.15, 0.62])
    init_pose_0[:3, :3] = R.from_euler("xyz", [90, 0.0, 0.0], degrees=True).as_matrix()
    init_pose_1 = np.eye(4, dtype=np.float32)
    init_pose_1[:3, 3] = np.array([-0.05, 0.115, 0.75])
    init_pose_1[:3, :3] = R.from_euler("xyz", [100, 0.0, 0.0], degrees=True).as_matrix()
    init_pose = [init_pose_0, init_pose_1]

    # Load obj file
    mesh_list = []
    for idx, tracker_name in enumerate(tracker_name_list):
        obj_path = os.path.join(model_dir, tracker_name, f"{tracker_name}.obj")
        mesh = o3d.io.read_triangle_mesh(obj_path)
        mesh.compute_vertex_normals()
        mesh.transform(init_pose[idx])
        mesh_list.append(mesh)

    # Visualize the initial pose of object
    sequence_path = os.path.join(sequence_dir, f"{sequence_id:04d}")
    color_path = os.path.join(sequence_path, "color")
    depth_path = os.path.join(sequence_path, "depth")
    cam_info = json.load(open(os.path.join(sequence_path, "cam_info.json"), "r"))

    # Get the 3D points from color and depth images
    color_file = os.path.join(color_path, "0.png")
    depth_file = os.path.join(depth_path, "0.png")
    intrinsic = np.array(cam_info["intrinsic"])
    depth_scale = cam_info["depth_scale"]
    color = cv2.imread(color_file)
    # Convert to RGB
    color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
    depth = cv2.imread(depth_file, cv2.IMREAD_ANYDEPTH)
    points, colors = get_points(color, depth, intrinsic, depth_scale)

    # Visualize
    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors / 255.0)
    o3d.visualization.draw_geometries([pcd, *mesh_list, origin])

    #
    print("Save the initial pose:")
    for idx, tracker_name in enumerate(tracker_name_list):
        print(f"tracker_name: {tracker_name}")
        print("init_pose:")
        print_pose(init_pose[idx])
        print("inv_init_pose:")
