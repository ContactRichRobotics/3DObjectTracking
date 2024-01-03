""" STAR data format to ICG data format
We need to create a tracker folder for ICG
Required files:
- detector.yaml
- reconstruct.obj
"""

import os
import numpy as np
import yaml
import json
import open3d as o3d
import cv2
import argparse


def generate_tracker_config(
    tracker_name_list,
    model_dir,
    sequence_dir,
    sequence_id,
    export_dir,
    publisher_address="localhost",
    publisher_port=5555,
    subscriber_address="localhost",
    subscriber_port=5556,
    use_realsense=False,
):
    # config macro
    OBJECT_SCALE = 1.0  # the size of object, influencing the accept threshold for depth modality

    # Prepare path
    export_path = os.path.join(export_dir, f"{sequence_id:04d}")
    os.makedirs(export_path, exist_ok=True)

    for tracker_name in tracker_name_list:
        # generate the obj file if not exist
        obj_path = os.path.join(model_dir, tracker_name, f"{tracker_name}.obj")
        sequence_path = os.path.join(sequence_dir, f"{sequence_id:04d}")

        export_obj_path = os.path.join(export_path, f"{tracker_name}.obj")
        os.makedirs(os.path.dirname(export_obj_path), exist_ok=True)
        os.system("cp {} {}".format(obj_path, export_obj_path))

    # save the config yaml
    config_yaml_path = os.path.join(export_path, "config.yaml")
    config_s = cv2.FileStorage(config_yaml_path, cv2.FileStorage_WRITE)

    # save the camera yaml
    if use_realsense:
        config_s.startWriteStruct("RealSenseColorCamera", cv2.FileNode_SEQ)
    else:
        config_s.startWriteStruct("LoaderColorCamera", cv2.FileNode_SEQ)
    config_s.startWriteStruct("", cv2.FileNode_MAP)
    config_s.write("name", "loader_color")
    config_s.write("metafile_path", "camera_color.yaml")
    config_s.endWriteStruct()
    config_s.endWriteStruct()

    # save the normal color viewer
    config_s.startWriteStruct("NormalColorViewer", cv2.FileNode_SEQ)
    config_s.startWriteStruct("", cv2.FileNode_MAP)
    config_s.write("name", "color_viewer")
    config_s.write("color_camera", "loader_color")
    config_s.write("renderer_geometry", "renderer_geometry")
    config_s.endWriteStruct()
    config_s.endWriteStruct()

    # save the renderer geometry
    config_s.startWriteStruct("RendererGeometry", cv2.FileNode_SEQ)
    config_s.startWriteStruct("", cv2.FileNode_MAP)
    config_s.write("name", "renderer_geometry")
    config_s.write("bodies", tracker_name_list)
    config_s.endWriteStruct()
    config_s.endWriteStruct()

    # save the body
    config_s.startWriteStruct("Body", cv2.FileNode_SEQ)
    for tracker_name in tracker_name_list:
        config_s.startWriteStruct("", cv2.FileNode_MAP)
        config_s.write("name", tracker_name)
        config_s.write("metafile_path", f"{tracker_name}_object.yaml")
        config_s.endWriteStruct()
    config_s.endWriteStruct()

    # save the detector
    detector_method = "StaticDetector"
    config_s.startWriteStruct(detector_method, cv2.FileNode_SEQ)
    for tracker_name in tracker_name_list:
        config_s.startWriteStruct("", cv2.FileNode_MAP)
        config_s.write("name", f"{tracker_name}_detector")
        config_s.write("metafile_path", f"{tracker_name}_detector.yaml")
        config_s.write("optimizer", f"{tracker_name}_optimizer")
        config_s.endWriteStruct()
    config_s.endWriteStruct()

    # save the publisher
    config_s.startWriteStruct("ZMQPublisher", cv2.FileNode_SEQ)
    config_s.startWriteStruct("", cv2.FileNode_MAP)
    config_s.write("name", "publisher")
    config_s.write("metafile_path", "publisher.yaml")
    config_s.endWriteStruct()
    config_s.endWriteStruct()

    # save the subscriber
    config_s.startWriteStruct("ZMQSubscriber", cv2.FileNode_SEQ)
    config_s.startWriteStruct("", cv2.FileNode_MAP)
    config_s.write("name", "subscriber")
    config_s.write("metafile_path", "subscriber.yaml")
    config_s.endWriteStruct()
    config_s.endWriteStruct()

    # save the region model
    config_s.startWriteStruct("RegionModel", cv2.FileNode_SEQ)
    for tracker_name in tracker_name_list:
        config_s.startWriteStruct("", cv2.FileNode_MAP)
        config_s.write("name", f"{tracker_name}_region_model")
        config_s.write("metafile_path", "model.yaml")
        config_s.write("body", tracker_name)
        config_s.endWriteStruct()
    config_s.endWriteStruct()

    # save the region modality
    config_s.startWriteStruct("RegionModality", cv2.FileNode_SEQ)
    for tracker_name in tracker_name_list:
        config_s.startWriteStruct("", cv2.FileNode_MAP)
        config_s.write("name", f"{tracker_name}_region_modality")
        config_s.write("body", tracker_name)
        config_s.write("color_camera", "loader_color")
        config_s.write("region_model", f"{tracker_name}_region_model")
        config_s.write("metafile_path", "region_modality.yaml")
        config_s.startWriteStruct("measure_occlusions", cv2.FileNode_MAP)
        config_s.write("depth_camera", "loader_depth")
        config_s.endWriteStruct()
        config_s.endWriteStruct()
    config_s.endWriteStruct()

    # save the depth camera
    if not use_realsense:
        config_s.startWriteStruct("LoaderDepthCamera", cv2.FileNode_SEQ)
    else:
        config_s.startWriteStruct("RealSenseDepthCamera", cv2.FileNode_SEQ)
    config_s.startWriteStruct("", cv2.FileNode_MAP)
    config_s.write("name", "loader_depth")
    config_s.write("metafile_path", "camera_depth.yaml")
    config_s.endWriteStruct()
    config_s.endWriteStruct()

    # save the depth viewer
    config_s.startWriteStruct("NormalDepthViewer", cv2.FileNode_SEQ)
    config_s.startWriteStruct("", cv2.FileNode_MAP)
    config_s.write("name", "depth_viewer")
    config_s.write("depth_camera", "loader_depth")
    config_s.write("renderer_geometry", "renderer_geometry")
    config_s.endWriteStruct()
    config_s.endWriteStruct()

    # save the depth model
    config_s.startWriteStruct("DepthModel", cv2.FileNode_SEQ)
    for tracker_name in tracker_name_list:
        config_s.startWriteStruct("", cv2.FileNode_MAP)
        config_s.write("name", f"{tracker_name}_depth_model")
        config_s.write("metafile_path", "model.yaml")
        config_s.write("body", tracker_name)
        config_s.endWriteStruct()
    config_s.endWriteStruct()

    # save the depth modality
    config_s.startWriteStruct("DepthModality", cv2.FileNode_SEQ)
    for tracker_name in tracker_name_list:
        config_s.startWriteStruct("", cv2.FileNode_MAP)
        config_s.write("name", f"{tracker_name}_depth_modality")
        config_s.write("body", tracker_name)
        config_s.write("depth_camera", "loader_depth")
        config_s.write("depth_model", f"{tracker_name}_depth_model")
        config_s.write("metafile_path", "depth_modality.yaml")
        config_s.endWriteStruct()
    config_s.endWriteStruct()

    # save the link
    config_s.startWriteStruct("Link", cv2.FileNode_SEQ)
    for tracker_name in tracker_name_list:
        config_s.startWriteStruct("", cv2.FileNode_MAP)
        config_s.write("name", f"{tracker_name}_link")
        config_s.write("body", tracker_name)
        modality_list = [f"{tracker_name}_region_modality", f"{tracker_name}_depth_modality"]
        config_s.write("modalities", modality_list)
        config_s.endWriteStruct()
    config_s.endWriteStruct()

    # save the optimizer
    config_s.startWriteStruct("Optimizer", cv2.FileNode_SEQ)
    for tracker_name in tracker_name_list:
        config_s.startWriteStruct("", cv2.FileNode_MAP)
        config_s.write("name", f"{tracker_name}_optimizer")
        config_s.write("root_link", f"{tracker_name}_link")
        config_s.endWriteStruct()
    config_s.endWriteStruct()

    # save the tracker
    config_s.startWriteStruct("Tracker", cv2.FileNode_SEQ)
    config_s.startWriteStruct("", cv2.FileNode_MAP)
    config_s.write("name", "tracker")
    viewer_list = ["color_viewer"]
    viewer_list.append("depth_viewer")

    config_s.write("viewers", viewer_list)
    config_s.write("detectors", [f"{tracker_name}_detector" for tracker_name in tracker_name_list])
    config_s.write("optimizers", [f"{tracker_name}_optimizer" for tracker_name in tracker_name_list])
    config_s.write("publishers", ["publisher"])
    config_s.write("subscribers", ["subscriber"])
    config_s.endWriteStruct()
    config_s.endWriteStruct()
    config_s.release()

    # parse camera information
    # save the cam color
    camera_info_file = os.path.join(sequence_path, "cam_info.json")
    with open(camera_info_file, "r") as f:
        camera_info = json.load(f)
    intrinsic = camera_info["intrinsic"]
    depth_scale = camera_info["depth_scale"]
    f_u = intrinsic[0][0]
    f_v = intrinsic[1][1]
    pp_x = intrinsic[0][2]
    pp_y = intrinsic[1][2]
    img_width = camera_info["img_width"]
    img_height = camera_info["img_height"]

    config_yaml_path = os.path.join(export_path, "camera_color.yaml")
    cam_color_s = cv2.FileStorage(config_yaml_path, cv2.FileStorage_WRITE)
    if not use_realsense:
        cam_color_s.write("load_directory", os.path.join(sequence_path, "color"))
    cam_color_s.startWriteStruct("intrinsics", cv2.FileNode_MAP)
    cam_color_s.write("f_u", f_u)
    cam_color_s.write("f_v", f_v)
    cam_color_s.write("pp_x", pp_x)
    cam_color_s.write("pp_y", pp_y)
    cam_color_s.write("width", img_width)
    cam_color_s.write("height", img_height)
    cam_color_s.endWriteStruct()
    cam_color_s.write("camara2world_pose", np.eye(4))
    cam_color_s.write("depth_scale", depth_scale)
    cam_color_s.write("image_name_pre", "")
    cam_color_s.write("load_index", 0)
    cam_color_s.write("n_leading_zeros", 0)
    cam_color_s.write("image_name_post", "")
    cam_color_s.write("load_image_type", "png")
    cam_color_s.release()

    # save the cam depth
    config_yaml_path = os.path.join(export_path, "camera_depth.yaml")
    cam_depth_s = cv2.FileStorage(config_yaml_path, cv2.FileStorage_WRITE)
    if not use_realsense:
        cam_depth_s.write("load_directory", os.path.join(sequence_path, "depth"))
    cam_depth_s.startWriteStruct("intrinsics", cv2.FileNode_MAP)
    cam_depth_s.write("f_u", f_u)
    cam_depth_s.write("f_v", f_v)
    cam_depth_s.write("pp_x", pp_x)
    cam_depth_s.write("pp_y", pp_y)
    cam_depth_s.write("width", img_width)
    cam_depth_s.write("height", img_height)
    cam_depth_s.endWriteStruct()
    cam_depth_s.write("camara2world_pose", np.eye(4))
    cam_depth_s.write("depth_scale", depth_scale)
    cam_depth_s.write("image_name_pre", "")
    cam_depth_s.write("load_index", 0)
    cam_depth_s.write("n_leading_zeros", 0)
    cam_depth_s.write("image_name_post", "")
    cam_depth_s.write("load_image_type", "png")
    cam_depth_s.release()

    # save the detector
    for tracker_name in tracker_name_list:
        config_yaml_path = os.path.join(export_path, f"{tracker_name}_detector.yaml")
        detector_s = cv2.FileStorage(config_yaml_path, cv2.FileStorage_WRITE)
        init_pose = np.eye(4, dtype=np.float32)
        detector_s.write("link2world_pose", np.linalg.inv(init_pose))  # the object init position
        detector_s.write("reinit_iter", 0)  # no reinit
        detector_s.release()

    # save the model
    config_yaml_path = os.path.join(export_path, "model.yaml")
    model_s = cv2.FileStorage(config_yaml_path, cv2.FileStorage_WRITE)
    model_s.write("model_path", "INFER_FROM_NAME")
    model_s.release()

    # save the modality file
    config_yaml_path = os.path.join(export_path, "region_modality.yaml")
    modality_s = cv2.FileStorage(config_yaml_path, cv2.FileStorage_WRITE)
    modality_s.write("visualize_pose_result", 0)
    modality_s.write("visualize_gradient_optimization", 0)
    modality_s.write("visualize_hessian_optimization", 0)
    modality_s.write("visualize_lines_correspondence", 0)
    modality_s.write("visualize_points_correspondence", 0)
    modality_s.write("visualize_points_depth_image_correspondence", 0)
    modality_s.write("visualize_points_depth_rendering_correspondence", 0)
    modality_s.release()

    config_yaml_path = os.path.join(export_path, "depth_modality.yaml")
    modality_s = cv2.FileStorage(config_yaml_path, cv2.FileStorage_WRITE)
    modality_s.write("visualize_pose_result", 0)
    modality_s.write("visualize_gradient_optimization", 0)
    modality_s.write("visualize_hessian_optimization", 0)
    modality_s.write("visualize_correspondences_correspondence", 0)
    modality_s.write("visualize_points_correspondence", 0)
    modality_s.write("visualize_points_depth_rendering_correspondence", 0)
    modality_s.write("visualization_max_depth", 2.0)  # the max depth for visualization
    modality_s.startWriteStruct("considered_distances", cv2.FileNode_SEQ)
    modality_s.write("", 0.05 * OBJECT_SCALE)
    modality_s.write("", 0.02 * OBJECT_SCALE)
    modality_s.write("", 0.01 * OBJECT_SCALE)
    modality_s.endWriteStruct()
    modality_s.release()

    # save the object
    for tracker_name in tracker_name_list:
        config_yaml_path = os.path.join(export_path, f"{tracker_name}_object.yaml")
        object_s = cv2.FileStorage(config_yaml_path, cv2.FileStorage_WRITE)
        object_s.write("geometry_path", f"{tracker_name}.obj")
        object_s.write("geometry_unit_in_meter", 1.0)
        object_s.write("geometry_counterclockwise", 1)
        object_s.write("geometry_enable_culling", 1)
        object_s.write("geometry_enable_texture", 1)  # enable color
        object_s.write("geometry2body_pose", np.eye(4))  # the pose of the geometry in the body frame
        object_s.release()

    # save the publisher
    config_yaml_path = os.path.join(export_path, "publisher.yaml")
    publisher_s = cv2.FileStorage(config_yaml_path, cv2.FileStorage_WRITE)
    publisher_s.write("address", publisher_address)
    publisher_s.write("port", publisher_port)
    publisher_s.release()

    # save the subscriber
    config_yaml_path = os.path.join(export_path, "subscriber.yaml")
    subscriber_s = cv2.FileStorage(config_yaml_path, cv2.FileStorage_WRITE)
    subscriber_s.write("address", subscriber_address)
    subscriber_s.write("port", subscriber_port)
    subscriber_s.release()


if __name__ == "__main__":
    # parser = argparse.ArgumentParser()
    # parser.add_argument("--tracker_name", type=str, required=True)
    # parser.add_argument("--model_dir", type=str, required=True)
    # parser.add_argument("--sequence_dir", type=str, required=True)
    # parser.add_argument("--export_dir", type=str, required=True)
    # args = parser.parse_args()

    root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    tracker_name_list = ["plug", "socket"]
    model_dir = os.path.join(root_dir, "test_data", "model")
    sequence_dir = os.path.join(root_dir, "test_data", "sequence")
    sequence_id = 0
    export_dir = os.path.join(root_dir, "test_data", "config")
    publisher_address = "localhost"
    publisher_port = 5555
    subscriber_address = "localhost"
    subscriber_port = 5556

    # generate the tracker config
    generate_tracker_config(
        tracker_name_list,
        model_dir,
        sequence_dir,
        sequence_id,
        export_dir,
        publisher_address,
        publisher_port,
        subscriber_address,
        subscriber_port,
        use_realsense=True,
    )
