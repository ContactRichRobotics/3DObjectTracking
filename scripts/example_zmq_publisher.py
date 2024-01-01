import zmq
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

# Initialize ZeroMQ context and a publisher socket
context = zmq.Context()
publisher = context.socket(zmq.PUB)
publisher.bind("tcp://*:5556")  # Bind to a port, e.g., 5556

# Prepare your data: a list of 16-element arrays
plug_pose = np.eye(4)
plug_pose[:3, 3] = np.array([-0.2, 0.15, 0.62])
plug_pose[:3, :3] = R.from_euler("xyz", [90, 0.0, 0.0], degrees=True).as_matrix()
socket_pose = np.eye(4)
socket_pose[:3, 3] = np.array([-0.05, 0.115, 0.75])
socket_pose[:3, :3] = R.from_euler("xyz", [100, 0.0, 0.0], degrees=True).as_matrix()
pose_matrices = [plug_pose, socket_pose]

# Convert list of arrays to a single NumPy array for efficient sending
pose_matrices_np = np.array(pose_matrices).astype(np.float32)

# Serialize the NumPy array to a bytes object (if necessary)
# For NumPy arrays, serialization is straightforward
serialized_data = pose_matrices_np.tobytes()

while True:
    # Send the serialized data
    publisher.send(serialized_data)
    print("Data published successfully.")
