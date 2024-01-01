import zmq
import numpy as np

context = zmq.Context()
subscriber = context.socket(zmq.SUB)
subscriber.connect("tcp://localhost:5555")
subscriber.setsockopt_string(zmq.SUBSCRIBE, "")

while True:
    # Receive the message
    message = subscriber.recv()

    # Deserialize the data
    num_matrices = len(message) // (16 * 8)  # 16 doubles, each 8 bytes
    pose_matrices = np.frombuffer(message, dtype=np.float32).reshape(num_matrices, 16)

    # pose_matrices now contains a list of poses; the order is the same as the config
    print(pose_matrices)
