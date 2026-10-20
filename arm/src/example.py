import zmq
import time
import json
import numpy as np
from pprint import pprint
from scipy.spatial.transform import Rotation as R


def send_target_pose(zmq_ip="127.0.0.1", zmq_port=5555):
    """
    Sends target poses to the C++ UR5 robot controller using ZeroMQ.

    Args:
        zmq_ip (str): The IP address of the ZeroMQ server (default: "127.0.0.1").
        zmq_port (int): The port number of the ZeroMQ server (default: 5555).
    """
    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    publisher.bind(f"tcp://{zmq_ip}:{zmq_port}")

    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.CONFLATE, 1)  # Keep only the latest message
    socket.connect(f"tcp://{zmq_ip}:5556")
    socket.setsockopt_string(zmq.SUBSCRIBE, "")

    # Example: Send a series of target poses
    pose = np.array([0.13, -0.493, 0.34, 0., -0.7, -0.7, 0.])
    # pos = np.array([0.131, -0.493, 0.34]) + np.random.uniform(-0.08, 0.08, (3))


    def get_pose_diff(i, size):
        dx = 0.5 * size * np.sin(i / 25 * np.pi)
        dz = size * 0.5 * (1 - np.cos(i / 25 * np.pi))
        return np.array([dx, 0, -dz, 0, 0, 0, 0])
    
    def random_orientation():
        rx, ry, rz = np.random.uniform(-0.2, 0.2, 3)
        new = pose.copy()
        new[3:] = (R.from_euler('xyz', [rx, ry, rz]) * R.from_quat(new[3:])).as_quat()
        return new
    
    def step_rz(i):
        new = pose.copy()
        new[3:] = (R.from_euler('xyz', [0, 0, 0.2 * (i % 2)]) * R.from_quat(new[3:])).as_quat()
        return new
    
    pose_hist = []
    try:
        for i in range(51):
            start = time.time_ns()
            size = 0.15
            command = {
                "target_pose": list(pose + get_pose_diff(i, size))
                # "target_pose": list(random_orientation())
                # "target_pose": list(pose + np.array([0., 0., -(i%2)*0.01, 0., 0., 0., 0.]))
                # "target_pose": list(step_rz(i))
            }
            diff = np.linalg.norm((get_pose_diff(i, size) - get_pose_diff(i-1, size)), 2) 
            print(f"Sending target pose: {command}    step size: {diff:.4f}")
            publisher.send_json(command)

            # get state
            # state = socket.recv_json()
            # pprint(state)

            # input("wait for enter")
            while (time.time_ns() - start) * 1e-9 < 0.1:
                state = socket.recv_json()
                pose_hist.append([command["target_pose"], state["pos"], [time.time_ns(), 0., 0., 0., 0., 0., 0.]])

    except zmq.ZMQError as e:
        print(f"ZeroMQ error: {e}")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        import matplotlib.pyplot as plt

        def q2r(q):
            r = R.from_quat(q)
            return r.as_euler("xyz")

        pose_hist = np.array(pose_hist)
        plt.plot(pose_hist[:, 2, 0], pose_hist[:, 1, 2], "b", label="actual") 
        plt.plot(pose_hist[:, 2, 0], pose_hist[:, 0, 2], "r", label="target")     
        # plt.plot(pose_hist[:, 2, 0], pose_hist[:, 1, 2], "b", label="actual")     
        # z = [q2r(pose_hist[i, 1, 3:])[2] for i in range(len(pose_hist))]
        # zt = [q2r(pose_hist[i, 0, 3:])[2] for i in range(len(pose_hist))]
        # plt.plot(pose_hist[:, 2, 0], zt, "r", label="target")     
        # plt.plot(pose_hist[:, 2, 0], z, "b", label="actual")     

        # plt.axis("equal")
        plt.show()
        exit(0)


if __name__ == "__main__":
    send_target_pose()
