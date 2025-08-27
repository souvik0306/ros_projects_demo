#!/usr/bin/env python3
import rospy
import numpy as np
import onnxruntime as ort
from sensor_msgs.msg import Imu

# --- configuration ---
SEQLEN = 200                 # number of samples per inference call (>=10)
ONNX_PATH = "airimu_euroc.onnx"

# ONNX session (float32 inputs assumed)
onnx_model = ort.InferenceSession(ONNX_PATH, providers=["CPUExecutionProvider"])

# buffers
time_buf = []
acc_buf  = []
gyro_buf = []

def run_inference():
    """Prepare tensors and call the ONNX model."""
    # assemble numpy arrays
    time = np.array(time_buf, dtype=np.float64)
    acc  = np.array(acc_buf , dtype=np.float32)
    gyro = np.array(gyro_buf, dtype=np.float32)

    # compute dt and trim to match acc/gyro length
    dt = np.diff(time)[..., None]                   # (T-1, 1)
    acc  = acc[:-1]
    gyro = gyro[:-1]

    # expand batch dimension
    acc  = acc[None, ...]                           # (1, T-1, 3)
    gyro = gyro[None, ...]

    # run ONNX model
    corr_acc, corr_gyro = onnx_model.run(
        None, {"acc": acc, "gyro": gyro}
    )

    # use model outputs as needed (e.g., save, publish, etc.)
    rospy.loginfo(f"Model output shapes: {corr_acc.shape}, {corr_gyro.shape}")

def imu_callback(msg: Imu):
    """Collect IMU samples and trigger inference when buffer is full."""
    t = msg.header.stamp.to_sec()
    acc = (msg.linear_acceleration.x,
           msg.linear_acceleration.y,
           msg.linear_acceleration.z)
    gyro = (msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z)

    time_buf.append(t)
    acc_buf.append(acc)
    gyro_buf.append(gyro)

    if len(time_buf) >= SEQLEN:
        run_inference()

        # keep last 9 samples (network interval) for next window
        keep = 9
        time_buf[:] = time_buf[-keep:]
        acc_buf[:]  = acc_buf[-keep:]
        gyro_buf[:] = gyro_buf[-keep:]

def listener():
    rospy.init_node("imu_inference_node")
    rospy.Subscriber("/imu0", Imu, imu_callback, queue_size=1000)
    rospy.spin()

if __name__ == "__main__":
    listener()
