import os
import h5py
import cv2
import threading
import numpy as np
import math
import rclpy
from rclpy.node import Node
from sunrise_rdk.angle_io.MT6701_I2C import MT6701WithMux


class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')

        # angles
        self.angles = [math.nan] * 8
        self.mt6701_with_mux = MT6701WithMux(bus_num=0)
        self.all_angles = []

        # cam
        self.cam = cv2.VideoCapture("/dev/video0")
        if not self.cam.isOpened():
            self.get_logger().error("Cannot open camera")
        self.cam_frames = []

        # thread
        self.angle_lock = threading.Lock()
        self.cam_lock = threading.Lock()
        self.angle_event = threading.Event()
        self.cam_event = threading.Event()
        self.save_end_event = threading.Event()

        self.angle_thread = threading.Thread(target=self.angle_thread_function)
        self.cam_thread = threading.Thread(target=self.cam_thread_function)

        self.angle_thread.start()
        self.cam_thread.start()

    def angle_thread_function(self):
        while not self.save_end_event.is_set():
            self.angle_event.wait()
            self.angle_event.clear()

            with self.angle_lock:
                for channel in range(8):
                    angle_mux = self.mt6701_with_mux.MT6701_I2C_read_angle(channel)
                    if angle_mux is not None:
                        self.angles[channel] = angle_mux
                    else:
                        self.angles[channel] = math.nan

            self.all_angles.append(self.angles.copy())
            print(f"angles={len(self.all_angles)}")

    def cam_thread_function(self):
        while not self.save_end_event.is_set():
            self.cam_event.wait()
            self.cam_event.clear()

            with self.cam_lock:
                ret, frame = self.cam.read()
                if not ret:
                    print("Cannot read camera frame")
                    continue
                else:
                    # 将图像帧进行JPEG编码
                    encoded_frame = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])[1]

            self.cam_frames.append(encoded_frame.astype(np.uint8))
            print(f"cam_frames={len(self.cam_frames)}")
            print(self.cam_frames[-1][:10])  # 打印前10字节

    def zeropad_stack_arrays(self, array_list):
        # 获取最大长度
        max_len = max(len(arr) for arr in array_list)
        # 对不一致的部分进行零填充
        padded_arrays = [np.pad(arr, (0, max_len - len(arr)), 'constant') for arr in array_list]
        result = np.vstack(padded_arrays)
        return result.astype(np.uint8)

    def save_end(self):
        input("Press Enter to stop recording...\n")

        self.save_end_event.set()
        self.angle_event.set()
        self.cam_event.set()

        self.cam.release()

        self.get_logger().info("Start writing hdf5 file")

        # 使用zeropad_stack_arrays方法确保图像帧形状一致
        self.cam_frames = self.zeropad_stack_arrays(self.cam_frames)

        # 保存数据到HDF5文件
        with h5py.File('data.h5', 'w') as f:
            f.create_dataset('angles', data=self.all_angles)
            f.create_dataset('cam_frames', data=self.cam_frames)

        self.get_logger().info("Data saved successfully")


def main(args=None):
    rclpy.init(args=args)
    data_collector = DataCollector()
    data_collector.get_logger().info("Data collector initialized")

    input("Press Enter to start recording...\n")
    try:
        while not data_collector.save_end_event.is_set():
            data_collector.angle_event.set()
            data_collector.cam_event.set()
            rclpy.spin_once(data_collector, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    data_collector.save_end()
    data_collector.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
