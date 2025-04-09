#!/usr/bin/env python3
import os
import h5py
import cv2
import threading
import numpy as np
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sunrise_rdk.angle_io.MT6701_I2C import MT6701WithMux


class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')

        # 初始化角度数据相关变量
        self.angles = [math.nan] * 8
        self.mt6701_with_mux = MT6701WithMux(bus_num=0)
        self.all_angles = []

        # 初始化摄像头数据和 cv_bridge
        self.cam_frames = []
        self.bridge = CvBridge()
        self.cam_lock = threading.Lock()

        # 订阅 ROS2 图像话题
        self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10
        )

        # 角度采集相关线程和同步
        self.angle_lock = threading.Lock()
        self.angle_event = threading.Event()
        self.save_end_event = threading.Event()
        self.angle_thread = threading.Thread(target=self.angle_thread_function)
        self.angle_thread.start()

    def image_callback(self, msg):
        # 将 ROS 图像消息转换为 OpenCV 格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        # JPEG 编码图像，质量设置为 50
        ret, encoded_frame = cv2.imencode('.jpg', cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        if not ret:
            self.get_logger().error("Failed to encode image")
            return

        with self.cam_lock:
            self.cam_frames.append(encoded_frame.astype(np.uint8))
            self.get_logger().info(f"Received camera frame, total count: {len(self.cam_frames)}")
            print(encoded_frame[:10])  # 打印前10字节以调试

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
            print(f"Angles collected count: {len(self.all_angles)}")

    def zeropad_stack_arrays(self, array_list):
        # 获取最大长度
        max_len = max(len(arr) for arr in array_list)
        # 对所有数组进行零填充，确保各数组长度一致
        padded_arrays = [np.pad(arr, (0, max_len - len(arr)), 'constant') for arr in array_list]
        result = np.vstack(padded_arrays)
        return result.astype(np.uint8)

    def save_end(self):
        input("Press Enter to stop recording...\n")
        self.save_end_event.set()
        self.angle_event.set()  # 释放角度线程等待状态

        self.get_logger().info("Start writing HDF5 file")

        # 使用锁保护摄像头帧数据，并对数据进行零填充处理
        with self.cam_lock:
            cam_frames_array = self.zeropad_stack_arrays(self.cam_frames)

        # 保存角度和摄像头帧数据到 HDF5 文件
        with h5py.File('data.h5', 'w') as f:
            f.create_dataset('angles', data=self.all_angles)
            f.create_dataset('cam_frames', data=cam_frames_array)

        self.get_logger().info("Data saved successfully")


def main(args=None):
    rclpy.init(args=args)
    data_collector = DataCollector()
    data_collector.get_logger().info("Data collector initialized")

    input("Press Enter to start recording...\n")
    try:
        while not data_collector.save_end_event.is_set():
            data_collector.angle_event.set()
            rclpy.spin_once(data_collector, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    data_collector.save_end()
    data_collector.destroy_node()


if __name__ == '__main__':
    main()
