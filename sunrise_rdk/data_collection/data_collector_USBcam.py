import os
import h5py
import cv2
import threading
import numpy as np
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class DataCollector(Node):
    def __init__(self):
        super().__init__('data_collector')
        self.servo_num = 8
        self.angles = [math.nan] * self.servo_num
        self.all_angles = []
        self.cam_frames = []
        self.timestamps = []
        self.zero_pos = np.array([2048] * self.servo_num)
        self.rot_direction = np.array([-1, -1, 1, -1, 1, -1, -1, -1])

        # 视频采集初始化
        self.cam = cv2.VideoCapture('/dev/video0')
        if not self.cam.isOpened():
            self.get_logger().error('Cannot open camera')

        # 线程锁和停止事件
        self.lock = threading.Lock()
        self.save_end_event = threading.Event()

        # 订阅 /Exo/Angles 话题，仅更新最新角度
        self.create_subscription(
            Float32MultiArray,
            '/Exo/Angles',
            self.angle_callback,
            10
        )
        self.get_logger().info('Subscribed to /Exo/Angles, ready to receive data')

        # 相机采集线程
        self.cam_thread = threading.Thread(target=self.cam_thread_function, daemon=True)
        self.cam_thread.start()
        self.get_logger().info('Camera thread started')

        # 启动停止监听线程，只需按 Enter 停止采集
        threading.Thread(target=self.listen_stop, daemon=True).start()
        self.get_logger().info('Press Enter at any time to stop recording')

    def listen_stop(self):
        input()
        self.save_end_event.set()
        rclpy.shutdown()

    def angle_callback(self, msg: Float32MultiArray):
        data = list(msg.data)
        with self.lock:
            if len(data) >= self.servo_num:
                self.angles = data[:self.servo_num]
            else:
                self.angles = data + [math.nan] * (self.servo_num - len(data))
        self.get_logger().debug(f'Angle updated: {self.angles}')

    def cam_thread_function(self):
        while not self.save_end_event.is_set():
            ret, frame = self.cam.read()
            if not ret:
                self.get_logger().warning('Failed to capture camera frame')
                continue

            # 同步采样：图片和最新角度
            timestamp = self.get_clock().now().nanoseconds
            with self.lock:
                angles_snapshot = self.angles.copy()

            # 图像压缩
            encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])[1]

            # 存储
            self.timestamps.append(timestamp)
            self.all_angles.append(angles_snapshot)
            self.cam_frames.append(encoded.astype(np.uint8))

            idx = len(self.cam_frames)
            self.get_logger().info(f'Synchronized sample {idx}: frame and angles saved')

    def zeropad_stack_arrays(self, array_list):
        max_len = max(len(arr) for arr in array_list)
        padded = [np.pad(arr, (0, max_len - len(arr)), 'constant') for arr in array_list]
        return np.vstack(padded).astype(np.uint8)
    
    def exo_to_servo(self, exo_angles):
        self.target_pos = self.zero_pos + self.rot_direction * exo_angles * 2048 / 180
        return self.target_pos.astype(np.int32)

    def save_end(self):
        # 停止采集并释放资源
        self.cam.release()
        self.get_logger().info('Start writing HDF5 file')

        # 转换数据
        exo_angles = np.array(self.all_angles, dtype=np.int32)
        qpos = self.exo_to_servo(exo_angles)
        action = qpos.copy()
        frames = self.zeropad_stack_arrays(self.cam_frames)
        timestamps = np.array(self.timestamps, dtype=np.int64)

        # 写入 HDF5
        with h5py.File('data_record.hdf5', 'w') as f:
            f.create_dataset('action', data=action)
            grp_obs = f.create_group('observations')
            grp_obs.create_dataset('qpos', data=qpos)
            grp_obs.create_dataset('timestamps', data=timestamps)
            grp_imgs = grp_obs.create_group('images')
            grp_imgs.create_dataset('0', data=frames)

        self.get_logger().info('HDF5 data saved as data_record.hdf5')


def main(args=None):
    rclpy.init(args=args)
    dc = DataCollector()
    dc.get_logger().info('Data collector initialized and running')
    try:
        rclpy.spin(dc)
    except KeyboardInterrupt:
        dc.get_logger().info('Keyboard interrupt received, stopping...')

    # 保存并退出
    dc.save_end()
    dc.destroy_node()

if __name__ == '__main__':
    main()
