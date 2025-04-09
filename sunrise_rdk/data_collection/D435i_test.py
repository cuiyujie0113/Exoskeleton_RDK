import pyrealsense2 as rs
import numpy as np
import cv2

pipeline = rs.pipeline()
config = rs.config()
# 先只启用彩色流，降低数据获取压力
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

try:
    while True:
        try:
            # 将超时时间设为 10000 ms（10秒）
            frames = pipeline.wait_for_frames(timeout_ms=10000)
        except RuntimeError as e:
            print("Frame timeout:", e)
            continue  # 超时则跳过当前循环，再尝试获取帧

        # 获取彩色帧并判断是否存在
        color_frame = frames.get_color_frame() if frames else None
        if color_frame:
            color_image = np.asanyarray(color_frame.get_data())
            cv2.imshow('Color Stream', color_image)
        else:
            print("未获取到彩色帧，继续尝试...")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
