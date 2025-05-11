import cv2

# 尝试使用 /dev/video0 设备（可改成0看是否更兼容）
cam = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)

if not cam.isOpened():
    print("Cannot open /dev/video0")
    exit(1)

print("Camera opened successfully. Press 'q' to quit.")

while True:
    ret, frame = cam.read()
    if not ret:
        print("Failed to grab frame")
        break

    cv2.imshow("Camera Test - /dev/video0", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cam.release()
cv2.destroyAllWindows()
