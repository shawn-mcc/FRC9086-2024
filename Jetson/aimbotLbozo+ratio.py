from Jetson.Util.Utilities import Camera
import cv2


# cameraParameters = [Fx, Fy, Cx, Cy, [K1, K2, P1, P2, K3]]
cameraOne = Camera(1, 1024, 768, 90, [0, 0, 0, 0, [0, 0, 0, 0, 0]])
while True:
    frame = cameraOne.getFrame()
    if frame.any():
        detections = cameraOne.detectTags(frame)
        print(detections)

    cv2.imshow("Frame", frame)
    cv2.waitKey(1)