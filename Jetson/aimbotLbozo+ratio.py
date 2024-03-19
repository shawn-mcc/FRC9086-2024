import numpy as np

from Jetson.Util.Utilities import Camera
import cv2


# cameraParameters = [Fx, Fy, Cx, Cy, [K1, K2, P1, P2, K3]]
cameraOne = Camera(0, 1024, 768, 90, [247.1647301017403, 247.57058590418313, 322.3643270435404, 243.59710966927983, [-0.02380914, 0.01899008, 0.00108914, 0.0015457, -0.01937268]])
while True:
    frame = cameraOne.getFrame()
    if frame.any():
        detections = cameraOne.detectTags(frame)
        if detections:
            for tag in detections:
                print(tag)
                #tagID = detection.tag_id
                rvect, tvec = tag.pose_R, tag.pose_T
                #distance = detection.distance
                distance = np.linalg.norm(rvect - tvec)
                print(distance)
    cv2.imshow("Frame", frame)
    cv2.waitKey(1)