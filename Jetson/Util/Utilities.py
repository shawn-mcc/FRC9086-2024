from pupil_apriltags import Detector
from pygrabber.dshow_graph import FilterGraph
import cv2

def getDevices() -> dict[int, str]:
    devices = FilterGraph().get_input_devices()
    availableCameras = {}
    for device_index, device_name in enumerate(devices):
        availableCameras[device_index] = device_name
    if len(availableCameras) > 0:
        return availableCameras
    else:
        return availableCameras

class Camera:
    def __init__(self, camID: int, resolutionX: int, resolutionY: int, fps: int, cameraParameters: list):
        # Setup camera
        if camID not in getDevices():
            raise Exception("Invalid camID")
        else:
            self.camera = cv2.VideoCapture(camID, cv2.CAP_DSHOW)

            # Setting the resolution manually drastically reduces the fps?
            """
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, resolutionX)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, resolutionY)
            """
            self.camera.set(cv2.CAP_PROP_FPS, fps)

            #cameraParameters = [Fx, Fy, Cx, Cy, [K1, K2, P1, P2, K3]]
            # Intrinsic parameters
            self.fx = cameraParameters[0]
            self.fy = cameraParameters[1]
            self.cx = cameraParameters[2]
            self.cy = cameraParameters[3]

            # Distortion coefficients
            self.distortionCoefficients = cameraParameters[4]

            #Allow use of pose estimation
            self.poseEstimation = True

            #Define tag size in meters
            self.tagSize = 0.1651

            #Initilize detector
            self.detector = Detector(families="tag36h11",
                                     nthreads=1,
                                     quad_decimate=1.0,
                                     quad_sigma=0.0,
                                     refine_edges=1,
                                     decode_sharpening=0.25,
                                     debug=0)

    def getFrame(self):
        ret, frame = self.camera.read()
        return frame

    def detectTags(self, frame):
        if self.poseEstimation:
            if frame is not None:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                detections = self.detector.detect(frame,
                                                  estimate_tag_pose=True,
                                                  camera_params=[self.fx, self.fy, self.cx, self.cy],
                                                  tag_size=self.tagSize)

                #Check if detections are successful and pose_R is not Null
                if detections and detections[0].pose_R is not None:
                    return detections
                else:
                    return None
            elif frame == None:
                raise Exception("Frame cannot be None")
        else:
            raise Exception("No instrinsic parameters or distortion coefficients defined")