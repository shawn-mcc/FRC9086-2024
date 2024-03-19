import cv2
import os
import threading
from os import walk
from pygrabber.dshow_graph import FilterGraph

class CameraSetup:
    @staticmethod
    def getDevices() -> dict[int, str]:
        devices = FilterGraph().get_input_devices()
        availableCameras = {}
        for device_index, device_name in enumerate(devices):
            availableCameras[device_index] = device_name
        if len(availableCameras) > 0:
            return availableCameras
        else:
            return availableCameras

    @staticmethod
    def calibrateCamera(device: dict) -> None:
        folderName = device[1]
        #Check to see if calibrationImages for the selected camera exist
        try:
            imageNames = next(walk(f"Camera Calibration Images/{folderName}"))[2]
        except:
            print(f"No images found under: Camera Calibration Images/{folderName}")
            print("Starting image taker...")
            CameraSetup.imageTaker(device, f"Camera Calibratinator: {folderName}", f"{folderName}")
        #Check to see if there are actually images in the folder
        if len(imageNames) < 25:
            print(f"No images or not enough images found under: Camera Calibration Images/{folderName}")
            print("Starting image taker...")
            CameraSetup.imageTaker(device, f"Camera Calibratinator: {folderName}", f"{folderName}")

        #Get user input
        try:
            rows = int(input("How many rows are in your calibration pattern?"))
            columns = int(input("How many columns are in your calibration pattern?"))
            sqaureSize = float(input("What is the size of the sqaures (in meters) in your calibration pattern?"))
        except:
            print("Invalid response, rows should be an integer, columns should be an integer, sqaureSize should be a float.")
            CameraSetup.calibrateCamera(device)

        #Start calibration
        imagePoints = []
        objectPoints = []

        for imageName in imageNames:
            image = cv2.imread(f"Camera Calibration Images/{folderName}/{imageName}")
            ret, corners = cv2.findChessboardCorners(image, (columns, rows))
            if ret:
                imagePoints.append(corners)
                patternPoints = np.zeros((rows * columns, 3), np.float32)
                patternPoints[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
                patternPoints *= sqaureSize
                objectPoints.append(patternPoints)
                cv2.drawChessboardCorners(image, (columns, rows), corners, ret)
            cv2.imshow("Calibration", image)
            cv2.waitKey(1)
        imagePoints = np.array(imagePoints)
        objectPoints = np.array(objectPoints)
        imageSize = (image.shape[1], image.shape[0])
        ret, I, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(objectPoints, imagePoints, imageSize, None, None)

        #Intrisic Parameters
        fx = I[0, 0]
        fy = I[1, 1]
        cx = I[0, 2]
        cy = I[1, 2]

        #Distortion Coefficients
        k1 = distCoeffs[0][0]
        k2 = distCoeffs[0][1]
        p1 = distCoeffs[0][2]
        p2 = distCoeffs[0][3]
        k3 = distCoeffs[0][4]

        #Destroy windows and write data to .CUM file
        cv2.destroyWindow("Calibration")
        CalibrationUnitMarkup.writeToFile(f"Camera Calibration Data/{folderName}.cum", fx, fy, cx, cy, distCoeffs[0])
        print(f"Calibration data has been written to a Calibration Unit Markup file @ Camera Calibration Data/{folderName}.cum")


    @staticmethod
    def imageTaker(device: dict, windowName: str, folderName: str):
        os.makedirs(f"Camera Calibration Images/{folderName}", exist_ok=True)
        deviceNumber = device[0]
        capture = cv2.VideoCapture(deviceNumber, cv2.CAP_DSHOW)
        """
        This function displays to the user what the camera
        seeing and doesn't actually handle with any of the
        image taking
        """
        threadOFF = False
        def showView():
            while not threadOFF:
                ret, frame = capture.read()
                cv2.imshow(windowName, frame)
                cv2.waitKey(delay=1)

        thread = threading.Thread(target=showView)
        thread.start()

        """
        This it what actually takes the pictures and
        handles the closing of the windows
        """
        pictureCount = 1
        while True:
            userInput = input()
            ret, frame = capture.read()
            picturePath = os.getcwd() + f"\\Camera Calibration Images\\{folderName}\\{pictureCount}.png"
            cv2.imwrite(picturePath, frame)
            print(f"Picture Taken: {pictureCount} ({picturePath})")
            if pictureCount == 50:
                break
            else:
                pictureCount += 1

        threadOFF = True
        thread.join()
        capture.release()
        cv2.destroyWindow(windowName)

while True:
    avaialableCameras = CameraSetup.getDevices()
    print("Which camera would you like to use?")
    i = 0
    while i < len(avaialableCameras):
        print(f"{i}){avaialableCameras[i]}")
        i += 1

    try:
        camera = int(input())
        if camera > len(avaialableCameras):
            print(f"Invalid response, {camera} is out of range.")
            continue
        else:
            camera = [camera, str(avaialableCameras[camera]).strip()]
            break
    except:
        print("Invalid response, you should only reply with a digit")
        continue
CameraSetup.calibrateCamera(camera)