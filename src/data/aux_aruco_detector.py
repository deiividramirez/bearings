import matplotlib.pyplot as plt
from pathlib import Path
from cv2 import aruco
import cv2
import sys

path = Path(__file__).parent.absolute().parent.absolute()

num = sys.argv[1] if len(sys.argv) == 2 else 1

frame = cv2.imread(f"{path}/desired_{num}f.jpg")
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
parameters =  aruco.DetectorParameters()

corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

plt.figure()
plt.imshow(frame_markers, origin = "upper")

if ids is not None:
    for i in range(len(ids)):
        c = corners[i][0]
        plt.plot([c[:, 0].mean()], [c[:, 1].mean()], "+", label = "id={0}".format(ids[i]))

    plt.title(f"Aruco markers - Drone {num}")
    plt.legend()
    plt.show()

else:
    if len(rejectedImgPoints) > 0:
        for index, points in enumerate(rejectedImgPoints):
            c = points[0]
            plt.plot([c[:, 0]], [c[:, 1]], "x", label = "id={0}".format(index))
        plt.title("Rejected points")
        # plt.legend()
        plt.show()
