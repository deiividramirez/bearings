import matplotlib.pyplot as plt
from pathlib import Path
from cv2 import aruco
import cv2

path = Path(__file__).parent.absolute().parent.absolute()


frame = cv2.imread(f"{path}/desired_3f.jpg")
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters =  aruco.DetectorParameters()
corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

plt.figure()
plt.imshow(frame_markers, origin = "upper")
if ids is not None:
    for i in range(len(ids)):
        c = corners[i][0]
        plt.plot([c[:, 0].mean()], [c[:, 1].mean()], "+", label = "id={0}".format(ids[i]))
"""for points in rejectedImgPoints:
    y = points[:, 0]
    x = points[:, 1]
    plt.plot(x, y, ".m-", linewidth = 1.)"""
plt.legend()
plt.show()
