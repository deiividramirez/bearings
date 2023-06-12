import matplotlib.pyplot as plt
from pathlib import Path
from cv2 import aruco
import argparse
import cv2
import sys

parser = argparse.ArgumentParser(
    usage="generate gazebo models for AR tags",
    formatter_class=argparse.ArgumentDefaultsHelpFormatter,
)

parser.add_argument(
    "-d",
    "--drone-number",
    default=0,
    type=int,
    help="Drone number to be used for aruco detection",
)

parser.add_argument(
    "-i",
    "--default-image",
    default=0,
    type=int,
    help="First image or second image to be used for aruco detection",
)

args = parser.parse_args()

path = Path(__file__).parent.absolute().parent.absolute()

num = args.drone_number

frame = cv2.imread(f"{path}/desired{num}_{args.default_image}.jpg")
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
parameters = aruco.DetectorParameters()

corners, ids, rejectedImgPoints = aruco.detectMarkers(
    frame, aruco_dict, parameters=parameters
)
frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

# set window title
plt.figure(num=f"Drone {num}")
plt.imshow(frame_markers, origin="upper")

if ids is not None:
    for i in range(len(ids)):
        c = corners[i][0]
        plt.plot([c[:, 0].mean()], [c[:, 1].mean()], "+", label="id={0}".format(ids[i]))

    plt.title(f"Aruco markers - Drone {num}")
    plt.legend()
    plt.show()

else:
    if len(rejectedImgPoints) > 0:
        for index, points in enumerate(rejectedImgPoints):
            c = points[0]
            plt.plot([c[:, 0]], [c[:, 1]], "x", label="id={0}".format(index))
        plt.title("Rejected points")
        # plt.legend()
        plt.show()
