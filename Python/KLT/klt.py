# This script is the implementation of the Kanade-Lucas-Tomasi (KLT) feature tracker.
# The KLT tracker is a simple and efficient tracker that is used in many applications.
# The KLT tracker is based on the assumption that the image intensity does not change
# significantly between two consecutive frames. The KLT tracker is a local tracker
# that tracks the feature points in a small window around the feature point. The KLT
# tracker is a sparse tracker that tracks only a few feature points in the image.
# The KLT tracker is a point-based tracker that tracks the feature points in the image.

from pathlib import Path
import numpy as np
import time
import cv2
import sys
import os

path = str(Path(__file__).parents[2]) + "/src/data/img/"
imgs = os.listdir(path)
imgs.sort(key=lambda x: int(x.split('.')[0]))

ImagenDes = cv2.imread(str(Path(__file__).parents[2]) + "/src/desired.jpg")
# Read the first ImagenA
# ret, ImagenA = cap.read()
ImagenA = cv2.imread(path + imgs[0])

if ImagenA is None:
    print('Failed to load image file.')
    sys.exit(1)

descriptor = cv2.ORB_create(nfeatures=300)

# Convert the first ImagenA to grayscale
ImagenA_gray = cv2.cvtColor(ImagenA, cv2.COLOR_BGR2GRAY)
desired_gray = cv2.cvtColor(ImagenDes, cv2.COLOR_BGR2GRAY)

fptsImagenA, feat_ImagenA = descriptor.detectAndCompute(ImagenA_gray, None)
pointImagenDes, feat_ImagenDes = descriptor.detectAndCompute(
    desired_gray, None)

bf = cv2.BFMatcher(cv2.NORM_HAMMING)
best_matches = bf.knnMatch(feat_ImagenA, feat_ImagenDes, 2)
# Se filtran los mejores matches por medio del Lowe's Ratio Test
ratio_thresh = 0.7
matches = []
for m, n in best_matches:
    if m.distance <= ratio_thresh * n.distance:
        matches.append(m)

# Matches as keypoints
pointsToTrack = np.zeros((len(matches), 1, 2), dtype=np.float32)
for index, m in enumerate(matches):
    pointsToTrack[index] = fptsImagenA[m.queryIdx].pt

for i in range(1, len(imgs)):
    mask = np.zeros_like(ImagenA)
    # Create a mask image for drawing purposes
    # Read the second ImagenA
    # ret, ImagenA = cap.read()
    ImagenB = cv2.imread(path + imgs[i])

    # Convert the second ImagenB to grayscale
    ImagenB_gray = cv2.cvtColor(ImagenB, cv2.COLOR_BGR2GRAY)
    # Calculate optical flow (i.e. track feature points)
    temp = cv2.calcOpticalFlowPyrLK(ImagenA_gray, ImagenB_gray, pointsToTrack, None)

    next = temp[0]
    status = temp[1]
    err = temp[2]

    # Select good feature points for the previous position
    good_old = pointsToTrack[status == 1]

    # Select good feature points for the next position
    good_new = next[status == 1]

    # Draw the tracks
    for (new, old) in zip(good_new, good_old):
        a, b = new.ravel()
        c, d = old.ravel()
        a, b, c, d = int(a), int(b), int(c), int(d)
        mask = cv2.line(mask, (a, b), (c, d), (0, 255, 0), 2)
        mask = cv2.circle(mask, (a, b), 5, (255, 0, 0), -1)
        mask = cv2.circle(mask, (c, d), 5, (0, 0, 255), -1)

    # display in one window the original ImagenA and the tracked points
    output = cv2.add(ImagenB, mask)
    cv2.imshow("output", output)
    # cv2.imshow("desired", ImagenDes)
    cv2.imwrite('./out/' + imgs[i], output)
    time.sleep(0.1)
    cv2.waitKey(1)

    # Now update the previous frame and previous points
    ImagenA_gray = ImagenB_gray.copy()
    pointsToTrack = good_new.reshape(-1, 1, 2)


print(len(imgs))
cv2.waitKey(0)
cv2.destroyAllWindows()
