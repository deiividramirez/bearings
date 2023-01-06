"""
This script is used to generate mp4 from images.
The images are stored in the 'img' folder.
The mp4 is saved in the 'out_sim.mp4' file.

Created by: David Leonardo Ramírez Parada
Email: david.parada@cimat.mx
"""

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import cv2
import os

from pathlib import Path
path = Path(__file__).parent.absolute()


# Read images
img_dir = f'{path}/img'
img_names = os.listdir(img_dir)
img_names.sort(key=lambda x: int(x.split('.')[0]))

imgs = []
for iter, img_name in enumerate(img_names):
    print(f'Loading image {iter+1}|{len(img_names)}', end='\r')
    img = cv2.imread(os.path.join(img_dir, img_name))
    imgs.append(img)

# Save images into mp4
fps = 30
size = (imgs[0].shape[1], imgs[0].shape[0])
videoWriter = cv2.VideoWriter(f'{path}/out_sim.mp4', cv2.VideoWriter_fourcc('m', 'p', '4', 'v'), fps, size)
for iter, img in enumerate(imgs):
    print(f'Saving mp4 at {iter+1}|{len(imgs)}', end='\r')
    videoWriter.write(img)
videoWriter.release()
print('The video has been saved.')

# plot images
# fig = plt.figure()
# ims = []
# for img in imgs:
    # im = plt.imshow(img, animated=True)
    # ims.append([im])
# ani = animation.ArtistAnimation(fig, ims, interval=1000/fps, blit=True, repeat_delay=1000)
# print('The video is playing.')
# plt.show()
