"""
This script is used to generate GIF from images.
The images are stored in the 'img' folder.
The GIF is saved in the 'out_sim.gif' file.

Created by: David Leonardo Ramírez Parada
Email: david.parada@cimat.mx
"""

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import imageio
import cv2
import sys
import os

from pathlib import Path
path = Path(__file__).parent.absolute()

num = sys.argv[1] if len(sys.argv) == 2 else 1

# read images
img_dir = f'{path}/img'
img_all = os.listdir(img_dir)

img_names = []
for img_name in img_all:
    if img_name.split('_')[0] == str(num):
        img_names.append(img_name)

img_names.sort(key=lambda x: int(x.split('_')[1].split('.')[0]))

imgs = []
for iter, img_name in enumerate(img_names):
    print(f'Loading image {(iter+1):>4}|{len(img_names):<4}', end='\r')
    img = cv2.imread(os.path.join(img_dir, img_name))
    imgs.append(img)

# save images into GIF
fps = 30
size = (imgs[0].shape[1], imgs[0].shape[0])
gifWriter = imageio.get_writer(f'{path}/out_sim_{num}.gif', fps=fps)
for iter, img in enumerate(imgs):
    print(f'Saving gif at {(iter+1):>4}|{len(imgs):<4}', end='\r')
    gifWriter.append_data(img)
gifWriter.close()
print(f'The GIF has been saved as out_sim_{num}.gif')

# plot images
# fig = plt.figure()
# ims = []
# for img in imgs:
#     im = plt.imshow(img, animated=True)
#     ims.append([im])
# ani = animation.ArtistAnimation(fig, ims, interval=1000/fps, blit=True, repeat_delay=1000)
# print('The GIF is playing.')
# plt.show()
