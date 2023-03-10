import matplotlib.animation as animation
import matplotlib.pyplot as plt
import cv2
import os

# read images
img_dir = './Tesis/KLT/out'
img_names = os.listdir(img_dir)
img_names.sort(key=lambda x: int(x.split('.')[0]))

imgs = []
for img_name in img_names:
    img = cv2.imread(os.path.join(img_dir, img_name))
    imgs.append(img)

# save images into mp4
fps = 10
size = (imgs[0].shape[1], imgs[0].shape[0])
videoWriter = cv2.VideoWriter('./Tesis/KLT/out.mp4', cv2.VideoWriter_fourcc('m', 'p', '4', 'v'), fps, size)
for img in imgs:
    videoWriter.write(img)
videoWriter.release()
print('The video has been saved.')

# plot images
fig = plt.figure()
ims = []
for img in imgs:
    im = plt.imshow(img, animated=True)
    ims.append([im])
ani = animation.ArtistAnimation(fig, ims, interval=1000/fps, blit=True, repeat_delay=1000)
print('The video is playing.')
plt.show()
