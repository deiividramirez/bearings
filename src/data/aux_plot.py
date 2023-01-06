"""
This script is used to generate velocity and error plots.
The data is stored in the 'data' folder.
The plots are saved in the same folder with the name 'out_*.png'.

Created by: David Leonardo Ramírez Parada
Email: david.parada@cimat.mx
"""

import matplotlib.pyplot as plt
import numpy as np
import os

from pathlib import Path
path = Path(__file__).parent.absolute()


err = np.loadtxt(f"{path}/errors.txt")
err_pix = np.loadtxt(f"{path}/errors_pix.txt")
time = np.loadtxt(f"{path}/time.txt")
vx = np.loadtxt(f"{path}/Vx.txt")
vy = np.loadtxt(f"{path}/Vy.txt")
vz = np.loadtxt(f"{path}/Vz.txt")
vyaw = np.loadtxt(f"{path}/Vyaw.txt")
lamb = np.loadtxt(f"{path}/lambda.txt")

NUM = 0
try:
    if len(vx) < 3:
        exit()
except:
    exit()
for i in range(2, len(vx)):
    if np.linalg.norm(vx[i]-vx[i-1]) > 1e-3:
        NUM = i
        break


print(f"""


INFORMACIÓN DEL EXPERIMENTO

Tiempo de vuelo: {time[-1]} s
Error promedio: {np.mean(err[NUM:])} pixeles
Velocidad media en x: {np.mean(vx[NUM:])} u/s
Velocidad media en y: {np.mean(vy[NUM:])} u/s
Velocidad media en z: {np.mean(vz[NUM:])} u/s
Velocidad media en yaw: {np.mean(vyaw[NUM:])} u/s
Lambda promedio: {np.mean(lamb[NUM:])}

Error final: {err[-1]} pixeles
Velocidad final en x: {vx[-1]} u/s
Velocidad final en y: {vy[-1]} u/s
Velocidad final en z: {vz[-1]} u/s
Velocidad final en yaw: {vyaw[-1]} u/s""")

fig, ax = plt.subplots(3, 1, figsize=(6, 10))
# fig.canvas.set_window_title('Resultados del experimento')

ax[0].plot(time[NUM:], err[NUM:], "purple", label='Error (c)')
err_pix = err_pix / max(err_pix)
ax[0].plot(time[NUM:], err_pix[NUM:], "r", label='Error (px)')
box = ax[0].get_position()
ax[0].set_position([box.x0, box.y0, box.width * 0.99, box.height])
ax[0].legend(loc='center left', bbox_to_anchor=(1, 0.5))
# ax[0].legend(loc='best', shadow=True)
ax[0].set_ylabel('Error Promedio')


ax[1].plot(time[NUM:], vx[NUM:], label='$V_x$')
ax[1].plot(time[NUM:], vy[NUM:], label='$V_y$')
ax[1].plot(time[NUM:], vz[NUM:], label='$V_z$')
ax[1].plot(time[NUM:], vyaw[NUM:], label='$W_z$')
ax[1].legend(loc='center left', bbox_to_anchor=(1, 0.5))
ax[1].set_ylabel('Velocidades')


ax[2].plot(time[NUM:], lamb[NUM:], "r")
ax[2].set_ylabel('Lambda')
ax[2].set_xlabel('Tiempo (s)')

plt.tight_layout()
plt.savefig(f"{path}/out_velocidades.png", bbox_inches='tight', pad_inches=0.1)


fig, ax = plt.subplots(2, 1, figsize=(5, 10))
imgs = sorted(os.listdir(f"{path}/img"), key=lambda x: int(x.split(".")[0]))
des = f"{path.parent.absolute()}/desired2.jpg"

ax[0].imshow(plt.imread(des))
ax[0].set_ylabel("Imagen deseada")
ax[1].imshow(plt.imread(f"{path}/img/{imgs[-1]}"))
ax[1].set_ylabel("Imagen final")

[i.set_xticks([]) for i in ax]
[i.set_yticks([]) for i in ax]

plt.tight_layout()
plt.savefig(f"{path}/out_imagenes.png", bbox_inches='tight', pad_inches=0.1)

plt.show()
