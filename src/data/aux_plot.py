"""
This script is used to generate velocity and error plots.
The data is stored in the 'data' folder.
The plots are saved in the same folder with the name 'out_*.png'.

Created by: David Leonardo Ramírez Parada
Email: david.parada@cimat.mx
"""

import matplotlib.pyplot as plt
import numpy as np
import sys
import os

from pathlib import Path
path = Path(__file__).parent.absolute()

DRONE_COUNT = 5

lider = 0
if len(sys.argv) == 2:
    dron = sys.argv[1]
elif len(sys.argv) == 3:
    lider = sys.argv[1]
    dron = sys.argv[2]
else:

    fig, ax = plt.subplots(4, DRONE_COUNT, figsize=(15, 5), sharex=True, num=f"Velocidades y errores para todos los drones")
    fig3d, ax3d = plt.subplots(1, 1, figsize=(
        5, 5), subplot_kw=dict(projection='3d'), num=f"Posiciones para todos los drones")

    for dron in range(1, DRONE_COUNT+1):
        err = np.loadtxt(f"{path}/out/out_errors_{dron}.txt")
        err_pix = np.loadtxt(f"{path}/out/out_errors_pix_{dron}.txt")
        time = np.loadtxt(f"{path}/out/out_time_{dron}.txt")
        vx = np.loadtxt(f"{path}/out/out_Vx_{dron}.txt")
        vy = np.loadtxt(f"{path}/out/out_Vy_{dron}.txt")
        vz = np.loadtxt(f"{path}/out/out_Vz_{dron}.txt")
        vyaw = np.loadtxt(f"{path}/out/out_Vyaw_{dron}.txt")
        intx = np.loadtxt(f"{path}/out/out_integral_x_{dron}.txt")
        inty = np.loadtxt(f"{path}/out/out_integral_y_{dron}.txt")
        intz = np.loadtxt(f"{path}/out/out_integral_z_{dron}.txt")

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
Drone {dron}
Tiempo total -> {time[-1]}
Error final -> {err[-1]}
Velocidad final -> {vx[-1], vy[-1], vz[-1], vyaw[-1]}
        """)

        ax[0][dron-1].title.set_text(f"Drone {dron}")
        ax[0][dron-1].plot(time[NUM:], err[NUM:], "purple", label='Error (c)')
        if dron == 2 or dron == 1:
            err_pix_t = (err_pix) / max(err_pix[NUM+10:]) * max(err[NUM:])
            ax[0][dron-1].plot(time[NUM:], err_pix_t[NUM:],
                               "r", label='|Error (px)|')
        ax[0][dron-1].plot([time[NUM], time[-1]], [err[-1], err[-1]], ":",
                           c="purple", label=f"y={err[-1]:.3f}", alpha=.5)
        ax[0][dron-1].plot([time[NUM], time[-1]], [0, 0],
                           "k:", label="y=0", alpha=.5)
        box = ax[0][dron-1].get_position()
        ax[0][dron-1].set_position([box.x0, box.y0,
                                   box.width * 0.99, box.height])
        ax[0][dron-1].legend(loc='center left',
                             bbox_to_anchor=(1, 0.5), shadow=True)
        ax[0][dron-1].set_ylabel('Error Promedio')

        ax[1][dron-1].plot(time[NUM:], vx[NUM:], label='$V_x$')
        ax[1][dron-1].plot(time[NUM:], vy[NUM:], label='$V_y$')
        ax[1][dron-1].plot(time[NUM:], vz[NUM:], label='$V_z$')
        ax[1][dron-1].plot(time[NUM:], vyaw[NUM:], label='$W_z$')
        ax[1][dron-1].legend(loc='center left', bbox_to_anchor=(1, 0.5))
        ax[1][dron-1].set_ylabel('Velocidades')

        for i in ["kp", "kv", "kd"]:
            try:
                lamb = np.loadtxt(f"{path}/out/out_lambda_{i}_{dron}.txt")
                if np.any(lamb != 0):
                    ax[2][dron-1].plot(time[NUM:], lamb[NUM:],
                                       label="$\lambda_{"+i+"}$")
            except:
                pass
        ax[2][dron-1].legend(loc='center left', bbox_to_anchor=(1, 0.5))
        ax[2][dron-1].set_ylabel('Lambda')
        ax[2][dron-1].set_xlabel('Tiempo (s)')

        ax[3][dron-1].plot(time[NUM:], intx[NUM:], label='$I_x$')
        ax[3][dron-1].plot(time[NUM:], inty[NUM:], label='$I_y$')
        ax[3][dron-1].plot(time[NUM:], intz[NUM:], label='$I_z$')
        ax[3][dron-1].legend(loc='center left', bbox_to_anchor=(1, 0.5))
        ax[3][dron-1].set_ylabel('Integrales')

    fig.tight_layout()
    fig.savefig(f"{path}/out_velocidades_all.png",
                bbox_inches='tight', pad_inches=0.1)

    ax3d.title.set_text(f"Posiciones")
    for dron in range(1, DRONE_COUNT+1):
        x = np.loadtxt(f"{path}/out/out_X_{dron}.txt")
        y = np.loadtxt(f"{path}/out/out_Y_{dron}.txt")
        z = np.loadtxt(f"{path}/out/out_Z_{dron}.txt")
        ax3d.scatter(x, y, z, linewidth=0.5,
                     label=f"Drone {dron}", marker="<", alpha=.5)

    ax3d.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    ax3d.view_init(elev=30, azim=71, roll=0)

    fig3d.tight_layout()
    fig3d.savefig(f"{path}/out_posiciones_all.png",
                bbox_inches='tight', pad_inches=0.1)
    plt.show()
    exit()

# print(f"Drones {dron} - Lider {lider}")
err = np.loadtxt(f"{path}/out/out_errors_{dron}.txt")
err_pix = np.loadtxt(f"{path}/out/out_errors_pix_{dron}.txt")
time = np.loadtxt(f"{path}/out/out_time_{dron}.txt")
vx = np.loadtxt(f"{path}/out/out_Vx_{dron}.txt")
vy = np.loadtxt(f"{path}/out/out_Vy_{dron}.txt")
vz = np.loadtxt(f"{path}/out/out_Vz_{dron}.txt")
vyaw = np.loadtxt(f"{path}/out/out_Vyaw_{dron}.txt")
lamb = np.loadtxt(f"{path}/out/out_lambda_kp_{dron}.txt")
intx = np.loadtxt(f"{path}/out/out_integral_x_{dron}.txt")
inty = np.loadtxt(f"{path}/out/out_integral_y_{dron}.txt")
intz = np.loadtxt(f"{path}/out/out_integral_z_{dron}.txt")

NUM = 0
try:
    if len(vx) < 3:
        exit()
except:
    exit()
for i in range(2, len(vx)):
    if np.linalg.norm(vx[i]-vx[i-1]) > 1e-3 and err[i] < 10e5:
        NUM = i
        break


print(f"""
INFORMACIÓN DEL EXPERIMENTO PARA DRONE {dron} <==

Tiempo de vuelo: {time[-1]:5f} s
Error promedio: {np.mean(err[NUM:]):5f} pixeles
Velocidad media en x: {np.mean(vx[NUM:]):5f} u/s
Velocidad media en y: {np.mean(vy[NUM:]):5f} u/s
Velocidad media en z: {np.mean(vz[NUM:]):5f} u/s
Velocidad media en yaw: {np.mean(vyaw[NUM:]):5f} u/s
Lambda promedio: {np.mean(lamb[NUM:]):5f}

Error final: {err[-1]:5f} pixeles
Velocidad final en x: {vx[-1]:5f} u/s
Velocidad final en y: {vy[-1]:5f} u/s
Velocidad final en z: {vz[-1]:5f} u/s
Velocidad final en yaw: {vyaw[-1]:5f} u/s""" if lider == "1" else f"""
INFORMACIÓN DEL EXPERIMENTO PARA DRONE {dron} <==

Tiempo de vuelo: {time[-1]:5f} s
Velocidad media en x: {np.mean(vx[NUM:]):5f} u/s
Velocidad media en y: {np.mean(vy[NUM:]):5f} u/s
Velocidad media en z: {np.mean(vz[NUM:]):5f} u/s
Velocidad media en yaw: {np.mean(vyaw[NUM:]):5f} u/s
Lambda promedio: {np.mean(lamb[NUM:]):5f}

Velocidad final en x: {vx[-1]:5f} u/s
Velocidad final en y: {vy[-1]:5f} u/s
Velocidad final en z: {vz[-1]:5f} u/s
Velocidad final en yaw: {vyaw[-1]:5f} u/s""")

if lider == "1":
    # PRIMER PLOT
    fig, ax = plt.subplots(4, 1, figsize=(5, 10), num = f"Velocidades y errores para el dron {dron}")
    fig.suptitle(f"Velocidades y errores para el dron {dron}")

    ax[0].plot(time[NUM:], err[NUM:], "purple", label='Error (c)')
    err_pix = err_pix / max(err)
    ax[0].plot(time[NUM:], err_pix[NUM:], "r", label='|Error (px)|')
    ax[0].plot([time[NUM], time[-1]], [0, 0], "k--", label="y=0")
    ax[0].plot([time[NUM], time[-1]], [err[-1], err[-1]], "k--", label=f"y={err[-1]:5f}")
    box = ax[0].get_position()
    ax[0].set_position([box.x0, box.y0, box.width * 0.99, box.height])
    ax[0].legend(loc='center left', bbox_to_anchor=(1, 0.5), shadow=True)
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

    ax[3].plot(time[NUM:], intx[NUM:], label='$I_x$')
    ax[3].plot(time[NUM:], inty[NUM:], label='$I_y$')
    ax[3].plot(time[NUM:], intz[NUM:], label='$I_z$')
    ax[3].legend(loc='center left', bbox_to_anchor=(1, 0.5))
    ax[3].set_ylabel('Integrales')

    plt.tight_layout()
    plt.savefig(f"{path}/out_velocidades_{dron}.png",
                bbox_inches='tight', pad_inches=0.1)

    # SEGUNDO PLOT
    fig, ax = plt.subplots(2, 1, figsize=(5, 6))

    fig.suptitle(f"Comparación para el {dron}")

    oslist = os.listdir(f"{path}/img")
    filtered = [i for i in oslist if f"{dron}_" in i]
    imgs = sorted(filtered, key=lambda x: int(x.split(".")[0]))

    des = f"{path.parent.absolute()}/desired_{dron}f.jpg"

    ax[0].imshow(plt.imread(des))
    ax[0].set_ylabel("Imagen deseada")
    if len(imgs) > 0:
        ax[1].imshow(plt.imread(f"{path}/img/{imgs[-1]}"))
        ax[1].set_ylabel("Imagen final")

    [i.set_xticks([]) for i in ax]
    [i.set_yticks([]) for i in ax]

    plt.tight_layout()
    plt.savefig(f"{path}/out_imagenes_{dron}.png",
                bbox_inches='tight', pad_inches=0.1)

else:
    fig, ax = plt.subplots(4, 1, figsize=(5, 8), num=f"Velocidades y errores para el dron {dron}")
    fig.suptitle(f"Velocidades y errores para el dron {dron}")

    ax[0].plot(time[NUM:], err[NUM:], "purple", label='Error (c)')
    if np.any(err_pix != 0):
        err_pix = err_pix / max(err_pix) * max(err)
        ax[0].plot(time[NUM:], err_pix[NUM:], "r", label='|Error (px)|')
    ax[0].plot([time[NUM], time[-1]], [0, 0], "k--", label="y=0")
    ax[0].plot([time[NUM], time[-1]], [err[-1], err[-1]], "k--", label=f"y={err[-1]:5f}")
    box = ax[0].get_position()
    ax[0].set_position([box.x0, box.y0, box.width * 0.99, box.height])
    ax[0].legend(loc='center left', bbox_to_anchor=(1, 0.5), shadow=True)
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

    ax[3].plot(time[NUM:], intx[NUM:], label='$I_x$')
    ax[3].plot(time[NUM:], inty[NUM:], label='$I_y$')
    ax[3].plot(time[NUM:], intz[NUM:], label='$I_z$')
    ax[3].legend(loc='center left', bbox_to_anchor=(1, 0.5))
    ax[3].set_ylabel('Integrales')

    plt.tight_layout()
    plt.savefig(f"{path}/out_velocidades_{dron}.png",
                bbox_inches='tight', pad_inches=0.1)
plt.show()
