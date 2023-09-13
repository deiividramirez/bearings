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

if len(sys.argv) > 1:
    DRONE_COUNT = np.array(sys.argv[1:], dtype=int)
else:
    DRONE_COUNT = np.array([i for i in range(1, DRONE_COUNT + 1)])

print(f"Plots para {DRONE_COUNT} drones")

fig, ax = plt.subplots(
    4,
    len(DRONE_COUNT),
    sharex=True,
    num=f"Velocidades y errores para todos los drones",
    # figsize is variable depending on the number of drones
    figsize=(4 * len(DRONE_COUNT), 5),
)
ax.shape = (4, len(DRONE_COUNT))

for index, dron in enumerate(DRONE_COUNT):
    err = np.loadtxt(f"{path}/out/out_errors_{dron}.txt")
    err_pix_real = np.loadtxt(f"{path}/out/out_errors_pix_{dron}.txt")
    time = np.loadtxt(f"{path}/out/out_time_{dron}.txt")
    vx = np.loadtxt(f"{path}/out/out_Vx_{dron}.txt")
    vy = np.loadtxt(f"{path}/out/out_Vy_{dron}.txt")
    vz = np.loadtxt(f"{path}/out/out_Vz_{dron}.txt")
    vyaw = np.loadtxt(f"{path}/out/out_Vyaw_{dron}.txt")
    intx = np.loadtxt(f"{path}/out/out_integral_x_{dron}.txt")
    inty = np.loadtxt(f"{path}/out/out_integral_y_{dron}.txt")
    intz = np.loadtxt(f"{path}/out/out_integral_z_{dron}.txt")
    execution_data = np.loadtxt(f"{path}/out/execution_data_{dron}.txt")

    NUM = 0

    print(
        f"""
Drone {dron}
Tiempo total -> {time[-1]}
Error final -> {err[-1]}, Max -> {max(err[NUM:])}
Error pixels final -> {err_pix_real[-1]}, Max -> {max(err_pix_real[NUM:])}
Velocidad final -> {vx[-1], vy[-1], vz[-1], vyaw[-1]}

Excecution data: {np.mean(execution_data)}
Max time: {np.max(execution_data)}
Min time: {np.min(execution_data)}
    """
    )

    ax[0][index].title.set_text(f"Drone {dron}")
    if np.any(err_pix_real != 0):
        err_pix = (err_pix_real) / max(err_pix_real[NUM + 10 :]) * max(err[NUM:])
        # err_pix = err_pix_real
        ax[0][index].plot(time[NUM:], err_pix[NUM:], "r", label="Error (px)")
        ax[0][index].plot(
            [time[NUM], time[-1]],
            [err_pix[-1], err_pix[-1]],
            "r--",
            label=f"y={err_pix[-1]:.3f}",
            alpha=0.5,
        )
        ax[0][index].plot(
            [time[NUM], time[-1]],
            [err_pix_real[-1], err_pix_real[-1]],
            "r--",
            label=f"y={err_pix_real[-1]:.3f}",
            alpha=0.5,
        )

    ax[0][index].plot(time[NUM:], err[NUM:], "purple", label="Error (c)")
    ax[0][index].plot(
        [time[NUM], time[-1]],
        [err[-1], err[-1]],
        "--",
        c="purple",
        label=f"y={err[-1]:.3f}",
        alpha=0.5,
    )

    box = ax[0][index].get_position()
    ax[0][index].set_position([box.x0, box.y0, box.width * 0.99, box.height])
    ax[0][index].legend(loc="center left", bbox_to_anchor=(1, 0.5), shadow=True)
    ax[0][index].set_ylabel("Error Promedio")

    ax[1][index].plot(time[NUM:], vx[NUM:], label="$V_x$")
    ax[1][index].plot(time[NUM:], vy[NUM:], label="$V_y$")
    ax[1][index].plot(time[NUM:], vz[NUM:], label="$V_z$")
    ax[1][index].plot(time[NUM:], vyaw[NUM:], label="$W_z$")
    ax[1][index].legend(loc="center left", bbox_to_anchor=(1, 0.5))
    ax[1][index].set_ylabel("Velocidades")

    for index_i, i in zip(["kvp", "kvi", "kw"], ["kp", "kv", "kd"]):
        try:
            lamb = np.loadtxt(f"{path}/out/out_lambda_{i}_{dron}.txt")
            if np.any(lamb != 0):
                ax[2][index].plot(
                    time[NUM:], lamb[NUM:], label="$\lambda_{" + index_i + "}$"
                )
                ax[2][index].plot(
                    [time[NUM], time[-1]],
                    [lamb[-1], lamb[-1]],
                    "k--",
                    label=f"y={lamb[-1]:5f}",
                    alpha=0.5,
                )
        except:
            pass
    ax[2][index].legend(loc="center left", bbox_to_anchor=(1, 0.5))
    ax[2][index].set_ylabel("Lambda")
    ax[2][index].set_xlabel("Tiempo (s)")

    ax[3][index].plot(time[NUM:], intx[NUM:], label="$I_x$")
    ax[3][index].plot(time[NUM:], inty[NUM:], label="$I_y$")
    ax[3][index].plot(time[NUM:], intz[NUM:], label="$I_z$")
    ax[3][index].legend(loc="center left", bbox_to_anchor=(1, 0.5))
    ax[3][index].set_ylabel("Integrales")

fig.tight_layout()
print(f"Saving plot in {path}/out_velocidades.png")
exit()
# fig.savefig(f"{path}/out_velocidades.png", bbox_inches="tight", pad_inches=0.1)

########################################################################################

# fig3d, ax3d = plt.subplots(
#     1,
#     1,
#     subplot_kw=dict(projection="3d"),
#     num=f"Posiciones para todos los drones",
# )

# ax3d.title.set_text(f"Posiciones")
# for dron in np.array([i for i in range(1, 6)]):
#     try:
#         x = np.loadtxt(f"{path}/out/out_X_{dron}.txt")
#         y = np.loadtxt(f"{path}/out/out_Y_{dron}.txt")
#         z = np.loadtxt(f"{path}/out/out_Z_{dron}.txt")
#         ax3d.scatter(
#             x, y, z, linewidth=0.5, label=f"Drone {dron}", marker="<", alpha=0.5
#         )
#     except:
#         pass

# ax3d.legend(loc="center left", bbox_to_anchor=(1, 0.5))
# ax3d.view_init(elev=30, azim=71, roll=0)

# fig3d.tight_layout()
# fig3d.savefig(f"{path}/out_posiciones.png", bbox_inches="tight", pad_inches=0.1)

plt.show()
