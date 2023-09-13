"""
This script is used to generate velocity and error plots.
The data is stored in the 'data' folder.
The plots are saved in the same folder with the name 'out_*.png'.

Created by: David Leonardo Ramírez Parada
Email: david.parada@cimat.mx
"""
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

from pathlib import Path

path = Path(__file__).parent.absolute()
plt.rcParams["figure.autolayout"] = True
DRONE_COUNT = range(1, 5)
COLORS = [
    "red",
    "green",
    "blue",
    "orange",
    "yellow",
    "black",
    "pink",
    "purple",
    "cyan",
    "brown",
]

print(f"Plots para {DRONE_COUNT} drones")

forcount = (
    ("Feedback Control Error", "out_errors_", "Error"),
    ("Normalized Pixel Error", "out_errors_pix_", "Error (px)"),
    ("Velocities", "out_V", "$V"),
    ("Adaptive Gains", "out_lambda_", "Gain $"),
    ("Super-twisting support", "out_integral_", "$I"),
)


for title, txt, label in forcount:
    print(f"Plotting {title} for all drones with {txt}*")
    labArray = []
    # fig, ax = plt.subplots(
    #     1,
    #     5,
    #     sharex=True,
    #     num=f"{title}",
    #     # figsize is variable depending on the number of drones
    #     figsize=(3 * len(DRONE_COUNT), 5),
    # )
    fig = plt.figure(num=f"{title}", figsize=(3 * len(DRONE_COUNT), 3))
    # the first four cols are bigger than the last one
    # gs = fig.add_gridspec(5, 1) does not work because it is 5 rows and 1 col
    gs = fig.add_gridspec(
        ncols=5, nrows=1, width_ratios=[2, 2, 2, 2, 0.1]
    )  # , height_ratios=[2,])
    ax = []
    for i in range(4):
        ax.append(fig.add_subplot(gs[0, i]))
    ax.append(fig.add_subplot(gs[0, 4]))

    ax = np.array(ax).flatten()

    ax[-1].axis("off")
    ax[-1].axis("tight")

    # set fig title
    fig.suptitle(title)

    for drone in range(1, 5):
        ax[drone - 1].title.set_text(
            f"Drone {drone} {'(Leader)' if drone in (0,1) else '(Follower)'}"
        )
        time = np.loadtxt(f"{path}/out/out_time_{drone}.txt")
        ax[drone - 1].set_xticks(np.linspace(0, time[-1], 5, endpoint=True))
        # set equal axis for all plots
        ax[drone - 1].set_xlim([0, time[-1]])
        if "Vel" in title:
            x = np.vstack(
                (
                    np.loadtxt(f"{path}/out/{txt}x_{drone}.txt"),
                    np.loadtxt(f"{path}/out/{txt}y_{drone}.txt"),
                    np.loadtxt(f"{path}/out/{txt}z_{drone}.txt"),
                    np.loadtxt(f"{path}/out/{txt}yaw_{drone}.txt"),
                ),
            )
        elif "Super" in title:
            x = np.vstack(
                (
                    np.loadtxt(f"{path}/out/{txt}x_{drone}.txt"),
                    np.loadtxt(f"{path}/out/{txt}y_{drone}.txt"),
                    np.loadtxt(f"{path}/out/{txt}z_{drone}.txt"),
                ),
            )
        elif "Adaptive" in title:
            x = np.vstack(
                (
                    np.loadtxt(f"{path}/out/{txt}kp_{drone}.txt"),
                    np.loadtxt(f"{path}/out/{txt}kv_{drone}.txt"),
                    np.loadtxt(f"{path}/out/{txt}kd_{drone}.txt"),
                )
            )
        else:
            x = np.loadtxt(f"{path}/out/{txt}{drone}.txt")

        print(
            f"Drone {drone} -> {x.shape} -> time: {time.shape} -> txt: {txt}{drone}.txt"
        )
        # exit()
        if len(x.shape) == 1:
            ax[drone - 1].plot(time, x, color=COLORS[0])
        else:
            for i in range(x.shape[0]):
                ax[drone - 1].plot(time, x[i], color=COLORS[i])

    if "Vel" in title:
        labArray.append(f"{label}_x$ (m/s)")
        labArray.append(f"{label}_y$ (m/s)")
        labArray.append(f"{label}_z$ (m/s)")
        labArray.append(f"{label}_" + "{" + "yaw" + "}" + "$ (m/s)")
    elif "Super" in title:
        labArray.append(f"{label}_x$")
        labArray.append(f"{label}_y$")
        labArray.append(f"{label}_z$")
    elif "Adaptive" in title:
        labArray.append(f"{label}" + "{" + "kp" + "}$")
        labArray.append(f"{label}" + "{" + "kv" + "}$")
        labArray.append(f"{label}" + "{" + "kd" + "}$")
    else:
        labArray.append(f"{label}")

    # set all labels from the other plots in the ax[-1] space
    ax[-1].legend(
        [Line2D([0], [0], color=COLORS[i]) for i in range(10)],
        labArray,
        loc="center right",
    )
    fig.savefig(f"{path}/{txt}.png", bbox_inches="tight", pad_inches=0.1, dpi=300, transparent=True)
    fig.savefig(f"{path}/{txt}.svg", bbox_inches="tight", transparent=True, dpi=300)


# ================================================================================

# if 3 not in DRONE_COUNT or 4 not in DRONE_COUNT:
#     fig, ax = plt.subplots(
#         4,
#         len(DRONE_COUNT),
#         sharex=True,
#         num=f"Velocidades y errores para todos los drones",
#         # figsize is variable depending on the number of drones
#         figsize=(4 * len(DRONE_COUNT), 5),
#     )
# else:
#     fig, ax = plt.subplots(
#         5,
#         len(DRONE_COUNT),
#         sharex=True,
#         num=f"Velocidades y errores para todos los drones",
#         # figsize is variable depending on the number of drones
#         figsize=(4 * len(DRONE_COUNT), 5),
#     )
# ax.shape = (-1, len(DRONE_COUNT))

# for index, dron in enumerate(DRONE_COUNT):
#     err = np.loadtxt(f"{path}/out/out_errors_{dron}.txt")
#     err_pix_real = np.loadtxt(f"{path}/out/out_errors_pix_{dron}.txt")
#     time = np.loadtxt(f"{path}/out/out_time_{dron}.txt")
#     vx = np.loadtxt(f"{path}/out/out_Vx_{dron}.txt")
#     vy = np.loadtxt(f"{path}/out/out_Vy_{dron}.txt")
#     vz = np.loadtxt(f"{path}/out/out_Vz_{dron}.txt")
#     vyaw = np.loadtxt(f"{path}/out/out_Vyaw_{dron}.txt")
#     intx = np.loadtxt(f"{path}/out/out_integral_x_{dron}.txt")
#     inty = np.loadtxt(f"{path}/out/out_integral_y_{dron}.txt")
#     intz = np.loadtxt(f"{path}/out/out_integral_z_{dron}.txt")
#     execution_data = np.loadtxt(f"{path}/out/execution_data_{dron}.txt")

#     NUM = 0

#     print(
#         f"""
# Drone {dron}
# Tiempo total -> {time[-1]}
# Error final -> {err[-1]}, Max -> {max(err[NUM:])}
# Error pixels final -> {err_pix_real[-1]}, Max -> {max(err_pix_real[NUM:])}
# Velocidad final -> {vx[-1], vy[-1], vz[-1], vyaw[-1]}

# Excecution data: {np.mean(execution_data)}
# Max time: {np.max(execution_data)}
# Min time: {np.min(execution_data)}
#     """
#     )

#     ax[0][index].title.set_text(f"Drone {dron}")
#     ax[0][index].plot(time[NUM:], err[NUM:], "purple", label="Error (c)")
#     ax[0][index].plot(
#         [time[NUM], time[-1]],
#         [err[-1], err[-1]],
#         "--",
#         c="purple",
#         label=f"y={err[-1]:.3f}",
#         alpha=0.5,
#     )

#     box = ax[0][index].get_position()
#     ax[0][index].set_position([box.x0, box.y0, box.width * 0.99, box.height])
#     ax[0][index].legend(loc="center left", bbox_to_anchor=(1, 0.5), shadow=True)
#     ax[0][index].set_ylabel("Control Error")

#     err_pix = err_pix_real
#     ax[1][index].plot(time[NUM:], err_pix[NUM:], "r", label="Error (px)")
#     ax[1][index].plot(
#         [time[NUM], time[-1]],
#         [err_pix[-1], err_pix[-1]],
#         "r--",
#         label=f"y={err_pix[-1]:.3f}",
#         alpha=0.5,
#     )
#     box = ax[1][index].get_position()
#     ax[1][index].set_position([box.x0, box.y0, box.width * 0.99, box.height])
#     ax[1][index].legend(loc="center left", bbox_to_anchor=(1, 0.5), shadow=True)
#     ax[1][index].set_ylabel("Normalized Pixel Error")

#     ax[2][index].plot(time[NUM:], vx[NUM:], label="$V_x$")
#     ax[2][index].plot(time[NUM:], vy[NUM:], label="$V_y$")
#     ax[2][index].plot(time[NUM:], vz[NUM:], label="$V_z$")
#     ax[2][index].plot(time[NUM:], vyaw[NUM:], label="$W_z$")
#     ax[2][index].legend(loc="center left", bbox_to_anchor=(1, 0.5))
#     ax[2][index].set_ylabel("Velocidades")

#     for index_i, i in zip(["kvp", "kvi", "kw"], ["kp", "kv", "kd"]):
#         try:
#             lamb = np.loadtxt(f"{path}/out/out_lambda_{i}_{dron}.txt")
#             if np.any(lamb != 0):
#                 ax[3][index].plot(
#                     time[NUM:], lamb[NUM:], label="$\lambda_{" + index_i + "}$"
#                 )
#                 ax[3][index].plot(
#                     [time[NUM], time[-1]],
#                     [lamb[-1], lamb[-1]],
#                     "k--",
#                     label=f"y={lamb[-1]:5f}",
#                     alpha=0.5,
#                 )
#         except:
#             pass
#     ax[3][index].legend(loc="center left", bbox_to_anchor=(1, 0.5))
#     ax[3][index].set_ylabel("Lambda")
#     ax[3][index].set_xlabel("Tiempo (s)")

#     if dron not in (1,2):
#         ax[4][index].plot(time[NUM:], intx[NUM:], label="$I_x$")
#         ax[4][index].plot(time[NUM:], inty[NUM:], label="$I_y$")
#         ax[4][index].plot(time[NUM:], intz[NUM:], label="$I_z$")
#         ax[4][index].legend(loc="center left", bbox_to_anchor=(1, 0.5))
#         ax[4][index].set_ylabel("Integrales")

# fig.tight_layout()
# print(f"Saving plot in {path}/out_velocidades.png")
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
