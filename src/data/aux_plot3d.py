"""
This script is used to generate velocity and error plots.
The data is stored in the 'data' folder.
The plots are saved in the same folder with the name 'out_*.png'.

Created by: David Leonardo Ramírez Parada
Email: david.parada@cimat.mx
"""

import matplotlib.pyplot as plt
import matplotlib
import numpy as np
import sys
import os

font = {'size'   : 11}
matplotlib.rc('font', **font)
plt.rcParams["figure.autolayout"] = True

from pathlib import Path


def rotMat(yaw):
    return np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1],
        ]
    )


path = Path(__file__).parent.absolute()

colors = ["cyan", "blue", "green", "orange", "purple"]

DRONE_COUNT = 4
if len(sys.argv) > 1:
    DRONE_COUNT = np.array(sys.argv[1:], dtype=int)
else:
    DRONE_COUNT = np.array([i for i in range(1, DRONE_COUNT + 1)])

print(f"Plots para {DRONE_COUNT} drones")

WINDOW = np.array(
    [
        [0, 1.2, 0.7],
        [0, -1.2, 0.7],
        [0, 1.2, 2.2],
        [0, -1.2, 2.2],
    ]
)

fig, ax = plt.subplots(
    1,
    1,
    sharex=True,
    # num=f"3D representation of the drones",
    # figsize is variable depending on the number of drones
    figsize=(10, 8),
    subplot_kw=dict(projection="3d"),
)
# fig.set_size_inches(12, 4, forward=True)

# echo "bebop2" && echo
# "DRON 1" && rosrun placing_iris placing_iris bebop2_1 -2 -.5 1.4 0 && echo
# "DRON 2" && rosrun placing_iris placing_iris bebop2_2 -2 .5 1.4 0 && echo
# "DRON 3" && rosrun placing_iris placing_iris bebop2_3 -3 0.2 1.5 0 && echo
# "DRON 4" && rosrun placing_iris placing_iris bebop2_4 -3 -.2 1.4 0
# ❯ echo "bebop2" && echo
# "DRON 1" && rosrun placing_iris placing_iris bebop2_1 2 -.5 1.4 0 && echo
# "DRON 2" && rosrun placing_iris placing_iris bebop2_2 2 .5 1.4 0 && echo
# "DRON 3" && rosrun placing_iris placing_iris bebop2_3 1 0.2 1.5 0 && echo
# "DRON 4" && rosrun placing_iris placing_iris bebop2_4 1 -.2 1.4 0

deseada_drone1 = [2, -0.5, 1.4]
deseada_drone2 = [2, 0.5, 1.4]
deseada_drone3 = [1, 0.2, 1.5]
deseada_drone4 = [1, -0.2, 1.4]
# deseada_drone5 = [-4, 0, 1.4]

deseadas = np.array(
    [
        deseada_drone1,
        deseada_drone2,
        deseada_drone3,
        deseada_drone4,
        # deseada_drone5,
    ]
)
deseada_drone1_1 = [-2, -0.5, 1.4]
deseada_drone2_1 = [-2, 0.5, 1.4]
deseada_drone3_1 = [-3, 0.2, 1.5]
deseada_drone4_1 = [-3, -0.2, 1.4]
# deseada_drone5_1 = [-4, 0, 1.4]
deseadas_1 = np.array(
    [
        deseada_drone1_1,
        deseada_drone2_1,
        deseada_drone3_1,
        deseada_drone4_1,
        # deseada_drone5_1,
    ]
)

ax.scatter(
    deseadas[:, 0],
    deseadas[:, 1],
    deseadas[:, 2],
    marker="*",
    linewidth=0.5,
    alpha=0.8,
    # color=colors[:],
    color="k",
    zorder=5,
)
ax.scatter(
    deseadas_1[:, 0],
    deseadas_1[:, 1],
    deseadas_1[:, 2],
    marker="*",
    # linewidth=0.5,
    alpha=0.8,
    # color=colors[:],
    color="k",
    label="Desired",
    zorder=5,
)
X = [np.loadtxt(f"{path}/out/out_X_{dron}.txt") for dron in DRONE_COUNT]
Y = [np.loadtxt(f"{path}/out/out_Y_{dron}.txt") for dron in DRONE_COUNT]
Z = [np.loadtxt(f"{path}/out/out_Z_{dron}.txt") for dron in DRONE_COUNT]
YAW = [np.loadtxt(f"{path}/out/out_YAW_{dron}.txt") for dron in DRONE_COUNT]

ax.scatter(
    X[0][0], Y[0][0], Z[0][0], marker="o", color="red", label="Initial", zorder=10
)
[
    ax.scatter(X[i][0], Y[i][0], Z[i][0], marker="o", color="red", zorder=10)
    for i in range(1, len(X))
]
ax.scatter(
    X[0][-1], Y[0][-1], Z[0][-1], marker="o", color="purple", label="Final", zorder=10
)
[
    ax.scatter(X[i][-1], Y[i][-1], Z[i][-1], marker="o", color="purple", zorder=10)
    for i in range(1, len(X))
]

for index, dron in enumerate(DRONE_COUNT):
    # x = np.loadtxt(f"{path}/out/out_X_{dron}.txt")
    # y = np.loadtxt(f"{path}/out/out_Y_{dron}.txt")
    # z = np.loadtxt(f"{path}/out/out_Z_{dron}.txt")
    # yaw = np.loadtxt(f"{path}/out/out_YAW_{dron}.txt")
    x = X[index]
    y = Y[index]
    z = Z[index]
    yaw = YAW[index]

    NUM = 0
    try:
        if len(x) < 3:
            exit()
    except:
        exit()

    print(
        f"""
Drone {dron}
Posición inicial -> {x[0], y[0], z[0], yaw[0]}
Posición final -> {x[-1], y[-1], z[-1], yaw[-1]}
Posición final deseada -> {deseada_drone1 if dron == 1 else deseada_drone2 if dron == 2 else deseada_drone3 if dron == 3 else deseada_drone4 if dron == 4 else deseada_drone5}
Error final -> ({x[-1] - deseadas[index][0]}, {y[-1] - deseadas[index][1]}, {z[-1] - deseadas[index][2]})
Norm error final -> {np.linalg.norm([x[-1] - deseadas[index][0], y[-1] - deseadas[index][1], z[-1] - deseadas[index][2]]):.8f}
    """
    )

    ax.plot3D(
        x,
        y,
        z,
        "-",
        linewidth=3,
        label=f"Drone {dron}",
        alpha=1,
        color=colors[index],
        zorder=1,
    )

    # draw yaw angle from point initial
    # yaw = [
    #     np.dot(0.2 * rotMat(yaw[i]), np.array([1, 0, 0])) for i in range(len(yaw))
    # ] + np.array([x, y, z]).T
    # line from x,y,z to yaw
    # ax.plot3D(
    #     yaw[:, 0],
    #     yaw[:, 1],
    #     yaw[:, 2],
    #     "-",
    #     linewidth=1,
    #     color=colors[index],
    #     zorder=2,
    # ) #this is not what i wanted
    # line from x,y,z to yaw
    # [ax.plot3D(
    #     [x[i], yaw[i][0]],
    #     [y[i], yaw[i][1]],
    #     [z[i], yaw[i][2]],
    #     "-",
    #     linewidth=1,
    #     color=colors[index],
    #     zorder=2,
    #     alpha = 0.5,                                
    # ) for i in range(0, len(yaw), 10)]
    # yaw = rotMat(yaw)
    # yaw = np.dot(yaw, np.array([1, 0, 0]))
    # yaw = yaw * 0.2
    # yaw = yaw + np.array([x[0], y[0], z[0]])
    # ax.plot3D(
    #     [x[0], yaw[0]],
    #     [y[0], yaw[1]],
    #     [z[0], yaw[2]],
    #     "-",
    #     linewidth=1,
    #     color=colors[index],
    #     zorder=2,
    # )

    # draw window
    ax.plot3D(
        [WINDOW[0][0], WINDOW[1][0], WINDOW[3][0], WINDOW[2][0], WINDOW[0][0]],
        [WINDOW[0][1], WINDOW[1][1], WINDOW[3][1], WINDOW[2][1], WINDOW[0][1]],
        [WINDOW[0][2], WINDOW[1][2], WINDOW[3][2], WINDOW[2][2], WINDOW[0][2]],
        "-",
        linewidth=1,
        color="black",
    )


# set azimutal angle
# ax.view_init(elev=35, azim=-40, roll=0)
ax.view_init(elev=20, azim=-80, roll=0)

# ax.legend(loc="center left", bbox_to_anchor=(1, 0.5))
# alittle bigger legend
# ax.legend(
#     loc="best", 
#     # bbox_to_anchor=(1, 0.5), 
#     shadow=True
# )

# equal axis
ax.set_aspect("equal")
xtemp = np.array([ax.get_xlim3d()[0], ax.get_xlim3d()[1]])
ax.set_xticks(np.linspace(xtemp[0], xtemp[1], 10))
ax.set_xticklabels(["{:.1f}".format(i) for i in np.linspace(xtemp[0], xtemp[1], 10)])
ytemp = np.array([ax.get_ylim3d()[0], ax.get_ylim3d()[1]])
ax.set_yticks(np.linspace(ytemp[0], ytemp[1], 5))
ax.set_yticklabels(["{:.1f}".format(i) for i in np.linspace(ytemp[0], ytemp[1], 5)])
ztemp = np.array([ax.get_zlim3d()[0], ax.get_zlim3d()[1]])
ax.set_zticks(np.linspace(ztemp[0], ztemp[1], 5))
ax.set_zticklabels(["{:.1f}".format(i) for i in np.linspace(ztemp[0], ztemp[1], 5)])

fig.tight_layout()
fig.savefig(
    f"{path}/sim_out_plot3d.png",
    bbox_inches="tight",
    pad_inches=0.1,
    transparent=True,
    dpi=300,
)

fig.savefig(
    f"{path}/sim_out_plot3d.svg",
    bbox_inches="tight",
    pad_inches=0.1,
    transparent=True,
    dpi=300,
)

plt.show()
