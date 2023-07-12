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

colors = ["cyan", "blue", "green", "orange", "purple"]

DRONE_COUNT = 5
if len(sys.argv) > 1:
    DRONE_COUNT = np.array(sys.argv[1:], dtype=int)
else:
    DRONE_COUNT = np.array([i for i in range(1, DRONE_COUNT + 1)])

print(f"Plots para {DRONE_COUNT} drones")

fig, ax = plt.subplots(
    1,
    1,
    sharex=True,
    num=f"3D representation of the drones",
    # figsize is variable depending on the number of drones
    figsize=(8, 8),
    subplot_kw=dict(projection="3d"),
)

deseada_drone1 = [2.5, -0.5, 1.4]
deseada_drone2 = [2.5, 0.5, 1.4]
deseada_drone3 = [1.5, 0.5, 1.5]
deseada_drone4 = [1.5, -0.5, 1.3]
deseada_drone5 = [0.5, 0, 1.4]

deseadas = np.array(
    [deseada_drone1, deseada_drone2, deseada_drone3, deseada_drone4, deseada_drone5]
)

deseada_drone1_1 = [-2, -0.5, 1.4]
deseada_drone2_1 = [-2, 0.5, 1.4]
deseada_drone3_1 = [-3, 0.5, 1.5]
deseada_drone4_1 = [-3, -0.5, 1.3]
deseada_drone5_1 = [-4, 0, 1.4]
deseadas_1 = np.array(
    [
        deseada_drone1_1,
        deseada_drone2_1,
        deseada_drone3_1,
        deseada_drone4_1,
        deseada_drone5_1,
    ]
)

for index, dron in enumerate(DRONE_COUNT):
    x = np.loadtxt(f"{path}/out/out_X_{dron}.txt")
    y = np.loadtxt(f"{path}/out/out_Y_{dron}.txt")
    z = np.loadtxt(f"{path}/out/out_Z_{dron}.txt")
    yaw = np.loadtxt(f"{path}/out/out_YAW_{dron}.txt")

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
Norm error final -> {np.linalg.norm([x[-1] - deseadas[index][0], y[-1] - deseadas[index][1], z[-1] - deseadas[index][2]])}
    """
    )

    ax.plot3D(
        deseadas[dron - 1][0],
        deseadas[dron - 1][1],
        deseadas[dron - 1][2],
        "*",
        linewidth=0.5,
        alpha=0.5,
        color=colors[dron - 1],
    )
    ax.plot3D(
        deseadas_1[dron - 1][0],
        deseadas_1[dron - 1][1],
        deseadas_1[dron - 1][2],
        "*",
        linewidth=0.5,
        alpha=0.5,
        color=colors[dron - 1],
    )

    ax.plot3D(
        x,
        y,
        z,
        ".-",
        linewidth=0.5,
        label=f"Drone {dron}",
        alpha=0.5,
        color=colors[index],
    )
    ax.plot3D(x[0], y[0], z[0], "o", linewidth=0.5, color="red")
    ax.plot3D(x[-1], y[-1], z[-1], "o", linewidth=0.5, color="green")


# set azimutal angle
ax.view_init(elev=20, azim=-170, roll=0)

ax.legend(loc="center left", bbox_to_anchor=(1, 0.5))

# equal axis
ax.set_aspect("equal")
fig.tight_layout()
fig.savefig(f"{path}/out_plot3d.png", bbox_inches="tight", pad_inches=0.1)

plt.show()
