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
    1,
    1,
    sharex=True,
    num=f"Velocidades y errores para todos los drones",
    # figsize is variable depending on the number of drones
    figsize=(8, 8),
    subplot_kw=dict(projection="3d"),
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
    """
    )
    
    ax.plot3D(x, y, z, ".-", linewidth=0.5, label=f"Drone {dron}", alpha=0.5)
    ax.plot3D(
        x[0], y[0], z[0], "o", linewidth=0.5, alpha=0.5, color="red"
    )
    ax.plot3D(
        x[-1], y[-1], z[-1], "o", linewidth=0.5, alpha=0.5, color="green"
    )

    # Primer posición (2.5 -.5 1.6) y, (2.5 .5 1.6)
    ax.plot3D(
        [2.5, 2.5], [-.5, .5], [1.6, 1.6], "o", linewidth=0.5, alpha=0.5, color="black"
    )

    # Segunda posición (-2 -.5 1.5) y, (-2 .5 1.5)
    ax.plot3D(
        [-2, -2], [-.5, .5], [1.5, 1.5], "o", linewidth=0.5, alpha=0.5, color="black"
    )

    
# set azimutal angle
ax.view_init(elev=20, azim=-170, roll=0)

ax.legend(loc="center left", bbox_to_anchor=(1, 0.5))

# equal axis
ax.set_aspect("equal")
fig.tight_layout()
fig.savefig(f"{path}/out_velocidades.png", bbox_inches="tight", pad_inches=0.1)

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
