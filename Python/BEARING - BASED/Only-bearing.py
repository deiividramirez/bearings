from mpl_toolkits.mplot3d.axes3d import Axes3D
from matplotlib import animation
import matplotlib.pylab as plt
import numpy as np

from pathlib import Path
path = Path(__file__).parent.absolute()

P1 = 1
P2 = 2

# Colores y etiquetas de los drones
colores = ["red", "green", "blue", "orange",
           "purple", "black", "yellow", "pink"]

labels = [f"Dron {i}" if i != P1 and i !=
          P2 else f"Dron {i}*" for i in range(8)]
trayec = ["gray" if i != P1 and i != P2 else "green" for i in range(8)]
colores[P1] = "green"
colores[P2] = "green"

# VARIBALES PARA CONTROL DE GUARDADO
# ----------------------------------------------------------------------------------------------------------------------

# Guardar en gif
save = True
save = False

# Mostrar estadisticas
estad = True
# estad = False

# Mostrar animación
anim = True
# anim = False

# ----------------------------------------------------------------------------------------------------------------------


def ortProj(x: np.array):
    """
    Función la cual calcula la matriz de proyección ortogonal de un vector

    Parámetros:
        x: np.array

    Retorno:
        Matriz de tipo np.array
    """
    m = len(x)
    x = x.reshape(m, 1)
    return np.eye(m) - np.dot(x, x.T) / np.linalg.norm(x)**2 if np.linalg.norm(x) != 0 else np.zeros((m, m))


def normalize(x: np.array):
    """
    Función la cual normaliza un vector

    Parámetros:
        x: np.array

    Retorno:
        Vector de tipo np.array
    """
    return x / np.linalg.norm(x) if np.linalg.norm(x) != 0 else np.zeros(x.shape)

# CONFIGURACIÓN INICIAL
# ----------------------------------------------------------------------------------------------------------------------


# Dimensión y número de drones
d = 3
n = 4

gijA = np.zeros((n, n, d))

gijA[0, 2, :] = np.array([0, -1, 0])
gijA[0, 1, :] = np.array([-1, -1, 0])
gijA[3, 2, :] = np.array([1, -1, 0])
gijA[3, 1, :] = np.array([0, -1, 0])

for i in range(n):
    for j in range(n):
        gijA[i, j, :] = normalize(gijA[i, j, :])
print(f"Bearings: {gijA}\n")
gijA[0, 3] = np.array([0, 0, 0])
gijA[3, 0] = np.array([0, 0, 0])
# exit()


# CONDICIONES INICIALES
# ----------------------------------------------------------------------------------------------------------------------

lider_de_ac = np.loadtxt(f"{path}/src/lider1_ace.dat", skiprows=1)
lider_iz_ac = np.loadtxt(f"{path}/src/lider2_ace.dat", skiprows=1)

x = np.array(
    [
        [8, 2, 11],
        [-5, 0, 10],
        [5, 0, 10],
        [6, 3, 12],
    ],
    dtype=float,
)

# x[0][0] = np.random.uniform(0, 10)
# x[0][1] = np.random.uniform(150, 160)
# x[0][2] = np.random.uniform(10, 20)

# x[3][0] = np.random.uniform(0, 10)
# x[3][1] = np.random.uniform(150, 160)
# x[3][2] = np.random.uniform(10, 20)


# Vector de velocidades iniciales cero
v = np.zeros((n, d))
error = np.zeros(n)

# Arrays para guardar la posición y velocidades de los drones
arrx = [x]
arrv = [v]
errorv = [error]
tempv = [np.zeros((n, d))]
print(f"Posiciones iniciales: {x} \n")
print(f"Velocidades iniciales: {v} \n")


# PLOTEO CONDICIONES INICIALES SI: estad = True
# ----------------------------------------------------------------------------------------------------------------------

# if estad:
#     fig = plt.figure()
#     ax = plt.axes(projection='3d')
#     plt.subplots_adjust(right=0.8)
#     ax.set_title("Condiciones iniciales")
#     ax.set_box_aspect([1, 1, 1])
#     for i in range(n):
#         for j in range(n):
#             if i != P1 and i != P2:
#                 ax.plot3D([x[i, 0], x[i, 0] + gijA[i, j, 0]], [x[i, 1], x[i, 1] + gijA[i, j, 1]],
#                           [x[i, 2], x[i, 2] + gijA[i, j, 2]], color=colores[j], alpha=0.5)
#         ax.plot3D([x[i, 0]], [x[i, 1]], [x[i, 2]],
#                   'o', color=colores[i], alpha=0.5, label=labels[i])
#     ax.legend(bbox_to_anchor=(1.1, 0.5), loc="center left", borderaxespad=0)
#     if save:
#         plt.savefig("./src/out/condiciones_iniciales.png",
#                     dpi=300, bbox_inches='tight', transparent=True)


# CONFIGURACIÓN DE TIEMPO DE EJECUCIÓN Y dt
# ----------------------------------------------------------------------------------------------------------------------

tmax = 20
dt = 0.01
t = [i * dt for i in range(int(tmax / dt))]


# SIMULACIÓN
# ----------------------------------------------------------------------------------------------------------------------

# Ganancias del controlador
Kp = 5
Kv = 0.1

# Simulación con Euler
vnew = np.zeros((n, d))
vB = np.zeros((n, d))
u = [np.zeros((n, d))]
for num, tt in enumerate(t):
    error = np.zeros(n)
    temp = np.zeros((n, d))  
    for i in range(n):
        if i != P1 and i != P2:

            # sumaP = np.zeros((d, d))
            # for j in range(n):
            #     if i != j:
            #         sumaP += ortProj(gijA[i, j, :])

            suma1 = np.zeros(d)
            suma2 = np.zeros(d)
            deriv = np.zeros(d)
            for j in range(n):
                if i != j and np.any(gijA[i, j, :] != 0):
                    # deriv += ortProj(normalize(x[j]-x[i])) @ (v[j, :] - v[i, :]) / np.linalg.norm(x[j]-x[i])
                    
                    # BEARING-ONLY FORMATION TRACKING CONTROL OF MULTIAGENT SYSTEMS - SHIYU ZHAO
                    # suma1 += normalize(x[j]-x[i]) - gijA[i, j, :]
                    suma2 -= ortProj(normalize(x[j]-x[i])) @ (gijA[i, j, :])

                    error[i] += np.linalg.norm(normalize(x[j] - x[i]) - gijA[i, j, :])

                # print(f"Error {i},{j}: {errorc}, {normalize(x[j]-x[i])}, {gijA[i, j, :]}")


            temp[i, :] = (tempv[-1][i, :] + dt * np.sign(suma1+suma2))

            vB[i, :] = Kp*np.abs(suma1+suma2)**(1/2)*np.sign(suma1+suma2) + Kv*temp[i, :]
            # vB[i, :] = Kp*(suma1 + suma2) + Kv*temp[i, :]
            # vB[i, :] = Kp*(suma1 + suma2) + 1 * deriv + Kv*temp[i, :]
            # vB[i, :] = Kp*(suma1 + suma2 )
        # else:
        #     if i == P2:
        #         # vnew[i, :] = lider_iz_ac[num]
        #         vnew[i, :] = 0
        #     else:
        #         # vnew[i, :] = lider_de_ac[num]
        #         vnew[i, :] = 0

    tempv.append(temp)

    v = v + dt * vnew
    for i in range(n):
        if i != P1 and i != P2:
            v[i, :] = vB[i, :]
            vnew[i, :] = vB[i, :]
        else:
            v[i, :] = 0#np.sin(tt/3)
            vnew[i, :] = 0#np.sin(tt/3)
    x = x + dt * v

    arrx.append(x)
    arrv.append(v)
    errorv.append(error)

print(f"Posiciones finales: {x} \n")

# Conversión a numpy array para poder plotear
arrx = np.array(arrx)
arrv = np.array(arrv)
errorv = np.array(errorv)
tempv = np.array(tempv)


# PLOTEO DE VELOCIDADES SI: estad = True
# ----------------------------------------------------------------------------------------------------------------------
if estad:
    fig1, ax1 = plt.subplots(3, 1, figsize=(5, 8), num=f"Velocidades, error e integración", sharex=True)

    ax1[0].set_title("Velocidades")
    plt.subplots_adjust(right=0.75)
    for i in range(n):
        if i != P1 and i != P2:
            ax1[0].plot(
                t,
                arrv[1:, i],
                # color=colores[i],
                # alpha=(1, .8, .6),
                label=(labels[i]+" x", labels[i]+" y", labels[i]+" z"),
            )
    ax1[0].plot(
        t,
        arrv[1:, P1],
        color=colores[P1],
        alpha=.5,
        label=("Desired x", "Desired y", "Desired z"),
    )

    ax1[0].set_xlabel("Tiempo [s]")
    ax1[0].set_ylabel("Velocidad [m/s]")
    ax1[0].legend(bbox_to_anchor=(1.02, 0.5), loc="center left", borderaxespad=0)

    ax1[1].set_title("Error")
    plt.subplots_adjust(right=0.75)
    for i in range(n):
        if i != P1 and i != P2:
            ax1[1].plot(t, errorv[1:, i], color=colores[i], alpha=1, label=labels[i])
            ax1[1].plot(
                [0, tmax],
                [errorv[-1, i], errorv[-1, i]],
                "--",
                color=colores[i],
                label=f"y = {errorv[-1, i]:.3f}",
            )
    ax1[1].set_xlabel("Tiempo [s]")
    ax1[1].set_ylabel("Error [m]")
    ax1[1].legend(bbox_to_anchor=(1.02, 0.5), loc="center left", borderaxespad=0)

    ax1[2].set_title("Integración de bearings")
    plt.subplots_adjust(right=0.75)
    for i in range(n):
        if i != P1 and i != P2:
            ax1[2].plot(
                t,
                np.mean(tempv[1:, i], axis=1),
                color=colores[i],
                alpha=1,
                label=labels[i],
            )

    ax1[2].set_xlabel("Tiempo [s]")
    ax1[2].set_ylabel("Integración [rad]")
    ax1[2].legend(bbox_to_anchor=(1.02, 0.5), loc="center left", borderaxespad=0)

    plt.tight_layout()

    # if save:
    #     plt.savefig(
    #         "./src/out/velocidades.png", dpi=300, bbox_inches="tight", transparent=True
    #     )


# ANIMACIÓN DE LAS POSICIONES
# ----------------------------------------------------------------------------------------------------------------------
def animate(num: int):
    # Configuración del plot
    ax.cla()
    num = 10 * num % len(t)
    ax.set_title(f"Tiempo: {t[num]:.2f}s. Error total: {np.sum(errorv[num]):.5f}m")
    ax.set_xlim3d(np.min(arrx[num, :, 0]) - 1, np.max(arrx[num, :, 0]) + 1)
    ax.set_ylim3d(np.min(arrx[num, :, 1]) - 1, np.max(arrx[num, :, 1]) + 1)
    ax.set_zlim3d(np.min(arrx[num, :, 2]) - 1, np.max(arrx[num, :, 2]) + 1)
    ax.set_box_aspect([1, 1, 1])

    # Recorrer todos los drones
    for i in range(n):
        if i != P1 or i != P2:
            # Ploteo de los puntos iniciales
            ax.plot3D(
                [arrx[1, i, 0]],
                [arrx[1, i, 1]],
                [arrx[1, i, 2]],
                "o",
                color=trayec[i],
                alpha=1,
            )

            # Ploteo de los puntos finales
            ax.plot3D(
                [arrx[-1, i, 0]],
                [arrx[-1, i, 1]],
                [arrx[-1, i, 2]],
                "o",
                color=trayec[i],
                alpha=1,
            )

        for j in range(n):
            # Ploteo de los bearings
            if i != P1 and i != P2 and np.any(gijA[i, j, :] != 0):
                temporal = gijA[i, j, :]

                ax.plot3D(
                    [arrx[num, i, 0], arrx[num, i, 0] + temporal[0]],
                    [arrx[num, i, 1], arrx[num, i, 1] + temporal[1]],
                    [arrx[num, i, 2], arrx[num, i, 2] + temporal[2]],
                    color="purple",
                    alpha=1,
                )

        temporal = arrv[num, i, :]
        ax.plot3D(
            [arrx[num, i, 0], arrx[num, i, 0] + temporal[0]],
            [arrx[num, i, 1], arrx[num, i, 1] + temporal[1]],
            [arrx[num, i, 2], arrx[num, i, 2] + temporal[2]],
            color="k",
            alpha=1,
        )

        # Ploteo de los drones
        ax.plot3D(
            [arrx[num, i, 0]],
            [arrx[num, i, 1]],
            [arrx[num, i, 2]],
            "o",
            color=colores[i],
            alpha=1,
            label=labels[i],
        )

        # Ploteo de la trayectoria dada por los drones
        ax.plot3D(
            arrx[1:num, i, 0],
            arrx[1:num, i, 1],
            arrx[1:num, i, 2],
            "--",
            color=trayec[i],
            alpha=0.8,
        )

    ax.legend(bbox_to_anchor=(1.1, 0.5), loc="center left", borderaxespad=0)
    plt.tight_layout()

# Animación
if anim:
    fig = plt.figure(figsize=(6, 6), num=f"Simulación")
    ax = fig.add_subplot(111, projection="3d")

    # ax.azim = -15
    # ax.elev = 15
    # ax.dist = 8
    ax.set_title("Simulación")
    plt.subplots_adjust(right=0.8)
    line_ani = animation.FuncAnimation(
        fig, animate, interval=1, frames=len(t) // 10, repeat=False
    )
    # animate(-1)

    # GUARDAR SIMULACIÓN EN ARCHIVO .gif
    # ----------------------------------------------------------------------------------------------------------------------

    if save:
        print("\nGuardando animación")
        f = "./src/out/formation.gif"
        writergif = animation.PillowWriter(fps=len(t) / (tmax + 1))
        line_ani.save(f, writer=writergif)
        print("Se guardó.")

if estad or anim:
    plt.show()
