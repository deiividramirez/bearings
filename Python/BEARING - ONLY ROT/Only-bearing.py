from mpl_toolkits.mplot3d.axes3d import Axes3D
from matplotlib import animation
import matplotlib.pylab as plt
import numpy as np

from pathlib import Path
path = Path(__file__).parent.absolute()

plt.rcParams["keymap.quit"] = ['ctrl+w', 'cmd+w', 'q']
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


def ortProj(x: np.array) -> np.array:
    """
    Función la cual calcula la matriz de proyección ortogonal de un vector

    Parámetros:
        x: np.array

    Retorno:
        Matriz de tipo np.array
    """
    m = len(x)
    x = x.reshape(m, 1)
    return (
        np.eye(3) - np.dot(x, x.T) / np.linalg.norm(x) ** 2
        if np.linalg.norm(x) != 0
        else np.zeros((m, m))
    )


def normalize(x: (np.array or list)) -> np.array:
    """
    Función la cual normaliza un vector

    Parámetros:
        x: np.array

    Retorno:
        Vector de tipo np.array
    """
    if len(x) == 3:
        return x / np.linalg.norm(x) if np.linalg.norm(x) != 0 else np.zeros(3)
    else:
        N = []
        for i in range(x.shape[0]):
            N.append(normalize(x[i]))
        return np.array(N)


def skewMatrix(x: np.array) -> np.array:
    """
    Función la cual calcula la matriz de producto cruz de un vector

    Parámetros:
        x: np.array

    Retorno:
        Matriz de tipo np.array
    """
    if len(x) == 3:
        return np.array([[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]])
    else:
        S = []
        for i in range(x.shape[0]):
            S.append(skewMatrix(x[i]))
        return np.array(S)


def skewVector(x: np.array) -> np.array:
    """
    Función la cual calcula el vector de producto cruz de un vector

    Parámetros:
        x: np.array

    Retorno:
        Vector de tipo np.array
    """
    if len(x) == 3:
        return np.array([x[2, 1], x[0, 2], x[1, 0]])
    else:
        S = []
        for i in range(x.shape[0]):
            S.append(skewVector(x[i]))
        return np.array(S)


def decomposeR(R: np.array, way: int = 3) -> np.array:
    """
    Función la cual calcula la descomposición de la matriz de rotación en los ángulos de Euler

    Parámetros:
        R: np.array

    Retorno:
        Vector de tipo np.array -> [psi: Angulo de rotación en el eje x
                                    theta: Angulo de rotación en el eje y,
                                    phi: Angulo de rotación en el eje z,]
    """
    if R.shape == (3, 3):
        if way == 1:
            theta = (
                -np.arcsin(R[2, 0])
                if R[2, 0] != 1
                else np.pi / 2
                if R[2, 0] == 1
                else -np.pi / 2
            )
            psi = np.arctan2(R[2, 1] / np.cos(theta), R[2, 2] / np.cos(theta))
            phi = np.arctan2(R[1, 0] / np.cos(theta), R[0, 0] / np.cos(theta))
        elif way == 2:
            theta = np.arctan2(-R[2, 0], np.sqrt(R[2, 1] ** 2 + R[2, 2] ** 2))
            psi = np.arctan2(R[2, 1], R[2, 2])
            phi = np.arctan2(R[1, 0], R[0, 0])
        elif way == 3:
            theta = (
                -np.arcsin(R[2, 0])
                if R[2, 0] != 1
                else np.pi / 2
                if R[2, 0] == 1
                else -np.pi / 2
            )
            psi = np.arctan2(R[2, 1], R[2, 2])
            phi = np.arctan2(R[1, 0], R[0, 0])

        return np.array([psi, theta, phi])
    else:
        phi = []
        theta = []
        psi = []
        for i in range(R.shape[0]):
            phi.append((DECOM := decomposeR(R[i], way))[0])
            theta.append(DECOM[1])
            psi.append(DECOM[2])
        return np.array([phi, theta, psi]).T


def composeR(psi: float, theta: float, phi: float) -> np.array:
    """
    Función la cual calcula la matriz de rotación a partir de los ángulos de Euler

    Parámetros:
        psi:   float -> Ángulo de rotación en el eje x
        theta: float -> Ángulo de rotación en el eje y
        phi:   float -> Ángulo de rotación en el eje z

    Retorno:
        Matriz de rotación tipo np.array
    """
    return (
        np.array(
            [
                [
                    np.cos(phi) * np.cos(theta),
                    np.cos(phi) * np.sin(theta) * np.sin(psi)
                    - np.sin(phi) * np.cos(psi),
                    np.cos(phi) * np.sin(theta) * np.cos(psi)
                    + np.sin(phi) * np.sin(psi),
                ],
                [
                    np.sin(phi) * np.cos(theta),
                    np.sin(phi) * np.sin(theta) * np.sin(psi)
                    + np.cos(phi) * np.cos(psi),
                    np.sin(phi) * np.sin(theta) * np.cos(psi)
                    - np.cos(phi) * np.sin(psi),
                ],
                [
                    -np.sin(theta),
                    np.cos(theta) * np.sin(psi),
                    np.cos(theta) * np.cos(psi),
                ],
            ]
        )
        if np.cos(theta) != 0
        else np.zeros((3, 3))
    )


def getFrames(Q: np.array) -> np.array:
    xw = np.zeros((Q.shape[0], 3, 3))
    for i in range(Q.shape[0]):
        xw[i] = Q[i] @ np.eye(3)
    return xw


# CONFIGURACIÓN INICIAL
# ----------------------------------------------------------------------------------------------------------------------

# Dimensión y número de drones
d = 3
n = 4

# Drones líderes
P1 = 1
P2 = 2

# Colores aleatorios pero claro y etiquetas de los n drones
colores = ["red", "blue", "green", "orange"]
# colores = [np.random.rand(3) for i in range(n)]

labels = [f"Dron {i}" if i != P1 and i != P2 else f"Dron {i}*" for i in range(n)]
trayec = ["gray" for i in range(n)]
colores[P1] = "black"
colores[P2] = "black"

gijA = np.zeros((n, n, d))

gijA[0, 2, :] = np.array([0, -1, 0])
gijA[0, 1, :] = np.array([-1, -1, 0])
gijA[3, 2, :] = np.array([1, -1, 0])
gijA[3, 1, :] = np.array([0, -1, 0])

for i in range(n):
    for j in range(n):
        gijA[i, j, :] = normalize(gijA[i, j, :])


# print(f"Bearings: {gijA}\n")
gijA[0, 3] = np.array([0, 0, 0])
gijA[3, 0] = np.array([0, 0, 0])


# CONDICIONES INICIALES
# ----------------------------------------------------------------------------------------------------------------------

x = np.array(
    [
        [8, 2, 11],
        [-5, 0, 10],
        [5, 0, 10],
        [6, 3, 12],
    ],
    dtype=float,
)


# Vector de velocidades iniciales cero
v = np.zeros((n, d))
w = np.zeros((n, d, d))

error = np.zeros(n)
integralSigno = np.zeros((n, d))

Q = np.array(
    [
        composeR(
            0,
            np.random.rand() * 2 * np.pi / 3 - np.pi /3,
            np.random.rand() * 2 * np.pi / 3 - np.pi /3,
        ),
        composeR(0, 0, 0),
        composeR(0, 0, 0),
        composeR(
            np.random.rand() * 2 * np.pi / 3 - np.pi /3,
            0,
            np.random.rand() * 2 * np.pi / 3 - np.pi /3,
        ),
    ]
)

xw = getFrames(Q)

# Arrays para guardar la posición y velocidades de los drones
arrx = [x]
arrxw = [xw]
arrv = [v]
arrw = [w]
arrQ = [Q]

errorv = [error]
arrIntSigno = [np.zeros((n, d))]

print(f"Posiciones iniciales: {x} \n")
print(f"Velocidades iniciales: {v} \n")

tmax = 50
dt = 0.01
t = [i * dt for i in range(int(tmax / dt))]

# SIMULACIÓN
# ----------------------------------------------------------------------------------------------------------------------

# Ganancias del controlador
Kp = 2
Kv = 0.1

Kw = 0.2

for num, tt in enumerate(t):
    error = np.zeros(n)
    # if num%500 == 0:
    #     arrIntSigno[-1] = np.zeros((n, d))
    for i in range(n):
        if i != P1 and i != P2:
            suma1 = np.zeros(d)
            suma2 = np.zeros(d)

            suma2_w = np.zeros((d, d))

            for j in range(n):
                if i != j and np.any(gijA[i, j, :] != 0):
                    # suma1 += (
                    #     (ACTUAL_BEAR := ( Q[i].T @ normalize(x[j] - x[i])))
                    #     - ((np.eye(3) + Q[i].T @ Q[j]) / 2 @ gijA[i, j, :])
                    # )
                    suma2 -= (
                        ortProj((ACTUAL_BEAR := (Q[i].T @ normalize(x[j] - x[i]))))
                        @ ((np.eye(3) + Q[i].T @ Q[j]) / 2)
                        @ gijA[i, j, :]
                    )
                    suma2_w += -(Q[j].T @ Q[i] - Q[i].T @ Q[j])

                    error[i] += np.linalg.norm(ACTUAL_BEAR - gijA[i, j, :])

            suma3 = suma1 + suma2
            integralSigno[i] = arrIntSigno[-1][i] + dt * np.sign(suma3)

            sum3_w = suma2_w

            v[i] = (
                Kp * np.abs(suma3) ** (0.5) * np.sign(suma3)
                + Kv * integralSigno[i, :]
                # suma3
            )

            w[i] = (
                Kw * sum3_w
                # (np.abs(sum3_w) ** (.5) * np.sign(sum3_w))
                # (sum3_w)
            )

        else:
            v[i] = np.array([0, 0, 0])
            # w[i] = skewMatrix([0, 0, -decomposeR(Q[i])[-1]])
            # w[i] = skewMatrix([.1, 0, 0])
            # w[i] = skewMatrix([.1, 0, -decomposeR(Q[i])[-1]])

        x[i] += dt * Q[i] @ v[i]
        Q[i] += dt * (Q[i] @ w[i])

    arrx.append(x.copy())
    arrxw.append(getFrames(Q))

    arrv.append(v.copy())
    arrw.append(w.copy())
    arrQ.append(Q.copy())

    errorv.append(error.copy())
    arrIntSigno.append(integralSigno.copy())

print(f"Posiciones finales: {x} \n")
# Conversión a numpy array para poder plotear
arrx = np.array(arrx)
arrxw = np.array(arrxw)

arrv = np.array(arrv)
arrw = np.array([skewVector(W) for W in arrw])
arrQ = np.array([decomposeR(Q) for Q in arrQ])
print(f"Rotaciones finales: {arrQ[-1]} \n")

errorv = np.array(errorv)
arrIntSigno = np.array(arrIntSigno)


# PLOTEO DE VELOCIDADES SI: estad = True
# ----------------------------------------------------------------------------------------------------------------------
if estad:
    fig1, ax1 = plt.subplots(
        4, 1, figsize=(6, 8), num=f"Velocidades, error e integración", sharex=True
    )

    ax1[0].set_title("Velocidades lineales")
    plt.subplots_adjust(right=0.75)
    for i in range(n):
        if i != P1 and i != P2:
            ax1[0].plot(
                t,
                arrv[1:, i],
                # color=colores[i],
                # alpha=(1, .8, .6),
                label=(labels[i] + " x", labels[i] + " y", labels[i] + " z"),
            )
    ax1[0].plot(
        t,
        arrv[1:, P1],
        color=colores[P1],
        alpha=0.5,
        label=("Desired x", "Desired y", "Desired z"),
    )

    ax1[0].set_xlabel("Tiempo [s]")
    ax1[0].set_ylabel("Velocidad [m/s]")
    ax1[0].legend(bbox_to_anchor=(1.02, 0.5), loc="center left", borderaxespad=0)

    ax1[1].set_title("Velocidades angulares")
    plt.subplots_adjust(right=0.75)
    for i in range(n):
        if i != P1 and i != P2:
            ax1[1].plot(
                t,
                arrw[1:, i],
                # color=colores[i],
                # alpha=(1, .8, .6),
                label=(labels[i] + " x", labels[i] + " y", labels[i] + " z"),
            )
    ax1[1].plot(
        t,
        arrv[1:, P1],
        color=colores[P1],
        alpha=0.5,
        label=("Desired x", "Desired y", "Desired z"),
    )

    ax1[1].set_xlabel("Tiempo [s]")
    ax1[1].set_ylabel("Velocidad [m/s]")
    ax1[1].legend(bbox_to_anchor=(1.02, 0.5), loc="center left", borderaxespad=0)


    ax1[2].set_title("Error")
    plt.subplots_adjust(right=0.75)
    for i in range(n):
        if i != P1 and i != P2:
            ax1[2].plot(t, errorv[1:, i], color=colores[i], alpha=1, label=labels[i])
            ax1[2].plot(
                [0, tmax],
                [errorv[-1, i], errorv[-1, i]],
                "--",
                color=colores[i],
                label=f"y = {errorv[-1, i]:.3f}",
            )
    ax1[2].set_xlabel("Tiempo [s]")
    ax1[2].set_ylabel("Error [m]")
    ax1[2].legend(bbox_to_anchor=(1.02, 0.5), loc="center left", borderaxespad=0)

    ax1[3].set_title("Integración de bearings")
    plt.subplots_adjust(right=0.75)
    for i in range(n):
        if i != P1 and i != P2:
            ax1[3].plot(
                t,
                arrIntSigno[1:, i],
                # color=colores[i],
                alpha=1,
                label=[f"{labels[i]} x", f"{labels[i]} y", f"{labels[i]} z"],
            )

    ax1[3].set_xlabel("Tiempo [s]")
    ax1[3].set_ylabel("Integración [rad]")
    ax1[3].legend(bbox_to_anchor=(1.02, 0.5), loc="center left", borderaxespad=0)

    plt.tight_layout()


    fig2, ax2 = plt.subplots(
        2, 3, figsize=(8, 5), num=f"Posiciones y ángulos", sharex=True
    )

    ax2[0, 0].set_title("X")
    # plt.subplots_adjust(right=0.75)
    for i in range(n):
        ax2[0, 0].plot(
            t,
            arrx[1:, i, 0],
            color=colores[i],
            alpha=1,
            label=labels[i],
        )

    ax2[0, 0].set_xlabel("Tiempo [s]")
    ax2[0, 0].set_ylabel("Pos [m]")

    ax2[0, 1].set_title("Y")
    for i in range(n):
        ax2[0, 1].plot(
            t,
            arrx[1:, i, 1],
            color=colores[i],
            alpha=1,
            label=labels[i],
        )

    ax2[0, 1].set_xlabel("Tiempo [s]")
    ax2[0, 1].set_ylabel("Pos [m]")

    ax2[0, 2].set_title("Z")
    plt.subplots_adjust(right=0.75)
    for i in range(n):
        ax2[0, 2].plot(
            t,
            arrx[1:, i, 2],
            color=colores[i],
            alpha=1,
            label=labels[i],
        )

    ax2[0, 2].set_xlabel("Tiempo [s]")
    ax2[0, 2].set_ylabel("Pos [m]")
    ax2[0, 2].legend(bbox_to_anchor=(1.1, 0.5), loc="center left", borderaxespad=0)

    ax2[1, 0].set_title("Roll")
    for i in range(n):
        ax2[1, 0].plot(
            t,
            arrQ[1:, i, 0],
            color=colores[i],
            alpha=1,
            label=labels[i],
        )

    ax2[1, 0].set_xlabel("Tiempo [s]")
    ax2[1, 0].set_ylabel("Áng [rad]")

    ax2[1, 1].set_title("Pitch")
    for i in range(n):
        ax2[1, 1].plot(
            t,
            arrQ[1:, i, 1],
            color=colores[i],
            alpha=1,
            label=labels[i],
        )

    ax2[1, 1].set_xlabel("Tiempo [s]")
    ax2[1, 1].set_ylabel("Áng [rad]")

    ax2[1, 2].set_title("Yaw")
    plt.subplots_adjust(right=0.75)
    for i in range(n):
        ax2[1, 2].plot(
            t,
            arrQ[1:, i, 2],
            color=colores[i],
            alpha=1 if i != P1 and i != P2 else 0.5,
            label=labels[i],
        )

    ax2[1, 2].set_xlabel("Tiempo [s]")
    ax2[1, 2].set_ylabel("Áng [rad]")
    ax2[1, 2].legend(bbox_to_anchor=(1.1, 0.5), loc="center left", borderaxespad=0)

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
        Qi = composeR(arrQ[num, i, 0], arrQ[num, i, 1], arrQ[num, i, 2])
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
                temporal = Qi @ gijA[i, j, :]

                ax.plot3D(
                    [arrx[num, i, 0], arrx[num, i, 0] + temporal[0]],
                    [arrx[num, i, 1], arrx[num, i, 1] + temporal[1]],
                    [arrx[num, i, 2], arrx[num, i, 2] + temporal[2]],
                    color="purple",
                    alpha=1,
                )

        temporal = Qi @ arrv[num, i, :]
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

        # Ploteo de los ángulos de los drones
        ax.plot3D(
            [arrx[num, i, 0], arrx[num, i, 0] + arrxw[num, i, 0, 0]],
            [arrx[num, i, 1], arrx[num, i, 1] + arrxw[num, i, 1, 0]],
            [arrx[num, i, 2], arrx[num, i, 2] + arrxw[num, i, 2, 0]],
            color="red",
            alpha=1,
        )

        ax.plot3D(
            [arrx[num, i, 0], arrx[num, i, 0] + arrxw[num, i, 0, 1]],
            [arrx[num, i, 1], arrx[num, i, 1] + arrxw[num, i, 1, 1]],
            [arrx[num, i, 2], arrx[num, i, 2] + arrxw[num, i, 2, 1]],
            color="green",
            alpha=1,
        )

        ax.plot3D(
            [arrx[num, i, 0], arrx[num, i, 0] + arrxw[num, i, 0, 2]],
            [arrx[num, i, 1], arrx[num, i, 1] + arrxw[num, i, 1, 2]],
            [arrx[num, i, 2], arrx[num, i, 2] + arrxw[num, i, 2, 2]],
            color="blue",
            alpha=1,
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
