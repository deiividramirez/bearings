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

x = np.array([
    [2.91336507, 1.76084991, 10.34150496],
    [-5, 0, 10],
    [5, 0, 10],
    [4.91667537, 2.53318023, 11.42084397],
])

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
dt = .01
t = np.linspace(0, tmax, 5*lider_iz_ac.shape[0])
# t = np.linspace(0, tmax, 1000)


# SIMULACIÓN
# ----------------------------------------------------------------------------------------------------------------------

# Ganancias del controlador
Kp = 5
Kv = .25

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

            suma = np.zeros(d)
            for j in range(n):
                if i != j and np.any(gijA[i, j, :] != 0):
                    # suma += ortProj(normalize(x[j]-x[i])) @ (v[j, :] - v[i, :])
                    
                    # BEARING-ONLY FORMATION TRACKING CONTROL OF MULTIAGENT SYSTEMS - SHIYU ZHAO
                    suma += normalize(x[j]-x[i]) - gijA[i, j, :]


                    suma -= ortProj(normalize(x[j]-x[i])) @ (gijA[i, j, :])

                    error[i] += np.linalg.norm(normalize(x[j] - x[i]) - gijA[i, j, :])

                # print(f"Error {i},{j}: {errorc}, {normalize(x[j]-x[i])}, {gijA[i, j, :]}")

            temp[i, :] = (tempv[-1][i, :] + dt * suma)
            vB[i, :] = Kp*suma + Kv*temp[i, :]
            # vB[i, :] = Kp * suma 
        else:
            if i == P2:
                # vnew[i, :] = lider_iz_ac[num]
                vnew[i, :] = 0
            else:
                # vnew[i, :] = lider_de_ac[num]
                vnew[i, :] = 0

    tempv.append(temp)

    v = v + dt * vnew
    for i in range(n):
        if i != P1 and i != P2:
            v[i, :] = vB[i, :]
            vnew[i, :] = vB[i, :]
        else:
            v[i, :] = 2.5
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

"""
# PLOT FORMACIÓN Y BEARINGS SI: estad = True
# ----------------------------------------------------------------------------------------------------------------------

# if estad:
#     fig = plt.figure()
#     ax = plt.axes(projection='3d')
#     plt.subplots_adjust(right=0.7)
#     ax.set_title("Formación deseada y bearings")
#     ax.set_box_aspect([1, 1, 1])
#     for i in range(n):
#         for j in range(n):
#             if i != P1 and i != P2 and np.any(gijA[i, j, :]):
#                 ax.plot3D([All[i, 0], All[i, 0] + gijA[i, j, 0]], [All[i, 1], All[i, 1] + gijA[i, j, 1]],
#                           [All[i, 2], All[i, 2] + gijA[i, j, 2]], color=colores[j], alpha=0.5)

#         ax.plot3D([All[i, 0]], [All[i, 1]], [All[i, 2]],
#                   'o', color=colores[i], alpha=0.5, label=labels[i])

#         # Ploteo de los drones en posición final
#         # ax.plot3D([arrx[-1, i, 0]], [arrx[-1, i, 1]], [
#         #           arrx[-1, i, 2]], 'o', color=colores[i], alpha=0.5, label=labels[i]+" (F)")

#     ax.legend(bbox_to_anchor=(1.1, 0.5), loc="center left", borderaxespad=0)
#     if save:
#         plt.savefig("./src/out/formacion.png", dpi=300,
#                     bbox_inches='tight', transparent=True)

# if estad:
#     fig = plt.figure()
#     ax = plt.axes(projection='3d')
#     plt.subplots_adjust(right=0.7)
#     ax.set_title("Formación final")
#     # ax.set_box_aspect([1, 1, 1])
#     for i in range(n):
#         for j in range(n):
#             if i != P1 and i != P2 and np.any(gijA[i, j, :]):
#                 ax.plot3D([arrx[-1, i, 0], arrx[-1, i, 0] + gijA[i, j, 0]], [arrx[-1, i, 1], arrx[-1, i, 1] + gijA[i, j, 1]],
#                           [10, 10 + gijA[i, j, 2]], color=colores[j], alpha=0.5)
#         # Ploteo de los drones en posición final
#         ax.plot3D([arrx[-1, i, 0]], [arrx[-1, i, 1]], [10],
#                     'o', color=colores[i], alpha=0.5, label=labels[i]+" (F)")
#         # print(f"Posición final de {labels[i]}: {arrx[-1, i, 0]} {arrx[-1, i, 1]} {arrx[-1, i, 2]}")
#     ax.legend(bbox_to_anchor=(1.1, 0.5), loc="center left", borderaxespad=0)
#     if save:
#         plt.savefig("./src/out/formacion_final.png", dpi=300,
#                     bbox_inches='tight', transparent=True)
"""

# PLOTEO DE VELOCIDADES SI: estad = True
# ----------------------------------------------------------------------------------------------------------------------
if estad:
    fig, ax = plt.subplots(3, 1, sharex=True)
    # fig, ax = plt.subplots(2, 1, sharex=True)
    ax[0]
    ax[0].set_title("Velocidades")
    plt.subplots_adjust(right=0.75)
    [ax[0].plot(t, np.mean(arrv[1:, i], axis=1), color=colores[i], alpha=1,
                label=labels[i]) for i in range(n)]
    # [ax[0].plot(t, arrv[1:, i, 1], color=colores[i], alpha=.8) for i in range(n)] 
    # [ax[0].plot(t, arrv[1:, i, 2], color=colores[i], alpha=.5) for i in range(n)]
    ax[0].set_xlabel("Tiempo [s]")
    ax[0].set_ylabel("Velocidad [m/s]")
    ax[0].legend(bbox_to_anchor=(1.02, 0.5),
                 loc="center left", borderaxespad=0)

    ax[1]
    ax[1].set_title("Error")
    plt.subplots_adjust(right=0.75)
    [ax[1].plot(t, errorv[1:, i], color=colores[i], alpha=1,
                label=labels[i]) for i in range(n)]
    ax[1].set_xlabel("Tiempo [s]")
    ax[1].set_ylabel("Error [m]")
    ax[1].legend(bbox_to_anchor=(1.02, 0.5),
                 loc="center left", borderaxespad=0)

    ax[2]
    ax[2].set_title("Integración de bearings")
    plt.subplots_adjust(right=0.75)
    [ax[2].plot(t, np.mean(tempv[1:, i], axis=1), color=colores[i], alpha=1,
                label=labels[i]) for i in range(n)]
    ax[2].set_xlabel("Tiempo [s]")
    ax[2].set_ylabel("Integración [rad]")
    ax[2].legend(bbox_to_anchor=(1.02, 0.5),
                 loc="center left", borderaxespad=0)

    plt.tight_layout()

    if save:
        plt.savefig("./src/out/velocidades.png", dpi=300,
                    bbox_inches='tight', transparent=True)


# ANIMACIÓN DE LAS POSICIONES
# ----------------------------------------------------------------------------------------------------------------------
def animate(num: int):
    # Configuración del plot
    ax.cla()
    num = 10*num % len(t)
    ax.set_title(f"Tiempo: {t[num]:.2f}s. Error total: {np.sum(errorv[num]):.3f}m")
    ax.set_xlim3d(np.min(arrx[num, :, 0])-1, np.max(arrx[num, :, 0])+1)
    ax.set_ylim3d(np.min(arrx[num, :, 1])-1, np.max(arrx[num, :, 1])+1)
    ax.set_zlim3d(np.min(arrx[num, :, 2])-1, np.max(arrx[num, :, 2])+1)
    ax.set_box_aspect([1, 1, 1])

    # Ventana
    # dist = 2.5
    # t = np.linspace(0, 2*dist, 100)
    # o = np.ones(t.shape)
    # ax.plot3D(-dist*o, np.zeros(t.shape), t-dist+5, "black")
    # ax.plot3D(t-dist, np.zeros(t.shape), -dist*o+5, "black")

    # t = np.linspace(-2*dist, 0, 100)
    # ax.plot3D(t+dist, np.zeros(t.shape), dist*o+5, "black")
    # ax.plot3D(dist*o, np.zeros(t.shape), t+dist+5, "black")

    # Recorrer todos los drones
    for i in range(n):
        for j in range(n):
            # Ploteo de los bearings
            if i != P1 and i != P2:
                ax.plot3D([arrx[num, i, 0], arrx[num, i, 0] + gijA[i, j, 0]],
                          [arrx[num, i, 1], arrx[num, i, 1] + gijA[i, j, 1]],
                          [arrx[num, i, 2], arrx[num, i, 2] + gijA[i, j, 2]], color=colores[j], alpha=0.5)
        ax.plot3D([arrx[num, i, 0], arrx[num, i, 0] + arrv[num, i, 0]],
                  [arrx[num, i, 1], arrx[num, i, 1] + arrv[num, i, 1]],
                  [arrx[num, i, 2], arrx[num, i, 2] + arrv[num, i, 2]], color=colores[i], alpha=0.5)

        # Ploteo de los drones
        ax.plot3D([arrx[num, i, 0]],
                  [arrx[num, i, 1]],
                  [arrx[num, i, 2]], 'o', color=colores[i], alpha=0.5, label=labels[i])
        # Ploteo de la trayectoria dada por los drones
        ax.plot3D(arrx[:num, i, 0],
                  arrx[:num, i, 1],
                  arrx[:num, i, 2], '-', color=trayec[i], alpha=0.5)
        # Ploteo de los puntos iniciales
        ax.plot3D([arrx[0, i, 0]],
                  [arrx[0, i, 1]],
                  [arrx[0, i, 2]], 'o', color=trayec[i], alpha=0.5)
    ax.legend(bbox_to_anchor=(1.1, 0.5), loc="center left", borderaxespad=0)

    # """
    # Scaling is done from here...
    # """
    # x_scale = 1
    # y_scale = 3
    # z_scale = 1

    # scale=np.diag([x_scale, y_scale, z_scale, 1.0])
    # scale=scale*(1.0/scale.max())
    # scale[3,3]=1.0

    # def short_proj():
    #     return np.dot(Axes3D.get_proj(ax), scale)

    # ax.get_proj=short_proj
    # """
    # to here
    # """


# Animación
if anim:
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    # ax.azim = -15
    # ax.elev = 15
    # ax.dist = 8
    ax.set_title("Simulación")
    plt.subplots_adjust(right=0.8)
    line_ani = animation.FuncAnimation(
        fig, animate, interval=1, frames=len(t)//10,  repeat=False)
    # animate(-1)

    # GUARDAR SIMULACIÓN EN ARCHIVO .gif
    # ----------------------------------------------------------------------------------------------------------------------

    if save:
        print("\nGuardando animación")
        f = "./src/out/formation.gif"
        writergif = animation.PillowWriter(fps=len(t)/(tmax+1))
        line_ani.save(f, writer=writergif)
        print("Se guardó.")

if estad or anim:
    plt.show()
