import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np

from CameraModels.PlanarCamera import PlanarCamera
from Utils.ImageJacobian import ImageJacobian

# Se crean los puntos en 3D
nPoints = 8
xx = np.array([-0.5, -0.5,  0.5,  0.5,  0,  0, -1, 1])
yy = np.array([-0.5,  0.5,  0.5, -0.5,  1, -1,  0, 0])
zz = np.array([0.5,  0.5,  0.5,  0.5,  0,  0,  0, 0])

# Matriz de puntos en 3D
w_points = np.vstack([xx, yy, zz])

# ====================================================================
# POSICIÓN DESEADA DE LA CÁMARA
target_x = 0
target_y = 0
target_z = 2.0
target_roll = np.deg2rad(0.0)  # Degrees to radians 'x'
target_pitch = np.deg2rad(0.0)  # Degrees to radians 'y'
target_yaw = np.deg2rad(0.0)  # Degrees to radians 'z'

camera1 = PlanarCamera()  # Set the target camera
camera1.set_position(target_x, target_y, target_z,
                     target_roll, target_pitch, target_yaw)
K1 = camera1.K
K1_inv = np.linalg.inv(K1)


p1 = camera1.projection(w_points, nPoints)  # Project the points for camera 1

# ====================================================================
# POSICIÓN INICIAL DE LA CÁMARA
# PRUEBA #1
# init_x_pos =  2.0
# init_y_pos = -2.0
# init_z_pos =  3.5

# PRUEBA #2
init_x_pos = -2.0
init_y_pos =  2.0
init_z_pos =  5.0

init_roll  = np.deg2rad(0)
init_pitch = np.deg2rad(0)
init_yaw   = np.deg2rad(0)

saveFIG = 1
saveIMG = f"./out/ini_{init_x_pos:.0f}_{init_y_pos:.0f}_{init_z_pos:.0f}_{init_roll:.2f}_{init_pitch:.2f}_{init_yaw:.2f}.png"

camera2 = PlanarCamera()  # Set the init camera
camera2.set_position(init_x_pos, init_y_pos, init_z_pos,
                     init_roll, init_pitch, init_yaw)
K2 = camera2.K
K2_inv = np.linalg.inv(K2)

camera2.set_noise(0.0)

# Timing parameters
dt = 0.01   # Time Delta, seconds.
t0 = 0      # Start time of the simulation
t1 = 10    # (1) Define an adequate end time
t  = t0

# Initial controls
v     = np.array([[0], [0], [0]])  # speed in m/s
omega = np.array([[0], [0], [0]])  # angular velocity in rad/s
U     = np.vstack((v, omega))

# Variables initialization
steps  = int((t1 - t0)/dt + 1)         # Quantity of simulation steps
UArray = np.zeros((6, steps))         # Matrix to save controls history
tArray = np.zeros(steps)              # Matrix to save the time steps
# List to save points positions on the image
pixelCoordsArray  = np.zeros((2*nPoints, steps))
# Matrix to save error points positions
averageErrorArray = np.zeros(steps)
# Matrix to save  camera positions
positionArray     = np.zeros((3, steps))

I = np.eye(3, 3)
# (2) Define an adequate control gain
lambda_value = 2
Gain = lambda_value * np.eye(6, 6)

# (3) Define an adequate estimated fixed depth
z_fixed = 3  # Fixed depth value not null
# Temporally array for normalizing the image points
p_aux = np.vstack((p1, np.ones((1, nPoints))))
p1n = K1_inv.dot(p_aux)
vecDesired = p1n[0:2, :].reshape((2*nPoints, 1), order='F')
p2 = camera2.projection(w_points, nPoints)

# (4) Complete the function of the interaction matrix
# depth = np.array( [ np.linalg.norm(np.array([w_points[0, i], w_points[1, i], w_points[2, i]]) - np.array([x_pos, y_pos, z_pos])) for i in range(nPoints) ] )
depth = z_fixed * np.ones((nPoints, 1))
Lo = ImageJacobian(p1n, depth, 1)
Lo_inv = np.linalg.pinv(Lo)

x_pos = init_x_pos
y_pos = init_y_pos
z_pos = init_z_pos

roll = init_roll
pitch = init_pitch
yaw = init_yaw

p20 = []

# integral = np.zeros((16, 1))
# integral_array = np.zeros((16, steps))     

# w = np.zeros((2, 1))   # Vector to save the observer points
# vp = np.zeros((3, 1))  # Vector to save the observer velocity
# # w_array = np.zeros((3, steps))   # Matrix to save the observer points history
# vp_array = np.zeros((3, steps))  # Matrix to save the observer velocity history
# U_temp = np.zeros((6, 1))

# integrald = np.zeros((16, 1))
# integrald_array = np.zeros((16, steps))     

UP = "\x1B[3A"
CLR = "\x1B[0K"
print("\n\n")
np.set_printoptions(precision = 4, suppress = True)

for j in range(0, steps):

    # ===================== Calculate new translation and rotation values using Euler's method====================
    x_pos += dt * U[0, 0]
    # Note the velocities change due the camera framework
    y_pos += dt * U[1, 0]
    z_pos += dt * U[2, 0]
    roll  += dt * U[3, 0]
    pitch += dt * U[4, 0]
    yaw   += dt * U[5, 0]

    camera2.set_position(x_pos, y_pos, z_pos, roll, pitch, yaw)

    # NN = lambda t: np.sin(t/2)
    NN = lambda t: 0
    fx = lambda t: np.zeros(t.shape)+NN(t) if type(t) == np.ndarray else NN(t)
    w_points += dt*fx(j*dt)

    # (5) Project the points from the new current pose
    p2 = camera2.projection(w_points, nPoints)

    # Temporally array for normalizing the image points
    p_aux = np.vstack((p2, np.ones((1, nPoints))))
    p2n = K2_inv.dot(p_aux)
    vecCurrent = p2n[0:2, :].reshape((2*nPoints, 1), order='F')

# ==================================== CONTROL COMPUTATION =======================================

    # (6) Define the error vector (e=s-s*)
    e = vecDesired - vecCurrent

    # # (4) Complete the function of the interaction matrix
    depth = np.array( [ np.linalg.norm(np.array([w_points[0, i], w_points[1, i], w_points[2, i]]) - np.array([x_pos, y_pos, z_pos])) for i in range(nPoints) ] )
    Lo = ImageJacobian(p1n, depth, 1)
    Lo_inv = np.linalg.pinv(Lo)

    # INTEGRAL
    # integral += dt * e
    # integral_array[:, j] = integral[:,0]

    # APROX VEL 3D
    # H = .001*np.eye(2, 2)
    # Lo_inv_temp = np.linalg.pinv(Lo[2:4, :3])
    # w += dt * (  (Lo[2:4, :] @ U_temp) + H@(p1[:,1].reshape(2,1) - w)  )
    # vp = Lo_inv_temp @ (H @ (p1[:,1].reshape(2,1) - w))
    # vp_array[:, j] = vp[:,0] 

    # DESLIZANTE INTEGRAL
    # integrald += dt * e
    # integrald_array[:, j] = integrald[:,0]

    # (7) Define the control law (U=-gain*pseudoinv(Lo)*e)
    # U = -Gain @ Lo_inv @ e                                                  # CONTROL ORIGINAL
    # U = -Lo_inv @ (5*e + 1*integral)                                        # CONTROL INTEGRAL
    # U = -Gain @ Lo_inv @ e + np.concatenate((vp, np.zeros((3,1))), axis=0)  # CONTROL APROX VEL 3D
    # U_temp = -Gain @ Lo_inv @ e
    # U = Lo_inv @ (-lambda_value*np.abs(e)**(1/2)*np.sign(e))                # CONTROL DESLIZANTE
    # U = Lo_inv @ (-lambda_value*np.abs(e)**(1/2)*np.sign(e) - .5*integrald)  # CONTROL DESLIZANTE INTEGRAL
    # U_temp = U

    # Avoiding numerical error
    U[np.abs(U) < 1.0e-9] = 0.0

    # Copy data for plot
    UArray[:, j] = U[:, 0]
    tArray[j] = t

    pixelCoordsArray[:, j] = p2.reshape((2*nPoints, 1), order='F')[:, 0]

    positionArray[0, j] = x_pos
    positionArray[1, j] = y_pos
    positionArray[2, j] = z_pos

#  ========= Average feature error =====================
    pixel_error = p2-p1
    # averageErrorArray[j] = np.mean(np.linalg.norm(
    #     pixel_error.reshape((2*nPoints, 1), order='F')))
    averageErrorArray[j] = np.linalg.norm(pixel_error)

    t += dt

    if (j) % 5 == 0:
        print(f"""{UP}{j}/{steps} -> \tx: {x_pos:.4f}, y: {y_pos:.4f}, z: {z_pos:.4f}, roll: {roll:.4f}, pitch: {pitch:.4f}, yaw: {yaw:.4f}{CLR}
\tU: {U.T}, |e|: {averageErrorArray[j-1]:.4f}...{CLR}\n""")
    
    if averageErrorArray[j] < 1e-3:
        break


print(f"""{UP}{j}/{steps} -> \tx: {x_pos:.5f}, y: {y_pos:.5f}, z: {z_pos:.5f}, roll: {roll:.5f}, pitch: {pitch:.5f}, yaw: {yaw:.5f}{CLR}
\tU: {np.round(U.T, 5)}, |e|: {averageErrorArray[j-1]:.5f}{CLR}\n""")

print(f"Finished at: {j} steps -- Error: {averageErrorArray[j-1]}")



# fig, ax = plt.subplots(1, 1)

# # ax.set_title('Aprox velocidad del punto 3D')
# ax.set_title('Integral de error')

# ax.plot(tArray, fx(tArray), "--", label=f'Deseada')
# # for i in range(vp_array.shape[0]):
# #     ax.plot(tArray, vp_array[i, :], label=f'vp{i}*')
# # for i in range(integral_array.shape[0]):
# #     ax.plot(tArray, integral_array[i, :], label=f'integral{i}')
# for i in range(integrald_array.shape[0]):
#     ax.plot(tArray, integrald_array[i, :], label=f'integral{i}')

# plt.legend()


# ======================================  Draw cameras ========================================
colores = ['red', 'blue', 'green', 'black',
           'yellow', 'orange', 'purple', 'pink']

# fig = plt.figure(figsize=(15, 10))
fig = plt.figure()
fig.suptitle('World setting')

ax = fig.add_subplot(2, 2, 1, projection='3d')
ax = fig.gca(projection='3d')
ax.plot(xx, yy, zz, 'o')

ax.plot(positionArray[0, :], positionArray[1, :],
        positionArray[2, :])  # Plot camera trajectory


axis_scale = 0.5
camera_scale = 0.09
camera1.draw_camera(ax, scale=camera_scale, color='red')
camera1.draw_frame(ax, scale=axis_scale, c='black')

camera2.set_position(init_x_pos, init_y_pos, init_z_pos,
                     init_roll, init_pitch, init_yaw)
camera2.draw_camera(ax, scale=camera_scale, color='blue')
camera2.draw_frame(ax, scale=axis_scale, c='black')
limit_x = 1.0
limit_y = 1.0
limit_z = 1.0

ax.set_xlabel("$w_x$")
ax.set_ylabel("$w_y$")
ax.set_zlabel("$w_z$")
# ax.grid(True)

# ======================================  Plot the pixels ==========================================
ax = fig.add_subplot(2, 2, 2)
ax.set_aspect('equal')

p20 = pixelCoordsArray[:, 0].reshape((2, nPoints), order='F')

ax.set_ylim(0, camera1.height)
ax.set_xlim(0, camera1.width)
# ax.grid(True)


for i in range(0, 2*nPoints, 2):
    ax.plot(pixelCoordsArray[i, 0:j],
            pixelCoordsArray[i+1, 0:j], color=colores[i//2], alpha=0.5)
    # ax.plot(pixelCoordsArray[i,-1], pixelCoordsArray[i+1,-1], 'o', color=colores[i//2], alpha=0.5)
    if i != 0:
        ax.plot(p1[0, i//2],  p1[1, i//2], 'o', color=colores[i//2])
        ax.plot(p20[0, i//2], p20[1, i//2], '.', color=colores[i//2])
    else:
        ax.plot(p1[0, i//2],  p1[1, i//2], 'o',
                label="Deseada", color=colores[i//2])
        ax.plot(p20[0, i//2], p20[1, i//2], '.',
                label="Inicio", color=colores[i//2])

ax.legend(loc="right")

# ======================================  Plot the controls ========================================
ax = fig.add_subplot(2, 2, 3)
ax.plot(tArray, UArray[0, :], label='$V_x$')
ax.plot(tArray, UArray[1, :], label='$V_y$')
ax.plot(tArray, UArray[2, :], label='$V_z$')
ax.plot(tArray, UArray[3, :], label='$\omega_x$')
ax.plot(tArray, UArray[4, :], label='$\omega_y$')
ax.plot(tArray, UArray[5, :], label='$\omega_z$')
ax.grid(True)
ax.legend(loc="right")

# ======================================  Plot the pixels position ===================================
ax = fig.add_subplot(2, 2, 4)
ax.plot([0, tArray[j-1]], [averageErrorArray[j-1], averageErrorArray[j-1]], "--", alpha=0.5, label=f"y = {averageErrorArray[j-1]:.4f}")
ax.plot(tArray, averageErrorArray, label='Average error')

# ax.grid(True)
ax.legend(loc="right")

if saveFIG:
    plt.savefig(saveIMG, bbox_inches='tight', pad_inches=0.0, transparent=True)
plt.show()
