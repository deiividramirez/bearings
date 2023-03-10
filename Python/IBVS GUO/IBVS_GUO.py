# -*- coding: utf-8 -*-
"""
    2018
    @author: robotics lab (Patricia Tavares)
    @email: patricia.tavares@cimat.mx
    This code cointains a position based control
    of a camera using the Essential Matrix.

    Edited by: 
        David Leonardo Ramírez Parada
        Mail: david.parada@cimat.mx
        2022 - 2023
"""
from Functions.PlanarCamera import PlanarCamera
from Functions.Addons import *
import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np
import sys

# PATH FOR EASY EXECUTION
PATH = Path(__file__).parent.absolute()

def main():
    global CONTROL
    args = sys.argv
    if len(args) == 1:
        print("Se usará el control por defecto := 1 (1/dist)")
        CONTROL = 1
    else:
        print("Se usará el control := 2 (dist)")
        CONTROL = int(args[1])

    UP = "\x1B[3A"
    CLR = "\x1B[0K"
    print("\n\n")

    # =========================== 3D POINTS FROM DATA ===========================
    xx = np.loadtxt(f'{PATH}/cloud/x.data')
    yy = np.loadtxt(f'{PATH}/cloud/y.data')
    zz = np.loadtxt(f'{PATH}/cloud/z.data')

    n_points = len(xx)                       # Number of points
    w_points = np.vstack([xx, yy, zz])       # 3D points
    print("Number of points: ", n_points) 

    # =========================== TARGET CAMERA ===========================
    target_x = 0.0                     # Target camera position 'x'
    target_y = 0.0                     # Target camera position 'y'
    target_z = 6.0                     # Target camera position 'z'
    target_roll = np.deg2rad(0.0)      # Degrees to radians 'x'
    target_pitch = np.deg2rad(0.0)     # Degrees to radians 'y'
    target_yaw = np.deg2rad(0.0)       # Degrees to radians 'z'
    target_camera = PlanarCamera()     # Set the target camera
    target_camera.set_position(target_x, 
                               target_y, 
                               target_z, 
                               target_roll, 
                               target_pitch, 
                               target_yaw)

    # PROJECT THE 3D POINTS TO THE IMAGE PLANE OF THE TARGET CAMERA
    p_target = target_camera.projection(w_points, n_points)

    # =========================== ACTUAL CAMERA AND INITIAL POSITION ===========================
    init_x = 2.0                     # Initial camera position 'x'
    init_y = -3.0                    # Initial camera position 'y'
    init_z = 10.0                    # Initial camera position 'z'
    init_pitch = np.deg2rad(0.0)     # Degrees to radians 'x'
    init_roll = np.deg2rad(0.0)      # Degrees to radians 'y'
    init_yaw = np.deg2rad(0.0)       # Degrees to radians 'z'
    moving_camera = PlanarCamera()   # Set the init camera
    moving_camera.set_position(init_x, 
                               init_y, 
                               init_z, 
                               init_roll, 
                               init_pitch, 
                               init_yaw)

    # PROJECT THE 3D POINTS TO THE IMAGE PLANE OF THE ACTUAL CAMERA
    p_moving = moving_camera.projection(w_points, n_points)


    # =========================== GLOBAL PARAMS FOR SIMULATION ===========================
    dt = 0.01        # Time Delta, integration step
    t0 = 0           # Start time of the simulation
    steps = 1_500    # max iterations

    # =========================== CONTROL VARIABLES AND STUFF ===========================
    U = np.zeros((6, 1))                    # Control vector
    UArray = np.zeros((6, steps))           # Matrix to save controls history
    tArray = np.zeros(steps)                # Matrix to save the time steps
    
    # List to save points positions on the image
    pixelCoordsArray = np.zeros((2*n_points, steps))
    
    # Matrix to save error points positions
    ErrorArray = np.zeros(steps)
    
    # Matrix to save  camera positions
    positionArray = np.zeros((3, steps))
    
    # Matrix to save distances between points
    distancesArray = np.zeros((6, steps))
    
    # Identity matrix 3x3
    I = np.eye(3, 3)

    lamb = 10   # Gain for the control
    t = t0      # Actual time

    # =========================== AUXILIAR VARIABLES ===========================
    x_pos = init_x         # Actual camera position 'x'
    y_pos = init_y         # Actual camera position 'y'
    z_pos = init_z         # Actual camera position 'z'
    roll = init_roll       # Actual camera rotation 'x'
    pitch = init_pitch     # Actual camera rotation 'y'
    yaw = init_yaw         # Actual camera rotation 'z'
    
    p20 = []               # List to save the points
    countIndex = 0         # Counter for the iterations
    err_pix = 10           # error in pixels

    # =========================== BEGIN ALGORITHM ===========================
    while(countIndex < steps and err_pix > 1e-3):
        # while( countIndex < steps):

        # ===================== CALCULATE NEW TRANSLATION AND ROTATION VALUES USING EULER'S METHOD ====================
        # Note the velocities change due the camera framework
        
        # Position
        x_pos += dt * U[0, 0]
        y_pos += dt * U[1, 0]
        z_pos += dt * U[2, 0]
        
        # Rotation
        roll += dt * U[3, 0]
        pitch += dt * U[4, 0]
        yaw += dt * U[5, 0]

        moving_camera.set_position(x_pos, y_pos, z_pos, roll, pitch, yaw)
        p_moving = moving_camera.projection(w_points, n_points)

        # Send the points to the sphere through the generic model of the camera
        p1s, p2s, p23D = toSpehere(p_target, p_moving, target_camera)

        # We calculate the distances between the points of the sphere
        # and we calculate the error, we only return the 500 distances with the highest error
        error, distancias = distances(p1s, p2s, CONTROL=CONTROL)
        p1 = p_target[0:2, :].reshape((2*n_points, 1), order='F')
        p2 = p_moving[0:2, :].reshape((2*n_points, 1), order='F')
        errori = p2 - p1

        # Get the Jacobian matrix L for translation but not rotation
        L = Lvl(p23D, p2s, distancias, CONTROL=CONTROL)

        # Get the pseudo inverse of the Jacobian matrix
        Lo = np.linalg.pinv(L)

    # ==================================== CONTROL COMPUTATION =======================================
        """
            ERROR CONTROL
            It must be noted that this error is only for
            the translational part of the control, so
            it is not used for rotation control
        """
        U_temp = -Lo @ error
        U_temp2 = np.array([[target_roll - roll], [target_pitch - pitch], [target_yaw - yaw]])
        U = lamb * np.concatenate((U_temp, U_temp2), axis=0)
        # U = lamb * np.concatenate((U_temp, np.zeros((3,1))), axis=0) #* np.linalg.norm(error)

        # Avoiding numerical error
        # U[np.abs(U) < 1.0e-9] = 0.0

        # Copy data for plot
        UArray[:, countIndex] = U.reshape((6,))
        tArray[countIndex] = t
        pixelCoordsArray[:, countIndex] = p_moving.reshape(
            (2 * n_points, 1), order='F')[:, 0]
        positionArray[0, countIndex] = x_pos
        positionArray[1, countIndex] = y_pos
        positionArray[2, countIndex] = z_pos
        distancesArray[:, countIndex] = [i.dist for i in distancias]
        # print([i.dist for i in distancias])
        # print(len(distancias))
        # exit()

    # =================================== Average feature error ======================================

        ErrorArray[countIndex] = np.linalg.norm(error)
        err_pix = np.linalg.norm(error)

    # ==================================== Printing Dat =======================================
        t += dt
        countIndex += 1
        if (countIndex) % 5 == 0:
            print(f"""{UP}{countIndex}/{steps} -> \tx: {x_pos:.5f}, y: {y_pos:.5f}, z: {z_pos:.5f}, roll: {roll:.5f}, pitch: {pitch:.5f}, yaw: {yaw:.5f}{CLR}
    \tU: {np.round(U.T, 5)}, |e|: {np.linalg.norm(error):.5f}...{CLR}\n""")

        if err_pix > 600:
            break

    print(f"""{UP}{countIndex}/{steps} -> \tx: {x_pos:.5f}, y: {y_pos:.5f}, z: {z_pos:.5f}, roll: {roll:.5f}, pitch: {pitch:.5f}, yaw: {yaw:.5f}{CLR}
    \tU: {np.round(U.T, 5)}, |e|: {np.linalg.norm(error):.5f}{CLR}\n""")

    print(f"Finished at: {countIndex} steps -- Error: {np.linalg.norm(error)}")

    # ======================================  Draw cameras ========================================
    colores = np.random.rand(n_points, 3)

    # fig = plt.figure(figsize=(10, 10))
    fig = plt.figure()
    fig.suptitle(
        f'Control #{1 if CONTROL==1 else 2}: {n_points} points', fontsize=16)

    ax = fig.add_subplot(2, 2, 1, projection='3d')
    ax = fig.gca()

    ax.plot(xx, yy, zz, 'o')
    ax.plot(positionArray[0, 0:countIndex], positionArray[1, 0:countIndex],
            positionArray[2, 0:countIndex])  # Plot camera trajectory

    axis_scale = 0.5
    camera_scale = 0.02

    target_camera.draw_camera(ax, scale=camera_scale, color='red')
    target_camera.draw_frame(ax, scale=axis_scale, c='black')
    moving_camera.set_position(x_pos, y_pos, z_pos, roll, pitch, yaw)
    moving_camera.draw_camera(ax, scale=camera_scale, color='black')
    moving_camera.draw_frame(ax, scale=axis_scale, c='black')
    moving_camera.set_position(
        init_x, init_y, init_z, init_roll, init_pitch, init_yaw)
    moving_camera.draw_camera(ax, scale=camera_scale, color='blue')
    moving_camera.draw_frame(ax, scale=axis_scale, c='black')

    ax.set_xlabel("$w_x$")
    ax.set_ylabel("$w_y$")
    ax.set_zlabel("$w_z$")

    # ======================================  Plot the pixels ==========================================
    ax = fig.add_subplot(2, 2, 2)
    p20 = pixelCoordsArray[:, 0].reshape((2, n_points), order='F')

    ax.set_ylim(0, target_camera.height)
    ax.set_xlim(0, target_camera.width)

    for i in range(0, 2*n_points, 2):
        ax.plot(pixelCoordsArray[i, 0:countIndex],
                pixelCoordsArray[i+1, 0:countIndex], color=colores[i//2], alpha=0.5)
        if i != 0:
            ax.plot(p_target[0, i//2],  p_target[1, i//2],
                    'o', color=colores[i//2])
            ax.plot(p20[0, i//2], p20[1, i//2], '.', color=colores[i//2])
        else:
            ax.plot(p_target[0, i//2],  p_target[1, i//2], 'o',
                    label="Deseada", color=colores[i//2])
            ax.plot(p20[0, i//2], p20[1, i//2], '.',
                    label="Inicio", color=colores[i//2])

    ax.legend(loc="best")

    # ======================================  Plot the controls ========================================
    ax = fig.add_subplot(2, 2, 3)

    ax.plot(tArray[0:countIndex], UArray[0, 0:countIndex], label='$v_x$')
    ax.plot(tArray[0:countIndex], UArray[1, 0:countIndex], label='$v_y$')
    ax.plot(tArray[0:countIndex], UArray[2, 0:countIndex], label='$v_z$')
    ax.plot(tArray[0:countIndex], UArray[3, 0:countIndex], label='$\omega_x$')
    ax.plot(tArray[0:countIndex], UArray[4, 0:countIndex], label='$\omega_y$')
    ax.plot(tArray[0:countIndex], UArray[5, 0:countIndex], label='$\omega_z$')

    ax.grid(True)
    ax.legend(loc="center right")

    # ======================================  Plot the pixels position ===================================

    ax = fig.add_subplot(2, 2, 4)
    ax.plot(tArray[0:countIndex], ErrorArray[0:countIndex], label='$Error$')

    ax.grid(True)
    ax.legend(loc="upper right")

    fig, ax = plt.subplots()
    ax.set_title("Distancias")
    for i in range(6):
        ax.plot(distancesArray[i, 0:countIndex], label=f'p{i}', color=colores[i//2])
        ax.plot([0, countIndex], [distancias[i].dist2, distancias[i].dist2],
                '--', color=colores[i//2], label=f'p{i}*')

    ax.legend(loc="upper right")
    plt.show()


if __name__ == "__main__":
    main()
