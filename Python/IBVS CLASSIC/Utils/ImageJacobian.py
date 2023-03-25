import numpy as np

def ImageJacobian(image_points, z_world_points, focal_length):

    """

    :param image_points:
    :param z_world_points:
    :param camera:
    :return:
    """
    f = focal_length

    points_counter = image_points.shape[1]

    L = np.zeros((2 * points_counter, 6))

    for i in range(0, points_counter):
        u = image_points[0, i]
        v = image_points[1, i]

        z = z_world_points[i]

        L[2 * i, 0] = -f / z
        L[2 * i, 1] = 0
        L[2 * i, 2] = u / z
        L[2 * i, 3] = u * v / f
        L[2 * i, 4] = -(f ** 2 + u ** 2) / f
        L[2 * i, 5] = v

        L[2 * i + 1, 0] = 0
        L[2 * i + 1, 1] = -f / z
        L[2 * i + 1, 2] = v / z
        L[2 * i + 1, 3] = (f ** 2 + v ** 2) / f
        L[2 * i + 1, 4] = -u * v / f
        L[2 * i + 1, 5] = -u
    
    return L
