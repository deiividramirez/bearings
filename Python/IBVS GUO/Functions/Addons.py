from numpy.linalg import inv
import numpy as np

#================================================================Functions
class vecDist:
    def __init__(self, i: int, j:int, dist:float, dist2:float) -> None:
        self.i = i
        self.j = j
        self.dist = dist
        self.dist2 = dist2

    def __repr__(self) -> str:
        return f"i: {self.i} <-> j: {self.j}: d1 -> {self.dist:5f} - d2 -> {self.dist2:5f}"

def toSpehere(p1, p2, camera):
    p1s  = np.zeros((3, p1.shape[1]))
    p2s  = np.zeros((3, p2.shape[1]))
    p23D = np.zeros((3, p2.shape[1]))
    for i in range(p1.shape[1]):
        temp = np.concatenate( [p1[:, i], [1]] ).reshape(3,1)
        temp = inv(camera.K) @ temp
        p1s[:, i] = temp.T / np.linalg.norm(temp)
        
        temp = np.concatenate( [p2[:, i], [1]] ).reshape(3,1)
        temp = inv(camera.K) @ temp
        p2s[:, i] = temp.T / np.linalg.norm(temp)
        p23D[:, i] = temp.T

    return p1s, p2s, p23D

def distances(p1, p2, CONTROL):
    distancias, error = [], []
    for i in range(p1.shape[1]):
        for j in range(i):
        # for j in range(p1.shape[1]):
            if i != j:
                dist  = np.sqrt( 2 - 2 * np.dot(p2[:,i], p2[:,j]) )
                dist2 = np.sqrt( 2 - 2 * np.dot(p1[:,i], p1[:,j]) ) 
                if dist <= 1e-9 or dist2 <= 1e-9:
                    continue

                distancias.append(
                    vecDist(i, j, 1/dist, 1/dist2) if CONTROL == 1 else vecDist(i, j, dist, dist2)
                )
            
    distancias = sorted(distancias, key=lambda x: x.dist, reverse=True)
    error = [i.dist2 - i.dist for i in distancias]
    return np.array(error).reshape(len(error), 1), distancias

def ortoProj(p1):
    return np.eye(3) - p1 @ p1.T

def Lvl(p23D, p2s, distancias, CONTROL):
    n = len(distancias)
    L = np.zeros((n, 3))

    for i in range(n):
        s = -distancias[i].dist**3 if CONTROL == 1 else 1/distancias[i].dist
        
        temp = s * ( (p2s[:, distancias[i].i].reshape(1,3)) @ ortoProj(p2s[:, distancias[i].j].reshape(3,1))/2 + 
                     (p2s[:, distancias[i].j].reshape(1,3)) @ ortoProj(p2s[:, distancias[i].i].reshape(3,1))/2 )
        
        L[i, :] = temp
    return L