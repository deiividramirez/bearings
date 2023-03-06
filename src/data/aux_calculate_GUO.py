import numpy as np

def normalize(x):
    return x/np.linalg.norm(x) if np.linalg.norm(x) != 0 else x

def distances(p1, p2):
    dist1 = []
    dist2 = []
    for i in range(p1.shape[0]):
      for j in range(p1.shape[0]):
      # for j in range(i):
        if i != j:
          dist1.append([np.sqrt(2-2*np.dot(p1[i], p1[j])), i, j])
          dist2.append([np.sqrt(2-2*np.dot(p2[i], p2[j])), i, j])
    return np.array(dist1), np.array(dist2)

def ProjOrto(x):
  temp = x.copy()
  temp.shape = (3,1)
  return np.eye(3) - np.dot(temp, temp.T)

def Laplacian(distances, p2s):
  Lapl = np.zeros((distances.shape[0], 3))
  for i in range(distances.shape[0]):
    for j in range(distances.shape[0]):
      if i != j:
        s = 1/distances[i, 0]
        # temp = s*(p2s[distances[i, 1]] @ ProjOrto(p2s[distances[j, 2]]) + p2s[distances[j, 2]] @ ProjOrto(p2s[distances[i, 1]]))
        Lapl[i] = s * (p2s[int(distances[i, 1])] @ ProjOrto(p2s[int(distances[j, 2])]) + p2s[int(distances[j, 2])] @ ProjOrto(p2s[int(distances[i, 1])]))

  return Lapl


K = np.array([[700,   0,    960],
              [ 0,    700,   540],
              [0,     0,     1]])

p1 = np.array([
  [1372, 312, 1],
  [1564, 311, 1],
  [1564, 502, 1],
  [1373, 502, 1],
])

p2 = np.array([
  [1169, 466, 1],
  [1309, 466, 1],
  [1310, 605, 1],
  [1169, 605, 1],
])

p1s = []
p2s = []

for i in range(p1.shape[0]):
    p1s.append(normalize(np.linalg.inv(K) @ p1[i]))
    p2s.append(normalize(np.linalg.inv(K) @ p2[i]))
  
p1s = np.array(p1s)
p2s = np.array(p2s)

# print(p1s)
# print(p2s)

distancias = distances(p1s, p2s)
Lap = Laplacian(distancias[1], p2s)
print(Lap, Lap.shape)

Lap_pinv = np.linalg.pinv(Lap)
print(Lap_pinv, Lap_pinv.shape)

print(np.linalg.det((Lap.T @ Lap)))
print((Lap @ Lap_pinv).shape)