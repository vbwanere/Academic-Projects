import numpy as np

a = np.array([[7, 2, 3, 4],
              [5, 6, 7, 8],
              [3, 6, 9, -2]])

b = np.array([[1, 1, -1],
              [-1, 1, -1],
              [1, 1, 1]])
c = b * a[:, 0]
v = np.array([-1, 1, 2, -3, 8])
v = v[v >0]

x = np.array([1, 2, 4, 3])
x = x.reshape(4, 1)
y = np.array([4, 6, 7, 13])
y = y.reshape(4, 1)
z = np.array([14, 22, 42, 31])
z = z.reshape(4, 1)
#Z = np.array([x, y, z])
Z = np.concatenate((x, y, z), axis=1)
#print(Z.transpose())
#print(Z[:, 0])
Pc = np.array([[-200, -200],
               [200, -200],
               [200, 200],
               [-200, 200]])
Pc2 = np.array([Pc[2,0], Pc[2, 1], 1])

print(np.dot(x.T,y))