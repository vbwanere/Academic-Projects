import numpy as np
from solve_p3p import *
Pw = np.array([[1,2,0], [7,8,0], [8,3,0], [5,6,0]])
K = np.array([[1000, 0, 0], [0, 1000, 0], [0, 0, 1]])

R_cw = np.array([[1, -0.2, 0.4], [-0.5, 0.3, 1], [0.5, 1, 0.4]])
t_cw = np.array([100, 100, 100])
Pc = np.ones((4, 2))
for i in range(4):
    Rwt = R_cw @ Pw[i, :] + t_cw
    Pc0 = K @ Rwt
    Pc0 = Pc0/Pc0[2]
    Pc0 = np.delete(Pc0, 2, 0)
    Pc[i, :] = Pc0
Pc = np.round(Pc)
print("Pcorg = ", Pc)
R, t = P3P(Pc, Pw, K)
print("Rnew", R)
print("t_new", t)

