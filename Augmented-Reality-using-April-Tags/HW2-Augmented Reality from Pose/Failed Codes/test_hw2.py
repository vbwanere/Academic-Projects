from solve_p3p1 import P3P
import numpy as np

Pc = np.array([[1,2],[9,4],[7,2],[4,3]])
Pw = np.array([[1,2,5], [7,8,3], [8,3,6], [5,6,3]])
R, t = P3P(Pc, Pw, K=np.eye(3))
