import numpy as np
from solve_p3p import *

Pc = np.array([[111,424],[967,722],[317,294],[434,377]])
Pw = np.array([[1,2,5], [7,8,3], [8,3,6], [5,6,3]])
K = np.array([[1000, 0, 0],
              [0, 1000, 0],
              [0, 0, 1]])

def Procrustes(X, Y):
    """
    Solve Procrustes: Y = RX + t

    Input:
        X: Nx3 numpy array of N points in camera coordinate (returned by your P3P)
        Y: Nx3 numpy array of N points in world coordinate
    Returns:
        R: 3x3 numpy array describing camera orientation in the world (R_wc)
        t: (3,) numpy array describing camera translation in the world (t_wc)

    """
    ##### STUDENT CODE START #####

    Xb = np.mean(X,axis =0)
    Yb = np.mean(Y,axis =0)

    X = (X-Xb)
    X = X.T
    Y = (Y - Yb)
    Y = Y.T

    R = Y @ X.T

    U,S,Vt = np.linalg.svd(R)
    V = Vt.T
    A = np.eye(3)
    A[2,2] = np.linalg.det(V @ U.T)
    R = np.dot(U, np.dot(A, V.T))

    t = Yb - R @ Xb
    t = np.array(t).squeeze() #shape of 3*1

    ##### STUDENT CODE END #####
    return R, t
# R, t = Procrustes(Pw[1:], Pc[1:])
# print(R)
# print(t)
#compute distances between points
B_coor = Pw[1, :]
C_coor = Pw[2, :]
D_coor = Pw[3, :]
a = np.linalg.norm(C_coor - D_coor)
b = np.linalg.norm(B_coor - D_coor)
c = np.linalg.norm(B_coor - C_coor)
print("a", a)
print("b", b)
print("c", c)
# take u,v as pixel coordinates from pixel array
u = Pc[:,0]
v = Pc[:,1]

#converting them into homogeneous coordinates

u_2 = np.reshape(u,(4,1))
v_2 = np.reshape(v,(4,1))
ones_2 = np.ones((4,1))

uv_homo = np.concatenate((u_2,v_2,ones_2), axis = 1)
# get the calibrated pts after multiplying it with K, and transpose it to get a 4*3 matrix
pts_cb =(np.linalg.inv(K) @ uv_homo.T).T
# now get unit vectors j to get angles alpha, beta and gamma
j1 = np.array([(1 / np.linalg.norm(pts_cb[1, :])) * pts_cb[1, :]])
j2 = np.array([(1 / np.linalg.norm(pts_cb[2, :])) * pts_cb[2, :]])
j3 = np.array([(1 / np.linalg.norm(pts_cb[3, :])) * pts_cb[3, :]])
print("j1", j1)
print("j2", j2)
print("j3", j3)
# define angles alpha, beta and gamma
cos_a = np.dot(j2, j3.T)
cos_b = np.dot(j1, j3.T)
cos_g = np.dot(j1, j2.T)
print("cos_a", cos_a)
print("cos_b", cos_b)
print("cos_g", cos_g)
# define the coeffs of the equation in v

A0 = (1 + (a ** 2 - c ** 2) / b ** 2) ** 2 - 4 * (a ** 2 / b ** 2) * cos_g ** 2
A1 = 4 * (-((a ** 2 - c ** 2) / b ** 2) * (1 + ((a ** 2 - c ** 2) / b ** 2)) * cos_b + 2 * (a ** 2 / b ** 2) * (
            cos_g ** 2) * cos_b - (1 - ((a ** 2 + c ** 2) / b ** 2)) * cos_a * cos_g)
A2 = 2 * (((a ** 2 - c ** 2) / b ** 2) ** 2 - 1 + 2 * ((a ** 2 - c ** 2) / b ** 2) ** 2 * cos_b ** 2 + 2 * (
            (b ** 2 - c ** 2) / b ** 2) * cos_a ** 2 - 4 * (
                      (a ** 2 + c ** 2) / b ** 2) * cos_a * cos_b * cos_g + 2 * (
                      (b ** 2 - a ** 2) / b ** 2) * cos_g ** 2)
A3 = 4 * (((a ** 2 - c ** 2) / b ** 2) * (1 - ((a ** 2 - c ** 2) / b ** 2)) * cos_b - (
            1 - ((a ** 2 + c ** 2) / b ** 2)) * cos_a * cos_g + 2 * (c ** 2 / b ** 2) * (
                      cos_a ** 2) * cos_b)
A4 = ((a ** 2 - c ** 2) / b ** 2 - 1) ** 2 - 4 * (c ** 2 / b ** 2) * cos_a ** 2

# compile a coeff array
coeffs = [A4,A3,A2,A1,A0]
A = np.array(coeffs)
A = np.ravel(A)
print("A", A)
# use .roots to get the roots ( 4 total, 2 likely real)
sols = np.roots(A.T)
sols = np.array(sols)
sols = np.real(sols)
sols = sols[sols >0]
print("v", sols)
Opt_R = None
Opt_t = None
least_error = 100000000000000000000000000
for i in range(sols.shape[0]):

    u = ((-1 + (a ** 2 - c ** 2) / b ** 2) * np.square(sols[i]) - 2 * ((a ** 2 - c ** 2) / b ** 2) * cos_b * sols[i] + 1 + ((a ** 2 - c ** 2) / b ** 2)) / (2 * (cos_g - sols[i] * cos_a))
    print("u", u)
    s_1 = np.sqrt((b ** 2) / (1 + sols[i] ** 2 - 2 * sols[i] * cos_b))
    print("s1", s_1)
    s_2 = u * s_1
    s_3 = sols[i] * s_1
    print("s2", s_2)
    print("s3", s_3)

    o = (s_1*j1)
    l = (s_2*j2)
    q = (s_3*j3)
    p = np.vstack((o,l,q))

    print("p = ", p)
    R, t = Procrustes(Pw[1:], p)
    print("R = ", R)
    print("t = ", t)

    P = np.dot(K, np.dot(R, Pw[0, :].T) + t)
    #P will be an array now, normalization
    P = P/P[2]
    P = np.delete(P, 2, 0)
    print("p_testg", P)
    print("Pc0 = ", Pc[0])
    #calculate error wrt  the unused coordinate in gurnert's equation?
    difference = (P - Pc[0])
    err = np.linalg.norm(difference)
    print("Error", err)

    if err < least_error:
        least_error = err
        Opt_R = R
        Opt_t = t

Opt_Ri = np.linalg.inv(Opt_R)
print("Opt_Ri", Opt_Ri)
gan = Opt_Ri @ Opt_t
gan = -gan
print("gan", gan)

Pc1 = np.array([[111,424],[967,722],[317,294],[434,377]])
Pw = np.array([[1,2,5], [7,8,3], [8,3,6], [5,6,3]])
K = np.array([[1000, 0, 0],
              [0, 1000, 0],
              [0, 0, 1]])
Rr, tt = P3P(Pc, Pw, K)
print("Rr", Rr)
print("tt", tt)
R_cw = np.array([2, 3, 4], [1, 5, 7], [3, 8, 9])
t_cw = np.array([8, 11, 4])
Pc1= K @ (R_cw @ Pw + t_cw)
