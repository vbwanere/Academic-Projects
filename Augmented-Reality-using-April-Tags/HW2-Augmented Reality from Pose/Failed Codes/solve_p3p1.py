import numpy as np
import math

def P3P(Pc, Pw, K=np.eye(3)):
    """
    Solve Perspective-3-Point problem, given correspondence and intrinsic

    Input:
        Pc: 4x2 numpy array of pixel coordinate of the April tag corners in (x,y) format
        Pw: 4x3 numpy array of world coordinate of the April tag corners in (x,y,z) format
    Returns:
        R: 3x3 numpy array describing camera orientation in the world (R_wc)
        t: (3,) numpy array describing camera translation in the world (t_wc)

    """

    ##### STUDENT CODE START #####
    ones = np.ones((4,1))
    Pc = np.concatenate((Pc, ones), axis=1)
    # print(Pc)
    #### Calculating the norms for getting a, b and c. #####
    a = np.linalg.norm(Pw[0,:]-Pw[1,:])
    b = np.linalg.norm(Pw[1,:]-Pw[2,:])
    c = np.linalg.norm(Pw[2,:]-Pw[0,:])
    # print(a, b, c)
    ##### Calculate Caliberated coordinates and unit vectors####
    Pc = np.linalg.inv(K) @ Pc.T
    Pc = Pc.T
    # print(Pc[0])
    i = Pc[0]/np.linalg.norm(Pc[0])
    j = Pc[1]/np.linalg.norm(Pc[1])
    k = Pc[2]/np.linalg.norm(Pc[2])
    # print(i, j, k)
    ### calculating the angles ###
    alpha = np.arccos(np.dot(j,k))
    beta = np.arccos(np.dot(i,j))
    gamma = np.arccos(np.dot(i,j))
    # print(math.cos(alpha), math.cos(beta), gamma)
    ### Calculating the equations ###
    A0 = (1 + (a **2 - c **2) / b **2) **2 - 4 * (a **2 / b **2) * math.cos(gamma) **2
    A1 = 4 * (-((a **2 - c **2) / b **2) * (1 + ((a **2 - c **2) / b **2)) * math.cos(beta) + 2 * (a **2 / b **2) * (math.cos(gamma) **2) * math.cos(beta) - (1 - ((a **2 + c **2) / b **2)) * math.cos(alpha) * math.cos(gamma))
    A2 = 2 * (((a **2 - c **2) / b **2) **2 - 1 + 2 * ((a **2 - c **2) / b **2) **2 * math.cos(beta) **2 + 2 * ((b **2 - c **2) / b **2) * math.cos(alpha) **2 - 4 * ((a **2 + c **2) / b **2) * math.cos(alpha) * math.cos(beta) * math.cos(gamma) + 2 * ((b **2 - a **2) / b **2) * math.cos(gamma) **2)
    A3 = 4 * (((a **2 - c **2) / b **2) * (1 - ((a **2 - c **2) / b **2)) * math.cos(beta) - (1 - ((a **2 + c **2) / b **2)) * math.cos(alpha) * math.cos(gamma) + 2 * (c **2 / b **2) * (math.cos(alpha) **2) * math.cos(beta))
    A4 = (((a **2 - c **2) / b **2) - 1) **2 - 4 * (c **2 / b **2) * math.cos(alpha) **2
    coeff = [A0, A1, A2, A3, A4]
    V = np.roots(coeff)
    V = V.real
    # print(V)
    # for i in range(4):
    #     print(V[i])
    U = []
    for i in range(4):
        U_i = ((-1 + (a **2 - c **2) / b **2) * (V[i] **2) - 2 * ((a **2 - c **2) / b **2) * math.cos(beta * V[i]) + 1 + ((a **2 - c **2) / b **2))/ (2 * math.cos(gamma) - V[i] * math.cos(alpha))
        U.append(U_i)
    # print(U)  
    s1 = b ** 2 / ((1 + V[i])**2) - 2 * V[i] * math.cos(beta)
    s2 = U[i] * s1
    s3 = V[i] * s1
    R = 1
    t = 1
    # Invoke Procrustes function to find R, t
    # You may need to select the R and t that could transoform all 4 points correctly. 
    # R,t = Procrustes(Pc_3d, Pw[1:4])
    ##### STUDENT CODE END #####

    return R, t

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
    R = 1
    t = 1

    ##### STUDENT CODE END #####

    return R, t
