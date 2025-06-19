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

    #Define world coordinates for correspondance with pixel coordinates:
    pw = np.ones((3, 3))
    pw[0, :] = Pw[0, :]
    pw[1, :] = Pw[1, :]
    pw[2, :] = Pw[2, :]

    # Define points in world to find distance:
    Pwh = np.delete(Pw, 2, 1)
    P1 = Pwh[0, :]
    P2 = Pwh[1, :]
    P3 = Pwh[2, :]
    ones = np.ones((Pwh.shape[0], 1))
    Pw = np.concatenate((Pwh, ones), axis=1)

    #Find distaces between points in world:
    a = math.dist(P2, P3)**2
    b = math.dist(P1, P3)**2
    c = math.dist(P1, P2)**2
    ac = (a - c) / b
    ab = (a + c) / b
    bc = (b - c) / b
    ba = (b - a) / b

    #Define projections of the above points on image plane in camera coordinates
    #p = np.array([[Pc[0, 0] - K[0, 2], Pc[0, 1] - K[1, 2], K[0, 0]],
    #              [Pc[3, 0] - K[0, 2], Pc[3, 1] - K[1, 2], K[0, 0]],
    #              [Pc[1, 0] - K[0, 2], Pc[1, 1] - K[1, 2], K[0, 0]]])
    p = np.array([[Pc[0, 0], Pc[0, 1], 1],
                  [Pc[1, 0], Pc[1, 1], 1],
                  [Pc[2, 0], Pc[2, 1], 1]])
    Ki = np.linalg.inv(K)
    #Calculate unit vectors from point p1, p2 and p3 in camera coodinates
    # to calculate angles alpha, beta and gamma:
    pu = np.ones((3, 3))
    for i in range(pu.shape[0]):
        pu[i, :] = (Ki @ p[i, :]) / (np.linalg.norm(Ki @ p[i, :]))

    #Find angles between the vectors:
    al = np.dot(pu[1, :], pu[2, :])
    bt = np.dot(pu[0, :], pu[2, :])
    gm = np.dot(pu[0, :], pu[1, :])

    #Frame the Polynomial:
    A4 = (ac - 1)**2 - 4 * c / b * al * al
    A3 = 4 * (ac * (1-ac) * bt - (1 - ab) * al * gm + 2 * c / b * al * al * bt)
    A2 = 2 * (ac * ac - 1 + 2 * ac * ac * bt * bt + 2 * bc * al * al - 4 * ab * al * bt * gm + 2 * ba * gm * gm)
    A1 = 4 * (-ac * (1 + ac) * bt + 2 * a / b * gm * gm * bt - (1 - ab) * al * gm)
    A0 = (1 + ac)**2 - 4 * a / b * gm * gm

    #Roots of the Polynomial:
    A = [A4, A3, A2, A1, A0]
    v = np.roots(A)
    v = v.real
    v = np.array(v)
    v = v[v > 0]
    v = v.reshape((len(v), 1))

    u = np.ones((4, 1))
    for i in range(len(v)):
        u[i, 0] = ((-1 + ac) * v[i] * v[i] - 2 * ac * bt * v[i] + 1 + ac) / (2 * (gm - v[i] * al))

    #Calculating s1, s2, s3:
    s1 = np.ones((len(v), 1))
    for i in range(len(v)):
        s1[i] = math.sqrt(c**2 / (1 + u[i, 0] * u[i, 0] - 2 * u[i, 0] * gm))
    s2 = u * s1
    s3 = v * s1
    s = np.concatenate((s1, s2, s3), axis=1)
    s = s.transpose()
    #3D vertex of points P1, P2 and P3 in camera coordinates from s1, s2, s3:
    tolerance = 10000000000
    p2p = np.array([Pc[3, 0], Pc[3, 1], 1])
    for i in range(s.shape[1]):
        Pc3 = pu.T * s[:, i]
        Pc3 = Pc3.T
        R, t = Procrustes(Pc3, pw)
        p4p = K @ np.linalg.inv(R) @ (Pw[3, :].T - t)
        p4p = p4p / p4p[2]
        error = np.linalg.norm(p4p - p2p)
        if error < tolerance:
            tolerance = error
            R = R
            t = t
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
    #Transpose to make it 3*N
    #Find centroid of X and Y
    Y_av = np.mean(Y, axis=0)
    X_av = np.mean(X, axis=0)
    A = (Y - Y_av).T
    B = (X - X_av).T
    #Find SVD:
    U, S, Vt = np.linalg.svd(B @ A.T, full_matrices=True)
    #Find R
        #Make sure det(R) = 1
    s = np.eye((3))
    s[2, 2] = np.linalg.det(Vt.T @ U.T)
    r = Vt.T @ s
    R = r @ U.T
    #Find t from its definition
    t = Y_av - R @ X_av
    t = np.reshape(t, [3,])
    return R, t
