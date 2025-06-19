import numpy as np

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
    # Define points in world to find distance:
    Ki = np.linalg.inv(K)
    Pwh = np.delete(Pw, 2, 1)
    ones = np.ones((Pwh.shape[0], 1))
    Pw1 = np.concatenate((Pwh, ones), axis=1)
    a = np.linalg.norm(Pw[2, :] - Pw[3, :])
    b = np.linalg.norm(Pw[1, :] - Pw[3, :])
    c = np.linalg.norm(Pw[1, :] - Pw[2, :])

    ph = np.array([[Pc[1, 0], Pc[1, 1], 1],
                  [Pc[2, 0], Pc[2, 1], 1],
                  [Pc[3, 0], Pc[3, 1], 1]])
    pt = ph.T
    p = Ki @ pt
    pu = np.ones((3, 3))
    for i in range(pu.shape[0]):
        pu[i, :] = (Ki @ p[i, :]) / (np.linalg.norm(Ki @ p[i, :]))
    cos_al = np.dot((p[:, 1] / np.linalg.norm(p[:, 1])), (p[:, 2] / np.linalg.norm(p[:, 2])).T)
    cos_bt = np.dot((p[:, 0] / np.linalg.norm(p[:, 0])), (p[:, 2] / np.linalg.norm(p[:, 2])).T)
    cos_gm = np.dot((p[:, 0] / np.linalg.norm(p[:, 0])), (p[:, 1] / np.linalg.norm(p[:, 1])).T)

    A0 = (1 + (a ** 2 - c ** 2) / b ** 2) ** 2 - 4 * (a ** 2 / b ** 2) * cos_gm ** 2
    A1 = 4 * (-((a ** 2 - c ** 2) / b ** 2) * (1 + ((a ** 2 - c ** 2) / b ** 2)) * cos_bt + 2 * (a ** 2 / b ** 2) *
              (cos_gm ** 2) * cos_bt - (1 - ((a ** 2 + c ** 2) / b ** 2)) * cos_al * cos_gm)
    A2 = 2 * (((a ** 2 - c ** 2) / b ** 2) ** 2 - 1 + 2 * ((a ** 2 - c ** 2) / b ** 2) ** 2 * cos_bt ** 2 + 2 *
              ((b ** 2 - c ** 2) / b ** 2) * cos_al ** 2 - 4 * ((a ** 2 + c ** 2) / b ** 2) * cos_al * cos_bt * cos_gm + 2 *
              ((b ** 2 - a ** 2) / b ** 2) * cos_gm ** 2)
    A3 = 4 * (((a ** 2 - c ** 2) / b ** 2) * (1 - ((a ** 2 - c ** 2) / b ** 2)) * cos_bt -
              (1 - ((a ** 2 + c ** 2) / b ** 2)) * cos_al * cos_gm + 2 * (c ** 2 / b ** 2) * (cos_al ** 2) * cos_bt)
    A4 = ((a ** 2 - c ** 2) / b ** 2 - 1) ** 2 - 4 * (c ** 2 / b ** 2) * cos_al ** 2

    A = [A4, A3, A2, A1, A0]
    v = np.roots(A)
    v = v.real
    v = np.array(v)
    v = v[v > 0]
    v = v.reshape((len(v), 1))
    u = np.ones((4, 1))

    for i in range(len(v)):
        u[i, 0] = ((-1 + (a ** 2 - c ** 2) / b ** 2) * np.square(v[i]) - 2 * ((a ** 2 - c ** 2) / b ** 2) * cos_bt * v[i] +
                   1 + ((a ** 2 - c ** 2) / b ** 2)) / (2 * (cos_gm - v[i] * cos_al))
    # Calculating s1, s2, s3:
    s1 = np.ones((len(v), 1))
    for i in range(len(v)):
        s1[i, 0] = math.sqrt(c ** 2 / (1 + u[i, 0] * u[i, 0] - 2 * u[i, 0] * cos_gm))
    s2 = u * s1
    s3 = v * s1
    s = np.concatenate((s1, s2, s3), axis=1)
    s = s.transpose()
    # 3D vertex of points P1, P2 and P3 in camera coordinates from s1, s2, s3:
    tolerance = 10000000000
    p2p = np.array([Pc[2, 0], Pc[2, 1], 1])
    for i in range(s.shape[1]):
        Pc3 = p.T * s[:, i]
        Pc3 = Pc3.T
        R, t = Procrustes(Pc3, Pw[1:])
        p4p = Ki @ np.linalg.inv(R) @ (Pw[3, :].T + t)
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




