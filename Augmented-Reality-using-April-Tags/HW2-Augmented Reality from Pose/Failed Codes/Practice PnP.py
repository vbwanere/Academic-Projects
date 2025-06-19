from est_homography import est_homography
import numpy as np
def PnP(Pc, Pw, K=np.eye(3)):
    """
    Solve Perspective-N-Point problem with collineation assumption, given correspondence and intrinsic

    Input:
        Pc: 4x2 numpy array of pixel coordinate of the April tag corners in (x,y) format
        Pw: 4x3 numpy array of world coordinate of the April tag corners in (x,y,z) format
    Returns:
        R: 3x3 numpy array describing camera orientation in the world (R_wc)
        t: (3, ) numpy array describing camera translation in the world (t_wc)

    """
    # Homography Approach
    # Following slides: Pose from Projective Transformation

    #Drop z coordinate of Pw for homography est:
    Pwh = np.delete(Pw, 2, 1)

    #Estimate homography:
    H = est_homography(Pwh, Pc)

    #Estimate K_inv*H:
    K_inv = np.linalg.inv(K)
    H_p = np.matmul(K_inv, H)
    h1_p = H_p[:, 0]
    h2_p = H_p[:, 1]
    h3_p = H_p[:, 2]
    h_cr = np.cross(h1_p, h2_p)

    #Take SVD of (h1_p, h2_p, h_cr)
    USVt = np.stack((h1_p, h2_p, h_cr), axis=1)
    u, s, vh = np.linalg.svd(USVt, full_matrices=True)

    #Reconstruct singular values:
    det_uv = np.linalg.det(np.matmul(u, vh))
    s = np.eye(3)
    s[2, 2] = det_uv
    #Find R = u*s*vh
    us = u @ s
    Rc_w = us @ vh


    #Calculate translation: h3_p/|h1_p|
    Tc_w = h3_p/(np.linalg.norm(h1_p))
    # if Tc_w[2] < 0:
    #     Tc_w = - Tc_w

    #Find camera pose with respect to world:
    Rc_w_inv = np.linalg.inv(Rc_w)

    Rw_c = np.matmul(Rc_w_inv, K_inv)
    R = Rw_c

    Tw_c = -np.matmul(Rc_w_inv, Tc_w)
    t = Tw_c

    return R, t

s = 0.14
Pw = np.array([[-s/2, -s/2, 0],
               [s/2, -s/2, 0],
               [s/2, s/2, 0],
               [-s/2, s/2, 0]])

Pc = np.array([[-s/8, -s/8],
              [s/8, -s/8],
              [s/8, s/8],
              [-s/8, s/8]])

K = np.array([[823.8, 0.0, 304.8],
              [0.0, 822.8, 236.3],
              [0.0, 0.0, 1.0]])

P = PnP(Pc, Pw, K)
print(P)

