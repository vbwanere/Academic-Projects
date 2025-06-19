import numpy as np

def est_pixel_world(pixels, R_wc, t_wc, K):
    """
    Estimate the world coordinates of a point given a set of pixel coordinates.
    The points are assumed to lie on the x-y plane in the world.
    Input:
        pixels: N x 2 coordinates of pixels
        R_wc: (3, 3) Rotation of camera in world
        t_wc: (3, ) translation from world to camera
        K: 3 x 3 camara intrinsics
    Returns:
        Pw: N x 3 points, the world coordinates of pixels
    """

    t_wc = np.expand_dims(t_wc, axis=1)

    #Take transpose of pixels for convenience and convert to homogeneous coordinates:
    pixels_1 = np.transpose(pixels)
    ones = np.ones((1, pixels_1.shape[1]))
    pixels_tr = np.concatenate((pixels_1, ones), axis=0)

    K_inv = np.linalg.inv(K)
    R_K = R_wc @ K_inv

    #Expression for world coordinates:
    P1 = (R_K) @ pixels_tr + t_wc
    b = P1[2, :]
    P2 = P1/b
    P2[2, :] = 0
    Pw = np.transpose(P2)

    return Pw
