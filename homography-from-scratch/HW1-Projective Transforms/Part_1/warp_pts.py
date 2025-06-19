import numpy as np
from est_homography import est_homography


def warp_pts(X, Y, interior_pts):
    """
    First compute homography from video_pts to logo_pts using X and Y,
    and then use this homography to warp all points inside the soccer goal

    Input:
        X: 4x2 matrix of (x,y) coordinates of goal corners in video frame
        Y: 4x2 matrix of (x,y) coordinates of logo corners in penn logo
        interior_pts: Nx2 matrix of points inside goal
    Returns:
        warped_pts: Nx2 matrix containing new coordinates for interior_pts.
        These coordinate describe where a point inside the goal will be warped
        to inside the penn logo. For this assignment, you can keep these new
        coordinates as float numbers.

    """
    H = est_homography(Y, X)
    X = np.array([[141.54, 175.33], [293.25, 142.65], [295.5, 219.99], [144.90, 261.38]])
    Y = np.array([[0, 0], [990, 0], [990, 400], [0, 400]])
    interior_pts = ([[140, 170],
                     [290, 140],
                     [291, 215],
                     [140, 260]])
    ips = interior_pts
    ips_hmgs = np.insert(ips, 2, 1, axis=1)
    warp_points = []
    for i in range(0, 4):
        wp = np.matmul(H, ips_hmgs[i])
        w_p = wp / (wp[2])
        w_p = np.reshape(w_p, (1, 3))
        w_p = np.delete(w_p, 2, axis=1)
        warp_points.append(w_p)
    warp_points = np.array(warp_points)
    warp_points = np.reshape(warp_points, (4, 2))

    return warp_pts
