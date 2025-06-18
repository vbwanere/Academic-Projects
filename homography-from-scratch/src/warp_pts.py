import numpy as np
from est_homography import est_homography


def warp_pts(X, Y, interior_pts):
    """
    First compute homography from video_pts to logo_pts using X and Y,
    and then use this homography to warp all points inside the soccer goal

    Input:8
        X: 4x2 matrix of (x,y) coordinates of goal corners in video frame
        Y: 4x2 matrix of (x,y) coordinates of logo corners in penn logo
        interior_pts: Nx2 matrix of points inside goal
    Returns:
        warped_pts: Nx2 matrix containing new coordinates for interior_pts.
        These coordinate describe where a point inside the goal will be warped
        to inside the penn logo. For this assignment, you can keep these new
        coordinates as float numbers.

    """
    H = est_homography(X, Y)

    ips = interior_pts
    ips_hmgs = np.insert(ips, 2, 1, axis=1)
    warped_pts = []
    for i in range(0, interior_pts.shape[0]):
        wp = np.matmul(H, ips_hmgs[i])
        w_p = wp / (wp[2])
        w_p = np.reshape(w_p, (1, 3))
        w_p = np.delete(w_p, 2, axis=1)
        warped_pts.append(w_p)
    warped_pts = np.array(warped_pts)
    warped_pts = np.squeeze(warped_pts, axis=1)
    return warped_pts
