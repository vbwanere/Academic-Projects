import numpy as np
from est_homography import est_homography
# corners = np.load('corners.npy')
# print(corners)
s = 0.14
Pw = np.array([[-s/2, -s/2],
               [s/2, -s/2],
               [s/2, s/2],
               [-s/2, s/2]])

Pc = np.array([[-s/8, -s/8],
              [s/8, -s/8],
              [s/8, s/8],
              [-s/8, s/8]])

H = est_homography(Pw, Pc)

K = np.array([[823.8, 0.0, 304.8],
              [0.0, 822.8, 236.3],
              [0.0, 0.0, 1.0]])
K_inv = np.linalg.inv(K)
H_d = np.matmul(K_inv, H)

#print(np.matmul(H, [-s/2, -s/2, 1]))

u, s, vh = np.linalg.svd(H_d, full_matrices=True)

# print("u = ", u)
# print("\ns = ", s)
# print("\nvh = ", vh[:2, :2])

r1_r2 = u[:, :2] @ vh[:2, :2]
r1 = r1_r2[:, 0]
r2 = r1_r2[:, 1]
c = np.cross(r1, r2)
lam = np.mean(s[:2])
Tc_w = H_d[:, 2] / lam
Rc_w1 = np.stack((r1, r2, c), axis=1)
#print(Rc_w1)
#Making det(Rc_w1) = 1
u, s, vh = np.linalg.svd(Rc_w1, full_matrices=True)
det_uv = np.linalg.det(np.matmul(u, vh))
s = np.eye(3)
s[2, 2] = det_uv
us = u @ s
Rc_w = us @ vh
#print(Rc_w - Rc_w1)
#detRc_w = np.linalg.det(Rc_w)

Rc_w_inv = np.linalg.inv(Rc_w)

Rw_c = np.matmul(Rc_w_inv, K_inv)
R = Rw_c

Tw_c = np.matmul(Rc_w_inv, Tc_w)
t = Tw_c
# print(r1_r2)
# print(r1)
# print(r2)
# print(c)
# print(Rc_w)
# print(Tc_w)

#print("Rw_c = \n", R)
#print("\nTw_c = \n", t)

s = 0.14
# Pw = np.array([[-s/2, -s/2],
#                [s/2, -s/2],
#                [s/2, s/2],
#                [-s/2, s/2]])

Pc = np.array([[-s/8, -s/8],
              [s/8, -s/8],
              [s/8, s/8],
              [-s/8, s/8]])

R_wc = np.array([[1, 2, 2],
                 [0.5, 1, 0.3],
                 [1, 1.5, 2]])

pixels = np.array([[12, 18],
                   [15, 24],
                   [3, 12],
                   [20, 14],
                   [8, 6]])
t_wc = np.ones((3, ))
t_wc = np.expand_dims(t_wc, axis=1)
K = np.array([[823.8, 0.0, 304.8],
              [0.0, 822.8, 236.3],
              [0.0, 0.0, 1.0]])

pixels_1 = np.transpose(pixels)
ones = np.ones((1, pixels_1.shape[1]))
pixels_tr = np.concatenate((pixels_1, ones), axis=0)

R_cw = np.linalg.inv(R_wc)
K_inv = np.linalg.inv(K)
R_K = R_cw @ K_inv
R_t = R_cw @ t_wc

P1 = (R_K) @ pixels_tr -(R_t)
b = P1[2, :]
P2 = P1/b
P2[2, :] = 0
Pw = P2
print(Pw)

H = np.array([[1, 2, 4, 3],
              [2, 3, 5, 6],
              [6, 8, 7, 2],
              [0, 0, 0, 1]])

R = np.array([[1, 2, 4],
              [2, 3, 5],
              [6, 8, 7]])

T = np.array([4, 5, 7])

R1 = np.linalg.inv(R)

print(R1 @ T)
