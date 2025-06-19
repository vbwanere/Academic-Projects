import numpy as np
import math

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
    Y = Y.T
    X = X.T
    #Find centroid of X and Y
    Y_av = np.mean(Y, axis=1)
    X_av = np.mean(X, axis=1)
    A = Y - Y_av
    B = X - X_av
    #Find SVD:
    U, S, Vt = np.linalg.svd(B @ A.T, full_matrices=True)
    #Find R
        #Make sure det(R) = 1
    s = np.eye((3))
    s[2, 2] = np.linalg.det(Vt.T @ U.T)
    r = Vt.T @ s
    R_wc = r @ U.T
    R = R_wc
    #Find t from its definition
    t = Y_av - R @ X_av
    return R, t

Pc = np.array([[-200, -200],
               [200, -200],
               [200, 200],
               [-200, 200]])

Pw = np.array([[-10, -10, 0],
               [10, -10, 0],
               [10, 10, 0],
               [-10, 10, 0]])

K = np.array([[1000, 0, 0],
              [0, 1000, 0],
              [0, 0, 1]])
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

#Define world coordinates for correspondances with pixel coordinates:
pw = np.ones((3, 3))
pw[0, :] = Pw[0, :]
pw[1, :] = Pw[3, :]
pw[2, :] = Pw[1, :]

# Define points in world to find distance:
Pwh = np.delete(Pw, 2, 1)
P1 = Pwh[0, :]
P2 = Pwh[3, :]
P3 = Pwh[1, :]
ones = np.ones((Pwh.shape[0], 1))
Pw = np.concatenate((Pwh, ones), axis=1)

#Find distances between points in world:
a = math.dist(P2, P3)
b = math.dist(P1, P3)
c = math.dist(P1, P2)
ac = (a - c) / b
ab = (a + c) / b
bc = (b - c) / b
ba = (b - a) / b

#Define projections of the above points on image plane in camera coordinates
#p = np.array([[Pc[0, 0] - K[0, 2], Pc[0, 1] - K[1, 2], K[0, 0]],
#              [Pc[3, 0] - K[0, 2], Pc[3, 1] - K[1, 2], K[0, 0]],
#              [Pc[1, 0] - K[0, 2], Pc[1, 1] - K[1, 2], K[0, 0]]])
p = np.array([[Pc[0, 0], Pc[0, 1], 1],
              [Pc[3, 0], Pc[3, 1], 1],
              [Pc[1, 0], Pc[1, 1], 1]])
p = (np.linalg.inv(K) @ p.T).T

#Calculate unit vectors from point p1, p2 and p3 in camera coodinates
# to calculate angles alpha, beta and gamma:
pu = np.ones((3, 3))
for i in range(pu.shape[0]):
    pu[i, :] = p[i, :] / (np.linalg.norm(p[i, :]))

#Find angles between the vectors:
al = np.dot(pu[1, :], pu[2, :])
bt = np.dot(pu[0, :], pu[2, :])
gm = np.dot(pu[0, :], pu[1, :])

#Frame the Polynomial:
A4 = (ac - 1)**2 - 4 * c / b * al * al
A3 = 4 * (ac * (1-ac) * bt - (1 - ab) * al * gm + 2 * c / b * al * al * bt)
A2 = 2 * (ac * ac - 1 + 2 * ac * ac * bt * bt + 2 * bc * al * al -
          4 * ab * al * bt * gm + 2 * ba * gm * gm)
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
p2p = np.array([Pc[2, 0], Pc[2, 1], 1])
for i in range(s.shape[1]):
    Pc3 = p * s[:, i]
    Pc3 = Pc3.T
    R, t = Procrustes(Pc3, pw)
    p4p = K @ np.linalg.inv(R) @ (Pw[2, :].T - t)
    p4p = p4p / p4p[2]
    error = np.linalg.norm(p4p - p2p)
    if error < tolerance:
        tolerance = error
        R = R
        t = t
a = np.array([2, 4, 5, 3])
a = np.reshape(a, (4, 1))
print(a*a)

Y = np.ones((4, 3))
Y_b = print(np.mean(Y))



a 5.196152422706632
b 2.8284271247461903
c 5.916079783099616
pu1 [0.00899956 0.00399981 0.9999515 ]
pu2 [0.00699966 0.0019999  0.9999515 ]
pu3 [0.00399981 0.00299985 0.9999515 ]
cos_al 0.9999370061104073
cos_bt 0.9999510047525391
cos_gm 0.9999740025217554
A [-13.497795283308278, 53.991865188262196, -80.98964242697613, 53.994870628809366, -13.499298077211622]
v [0.99995102 0.99995102 0.99502927 1.00511938]
u -0.0001406703915718881
s1 285.73164241577945
s2 -0.04019398202310641
s3 285.71764637915817
u -0.0001406703915718881
s1 285.73164241577945
s2 -0.04019398202310641
s3 285.71764637915817
u 0.9780039941773322
s1 255.85196454145378
s2 250.22424323965896
s3 254.58019232820644
u 1.0221904113332583
s1 253.28613961576454
s2 258.9066632388514
s3 254.58280759467362
R [[ 0.73511939 -0.66003263 -0.15477857]
 [ 0.22865094  0.45631918 -0.85993695]
 [ 0.63821488  0.59676606  0.48636616]]
t [  45.69925863  225.94647201 -122.55596377]


AJG

a 5.196152422706632
b 2.8284271247461903
c 5.916079783099616
j1 [[0.00899956 0.00399981 0.9999515 ]]
j2 [[0.00699981 0.00199995 0.9999735 ]]
j3 [[0.00399995 0.00299996 0.9999875 ]]
cos_a [[0.999995]]
cos_b [[0.999987]]
cos_g [[0.999996]]
A [-13.49982501  53.99905609 -80.99852916  53.99919009 -13.49989201]
v [0.99998728 0.99998728 1.00151786 0.99848951]
u [[-0.02002257]]
s1 [[554.73274095]]
s2 [[-11.10717626]]
s3 [[554.72568266]]
Pcam [[ 4.99235255e+00  2.21882335e+00  5.54705838e+02]
 [-7.77481735e-02 -2.22137639e-02 -1.11068819e+01]
 [ 2.21887499e+00  1.66415625e+00  5.54718749e+02]]
u [[-0.02002257]]
s1 [[554.73274095]]
s2 [[-11.10717626]]
s3 [[554.72568266]]
Pcam [[ 4.99235255e+00  2.21882335e+00  5.54705838e+02]
 [-7.77481735e-02 -2.22137639e-02 -1.11068819e+01]
 [ 2.21887499e+00  1.66415625e+00  5.54718749e+02]]
u [[1.01076572]]
s1 [[531.30187238]]
s2 [[537.0217189]]
s3 [[532.10831577]]
Pcam [[  4.78148496   2.12510442 531.27610611]
 [  3.75905242   1.07401498 537.00748839]
 [  2.12840666   1.59630499 532.10166454]]
u [[0.98924683]]
s1 [[532.25177195]]
s2 [[526.5283781]]
s3 [[531.44781294]]
Pcam [[  4.79003364   2.12890384 532.22595962]
 [  3.68560098   1.05302885 526.51442565]
 [  2.12576468   1.59432351 531.44116996]]
[[ 0.90453976 -0.32477933  0.27627198]
 [ 0.42200129  0.58917844 -0.68904548]
 [ 0.06101424  0.73985617  0.66999262]]
[-143.41154579  370.80352261 -354.81506518]



