import numpy as np

def pose_candidates_from_E(E):
  ##Note: each candidate in the above list should be a dictionary with keys "T", "R"
  """ YOUR CODE HERE
  """
  U, S, Vt = np.linalg.svd(E, full_matrices=True)
  r = np.array([[0, -1, 0],
                [1, 0, 0],
                [0, 0, 1]])
  
  T_p = U[:, 2]
  T_n = (-1) * T_p

  T = [T_p, T_p, T_n, T_n]

  R_p = U @ r.T @ Vt
  R_n = U @ r @ Vt

  R = [R_p, R_n, R_p, R_n]

  transform_candidates = [{'T': T[i], 'R': R[i]} for i in range(0, 4)]

  """ END YOUR CODE
  """
  return transform_candidates