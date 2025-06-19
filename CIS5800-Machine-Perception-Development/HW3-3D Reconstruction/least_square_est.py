import numpy as np

def least_squares_estimation(X1, X2):

  """ YOUR CODE HERE
  """
  A = np.zeros((X1.shape[0], 9))

  for i in range(X1.shape[0]):
    A[i, :] = np.reshape(np.matmul(X2[i, :].reshape(1, 3).T, X1[i, :].reshape(1, 3)).T, (1, 9))
  

  # print("A = ", A)
  
  U, S, Vt = np.linalg.svd(A, full_matrices=True)
  E1 = Vt[8, :].reshape(3, 3)
  E1 = E1.T

  u, s, vt = np.linalg.svd(E1, full_matrices=True)
  eye = np.eye(E1.shape[0])
  eye[2, 2] = 0
  E = u @ eye @ vt

  """ END YOUR CODE
  """
  return E
