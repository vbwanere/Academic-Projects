from lse import least_squares_estimation
import numpy as np

def ransac_estimator(X1, X2, num_iterations=6):
    sample_size = 8

    eps = 10**-4

    best_inliers = None
    best_E = None
    best_num_inliers = -1

    for i in range(num_iterations):
        permuted_indices = np.random.RandomState(seed=(i*10)).permutation(np.arange(X1.shape[0]))
        sample_indices = permuted_indices[:sample_size]
        test_indices = permuted_indices[sample_size:]
        inliers = np.array([])

        """ YOUR CODE HERE
        """

        E = least_squares_estimation(X1[sample_indices, :], X2[sample_indices, :])

        for j in range(test_indices.shape[0]):
            d1 = ((X1[test_indices[j], :] @ E.T @ X2[test_indices[j], :]).item() ** 2) \
                / ((E.T @ X2[test_indices[j], :])[0] ** 2 + \
                          (E.T @ X2[test_indices[j], :])[1] ** 2)
            
            d2 = ((X2[test_indices[j], :] @ E @ X1[test_indices[j], :]).item() ** 2) \
                / ((E @ X1[test_indices[j], :])[0] ** 2 + \
                          (E @ X1[test_indices[j], :])[1] ** 2)
            e = d1 + d2

            if e < eps:
                inliers = np.append(inliers, test_indices[j])

        inliers = np.concatenate((sample_indices, inliers), axis=None).astype(int)
        
        """ END YOUR CODE
        # """
        if inliers.shape[0] > best_num_inliers:
            best_num_inliers = inliers.shape[0]
            best_E = E
            best_inliers = inliers
            
    return best_E, best_inliers