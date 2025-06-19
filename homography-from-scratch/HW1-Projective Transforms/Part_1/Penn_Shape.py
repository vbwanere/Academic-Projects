import os
import glob
import numpy as np
import cv2


penn = cv2.imread("../data/barcelona/images/logos/penn_engineering_logo.png")
penn_y, penn_x, _ = penn.shape
penn_corners = np.array([[0, 0], [penn_x, 0], [penn_x, penn_y], [0, penn_y]])
print(penn_corners)

