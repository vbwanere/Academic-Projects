"""
Visualization script for part 1 (Barcelona)

Note: You don't have to change this script for the assignment, but you
can if you'd like to change the images or other parameters

"""

import os
import glob
import numpy as np
import cv2

from warp_pts import warp_pts
import utils


def main():
    # Load Penn logo image, and get its corners
    penn_logo = cv2.imread("/home/vbwanere/OneDrive/GitHub_repos/GitHub_repos_python/homography_from_scratch/data/images/logos/penn_engineering_logo.png")
    penn_logo_y, penn_logo_x, _ = penn_logo.shape
    penn_logo_corners = np.array([[0, 0], [penn_logo_x, 0], [penn_logo_x, penn_logo_y], [0, penn_logo_y]])

    # Load all image paths, and the goal corners in each image
    img_files = sorted(glob.glob("/home/vbwanere/OneDrive/GitHub_repos/GitHub_repos_python/homography_from_scratch/data/images/barca_real/*.png"))
    goal_data = np.load("/home/vbwanere/OneDrive/GitHub_repos/GitHub_repos_python/homography_from_scratch/data/BarcaReal_pts.npy")

    # Process all images
    processed_imgs = []
    for i in range(len(goal_data)):
        goal = cv2.imread(img_files[i])
        goal_corners = goal_data[i]
        # Warping
        int_pts = utils.calculate_interior_pts(goal.shape, goal_corners)
        warped_pts = warp_pts(goal_corners, penn_logo_corners, int_pts)
        projected_img = utils.inverse_warping(goal, penn_logo, int_pts, warped_pts)

        processed_imgs.append(projected_img)
        print("Processed images " + str(i+1) + " of " + str(len(goal_data)))

    # Save some examples
    save_ind = [0, 25, 50, 75, 100, 125]
    if not os.path.exists("part_1_results"):
        os.mkdir("results")

    for ind in save_ind:
        cv2.imwrite("results/frame_" + str(ind) + ".png", processed_imgs[ind])

    # Visualize the sequence of projected images
    for im in processed_imgs:
        cv2.imshow("display", im)
        cv2.waitKey(200)  
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()