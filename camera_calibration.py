import numpy as np
#import pandas as pd
import copy
import os
import math

def camera_calibration_matrix():
	# https: // www.mathworks.com / help / vision / ref / cameracalibrator - app.html
	# https: // www.youtube.com / watch?v = nRVuLFQ_Bng & ab_channel = CyrillStachniss
	# https: // www.analyticsvidhya.com / blog / 2021 / 10 / a - comprehensive - guide -for -camera - calibration - in -computer - vision /  #:~:text=The%20camera%20matrix%20is%20a,algorithm%20computes%20the%20camera%20matrix.

	import cv2
	import numpy as np
	import glob
	# Defining the dimensions of checkerboard
	CHECKERBOARD = (6, 8)
	criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
	# Creating vector to store vectors of 3D points for each checkerboard image
	objpoints = []
	# Creating vector to store vectors of 2D points for each checkerboard image
	imgpoints = []
	# Defining the world coordinates for 3D points
	objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
	objp[0, :, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
	prev_img_shape = None
	# Extracting path of individual image stored in a given directory
	source_path =  r'C:\Users\melzo\OneDrive\Documents\GitHub\targeting-math-sim\images'
	images = [f for f in glob.glob('picam-pictures/*.jpg')]
	#images = glob.glob('./images/*.jpg')

	for fname in images:
		img = cv2.imread(fname)
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		# Find the chess board corners
		# If desired number of corners are found in the image then ret = true
		ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD,
												 cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
		"""
        If desired number of corner are detected,
        we refine the pixel coordinates and display
        them on the images of checker board
        """
		if ret == True:
			objpoints.append(objp)
		# refining pixel coordinates for given 2d points.
			corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
			imgpoints.append(corners2)
		# Draw and display the corners
		#img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
		#cv2.imshow('img', img)
		#cv2.waitKey(0)
	cv2.destroyAllWindows()
	h, w = img.shape[:2]


	"""
    Performing camera calibration by
    passing the value of known 3D points (objpoints)
    and corresponding pixel coordinates of the
    detected corners (imgpoints)
    """

	ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
	#print("Camera matrix : n")
	#print(mtx)
	#print("dist : n")
	#print(dist)
	#print("rvecs : n")
	#print(rvecs)
	#print("tvecs : n")
	#print(tvecs)

	return mtx

##########

path = "./picam-pictures/" # Where to store these files relative to current path (concatenates with filename so add trailing "/")
filename = "image" # Base name of image files 
height = 480 # Height in pixels of output image
width = 640 # Width in pixels of output image
delay = 5000 # Delay between each picture (ms)
pictures = 20 # Number of pictures you want taken

#for i in range(pictures):
#	os.system("raspistill -o " + path + filename + str(i) + ".jpg -h " + str(height) + " -w " + str(width) + " -t " + str(delay))

CCM = camera_calibration_matrix()
CCM_inv = np.linalg.inv(CCM)

# adding column and row to be able to multiply CCM_inv with other matrices
column_to_be_added = np.array([[0], [0], [0]])
newrow = np.array([0, 0, 0, 1])
CCM_inv = np.append(CCM_inv, column_to_be_added, axis=1)
CCM_inv = np.vstack([CCM_inv, newrow])

print(CCM)
print(CCM_inv)




