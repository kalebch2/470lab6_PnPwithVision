#!/usr/bin/env python

import cv2
import numpy as np

# ============================= Student's code starts here ===================================

# Params for camera calibration
theta = 0 
beta = 740.0
tx = -0.315878378378
ty = -0.146283783784
Or = 480/2 
Oc = 640/2 


# Function that converts image coord to world coord
def IMG2W(x,y):
	xw = (y - Or)/beta - tx
	yw = (x - Oc)/beta - ty

	xy_w = str(xw) + str(' ') + str(yw)

	return xy_w


def detector_init():

	# Setup SimpleBlobDetector parameters.
	params = cv2.SimpleBlobDetector_Params()

	# Filter by Color
	params.filterByColor = True
	params.blobColor = 255

	# Filter by Area.
	params.filterByArea = True
	params.minArea = 50
	params.maxArea = 700

	# Filter by Circularity
	params.filterByCircularity = True
	params.minCircularity = 0.35
	params.maxCircularity = 1.0

	# Filter by Inertia
	params.filterByInertia = True
	params.minInertiaRatio = 0.1
	params.maxInertiaRatio = 1.0

	# Filter by Convexity
	params.filterByConvexity = True
	params.minConvexity = 0.1
	params.maxConvexity = 1.0

	# Create a detector with the parameters
	blob_detector = cv2.SimpleBlobDetector_create(params)

	return blob_detector


def blob_search(detector, image_raw, color):
	########## CROP and MASK (w/ Red and Green HSVs) ##########
	image = image_raw.copy()

	hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

	if color == "red":
		lower = (0,100,100)
		upper = (35,256,256)
		lower2 = (150,100,100)
		upper2 = (180,256, 256)

	elif color == "green":
		lower = (40,100,100)
		upper = (90,256,256)
		lower2 = (40,100,100)
		upper2 = (90,256,256)

	mask_image = cv2.inRange(hsv_image, lower, upper) + cv2.inRange(hsv_image, lower2, upper2)

	crop_top_row = 155
	crop_bottom_row = 350
	crop_top_col = 185
	crop_bottom_col = 490
	crop_image = mask_image[crop_top_row:crop_bottom_row, crop_top_col:crop_bottom_col]

	########## Identify KEYPOINTS and draw CENTROIDS & CIRCLES ##########
	keypoints = detector.detect(crop_image)

	blob_image_center = []

	for k in keypoints:
		x = int(k.pt[0]) + crop_top_col
		y = int(k.pt[1]) + crop_top_row
		k.pt = tuple([x,y])

		blob_image_center.append(str(x)+' '+str(y))

		# Centroid
		image = cv2.circle(image, (x,y), 1, (0,255,0), -1)

	# Circle
	image = cv2.drawKeypoints(image, keypoints, np.array([]), (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

	########## BLOCK? Convert to WORLD COORDINATES ##########
	xw_yw = []

	if(len(blob_image_center) == 0):
		m=1
		#print("No block found!")
	else:
		for i in range(len(blob_image_center)):
			x = int(blob_image_center[i].split()[0])
			y = int(blob_image_center[i].split()[1])
			#print("Blob found! ({0}, {1})".format(x, y))

			xw_yw.append(IMG2W(x,y))

	cv2.namedWindow("Camera View")
	cv2.imshow("Camera View", image[100:360, 60:500])

	cv2.waitKey(2)

	return xw_yw