from __future__ import division
import numpy as np
import cv2
import csv
import data
import random


def poly_to_int(poly):
	return [[int(p[0]), int(p[1])] for p in poly]


def draw_polygons(polygons=[], image_path=('polygons' + str(random.randint(1, 1000)) + '.jpg')):
	img = np.zeros((data.IMAGE_RESOLUTION[1], data.IMAGE_RESOLUTION[0], 3), dtype="uint8")
	for poly in polygons:
		cv2.polylines(img, [np.array(poly_to_int(poly))], True, (0, 255, 0), thickness=1)
	cv2.imwrite(image_path, img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()


def detect_shapes(img_path, output_csv_path='piecesVertices.csv', thresh_range=(80, 255), approx_epsilon=0.017):
	img = cv2.imread(img_path)
	img = cv2.GaussianBlur(img, (1, 1), 0)
	img = cv2.resize(img, data.IMAGE_RESOLUTION)

	img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

	# lower mask (0-10)
	lower_red = np.array([0, 40, 40])
	upper_red = np.array([20, 255, 255])
	mask0 = cv2.inRange(img_hsv, lower_red, upper_red)

	# upper mask (170-180)
	lower_red = np.array([160, 40, 40])
	upper_red = np.array([180, 255, 255])
	mask1 = cv2.inRange(img_hsv, lower_red, upper_red)

	# join my masks
	mask = mask0 + mask1

	output_img = img.copy()
	output_img[np.where(mask == 0)] = 0

	output_img = cv2.cvtColor(output_img, cv2.COLOR_HSV2BGR)
	output_img = cv2.cvtColor(output_img, cv2.COLOR_BGR2GRAY)

	_, output_img = cv2.threshold(output_img, thresh_range[0], thresh_range[1], cv2.THRESH_BINARY_INV)

	# open the file in the write mode
	with open(output_csv_path, 'w') as f:
		# create the csv writer
		writer = csv.writer(f)

		writer.writerow(['piece', 'x', 'y'])

		# inverting the image
		output_img = 255 - output_img
		cv2.imshow("img", output_img)
		cv2.waitKey(0)
		cv2.destroyAllWindows()

		contours = cv2.findContours(output_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[-2]
		piece_num = 0
		for contour in contours:
			approx = cv2.approxPolyDP(contour, approx_epsilon * cv2.arcLength(contour, closed=True), closed=True)
			approx_int = [[int(point[0][0]), int(point[0][1])] for point in approx]
			approx_area = cv2.contourArea(np.array(approx_int))
			if approx_area < 700:
				continue

			print(len(approx))
			cv2.drawContours(img, [approx], 0, (0, 255, 255), 3)

			for point in approx:
				x = point[0][0]
				y = point[0][1]
				writer.writerow([piece_num, x / data.IMAGE_SCALING_FACTOR, y / data.IMAGE_SCALING_FACTOR])

			piece_num += 1

	cv2.imwrite('camera_capture.jpg', img)
	cv2.imshow('result', img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()


if __name__ == '__main__':
	detect_shapes('/home/kuka/.ros/snapshot.jpg')
