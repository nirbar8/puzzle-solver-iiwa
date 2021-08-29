#! /usr/bin/env python
import cv2
import time
import sys
import data


def snapshot(image_path='snapshot.jpg'):
	cap = cv2.VideoCapture(0)
	cap.set(cv2.CAP_PROP_FRAME_WIDTH, data.IMAGE_RESOLUTION[0])
	cap.set(cv2.CAP_PROP_FRAME_HEIGHT, data.IMAGE_RESOLUTION[1])
	cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)  # auto focus
	time.sleep(1)  # time for focus
	(_, frame) = cap.read()
	cv2.imwrite(image_path, frame)
	cap.release()
	return image_path


if __name__ == '__main__':
	sys.exit(snapshot())
