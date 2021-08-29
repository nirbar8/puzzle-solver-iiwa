#! /usr/bin/env python
from robot_controller import *
import rospy, moveit_commander
from geometry_msgs.msg import Pose
import shape_detection
import csv
import sys
import os
import time
import data


def init_ros():
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('solver_node', anonymous=True)
	publisher = rospy.Publisher('move_iiwa_gaze', Pose, queue_size=10)
	controller = RobotController()
	return controller, publisher


def parse_pieces_csv(path_to_pieces):
	pieces = []
	with open(path_to_pieces, 'r') as file:
		csv_reader = csv.reader(file)
		_ = next(csv_reader)

		current_piece_num = 0
		current_piece = []
		for row in csv_reader:
			if int(row[0]) != current_piece_num:  # point of new piece
				pieces.append(current_piece)  # adding the last piece in pieces
				current_piece = []  # moving to new piece
				current_piece_num = int(row[0])  # new piece index
			current_piece.append([float(row[1]), float(row[2])])  # inserting x,y of point
		pieces.append(current_piece)
		return pieces


def normalize_pieces(pieces):
	flat_points = [item for sublist in pieces for item in sublist]
	xs = [p[0] for p in flat_points]
	ys = [p[1] for p in flat_points]
	x_range = (min(xs), max(xs))
	y_range = (min(ys), max(ys))

	if x_range[1] - x_range[0] > 100 or y_range[1] - y_range[0] > 100:
		print("distance between pieces is too big to normalize - probably solver failed")
		return ValueError

	pieces = [[[point[0] - x_range[0], point[1] - y_range[0]] for point in piece] for piece in pieces]
	return pieces


def wait_for_solver_results():
	while not os.path.exists(data.RESULT_PIECES_PATH):
		print("waiting for results file...")
		time.sleep(2)


def main():
	controller, publisher = init_ros()
	rospy.sleep(5)
	controller.take_puzzle_picture()
	rospy.sleep(1)
	shape_detection.detect_shapes('snapshot.jpg', data.INPUT_PIECES_PATH)
	rospy.sleep(1)
	pieces = parse_pieces_csv(data.INPUT_PIECES_PATH)
	pieces = [[[point[0] * data.IMAGE_SCALING_FACTOR, point[1] * data.IMAGE_SCALING_FACTOR] for point in piece] for piece in pieces]
	shape_detection.draw_polygons(pieces, 'polygons.jpg')
	wait_for_solver_results()

	result_pieces = parse_pieces_csv(data.RESULT_PIECES_PATH)
	result_pieces = normalize_pieces(result_pieces)
	result_pieces = [[[point[0] * data.IMAGE_SCALING_FACTOR, point[1] * data.IMAGE_SCALING_FACTOR] for point in piece] for piece in result_pieces]
	shape_detection.draw_polygons(result_pieces, 'assembled_polygons.jpg')
	rospy.loginfo("waiting for raspberry pi service...")
	rospy.wait_for_service('/iiwa/piface')  # required for suction of pieces
	controller.assemble_puzzle(pieces, result_pieces)
	rospy.sleep(5)
	rospy.loginfo("deleting csv files...")
	os.remove(data.INPUT_PIECES_PATH)
	os.remove(data.RESULT_PIECES_PATH)


if __name__ == '__main__':
	sys.exit(main())
