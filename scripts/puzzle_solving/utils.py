from __future__ import division
import numpy as np
import math


def distance(p1, p2):
	"""
	calculates and returns the distance between the 2 points p1, p2
	:param p1: array of size 2 representing (x,y)
	:param p2: array of size 2 representing (x,y)
	:return: distance between p1 and p2
	"""
	return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def epsilon_equal(n1, n2, epsilon):
	"""
	determines whether or not the difference between n1 and n2 is at most epsilon
	:param n1: number
	:param n2: number
	:param epsilon: number
	:return: true iff |n1 - n2| <= epsilon
	"""
	return np.abs(n1 - n2) <= epsilon


def get_centroid(polygon):
	"""
	calculates the centroid of the polygon
	:param polygon: array of arrays of size 2 representing (x,y) of the polygon's vertices
	:return: array representing (x,y) of the polygon's centroid
	"""
	x = [p[0] for p in polygon]
	y = [p[1] for p in polygon]
	centroid = [sum(x) / len(polygon), sum(y) / len(polygon)]
	return centroid


def calc_rotation_angle(polygon, rotated_polygon):
	"""
	:param polygon: original polygon
	:param rotated_polygon: same polygon but rotated with some angle theta
	:return: returns the rotation angle theta
	"""
	centroid = get_centroid(polygon)
	rotated_polygon_centroid = get_centroid(rotated_polygon)

	# selecting an arbitrary vertex in the polygon
	arbitrary_vertex = polygon[0]
	d = distance(arbitrary_vertex, centroid)

	# identifying this same vertex in the rotated polygon
	corresponding_vertex = None
	for vertex in rotated_polygon:
		dist = distance(vertex, rotated_polygon_centroid)
		if epsilon_equal(dist, d, 0.01):
			corresponding_vertex = vertex
			break

	if corresponding_vertex is None:
		print("Error: no vertex was found")

	class Vector:
		def __init__(self, x, y):
			self.x = x
			self.y = y

	vec1 = Vector(arbitrary_vertex[0] - centroid[0], arbitrary_vertex[1] - centroid[1])
	vec2 = Vector(corresponding_vertex[0] - rotated_polygon_centroid[0], corresponding_vertex[1] - rotated_polygon_centroid[1])
	rotation = math.atan2(vec2.y, vec2.x) - math.atan2(vec1.y, vec1.x)
	if rotation > np.pi:
		rotation -= 2 * np.pi
	elif rotation < -np.pi:
		rotation += 2 * np.pi
	return rotation
