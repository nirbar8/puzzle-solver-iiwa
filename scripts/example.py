#! /usr/bin/env python
import rospy, sys, moveit_commander
from rospy import init_node, is_shutdown
from geometry_msgs.msg import Pose, Point, Quaternion
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

coord = {'home_pos': [0, 0.5, 0.5, 0.70710678118, -0.70710678118, 0, 0],
         'cam_pos': [0.08, 0.56, 0.722, -0.713126476786, 0.700516669565, 0.0170209801914, 0.0209119583364],
         'example1': [0.5, 0.5, 0.3, 0.70710678118, -0.70710678118, 0, 0],
         'example2': [0.123257153412, 0.498227583973, 0.73370046098, 0.703620939698, -0.706956192032, -0.0653941208549,
                      0.0292254122952],
         'example3': [0.5, 0.3, 0.1, 1, 0, 0, 0],
         'example4': [0.5, 0.3, 0.3, 1, 0, 0, 0],
         'example5': [0.4, 0.6, 0.3, 1, 0, 0, 0],
         'example6': [0.4, 0.6, 0.1, 1, 0, 0, 0],
         'paper_1above': [-0.4679, 0.3694, 0.4428, -0.9946, 0.02399, 0.09074, 0.04323],
         'paper_1': [-0.4679, 0.3694, 0.22, -0.9946, 0.02399, 0.09074, 0.04323],
         'release': [-0.035, 0.741, 0.2373, 0.791, -0.6019, -0.09735, 0.04579],
         'release_above': [-0.035, 0.741, 0.4373, 0.791, -0.6019, -0.09735, 0.04579],
         }


class abc:
	def __init__(self, groupname='manipulator'):
		moveit_commander.roscpp_initialize(sys.argv)
		self._robot = moveit_commander.RobotCommander()
		self._scenes = moveit_commander.PlanningSceneInterface()
		self._groupname = groupname
		self._group = moveit_commander.MoveGroupCommander(self._groupname)
		self._group.set_max_velocity_scaling_factor(0.1)  # limit the max speed of the joints

	def _go_to_pose(self, re_po=[]):
		order = list()
		pose = geometry_msgs.msg.Pose()

		order = (Pose(position=Point(x=re_po[0], y=re_po[1], z=re_po[2]),
		              orientation=Quaternion(x=re_po[3], y=re_po[4], z=re_po[5], w=re_po[6])))
		self._group.set_pose_target(order)
		self._group.go(wait=True)
		self._group.stop()
		self._group.clear_pose_targets()

	def _select_point(self):
		self._go_to_pose(coord['cam_pos'])
		rospy.sleep(2)
		'''
		self._go_to_pose(coord['example1'])
		rospy.sleep(2)
		self._go_to_pose(coord['example2'])
		rospy.sleep(2)
		self._go_to_pose(coord['example3'])
		rospy.sleep(2)
		self._go_to_pose(coord['example4'])
		rospy.sleep(2)
		self._go_to_pose(coord['example5'])
		rospy.sleep(2)		
		self._go_to_pose(coord['example6'])
		rospy.sleep(2)
		'''
		return


def main():
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('move_iiwa_gaze', anonymous=True)
	pub = rospy.Publisher('move_iiwa_gaze', Pose, queue_size=10)
	ca = abc(groupname='manipulator')
	ca._select_point()


if __name__ == '__main__':
	sys.exit(main())
