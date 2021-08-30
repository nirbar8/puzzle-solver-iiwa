import rospy, sys, moveit_commander
from rospy import init_node, is_shutdown
from geometry_msgs.msg import Pose, Point, Quaternion
from std_srvs.srv import *
from kukaio.srv import *
import moveit_commander
import moveit_msgs.msg
import numpy as np
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import shape_detection
import camera_capture
import utils
import cv2
import data


class RobotController:
    def __init__(self, groupname=data.GROUP_NAME):
        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scenes = moveit_commander.PlanningSceneInterface()
        self._groupname = groupname
        self._group = moveit_commander.MoveGroupCommander(self._groupname)
        self._group.set_max_velocity_scaling_factor(0.5)  # limit the max speed of the joints
        self._group.set_max_acceleration_scaling_factor(0.5)  # limit the max speed of the joints
        self._suction_request = rospy.ServiceProxy('/iiwa/piface', KukaString)

    def _go_to_pose(self, re_po):
        print("planning to go to " + str(re_po[:3]))
        order = (Pose(position=Point(x=re_po[0], y=re_po[1], z=re_po[2]),
                      orientation=Quaternion(x=re_po[3], y=re_po[4], z=re_po[5], w=re_po[6])))
        self._group.set_pose_target(order)
        self._group.go(wait=True)
        self._group.stop()
        self._group.clear_pose_targets()

    def _go_to_joint_goal(self, re_po):
        self._group.set_joint_value_target(re_po)
        self._group.go(wait=True)
        self._group.stop()
        self._group.clear_pose_targets()

    def go_to_home(self):
        self._go_to_pose(data.coord['home_pos'])

    def take_puzzle_picture(self, position_coord='cam_pos', image_path=''):
        self._go_to_pose(data.coord[position_coord])
        rospy.loginfo('taking a snapshot from camera')
        if image_path == '':
            camera_capture.snapshot()
        else:
            camera_capture.snapshot(image_path)

    @staticmethod
    def point_to_physical_coordinates(point):
        return [(point[0] / data.IMAGE_RESOLUTION[0]) * (data.X_RANGE[1] - data.X_RANGE[0]) + data.X_RANGE[0],
                (point[1] / data.IMAGE_RESOLUTION[1]) * -(data.Y_RANGE[1] - data.Y_RANGE[0]) + data.Y_RANGE[1]]

    def _change_suction(self, value):
        # rospy.loginfo('sending suction request with value ' + str(value))
        response = self._suction_request('suction', value)

    # rospy.loginfo("response: %s, %s" % (response.success, response.message))

    def suction_on(self):
        self._change_suction(1)
        self._suction_request('green_leds', 1)

    def suction_off(self):
        self._change_suction(0)
        self._suction_request('green_leds', 0)

    @staticmethod
    def fix_assembled_coord(centroid_assembled_coord, rotation_angle):
        r = data.SUCTION_CUP_OFFSET       # distance between suction cup and center of end effector (gripper)
        x = r * math.cos(rotation_angle)
        y = r * math.sin(rotation_angle)
        return [centroid_assembled_coord[0] + x - r, centroid_assembled_coord[1] - y]

    def assemble_puzzle(self, pieces, assembled_pieces):
        for i in range(len(pieces)):
            # print("the area of piece %d is %d" % (i, cv2.contourArea(np.array(pieces[i]))))
            # print("the area of assembled piece %d is %d" % (i, cv2.contourArea(np.array(assembled_pieces[i]))))

            centroid = utils.get_centroid(pieces[i])
            centroid_assembled = utils.get_centroid(assembled_pieces[i])
            rotation_angle = utils.calc_rotation_angle(pieces[i], assembled_pieces[i])
            centroid_coord = self.point_to_physical_coordinates(centroid)
            centroid_assembled_coord = self.point_to_physical_coordinates(centroid_assembled)
            centroid_assembled_coord = self.fix_assembled_coord(centroid_assembled_coord, rotation_angle)
            centroid_assembled_coord = [centroid_assembled_coord[0] - data.X_SOLVING_AREA_OFFSET, centroid_assembled_coord[1] - data.Y_SOLVING_AREA_OFFSET]

            rospy.loginfo("\nPIECE " + str(
                i) + ":\nmoving: from centroid " + centroid.__str__() + " to " + centroid_assembled.__str__() +
                          "\nrotate: " + str(rotation_angle / (np.pi / 180)) + "\n")

            # Fetching the piece
            goal = centroid_coord + [data.Z_SUCTION + 0.4] + data.SUCTION_ORIENTATION
            self._go_to_pose(goal)
            rospy.sleep(0.5)
            goal[2] = data.Z_SUCTION + 0.1
            self._go_to_pose(goal)
            rospy.sleep(0.5)
            goal[2] = data.Z_SUCTION
            self._go_to_pose(goal)
            rospy.sleep(0.5)
            self.suction_on()
            goal[2] = data.Z_SUCTION + 0.3
            self._go_to_pose(goal)
            rospy.sleep(0.5)

            # Placing the piece
            goal = centroid_assembled_coord + goal[2:]
            self._go_to_pose(goal)
            rospy.sleep(0.5)

            # Rotating the piece
            joints = self._group.get_current_joint_values()
            joints[6] += rotation_angle
            if joints[6] > np.pi:
                joints[6] -= 2 * np.pi
            elif joints[6] < -np.pi:
                joints[6] += 2 * np.pi

            if joints[6] > math.radians(175):
                rospy.loginfo("not accurate")
                joints[6] = math.radians(169.9)
            elif joints[6] < math.radians(-175):
                rospy.loginfo("not accurate")
                joints[6] = math.radians(-169.9)
            rospy.loginfo("joint[6] is %f" % joints[6])
            self._go_to_joint_goal(joints)
            rospy.sleep(0.5)

            goal1 = self._group.get_current_pose().pose
            goal1.position.z = data.Z_SUCTION + 0.07
            self._group.set_pose_target(goal1)
            self._group.go(wait=True)
            self._group.stop()
            self._group.clear_pose_targets()
            rospy.sleep(0.5)

            goal1 = self._group.get_current_pose().pose
            goal1.position.z = data.Z_SUCTION + 0.02
            self._group.set_pose_target(goal1)
            self._group.go(wait=True)
            self._group.stop()
            self._group.clear_pose_targets()
            rospy.sleep(0.5)

            self.suction_off()

            goal1 = self._group.get_current_pose().pose
            goal1.position.z = data.Z_SUCTION + 0.07
            self._group.set_pose_target(goal1)
            self._group.go(wait=True)
            self._group.stop()
            self._group.clear_pose_targets()
            rospy.sleep(0.5)

            goal[2] = data.Z_SUCTION + 0.4
            self._go_to_pose(goal)
            rospy.sleep(0.5)
