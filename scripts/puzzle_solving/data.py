
INPUT_PIECES_PATH = '/home/kuka/PuzzleSolver/results_shared/piecesVertices.csv'
RESULT_PIECES_PATH = '/home/kuka/PuzzleSolver/results_shared/results.csv'


X_SOLVING_AREA_OFFSET = 0.3
Y_SOLVING_AREA_OFFSET = 0.15

X_CAM_POS = 0.058
Y_CAM_POS = 0.53
Z_CAM_POS = 0.72

coord = {'home_pos': [0, 0.5, 0.5, 0.70710678118, -0.70710678118, 0, 0],
         'cam_pos': [X_CAM_POS, Y_CAM_POS, Z_CAM_POS, -0.702209609289, 0.71148764079, 0.0162201139697, 0.0205890632988],
         'final_cam_pos': [X_CAM_POS-X_SOLVING_AREA_OFFSET, Y_CAM_POS-Y_SOLVING_AREA_OFFSET, Z_CAM_POS, -0.702209609289, 0.71148764079, 0.0162201139697, 0.0205890632988],
         }

# Z_SUCTION represents the position on the Z axis (height) of the robot that close enough to the table
# for sucking the puzzle piece (the piece's height is 5mm).
# The value is in the scale of "LBR iiwa" robot cartesian coordinates.
Z_SUCTION = 0.1245

# SUCTION_ORIENTATION represents the orientation of the end effector for facing down (with the suction cup).
# x and y size (sign irrelevant) should be sqrt(0.5). z,w should be zero.
# The value is in the scale of "LBR iiwa" robot xyzw orientation.
SUCTION_ORIENTATION = [-0.70710678118, 0.70710678118, 0, 0]

# arm name is "manipulator" (determined by the manufacturer). change groupname when switching robots.
GROUP_NAME = 'manipulator'

# new image range
X_RANGE = [-0.12, 0.523]
Y_RANGE = [0.33, 0.81]

IMAGE_RESOLUTION = (800, 600)

IMAGE_SCALING_FACTOR = 8

# distance between suction cup and center of end effector (gripper)
SUCTION_CUP_OFFSET = 0.052
