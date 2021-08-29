
INPUT_PIECES_PATH = '/home/kuka/PuzzleSolver/results_shared/piecesVertices.csv'
RESULT_PIECES_PATH = '/home/kuka/PuzzleSolver/results_shared/results.csv'

coord = {'home_pos': [0, 0.5, 0.5, 0.70710678118, -0.70710678118, 0, 0],
         'cam_pos': [0.08, 0.56, 0.725, -0.713126476786, 0.700516669565, 0.0170209801914, 0.0209119583364],
         }

# Z_SUCTION represents the position on the Z axis (height) of the robot that close enough to the table
# for sucking the puzzle piece (the piece's height is 5mm).
# The value is in the scale of "LBR iiwa" robot cartesian coordinates.
Z_SUCTION = 0.125

# SUCTION_ORIENTATION represents the orientation of the end effector for facing down (with the suction cup).
# x and y size (sign irrelevant) should be sqrt(0.5). z,w should be zero.
# The value is in the scale of "LBR iiwa" robot xyzw orientation.
SUCTION_ORIENTATION = [0.70710678118, -0.70710678118, 0, 0]

# arm name is "manipulator" (determined by the manufacturer). change groupname when switching robots.
GROUP_NAME = 'manipulator'

# new image range
X_RANGE = [-0.12, 0.523]
Y_RANGE = [0.33, 0.81]

IMAGE_RESOLUTION = (800, 600)

IMAGE_SCALING_FACTOR = 8

# distance between suction cup and center of end effector (gripper)
SUCTION_CUP_OFFSET = 0.052

X_SOLVING_AREA_OFFSET = 0.4
Y_SOLVING_AREA_OFFSET = 0
