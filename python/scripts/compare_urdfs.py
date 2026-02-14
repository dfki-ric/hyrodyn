import hyrodyn
import time
import numpy as np
from urdfpy import URDF

# Path to working URDF (which have been tested on the real system)
path_to_urdf_working =  "/home/dfki.uni-bremen.de/skumar/repos/githb/rock_rh5/bundles/dual_arm_exo/dual_arm.urdf"
path_to_submechamisms_working = "/home/dfki.uni-bremen.de/skumar/repos/githb/rock_rh5/bundles/dual_arm_exo/submechanisms_dual.yml"

# Path to new URDF (newly exported from SolidWorks-2-URDF Exporter)
path_to_urdf_new =  "/home/dfki.uni-bremen.de/skumar/repos/githb/rock_rh5/bundles/dual_arm_exon/urdf/recupera_exo.urdf"
path_to_submechamisms_new = "/home/dfki.uni-bremen.de/skumar/repos/githb/rock_rh5/bundles/dual_arm_exon/urdf/submechanisms_dual.yml"

robot_working = hyrodyn.RobotModel(path_to_urdf_working, path_to_submechamisms_working)
robot_new = hyrodyn.RobotModel(path_to_urdf_new, path_to_submechamisms_new)

# Compute the COM properties of the robot in zero configuration
robot_working.calculate_com_properties()
print("Working Robot: ")
print("Total mass in zero config = ",robot_working.mass)
print("COM in zero config = ",robot_working.com)

robot_new.calculate_com_properties()
print("New Robot: ")
print("Total mass in zero config = ",robot_new.mass)
print("COM in zero config = ",robot_new.com)

# Excite the robot
robot_working.y = np.full(robot_working.y.size, 0.1)
robot_new.y = np.full(robot_new.y.size, 0.1)
print("Moving the robot to a new configuration! y = ",robot_working.y)

# Compute the COM properties of the robot again
robot_working.calculate_com_properties()
print("COM of working robot in new config = ",robot_working.com)

robot_new.calculate_com_properties()
print("COM of new robot in new config = ",robot_new.com)

# test the visualization
robot = URDF.load(path_to_urdf_working)
robot.animate(cfg_trajectory={'left_arm_shoulder_joint0' : [-np.pi / 4, np.pi / 4]})

