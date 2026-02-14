import hyrodyn
import time
import numpy as np
from urdfpy import URDF

# Path to working URDF (which have been tested on the real system)
path_to_urdf =  "/home/dfki.uni-bremen.de/skumar/repos/githb/rock_rh5/control/hyrodyn/data/rh5_CAD2URDF/submodel/lower_body/urdf/leg_darts.urdf"
path_to_submechamisms = "/home/dfki.uni-bremen.de/skumar/repos/githb/rock_rh5/control/hyrodyn/data/rh5_CAD2URDF/submodel/lower_body/urdf/submechanisms_leg.yml"

robot = hyrodyn.RobotModel(path_to_urdf, path_to_submechamisms)

# Compute the COM properties of the robot in zero configuration
robot.calculate_com_properties()
print("Working Robot: ")
print("Total mass in zero config = ",robot.mass)
print("COM in zero config = ",robot.com)

computation_time = 0.0
for i in range(247,1196):
	tic = time.clock()   
	robot.calculate_inverse_dynamics()
	toc = time.clock()
	computation_time += (toc-tic)
print("Computation Time for IDyn = ",computation_time/(1196-247))

# test the visualization
robot_viz = URDF.load(path_to_urdf)
robot_viz.animate(cfg_trajectory={'LLHip1' : [-np.pi / 4, np.pi / 4]})

