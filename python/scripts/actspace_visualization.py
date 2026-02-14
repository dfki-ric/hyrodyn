import numpy as np
import os
import sys
import random
import pandas as pd
import matplotlib.pyplot as plt
import argparse
from argparse import RawTextHelpFormatter
import yaml
import itertools
from urdfpy import URDF 
from mpl_toolkits.mplot3d import Axes3D
import trimesh
import cspace_visualization as cspaceviz
	
def main (path_to_urdf, path_to_submechamisms, path_to_jointlimits, num_discretization_steps_per_joint = 5):
	"""Return the print-outs as a dictionary
	----- Parameters -----
	path_to_urdf, path_to_submechamisms, path_to_jointlimits : strings
	paths to the urdf, submechanism and jointlimits
	num_discretization_steps_per_joint : integer, optional argument
	Number of discretization steps per joint (defaults to 5 which means for a 6 joint robot 5^6 = 15625 discretized points in the input joint space)
	Returns
	dict_min_max_forces : dictionary
	dictionary of the min and max forces according to the actuators
	"""

	# Switch to build folder to run the C++ program
	os.chdir("../../hyrodyn/applications/robot_analysis/build")
	# options available in hyrodyn_analyze_robot C++ program: [1: CSPACE, 2: ACTSPACE, 3: STATICS, 4: WORKSPACE, 5: INDSPACE, 6: ALL, Any other integer: All joint space analyses (no workspace)]
	os.system("./hyrodyn_analyze_robot" + " --filepath_urdf " + path_to_urdf  + " --filepath_submechanisms " + path_to_submechamisms + " --filepath_jointlimits " + path_to_jointlimits + " --num_steps " + str(num_discretization_steps_per_joint) + " --option " + str(2))
	# Switch back to python folder so that the results are stored here
	os.chdir("../../../../hyrodynpy/scripts")

	abs_path_to_urdf = os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_urdf
	abs_path_to_submechanisms = os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_submechamisms

	# Load the robot in urdfpy
	viz = URDF.load(abs_path_to_urdf)

	# Load the position and orientation coordinates of the end-effector
	urdf_file_name_with_ext = os.path.basename(abs_path_to_urdf)
	urdf_file_name = os.path.splitext(urdf_file_name_with_ext)[0]

	# input file where the actspace is stored
	input_file = os.environ ["AUTOPROJ_CURRENT_ROOT"] + "/control/hyrodyn/applications/robot_analysis/results/" + urdf_file_name + "_actuatorstate_u.csv"
	# read the csv file
	actspace_data = pd.read_csv(input_file)
	actspace_keys = list(actspace_data)
	print(actspace_keys)

	# Extract the minimum and maximum positions
	min_pos_u = actspace_data.min()
	max_pos_u = actspace_data.max()
	print("Minimum position of each joint in spanning tree (units: rad for revolute joints, m for prismatic joints):\n", min_pos_u)
	print("Maximum position of each joint in spanning tree (units: rad for revolute joints, m for prismatic joints):\n", max_pos_u)

	jointnames_independent, jointnames_spanningtree, jointnames_active, names_submechanisms = cspaceviz.extract_submechanisms_info_yml(abs_path_to_submechanisms)
	print("Active joints grouped via submechanisms", jointnames_active)
	print("Spanning tree joints grouped via submechanisms", jointnames_spanningtree)
	print("Independent joints grouped via submechanisms", jointnames_independent)

	# change directory and create a new folder to store the images
	parent_dir = os.environ["AUTOPROJ_CURRENT_ROOT"] + "/control/hyrodynpy/scripts/"
	directory = "results"
	path = os.path.join(parent_dir, directory)
	if not os.path.exists(path):
		os.mkdir(path)
	os.chdir(path)
	subdirectory = str(viz.name)
	subpath = os.path.join(path, subdirectory)
	if not os.path.exists(subpath):
		os.mkdir(subpath)
	os.chdir(subpath)
	finaldirectory = "actspace"
	finalpath = os.path.join(subpath, finaldirectory)
	if not os.path.exists(finalpath):
		os.mkdir(finalpath)
	os.chdir(finalpath)

	# Plot all the 2D slices and store it into a new folder
	finaldirectory = "2d_slices"
	twodslices_path = os.path.join(finalpath, finaldirectory)
	if not os.path.exists(twodslices_path):
		os.mkdir(twodslices_path)
	os.chdir(twodslices_path)
	print("In ", twodslices_path)
	for i in range(len(jointnames_active)):	
		finaldirectory = names_submechanisms[i]
		temppath = os.path.join(twodslices_path, finaldirectory)
		if not os.path.exists(temppath):
			os.mkdir(temppath)
		os.chdir(temppath)
		print("In ", temppath)
		twoDslices_actspace = cspaceviz.get_ND_slices(jointnames_active[i], 2)
		for twoDslice in twoDslices_actspace:
			cspaceviz.plot_2D_slice(actspace_data, twoDslice[0], twoDslice[1], "active", interactive_mode = False)
		os.chdir("../")
	os.chdir("../../../")

	# Plot all the 3D slices and store it into a new folder
	finaldirectory = "3d_slices"
	finalpath = os.path.join(finalpath, finaldirectory)
	if not os.path.exists(finalpath):
		os.mkdir(finalpath)
	os.chdir(finalpath)
	print("In ", finalpath)
	
	for i in range(len(jointnames_active)):	
		finaldirectory = names_submechanisms[i]
		temppath = os.path.join(finalpath, finaldirectory)
		if not os.path.exists(temppath):
			os.mkdir(temppath)
		os.chdir(temppath)
		print("In ", temppath)
		threeDslices_actspace = cspaceviz.get_ND_slices(jointnames_active[i], 3)
		if threeDslices_actspace is not []:
			for threeDslice in threeDslices_actspace:
				cspaceviz.plot_3D_slice(actspace_data, threeDslice[0], threeDslice[1], threeDslice[2], "active", interactive_mode = False)
		os.chdir("../")

	os.chdir("../../../../")
	
	# Create dictionary of minimum and maximum position in actuation space
	dict_min_max_pos_u = {a: {"min_pos": b, "max_pos": c} for a, b, c in zip(actspace_keys, min_pos_u.tolist(), max_pos_u.tolist())}
	return dict_min_max_pos_u

if __name__ == '__main__':	

	# Create the parser
	parser = argparse.ArgumentParser(description = "Generate a visualization of the actuation space using HyRoDyn where all reachable points are shown as black spheres. Robot model in HyRoDyn is built using the URDF and submechanisms yaml file. Joint limits described in a yaml file (in RoCK convention) will be used to discretize the input space. The program exports the visualization of all possible 2D slices of the actuation space joints wherever there is a certain dependency between actspace variables and groups them via submechanism names in different folders. \n\nBasic Usage Example: \n\npython3 actspace_visualization.py /control/hyrodyn/data/hybrid/rh5/leg/urdf/leg.urdf /control/hyrodyn/data/hybrid/rh5/leg/urdf/submechanisms.yml /control/hyrodyn/data/hybrid/rh5/leg/urdf/joint_limits.yml", formatter_class=RawTextHelpFormatter)

	# Add the arguments
	parser.add_argument("urdf_path", type = str,
	                    help="URDF path for the robot with respect to AUTOPROJ_CURRENT_ROOT")
	parser.add_argument("submechanism_yml", type = str, 
	                    help="YAML file describing the submechanisms with respect to AUTOPROJ_CURRENT_ROOT")
	parser.add_argument("joint_limits_yml", type = str,
	                    help="YAML file of describing the joint limits with respect to AUTOPROJ_CURRENT_ROOT")
	parser.add_argument("--num_discretization_steps_per_joint", type = int, default = 5,
	                    help="Number of discretization steps per joint (defaults to 5 which means for a 6 joint robot 5^6 = 15625 discretized points in the input joint space)")

	# Execute the parse_args method
	args, unknown = parser.parse_known_args()

	# Store the variables in the args
	path_to_urdf = args.urdf_path
	path_to_submechamisms = args.submechanism_yml
	path_to_jointlimits = args.joint_limits_yml
	num_discretization_steps_per_joint = args.num_discretization_steps_per_joint

	# Call the main function
	main (path_to_urdf, path_to_submechamisms, path_to_jointlimits, num_discretization_steps_per_joint)

