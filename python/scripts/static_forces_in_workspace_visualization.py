import pyrender
import numpy as np
import trimesh
import os
import sys
from urdfpy import URDF 
import random
import pandas as pd
import matplotlib.pyplot as plt
import argparse
from argparse import RawTextHelpFormatter
import workspace_visualization as wv #import modules from the worspace_visualization script by using wv
from swing_my_robot import mesh_orientation

def extract_actuatorforces_from_csv (input_file_forces, number_of_points_in_workspace = 100):
	"""Extract the forces of the actuators from csv file.
	----- Parameters -----
	input_file_forces : string
	Path to the csv file
	number_of_points_in_workspace : integer
	Number of the points in the workspace visualization;
	By default it is set to 100
	Returns
	data : data frame
	frame with all forces of each actuator
	actuator_forces, actuator_names : lists
	lists of the actuator names and of the associated force values
	"""
	data = pd.read_csv(input_file_forces, nrows = number_of_points_in_workspace)
	actuator_names = list(data)
	return data, actuator_names

def min_max_actuatorforces (data):
	"""Generate the minimum and maximum forces of each actuator
	----- Parameters -----
	data : data frame
	frame with all forces of each actuator
	Returns
	min_forces, max_forces : output of specific values of the data frame,
	one with the minimum forces of each actuator
	and the other one with the maximum forces.
	"""
	min_forces = data.min()
	max_forces = data.max()
	return min_forces, max_forces

def Norm_StaticTorque(data):
	"""Norm the values of each row of the csv file.
	----- Parameters -----
	data : data frame
	frame with all forces of each actuator
	Returns
	list_norm_torque : list
	A list of the normed values.
	"""
	list_norm_torque = []
	for i in range (len(data)):
		row_array = data.iloc[i].values
		list_norm_static_torque = list_norm_torque.append(np.linalg.norm (row_array))
	return list_norm_torque

def main (path_to_urdf, path_to_submechamisms, path_to_jointlimits, number_of_points_in_workspace=100, num_discretization_steps_per_joint = 5, fix_mesh_orientation=False):
	"""Return the print-outs as a dictionary
	----- Parameters -----
	path_to_urdf, path_to_submechamisms, path_to_jointlimits : strings
	pathes to the urdf, submechanism and jointlimits
	body_name : string
	name of the body part which will be visualized
	number_of_points_in_workspace : integer, optional argument
	number of points shown in the workspace, by default it is set to 100
	Returns
	dict_min_max_forces : dictionary
	dictionary of the min and max forces according to the actuators
	"""
	# Path to URDF, submechanism and joint_limits files
	# path_to_urdf = urdf_path 
	# path_to_urdf = "/control/hyrodyn/data/hybrid/recupera/right_arm_teleop/urdf/right_arm_teleop.urdf" 
	# path_to_submechamisms = submechanism_yml
	# path_to_jointlimits = joint_limits_yml
	# body_name = body_name

	# Switch to build folder to run the C++ program
	os.chdir("../../hyrodyn/applications/robot_analysis/build")
	# options available in hyrodyn_analyze_robot C++ program: [1: CSPACE, 2: ACTSPACE, 3: STATICS, 4: WORKSPACE, 5: INDSPACE, 6: ALL, Any other integer: All joint space analyses (no workspace)]
	os.system("./hyrodyn_analyze_robot" + " --filepath_urdf " + path_to_urdf  + " --filepath_submechanisms " + path_to_submechamisms + " --filepath_jointlimits " + path_to_jointlimits + " --num_steps " + str(num_discretization_steps_per_joint) + " --option " + str(3))
	# Switch back to python folder so that the results are stored here
	os.chdir("../../../../hyrodynpy/scripts")

	abs_path_to_urdf = os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_urdf
	# Load the robot in urdfpy
	viz = URDF.load(abs_path_to_urdf)

	# Add fix mesh orientation
	if fix_mesh_orientation:
        	mesh_orientation(viz)

	# Create the pyrender scene
	scene = wv.create_scene_with_robot(viz)

	# Load the position and orientation coordinates of the end-effector
	urdf_file_name_with_ext = os.path.basename(abs_path_to_urdf)
	urdf_file_name = os.path.splitext(urdf_file_name_with_ext)[0]

	# input_file = os.environ ["AUTOPROJ_CURRENT_ROOT"] + "/control/hyrodyn/applications/robot_analysis/results/right_arm_teleop_forwardkinematics_x.csv"
	input_file_poses = os.environ ["AUTOPROJ_CURRENT_ROOT"] + "/control/hyrodyn/applications/robot_analysis/results/" + urdf_file_name + "_forwardkinematics_x.csv"
	input_file_forces = os.environ ["AUTOPROJ_CURRENT_ROOT"] + "/control/hyrodyn/applications/robot_analysis/results/" + urdf_file_name + "_actuatortorques_Tau.csv"

	# Extract the positions and orientations of the spheres
	x, y, z, qw, qx, qy, qz, cond1, cond2 = wv.extract_poses_from_csv (input_file_poses, number_of_points_in_workspace)
	poses = wv.extract_SE3_poses (x, y, z, qw, qx, qy, qz)
	print("Number of points used in workspace visualization: ", number_of_points_in_workspace)
	data, actuator_names = extract_actuatorforces_from_csv (input_file_forces, number_of_points_in_workspace)

	# Normalize the forces of the actuated joints to [0,1]
	list_norm_torque = Norm_StaticTorque(data)
	list_normalized_static_torque = wv.NormalizeData(list_norm_torque)

	# Extract the minimum and maximum forces
	min_forces, max_forces = min_max_actuatorforces (data)
	print("Names of active joints: ", actuator_names)
	print("Minimum forces of each actuated joint:\n", min_forces)
	print("Maximum forces of each actuated joint:\n", max_forces)

	# scene = add_sphere_mesh_in_scene (scene, poses)	# use this when you do not want colored spheres (much faster because all of the point cloud is a single mesh)
	scene = wv.add_sphere_mesh_in_scene (scene, poses, list_normalized_static_torque) # workspace with conditioning (slow as each point is a separate mesh in the point cloud)

	# Prepare camera views for orthographic camera
	print("scene bounds", scene.bounds)
	scene_bounds = np.abs(np.diff(scene.bounds, axis = 0)).flatten()
	print("scene bounds diff", scene_bounds)
	scene_bounds_x = scene_bounds[0]
	scene_bounds_y = scene_bounds[1]
	scene_bounds_z = scene_bounds[2]

	# camera view dictionary for orthographic camera
	static_torque_camera_dict_orthographic = wv.return_camera_poses(scene_bounds_x, scene_bounds_y, scene_bounds_z)

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
	finaldirectory = "statics"
	finalpath = os.path.join(subpath, finaldirectory)
	if not os.path.exists(finalpath):
		os.mkdir(finalpath)
	os.chdir(finalpath)

	wv.save_workspace_image(scene, static_torque_camera_dict_orthographic)

	# Prepare camera views for perspective camera
	s = np.sqrt(2)/2
	camera_pose = np.array([
	       [0.0, -s,   s,   1.0],
	       [1.0,  0.0, 0.0, 0.0],
	       [0.0,  s,   s,   1.0],
	       [0.0,  0.0, 0.0, 1.0],
	    ])
	static_torque_camera_dict_perspective = {"perspective": camera_pose}
	
	wv.save_workspace_image (scene, static_torque_camera_dict_perspective, "perspective")
	os.chdir("../../../")
	
	# Create dictionary
	dict_min_max_forces = {a: {"min_forces": b, "max_forces": c} for a, b, c in zip(actuator_names, min_forces.tolist(), max_forces.tolist())}
	return dict_min_max_forces

if __name__ == '__main__':	

	# Create the parser
	parser = argparse.ArgumentParser(description = "Generate a visualization of the static forces / torques for each point in the workspace using HyRoDyn where all reachable points are shown as colorful spheres. The color of the spheres represent the amount of forces the robot experienced in this configuration (green: low amount, red: high amount). Robot model in HyRoDyn is built using the URDF and submechanisms yaml file. By default, the program extracts the coordinates of the centre of the spheres from csv file. One can additionally provide joint limits described in a yaml file (in RoCK convention) which will be used if provided. The program also exports the visualization of the workspace into seven png files by default which can be turned off on demand. The seven png files show the robot and it's zero configuration from left, right, bottom, top, back and front view and additionally in a perspective view. \n\nBasic Usage Example: \n\npython3 static_forces_in_workspace_visualization.py /control/hyrodyn/data/hybrid/rh5/leg/urdf/leg.urdf /control/hyrodyn/data/hybrid/rh5/leg/urdf/submechanisms.yml /control/hyrodyn/data/hybrid/rh5/leg/urdf/joint_limits.yml LLAnklePitch_Link", formatter_class=RawTextHelpFormatter)

	# Add the arguments
	parser.add_argument("urdf_path", type = str,
	                    help="URDF path for the robot with respect to AUTOPROJ_CURRENT_ROOT")
	parser.add_argument("submechanism_yml", type = str, 
	                    help="YAML file describing the submechanisms with respect to AUTOPROJ_CURRENT_ROOT")
	parser.add_argument("joint_limits_yml", type = str,
	                    help="YAML file of describing the joint limits with respect to AUTOPROJ_CURRENT_ROOT")
	parser.add_argument("--num_discretization_steps_per_joint", type = int, default = 5,
	                    help="Number of discretization steps per joint (defaults to 5 which means for a 6 joint robot 5^6 = 15625 discretized points in the input joint space)")
	parser.add_argument("--number_of_points_in_workspace", type = int, default = 100, 
			    help="Limit the number of points in the workspace visualization (defaults to 100)")
	parser.add_argument("--fix_mesh_orientation", action='store_true', help="This can fix some orientation issues with some OBJ meshes")

	# Execute the parse_args method
	args, unknown = parser.parse_known_args()

	# Store the variables in the args
	path_to_urdf = args.urdf_path
	path_to_submechamisms = args.submechanism_yml
	path_to_jointlimits = args.joint_limits_yml
	num_discretization_steps_per_joint = args.num_discretization_steps_per_joint
	number_of_points_in_workspace = args.number_of_points_in_workspace
	fix_mesh_orientation = args.fix_mesh_orientation

	# Call the main function
	main (path_to_urdf, path_to_submechamisms, path_to_jointlimits, number_of_points_in_workspace, num_discretization_steps_per_joint, fix_mesh_orientation=fix_mesh_orientation)

