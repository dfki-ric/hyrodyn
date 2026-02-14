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

# Extract the submechanisms information from YAML file describing the submechanisms
def extract_submechanisms_info_yml(path_to_submechanisms):
    """ Extract the submechanisms info from YAML file describing the submechanisms
    ---- Parameters ----
    path_to_submechanisms : path to the submechanisms file describing the submechanisms
    ---- Returns ---- 
    output : returns four lists describing the joint names in independent, spanning tree, active joint spaces and names of all submechanisms """

    jointnames_independent = []
    jointnames_active = []
    jointnames_spanningtree = []
    names_submechanisms = []

    # extract the submechanisms from the yaml
    with open(path_to_submechanisms) as submechanisms_yml:
        # Load the yaml file
        submechanisms = yaml.load(submechanisms_yml, Loader=yaml.FullLoader)
        subs = submechanisms["submechanisms"]
        try:
                exts = submechanisms["exoskeletons"]
        except:
                print("This robot has no exoskeletons defined.")
        for sub in subs:
                jointnames_independent.append(sub["jointnames_independent"]) 
                jointnames_spanningtree.append(sub["jointnames_spanningtree"]) 
                jointnames_active.append(sub["jointnames_active"]) 
                names_submechanisms.append(sub["contextual_name"])

    return jointnames_independent, jointnames_spanningtree, jointnames_active, names_submechanisms

def get_ND_slices(jointnames, n):
	""" N-dimensional Slices of Joint Space
	---- Parameters ---- 
	jointnames : list 
	Names of the joints
	n : int
	Number of dimensions 
	---- Returns ---- 
	output : returns ND slices of joint space """
	return list(itertools.combinations(jointnames,n))

def plot_2D_slice(jointspace_data, jointname_x, jointname_y, jointspace_type = "spanningtree", show_conditioning = False, show_conditioning_in_independent_jointspace = False, conditioning_type = "relative", interactive_mode = False):
	""" Create plots of 2 dimensional slices of configuration space
	---- Parameters ---- 
	jointspace_data : pandas dataframe
	pandas dataframe of file describing the joint space of the robot
	jointname_x, jointname_y : strings
	names of the plotted joints on x and y axes respectively
	show_conditioning : boolean value
	show the conditioning of the slices and map the color of the points according to the conditioning, by default this is set to False
	show_conditioning_in_independent_jointspace : boolean value
	show the conditioning in independent jointspace, by default this is set to False
	conditioning_type : argument
	create a conditioning between 0 and 1, by default this is set to relative
	interactive_mode : boolean value
	if this is true the plots are shown while running the program, by default this is set to False
	---- Returns ---- 
	output : returns 2D plots of each spanningtree joint in configuration space """
	plt.figure
	if jointspace_type == "spanningtree":
		joint_on_xaxis = "q_" + jointname_x
		joint_on_yaxis = "q_" + jointname_y
	elif jointspace_type == "independent":
		joint_on_xaxis = "y_" + jointname_x
		joint_on_yaxis = "y_" + jointname_y
	elif jointspace_type == "active":
		joint_on_xaxis = "u_" + jointname_x
		joint_on_yaxis = "u_" + jointname_y
	else:
		raise NameError('Provided jointspace_type not recognized. Possible options are: spanningtree, independent, active.')
	print("Generating the 2D plot: ", joint_on_xaxis + ' vs ' + joint_on_yaxis)
	jointdata_on_xaxis = jointspace_data[joint_on_xaxis]
	jointdata_on_yaxis = jointspace_data[joint_on_yaxis]
	if show_conditioning == True:
		if show_conditioning_in_independent_jointspace == True:
			cond = jointspace_data["inv_cond_loop_closure_independent_jointspace"]
		else:
			cond = jointspace_data["inv_cond_loop_closure_active_jointspace"]
		if conditioning_type == "relative":
			# Switch to relative conditioning i.e. min = 0 and max = 1
			cond = (cond - min(cond)) / (max(cond) - min(cond))	
		# If colors are not specified the function will interpolate between 0.0 values as red and 1.0 as green.
		point_color = trimesh.visual.color.linear_color_map (cond)	
		# Convert values to the range [0,1] as accepted by the scatter plot function
		point_color = point_color / 255		
		plt.scatter(jointdata_on_xaxis, jointdata_on_yaxis, color = point_color)
	else:
		plt.scatter(jointdata_on_xaxis, jointdata_on_yaxis)
	plt.xlabel(joint_on_xaxis)
	plt.ylabel(joint_on_yaxis)
	plt.title(joint_on_xaxis + ' vs ' + joint_on_yaxis)
	plt.savefig(joint_on_xaxis + '_vs_' + joint_on_yaxis)  
	if interactive_mode == True:
		plt.show()
	plt.close()

def plot_3D_slice(jointspace_data, jointname_x, jointname_y, jointname_z, jointspace_type = "spanningtree", show_conditioning = False, show_conditioning_in_independent_jointspace = False, conditioning_type = "relative", interactive_mode = False):
	""" Create plots of 3 dimensional slices of configuration space
	---- Parameters ---- 
	jointspace_data : pandas dataframe
	pandas dataframe of file describing the joint space of the robot
	jointname_x, jointname_y, jointname_z : strings
	names of the plotted joints
	show_conditioning : boolean value
	show the conditioning of the slices and map the color of the points according to the conditioning, by default this is set to False
	show_conditioning_in_independent_jointspace : boolean value
	show the conditioning in independent jointspace, by default this is set to False
	conditioning_type : argument
	create a conditioning between 0 and 1, by default this is set to relative
	interactive_mode : boolean value
	if this is true the plots are shown while running the program, by default this is set to False
	---- Returns ---- 
	output : returns 3D plots of each spanningtree joint in configuration space """
	fig = plt.figure()
	if jointspace_type == "spanningtree":
		joint_on_xaxis = "q_" + jointname_x
		joint_on_yaxis = "q_" + jointname_y
		joint_on_zaxis = "q_" + jointname_z
	elif jointspace_type == "independent":
		joint_on_xaxis = "y_" + jointname_x
		joint_on_yaxis = "y_" + jointname_y
		joint_on_zaxis = "y_" + jointname_z
	elif jointspace_type == "active":
		joint_on_xaxis = "u_" + jointname_x
		joint_on_yaxis = "u_" + jointname_y
		joint_on_zaxis = "u_" + jointname_z
	else:
		raise NameError('Provided jointspace_type not recognized. Possible options are: spanningtree, independent, active.')
	print("Generating the 3D plot: ", joint_on_xaxis + ' vs ' + joint_on_yaxis + ' vs ' + joint_on_zaxis)
	jointdata_on_xaxis = jointspace_data[joint_on_xaxis]
	jointdata_on_yaxis = jointspace_data[joint_on_yaxis]
	jointdata_on_zaxis = jointspace_data[joint_on_zaxis]
	ax = fig.add_subplot(111, projection='3d')
	if show_conditioning == True:
		if show_conditioning_in_independent_jointspace == True:
			cond = jointspace_data["inv_cond_loop_closure_independent_jointspace"]
		else:
			cond = jointspace_data["inv_cond_loop_closure_active_jointspace"]
		if conditioning_type == "relative":
			# Switch to relative conditioning i.e. min = 0 and max = 1
			cond = (cond - min(cond)) / (max(cond) - min(cond))	
		# If colors are not specified the function will interpolate between 0.0 values as red and 1.0 as green.
		point_color = trimesh.visual.color.linear_color_map (cond)	
		# Convert values to the range [0,1] as accepted by the scatter plot function
		point_color = point_color / 255		
		ax.scatter(jointdata_on_xaxis, jointdata_on_yaxis, jointdata_on_zaxis, color = point_color)
	else:
		ax.scatter(jointdata_on_xaxis, jointdata_on_yaxis, jointdata_on_zaxis)
	ax.set_xlabel(joint_on_xaxis)
	ax.set_ylabel(joint_on_yaxis)
	ax.set_zlabel(joint_on_zaxis)
	plt.title(joint_on_xaxis + ' vs ' + joint_on_yaxis + ' vs ' + joint_on_zaxis)
	plt.savefig(joint_on_xaxis + '_vs_' + joint_on_yaxis + '_vs_' + joint_on_zaxis)  
	if interactive_mode == True:
		plt.show()
	plt.close()

	
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
	os.system("./hyrodyn_analyze_robot" + " --filepath_urdf " + path_to_urdf  + " --filepath_submechanisms " + path_to_submechamisms + " --filepath_jointlimits " + path_to_jointlimits + " --num_steps " + str(num_discretization_steps_per_joint) + " --option " + str(1))
	# Switch back to python folder so that the results are stored here
	os.chdir("../../../../hyrodynpy/scripts")

	abs_path_to_urdf = os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_urdf
	abs_path_to_submechanisms = os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_submechamisms

	# Load the robot in urdfpy
	viz = URDF.load(abs_path_to_urdf)

	# Load the position and orientation coordinates of the end-effector
	urdf_file_name_with_ext = os.path.basename(abs_path_to_urdf)
	urdf_file_name = os.path.splitext(urdf_file_name_with_ext)[0]

	# input file where the cspace is stored
	input_file = os.environ ["AUTOPROJ_CURRENT_ROOT"] + "/control/hyrodyn/applications/robot_analysis/results/" + urdf_file_name + "_sysstate_q.csv"
	# read the csv file
	cspace_data = pd.read_csv(input_file)
	cspace_keys = list(cspace_data)
	print(cspace_keys)

	# Extract the minimum and maximum positions
	min_pos_q = cspace_data.min()
	max_pos_q = cspace_data.max()
	print("Minimum position of each joint in spanning tree (units: rad for revolute joints, m for prismatic joints):\n", min_pos_q)
	print("Maximum position of each joint in spanning tree (units: rad for revolute joints, m for prismatic joints):\n", max_pos_q)

	jointnames_independent, jointnames_spanningtree, jointnames_active, names_submechanisms = extract_submechanisms_info_yml(abs_path_to_submechanisms)
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
	finaldirectory = "cspace"
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
	for i in range(len(jointnames_spanningtree)):	
		finaldirectory = names_submechanisms[i]
		temppath = os.path.join(twodslices_path, finaldirectory)
		if not os.path.exists(temppath):
			os.mkdir(temppath)
		os.chdir(temppath)
		print("In ", temppath)
		twoDslices_cspace = get_ND_slices(jointnames_spanningtree[i], 2)
		for twoDslice in twoDslices_cspace:
			plot_2D_slice(cspace_data, twoDslice[0], twoDslice[1], "spanningtree", interactive_mode = False)
		os.chdir("../")
	os.chdir("../../../")

	# Plot all the 3D slices and store it into a new folder
	finaldirectory = "3d_slices"
	finalpath = os.path.join(finalpath, finaldirectory)
	if not os.path.exists(finalpath):
		os.mkdir(finalpath)
	os.chdir(finalpath)
	print("In ", finalpath)
	
	for i in range(len(jointnames_spanningtree)):	
		finaldirectory = names_submechanisms[i]
		temppath = os.path.join(finalpath, finaldirectory)
		if not os.path.exists(temppath):
			os.mkdir(temppath)
		os.chdir(temppath)
		print("In ", temppath)
		threeDslices_cspace = get_ND_slices(jointnames_spanningtree[i], 3)
		if threeDslices_cspace is not []:
			for threeDslice in threeDslices_cspace:
				plot_3D_slice(cspace_data, threeDslice[0], threeDslice[1], threeDslice[2], "spanningtree", interactive_mode = False)
		os.chdir("../")

	os.chdir("../../../../")
	
	# Create dictionary of minimum and maximum position in configuration space
	dict_min_max_pos_q = {a: {"min_pos": b, "max_pos": c} for a, b, c in zip(cspace_keys, min_pos_q.tolist(), max_pos_q.tolist())}
	return dict_min_max_pos_q

if __name__ == '__main__':	

	# Create the parser
	parser = argparse.ArgumentParser(description = "Generate a visualization of the configuration space using HyRoDyn where all reachable points are shown as black spheres. Robot model in HyRoDyn is built using the URDF and submechanisms yaml file. Joint limits described in a yaml file (in RoCK convention) will be used to discretize the input space. The program exports the visualization of all possible 2D slices of the configuration space joints wherever there is a certain dependency between cspace variables and groups them via submechanism names in different folders. \n\nBasic Usage Example: \n\npython3 cspace_visualization.py /control/hyrodyn/data/hybrid/rh5/leg/urdf/leg.urdf /control/hyrodyn/data/hybrid/rh5/leg/urdf/submechanisms.yml /control/hyrodyn/data/hybrid/rh5/leg/urdf/joint_limits.yml", formatter_class=RawTextHelpFormatter)

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

