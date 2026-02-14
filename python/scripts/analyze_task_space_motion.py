import hyrodyn 
import time 
import numpy as np 
import os 
import shutil
import yaml
import matplotlib.pyplot as plt
import pandas as pd
import urdfpy
from urdfpy import URDF 
import swing_my_robot as swingrob
import argparse
from argparse import RawTextHelpFormatter
from swing_my_robot import mesh_orientation

def define_pos_time_array(csv_file, lst_column_names):
	N = int(len(csv_file))
	dt = round((csv_file.index.values[-1])-(csv_file.index.values[-2]),2)
	Tf = N*dt
	time_vec = np.linspace(0, Tf, N)
	pos_x = csv_file[[col for col in csv_file if col.endswith("_x")]].values
	pos_y = csv_file[[col for col in csv_file if col.endswith("_y")]].values
	pos_z = csv_file[[col for col in csv_file if col.endswith("_z")]].values
	return time_vec, pos_x, pos_y, pos_z

# Function for create a single joint trajectory in each direction
def single_coord_traj (pos_zeroconfig, pos_input_min, pos_input_max, Tf, dt):
	""" Generate a sinusoidal trajectory in one direction (x, y or z) so that the joint moves from zero to a given minimum position,cback to zero position and then to a maximum joint position defined by user input in a total of Tf seconds in dt time steps.
        ---- Parameters ----
	pos_zeroconfig : float
	zero configuration position
	pos_input_min, pos_input_max : floats
	minimum and maximum position, defined by user input
	Tf : float
        The total time for the motion given in seconds.
        dt : float
        The timesteps. On each timestep the simulation will be integrated.
	Returns
	time, pos, vel, acc : lists
	lists of the time of motion, the position, velocity and acceleration of the joint during motion
	"""
	N = int(Tf / dt)
	if pos_input_min == 0.0:
		time = np.linspace(0, Tf, N)
		pos = pos_input_max * np.sin(np.pi * time / 2) + pos_zeroconfig
		pos = np.vstack(np.array(pos))
		vel = pos_input_max * (np.pi/2) * np.cos(np.pi * time / 2)
		vel = np.vstack(np.array(vel))
		acc = pos_input_max * (np.pi**2/4) * -np.sin(np.pi * time / 2)
		acc = np.vstack(np.array(acc))
	elif pos_input_max == 0.0:
		time = np.linspace(0, Tf, N)
		pos = pos_input_min * np.sin(np.pi * time / 2) + pos_zeroconfig
		pos = np.vstack(np.array(pos))
		vel = pos_input_min * (np.pi/2) * np.cos(np.pi * time / 2)
		vel = np.vstack(np.array(vel))
		acc = pos_input_min * (np.pi**2/4) * -np.sin(np.pi * time / 2)
		acc = np.vstack(np.array(acc))
	else:
		time = np.linspace(0, Tf/2, int(N/2))
		pos = np.append(pos_input_min * np.sin(np.pi * time / 2), pos_input_max * np.sin(np.pi * time / 2)) + pos_zeroconfig
		pos = np.vstack(np.array(pos))
		vel = np.append(pos_input_min * (np.pi/2) * np.cos(np.pi * time / 2), pos_input_max * (np.pi/2) * np.cos(np.pi * time / 2))
		vel = np.vstack(np.array(vel))
		acc = np.append(pos_input_min * (np.pi**2/4) * -np.sin(np.pi * time / 2), pos_input_max * (np.pi**2/4) * -np.sin(np.pi * time / 2))
		acc = np.vstack(np.array(acc))
	return time, pos, vel, acc

def generate_csv_file (name_space, time_series, body_names, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z):
	"""Generate csv files for the end-effector, independent, active and spanning-tree joints
	----- Parameters -----
	name_space : string
	the name of the used joint space
	time_series : array
	array of the animation time
	body_names : string 
	names of the joints
	pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_ : lists
	lists of the current position, velocity and acceleration in each direction of the robot's joint during motion
	force_x, force_y, force_z : lists
	lists of the actuator forces/torques, by default the list is empty
	Returns
	csv_file : csv file
	csv file with the position, velocity and acceleration in x, y and z direction of the end-effector, independent, active and spanning-tree joints
	"""
	index = time_series
	filename = name_space + "_task_space_motion.csv"
	columns_pos_x = ["pos_" + (name + "_x") for name in body_names]
	columns_pos_y = ["pos_" + (name + "_y") for name in body_names]
	columns_pos_z = ["pos_" + (name + "_z") for name in body_names]
	columns_vel_x = ["vel_" + (name + "_x") for name in body_names]
	columns_vel_y = ["vel_" + (name + "_y") for name in body_names]
	columns_vel_z = ["vel_" + (name + "_z") for name in body_names]
	columns_acc_x = ["acc_" + (name + "_x") for name in body_names]
	columns_acc_y = ["acc_" + (name + "_y") for name in body_names]
	columns_acc_z = ["acc_" + (name + "_z") for name in body_names]
	columns = columns_pos_x + columns_pos_y + columns_pos_z + columns_vel_x + columns_vel_y + columns_vel_z + columns_acc_x + columns_acc_y + columns_acc_z
	arr_pos_x = np.array(pos_x)	
	arr_pos_y = np.array(pos_y)
	arr_pos_z = np.array(pos_z)
	arr_vel_x = np.array(vel_x)
	arr_vel_y = np.array(vel_y)
	arr_vel_z = np.array(vel_z)
	arr_acc_x = np.array(acc_x)
	arr_acc_y = np.array(acc_y)
	arr_acc_z = np.array(acc_z)
	data = np.concatenate((arr_pos_x, arr_pos_y, arr_pos_z, arr_vel_x, arr_vel_y, arr_vel_z, arr_acc_x, arr_acc_y, arr_acc_z), axis = 1)
	df = pd.DataFrame(data, index, columns)
	df.index.name = "Time (s)"
	csv_file = df.to_csv(filename, sep=',',index=True)
	return csv_file

def plot_traj_EE (user_input, name_csv_file, name_filter, label_y, title, user_input_csv = "manual", csv_file_path = "",):
	"""Plot the trajectory of the robot's end-effector
	----- Parameters -----
	user_input : list of floats
	list of strings of the user input which defines the minimum and maximum position of the EE in specific directions
	name_csv_file : string
	name of the csv file which has to be loaded
	name_filter : string
	part of the column names which have to be filtered
	label_y : string
	label of the y-axis
	title : string
	title of the created plot
	Returns
	Plots for position, velocity and acceleration of the EE
	"""
	# Change to path of the provided csv file if neccessary to read the csv
	if user_input_csv == "csv" and csv_file_path != "":
		os.chdir(os.environ["AUTOPROJ_CURRENT_ROOT"] + csv_file_path)
	df = pd.read_csv(name_csv_file)
	if user_input_csv == "csv" and csv_file_path != "":
		os.chdir(os.environ["AUTOPROJ_CURRENT_ROOT"] + "/control/hyrodynpy/scripts/")
	filtered_df = df[[col for col in df if name_filter in col]]
	time = filtered_df.index
	name_col = list(filtered_df.columns)
	num_col = len(user_input)
	if len(user_input) == 3:
		lst_colors = ["blue", "red", "green"]
		fig, ax = plt.subplots(nrows = 1, ncols = num_col, sharey = True)
		for axis, colors, name in zip(ax, lst_colors, name_col):
			axis.plot(time, filtered_df[name], color = colors)
			axis.set_xlabel("Time")
			axis.set_title(name_filter + name[-1])
		fig.text(0.05, 0.5, label_y, ha="center", va="center", rotation=90)
		fig.suptitle(title)
		plt.savefig(title) 
		plt.close()
	elif len(user_input) == 2:
		lst_colors = ["blue", "red"]
		filtered_direction = df[[col for col in filtered_df if col[-1] in user_input]]
		name_col = list(filtered_direction.columns)
		fig, ax = plt.subplots(nrows = 1, ncols = num_col, sharey = True)
		for axis, colors, name in zip(ax, lst_colors, name_col):
			axis.plot(time, filtered_df[name], color = colors)
			axis.set_xlabel("Time")
			axis.set_title(name_filter + name[-1])
		fig.text(0.05, 0.5, label_y, ha="center", va="center", rotation=90)
		fig.suptitle(title)
		plt.savefig(title) 
		plt.close()
	elif len(user_input) == 1:
		filtered_direction = df[[col for col in filtered_df if col[-1] in user_input]]
		plt.figure()
		for column in filtered_direction:
			filtered_direction.plot()
			plt.legend(bbox_to_anchor=(0,-0.08,1,-0.08), loc="upper center", borderaxespad=0)
			plt.xlabel("Time")
			plt.ylabel(label_y)
			plt.title(title + "_" + user_input[0])
			plt.savefig(title, bbox_inches="tight") 
			plt.close()

def main(path_to_urdf, path_to_submechamisms, fix_mesh_orientation=False):
	# Get the name of the robot's body part and the direction(s) from user input in which the motion traj should be calculated
	user_input_EE = str(input("Please enter the name of the robot's body part of which the motion trajectory should be calculated. \nFor example you can use: right_exo_hand_adjustment \n"))
	user_input_csv = str(input("Please enter csv if you want to provide a csv file to analyze the robot's movement \nor enter manual in case you want to define the minimum and maximum positions of the robot manually. \n"))
	user_input_direction = list(str(input("Please enter the direction(s) for the robot's body part " + user_input_EE + " in which the motion trajectory should be calculated. \nThis can be x, y and/or z. \nIf you choose more than one direction, seperate them by space! \n")).split())
	if user_input_csv != "csv":
		for i in user_input_direction:
			min_direc, max_direc = [float(s) for s in input("Please enter the minimum and maximum position in " + i + " direction, separated by space. \n").split()]
			if i == "x":
				pos_input_min_x = min_direc 
				pos_input_max_x = max_direc 
			elif i == "y":
				pos_input_min_y = min_direc 
				pos_input_max_y = max_direc 
			elif i == "z":
				pos_input_min_z = min_direc 
				pos_input_max_z = max_direc
	elif user_input_csv == "csv":
		csv_provided = str(input("Please enter the name of the csv file you want to provide. Please pay attention to the correct spelling! \n"))
		csv_file_path = str(input("Please enter the file path of the provided csv with respect to AUTOPROJ_CURRENT_ROOT. \nIf you do not provide a file path, the csv file should be available locally in the hyrodynpy/script folder. \n"))

	# Path to urdf and submechanisms, define body name
	#path_to_urdf = os.environ["AUTOPROJ_CURRENT_ROOT"] + "/control/hyrodyn/data/hybrid/rh5/leg/urdf/leg.urdf"
	#path_to_submechamisms = os.environ["AUTOPROJ_CURRENT_ROOT"] + "/control/hyrodyn/data/hybrid/rh5/leg/urdf/submechanisms.yml"
	#body_name = user_input_EE

	#path_to_urdf = os.environ["AUTOPROJ_CURRENT_ROOT"] + "/control/hyrodyn/data/hybrid/recupera/right_arm_teleop/urdf/right_arm_teleop.urdf"
	#path_to_submechamisms = os.environ["AUTOPROJ_CURRENT_ROOT"] + "/control/hyrodyn/data/hybrid/recupera/right_arm_teleop/urdf/submechanisms.yml"

	# Path to URDF and Submechanisms files 
	path_to_urdf =  os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_urdf
	path_to_submechamisms = os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_submechamisms
	body_name = user_input_EE

	# Read csv file and get the time index, pos_x, pos_y and pos_z of the EE from this file 
	if user_input_csv == "csv":
		if csv_file_path != "":
			os.chdir(os.environ["AUTOPROJ_CURRENT_ROOT"] + csv_file_path)
		csv_file = pd.read_csv(csv_provided)
		csv_file.set_index("Time (s)", inplace = True)
		lst_column_names = list(csv_file.columns)
		lst_desired_poses = csv_file.to_numpy().tolist()
		time_vec, pos_x, pos_y, pos_z = define_pos_time_array(csv_file, lst_column_names)
		print("positions are defined by external csv file.")
		# Get minimum and maximum posiions
		pos_input_min_x = min([min(val) for val in pos_x])
		pos_input_max_x = max([max(val) for val in pos_x])
		pos_input_min_y = min([min(val) for val in pos_y])
		pos_input_max_y = max([max(val) for val in pos_y])
		pos_input_min_z = min([min(val) for val in pos_z])
		pos_input_max_z = max([max(val) for val in pos_z])
		if csv_file_path != "":
			os.chdir(os.environ["AUTOPROJ_CURRENT_ROOT"] + "/control/hyrodynpy/scripts/")

	# Load the robot model in HyRoDyn
	robot = hyrodyn.RobotModel(path_to_urdf, path_to_submechamisms)

	independent_jointnames = robot.jointnames_independent
	spanningtree_jointnames = robot.jointnames_spanningtree
	active_jointnames = robot.jointnames_active
	# Create a configuration trajectory dict for all the spanning tree joints
	keys = spanningtree_jointnames
	values = []
	for key in keys:
		values.append([])
	config_trajectory = dict(zip(keys, values))

	# Forward Kinematics
	robot.calculate_forward_kinematics(body_name)
	print("Forward kinematics of the body " + body_name + " ([X Y Z Qx Qy Qz Qw]):", robot.pose)

	# Inverse Kinematics
	body_names = [user_input_EE]
	if user_input_csv != "csv":
		desired_pose = np.zeros(7)
		desired_pose = robot.pose[0]
	elif user_input_csv == "csv":
		desired_pose = lst_desired_poses[0]
		desired_pose = robot.pose[0] 

	if user_input_csv != "csv":
		pos_zeroconfig_x = robot.pose[0][0] # For X coordinate example
		pos_zeroconfig_y = robot.pose[0][1] # For Y coordinate example
		pos_zeroconfig_z = robot.pose[0][2] # For Z coordinate example 
		pos_zeroconfig = np.linalg.norm(robot.pose[0])
	elif user_input_csv == "csv":
		pos_zeroconfig_x = lst_desired_poses[0][0] # For X coordinate example
		pos_zeroconfig_y = lst_desired_poses[0][1] # For Y coordinate example
		pos_zeroconfig_z = lst_desired_poses[0][2] # For Z coordinate example 
		pos_zeroconfig = np.linalg.norm(lst_desired_poses[0])

	if user_input_csv != "csv":
		Tf = 4.0
		dt = 0.02
		N = 200
		time_vec = np.linspace(0, Tf, N)

		# Create trajectory plots of the EE in each direction
		if "x" in user_input_direction:
			print("pos_input_min_x:", pos_input_min_x)
			print("pos_input_max_x:", pos_input_max_x)
		else:
			pos_input_min_x = pos_input_max_x = pos_zeroconfig_x
		if "y" in user_input_direction:
			print("pos_input_min_y:", pos_input_min_y)
			print("pos_input_max_y:", pos_input_max_y)
		else:
			pos_input_min_y = pos_input_max_y = pos_zeroconfig_y
		if "z" in user_input_direction:
			print("pos_input_min_z:", pos_input_min_z)
			print("pos_input_max_z:", pos_input_max_z)
		else:
			pos_input_min_z = pos_input_max_z = pos_zeroconfig_z
		time, pos_x, vel_x, acc_x = single_coord_traj (pos_zeroconfig_x, pos_input_min_x, pos_input_max_x, Tf, dt)
		time, pos_y, vel_y, acc_y = single_coord_traj (pos_zeroconfig_y, pos_input_min_y, pos_input_max_y, Tf, dt)
		time, pos_z, vel_z, acc_z = single_coord_traj (pos_zeroconfig_z, pos_input_min_z, pos_input_max_z, Tf, dt)
		lst_pos_input = []
		for i in range(len(pos_x)):
			pos_input = [float(pos_x[i]), float(pos_y[i]), float(pos_z[i])]
			lst_pos_input.append(pos_input)
	elif user_input_csv == "csv":
		lst_pos_input = []
		for i in range(len(lst_desired_poses)):
			pos_input_csv = [float(lst_desired_poses[i][0]), float(lst_desired_poses[i][1]), float(lst_desired_poses[i][2])]
			lst_pos_input.append(pos_input_csv)

	def IK_sys_state (lst_pos_input, desired_pose):
		log_y = []
		log_yd = []
		log_ydd = []
		log_q = []
		log_qd = []
		log_qdd = []
		log_u = []
		log_ud = []
		log_udd = []
		log_tau = []
		if user_input_csv != "csv":
			for i in range(len(lst_pos_input)):
				desired_pose[0:3]=lst_pos_input[i]
				robot.pose_input = [desired_pose]
				robot.calculate_inverse_kinematics(body_names)
				print("Inverse Kinematics output joint config = ",robot.y)
				# Compute full state of the spanning tree
				robot.calculate_system_state()
				log_y.append(robot.y.flatten())
				log_yd.append(robot.yd.flatten())
				log_ydd.append(robot.ydd.flatten())
				log_q.append(robot.Q.flatten())
				log_qd.append(robot.QDot.flatten())
				log_qdd.append(robot.QDDot.flatten())
				log_u.append(robot.u.flatten())
				log_ud.append(robot.ud.flatten())
				log_udd.append(robot.udd.flatten())
				log_tau.append(robot.Tau_actuated.flatten())
				for j in range(len(spanningtree_jointnames)):
					config_trajectory[keys[j]].append(robot.Q[0][j])
		elif user_input_csv == "csv":
			for i in range(len(lst_pos_input)):
				desired_pose[0:3] = lst_pos_input[i]
				robot.pose_input = [desired_pose]
				robot.calculate_inverse_kinematics(body_names)
				print("Inverse Kinematics output joint config = ",robot.y)
				# Compute full state of the spanning tree
				robot.calculate_system_state()
				log_y.append(robot.y.flatten())
				log_yd.append(robot.yd.flatten())
				log_ydd.append(robot.ydd.flatten())
				log_q.append(robot.Q.flatten())
				log_qd.append(robot.QDot.flatten())
				log_qdd.append(robot.QDDot.flatten())
				log_u.append(robot.u.flatten())
				log_ud.append(robot.ud.flatten())
				log_udd.append(robot.udd.flatten())
				log_tau.append(robot.Tau_actuated.flatten())
				for j in range(len(spanningtree_jointnames)):
					config_trajectory[keys[j]].append(robot.Q[0][j])
		return log_y, log_yd, log_ydd, log_q, log_qd, log_qdd, log_u, log_ud, log_udd, log_tau

	# Load the robot model in URDFPY
	print("Loading URDF:", path_to_urdf)
	viz = urdfpy.URDF.load(path_to_urdf)

	# Add fix mesh orientation
	if fix_mesh_orientation:
        	mesh_orientation(viz)

	# Load the robot model in HyRoDyn
	robot = hyrodyn.RobotModel(path_to_urdf, path_to_submechamisms)	
	# Load submechanism_yml
	names_submechanisms = []
	list_submech_joints = []
	with open (path_to_submechamisms) as submechanisms_yml:
		submechanisms = yaml.load(submechanisms_yml, Loader = yaml.FullLoader)
		subs = submechanisms ["submechanisms"]
		for sub in subs:
			names_submechanisms.append(sub["contextual_name"])
			list_submech_joints.append(sub["jointnames_spanningtree"])

	# change directory and create a new folder to store the csv files and trajectory plots
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
	finaldirectory = "task_space_motion"
	finalpath = os.path.join(subpath, finaldirectory)
	if not os.path.exists(finalpath):
		os.mkdir(finalpath)
	os.chdir(finalpath)

	# Generate csv files by calculating the IK and compute the full system state
	if user_input_csv != "csv":
		generate_csv_file (body_name, time_vec, body_names, pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z) 
	log_y, log_yd, log_ydd, log_q, log_qd, log_qdd, log_u, log_ud, log_udd, log_tau = IK_sys_state (lst_pos_input, desired_pose)

	lst_IK_not_solved = []
	lst_IK_solved = []
	for IK_output, step in zip(log_y[1:], range(len(lst_pos_input))):
		if np.all((IK_output == 0)):
			lst_IK_not_solved.append(step)
		else:
			lst_IK_solved.append(step)
	if len(lst_IK_not_solved) == 0:
		print("The inverse kinematics could be solved for all steps " + str(lst_IK_solved[0]) + " to " + str(lst_IK_solved[-1]) + "!")
	else:
		print("Cannot solve the inverse kinematics for the steps " + str(lst_IK_not_solved[0]) + " to " + str(lst_IK_not_solved[-1]) + ". \nFrom step " + str(lst_IK_solved[0]) + " on, until step " + str(lst_IK_solved[-1]) + " the inverse kinematics could be solved.")

	swingrob.generate_csv_file ("independent", time_vec, independent_jointnames, log_y, log_yd, log_ydd)
	swingrob.generate_csv_file ("active", time_vec, active_jointnames, log_u, log_ud, log_udd, log_tau)
	swingrob.generate_csv_file ("spanning_tree", time_vec, spanningtree_jointnames, log_q, log_qd, log_qdd)

	# Create subfolders and store the csv files there
	list_sub_folders = [csv_file for csv_file in os.listdir() if csv_file.endswith(".csv")]
	for filename in list_sub_folders:
		plot_path = os.path.abspath(filename)
		if body_name in filename:	
			finaldirectory = "EE_" + body_name
			EEtrajpath = os.path.join(finalpath, finaldirectory)
			if not os.path.exists(EEtrajpath):
				os.mkdir(EEtrajpath)
			os.rename(plot_path, EEtrajpath + "/" + filename)
		if "independent" in filename:
			finaldirectory = "indspace"
			indtrajpath = os.path.join(finalpath, finaldirectory)
			if not os.path.exists(indtrajpath):
				os.mkdir(indtrajpath)
			os.rename(plot_path, indtrajpath + "/" + filename)
		if "active" in filename:
			finaldirectory = "actspace"
			acttrajpath = os.path.join(finalpath, finaldirectory)
			if not os.path.exists(acttrajpath):
				os.mkdir(acttrajpath)
			os.rename(plot_path, acttrajpath + "/" + filename)
		if "spanning_tree" in filename:
			finaldirectory = "cspace"
			ctrajpath = os.path.join(finalpath, finaldirectory)
			if not os.path.exists(ctrajpath):
				os.mkdir(ctrajpath)
			os.rename(plot_path, ctrajpath + "/" + filename)
	os.chdir("../../../")

	# Plot trajectories
	# Create subfolder for EE and change directory
	if user_input_csv != "csv":
		os.chdir(EEtrajpath)
		name_csv_file = body_name + "_task_space_motion.csv"
		plot_traj_EE (user_input_direction, name_csv_file, "pos_", "Position of the EE " + body_name, "position_EE_" + body_name)
		plot_traj_EE (user_input_direction, name_csv_file, "vel_", "Velocity of the EE " + body_name, "velocity_EE_" + body_name)
		plot_traj_EE (user_input_direction, name_csv_file, "acc_", "Acceleration of the EE " + body_name, "acceleration_EE_" + body_name)
		os.chdir("../../../../")
	elif user_input_csv == "csv":
		name_csv_file = csv_provided
		plot_traj_EE (user_input_direction, name_csv_file, "pos_", "Position of the EE " + body_name, "position_EE_" + body_name, user_input_csv = "csv", csv_file_path = csv_file_path)
		plot_traj_EE (user_input_direction, name_csv_file, "vel_", "Velocity of the EE " + body_name, "velocity_EE_" + body_name, user_input_csv = "csv", csv_file_path = csv_file_path)
		plot_traj_EE (user_input_direction, name_csv_file, "acc_", "Acceleration of the EE " + body_name, "acceleration_EE_" + body_name, user_input_csv = "csv", csv_file_path = csv_file_path)
	list_plots_EE = [plot for plot in os.listdir() if plot.endswith(body_name + ".png")]
	for filename in list_plots_EE:
		plot_path = os.path.abspath(filename)
		finaldirectory = "EE_" + body_name
		EEtrajpath = os.path.join(finalpath, finaldirectory)
		if not os.path.exists(EEtrajpath):
			os.mkdir(EEtrajpath)
		os.rename(plot_path, EEtrajpath + "/" + filename)
	os.chdir("../")
	# Create subfolder for indspace and change directory
	finaldirectory = "indspace"
	indtrajpath = os.path.join(finalpath, finaldirectory)
	if not os.path.exists(indtrajpath):
		os.mkdir(indtrajpath)
	os.chdir(indtrajpath)
	swingrob.plot_traj("independent_joints_traj.csv", "y_", "Independent Joint Position", "positions_independent_joints")
	swingrob.plot_traj("independent_joints_traj.csv", "yd_", "Independent Joint Velocity", "velocity_independent_joints")
	swingrob.plot_traj("independent_joints_traj.csv", "ydd_", "Independent Joint Acceleration", "accelration_independent_joints")
	os.chdir("../")
	# Create subfolder for actspace and change directory
	finaldirectory = "actspace"
	acttrajpath = os.path.join(finalpath, finaldirectory)
	if not os.path.exists(acttrajpath):
		os.mkdir(acttrajpath)
	os.chdir(acttrajpath)
	swingrob.plot_traj("active_joints_traj.csv", "u_", "Actuator Joint Position", "positions_actuator_joints")
	swingrob.plot_traj("active_joints_traj.csv", "ud_", "Actuator Joint Velocity", "velocity_actuator_joints")
	swingrob.plot_traj("active_joints_traj.csv", "udd_", "Actuator Joint Acceleration", "accelration_actuator_joints")
	swingrob.plot_traj("active_joints_traj.csv", "Tau_", "Actuator Joint Forces", "actuator_joint_forces")
	os.chdir("../")
	# Create subfolder for cspace and change directory
	finaldirectory = "cspace"
	ctrajpath = os.path.join(finalpath, finaldirectory)
	if not os.path.exists(ctrajpath):
		os.mkdir(ctrajpath)
	os.chdir(ctrajpath)
	swingrob.plot_traj_span(names_submechanisms, list_submech_joints, "spanning_tree_joints_traj.csv", "q_", "Spanning-Tree Joint Position", "positions_spanning_tree_joints")
	swingrob.plot_traj_span(names_submechanisms, list_submech_joints, "spanning_tree_joints_traj.csv", "qd_", "Spanning-Tree Joint Velocity", "velocity_spanning_tree_joints")
	swingrob.plot_traj_span(names_submechanisms, list_submech_joints, "spanning_tree_joints_traj.csv", "qdd_","Spanning-Tree Joint Acceleration", "accelration_spanning_tree_joints")
	list_plot_traj = [plot for plot in os.listdir() if plot.endswith(".png")]
	for i in range(len(names_submechanisms)):
		finaldirectory = names_submechanisms[i]
		temppath = os.path.join(ctrajpath, finaldirectory)
		if not os.path.exists(temppath):
			os.mkdir(temppath)
	submech_folders = [submech.path for submech in os.scandir(ctrajpath) if submech.is_dir()]
	for filename in list_plot_traj:
		plot_path = os.path.abspath(filename)
		for submech_name in submech_folders:
			if str(os.path.basename(submech_name)) in filename:
				os.rename(plot_path, os.path.basename(submech_name) + "/" + filename)
	os.chdir("../")

	# Add the time to the configuration trajectory dictionary - needed for swinging animation
	if user_input_csv != "csv":
		config_trajectory["time"] = time
	elif user_input_csv == "csv":
		config_trajectory["time"] = time_vec

	# Visualize the robot
	swingrob.animate_custom(viz, config_trajectory, True, 5)
	os.chdir("../../../")

	# Create dictionary
	if len(user_input_direction) == 3:
		if len(lst_IK_not_solved) != 0:
			task_space_dict = {"Part of the robot's body analyzed":user_input_EE, "Direction to be analyzed":[str(direc) for x in user_input_direction for direc in x], "Minimum positions":{"x":pos_input_min_x, "y": pos_input_min_y, "z": pos_input_min_z}, "Maximum positions":{"x":pos_input_max_x, "y": pos_input_max_y, "z": pos_input_max_z}, "IK solved":{"first step solved":lst_IK_solved[0], "last step solved":lst_IK_solved[-1]}, "IK not solved":{"first step not solved":lst_IK_not_solved[0], "last step not solved":lst_IK_not_solved[-1]}}
		else:
			task_space_dict = {"Part of the robot's body analyzed":user_input_EE, "Direction to be analyzed":[str(direc) for x in user_input_direction for direc in x], "Minimum positions":{"x":pos_input_min_x, "y": pos_input_min_y, "z": pos_input_min_z}, "Maximum positions":{"x":pos_input_max_x, "y": pos_input_max_y, "z": pos_input_max_z}, "IK solved":{"first step solved":lst_IK_solved[0], "last step solved":lst_IK_solved[-1]}}
	elif len(user_input_direction) == 2:
		if all(direc in user_input_direction for direc in ["x", "y"]):
			if len(lst_IK_not_solved) != 0:
				task_space_dict = {"Part of the robot's body analyzed":user_input_EE, "Direction to be analyzed":[str(direc) for x in user_input_direction for direc in x], "Minimum positions":{"x":pos_input_min_x, "y": pos_input_min_y}, "Maximum positions":{"x":pos_input_max_x, "y": pos_input_max_y}, "IK solved":{"first step solved":lst_IK_solved[0], "last step solved":lst_IK_solved[-1]}, "IK not solved":{"first step not solved":lst_IK_not_solved[0], "last step not solved":lst_IK_not_solved[-1]}}
			else:
				task_space_dict = {"Part of the robot's body analyzed":user_input_EE, "Direction to be analyzed":[str(direc) for x in user_input_direction for direc in x], "Minimum positions":{"x":pos_input_min_x, "y": pos_input_min_y}, "Maximum positions":{"x":pos_input_max_x, "y": pos_input_max_y}, "IK solved":{"first step solved":lst_IK_solved[0], "last step solved":lst_IK_solved[-1]}}
		elif all(direc in user_input_direction for direc in ["x", "z"]):
			if len(lst_IK_not_solved) != 0:
				task_space_dict = {"Part of the robot's body analyzed":user_input_EE, "Direction to be analyzed":[str(direc) for x in user_input_direction for direc in x], "Minimum positions":{"x":pos_input_min_x, "z": pos_input_min_z}, "Maximum positions":{"x":pos_input_max_x, "z": pos_input_max_z}, "IK solved":{"first step solved":lst_IK_solved[0], "last step solved":lst_IK_solved[-1]}, "IK not solved":{"first step not solved":lst_IK_not_solved[0], "last step not solved":lst_IK_not_solved[-1]}}
			else:
				task_space_dict = {"Part of the robot's body analyzed":user_input_EE, "Direction to be analyzed":[str(direc) for x in user_input_direction for direc in x], "Minimum positions":{"x":pos_input_min_x, "z": pos_input_min_z}, "Maximum positions":{"x":pos_input_max_x, "z": pos_input_max_z}, "IK solved":{"first step solved":lst_IK_solved[0], "last step solved":lst_IK_solved[-1]}}
		elif all(direc in user_input_direction for direc in ["y", "z"]):
			if len(lst_IK_not_solved) != 0:
				task_space_dict = {"Part of the robot's body analyzed":user_input_EE, "Direction to be analyzed":[str(direc) for x in user_input_direction for direc in x], "Minimum positions":{"y":pos_input_min_y, "z": pos_input_min_z}, "Maximum positions":{"y":pos_input_max_y, "z": pos_input_max_z}, "IK solved":{"first step solved":lst_IK_solved[0], "last step solved":lst_IK_solved[-1]}, "IK not solved":{"first step not solved":lst_IK_not_solved[0], "last step not solved":lst_IK_not_solved[-1]}}
			else:
				task_space_dict = {"Part of the robot's body analyzed":user_input_EE, "Direction to be analyzed":[str(direc) for x in user_input_direction for direc in x], "Minimum positions":{"y":pos_input_min_y, "z": pos_input_min_z}, "Maximum positions":{"y":pos_input_max_y, "z": pos_input_max_z}, "IK solved":{"first step solved":lst_IK_solved[0], "last step solved":lst_IK_solved[-1]}}
	elif len(user_input_direction) == 1:
		if (user_input_direction == ["x"]):
			if len(lst_IK_not_solved) != 0:
				task_space_dict = {"Part of the robot's body analyzed":user_input_EE, "Direction to be analyzed":[str(direc) for x in user_input_direction for direc in x], "Minimum position":{"x":pos_input_min_x}, "Maximum position":{"x":pos_input_max_x}, "IK solved":{"first step solved": lst_IK_solved[0], "last step solved":lst_IK_solved[-1]}, "IK not solved":{"first step not solved":lst_IK_not_solved[0], "last step not solved":lst_IK_not_solved[-1]}}
			else:
				task_space_dict = {"Part of the robot's body analyzed":user_input_EE, "Direction to be analyzed":[str(direc) for x in user_input_direction for direc in x], "Minimum position":{"x":pos_input_min_x}, "Maximum position":{"x":pos_input_max_x}, "IK solved":{"first step solved": lst_IK_solved[0], "last step solved":lst_IK_solved[-1]}}
		elif (user_input_direction == ["y"]):
			if len(lst_IK_not_solved) != 0:
				task_space_dict = {"Part of the robot's body analyzed":user_input_EE, "Direction to be analyzed":[str(direc) for x in user_input_direction for direc in x], "Minimum position":{"y":pos_input_min_y}, "Maximum position":{"y":pos_input_max_y}, "IK solved":{"first step solved":lst_IK_solved[0], "last step solved":lst_IK_solved[-1]}, "IK not solved":{"first step not solved":lst_IK_not_solved[0], "last step not solved":lst_IK_not_solved[-1]}}
			else:
				task_space_dict = {"Part of the robot's body analyzed":user_input_EE, "Direction to be analyzed":[str(direc) for x in user_input_direction for direc in x], "Minimum position":{"y":pos_input_min_y}, "Maximum position":{"y":pos_input_max_y}, "IK solved":{"first step solved":lst_IK_solved[0]}}
		elif (user_input_direction == ["z"]):
			if len(lst_IK_not_solved) != 0:
				task_space_dict = {"Part of the robot's body analyzed":user_input_EE, "Direction to be analyzed":[str(direc) for x in user_input_direction for direc in x], "Minimum position":{"z":pos_input_min_z}, "Maximum position":{"z":pos_input_max_z}, "IK solved":{"first step solved":lst_IK_solved[0], "last step solved":lst_IK_solved[-1]}, "IK not solved":{"first step not solved":lst_IK_not_solved[0], "last step not solved":lst_IK_not_solved[-1]}}
			else:
				task_space_dict = {"Part of the robot's body analyzed":user_input_EE, "Direction to be analyzed":[str(direc) for x in user_input_direction for direc in x], "Minimum position":{"z":pos_input_min_z}, "Maximum position":{"z":pos_input_max_z}, "IK solved":{"first step solved":lst_IK_solved[0], "last step solved":lst_IK_solved[-1]}}
	return task_space_dict

if __name__ == '__main__':
	# Create the parser
	parser = argparse.ArgumentParser(description = "Analyze the motion of a robot's body part in task space using HyRoDyn. Robot model in HyRoDyn is built using the URDF and submechanisms yaml file. The program asks for the name of the robot's body part which should be analysed and for a csv file which can be provided and which is then used for the analysis of the motion. Alternatively the user can provide minimum and maximum positions in x, y and/or z direction based on which the movement will be analysed. The program also exports csv files for the independent, actuator and spanning-tree joints which include the position, velocity and acceleration of each joint based on the position input defined by the user. Moreover the visualization of the movement in each direction (x, y and z) is exported as png files for each group of joints and an animation of the movement will be stored in gif format. \n\nBasic Usage Example: \n\npython3 analyze_task_space_motion.py /control/hyrodyn/data/hybrid/recupera/right_arm_teleop/urdf/right_arm_teleop.urdf /control/hyrodyn/data/hybrid/recupera/right_arm_teleop/urdf/submechanisms.yml", formatter_class=RawTextHelpFormatter)

	# Add the arguments
	parser.add_argument("urdf_path", type = str,
		            help="URDF path for the robot with respect to AUTOPROJ_CURRENT_ROOT")
	parser.add_argument("submechanism_yml", type = str, 
		            help="YAML file describing the submechanisms with respect to AUTOPROJ_CURRENT_ROOT")
	parser.add_argument("--fix_mesh_orientation", action='store_true', help="This can fix some orientation issues with some OBJ meshes")

	# Execute the parse_args method
	args, unknown = parser.parse_known_args()
	
	# Store the variables in the args
	path_to_urdf = args.urdf_path
	path_to_submechamisms = args.submechanism_yml
	fix_mesh_orientation = args.fix_mesh_orientation

	# Call the main function
	main(path_to_urdf, path_to_submechamisms, fix_mesh_orientation=fix_mesh_orientation)

