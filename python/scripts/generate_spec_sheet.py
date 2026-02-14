from pylatex import Document, Section, Subsection, Command, Figure, SubFigure, Table, Tabular, NewPage, Center, Math, Matrix, VectorName, Itemize, NoEscape, Alignat
from pylatex.utils import bold, italic
from pylatex.package import Package
import os
import argparse
from argparse import RawTextHelpFormatter
import numpy as np
import yaml

def math_array(array_input, vec_name):
	""" Generate an array in Math environment of pylatex
	Parameters
	----------
	array_input : string
	string of the floats which should be converted into an array
	vec_name : string
	name of the created array
	Returns
	-------
	math : array created in math environment of pylatex
	"""
	a_remove_linebreaks = array_input.replace("[[", "").replace("]]", "").replace("\ ", "").replace("\n", ""). replace("  ", " ")
	list_a = list(a_remove_linebreaks.split())
	a_as_array = np.asarray([[float(y) for y in x.split(',')] for x in list_a])
	vec = Matrix(a_as_array)
	math = Math(data=[VectorName(vec_name), " = ", vec])
	return math


def itemize_joints(joint_name, joint_list):
	""" Generate a listing with items
	Parameters
	----------
	joint_name : string
	name of the class of the joints
	joint_list : list
	list of all joint names and their DOF according to the defined joint_name
	"""
	if joint_name in joint_list:
		dof_names = joint_list[joint_name]
		dof_names_values = dof_names.values()
		for joints_dof_names in dof_names_values:
			if type(joints_dof_names) is list:
				names_lists = joints_dof_names
			elif type(joints_dof_names) is int:
				dof = joints_dof_names
	doc.append("The " + joint_name + " of the robot have " + str(dof) + " degrees of freedom and they name:")
	with doc.create (Itemize()) as itemize:
		for item in names_lists:
			itemize.add_item(item)


def generate_zero_config_txt (doc, spec_sheet_yml):
	""" Generate text including arrays and information of the robot's zero configuration
	Parameters
	----------
	doc : document
	spec_sheet_yml : string
	path to the spec sheet yaml file
	Returns
	-------
	doc : document
	document with added text and arrays according to the zero configuration analysis of the robot
	"""
	with open(spec_sheet_yml) as data_yml:
		data_yml = yaml.full_load(data_yml)
		if "Task Space Motion" in data_yml:
			nested_data = (data_yml["Task Space Motion"])
			body_part = nested_data.get("Part of the robot's body analyzed")
		if "Analysis of the robot's zero configuration" in data_yml:
			nested_data = (data_yml["Analysis of the robot's zero configuration"])
			nested_data_values = nested_data.values()
			if "Names and degrees of freedom" in nested_data:
				joint_list = nested_data["Names and degrees of freedom"]
				joint_names = joint_list.keys()
				with doc.create(Subsection("Joint names and their degrees of freedom")):
					doc.append("The Hybrid Robot Dynamics Software uses the modularity of submechanisms. With that it gives three different subsets of joints. First one are the spanning-tree joints ")
					doc.append(italic("q"))
					doc.append(". That are all joints belonging to the spanning tree chosen by a regular numbering scheme. Second are the independent joints ")
					doc.append(italic("y"))
					doc.append(", which are the set of independent variables selected such that they define ")
					doc.append(italic("q "))
					doc.append(" uniquely. At least, there are the active joints ")
					doc.append(italic("u"))
					doc.append(", which are all joints containing the actuators. \nThe names of the joints and the degrees of freedom are given in detail below, grouped by the three subsets.\n")
					for name in joint_names:
						itemize_joints(name, joint_list)
			if "Joint positions" in nested_data:
				joint_pos = list((nested_data["Joint positions"].values()))
				all_joints = joint_pos[0]
				act_joints = joint_pos[1]
				math_all_joints = math_array(all_joints, italic("q"))
				math_act_joints = math_array(act_joints, italic("u"))
				with doc.create(Subsection("Positions of the joints")):
					doc.append("The position of the full system state ")
					doc.append(italic("q "))
					doc.append("as well as the position state of the active joints ")
					doc.append(italic("u "))
					doc.append("are given at robot's zero configuration. For the computation of ")
					doc.append(italic("q"))
					doc.append(", the state of the active joint positions is used as input. Using the formula ")
					doc.append(NoEscape(r"$q=\gamma(y)$"))
					doc.append(", whereby ")
					doc.append(NoEscape(r"$\gamma$ "))
					doc.append("is the loop-closure funtion, the following position state of the spanning tree is calculated: \n")
					doc.append("The position of the robot's spanning tree joints is:")
					doc.append(math_all_joints)
					doc.append("With the help of the independent joint positions, the position of the actuator joints is calculated as:")
					doc.append(math_act_joints)
			if "Mass of the robot" in nested_data:
				mass_config = list(nested_data["Mass of the robot"].values())
				centre_of_mass = mass_config[0]
				math_mass = math_array(centre_of_mass, "c")
				with doc.create(Subsection("Mass of the robot")):
					doc.append("Using HyRoDyn, one could easily get general information about the robot, like the mass and the center of the mass. The calculation is done, using the independent joint state ")
					doc.append(NoEscape(r"$y$, "))
					doc.append(NoEscape(r"$\dot y$ "))
					doc.append("and ")
					doc.append(NoEscape(r"$\ddot y$"))
					doc.append(". \nIn it's zero configuration the total moving mass of the " + os.path.basename(path_to_results_folder) + " is: ")
					doc.append(NoEscape(r"$m = $"))
					doc.append(str(round(float(mass_config[1]), 2)) + " ")
					doc.append("and the center of mass in x, y and z direction is at: ")
					doc.append(math_mass)
			if "Kinematics and dynamics" in nested_data:
				fk_invd = list(nested_data["Kinematics and dynamics"].values())
				fk_array = fk_invd[0]
				invd_array = fk_invd[1]
				math_fk = math_array(fk_array, "X")
				math_invd = math_array(invd_array, NoEscape(r"\tau"))
				with doc.create(Subsection("Kinematics and dynamics of the robot")):
					doc.append("Kinematics and dynamics are the fundamentals of modeling and control of robotic systems. With kinematics the motion mechanics of the robot can be described, which includes the terms position, velocity and acceleration, whereas dynamics specify the relationship between the forces and moments acting on the robot and the motion they produce. We differentiate between forward an dinverse kinematics as well as between forward and inverse dynamics. Forward kinematics determines the configuration of the robot’s end-effector, given the joint variables ")
					doc.append(NoEscape(r"$y, \dot y, \ddot y$"))
					doc.append(". The inverse kinematics describes the inverse problem, how to get ")
					doc.append(NoEscape(r"$y, \dot y, \ddot y$ "))
					doc.append("using the given configuration of the robot. \nHere, the forward kinematics of the robot's end-effector " + body_part + " are calculated: ")
					doc.append(math_fk)
					doc.append("Regarding the dynamics, the forward problem is used to calculate the robot’s acceleration, given the joint forces and torques ")
					doc.append(NoEscape(r"$\tau$"))
					doc.append(". In contrast, with inverse dynamics ")
					doc.append(NoEscape(r"$\tau$ "))
					doc.append("is calculated, whereby the configuration and a desired acceleration of the robot are given. \nThe inverse dynamics, so the joint forces/torques of the robot " + os.path.basename(path_to_results_folder) + " are defined by: ")
					doc.append(math_invd)

	return doc

def recursive_items(dictionary):
	""" Get the keys and values of the lowest level of nested dictionary
	Parameters
	----------
	dictionary : dictionary
	nested dictionary with multiple level
	Returns
	-------
	key, value : strings
	keys and values as strings of the lowest level of the nested dictionary
	"""
	for key, value in dictionary.items():
		if type(value) is dict:
			yield from recursive_items(value)
		else:
			yield (key, value)

def glue_multiple_images(doc, images, images_per_row = 2, images_per_page = 8, fig_caption="", use_filename_as_subfig_caption = False, deactivate_subfig_caption = True):
	""" Create plots and add it to the document
	Parameters
	----------
	doc : document
	images : png-files
	images_per_row, images_per_page : integer
	defines how many images per row and per page will be shown in the document
	fig_caption : string
	caption of the created figure
	use_filename_as_subfig_caption : boolean argument
	use the filename as the caprtion of the subfigure, by default this is set to False
	deactivate_subfig_caption : boolean argument
	deactivate the subcaptions of the figures, by default this is set to True
	Returns
	-------
	doc : document
	the document is been added with the images
	"""
	if len(images) == 1:
		with doc.create(Figure(position="h!")) as plot:
			plot.add_image(images[0], width=NoEscape(r"0.75\linewidth"))
			plot.add_caption(fig_caption)
			return doc

	num_images = len(images)
	# We switch to Hindu-Arabic numerals if number of subfigures is greater than 26 as alphabetical ordering does not make sense.
	if num_images > 26:
		doc.append(NoEscape(r"\renewcommand{\thesubfigure}{\arabic{subfigure}}"))
	image_width = 0.95/images_per_row
	images_in_bins = [images[i:i + images_per_page] for i in range(0, len(images), images_per_page)]
	# print("images_in_bins", images_in_bins)
	current_page_count = 1
	for image_set in images_in_bins:
		current_images_per_row = 1
		with doc.create(Figure(position="h!")) as fig:
			if current_page_count > 1:
				doc.append(NoEscape(r"\ContinuedFloat"))
			for i in range(len(image_set)):
				with doc.create(SubFigure(position="h!",width=NoEscape(r""+str(image_width)+"\linewidth"))) as subfig:
					subfig.add_image(image_set[i], width=NoEscape(r"\linewidth"))
					if deactivate_subfig_caption is False:
						if use_filename_as_subfig_caption is True:
							title = os.path.splitext(os.path.split(image_set[i])[1])[0].replace("_", " ").capitalize()
							subfig.add_caption(title)
						else:
							subfig.add_caption("")
				doc.append(NoEscape(r"\hfill"))
				if current_images_per_row == images_per_row:
					doc.append("\n")
					doc.append(NoEscape(r"\vfill"))
					current_images_per_row = 0
				current_images_per_row = current_images_per_row + 1
			if current_page_count == len(images_in_bins):
				fig.add_caption(fig_caption)
			else:
				fig.add_caption(fig_caption + " (contd.)")
			current_page_count = current_page_count + 1
	# Switch back to alphabetic ordering
	if num_images > 26:
		doc.append(NoEscape(r"\renewcommand{\thesubfigure}{\alph{subfigure}}"))
	return doc

def generate_cspace_robot(doc, cspace_folder_path, images_per_row = 2, images_per_page = 8):
	""" Create plots and add it to the document
	Parameters
	----------
	doc : document
	cspace_folder_path : string
	path to the folder where the images are stored
	images_per_row, images_per_page : integer
	defines how many images per row and per site will be shown in the document
	Returns
	-------
	doc : document
	the document is been added with the images according to the submechanisms
	"""
	submechanisms = [d for d in os.listdir(cspace_folder_path) if os.path.isdir(os.path.join(cspace_folder_path, d))]
	print("submechanisms", submechanisms)
	for submech in submechanisms:
		cspace_folder_path_to_submech = os.path.join(cspace_folder_path, submech)
		if os.path.isdir(os.path.join(cspace_folder_path_to_submech, "trimmed")):
			cspace_folder_path_to_submech = cspace_folder_path_to_submech + "/trimmed"
		rel_cspace_folder_path = os.path.relpath(cspace_folder_path_to_submech, os.getcwd())
		cspace_images = [f for f in os.listdir(cspace_folder_path_to_submech) if os.path.isfile(os.path.join(cspace_folder_path_to_submech, f))]
		cspace_images = [os.path.join(rel_cspace_folder_path, cs) for cs in cspace_images]

		if "2d" in cspace_folder_path:
			dim = "2D"
		elif "3d" in cspace_folder_path:
			dim = "3D"

		if "actspace" in cspace_folder_path:
			jointspace_type = " Actuation "
		elif "indspace" in cspace_folder_path:
			jointspace_type = " Independent joint "
		elif "cspace" in cspace_folder_path:
			jointspace_type = " Configuration "

		fig_title = dim + jointspace_type + " space slices of " + submech.replace("_", " ") + " submechanism"
		doc = glue_multiple_images(doc, cspace_images, images_per_row, images_per_page, fig_title, False, True)
	return doc

def generate_cspace_tables (doc, spec_sheet_yml, key_space):
	""" Create joint space limits table out of a yaml file
	Parameters
	----------
	doc : document
	spec_sheet_yml : string
	path to the spec sheet yaml file
	key_space : string
	defines the space in which the joints actuated
	Returns
	-------
	doc : document
	the document is been added with the tables according to the joints
	"""
	with open(spec_sheet_yml) as data_yml:
		data_yml = yaml.full_load(data_yml)
		if "Minimum and maximum position of submechanism in different spaces" in data_yml:
			nested_data = (data_yml["Minimum and maximum position of submechanism in different spaces"])
			if key_space in nested_data:
				joint_pos = (nested_data[key_space])
				if "inv_cond_loop_closure_active_jointspace" in joint_pos:
					joint_pos.pop("inv_cond_loop_closure_active_jointspace")
				if "inv_cond_loop_closure_independent_jointspace" in joint_pos:
					joint_pos.pop("inv_cond_loop_closure_independent_jointspace")
				joint_names = list(joint_pos.keys())
				min_pos = [min_val["min_pos"] for key, min_val in joint_pos.items()]
				max_pos = [max_val["max_pos"] for key, max_val in joint_pos.items()]
				with doc.create(Table(position="h!")) as table:
					with doc.create(Center()) as centered:
						with centered.create(Tabular("l|c|c")) as tabular:
							header_row = (["Joint name", "Minimum position", "Maximum position"])
							tabular.add_row(header_row, mapper=[bold])
							tabular.add_hline()
							for key, i, j in zip(joint_pos, min_pos, max_pos):
								key = key.replace("q_","")
								key = key.replace("u_","")
								key = key.replace("y_","")
								if "_Act" in key:
									tabular.add_row(key, (str(round(i,2)) + " m"), (str(round(j,2)) + " m"))
								else:
									tabular.add_row(key, (str(round(i,2)) + " rad"), (str(round(j,2)) + " rad"))
							tabular.add_hline()
					table.add_caption(key_space)
	return doc

def generate_workspace_table (doc, spec_sheet_yml):
	""" Create workspace bounding box table out of a yaml file
	Parameters
	----------
	doc : document
	spec_sheet_yml : string
	path to the spec sheet yaml file
	Returns
	-------
	doc : document
	the document with added tex code for workspace bounding box table
	"""
	with open(spec_sheet_yml) as data_yml:
		data_yml = yaml.full_load(data_yml)
		if "Workspace bounding box" in data_yml:
			nested_data = (data_yml["Workspace bounding box"])
			bb_info = list(nested_data.keys())
			bb_info.pop()
			value_list = []
			for key, value in recursive_items(nested_data):
				value_list.append(value)
			x_list_val = [value_list[0], value_list[3], value_list[6], value_list[9]]
			y_list_val = [value_list[1], value_list[4], value_list[7], value_list[10]]
			z_list_val = [value_list[2], value_list[5], value_list[8], value_list[11]]
			with doc.create(Table(position="h!")) as table:
				with doc.create(Center()) as centered:
					with centered.create(Tabular("l|c|c|c")) as tabular:
						header_row = (["Bounding Box", "X", "Y", "Z"])
						tabular.add_row(header_row, mapper=[bold])
						tabular.add_hline()
						for key, i, j, k in zip(bb_info, x_list_val, y_list_val, z_list_val):
							tabular.add_row(key, (str(round(i,2)) + " m"), (str(round(j,2)) + " m"), (str(round(k,2)) + " m"))
						tabular.add_hline()
				table.add_caption("Dimensions of the workspace bounding box with a volume of " + str(round(value_list[12], 2)) + "m³.")
		return doc

def generate_torque_limits_table (doc, spec_sheet_yml):
	""" Create torque limit table out of a yaml file
	Parameters
	----------
	doc : document
	spec_sheet_yml : string
	path to the spec sheet yaml file
	Returns
	-------
	doc : document
	the document with added tex code for torque limit table
	"""
	with open(spec_sheet_yml) as data_yml:
		data_yml = yaml.full_load(data_yml)
		if "Minimum and maximum forces" in data_yml:
			nested_data = (data_yml["Minimum and maximum forces"])
			joint_names = list(nested_data.keys())
			min_forces = [min_for["min_forces"] for key, min_for in nested_data.items()]
			max_forces = [max_for["max_forces"] for key, max_for in nested_data.items()]
			with doc.create(Table(position="h!")) as table:
				with doc.create(Center()) as centered:
					with centered.create(Tabular("l|c|c")) as tabular:
						header_row = (["Joint Name", "Minimum Force/Torque", "Maximum Force/Torque"])
						tabular.add_row(header_row, mapper=[bold])
						tabular.add_hline()
						for key, i, j in zip(joint_names, min_forces, max_forces):
							key = key.replace("Tau_","")
							if "_Act" in key:
								tabular.add_row(key, (str(round(i,2)) + " N"), (str(round(j,2)) + " N"))
							else:
								tabular.add_row(key, (str(round(i,2)) + " Nm"), (str(round(j,2)) + " Nm"))
						tabular.add_hline()
				table.add_caption("Minimum and maximum forces/torques at the seperate joints.")
	return doc

def generate_taskspace_text(doc, spec_sheet_yml):
	with open(spec_sheet_yml) as data_yml:
		data_yml = yaml.full_load(data_yml)
		if "Task Space Motion" in data_yml:
			nested_data = (data_yml["Task Space Motion"])
			if "IK not solved" in nested_data and "IK solved" in nested_data:
				steps_not_solved = (nested_data["IK not solved"])	
				steps_solved = (nested_data["IK solved"])
				doc.append("The inverse kinematics could not be solved for all steps. There was no solution for the steps " + str(steps_not_solved["first step not solved"]) + " to " + str(steps_not_solved["last step not solved"]) + ". The steps " + str(steps_solved["first step solved"]) + " to " + str(steps_solved["last step solved"]) + " could be solved.")
			elif "IK solved" in nested_data:
				steps_solved = (nested_data["IK solved"])
				doc.append("Here, the inverse kinematics could be solved for all steps " + str(steps_solved["first step solved"]) + " to " + str(steps_solved["last step solved"]) + ".")
			elif "IK not solved" in nested_data:
				steps_not_solved = (nested_data["IK not solved"])
				doc.append("Here, the inverse kinematics failed for all steps " + str(steps_not_solved["first step not solved"]) + " to " + str(steps_not_solved["last step not solved"]) + ".")
	return doc

def generate_taskspace_table (doc, spec_sheet_yml):
	""" Create taskspace motion table out of a yaml file
	Parameters
	----------
	doc : document
	spec_sheet_yml : string
	path to the spec sheet yaml file
	Returns
	-------
	doc : document
	the document with added tex code for taskspace motion table
	"""
	with open(spec_sheet_yml) as data_yml:
		data_yml = yaml.full_load(data_yml)
		if "Task Space Motion" in data_yml:
			nested_data = (data_yml["Task Space Motion"])
			body_part = nested_data.get("Part of the robot's body analyzed")
			min_max_pos_data = dict(nested_data)
			min_max_pos_data.pop("Part of the robot's body analyzed")
			min_max_pos_data.pop("Direction to be analyzed")
			if "IK not solved" in nested_data:
				min_max_pos_data.pop("IK not solved")
			if "IK solved" in nested_data:
				min_max_pos_data.pop("IK solved")
			if len(nested_data["Direction to be analyzed"]) == 3:
				with doc.create(Table(position="h!")) as table:
					with doc.create(Center()) as centered:
						with centered.create(Tabular("l|c|c|c")) as tabular:
							key_lst = []
							for key, value in recursive_items(min_max_pos_data):
								key_lst.append(key)
							header_row = ([body_part, key_lst[0], key_lst[1], key_lst[2]])
							tabular.add_row(header_row, mapper=[bold])
							tabular.add_hline()
							for key, value in zip(min_max_pos_data, min_max_pos_data.values()):
								val_lst = []
								for val in value.values():
									val_lst.append(val)
								tabular.add_row(key, (str(round(val_lst[0],2)) + " m"), (str(round(val_lst[1],2)) + " m"), (str(round(val_lst[2],2)) + " m"))
							tabular.add_hline()
				table.add_caption("Defined task space motion by minimum and maximum positions in " + key_lst[0] + ", " + key_lst[1] + " and " + key_lst[2] + " directions")
			elif len(nested_data["Direction to be analyzed"]) == 2:
				with doc.create(Table(position="h!")) as table:
					with doc.create(Center()) as centered:
						with centered.create(Tabular("l|c|c")) as tabular:
							key_lst = []
							for key, value in recursive_items(min_max_pos_data):
								key_lst.append(key)
							header_row = ([body_part, key_lst[0], key_lst[1]])
							tabular.add_row(header_row, mapper=[bold])
							tabular.add_hline()
							for key in zip(min_max_pos_data):
								tabular.add_row(key, (str(round(val_lst[0],2)) + " m"), (str(round(val_lst[1],2)) + " m"))
							tabular.add_hline()
				table.add_caption("Defined task space motion by minimum and maximum positions in " + key_lst[0] + " and " + key_lst[1] + " directions")
			elif len(nested_data["Direction to be analyzed"]) == 1:
				with doc.create(Table(position="h!")) as table:
					with doc.create(Center()) as centered:
						with centered.create(Tabular("l|c")) as tabular:
							key_lst = []
							for key, value in recursive_items(min_max_pos_data):
								key_lst.append(key)
							header_row = ([body_part, key_lst[0]])
							tabular.add_row(header_row, mapper=[bold])
							tabular.add_hline()
							for key in zip(min_max_pos_data):
								tabular.add_row(key, (str(round(val_lst[0],2)) + " m"))
							tabular.add_hline()
				table.add_caption("Defined task space motion by minimum and maximum positions in " + key_lst[0] + " direction")
		return doc

def preface_doc(doc):
	with doc.create(Section("Preface", numbering=False)):
		doc.append("This specification sheet is created using the framework HyRoDyn, the Hybrid Robot Dynamics Software. This tool is developed by the DFKI for help solving kinematics and dynamics of robotic systems. With the implemented python applications, HyRoDyn can be used for the purpose of robot analysis, prototyping and testing. All gained information are summarized in a legible form in this sheet. Moreover, the approaches to analytical calculations are described to make the results more transparent. It gives multiple sections, devided according to the seperate analysis tools, e.g. the configuration-, actuation-, and independent-space, the statics of the robotic system, the workspace, and the taskspace are calculated. Furthermore, general information like the mass, the joint names or the degrees of freedom are given.")

def zero_config_doc(doc):
	with doc.create(Section("Zero Configuration Analysis")):
		doc.append("This section presents the zero configuration analysis and gives some general information about the concerned robot, the " + os.path.basename(path_to_results_folder) + ". To get these information, the robot model is loaded in HyRoDyn.")
		doc = generate_zero_config_txt (doc, spec_sheet_yml)
		doc.generate_tex("zero_config_" + folder_name)

def swing_my_robot_doc(doc):
	with doc.create(Section("Swing My Robot Analysis")):
		doc.append("This section presents the swinging animation analysis of the concerned robot " + os.path.basename(path_to_results_folder) + " based on graphs of the joint's position, velocity and acceleration. To create these graphs, the minimum and maximum position of the spanning-tree joints ")
		doc.append(italic("q"))
		doc.append(", the independent joints ") 
		doc.append(italic("y "))
		doc.append("and the actuator joints ")
		doc.append(italic("u "))
		doc.append("are required. These values can be extracted from the joint limits yaml file and with it a trajectory can be computed such that each joint moves between it's minimum and maximum position. Using the derivations of the position-trajectory, the velocity and the acceleration of each joint can also be calculated and visualized. \nFor the depiction of the spanning-tree joints, the joints are grouped by submechanism. Additionally, the forces/torques acting on the actuators are shown in one plot. The creation of the plots is done by using the tool matplotlib. In the following you can see all the graphs for the joints position, velocity and acceleration.")
		cspace_traj_path = os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_results_folder + "/swing_my_robot/cspace"
		submechanism_folders = [d for d in os.listdir(cspace_traj_path) if os.path.isdir(os.path.join(cspace_traj_path, d))]
		for submech in submechanism_folders:
			cspace_folder_path_to_submech = os.path.join(cspace_traj_path, submech)
			spanning_tree_images = [os.path.join(cspace_folder_path_to_submech, an) for an in os.listdir(cspace_folder_path_to_submech) if an.endswith(".png")]
			doc = glue_multiple_images(doc, spanning_tree_images, 2, 6, "Position, Velocity and Acceleration of the " + submech.replace("_", " ") + " Submechanisms ", False, True)
		indspace_traj_path = os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_results_folder + "/swing_my_robot/indspace"
		actspace_traj_path = os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_results_folder + "/swing_my_robot/actspace"
		independent_images = [os.path.join(indspace_traj_path, an) for an in os.listdir(indspace_traj_path) if an.endswith(".png")]
		actuator_images = [os.path.join(actspace_traj_path, an) for an in os.listdir(actspace_traj_path) if an.endswith(".png")]
		doc = glue_multiple_images(doc, independent_images, 2, 6, "Position, Velocity and Acceleration of the Robot's Independent Joints", False, True)
		doc = glue_multiple_images(doc, actuator_images, 2, 6, "Position, Velocity, Acceleration and Forces of the Robot's Actuator Joints", False, True)
		doc.append(NoEscape(r"\newpage"))
		doc.generate_tex("swing_my_robot_" + folder_name)

def workspace_doc(doc):
	with open(spec_sheet_yml) as data_yml:
		data_yml = yaml.full_load(data_yml)
		if "Task Space Motion" in data_yml:
			nested_data = (data_yml["Task Space Motion"])
			body_part = nested_data.get("Part of the robot's body analyzed")
	with doc.create(Section("Workspace Analysis")):
		workspace_folder_path = os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_results_folder + "/workspace"
		if os.path.isdir(os.path.join(workspace_folder_path, "trimmed")):
			workspace_folder_path = workspace_folder_path + "/trimmed"
		rel_workspace_folder_path = os.path.relpath(workspace_folder_path, os.getcwd())
		workspace_images = [f for f in os.listdir(rel_workspace_folder_path) if f.endswith(".png") and os.path.isfile(os.path.join(rel_workspace_folder_path, f))]
		workspace_images = [os.path.join(rel_workspace_folder_path, wm) for wm in workspace_images]
		doc.append("This section presents the workspace analysis of the concerned robot. \nThe workspace is specified by the configurations the robot's end-effector can reach. One point in the workspace my be achievable by more than one configuration, which means that this point does not fully specify the configuration. However, all points in the workspace are reachable by at least one configuration of the end-effector. \nWith the workspace tool of the Robot Analysis Application in HyRoDyn, the robot with it's workspace (outlined by a box) will be visualized in seven different views. The reachable points for the end-effector " + body_part + " are shown as colorful spheres in the bounding box. Thereby, the colors are calculated with condition values so that easy reachable points are marked in green and the ones which are not easy to reach in red. \nWorkspace visualization of the " + os.path.basename(path_to_results_folder) + " in seven different views:")
		doc = glue_multiple_images(doc, workspace_images, 2, 6, "Workspace Analysis", True, False)
		with doc.create(Subsection("Workspace bounding box")):
			doc.append("The dimensions of the bounding box are computed in each direction. Additionally the volume is calculated and given.")
			doc = generate_workspace_table (doc, spec_sheet_yml)
		doc.append(NoEscape(r"\newpage"))
		doc.generate_tex("workspace_" + folder_name)

def statics_doc(doc):
	with doc.create(Section("Static Force Analysis")):
		static_folder_path = os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_results_folder + "/statics"
		if os.path.isdir(os.path.join(static_folder_path, "trimmed")):
			static_folder_path = static_folder_path + "/trimmed"
		rel_static_folder_path = os.path.relpath(static_folder_path, os.getcwd())
		statics_images = [f for f in os.listdir(rel_static_folder_path) if os.path.isfile(os.path.join(rel_static_folder_path, f))]
		statics_images = [os.path.join(rel_static_folder_path, wm) for wm in statics_images]
		doc.append("This section presents results of the static force analysis of the concerned robot. \nThe robot analysis of HyRoDyn includes another python tool for representing the amount of forces the robot experienced in each configuration of the end-effector in the workspace. This tool works similar to the one of the workspace analysis. The visualized forces/torques are taken from the generated csv file which is created by using HyRoDyn. Again seven images with different views of the " + os.path.basename(path_to_results_folder) + " are created. In comparison to the workspace images, here the colors of the spheres show the amount of forces, the robot experienced at this point. So red means a high amount and green a low amount. \nBelow, the static force visualization of the " + os.path.basename(path_to_results_folder) + " in seven different views is printed:")
		doc = glue_multiple_images(doc, statics_images, 2, 6, "Static Force Analysis", True, False)
		with doc.create(Subsection("Static forces/torques")):
			doc.append("For a better overview, the minimum and maximum forces/torques are summarized for each separate joint in a table:")
			doc = generate_torque_limits_table (doc, spec_sheet_yml)
		doc.append(NoEscape(r"\newpage"))
		doc.generate_tex("statics_" + folder_name)
	
def cspace_doc(doc):
	with doc.create(Section("Configuration Space Analysis")):
		doc.append("This section presents the configuration space analysis of the " + os.path.basename(path_to_results_folder) + ". \nThe configuration or C-space is the n-dimensional space containing all possible positions of the robot and one specific configuration can be represented by a point in this space. The dimension of this space is defined by the number of independent parameters ")
		doc.append(italic("n "))
		doc.append("to describe the position and orientation of the robot. \nClosed loops constraint the dimension of the C-Space. These so called holonomic constraints can be computed using implicit parametrization. The loop-cosure equations have the form ")
		doc.append(NoEscape(r"$g(q)=0$ ."))
		with doc.create(Subsection("Joint positions in configuration space")):
			doc.append("In this subsection, the minimum and maximum position in the configuration space of all spanning-tree joints ")
			doc.append(italic("q "))
			doc.append("are summarized in a table.")
			doc = generate_cspace_tables (doc, spec_sheet_yml, "Minimum and maximum position in configuration space")
		with doc.create(Subsection("Configuration space visualization 2D")):
			doc.append("With the C-Space tool in the robot analysis of HyRoDyn, multiple two dimensional plots are created using matplotlib. In these plots multiple configurations of two joints are plotted against each other, whenever there is a cetain dependency between the configurations of these two joints, as shown in the following.")
			doc = generate_cspace_robot(doc, os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_results_folder + "/cspace/2d_slices/")
		with doc.create(Subsection("Configuration space visualization 3D")):
			doc.append("Moreover, the C-Space analysis presents also three-dimensional plots, which show certain dependencies between multiple configurations of three joints in the C-Space. These plots can be seen here.")
			doc = generate_cspace_robot(doc, os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_results_folder + "/cspace/3d_slices/", 2, 6)
		doc.append(NoEscape(r"\newpage"))
		doc.generate_tex("cspace_" + folder_name)

def actspace_doc(doc):
	with doc.create(Section("Actuation Space Analysis")):
		doc.append("This section presents the actuation space analysis of the concerned robot. \nThe actuation space includes all configurations defined by the actuator joints.")
		with doc.create(Subsection("Joint positions in actuation space")):
			doc.append("In this subsection, the minimum and maximum position in the actuation space of all actuator joints ")
			doc.append(italic("u "))
			doc.append("are summarized in a table.")
			doc = generate_cspace_tables (doc, spec_sheet_yml, "Minimum and maximum position in actuator space")
		with doc.create(Subsection("Actuation space visualization 2D")):
			doc.append("With the Act-Space tool in the robot analysis of HyRoDyn, multiple two dimensional plots are created using matplotlib. In these plots multiple configurations of two actuator joints are plotted against each other to show cetain dependencies between the configurations of these two actuator joints. The plots are shown below.")
			doc = generate_cspace_robot(doc, os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_results_folder + "/actspace/2d_slices/")
		with doc.create(Subsection("Actuation space visualization 3D")):
			doc.append("As for the C-Space, the Act-Space analysis presents also three-dimensional plots, which show certain dependencies between multiple configurations of three actuator joints in the Act-Space. These plots are also shown here.")
			doc = generate_cspace_robot(doc, os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_results_folder + "/actspace/3d_slices/", 2, 6)
		doc.append(NoEscape(r"\newpage"))
		doc.generate_tex("actspace_" + folder_name)

def indspace_doc(doc):
	with doc.create(Section("Independent Joint Space Analysis")):
		doc.append("This section presents the independent joint space analysis of the concerned robot, the " + os.path.basename(path_to_results_folder) + ". Similar to the actuation space, the independent space is defined by the independent joint configurations.")
		with doc.create(Subsection("Joint positions in independent space")):
			doc.append("In this subsection, the minimum and maximum position in the independent space of all independent joints ")
			doc.append(italic("y "))
			doc.append("are summarized in a table.")
			doc = generate_cspace_tables (doc, spec_sheet_yml, "Minimum and maximum position in independent space")
		with doc.create(Subsection("Independent space visualization 2D")):
			doc.append("Similarly to the other space analysis, the Ind-Space tool in the robot analysis of HyRoDyn creates multiple two dimensional plots using matplotlib. In these plots multiple configurations of two independent joints are plotted against each other to show cetain dependencies between the configurations of these two joints, as shown here.")
			doc = generate_cspace_robot(doc, os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_results_folder + "/indspace/2d_slices/")
		with doc.create(Subsection("Independent space visualization 3D")):
			doc.append("Additionally, the Ind-Space analysis presents also three-dimensional plots, which show certain dependencies between multiple configurations of three independent joints in this space. These plots are also shown below.")
			doc = generate_cspace_robot(doc, os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_results_folder + "/indspace/3d_slices/", 2, 6)
		doc.append(NoEscape(r"\newpage"))
		doc.generate_tex("indspace_" + folder_name)

def taskspace_doc(doc):
	with doc.create(Section("Task Space Motion")):
		doc.append("This section presents the motion of a part of the concerned robot in the task space. \nThe tsak space is the space, in which the robot's task can be naturally expressed and it is independent of the robot. So in comparison to the workspace, the task space is driven by the task and some points in that space may not be reachable. \nThe python tool for the analysis of the task space animates the motion of the robot's end-effector by computing a trajectory with help of the inverse kinematic. ")
		doc = generate_taskspace_text(doc, spec_sheet_yml)
		with doc.create(Subsection("Defined Minimum and Maximum Position of Task Space Motion")):
			doc.append("The minimum and maximum position of the EE can either be defined manually by the user or with a presented csv file. The choosen values are summarized in the following table: ")
			doc = generate_taskspace_table (doc, spec_sheet_yml)
		with doc.create(Subsection("Motion of the robot in the task space")):
			doc.append("As the position of the EE ")
			doc.append(italic("X "))
			doc.append("is defined, the inverse kinematics can be used to calculate the joint variables ")
			doc.append(italic("y: "))
			doc.append(NoEscape(r"$y=f^{-1}(X)$"))
			doc.append(".\nFrom the calculated motion of the robot, plots are generated which show the position, velocity and acceleration of the spanning-tree, independent and actuator joints. The spanning-tree joints are grouped by submechanisms and for the actuator joints, the forces/torques are shown as well. Additionally, the position, velocity and acceleration of the EE are printed separately for each direction x, y and z.")
			cspace_traj_path = os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_results_folder + "/task_space_motion/cspace"
			submechanism_folders = [d for d in os.listdir(cspace_traj_path) if os.path.isdir(os.path.join(cspace_traj_path, d))]
			for submech in submechanism_folders:
				cspace_folder_path_to_submech = os.path.join(cspace_traj_path, submech)
				spanning_tree_images = [os.path.join(cspace_folder_path_to_submech, an) for an in os.listdir(cspace_folder_path_to_submech) if an.endswith(".png")]
				doc = glue_multiple_images(doc, spanning_tree_images, 2, 6, "Position, Velocity and Acceleration of the Robot's Submechanisms " + submech.replace("_", " "), False, True)
			taskspace_folder_path = os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_results_folder + "/task_space_motion"
			EE_task_space_folder = [x for x in os.listdir(taskspace_folder_path) if x.startswith("EE_")][0]
			EE_traj_path = os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_results_folder + "/task_space_motion/" + EE_task_space_folder
			indspace_traj_path = os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_results_folder + "/task_space_motion/indspace"
			actspace_traj_path = os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_results_folder + "/task_space_motion/actspace"
			independent_images = [os.path.join(indspace_traj_path, an) for an in os.listdir(indspace_traj_path) if an.endswith(".png")]
			actuator_images = [os.path.join(actspace_traj_path, an) for an in os.listdir(actspace_traj_path) if an.endswith(".png")]
			EE_images = [os.path.join(EE_traj_path, an) for an in os.listdir(EE_traj_path) if an.endswith(".png")]
			doc = glue_multiple_images(doc, independent_images, 2, 6, "Position, Velocity and Acceleration of the Robot's Independent Joints", False, True)
			doc = glue_multiple_images(doc, actuator_images, 2, 6, "Position, Velocity, Acceleration and Forces of the Robot's Actuator Joints", False, True)
			doc = glue_multiple_images(doc, EE_images, 2, 6, "Position, Velocity and Acceleration of the Robot's analyzed Body Part " + EE_task_space_folder.replace("EE_", "") , False, True)
		doc.generate_tex("taskspace_" + folder_name)


if __name__ == '__main__':

	# Create the parser
	parser = argparse.ArgumentParser(description = "Generate a specification sheet of the robot. This program uses the generated YAML file with information about the robot's joints, workspace and the amount of forces. It also provides visualization in form of pnd files or 2D or 3D plots of the robots and workspace, statics, configuration space, actuation space and independent space. One have to provide two necessary arguments which is firstly the path to the results folder in which all the nformation are stored. Secondly the path to the yaml file of the robot. Additionally the optional argument can be used to define which information should be inserted into the pdf file. \n\nBasic Usage Example of the RH5 leg: \n\npython3 generate_spec_sheet.py /control/hyrodynpy/scripts/results/lower_body /control/hyrodynpy/scripts/results/lower_body/lower_body_data.yml \n\nBasic Usage Example of Recupera right arm: \n\npython3 generate_spec_sheet.py /control/hyrodynpy/scripts/results/_exo_arm_002_001 /control/hyrodynpy/scripts/results/_exo_arm_002_001/_exo_arm_002_001_data.yml \n\nUsage example with optional argument to store only the zero configuration (1), workpace (3) and taskspace analysis (8): \n\npython3 generate_spec_sheet.py /control/hyrodynpy/scripts/results/lower_body /control/hyrodynpy/scripts/results/lower_body/lower_body_data.yml --option_add_information_to_pdf 1 3 8", formatter_class=RawTextHelpFormatter)

	# Add the arguments
	parser.add_argument("results_folder_path", type = str,
			help="Path to the results folder of the robot part which has to be analyzed with respect to AUTOPROJ_CURRENT_ROOT")
	parser.add_argument("yml_file_path", type = str,
			help="Path to the generated yaml file of the robot with respect to AUTOPROJ_CURRENT_ROOT")
	parser.add_argument("--option_add_information_to_pdf", nargs = "+",
			help="Define which information should be included into the generated pdf: \n1: Zero configuration analysis \n2: Swing my robot \n3: Workspace analysis \n4: Static forces in workspace \n5: Configuration space analysis \n6: Actuation space analysis \n7: Independent space analysis \n8: Taskspace motion analysis")

	# Execute the parse_args method
	args, unknown = parser.parse_known_args()

	# Store the variables in the args
	path_to_results_folder = args.results_folder_path
	path_to_yml = args.yml_file_path
	optional_information = args.option_add_information_to_pdf

	# change directory to store the pdf file in the correct folder
	parent_dir = os.environ["AUTOPROJ_CURRENT_ROOT"] +  path_to_results_folder
	directory = "spec_sheet"
	path_spec_sheet = os.path.join(parent_dir, directory)
	if not os.path.exists(path_spec_sheet):
		os.mkdir(path_spec_sheet)
	os.chdir(path_spec_sheet)
	
	# Get the name of the part of the robot which was analyzed and of the yml file
	folder_name = os.path.basename(path_to_results_folder)
	spec_sheet_yml = os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_yml

	# Document with `\maketitle` command activated
	doc = Document(documentclass="article", inputenc="utf8")

	#doc.packages.append(Command(r"\\usepackage{float}"))
	doc.preamble.append(Command("title", "Robot Specification Sheet"))
	doc.preamble.append(Command("author", "Author: Hybrid Robot Dynamics (HyRoDyn)"))
	doc.preamble.append(Command("date", NoEscape(r"\today")))
	doc.append(NoEscape(r"\maketitle"))
	preface_doc(doc)

	if args.option_add_information_to_pdf is None:
		zero_config_doc(doc)
		swing_my_robot_doc(doc)
		workspace_doc(doc)
		statics_doc(doc)
		cspace_doc(doc)
		actspace_doc(doc)
		indspace_doc(doc)
		taskspace_doc(doc)
		print("All analysis results will be inserted into the pdf file.")
	else:
		if "1" in args.option_add_information_to_pdf:
			zero_config_doc(doc)
		if "2" in args.option_add_information_to_pdf:
			swing_my_robot_doc(doc)
		if "3" in args.option_add_information_to_pdf:
			workspace_doc(doc)
		if "4" in args.option_add_information_to_pdf:
			statics_doc(doc)
		if "5" in args.option_add_information_to_pdf:
			cspace_doc(doc)
		if "6" in args.option_add_information_to_pdf:
			actspace_doc(doc)
		if "7" in args.option_add_information_to_pdf:
			indspace_doc(doc)
		if "8" in args.option_add_information_to_pdf:
			taskspace_doc(doc)
		print("Just the selected analysis results " + str(args.option_add_information_to_pdf) + " will be inserted into the pdf file.")

	# Create tex files folder
	list_tex_files = [tex_file for tex_file in os.listdir() if tex_file.endswith(".tex")]
	for filename in list_tex_files:
		section_texfiles = os.path.abspath(filename)
		if "robot_spec_sheet_" not in filename:	
			finaldirectory = "content"
			path_tex_files = os.path.join(path_spec_sheet, finaldirectory)
			if not os.path.exists(path_tex_files):
				os.mkdir(path_tex_files)
			os.rename(section_texfiles, path_tex_files + "/" + filename)

	# Generate the pdf and the tex file
	doc.generate_pdf("robot_spec_sheet_" + folder_name, clean_tex=False)
	tex = doc.dumps()  # The document as string in LaTeX syntax

	os.chdir("../../../")
