#!python3

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
from swing_my_robot import mesh_orientation

def matrix_from_quaternion(qw, qx, qy, qz):
	"""Compute rotation matrix from quaternion.
	This typically results in an active rotation matrix.
	Parameters
	----------
	q : array-like, shape (4,)
	Unit quaternion to represent rotation: (qw, qx, qy, qz)
	Returns
	-------
	R : array-like, shape (3, 3)
	Rotation matrix
	"""
	x2 = 2.0 * qx * qx
	y2 = 2.0 * qy * qy
	z2 = 2.0 * qz * qz
	xy = 2.0 * qx * qy
	xz = 2.0 * qx * qz
	yz = 2.0 * qy * qz
	xw = 2.0 * qx * qw
	yw = 2.0 * qy * qw
	zw = 2.0 * qz * qw

	R = np.array([[1.0 - y2 - z2, xy - zw, xz + yw],
		  [xy + zw, 1.0 - x2 - z2, yz - xw],
		  [xz - yw, yz + xw, 1.0 - x2 - y2]])
	return R

def extract_poses_from_csv (input_file, number_of_points_in_workspace = 100):
	""" Extract the position and orientation information out of the csv file.
	This gives us lists of all variables.
	----- Parameters -----
	input_file : string
	Path to the csv file
	Returns
	x, y, z : lists of the position values in each direction
	qw, qx, qy, qz : lists of the orientation values
	cond1 : list of the condition values which describe how good a point is reachable (independent joint space)
	cond2 : list of the condition values which describe how good a point is reachable (actuation space)
	"""
	data_set = pd.read_csv(input_file, usecols = [0, 1, 2, 3, 4, 5, 6, 7, 8], nrows = number_of_points_in_workspace) # Use the first 7 colums only
	# Get the position information from csv 
	x = data_set["x"]
	y = data_set[" y"]
	z = data_set[" z"]
	# Generate quaternion out of the csv file for create the rotation matrix R
	qw = data_set[" qw"]
	qx = data_set[" qx"]
	qy = data_set[" qy"]
	qz = data_set[" qz"]
	cond1 = data_set[" cond1_inv"]
	cond2 = data_set[" cond2_inv"]
	return x, y, z, qw, qx, qy, qz, cond1, cond2

def workspace_bounding_box (x, y, z, qw, qx, qy, qz):
	""" Creating the workspace out of the position and orientation values.
	This function returns the minimum and maximum reachable points in each direction.
	----- Parameters -----
	x, y, z : lists of the position values in each direction
	qw, qx, qy, qz : lists of the orientation values
	Returns
	min_x, min_y, min_z : minimum reachable points in x, y, z
	max_x, max_y, max_z : maximum reachable points in x, y, z
	"""
	min_x = min(x)
	min_y = min(y)
	min_z = min(z)
	max_x = max(x)
	max_y = max(y)
	max_z = max(z)

	return min_x, min_y, min_z, max_x, max_y, max_z

def center_of_bounding_box (min_x, min_y, min_z, max_x, max_y, max_z):
	"""Returns the center of a bounding box.
	Returns
	x, y, z, coordinates of the center of the bounding box
	"""
	return (min_x+max_x)/2, (min_y+max_y)/2, (min_z+max_z)/2

def dimensions_of_bounding_box (min_x, min_y, min_z, max_x, max_y, max_z):
	"""Returns the dimension of the bounding box.
	----- Parameters -----
	min_x, min_y, min_z : minimum reachable points in x, y, z
	max_x, max_y, max_z : maximum reachable points in x, y, z
	Returns
	heigth, width and length of the bounding box
	"""
	h = abs(max_x - min_x) 
	w = abs(max_y - min_y) 
	l = abs(max_z - min_z) 
	return h,w,l

def volume_of_bounding_box_of_workspace (h,w,l):
	"""Calculate the Volume of the bounding box.
	Returns
	The Volume of the bounding box of the workspace	
	"""
	return h * w * l

def extract_SE3_poses (x, y, z, qw, qx, qy, qz):
	"""Extract the SE3 poses for each point of the csv file.
	This results in a homogeneous 4x4 transformation matrix.
	----- Parameters -----
	x, y, z : lists of the position values in each direction
	qw, qx, qy, qz : lists of the orientation values
	Returns
	tfs : array-like, shape (4, 4)
	Transformation matrix
	"""
	N = len(x) # Number of workspace points
	tfs = np.tile(np.eye(4), (N, 1, 1)) # we are creating two poses here

	# Create a transformation matrix for each row in the csv file
	for i in range(N):
		tf = np.eye(4)
		# Store everything in the transformation matrix
		tf[0,3] = x[i]
		tf[1,3] = y[i]
		tf[2,3] = z[i]
		tf[0:3, 0:3] = matrix_from_quaternion(qw[i], qx[i], qy[i], qz[i])
		tfs[i,:,:] = tf
	return tfs

def add_sphere_mesh_in_scene (scene, poses, cond = None):
	"""Add the mesh of every sphere of the workspace points in the scene.
	----- Parameters -----
	scene : scene of the robot and the workspace
	poses : poses of the sphere
	cond : condition value which describe how good a point is reachable
	Returns
	One scene with all reachable points in the workspace added
	"""	
	if cond is None:
		sm = trimesh.creation.uv_sphere (radius=0.01)
		alpha = 1.0 # opaquness
		sm.visual.vertex_colors = [1.0, 1.0, 1.0, alpha] # RGBA
		point_cloud_mesh = pyrender.Mesh.from_trimesh(sm, poses=poses)
		scene.add(point_cloud_mesh)
		'''
		box_mesh = trimesh.creation.box (point_cloud_mesh.extents)
		pose = np.eye(4)
		pose[0,3] = point_cloud_mesh.centroid[0]
		pose[1,3] = point_cloud_mesh.centroid[1]
		pose[2,3] = point_cloud_mesh.centroid[2]
		box_mesh.visual.vertex_colors = [1.0, 1.0, 1.0, 0.1] # RGBA
		scene.add(pyrender.Mesh.from_trimesh(box_mesh, poses = pose))
		'''
	else:
		sm = trimesh.creation.uv_sphere (radius=0.01)
		color_range = np.array([[255, 0, 0, 128],[0, 255, 0, 128]],dtype=np.uint8)	
		for i in range (len(cond)):
			ball_color = trimesh.visual.color.linear_color_map (cond[i], color_range)
			sm.visual.vertex_colors = ball_color.tolist()[0]
			m = pyrender.Mesh.from_trimesh (sm, poses = poses[i])
			scene.add(m)
	return scene

def add_bounding_box_in_scene(scene, h, w, l, x, y, z):
	"""Add the bounding box of the workspace to the scene.
	----- Parametes -----
	scene : the scene of the robot with the spheres
	h, w, l : height, width and length of the box
	x, y, z : position values
	Returns
	scene : A new scene where the bounding box is added
	"""
	box_mesh = trimesh.creation.box ([h, w, l])
	pose = np.eye(4)
	pose[0,3] = x
	pose[1,3] = y
	pose[2,3] = z
	box_mesh.visual.vertex_colors = [1.0, 1.0, 1.0, 0.2] # RGBA
	scene.add(pyrender.Mesh.from_trimesh(box_mesh, poses = pose))
	return scene

def NormalizeData(data):
	"""Normalize the list of numbers to values between 0.0 and 1.0"""
	return (data - min(data)) / (max(data) - min(data))

def save_workspace_image(scene, camera_view_dict, camera_type = "orthographic"):
	"""Save the images of the workspace.
	----- Parameters -----
	scene : the scene of the robot with the spheres
	camera_view_dict : Define the camera poses 
	with the keys of this dictionary (left, right, front and back)
	camera_type : specifies the type of the camera, 
	which is either orthografic or perspective
	Returns
	An image of the workspace as a png
	"""
	if camera_type == "perspective":
		# Set up the Perspective camera -- z-axis away from the scene, x-axis right, y-axis up
		cam = pyrender.PerspectiveCamera(yfov=np.pi / 3.0)
	else:
		# Setup the orthographic camera (default)
		cam = pyrender.OrthographicCamera(xmag=1.0, ymag=1.0)

	for key in camera_view_dict:
		print(key, camera_view_dict[key])
		# Switch between oc and pc here to change the camera and its poses
		camera_node = scene.add(cam, pose = camera_view_dict[key])

		# Setup the light
		light = pyrender.DirectionalLight(color=[1,1,1], intensity=1e3)
		light_node = scene.add(light, pose = camera_view_dict[key])

		#if camera_type == "perspective":
			#v = pyrender.Viewer(scene)

		# render scene
		r = pyrender.OffscreenRenderer(512,512)
		color, _ = r.render(scene)

		# Show the images 
		plt.figure()
		plt.axis('off')
		plt.imshow(color)
		#key_title = key.replace("_", " ").capitalize()
		#plt.title(key_title)  
		plt.savefig(key, dpi = 300)  
		#plt.show()

		# Clean up
		r.delete() # delete the renderer
		scene.remove_node(camera_node) # delete the camera from the scene
		scene.remove_node(light_node) # delete the light from the scene	
	
def return_camera_poses(scene_bounds_x, scene_bounds_y, scene_bounds_z):
	"""Return the camera views to take the workspace images.
	Returns
	camera_view_dict : Dictionary of the camera poses where the key is the name of the view (str) and value is the 4x4 homogenous transformation matrix describing the camera pose. 
	"""
	# Views when looking along the global X axis
	camera_pose_back_view = np.array([
	       [1.0, 0.0, 0.0, 0.0],
	       [0.0, 0.0, -1.0, -scene_bounds_y],
	       [0.0, 1.0, 0.0, 0.0],
	       [0.0,  0.0, 0.0, 1.0],
	    ])
	camera_pose_front_view = np.array([
	       [-1.0, 0.0, 0.0, 0.0],
	       [0.0, 0.0, 1.0, scene_bounds_y],
	       [0.0, 1.0, 0.0, 0.0],
	       [0.0,  0.0, 0.0, 1.0],
	    ])
	# Views when looking along the global Y axis
	camera_pose_right_view = np.array([
	       [0.0, 0.0, 1.0, scene_bounds_x],
	       [1.0, 0.0, 0.0, 0.0],
	       [0.0, 1.0, 0.0, 0.0],
	       [0.0,  0.0, 0.0, 1.0],
	    ])
	camera_pose_left_view = np.array([
	       [0.0, 0.0, -1.0, -scene_bounds_x],
	       [-1.0, 0.0, 0.0, 0.0],
	       [0.0, 1.0, 0.0, 0.0],
	       [0.0,  0.0, 0.0, 1.0],
	    ])
	# Views when looking along the global Z axis
	camera_pose_top_view = np.array([
	       [1.0, 0.0, 0.0, 0.0],
	       [0.0, 1.0, 0.0, 0.0],
	       [0.0, 0.0, 1.0, scene_bounds_z],
	       [0.0,  0.0, 0.0, 1.0],
	    ])
	camera_pose_bottom_view = np.array([
	       [1.0, 0.0, 0.0, 0.0],
	       [0.0, -1.0, 0.0, 0.0],
	       [0.0, 0.0, -1.0, -scene_bounds_z],
	       [0.0,  0.0, 0.0, 1.0],
	    ])
	# We should make the camera pose adjustable to the size of the robot. Idea: find a bounding box for the robot and measure its L,B,H. Adjust the distance of the camera from the robot according to these dimensions.
	camera_view_dict = {"left_view":camera_pose_left_view, "right_view":camera_pose_right_view, "front_view":camera_pose_front_view, "back_view":camera_pose_back_view, "top_view":camera_pose_top_view, "bottom_view": camera_pose_bottom_view}
	return camera_view_dict

def create_scene_with_robot(urdfpy_obj):
	"""Return the pyrender scene with the robot visualization in zero configuration
	----- Parameters -----
	urdfpy_obj : urdfpy object created with urdf robot model
	Returns
	scene : Dictionary of the camera poses where the key is the name of the view (str) and value is the 4x4 homogenous transformation matrix describing the camera pose. 
	"""
	# Create the pyrender scene
	scene = pyrender.Scene()

	# Show the robot in the zero config
	fk = urdfpy_obj.visual_trimesh_fk()	# add the possibility here to show robot in any given configuration
	for tm in fk:
		pose = fk[tm]
		mesh = pyrender.Mesh.from_trimesh(tm, smooth=False)
		scene.add (mesh, pose=pose)
	return scene

def main (path_to_urdf, path_to_submechamisms, path_to_jointlimits, body_name, number_of_points_in_workspace=100, dont_show_workspace_bounding_box=True, dont_show_conditioning_in_workspace=True, show_conditioning_independent_joint_space=True, num_discretization_steps_per_joint=5, fix_mesh_orientation=False):
	"""Return the print-outs as a dictionary
	----- Parameters -----
	path_to_urdf, path_to_submechamisms, path_to_jointlimits : strings
	pathes to the urdf, submechanism and jointlimits
	body_name : string
	name of the body part which will be visualized
	number_of_points_in_workspace : integer, optional argument
	number of points shown in the workspace, by default it is set to 100
	dont_show_workspace_bounding_box, dont_show_conditioning_in_workspace, show_conditioning_independent_joint_space : optional arguments
	by default they are all set to True
	Returns
	dict_bounding_box : dictionary
	dictionary which includes the main information of the bounding box
	"""

	# Path to URDF file
	# path_to_urdf = urdf_path 
	#path_to_urdf =  "/control/hyrodyn/data/hybrid/recupera/right_arm_teleop/urdf/right_arm_teleop.urdf" 

	# Path to Submechanism.yml file 
	# path_to_submechamisms = submechanism_yml

	# Path to joint limits file
	# path_to_jointlimits = joint_limits_yml

	# body_name = body_name

	# Switch to build folder to run the C++ program
	os.chdir("../../hyrodyn/applications/robot_analysis/build")
	# options available in hyrodyn_analyze_robot C++ program: [1: CSPACE, 2: ACTSPACE, 3: STATICS, 4: WORKSPACE, 5: INDSPACE, 6: ALL, Any other integer: All joint space analyses (no workspace)]
	os.system("./hyrodyn_analyze_robot" + " --filepath_urdf " + path_to_urdf  + " --filepath_submechanisms " + path_to_submechamisms + " --filepath_jointlimits " + path_to_jointlimits + " --num_steps " + str(num_discretization_steps_per_joint) + " --option " + str(4) + " --body_name " + body_name)
	# Switch back to python folder so that the results are stored here
	os.chdir("../../../../hyrodynpy/scripts")

	abs_path_to_urdf = os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_urdf
	# Load the robot in urdfpy
	viz = URDF.load(abs_path_to_urdf)

	# Add fix mesh orientation
	if fix_mesh_orientation:
        	mesh_orientation(viz)

	# Add the robot in zero config to the scene
	scene = create_scene_with_robot(viz)

	# Load the position and orientation coordinates of the end-effector
	urdf_file_name_with_ext = os.path.basename(abs_path_to_urdf)
	urdf_file_name = os.path.splitext(urdf_file_name_with_ext)[0]

	#input_file = os.environ ["AUTOPROJ_CURRENT_ROOT"] + "/control/hyrodyn/applications/robot_analysis/results/right_arm_teleop_forwardkinematics_x.csv"
	input_file = os.environ ["AUTOPROJ_CURRENT_ROOT"] + "/control/hyrodyn/applications/robot_analysis/results/" + urdf_file_name + "_forwardkinematics_x.csv"
	print("Number of points used in workspace visualization: ", number_of_points_in_workspace)
	x, y, z, qw, qx, qy, qz, cond1, cond2 = extract_poses_from_csv (input_file, number_of_points_in_workspace)
	poses = extract_SE3_poses (x, y, z, qw, qx, qy, qz)

	# Normalize the condition number values to [0,1]
	cond1_normalized = NormalizeData(cond1)
	cond2_normalized = NormalizeData(cond2)

	# Extract the bounding box
	min_x, min_y, min_z, max_x, max_y, max_z = workspace_bounding_box (x, y, z, qw, qx, qy, qz)
	h,w,l = dimensions_of_bounding_box (min_x, min_y, min_z, max_x, max_y, max_z)
	V = volume_of_bounding_box_of_workspace (h,w,l)
	x,y,z = center_of_bounding_box (min_x, min_y, min_z, max_x, max_y, max_z)

	print("Minimum values in each direction: ", min_x, min_y, min_z)
	print("Maximum values in each direction: ", max_x, max_y, max_z)
	print("Volume of the bounding box of the workspace: ", V)
	print("Center of workspace bounding box: ", x, y, z)
	print("Dimensions (H,W,L) of workspace bounding box: ", h, w, l)

	if dont_show_conditioning_in_workspace is True:
		print("Conditioning of the workspace is deactivated and will NOT be shown.")
		scene = add_sphere_mesh_in_scene (scene, poses)	# use this when you do not want colored spheres (much faster because all of the point cloud is a single mesh)
	else:
		print("Conditioning of the workspace is activated and will be shown.")
		if show_conditioning_independent_joint_space is True:
			print("Conditioning in independent joint space will be used.")
			scene = add_sphere_mesh_in_scene (scene, poses, cond1_normalized) # workspace with conditioning in independent joint space (slow as each point is a separate mesh in the point cloud)
		else:
			print("Conditioning in actuation space (default) will be used.")
			scene = add_sphere_mesh_in_scene (scene, poses, cond2_normalized) # workspace with conditioning in actuation space (slow as each point is a separate mesh in the point cloud)

	# Start the workspace visualizer
	if dont_show_workspace_bounding_box is False:
		print("A bounding box of the workspace will be added to the scene.")
		scene = add_bounding_box_in_scene(scene, h, w, l, x, y, z)
	else:
		print("A bounding box of the workspace will NOT be added to the scene.")

	# v = pyrender.Viewer(scene, use_raymond_lighting=True)

	# Prepare camera views for orthographic camera
	print("scene bounds", scene.bounds)
	scene_bounds = np.abs(np.diff(scene.bounds, axis = 0)).flatten()
	print("scene bounds diff", scene_bounds)
	scene_bounds_x = scene_bounds[0]
	scene_bounds_y = scene_bounds[1]
	scene_bounds_z = scene_bounds[2]

	# camera view dictionary for orthographic camera
	camera_view_dict_orthographic = return_camera_poses(scene_bounds_x, scene_bounds_y, scene_bounds_z)
	
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
	finaldirectory = "workspace"
	finalpath = os.path.join(subpath, finaldirectory)
	if not os.path.exists(finalpath):
		os.mkdir(finalpath)
	os.chdir(finalpath)

	save_workspace_image(scene, camera_view_dict_orthographic)

	# Prepare camera views for perspective camera
	s = np.sqrt(2)/2
	camera_pose = np.array([
	       [0.0, -s,   s,   1.0],
	       [1.0,  0.0, 0.0, 0.0],
	       [0.0,  s,   s,   1.0],
	       [0.0,  0.0, 0.0, 1.0],
	    ])
	camera_view_dict_perspective = {"perspective": camera_pose}

	save_workspace_image(scene, camera_view_dict_perspective, "perspective")
	os.chdir("../../../")


	# Create dictionary
	dict_bounding_box = {"Centre of the bounding box": {"x": x, "y": y, "z": z}, "Dimensions": {"height": h, "width": w, "length": l}, "Minimum dimensions": {"min_x": min_x, "min_y": min_y, "min_z": 		min_z}, "Maximum dimensions": {"max_x": max_x, "max_y": max_y, "max_z": max_z}, "Volume": V}
	return dict_bounding_box

if __name__ == '__main__':
	# Create the parser
	parser = argparse.ArgumentParser(description = "Generate a workspace visualization using HyRoDyn where all reachable points are shown as optionally colorful spheres and the whole workspace can be equipped with an optional boundary box. The color of the speres represent if this point is easy to reach (green) or not (red). Robot model in HyRoDyn is built using the URDF and submechanisms yaml file. The program extracts the coordinates of the workspace points from csv file generated by hyrodyn_analyze_robot C++ program (make sure it is available by building it) provided in applications/robot_analysis folder. The joint limits described in a yaml file (in RoCK convention) is used to discretize the input joint space. The program also exports the visualization of the workspace into four png files by default which can be turned off on demand. The four png files show the robot and it's workspace from left, right, top and front view.\n\nBasic Usage Example: \n\npython3 workspace_visualization.py /control/hyrodyn/data/hybrid/rh5/leg/urdf/leg.urdf /control/hyrodyn/data/hybrid/rh5/leg/urdf/submechanisms.yml /control/hyrodyn/data/hybrid/rh5/leg/urdf/joint_limits.yml LLAnklePitch_Link", formatter_class=RawTextHelpFormatter)

	# Add the arguments
	parser.add_argument("urdf_path", type = str,
		            help="URDF path for the robot with respect to AUTOPROJ_CURRENT_ROOT")
	parser.add_argument("submechanism_yml", type = str, 
		            help="YAML file describing the submechanisms with respect to AUTOPROJ_CURRENT_ROOT")
	parser.add_argument("joint_limits_yml", type = str,
		            help="YAML file of describing the joint limits with respect to AUTOPROJ_CURRENT_ROOT")
	parser.add_argument("body_name", type = str,
		            help="Name of the part of the robot's body which should be visualized")
	parser.add_argument("--dont_show_conditioning_in_workspace", action='store_true', 
		            help="Do not show conditioning of the workspace (no colored spheres)")
	parser.add_argument("--dont_show_workspace_bounding_box", action='store_true', 
		            help="Do not show the workspace bounding box")
	parser.add_argument("--show_conditioning_independent_joint_space", action='store_true', 
		            help="Use the conditioning information only in independent joint space (caution: this is not the true condition that is experienced by the actuator control)")
	parser.add_argument("--number_of_points_in_workspace", type = int, default = 100, 
		            help="Limit the number of points in the workspace visualization (defaults to 100)")
	parser.add_argument("--num_discretization_steps_per_joint", type = int, default = 5,
	                    help="Number of discretization steps per joint (defaults to 5 which means for a 6 joint robot 5^6 = 15625 discretized points in the input joint space)")
	parser.add_argument("--fix_mesh_orientation", action='store_true', help="This can fix some orientation issues with some OBJ meshes")

	# Execute the parse_args method
	args, unknown = parser.parse_known_args()
	
	# Store the variables in the args
	path_to_urdf = args.urdf_path
	path_to_submechamisms = args.submechanism_yml
	path_to_jointlimits = args.joint_limits_yml
	body_name = args.body_name
	number_of_points_in_workspace = args.number_of_points_in_workspace
	dont_show_workspace_bounding_box = args.dont_show_workspace_bounding_box
	dont_show_conditioning_in_workspace = args.dont_show_conditioning_in_workspace
	show_conditioning_independent_joint_space = args.show_conditioning_independent_joint_space
	num_discretization_steps_per_joint = args.num_discretization_steps_per_joint
	fix_mesh_orientation = args.fix_mesh_orientation

	# Call the main function
	main(path_to_urdf, path_to_submechamisms, path_to_jointlimits, body_name, number_of_points_in_workspace, dont_show_workspace_bounding_box, dont_show_conditioning_in_workspace, show_conditioning_independent_joint_space, num_discretization_steps_per_joint, fix_mesh_orientation=fix_mesh_orientation)


