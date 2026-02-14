import hyrodyn_py as hyrodyn 
import time 
import threading
import numpy as np 
import os 
import robomeshcat as rmc
# from urdfpy import URDF 

def put_sorted_list_into_bins(sorted_list, bins):
	""" Put sorted list into bins of various sizes
	---- Parameters ---- 
	sorted_list : list 
	Names of the joints in a certain space (e.g. independent joint space, spanning tree joint space, actuation space)
	bins: list 
	DOF of each submechanism in a certain space (e.g. independent joint space, spanning tree joint space, actuation space)
	---- Returns ---- 
	output : returns sorted list of joint names into bins of submechanisms """
	sorted_list_into_bins = []
	ind_start = 0
	for bin in bins:
		ind_end = ind_start + bin
		sorted_list_into_bins.append(sorted_list[ind_start:ind_end])
		ind_start = ind_end

	return sorted_list_into_bins

def motion_function(pose1, pose2, robot, scene, rob):
	for j in range(10):
		for i in range(100):
			# Excite/Move the Robot Independent Joints
			ratio = i / (100 - 1)
			robot.y = (1 - ratio) * pose1 + ratio * pose2
			robot.calculate_system_state()

			for joint_name, joint_value in zip(robot.jointnames_spanningtree, robot.Q):
				rob[joint_name] = np.array(joint_value)
			# rob[:] = np.array(robot.Q)
			scene.render()
			time.sleep(0.02)
		pose1, pose2 = pose2, pose1 # Swap poses for next iteration
	return


def main ():
	"""Return the print-outs as a dictionary
	----- Parameters -----
	active_joints, independent_joints, spanning_tree_joints : lists 
	lists of the robot's joints
	Returns
	dict_jointnames : dictionary
	dictionary of the joints of the robot
	"""
	# Path to URDF and Submechanism.yml files 
	# path_to_urdf =  "/home/dfki.uni-bremen.de/rkumar/hyrodyn_dev/hyrodyn/data/hybrid/rh5/leg/urdf/leg.urdf" 
	# path_to_submechamisms = "/home/dfki.uni-bremen.de/rkumar/hyrodyn_dev/hyrodyn/data/hybrid/rh5/leg/urdf/submechanisms.yml"

	path_to_urdf =  "data/mj_hyrodyn/submechanisms/singular_parallelogram_3.urdf"
	path_to_submechamisms =  "data/mj_hyrodyn/submechanisms/submechanisms.yml"
	# Load the robot model in HyRoDyn
	robot = hyrodyn.RobotModel(path_to_urdf, path_to_submechamisms)

	# Information of different joints in the robot
	active_joints = robot.jointnames_active
	independent_joints = robot.jointnames_independent
	spanning_tree_joints = robot.jointnames_spanningtree
	print("Names of active joints: ", active_joints)
	print("Names of independent joints: ", independent_joints)
	print("Names of spanning tree joints: ", spanning_tree_joints)

	# DOF in different spaces
	print("DOF of active joints: ", robot.active_dof)
	print("DOF of independent joints: ", robot.independent_dof)
	print("DOF of spanning tree joints: ", robot.spanningtree_dof)

	print("Submechanism active dof distribution: ", robot.submechanism_active_dof_distribution)
	print("Submechanism independent dof distribution: ", robot.submechanism_independent_dof_distribution)
	print("Submechanism spanning tree dof distribution: ", robot.submechanism_spanningtree_dof_distribution)

	spanning_tree_joints_grouped_via_submechanisms = put_sorted_list_into_bins(spanning_tree_joints, robot.submechanism_spanningtree_dof_distribution)
	print("Spanning tree joints grouped via submechanisms", spanning_tree_joints_grouped_via_submechanisms)
	independent_joints_grouped_via_submechanisms = put_sorted_list_into_bins(independent_joints, robot.submechanism_independent_dof_distribution)
	print("Independent joints grouped via submechanisms", independent_joints_grouped_via_submechanisms)
	active_joints_grouped_via_submechanisms = put_sorted_list_into_bins(active_joints, robot.submechanism_active_dof_distribution)
	print("Active joints grouped via submechanisms", active_joints_grouped_via_submechanisms)

	# active_dof  = len(active_joints)
	independent_joints_dof = len(independent_joints)
	# spanning_tree_joints_dof = len(spanning_tree_joints)
	# dict_jointnames_dof = {"Active joints": {"Names": active_joints, "DOF": active_dof} , "Independent joints": {"Names": independent_joints, "DOF": independent_joints_dof}, "Spanning tree joints": {"Names": spanning_tree_joints, "DOF": spanning_tree_joints_dof}}
	
	# Compute the COM properties of the robot in zero configuration
	robot.calculate_com_properties()
	print("Total moving mass in zero config = ",robot.mass)
	print("COM in zero config = ",robot.com)

	# Excite/Move the Robot Independent Joints
	# robot.y = np.array([0.0, 0.0, -0.15, 0.3, 0.0, -0.15])
	robot.y = np.zeros(independent_joints_dof)
	print("Input joint config = ",robot.y)

	# Compute full state of the spanning tree
	robot.calculate_system_state()
	print("Spanning tree joint positions (all the joints in the robot): ", robot.Q)
	print("Actuator joint positions (subset of spanning tree joints): ", robot.u)

	# # Forward Kinematics
	# body_name = "LLAnklePitch_Link"
	# robot.calculate_forward_kinematics(body_name)
	# print("Forward kinematics of the body " + body_name + " ([X Y Z Qx Qy Qz Qw]):", robot.pose)

	# # Forward Kinematics for multiple bodies
	# body_names = ["LLKnee_Link", "LLAnklePitch_Link"]
	# robot.calculate_forward_kinematics_multiple_bodies(body_names)
	# print("Forward kinematics of the body " + body_names[0] + " ([X Y Z Qx Qy Qz Qw]):", robot.poses[0])
	# print("Forward kinematics of the body " + body_names[1] + " ([X Y Z Qx Qy Qz Qw]):", robot.poses[1])

	# # Inverse Kinematics
	# body_names = ["LLAnklePitch_Link"]
	# robot.pose_input = [robot.pose]
	# robot.calculate_inverse_kinematics(body_names)
	# print("Inverse Kinematics output joint config = ",robot.y)

	# Inverse dynamics
	robot.calculate_inverse_dynamics()
	print("Inverse dynamics output: ", robot.Tau_actuated)

	# Forward dynamics
	robot.calculate_forward_dynamics()
	print("Forward dynamics output: ", robot.ydd)

	# Visualize the robot
	# keys = robot.jointnames_spanningtree
	# values = robot.Q[0].tolist()
	# config_dict = dict(zip(keys, [values]))
	# print(config_dict)

	rob = rmc.Robot(urdf_path=path_to_urdf)
	scene = rmc.Scene()
	scene.add_robot(rob)
	scene.render()
	# print(robot.Q)


	# pose1 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
	# pose2 = np.array([0.0, 0.0, -0.4, 0.8, 0.0, -0.4])
	# # motion_thread = threading.Thread(target=motion_function, args=(pose1, pose2, robot, scene, rob))
	# # motion_thread.start()
	# # motion_thread.join()

	# for j in range(10):
	# 	for i in range(100):
	# 		# Excite/Move the Robot Independent Joints
	# 		ratio = i / (100 - 1)
	# 		robot.y = (1 - ratio) * pose1 + ratio * pose2
	# 		robot.calculate_system_state()

	# 		for joint_name, joint_value in zip(robot.jointnames_spanningtree, robot.Q):
	# 			rob[joint_name] = np.array(joint_value)
	# 		# rob[:] = np.array(robot.Q)
	# 		scene.render()
	# 		time.sleep(0.01)
	# 	pose1, pose2 = pose2, pose1 # Swap poses for next iteration
	
	# return 

if __name__ == '__main__':
	# Call the main function
	main()

