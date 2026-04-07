import hyrodyn as hyrodyn 
import numpy as np 

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
	path_to_urdf =  "../robot/leg/urdf/leg.urdf" 
	path_to_submechamisms = "../robot/leg/urdf/submechanisms.yml"

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
	
	# Compute the COM properties of the robot in zero configuration
	robot.calculate_com_properties()
	print("Total moving mass in zero config = ",robot.mass)
	print("COM in zero config = ",robot.com)

	# Excite/Move the Robot Independent Joints
	robot.y = np.zeros(independent_joints_dof)
	print("Input joint config = ",robot.y)

	# Compute full state of the spanning tree
	robot.calculate_system_state()
	print("Spanning tree joint positions (all the joints in the robot): ", robot.Q)
	print("Actuator joint positions (subset of spanning tree joints): ", robot.u)

	# Forward Kinematics
	body_name = "LLAnklePitch_Link"
	robot.calculate_forward_kinematics(body_name)
	print("Forward kinematics of the body " + body_name + " ([X Y Z Qx Qy Qz Qw]):", robot.pose)

	# Forward Kinematics for multiple bodies
	body_names = ["LLKnee_Link", "LLAnklePitch_Link"]
	robot.calculate_forward_kinematics_multiple_bodies(body_names)
	print("Forward kinematics of the body " + body_names[0] + " ([X Y Z Qx Qy Qz Qw]):", robot.poses[0])
	print("Forward kinematics of the body " + body_names[1] + " ([X Y Z Qx Qy Qz Qw]):", robot.poses[1])

	# Inverse Kinematics
	body_names = ["LLAnklePitch_Link"]
	robot.pose_input = [robot.pose]
	robot.calculate_inverse_kinematics(body_names)
	print("Inverse Kinematics output joint config = ",robot.y)

	# Inverse dynamics
	robot.calculate_inverse_dynamics()
	print("Inverse dynamics output: ", robot.Tau_actuated)

	# Forward dynamics
	robot.calculate_forward_dynamics()
	print("Forward dynamics output: ", robot.ydd)

	# Mass-inertia matrix in actuation space
	robot.calculate_mass_interia_matrix_actuation_space()
	print("Mass-inertia matrix in actuation space (Hu):\n", robot.Hu)

	# Nonlinear effects (Coriolis + gravity) in actuation space
	robot.calculate_nle_actuation_space()
	print("Nonlinear effects in actuation space (Cu): ", robot.Cu)

if __name__ == '__main__':
	# Call the main function
	main()

