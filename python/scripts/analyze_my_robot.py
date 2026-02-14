from urdfpy import URDF 
import os 
import argparse
from argparse import RawTextHelpFormatter
import yaml

import workspace_visualization as workviz
import static_forces_in_workspace_visualization as statforce
import swing_my_robot as swingrob
import analyze_zero_config as zeroconfig
import cspace_visualization as cspaceviz
import actspace_visualization as actspaceviz
import indspace_visualization as indspaceviz
import analyze_task_space_motion as taskmotion

# Create the parser
parser = argparse.ArgumentParser(description = "Analyze the robot using the HyRoDyn framework. Robot model in HyRoDyn is built using the URDF and submechanisms yaml file. this program generates a YAML file with information about the robot's joints, workspace and the amount of forces. It also provides visualization and animation od the robot and it's workspace. One have to provide four necessary arguments which are the urdf, submechanism and joint limits path as well as the body name of the part of the robot's body which should be visualized. \n\nBasic Usage Example of the RH5 leg: \n\npython3 analyze_my_robot.py /control/hyrodyn/data/hybrid/rh5/leg/urdf/leg.urdf /control/hyrodyn/data/hybrid/rh5/leg/urdf/submechanisms.yml /control/hyrodyn/data/hybrid/rh5/leg/urdf/joint_limits.yml LLAnklePitch_Link \n\nBasic Usage Example of Recupera right arm: \n\npython3 analyze_my_robot.py /control/hyrodyn/data/hybrid/recupera/right_arm_teleop/urdf/right_arm_teleop.urdf /control/hyrodyn/data/hybrid/recupera/right_arm_teleop/urdf/submechanisms.yml /control/hyrodyn/data/hybrid/recupera/right_arm_teleop/urdf/joint_limits.yml right_exo_elbow", formatter_class=RawTextHelpFormatter)

# Add the arguments
parser.add_argument("urdf_path", type = str,
		help="URDF path for the robot with respect to AUTOPROJ_CURRENT_ROOT")
parser.add_argument("submechanism_yml", type = str, 
		help="YAML file describing the submechanisms with respect to AUTOPROJ_CURRENT_ROOT")
parser.add_argument("joint_limits_yml", type = str,
		help="YAML file of describing the joint limits with respect to AUTOPROJ_CURRENT_ROOT")
parser.add_argument("body_name", type = str,
		help="Name of the part of the robot's body which should be visualized")
parser.add_argument("--num_discretization_steps_per_joint", type = int, default = 5,
		help="Number of discretization steps per joint (defaults to 5 which means for a 6 joint robot 5^6 = 15625 discretized points in the input joint space)")
parser.add_argument("--fix_mesh_orientation", action='store_true', help="This can fix some orientation issues with some OBJ meshes")

# Execute the parse_args method
args, unknown = parser.parse_known_args()

# get the dictionaries
zeroconfig = zeroconfig.main(args.urdf_path, args.submechanism_yml, args.body_name)
swingrob_dict = swingrob.main (args.urdf_path, args.submechanism_yml, args.joint_limits_yml, fix_mesh_orientation=args.fix_mesh_orientation)
workviz_dict = workviz.main (args.urdf_path, args.submechanism_yml, args.joint_limits_yml, args.body_name, num_discretization_steps_per_joint=args.num_discretization_steps_per_joint, fix_mesh_orientation=args.fix_mesh_orientation)
statforce_dict = statforce.main (args.urdf_path, args.submechanism_yml, args.joint_limits_yml, num_discretization_steps_per_joint=args.num_discretization_steps_per_joint, fix_mesh_orientation=args.fix_mesh_orientation)
cspace_dict = cspaceviz.main(args.urdf_path, args.submechanism_yml, args.joint_limits_yml, args.num_discretization_steps_per_joint)
actspace_dict = actspaceviz.main(args.urdf_path, args.submechanism_yml, args.joint_limits_yml, args.num_discretization_steps_per_joint)
indspace_dict = indspaceviz.main(args.urdf_path, args.submechanism_yml, args.joint_limits_yml, args.num_discretization_steps_per_joint)
taskspace_dict = taskmotion.main(args.urdf_path, args.submechanism_yml, fix_mesh_orientation=args.fix_mesh_orientation)

abs_path_to_urdf = os.environ["AUTOPROJ_CURRENT_ROOT"] + args.urdf_path
# Load the robot in urdfpy
viz = URDF.load(abs_path_to_urdf)

# change directory to store the yaml file in the correct folder
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

# Create a whole dictionary with all information and dump it to a yaml file
robot_analyze_dict = {"Analysis of the robot's zero configuration": zeroconfig, "Joint limits poses": swingrob_dict, "Workspace bounding box": workviz_dict, "Minimum and maximum forces": statforce_dict, "Task Space Motion": taskspace_dict, "Minimum and maximum position of submechanism in different spaces": {"Minimum and maximum position in configuration space": cspace_dict, "Minimum and maximum position in actuator space": actspace_dict, "Minimum and maximum position in independent space": indspace_dict}}
with open(str(viz.name)+"_data.yml", "w") as outfile:
	yaml.dump(robot_analyze_dict, outfile, default_flow_style=False)

os.chdir("../../")




