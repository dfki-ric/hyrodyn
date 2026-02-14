import pandas as pd
import numpy as np 
import os 
from urdfpy import URDF 
import matplotlib.pyplot as plt
import swing_my_robot as smr
import argparse
from argparse import RawTextHelpFormatter

def main (path_to_urdf, animation_csv, show_plot = False, no_animation = False):
	"""Animates the robot based on a given configuration trajectory in configuration space
	----- Parameters -----
	path_to_urdf, animation_csv : strings
	"""

	abs_path_to_urdf = os.environ["AUTOPROJ_CURRENT_ROOT"] + path_to_urdf

	# Load the robot in urdfpy
	viz = URDF.load(abs_path_to_urdf)

	# input file where the cspace is stored
	input_file = os.environ ["AUTOPROJ_CURRENT_ROOT"] + animation_csv

	# read the csv file
	config_traj = pd.read_csv(input_file)
	config_keys = list(config_traj)
	print(config_keys)

	spanningtree_jointnames = smr.extract_joint_names(viz)

	# Create a configuration trajectory dict for all the spanning tree joints
	keys = spanningtree_jointnames
	values = []
	for key in keys:
		values.append(config_traj["q_"+key])
	config_trajectory = dict(zip(keys, values))

	# Add the time to the configuration trajectory dictionary - needed for swinging animation
	config_trajectory["time"] = config_traj["time"]

	if no_animation is False:
		smr.animate_custom(viz, config_trajectory, False, 1)

	if show_plot is True:
		plt.figure
		plt.plot(config_traj["time"], config_traj.loc[:, config_traj.columns != 'time'])
		plt.legend(spanningtree_jointnames)
		plt.show()

if __name__ == '__main__':	

	# Create the parser
	parser = argparse.ArgumentParser(description = "Animate the robot with configuration space trajectory provided in csv file. \n\nBasic Usage Example: \n\npython3 animate_my_robot.py /control/hyrodyn/data/serial/pendulum/urdf/pendulum.urdf /control/hyrodyn/build/src/animation.csv", formatter_class=RawTextHelpFormatter)

	# Add the arguments
	parser.add_argument("urdf_path", type = str,
	                    help="URDF path for the robot with respect to AUTOPROJ_CURRENT_ROOT")
	parser.add_argument("animation_csv", type = str, 
	                    help="CSV file describing the configuration trajectory with respect to AUTOPROJ_CURRENT_ROOT")
	parser.add_argument("--show_plot", action='store_true', 
	                    help="whether to show the plot of cspace trajectory")
	parser.add_argument("--no_animation", action='store_true', 
	                    help="whether to skip the animation")

	# Execute the parse_args method
	args, unknown = parser.parse_known_args()

	# Call the main function
	main (args.urdf_path, args.animation_csv, args.show_plot, args.no_animation)

