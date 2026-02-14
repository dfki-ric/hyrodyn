#!python3

import argparse
import os
import time
import yaml
from argparse import RawTextHelpFormatter
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import hyrodyn
import numpy as np
import urdfpy


# Function for create a single joint sinusoidal trajectory
def sinusoidal_single_joint_traj(q_min, q_max, Tf, dt):
    """ Generate a sinusoidal trajectory for one single joint such that the joint moves from zero to a given minimum joint position and back to zero position, and then again from zero to maximum joint position and back to the zero position in a total of Tf seconds in dt time steps.
        ---- Parameters ----
        q_min : float
        The minimum limit of the joint which can be reached during moving.
        q_max : float
        The maximum limit of the joint which can be reached during moving.
        Tf : float
        The total time for the motion given in seconds.
        dt : float
        The timesteps. On each timestep the simulation will be integrated.
        output : returns the generated trajectory for a single joint. """
    N = int(Tf / dt)
    time = np.linspace(0, int(Tf / 2), int(N / 2))
    theta = np.append(q_min * np.sin(np.pi * time / 2), q_max * np.sin(np.pi * time / 2))
    thetadot = np.append(q_min * (np.pi/2) * np.cos(np.pi * time / 2), q_max * (np.pi/2) * np.cos(np.pi * time / 2))
    thetaddot = np.append(q_min * (np.pi**2/4) * -np.sin(np.pi * time / 2), q_max * (np.pi**2/4) * -np.sin(np.pi * time / 2))
    return theta, thetadot, thetaddot

# Function for create a single joint cycloidal trajectory
def cycloidal_single_joint_traj(q_min, q_max, Tf, dt):
    """ Generate a cycloidal trajectory for one single joint such that the joint moves from zero to a given minimum joint position and back to zero position, and then again from zero to maximum joint position and back to the zero position in a total of Tf seconds in dt time steps.
        ---- Parameters ----
        q_min : float
        The minimum limit of the joint which can be reached during moving.
        q_max : float
        The maximum limit of the joint which can be reached during moving.
        Tf : float
        The total time for the motion given in seconds.
        dt : float
        The timesteps. On each timestep the simulation will be integrated.
        output : returns the generated trajectory for a single joint. """
    N = int(Tf / dt)
    time = np.linspace(0, int(Tf), int(N / 3))
    theta1 = (0 + ((q_min - 0) / Tf) * (time - (Tf / (2 * np.pi)) * np.sin(((2 * np.pi) / Tf) * time)))
    theta2 = (q_min + ((q_max - q_min) / Tf) * (time - (Tf / (2 * np.pi)) * np.sin(((2 * np.pi) / Tf) * time)))
    theta3 = (q_max + ((0 - q_max) / Tf) * (time - (Tf / (2 * np.pi)) * np.sin(((2 * np.pi) / Tf) * time)))
    theta1d = (((q_min - 0) / Tf) * (1 - np.cos(((2 * np.pi) / Tf) * time)))
    theta2d = (((q_max - q_min) / Tf) * (1 - np.cos(((2 * np.pi) / Tf) * time)))
    theta3d = (((0 - q_max) / Tf) * (1 - np.cos(((2 * np.pi) / Tf) * time)))
    theta1dd = (((q_max - q_min) / Tf) * (((2 * np.pi) / Tf) * np.sin(((2 * np.pi) / Tf) * time)))
    theta2dd = (((q_max - q_min) / Tf) * (((2 * np.pi) / Tf) * np.sin(((2 * np.pi) / Tf) * time)))
    theta3dd = (((q_max - q_min) / Tf) * (((2 * np.pi) / Tf) * np.sin(((2 * np.pi) / Tf) * time)))
    theta = np.append(np.append(theta1,theta2),theta3)
    thetadot = np.append(np.append(theta1d,theta2d),theta3d)
    thetaddot = np.append(np.append(theta1dd,theta2dd),theta3dd)
    if len(theta)<N:
        theta = np.concatenate([theta, np.zeros(N-len(theta))])
        thetadot = np.concatenate([thetadot, np.zeros(N-len(thetadot))])
        thetaddot = np.concatenate([thetaddot, np.zeros(N-len(thetaddot))])
    return theta, thetadot, thetaddot

# Function for create a single joint cubic polynomial trajectory
def polynomial_single_joint_traj(q_min, q_max, Tf, dt):
    """ Generate a cubic polynomial trajectory for one single joint such that the joint moves from zero to a given minimum joint position and back to zero position, and then again from zero to maximum joint position and back to the zero position in a total of Tf seconds in dt time steps.
        ---- Parameters ----
        q_min : float
        The minimum limit of the joint which can be reached during moving.
        q_max : float
        The maximum limit of the joint which can be reached during moving.
        Tf : float
        The total time for the motion given in seconds.
        dt : float
        The timesteps. On each timestep the simulation will be integrated.
        output : returns the generated trajectory for a single joint. """
    """"
    N = int(Tf / dt)
    time = np.linspace(0, int(Tf / 2), int(N / 2))
    theta = np.append((3 / (Tf / 2)**2) * q_min * time**2 - (2 / (Tf / 2)**3) * q_min * time**3, (3 / (Tf / 2)**2) * q_max * time**2 - (2 / (Tf / 2)**3) * q_max * time**3)
    thetadot = np.append((6 / (Tf / 2)**2) * q_min * time - (6 / (Tf / 2)**3) * q_min * time**2, (6 / (Tf / 2)**2) * q_max * time - (6 / (Tf / 2)**3) * q_max * time**2)
    thetaddot = np.append((6 / (Tf / 2)**2) * q_min - (12 / (Tf / 2)**3) * q_min * time, (6 / (Tf / 2)**2) * q_max - (12 / (Tf / 2)**3) * q_max * time)
    return theta, thetadot, thetaddot"""
    N = int(Tf / dt)
    time = np.linspace(0, int(Tf), int(N / 3))
    theta1 = ((3 / Tf**2) * (q_min-0) * time**2 - (2 / Tf**3) * (q_min-0) * time**3)
    theta2 = (q_min + (3 / Tf**2) * (q_max-q_min) * time**2 - (2 / Tf**3) * (q_max-q_min) * time**3)
    theta3 = (q_max + (3 / Tf**2) * (0-q_max) * time**2 - (2 / Tf**3) * (0-q_max) * time**3)
    thetadot1 = ((6 / Tf**2) * (q_min-0) * time - (6 / Tf**3) * (q_min-0) * time**2)
    thetadot2 = ((6 / Tf**2) * (q_max-q_min) * time - (6 / Tf**3) * (q_max-q_min) * time**2)
    thatadot3 = ((6 / Tf**2) * (0-q_max) * time - (6 / Tf**3) * (0-q_max) * time**2)
    thetaddot1 = ((6 / Tf**2) * (q_min-0) - (12 / Tf**3) * (q_min-0) * time)
    thetaddot2 = ((6 / Tf**2) * (q_max-q_min) - (12 / Tf**3) * (q_max-q_min) * time)
    thetaddot3 = ((6 / Tf**2) * (0-q_max) - (12 / Tf**3) * (0-q_max) * time)
    theta = np.append(np.append(theta1,theta2),theta3)
    thetadot = np.append(np.append(thetadot1,thetadot2),thatadot3)
    thetaddot = np.append(np.append(thetaddot1,thetaddot2),thetaddot3)
    if len(theta)<N:
        theta = np.concatenate([theta, np.zeros(N-len(theta))])
        thetadot = np.concatenate([thetadot, np.zeros(N-len(thetadot))])
        thetaddot = np.concatenate([thetaddot, np.zeros(N-len(thetaddot))])
    return theta, thetadot, thetaddot

# Generate trajectory for the whole robot configuration
def robot_traj(min_pos, max_pos, Tf, dt, independent_joint_names, single_joint_traj):
    """ Create the joint swinging trajectory for the whole robot.
    ---- Parameters ----
    min_pos : list of float
    The minimum limit of the joint which can be reached during moving.
    max_pos : list of float
    The maximum limit of the joint which can be reached during moving.
    Tf : float
    The total time for the motion of a single joint given in seconds.
    dt : float
    The timesteps. On each timestep the simulation will be integrated.
    independent_joint_names : list of string
    The names of the independent joints of the robot are stored here.
    output : returns the joint swinging trajectory of the whole robot as numpy array. """
    N = int(Tf / dt)
    robottraj_pos = []
    robottraj_vel = []
    robottraj_acc = []
    for joint1 in independent_joint_names:
        swingjointtraj_pos = []
        swingjointtraj_vel = []
        swingjointtraj_acc = []
        for joint2 in independent_joint_names:
            if joint1 == joint2:
                swingjointindex = independent_joint_names.index(joint2)
                start = min_pos[swingjointindex]
                end = max_pos[swingjointindex]
                singlejointtraj_pos, singlejointtraj_vel, singlejointtraj_acc = single_joint_traj(start, end, Tf, dt)
                swingjointtraj_pos.append(singlejointtraj_pos)
                swingjointtraj_vel.append(singlejointtraj_vel)
                swingjointtraj_acc.append(singlejointtraj_acc)
            else:
                swingjointtraj_pos.append(np.zeros(N))
                swingjointtraj_vel.append(np.zeros(N))
                swingjointtraj_acc.append(np.zeros(N))
        swingjointtraj_pos_array = np.array(swingjointtraj_pos).T
        swingjointtraj_vel_array = np.array(swingjointtraj_vel).T
        swingjointtraj_acc_array = np.array(swingjointtraj_acc).T
        robottraj_pos.append(swingjointtraj_pos_array.tolist())
        robottraj_vel.append(swingjointtraj_vel_array.tolist())
        robottraj_acc.append(swingjointtraj_acc_array.tolist())
    array_pos = np.array(robottraj_pos)
    array_vel = np.array(robottraj_vel)
    array_acc = np.array(robottraj_acc)
    robottraj_pos = np.vstack(array_pos)
    robottraj_vel = np.vstack(array_vel)
    robottraj_acc = np.vstack(array_acc)
    return robottraj_pos, robottraj_vel, robottraj_acc

# Function for extracting the joint names from URDF
def extract_joint_names(urdfpy_obj):
    """ Extract the movable joint names from urdfpy model
    ---- Parameters ----
    urdfpy_obj : object of URDFPY class
    ---- Returns ----
    output : returns list of string containing the names of all the movable joints (fixed and mimic joints are excluded) in the urdf file for which joint limits were defined. """
    joint_names = []
    for joint in urdfpy_obj.actuated_joints:
        jnt = urdfpy_obj.joint_map[joint.name]
        # 20210127 MS: Added checks if joint limits are available
        if jnt.limit is None:
            continue
        # 20210127 MS: For continous joints, we create limits
        if jnt.joint_type == 'continuous':
            jnt.limit.lower = -3.141
            jnt.limit.upper = 3.141
        # 20210127 MS: Added checks if joint limits are available
        if jnt.limit.lower is None:
            continue
        if jnt.limit.upper is None:
            continue
        joint_names.append(joint.name)
    return joint_names


# Extract the joint limits from URDF via urdfpy object
def extract_joint_limits(urdfpy_obj, jointnames):
    """ Extract the joint limits from URDF via urdfpy object
    ---- Parameters ----
    urdfpy_obj : object of URDFPY class
    jointnames : string
    Names of the joints in the urdf file
    output : returns two lists, one for the minimum and one for the maximum positions of the joints """
    min_pos = []
    max_pos = []
    for jointname in jointnames:
        jnt = urdfpy_obj.joint_map[jointname]
        # 20210127 MS: Added checks if joint is in joint_limit_cfgs
        if jnt.limit is None:
            continue
        if jnt.limit.lower is None:
            continue
        if jnt.limit.upper is None:
            continue
        min_pos.append(jnt.limit.lower)
        max_pos.append(jnt.limit.upper)
    if len(min_pos) != len(jointnames) or len(max_pos) != len(jointnames):
        min_pos = []
        max_pos = []
    return min_pos, max_pos


# Extract the joint limits from YAML file describing the joint limits
def extract_joint_limits_yml(path_to_jointlimits, jointnames):
    """ Extract the joint limits from YAML file describing the joint limits
    ---- Parameters ----
    path_to_jointlimits : path to the yaml file describing the joint limits (RoCK convention is used)
    jointnames : list of string
    Names of the joints in the urdf file for which you want to extract the joint limits
    output : returns two lists, one for the minimum and one for the maximum positions of the joints """
    # extract the joint limits from the yaml
    import yaml
    with open(path_to_jointlimits) as joint_limits_yml:
        # Load the yaml file
        joint_limits = yaml.load(joint_limits_yml, Loader=yaml.FullLoader)
        # Extract limits dict
        limits = joint_limits["limits"]
        # Extract names dict
        joint_names_in_yaml = limits["names"]
        # Extract elements dict
        joint_elements = limits["elements"]

        min_pos_in_yaml = []
        max_pos_in_yaml = []
        for ijoint in joint_elements:
            min_pos_in_yaml.append(ijoint["min"]["position"])
            max_pos_in_yaml.append(ijoint["max"]["position"])

    # Check if independent joint names in the hyrodyn robot model is the same as joints in the joint limits yaml file
    print("jointnames: ", jointnames)
    print("joint elements: ", joint_elements)
    print("joint names in yml: ", joint_names_in_yaml)
    min_pos = []
    max_pos = []
    if (len(jointnames) == len(joint_elements)) or (len(jointnames) == len(joint_names_in_yaml)):
        # The code below ensures that min and max joint limits lists in the yaml dont need to follow the same sequence as robot's joint names sequence.
        for name in jointnames:
            index = joint_names_in_yaml.index(name)
            min_pos.append(min_pos_in_yaml[index])
            max_pos.append(max_pos_in_yaml[index])
    else:
        assert (len(joint_elements) == len(joint_names_in_yaml)), "The joint names of the HyRoDyn file and the joint limits yaml file are not the same."
        independent_joint_limits = []
        min_pos_independent = []
        max_pos_independent = []
        for jointname in jointnames:
            if jointname in joint_names_in_yaml:
                index = joint_names_in_yaml.index(jointname)
                independent_joint_limits.append(joint_elements[index])
                min_pos.append(min_pos_in_yaml[index])
                max_pos.append(max_pos_in_yaml[index])
    return min_pos, max_pos


# Custom animation function to plot time based configuration trajectories using urdfpy
def animate_custom(self, cfg_trajectory=None, record=True, loop_count=1, use_collision=False, outfile=None):
    """Animate the robot through a configuration trajectory using urdfpy.
    Parameters
    ----------
    self : urdfpy object
    cfg_trajectory : dict or (m,n) float
        A map from joints or joint names to lists of configuration values
        for each joint along the trajectory, or a vector of
        vectors where the second dimension contains a value for each joint.
        If not specified, all joints will articulate from limit to limit.
        The trajectory steps are assumed to be equally spaced out in time.
    record : bool
        If True, the animation will be recorded and exported as a GIF file.
    loop_count : int
        Number of times the animation should loop
    use_collision : bool
        If True, the collision geom--trajectory_generationetry is visualized instead of
        the visual geometry.
    """

    # Save pyrender import for here for CI
    import pyrender

    # Create an array of time steps
    times = cfg_trajectory["time"]  # extract the time vector from the configuration trajectory
    cfg_trajectory.pop("time", None)  # delete the time information from the configuration trajectory dictionary

    # Create the scene
    if use_collision:
        fk = self.collision_trimesh_fk()
    else:
        fk = self.visual_trimesh_fk()

    node_map = {}
    scene = pyrender.Scene()
    for tm in fk:
        pose = fk[tm]
        mesh = pyrender.Mesh.from_trimesh(tm, smooth=False)
        node = scene.add(mesh, pose=pose)
        node_map[tm] = node

    # Get base pose to focus on
    blp = self.link_fk(links=[self.base_link])[self.base_link]

    # Pop the visualizer asynchronously
    v = pyrender.Viewer(scene, run_in_thread=True,
                        use_raymond_lighting=True,
                        record=record,
                        view_center=blp[:3, 3])

    # Now, run our loop
    i = 0
    j = 0
    while v.is_active and j < loop_count:
        cfg = {k: cfg_trajectory[k][i] for k in cfg_trajectory}

        if i < len(times) - 1:
            dt = times[i + 1] - times[i]  # extract the time step from time vector
        i = (i + 1) % len(times)

        if i == len(times) - 1:
            j = j + 1

        if use_collision:
            fk = self.collision_trimesh_fk(cfg=cfg)
        else:
            fk = self.visual_trimesh_fk(cfg=cfg)

        v.render_lock.acquire()
        for mesh in fk:
            pose = fk[mesh]
            node_map[mesh].matrix = pose
        v.render_lock.release()

        time.sleep(dt)

    # Close the animation viewer
    v.close_external()

    # Export the recorded animation
    if outfile is None:
        outfile = "swinging_animation.gif"
        print("Saving animation to", os.path.join(os.getcwd(), outfile))
    if record is True:
        v.save_gif(outfile)

def generate_csv_file(joint_space, time_series, jointnames, position, velocity, acceleration, force = []):
    """Generate csv files for the independent, active and spanning-tree joints
    ----- Parameters -----
    joint_space : string
    the name of the used joint space
    time_series : array
    array of the animation time
    jointnames : string 
    names of the joints
    position, velocity, acceleration : floats
    floats of the current position, velocity and acceleration of the robot's joint
    force : list
    by default the list is empty
    Returns
    csv_file : csv file
    csv file with the position, velocity and acceleration of the independent, active and spanning-tree joints
    """
    
    index = time_series
    filename = joint_space + "_joints_traj.csv"
    if joint_space == "independent":
        columns_position = ["y_" + name for name in jointnames] 
        columns_velocity = ["yd_" + name for name in jointnames] 
        columns_acceleration = ["ydd_" + name for name in jointnames]
    elif joint_space == "active":
        columns_position = ["u_" + name for name in jointnames] 
        columns_velocity = ["ud_" + name for name in jointnames] 
        columns_acceleration = ["udd_" + name for name in jointnames] 
    elif joint_space == "spanning_tree":
        columns_position = ["q_" + name for name in jointnames] 
        columns_velocity = ["qd_" + name for name in jointnames] 
        columns_acceleration = ["qdd_" + name for name in jointnames]
    else:
        print("Unknown joint space!")

    if len(force)!=0: 
        columns_force = ["Tau_" + name for name in jointnames]
        columns = columns_position + columns_velocity + columns_acceleration + columns_force
    else:
        columns = columns_position + columns_velocity + columns_acceleration

    pos = np.array(position)
    vel = np.array(velocity)
    acc = np.array(acceleration)
    if len(force)!=0:
        tau = np.array(force)
        data = np.concatenate((pos, vel, acc, tau), axis = 1)
    else:
        data = np.concatenate((pos, vel, acc), axis = 1)

    df = pd.DataFrame(data, index, columns)
    df.index.name = "Time (s)"
    csv_file = df.to_csv(filename, sep=',',index=True)

    return csv_file

def plot_traj_span(names_submechanisms, list_submech_joints, name_csv_file, name_filter, label_y, title):
    """Plot the trajectories of the spanning tree joint's position, velocity and acceleration grouped by submechanisms
    ----- Parameters -----
    names_submechanisms : list
    list of strings of the submechanisms names
    list_submech_joints : list
    lists with lists of the joints according grouped by the submechanisms
    name_csv_file : string
    name of the csv file which has to be loaded
    name_filter : string
    part of the column names which have to be filtered
    label_y : string
    label of the y-axis
    title : string
    title of the created plot
    Returns
    plots for position, velocity and acceleration of the spanning-tree joints for each submechanism
    """
    df = pd.read_csv(name_csv_file)
    list_filtered_submech_df = []
    for sublist in list_submech_joints:
        submech_df = df[[col for col in df if col.endswith(tuple(sublist))]]
        filtered_columns = [col for col in submech_df if col.startswith(name_filter)]
        filtered_df = df[filtered_columns]
        list_filtered_submech_df.append(filtered_df)                     
    for filtered_submech_df, i in zip(list_filtered_submech_df, names_submechanisms):
        plt.figure()
        for column in filtered_submech_df:
            filtered_submech_df.plot()
            plt.legend(bbox_to_anchor=(0,-0.08,1,-0.08), loc="upper center", mode="expand", borderaxespad=0, ncol = 2)
            plt.xlabel("Time")
            plt.ylabel(label_y)
            plt.title(title + "\n _submechanism_" + i)
            plt.savefig(title + "_submechanism_" + i, bbox_inches="tight") 
            plt.close()

def plot_traj(name_csv_file, name_filter, label_y, title):
    """Plot the trajectories of the joint's position, velocity and acceleration
    ----- Parameters -----
    name_csv_file : string
    name of the csv file which has to be loaded
    name_filter : string
    part of the column names which have to be filtered
    label_y : string
    label of the y-axis
    title : string
    title of the created plot
    Returns
    plots for position, velocity and acceleration of the joints 
    """
    df = pd.read_csv(name_csv_file)
    filtered_df = df[[col for col in df if col.startswith(name_filter)]]
    plt.figure()
    for column in filtered_df:
        filtered_df.plot()
        plt.legend(bbox_to_anchor=(0,-0.08,1,-0.08), loc="upper center", mode="expand", borderaxespad=0, ncol = 2)
        plt.xlabel("Time")
        plt.ylabel(label_y)
        plt.title(title)
        plt.savefig(title, bbox_inches="tight") 
        plt.close()

def mesh_orientation(self):
    import math
    print("Fixing mesh orientations: ", end='')
    for link in self.links:
        for visual in link.visuals:
            for mesh in visual.geometry.meshes:
                print(".", end='')
                mesh.apply_transform(urdfpy.xyz_rpy_to_matrix((0.0,0.0,0.0,math.pi / 2.0,0.0,0.0)))
    print(" DONE")


def main(urdf_path, submechanism_yml, joint_limits_yml, trajectory_generation, number_of_loops=1, export_animation=True, 
         export_animation_as_mp4=False, outfile=None, fix_mesh_orientation=False):
    """Return the print-outs as a dictionary
    ----- Parameters -----
    path_to_urdf, path_to_submechamisms, path_to_jointlimits : strings
    pathes to the urdf, submechanism and jointlimits
    body_name : string
    number_of_loops : integer, optional argument
    number of times the animation should loop
    export_animation : optional argument
    by default it is set to False
    export_animation_as_mp4 : optional argument
    by default it is set to True
    Returns
    dict_min_max_pos : dictionary
    dictionary of the min and max poses of the independent joints used for animation
    """
    # Print out what is given or not
    # print("Parsed arguments: {}".format(args))
    # print("joint_limits_yml in namespace?: {}".format("joint_limits_yml" in args))

    # Path to URDF
    if not os.path.isabs(urdf_path):
        if not os.path.isfile(urdf_path):
            path_to_urdf = os.path.join(os.environ["AUTOPROJ_CURRENT_ROOT"], urdf_path)
        else:
            path_to_urdf = os.path.abspath(urdf_path)
    else:
        if not os.path.isfile(urdf_path):
            path_to_urdf = os.environ["AUTOPROJ_CURRENT_ROOT"] + urdf_path
        else:
            path_to_urdf = os.path.abspath(urdf_path)

    # Path to Submechanism.yml file
    if submechanism_yml:
        if not os.path.isabs(submechanism_yml):
            if not os.path.isfile(submechanism_yml):
                path_to_submechamisms = os.path.join(os.environ["AUTOPROJ_CURRENT_ROOT"], submechanism_yml)
            else:
                path_to_submechamisms = os.path.abspath(submechanism_yml)
        else:
            if not os.path.isfile(submechanism_yml):
                path_to_submechamisms = os.environ["AUTOPROJ_CURRENT_ROOT"] + submechanism_yml
            else:
                path_to_submechamisms = os.path.abspath(submechanism_yml)
    else:
        print("No submechanisms yml file was provided. The program will treat the robot as an open chain and "
              "no kinematic loop will be respected.")
        path_to_submechamisms = ""

    # Path to joint limits file
    if joint_limits_yml:
        if not os.path.isabs(joint_limits_yml):
            if not os.path.isfile(joint_limits_yml):
                path_to_jointlimits = os.path.join(os.environ["AUTOPROJ_CURRENT_ROOT"], joint_limits_yml)
            else:
                path_to_jointlimits = os.path.abspath(joint_limits_yml)
        else:
            if not os.path.isfile(joint_limits_yml):
                path_to_jointlimits = os.environ["AUTOPROJ_CURRENT_ROOT"] + joint_limits_yml
            else:
                path_to_jointlimits = os.path.abspath(joint_limits_yml)
    else:
        path_to_jointlimits = ""

    # Load the robot model in URDFPY
    print("Loading URDF:", path_to_urdf)
    viz = urdfpy.URDF.load(path_to_urdf)

    if fix_mesh_orientation:
        mesh_orientation(viz)    

    """
    if fix_mesh_orientation:
        import math
        print("Fixing mesh orientations: ", end='')
        for link in viz.links:
            for visual in link.visuals:
                for mesh in visual.geometry.meshes:
                    print(".", end='')
                    mesh.apply_transform(urdfpy.xyz_rpy_to_matrix((0.0,0.0,0.0,math.pi / 2.0,0.0,0.0)))
        print(" DONE")"""

    if submechanism_yml:
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
        # Information of different joints in the robot
        print("Names of active joints: ", robot.jointnames_active)
        print("Names of independent joints: ", robot.jointnames_independent)
        print("Names of spanning tree joints: ", robot.jointnames_spanningtree)
        print("Names of submechanisms:", names_submechanisms)
        independent_jointnames = robot.jointnames_independent
        spanningtree_jointnames = robot.jointnames_spanningtree
        active_jointnames = robot.jointnames_active

    else:
        # We do not build a model in hyrodyn and rely purely on the open chain urdfpy model
        jointnames = extract_joint_names(viz)
        print("Joint names extracted from URDF which will be actuated (excludes fixed joints and mimic joints): ",
              jointnames)
        independent_jointnames = jointnames
        spanningtree_jointnames = jointnames
        active_jointnames = jointnames

    # 20210127 MS: Added missing check on array
    if not independent_jointnames:
        print("Could not find any independent joints to actuate")
        return

    # if- else statement for read out the joint limits from either from the joint_limits yaml file or from the urdf or
    # select specific values if the both before are not available
    if os.path.isfile(path_to_jointlimits):
        print("Joint limits will be taken from the joint limits YAML file")
        min_pos, max_pos = extract_joint_limits_yml(path_to_jointlimits, independent_jointnames)
    elif os.path.isfile(path_to_urdf):
        print("Joint limits will be taken from the URDF file")
        min_pos, max_pos = extract_joint_limits(viz, independent_jointnames)
        if min_pos == [] or max_pos == []:
            print(
                "Joint limits in urdf are not properly available, we select -0.5 and 0.5 as the min and max value for "
                "every joint.")
            # run the animation with -0.5 and 0.5 as joint limits
            min_pos = [-0.5] * len(independent_jointnames)
            max_pos = [0.5] * len(independent_jointnames)

    # Create a configuration trajectory dict for all the spanning tree joints
    keys = spanningtree_jointnames
    values = []
    for key in keys:
        values.append([])
    config_trajectory = dict(zip(keys, values))

    # Build the robot swinging trajectory
    Tf = 4
    dt = 0.01
    N = int(Tf / dt)
    independent_dof = len(independent_jointnames)
    time_vec = np.linspace(0, Tf, independent_dof * N)
    print("min_pos used in animation: ", min_pos)
    print("max_pos used in animation: ", max_pos)

    # Defining the used trajectory generation
    if args.trajectory_generation is None:
        robottraj_pos, robottraj_vel, robottraj_acc = robot_traj(min_pos, max_pos, Tf, dt, independent_jointnames, sinusoidal_single_joint_traj)
        print("No trajectory generation is defined. The sinusoidal trajectory will be used by default.")
    else:
        if "sinusoidal" in args.trajectory_generation:
            robottraj_pos, robottraj_vel, robottraj_acc = robot_traj(min_pos, max_pos, Tf, dt, independent_jointnames, sinusoidal_single_joint_traj)
            print("The sinusoidal trajectory will be used for the joints movement of the robot.")
        if "cycloidal" in args.trajectory_generation:
            robottraj_pos, robottraj_vel, robottraj_acc = robot_traj(min_pos, max_pos, Tf, dt, independent_jointnames, cycloidal_single_joint_traj)
            print("The cycloidal trajectory will be used for the joints movement of the robot.")
        if "polynomial" in args.trajectory_generation:
            robottraj_pos, robottraj_vel, robottraj_acc = robot_traj(min_pos, max_pos, Tf, dt, independent_jointnames, polynomial_single_joint_traj)
            print("The polynomial trajectory will be used for the joints movement of the robot.")

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
    finaldirectory = "swing_my_robot"
    finalpath = os.path.join(subpath, finaldirectory)
    if not os.path.exists(finalpath):
        os.mkdir(finalpath)
    os.chdir(finalpath)
    
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
    
    if submechanism_yml:
        # Solve the full system state for robot swinging trajectory in the independent joint space
        for i in range(len(robottraj_pos)):
            # Excite/Move the Robot Independent Joints
            robot.y = robottraj_pos[i]
            robot.yd = robottraj_vel[i]
            robot.ydd = robottraj_acc[i]
            log_y.append(robot.y.flatten())
            log_yd.append(robot.yd.flatten())
            log_ydd.append(robot.ydd.flatten())
            # Compute full state of the spanning tree
            robot.calculate_system_state()
            # Compute inverse dynamics
            robot.calculate_inverse_dynamics()
            log_q.append(robot.Q.flatten())
            log_qd.append(robot.QDot.flatten())
            log_qdd.append(robot.QDDot.flatten())
            log_u.append(robot.u.flatten())
            log_ud.append(robot.ud.flatten())
            log_udd.append(robot.udd.flatten())
            log_tau.append(robot.Tau_actuated.flatten())
            for j in range(len(spanningtree_jointnames)):
                config_trajectory[keys[j]].append(robot.Q[0][j])
    else:
        for i in range(len(robottraj_pos)):
            for j in range(len(spanningtree_jointnames)):
                config_trajectory[keys[j]].append(robottraj_pos[i][j])

    # Log files and plots are generated only when hyrodyn model is used i.e. a submechanism file was provided
    if submechanism_yml:
        # Create subfolder for indspace and change directory
        finaldirectory = "indspace"
        indtrajpath = os.path.join(finalpath, finaldirectory)
        if not os.path.exists(indtrajpath):
            os.mkdir(indtrajpath)
        os.chdir(indtrajpath)
        # Generate csv file for independent joint space trajectory
        generate_csv_file("independent", time_vec, independent_jointnames, log_y, log_yd, log_ydd)
        # Plot independent joint pos, vel and acc
        plot_traj("independent_joints_traj.csv", "y_", "Independent Joint Position", "positions_independent_joints")
        plot_traj("independent_joints_traj.csv", "yd_", "Independent Joint Velocity", "velocity_independent_joints")
        plot_traj("independent_joints_traj.csv", "ydd_", "Independent Joint Acceleration", "acceleration_independent_joints")
        os.chdir("../")

        # Create subfolder for actspace and change directory
        finaldirectory = "actspace"
        acttrajpath = os.path.join(finalpath, finaldirectory)
        if not os.path.exists(acttrajpath):
            os.mkdir(acttrajpath)
        os.chdir(acttrajpath)
        # Generate csv file for independent joint space trajectory
        generate_csv_file("active", time_vec, active_jointnames, log_u, log_ud, log_udd, log_tau)
        # Plot actuator joint pos, vel, acc and forces
        plot_traj("active_joints_traj.csv", "u_", "Actuator Joint Position", "positions_actuator_joints")
        plot_traj("active_joints_traj.csv", "ud_", "Actuator Joint Velocity", "velocity_actuator_joints")
        plot_traj("active_joints_traj.csv", "udd_", "Actuator Joint Acceleration", "acceleration_actuator_joints")
        plot_traj("active_joints_traj.csv", "Tau_", "Actuator Joint Forces", "actuator_joint_forces")
        os.chdir("../")

        # Create subfolders for cspace and for each submechanism and change directory
        finaldirectory = "cspace"
        spantrajpath = os.path.join(finalpath, finaldirectory)
        if not os.path.exists(spantrajpath):
            os.mkdir(spantrajpath)
        os.chdir(spantrajpath)
        # Generate csv file for spanning tree joint space trajectory
        generate_csv_file("spanning_tree", time_vec, spanningtree_jointnames, log_q, log_qd, log_qdd)
        # Plot actuator joint pos, vel and acc grouped by submechanisms
        plot_traj_span(names_submechanisms, list_submech_joints, "spanning_tree_joints_traj.csv", "q_", "Spanning-Tree Joint Position", "positions_spanning_tree_joints")
        plot_traj_span(names_submechanisms, list_submech_joints, "spanning_tree_joints_traj.csv", "qd_", "Spanning-Tree Joint Velocity", "velocity_spanning_tree_joints")
        plot_traj_span(names_submechanisms, list_submech_joints, "spanning_tree_joints_traj.csv", "qdd_","Spanning-Tree Joint Acceleration", "acceleration_spanning_tree_joints")
        list_plot_traj = [plot for plot in os.listdir() if plot.endswith(".png")]
        print("list_plot_traj:", list_plot_traj)
        for i in range(len(names_submechanisms)):
            finaldirectory = names_submechanisms[i]
            temppath = os.path.join(spantrajpath, finaldirectory)
            if not os.path.exists(temppath):
                os.mkdir(temppath)
        submech_folders = [submech.path for submech in os.scandir(spantrajpath) if submech.is_dir()]
        for filename in list_plot_traj:
            plot_path = os.path.abspath(filename)
            print("path to one file:", plot_path)
            for submech_name in submech_folders:
                if str(os.path.basename(submech_name)) in filename:
                    os.rename(plot_path, os.path.basename(submech_name) + "/" + filename)
        
        os.chdir("../../../../")

    # Add the time to the configuration trajectory dictionary - needed for swinging animation
    config_trajectory["time"] = time_vec

    print("The swinging animation will be looped " + str(number_of_loops) + " times.")

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
    finaldirectory = "swing_my_robot"
    finalpath = os.path.join(subpath, finaldirectory)
    if not os.path.exists(finalpath):
        os.mkdir(finalpath)
    os.chdir(finalpath)
    # Create robot visualization
    if export_animation is True:
        print("Robot Swinging Trajectory will be exported as GIF to", outfile)
        # by default, the function will record the animation for loop count = 1 and export it as gif.
        animate_custom(viz, config_trajectory, True, number_of_loops, outfile=outfile[:-3]+"gif" if outfile is not None else None)
        if export_animation_as_mp4 is True:
            print("Robot Swinging Trajectory will be additionally exported as MP4 to",
                  outfile[:-3]+"mp4" if outfile is not None else "swinging_animation.gif")
            if outfile is None:
                os.system("ffmpeg -f gif -i swinging_animation.gif -movflags faststart -pix_fmt yuv420p -vf 'scale=trunc(iw/2)*2:trunc(ih/2)*2' swinging_animation.mp4")
            else:
                os.system("ffmpeg -f gif -i "+outfile[:-3]+"gif -movflags faststart -pix_fmt yuv420p -vf 'scale=trunc(iw/2)*2:trunc(ih/2)*2' "+outfile[:-3]+"mp4")
    else:
        print("Robot Swinging Trajectory will NOT be exported as GIF.")
        animate_custom(viz, config_trajectory, False, number_of_loops)
    os.chdir("../../../")

    # Create dictionary
    dict_min_max_pos = {a: {"min_poses": b, "max_poses": c} for a, b, c in
                        zip(independent_jointnames, min_pos, max_pos)}
    return dict_min_max_pos


if __name__ == '__main__':

    # Create the parser
    parser = argparse.ArgumentParser(
        description="Generate a swing my robot animation using HyRoDyn where all joints are sequentially moved between "
                    "the min and max values of the joint limits from their zero configuration using sinusoidal "
                    "trajectories. Robot model in HyRoDyn is built using the URDF and submechanisms yaml file. By "
                    "default, the program extracts the joint limits from the URDF file. One can additionally provide "
                    "joint limits described in a yaml file (in RoCK convention) which will be used if provided. "
                    "The program also exports the animation into a GIF file by default which can be turned off on "
                    "demand. If you do not provide a submechanisms file, the robot is treated as an open chain "
                    "mechanism. \n\nBasic Usage Example: \n\npython3 swing_my_robot.py "
                    "/control/hyrodyn/data/hybrid/recupera/right_arm_teleop/urdf/right_arm_teleop.urdf "
                    "--submechanism_yml /control/hyrodyn/data/hybrid/recupera/right_arm_teleop/urdf/submechanisms.yml "
                    "--joint_limits_yml /control/hyrodyn/data/hybrid/recupera/right_arm_teleop/urdf/joint_limits.yml"
                    "\n\nIn case you want to deactivate the animation export: \n\npython3 swing_my_robot.py "
                    "/control/hyrodyn/data/hybrid/recupera/right_arm_teleop/urdf/right_arm_teleop.urdf "
                    "--submechanism_yml /control/hyrodyn/data/hybrid/recupera/right_arm_teleop/urdf/submechanisms.yml " 
                    "--joint_limits_yml /control/hyrodyn/data/hybrid/recupera/right_arm_teleop/urdf/joint_limits.yml "
                    "--do_not_export_animation\n\nIn case you want to loop through the animation a certain number of times: "
                    "\n\npython3 swing_my_robot.py "
                    "/control/hyrodyn/data/hybrid/recupera/right_arm_teleop/urdf/right_arm_teleop.urdf "
                    "--submechanism_yml /control/hyrodyn/data/hybrid/recupera/right_arm_teleop/urdf/submechanisms.yml "
                    "--joint_limits_yml /control/hyrodyn/data/hybrid/recupera/right_arm_teleop/urdf/joint_limits.yml "
                    "--do_not_export_animation --number_of_loops=2\n\nMinimal Usage Example: \n\npython3 swing_my_robot.py "
                    "/control/hyrodyn/data/hybrid/recupera/right_arm_teleop/urdf/right_arm_teleop.urdf",
        formatter_class=RawTextHelpFormatter)

    # Add the arguments
    parser.add_argument("urdf_path", type=str,
                        help="URDF path for the robot with respect to AUTOPROJ_CURRENT_ROOT")
    parser.add_argument("--submechanism_yml", type=str,
                        help="YAML file describing the submechanisms with respect to AUTOPROJ_CURRENT_ROOT")
    parser.add_argument("--joint_limits_yml", type=str,
                        help="YAML file of describing the joint limits with respect to AUTOPROJ_CURRENT_ROOT")
    parser.add_argument("--trajectory_generation", type=str, 
                        help="Defining the type of the generated trajectory (sinusoidal, cycloidal or polynomial)")
    parser.add_argument("--number_of_loops", type=int, default=1,
                        help="Number of times the animation should loop")
    parser.add_argument("--do_not_export_animation", action='store_true',
                        help="Do not export the swinging animation as GIF")
    parser.add_argument("--export_animation_as_mp4", action='store_true',
                        help="Additionally convert the GIF animation to MP4 format")
    parser.add_argument("--out", type=str,
                        help="The outputfile, if not given it will be placed in the CWD")
    # 20210311 MS: Some URDFs (e.g. the ones created by Phobos) have OBJ meshes attached. Somehow they have to be rotated by 90 deg around the X-axis to be valid
    # Therefore I have added this argument
    parser.add_argument("--fix_mesh_orientation", action='store_true', help="This can fix some orientation issues with some OBJ meshes")

    # Execute the parse_args method
    args, unknown = parser.parse_known_args()

    # Store the variables in the args
    path_to_urdf = args.urdf_path
    path_to_submechamisms = args.submechanism_yml
    path_to_jointlimits = args.joint_limits_yml
    trajectory_generation = args.trajectory_generation
    number_of_loops = args.number_of_loops
    export_animation = not args.do_not_export_animation
    export_animation_as_mp4 = args.export_animation_as_mp4
    fix_mesh_orientation = args.fix_mesh_orientation
    outfile = args.out
    if outfile is not None:
        outfile = os.path.abspath(outfile)

    # Call the main function
    main(path_to_urdf, path_to_submechamisms, path_to_jointlimits, trajectory_generation, number_of_loops=1, export_animation=export_animation, 
         export_animation_as_mp4=export_animation_as_mp4, outfile=outfile, fix_mesh_orientation=fix_mesh_orientation)
