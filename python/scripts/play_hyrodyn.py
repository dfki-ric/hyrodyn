import time
import yaml
import numpy as np
import viser
import hyrodyn as hyrodyn

from viser.extras import ViserUrdf
from yourdfpy import URDF


# -------------------------------------------------
# Utilities
# -------------------------------------------------

def load_joint_limits(path):
    """Load joint limits from YAML file."""
    with open(path, "r") as f:
        yaml_data = yaml.safe_load(f)

    names = yaml_data["limits"]["names"]
    elements = yaml_data["limits"]["elements"]

    limits = {}
    for name, elem in zip(names, elements):
        limits[name] = (
            elem["min"]["position"],
            elem["max"]["position"],
        )

    return limits


def build_urdf_joint_index(urdf):
    """Create mapping from joint name to URDF index."""
    urdf_joint_names = [
        name for name, joint in urdf.joint_map.items()
        if joint.type != "fixed"
    ]

    return urdf_joint_names, {name: i for i, name in enumerate(urdf_joint_names)}


def update_viser_robot(robot, viser_urdf, urdf_joint_names, urdf_index):
    """Update Viser robot configuration from HyRoDyn state."""
    cfg = np.zeros(len(urdf_joint_names))

    for joint_name, joint_value in zip(robot.jointnames_spanningtree, robot.Q):
        if joint_name in urdf_index:
            cfg[urdf_index[joint_name]] = joint_value

    viser_urdf.update_cfg(cfg)


# -------------------------------------------------
# GUI
# -------------------------------------------------

def create_joint_sliders(server, robot, joint_limits, update_callback):
    """Create sliders for independent joints."""
    sliders = []

    with server.gui.add_folder("Independent joints"):

        for i, joint_name in enumerate(robot.jointnames_independent):

            lower, upper = joint_limits.get(joint_name, (-np.pi, np.pi))

            slider = server.gui.add_slider(
                label=joint_name,
                min=lower,
                max=upper,
                step=0.001,
                initial_value=0.0,
            )

            sliders.append(slider)

            @slider.on_update
            def _(_event, idx=i):
                y = np.array(robot.y)
                y[idx] = sliders[idx].value
                robot.y = y

                robot.calculate_system_state()
                update_callback()

    return sliders


# -------------------------------------------------
# Main
# -------------------------------------------------

def main():

    path_to_urdf = "../robot/leg/urdf/leg.urdf"
    path_to_submechanisms = "../robot/leg/urdf/submechanisms.yml"
    path_to_joint_limits = "../robot/leg/urdf/joint_limits.yml"

    # Load robot
    robot = hyrodyn.RobotModel(path_to_urdf, path_to_submechanisms)

    # Load joint limits
    joint_limits = load_joint_limits(path_to_joint_limits)

    # Initial configuration
    robot.y = np.array([0.0, 0.0, -0.15, 0.3, 0.0, -0.15])
    robot.calculate_system_state()

    print("Active joints:", robot.jointnames_active)
    print("Independent joints:", robot.jointnames_independent)
    print("Spanning tree joints:", robot.jointnames_spanningtree)

    # -------------------------------------------------
    # Start Viser
    # -------------------------------------------------

    server = viser.ViserServer()

    urdf = URDF.load(path_to_urdf)

    viser_urdf = ViserUrdf(
        server,
        urdf,
        load_meshes=True,
    )

    urdf_joint_names, urdf_index = build_urdf_joint_index(urdf)

    def update_viser():
        update_viser_robot(robot, viser_urdf, urdf_joint_names, urdf_index)

    update_viser()

    server.scene.add_grid(
        "/grid",
        width=0.5,
        height=0.5,
    )

    server.scene.add_frame(
        "/world",
        axes_length=0.25,
        axes_radius=0.01,
    )

    # -------------------------------------------------
    # GUI
    # -------------------------------------------------

    sliders = create_joint_sliders(
        server,
        robot,
        joint_limits,
        update_viser,
    )

    reset_button = server.gui.add_button("Reset joints")

    @reset_button.on_click
    def _(_):
        robot.y = np.zeros(robot.independent_dof)

        for s in sliders:
            s.value = 0.0

        robot.calculate_system_state()
        update_viser()

    # -------------------------------------------------
    # Keep server alive
    # -------------------------------------------------

    print("Open browser at: http://localhost:8080")

    while True:
        time.sleep(10)


if __name__ == "__main__":
    main()
