#ifndef _HYRODYN_RH5_ROBOT_MODEL_HYRODYN_HPP_
#define _HYRODYN_RH5_ROBOT_MODEL_HYRODYN_HPP_

#include <Eigen/SVD>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>

#include <yaml-cpp/yaml.h>

#include <rbdl/rbdl.h>

#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL addon URDFReader not enabled."
#endif
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "HyRoDyn.hpp"
#include "SubmechanismsAssembly.hpp"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace std;
using Eigen::MatrixXd;
using namespace RRPR;

namespace hyrodyn {
class RobotModel_HyRoDyn {
protected:
  /// \brief RBDL Model
  Model m;
  /// \brief Vector of submechanisms
  std::vector<AssembleinHyRoDyn::submechanism> assembly;

  /// \brief Vector of exoskeletons
  std::vector<AssembleinHyRoDyn::exoskeleton> exteriors;
  /// \brief Vector of independent joint position in the zero configuration
  VectorNd y_zero;
  /// \brief Explicit Loop Constraint Set
  ExplicitLoopConstraints::ExplicitLoopConstraintSet *elcs;

public:
  /// \brief inverse of condition number of the Jacobian mapping from
  /// independent joint space to task space
  double inv_cond1;
  /// \brief inverse of condition number of the Jacobian mapping from actuator
  /// space to task space
  double inv_cond2;
  /// \brief inverse of condition number of the loop closure Jacobian mapping
  /// from independent joint space to spanning tree joint space
  double inv_cond_loop_closure_independent_jointspace;
  /// \brief inverse of condition number of the loop closure Jacobian mapping
  /// from actuator space to spanning tree joint space
  double inv_cond_loop_closure_active_jointspace;

  // interface variables for floating base systems
  /// \brief boolean for setting if its a floating base robot
  bool floating_base_robot = false;
  /// \brief (7x1) vector of floating robot pose, first 3 elements describe
  /// position coordinates, next 4 elements describe quaternion of orientation
  /// (x,y,z,w)
  VectorNd floating_robot_pose;
  /// \brief twist of the floating robot base (angular + linear)
  SpatialVector floating_robot_twist;
  /// \brief spatial acceleration of the floating robot base (angular + linear)
  SpatialVector floating_robot_accn;

  // interface variables for spanning tree (output for visualization)
  /// \brief vector of spanning tree joints position
  VectorNd Q;
  /// \brief vector of spanning tree joints velocity
  VectorNd QDot;
  /// \brief vector of spanning tree joints acceleration
  VectorNd QDDot;
  /// \brief Vector of spanning tree joints torque
  VectorNd Tau_spanningtree;

  // interface variables for independent joints (input)
  /// \brief vector of independent joint positions
  VectorNd y;
  /// \brief vector of independent joint velocities
  VectorNd yd;
  /// \brief vector of independent joint accelerations
  VectorNd ydd;
  /// \brief vector of independent joint torques (input)
  VectorNd Tau_independentjointspace_input;
  /// \brief vector of independent joint torques
  VectorNd Tau_independentjointspace;

  // interface variables for independent joints in the robot (input)
  /// \brief vector of independent joint positions of the robot
  VectorNd y_robot;
  /// \brief vector of independent joint velocities of the robot
  VectorNd yd_robot;
  /// \brief vector of independent joint accelerations of the robot
  VectorNd ydd_robot;

  // active joints (output for control)
  /// \brief vector of active joint positions
  VectorNd u;
  /// \brief vector of active joint velocities
  VectorNd ud;
  /// \brief vector of active joint accelelrations
  VectorNd udd;
  /// \brief vector of actuated torques
  VectorNd Tau_actuated;
  /// \brief vector of actuated torques assuming the floating base joints were
  /// actuated
  VectorNd Tau_actuated_floatingbase;

  // joint names
  /// \brief vector of joint names
  std::vector<string> jointnames;
  /// \brief vector of independent joint names
  std::vector<string> jointnames_independent;
  /// \brief vector of spanning tree joint names
  std::vector<string> jointnames_spanningtree;
  /// \brief vector of active joint names
  std::vector<string> jointnames_active;

  // DOF distribution of different submechnaisms in the robot in different
  // spaces
  /// \brief vector of submechanism degree of freedom distribution in the robot
  /// in active joint space
  std::vector<unsigned int> submechanism_active_dof_distribution;
  /// \brief vector of submechanism degree of freedom distribution in the robot
  /// in spanning tree joint space
  std::vector<unsigned int> submechanism_spanningtree_dof_distribution;
  /// \brief vector of submechanism degree of freedom distribution in the robot
  /// in independent joint space
  std::vector<unsigned int> submechanism_independent_dof_distribution;

  // DOF of the robot in different spaces
  /// \brief DOF of active joint space
  unsigned int active_dof;
  /// \brief DOF of spanning tree joint space
  unsigned int spanningtree_dof;
  /// \brief DOF of independent joint space
  unsigned int independent_dof;
  /// \brief DOF of independent joint space
  unsigned int floatingbase_dof = 0;

  // se*(3) which is the dual space of se(3) where forces and torques live
  /// \brief vector of active torques needed to balance the externally applied
  /// wrenches
  VectorNd Tau_actuated_ext;
  /// \brief vector of link names where external wrench is applied or FT sensors
  /// are placed to measure external wrenches
  std::vector<string> wrench_points;
  /// \brief vector of external wrenches (Tx, Ty, Tz, Fx, Fy, Fz)
  std::vector<Math::SpatialVector> f_ext;
  /// \brief vector of booleans deciding whether wrenches must be resolved in
  /// base coordinates (false) or body coordinates (true)
  std::vector<bool> wrench_resolution;
  /// \brief vector of booleans deciding whether wrenches must be included in an
  /// resistive (true) or assistive way (false)
  std::vector<bool> wrench_interaction;

  // task space
  /// \brief output pose of the body after solving the forward geometric model
  VectorNd pose;
  /// \brief output vector of poses of multiple bodies after solving the forward
  /// geometric model with body_names vector input
  std::vector<VectorNd> poses;
  /// \brief error tolerance for solving the inverse geometric model (defaults
  /// to 1e-8)
  double ik_error_tolerance = 1e-8;
  /// \brief input pose of the body for solving the inverse geometric model (in
  /// 7d pose_input vector, if you set the position part to zero (which is
  /// invalid) then it treats it as orientation only constraint, if you set the
  /// orientation part to zero (invalid quaternion), it is a position only
  /// constraint and otherwise it is a full pose constraint)
  std::vector<VectorNd> pose_input;
  /// \brief output twist of the body after solving the forward kinematic model
  SpatialVector twist;
  /// \brief output vector of twists of multiple bodies after solving the
  /// forward kinematic model with body_names vector input
  std::vector<SpatialVector> twists;
  /// \brief output spatial_acceleration of the body after solving the forward
  /// kinematic model
  SpatialVector spatial_acceleration;
  /// \brief output vector of spatial_accelerations of multiple bodies after
  /// forward kinematic model with body_names vector input
  std::vector<SpatialVector> spatial_accelerations;
  /// \brief output spatial_acceleration_bias of the body (Jdot*ydot) after
  /// solving the 2nd order forward kinematic model with setting joint
  /// accelerations to zero (hybrid version)
  SpatialVector spatial_acceleration_bias;

  // centroidal quantities
  /// \brief vector of center of mass position in base coordinates
  Vector3d com;
  /// \brief vector of input center of mass position in base coordinates for
  /// adding COM inverse kinematics constraint (defaults to zero i.e. constraint
  /// is inactive)
  Vector3d com_input = Vector3d::Zero();
  /// \brief vector of com linear velocity in base coordinates
  Vector3d com_vel;
  /// \brief vector of com linear acceleration in base coordinates
  Vector3d com_acc;
  /// \brief vector of com angular momentum in base coordinates
  Vector3d com_angularmomentum;
  /// \brief vector of change of angular momentum at com in base coordinates
  Vector3d com_angularmomentum_derivative;
  /// \brief total system mass
  double mass;
  /// \brief Position vector of the Zero-Moment-Point of the robot in base
  /// coordinates projected on the given contact surface
  Vector3d zmp;
  /// \brief A point on the contact surface plane
  Vector3d contact_surface_point;
  /// \brief Normal of the contact surface plane
  Vector3d contact_surface_normal;
  /// \brief Adjoint transformation (6x6) of a body in base coordinates
  Math::SpatialTransform adjoint_transformation_base;
  /// \brief COM Jacobian (3xm) projected to independent joint space in base
  /// coordinates
  Math::MatrixNd Jcom;
  /// \brief Body Jacobian (6xm) projected to independent joint space
  Math::MatrixNd Jb;
  /// \brief Space Jacobian (6xm) projected to independent joint space
  Math::MatrixNd Js;
  /// \brief Body Jacobian (6xp) projected to actuation space
  Math::MatrixNd Jbu;
  /// \brief Space Jacobian (6xp) projected to actuation space
  Math::MatrixNd Jsu;
  /// \brief Body Jacobian (6 x (p+floating_dof)) projected to actuation space
  /// including floating base coordinates
  Math::MatrixNd Jbufb;
  /// \brief Space Jacobian (6 x (p+floating_dof)) projected to actuation space
  /// including floating base coordinates
  Math::MatrixNd Jsufb;
  /// \brief Mass-inertia Matrix (mxm) projected to independent joint space
  Math::MatrixNd H;
  /// \brief Mass-inertia Matrix (pxp) projected to actuation space
  Math::MatrixNd Hu;
  /// \brief Mass-inertia Matrix ((p+floating_dof)x(p+floating_dof)) projected
  /// to actuation space including floating base coordinates
  Math::MatrixNd Hufb;

  //! Print a welcome to stdout
  void welcome();
  void load_submechanisms_yaml(string filepath);
  //! Loads the urdf and submechanism.yml file to create the hyrodyn robot model
  /*!
    \param filepath_urdf string file path to URDF.
    \param filepath_submechanisms string file path to submechanisms yml
  */
  //! Sets the gravity vector for hyrodyn robot model
  /*!
    \param gravity_vector acceleration due to gravity vector (defaults to
    [0,0,-9.81])
  */
  void set_gravity_vector(Vector3d gravity_vector);
  void load_robotmodel(string filepath_urdf, string filepath_submechanisms);
  //! Loads a smurf file and parses from it the urdf and submechanisms file to
  //! then create a RobotModel_HyRoDyn instance
  void load_smurf(string filepath_smurf);
  //! Parses  urdf and submechanisms file from a smurf file
  static void load_filepathes_from_smurf(string filepath_smurf,
                                         string &filepath_urdf,
                                         string &filepath_submechanisms);
  //! Compute the full system state (Q, QDot, QDDot) of the robot from the
  //! independent joint state (y, yd, ydd)
  void calculate_system_state();
  //! Compute the full system state (Q, QDot, QDDot) of the robot from the
  //! active joint state (u, ud, udd)
  void calculate_forward_system_state();
  //! Compute the inverse dynamics (Tau_actuated) of the robot from the
  //! independent joint state (y, yd, ydd)
  void calculate_inverse_dynamics();
  //! Compute the inverse dynamics (Tau_actuated) of the robot from the
  //! independent joint state (y, yd, ydd) including the floating base
  void calculate_inverse_dynamics_including_floatingbase();
  //! Compute the inverse dynamics in independent joint space
  //! (Tau_independentjointspace) of the robot from the independent joint state
  //! (y, yd, ydd)
  void calculate_inverse_dynamics_independentjointspace();
  //! Compute the constrained inverse dynamics (Tau_actuated) of the robot from
  //! the independent joint state (y, yd), spatial acceleration (xdd) and
  //! desired_wrench(fext)
  void calculate_constrained_inverse_dynamics();
  //! Compute the forward dynamics (ydd) of the robot from the independent joint
  //! state (y, yd, Tau_actuated)
  void calculate_forward_dynamics();
  //! Compute the inverse statics (Tau_actuated) of the robot from the
  //! independent joint state (y), external wrenches (f_ext) and wrench
  //! application points (wrench_points)
  void calculate_inverse_statics();
  //! Compute the simplified inverse dynamics (Tau_actuated) of the robot from
  //! the independent joint state (y) and independent joint torques
  //! (Tau_independentjointspace)
  void calculate_simplified_inverse_dynamics();
  //! Compute the forward kinematics (pose, twist, acceleration) of a single
  //! body of the robot from the independent joint state (y, yd, ydd)
  /*!
    \param body_name Body name defined in the URDF file for which forward
    kinematics should be solved.
  */
  void calculate_forward_kinematics(string body_name);
  //! Compute the forward kinematics (pose, twist, acceleration) of a number of
  //! bodies of the robot from the independent joint state (y, yd, ydd)
  /*!
    \param body_names Body names defined in the URDF file for which forward
    kinematics should be solved.
  */
  void
  calculate_forward_kinematics_multiple_bodies(std::vector<string> body_names);
  //! Compute the adjoint transformation of a body on the robot in base
  //! coordinates
  /*!
    \param body_name Body name defined in the URDF file for which adjoint
    transformation should be solved.
  */
  void calculate_adjoint_transformation(string body_name);
  //! Compute the inverse kinematics i.e. independent joint state (y) of the
  //! robot from the (pose_input) vector
  /*!
    \param body_names Body names defined in the URDF file for which inverse
    kinematics should be solved.
  */
  void calculate_inverse_kinematics(std::vector<string> body_names);
  //! Compute the inverse of condition number of the Jacobians (J*G,
  //! J*G*Gu.inverse) of a specific link on the robot (inv_cond1, inv_cond2)
  //! from independent joint state (y)
  /*!
    \param body_name Body name defined in the URDF file for which inverse
    kinematics should be solved.
  */
  void calculate_condition_number(string body_name,
                                  bool for_point_Jacobian = false);
  //! Compute the inverse of condition number of the loop closure Jacobians (G,
  //! G*Gu.inverse) of the robot (inv_cond1, inv_cond2) from independent joint
  //! state (y)
  void calculate_condition_number_loop_closure();
  //! Compute the com properties like com position, velocity and angular
  //! momentum (com, com_vel, com_angularmomentum) of the robot from independent
  //! joint state (y, yd)
  void calculate_com_properties();
  //! Compute the com Jacobian of the robot from independent joint state (y)
  void calculate_com_jacobian();
  //! Compute the Body Jacobian (Jb) of the robot from independent joint state
  //! (y) projected to independent joint space
  /*!
    \param body_name Body name defined in the URDF file for which Body Jacobian
    should be computed.
  */
  void calculate_body_jacobian(string body_name);
  //! Compute the Space Jacobian (Js) of the robot from independent joint state
  //! (y) projected to independent joint space
  /*!
    \param body_name Body name defined in the URDF file for which Space Jacobian
    should be computed.
  */
  void calculate_space_jacobian(string body_name);
  //! Compute the Body Jacobian (Jbu) of the robot from independent joint state
  //! (y) projected to actuation space
  /*!
    \param body_name Body name defined in the URDF file for which Body Jacobian
    should be computed.
  */
  void calculate_body_jacobian_actuation_space(string body_name);
  //! Compute the Space Jacobian (Jsu) of the robot from independent joint state
  //! (y) projected to actuation space
  /*!
    \param body_name Body name defined in the URDF file for which Space Jacobian
    should be computed.
  */
  void calculate_space_jacobian_actuation_space(string body_name);
  //! Compute the Body Jacobian (Jbufb) of the robot from independent joint
  //! state (y) projected to actuation space including the floating base
  /*!
    \param body_name Body name defined in the URDF file for which Body Jacobian
    should be computed.
  */
  void calculate_body_jacobian_actuation_space_including_floatingbase(
      string body_name);
  //! Compute the Space Jacobian (Jsufb) of the robot from independent joint
  //! state (y) projected to actuation space including the floating base
  /*!
    \param body_name Body name defined in the URDF file for which Space Jacobian
    should be computed.
  */
  void calculate_space_jacobian_actuation_space_including_floatingbase(
      string body_name);

  //! Compute the Spatial Acceleration Bias term (Jdot*ydot) of a given body of
  //! the robot from independent joint state (y, yd)
  /*!
    \param body_name Body name defined in the URDF file for which Spatial
    Acceleration Bias term should be computed.
  */
  void calculate_spatial_acceleration_bias(string body_name);
  //! Compute the mass-inertia matrix (H) from independent joint state (y)
  void calculate_mass_interia_matrix();
  //! Compute the mass-inertia matrix (Hu) from independent joint state (y)
  void calculate_mass_interia_matrix_actuation_space();
  //! Compute the mass-inertia matrix (Hufb) from independent joint state (y)
  //! including the floating base
  void calculate_mass_interia_matrix_actuation_space_including_floatingbase();

  //! Compute the Zero Moment Point (ZMP) of the robot from independent joint
  //! state (y, yd, ydd) and contact surface definition (contact_surface_point,
  //! contact_surface_normal)
  void calculate_zmp();
  //! Update the full system state from independent joint state of the
  //! robot(y_robot, yd_robot, ydd_robot) and floating base coordinates
  //! (floating_robot_pose, floating_robot_twist, floating_robot_accn) wrt to
  //! the world frame
  void update_all_independent_coordinates();
  // ExplicitLoopConstraints::ExplicitLoopConstraintSet& get_elcs(){return
  // *elcs;};

  double calculate_kinetic_energy();
  double calculate_potential_energy();
  double calculate_total_energy();
};

} // end namespace hyrodyn

#endif // _HYRODYN_RH5_ROBOT_MODEL_HYRODYN_HPP_
