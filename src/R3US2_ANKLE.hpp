/*
 * HyRoDyn - Hybrid Robot Dynamics
 * Copyright (c) 2018-2020 Shivesh Kumar <shivesh.kumar@dfki.de>
 * This submechanism library is contributed by Ibrahim Tijjani
 * <ibrahim.tijjani@dfki.de>. Licensed under the zlib license. See LICENSE for
 * more details.
 */

#ifndef R3US2_ANKLE_H
#define R3US2_ANKLE_H

#include <addons/urdfreader/urdfreader.h>
#include <math.h>
#include <rbdl/rbdl.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>

#include "ExplicitLoopConstraints.hpp"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace R3US2_ANKLE {

const double PI = 3.14159265;

/*! The geometric parameters of the r3us2_ankle mechanism are shown in the
 * following schematic: \image html active_ankle.png The topological graph of
 * the R3US2 mechanism is shown in the figure below: \image html
 * active_ankle.png
 * * From the above figure, one can notice that the mechanism has 6 independent
 * degrees of freedom (highlighted with red), 21 spanning tree degrees of
 * freedom (black edges), 3 x 2 = 6 cut joint degrees of freedom (dashed black
 * edges) and 3 active degrees of freedom (highlighted with yellow). \n
 * Following are the instructions for defining coordinate frames in the CAD
 * model for the purpose of URDF export:
 * 1. We go from base to end effector always. In order to extract all the
 * geometric properties of this mechanism, we need to define 15 unique frames in
 * the mechanism.
 * 2. For the universal joint on the constraint generator leg (passive middle
 * universal joint), define a coordinate frame (F1) where the x axis is aligned
 * with the 1st universal joint axis and y axis is aligned with the 2nd
 * universal joint axis.
 * 3. On each of the actuator legs of type US (universal-spherical): \n
 *    a. Define a coordinate system where x axis is the 1st universal joint axis
 * and y axis is the 2nd universal joint axis. This will result in the
 * definition of frames F2 and F5 for the two actuation legs. \n b. At the
 * universal joint, define another frame where the y axis is the 2nd universal
 * joint axis and z axis is the prismatic joint axis. This will result in the
 * definition of frames F3 and F6 for the two actuation legs. \n c. Define the
 * 3rd frame as a copy of 2nd frame translated along the prismatic joint axis
 * such that its origin lies at the center of the spherical joint. This will
 * result in the definition of frames F4 and F7 for the two actuation legs. \n
 *
 *
 * Instructions for definiting the joints in the URDF:
 * 1. For defining joint 1, select x axis of Frame 1 (F1) as the joint axis.
 * 2. For defining joint 2, select y axis of F1 as the joint axis.
 * 3. For defining joint 3, select z axis of F1 as the joint axis.
 * 4. For defining joint 4, select x axis of Frame 2 (F2) as the joint axis.
 * 5. For defining joint 5, select y axis of F2 as the joint axis.
 * 6. For defining joint 6, select z axis of F2 as the joint axis.
 * 7. To be completed...
 *
 * Assuming you named the 8 joints of the mechanism as "J1", "J2",..,"J21", here
 * is what you will use in your submechanisms.yml file:
 *
 *   name: <popular_name_of_mechanism> \n
 *   type: "R3US2_ANKLE" \n
 *   contextual_name: <part_of_the_robot_where_it_is_used> \n
 *   file_path: <urdf_path_of_mechanism> \n
 *   jointnames_independent: ["J1", "J2", "J3", "J4", "J5", "J6"] \n
 *   jointnames_spanningtree: ["J1", "J2", "J3", "J4", "J5", "J6", "J9", "J10",
 * "J11", "J12", "J13", "J14", "J15", "J16", "J17", "J18", "J19", "J20", "J21",
 * "J22", "J23"] \n jointnames_active: ["J8", "J13", "J18"] \n
 */

class r3us2_ankle : public ExplicitLoopConstraints::ExplicitLoopConstraintSet {
 protected:
  // Physical parameters of the mechanism

  /// \brief Spanning tree joint position state at zero configuration
  VectorXd Q_zero;
  /// \brief Spanning tree joints degrees of freedom of the mechanism
  unsigned int dof_spanningtree = 21;
  /// \brief Independent joints degrees of freedom of the mechanism
  unsigned int dof_independent = 3;
  /// \brief Active joints degrees of freedom
  unsigned int dof_active = 3;

  /// \brief Position vector of endeffector point 1 in ee frame
  Vector3d e1_ee;
  /// \brief Position vector of endeffector point 2 in ee frame
  Vector3d e2_ee;
  /// \brief Position vector of endeffector point 3 in ee frame
  Vector3d e3_ee;
  /// \brief Position vector of endeffector point 4 in ee frame
  Vector3d e4_ee;
  /// \brief Position vector of endeffector point 5 in ee frame
  Vector3d e5_ee;
  /// \brief Position vector of endeffector point 6 in ee frame
  Vector3d e6_ee;

  /// \brief Position vector of endeffector joint 6 in base frame
  Vector3d Position_EE;

  /// \brief Position vector of endeffector joint 6 in base frame
  Vector3d Position_T;
  /// \brief Position vector of endeffector joint 6 in ee frame
  Vector3d Position_EEprime;

  /// \brief Rotation of crank point 1 frame in motor 1 frame
  Matrix3d RotMat_c1;
  /// \brief Rotation of crank point 2 frame in motor 1 frame
  Matrix3d RotMat_c2;
  /// \brief Rotation of crank point 3 frame in motor 2 frame
  Matrix3d RotMat_c3;
  /// \brief Rotation of crank point 4 frame in motor 2 frame
  Matrix3d RotMat_c4;
  /// \brief Rotation of crank point 5 frame in motor 3 frame
  Matrix3d RotMat_c5;
  /// \brief Rotation of crank point 6 frame in motor 3 frame
  Matrix3d RotMat_c6;

  /// \brief Position vector of endeffector point 1 in EE frame
  Vector3d e1;
  /// \brief Position vector of endeffector point 2 in EE frame
  Vector3d e2;
  /// \brief Position vector of endeffector point 3 in EE frame
  Vector3d e3;
  /// \brief Position vector of endeffector point 4 in EE frame
  Vector3d e4;
  /// \brief Position vector of endeffector point 5 in EE frame
  Vector3d e5;
  /// \brief Position vector of endeffector point 6 in EE frame
  Vector3d e6;

  /// \brief Position vector of crank point 1 in motor 1 frame
  Vector3d c1;
  /// \brief Position vector of crank point 2 in motor 1 frame
  Vector3d c2;
  /// \brief Position vector of crank point 3 in motor 2 frame
  Vector3d c3;
  /// \brief Position vector of crank point 4 in motor 2 frame
  Vector3d c4;
  /// \brief Position vector of crank point 5 in motor 3 frame
  Vector3d c5;
  /// \brief Position vector of crank point 6 in motor 3 frame
  Vector3d c6;

  /// \brief Position vector of motor 1 joint in base frame
  Vector3d motor_1;
  /// \brief Position vector of lever 1 joint in base frame
  Vector3d lever_1_roll;
  /// \brief Position vector of lever 2 joint in base frame
  Vector3d lever_2_roll;
  /// \brief Position vector of motor 2 joint in base frame
  Vector3d motor_2;
  /// \brief Position vector of lever 3 joint in base frame
  Vector3d lever_3_roll;
  /// \brief Position vector of lever 4 joint in base frame
  Vector3d lever_4_roll;
  /// \brief Position vector of motor 3 joint in base frame
  Vector3d motor_3;
  /// \brief Position vector of lever 5 joint in base frame
  Vector3d lever_5_roll;
  /// \brief Position vector of lever 6 joint in base frame
  Vector3d lever_6_roll;

  /// \brief Rotation matrix of the end effector in base frame
  Matrix3d RotMat_Tprime;
  /// \brief Rotation matrix of the end effector in ee frame
  Matrix3d RotMat_EE;
  /// \brief Rotation matrix of the end effector in task space frame
  Matrix3d RotMat_T;

  /// \brief Transformation matrix of EE in zero configuration
  Eigen::Matrix4d G_T_E_zero;
  /// \brief Transformation matrix of EE in zero configuration
  Eigen::Matrix4d E_T_T_zero;
  /// \brief Transformation matrix of EE in zero configuration
  Eigen::Matrix4d G_T_T_zero;
  /// \brief Transformation matrix of EEprime in EE frame
  Eigen::Matrix4d Eprime_T_T;
  /// \brief Transformation matrix of EE in zero configuration
  Eigen::Matrix4d E_T_Eprime;
  /// \brief Transformation matrix of EE in zero configuration
  Eigen::Matrix4d TransMat_zero;
  /// \brief Transformation matrix of EE in zero configuration
  Eigen::Matrix4d G_T_Tprime;

 public:
  /// \brief RBDL Model of the R3US2_ANKLE mechanism
  Model m;
  double r, l, d;  // ASM Parameters
  double tolerance = 1e-06;
  MatrixXd G_T_E;

  // Constructor
  /** \brief Constructor of the r3us2_ankle mechanism class
   *
   * \param file_path file path to submechanism urdf or lua description
   * \param jointnames_spanningtree vector of spanning tree joint names
   * \param jointnames_independent vector of independent joint names
   */
  r3us2_ankle(string file_path, std::vector<string> jointnames_spanningtree,
              std::vector<string> jointnames_independent);

  // Contains the symbolic code
  /** \brief Returns spanning tree state at position (q) level from independent
   * joint position (y) by solving the loop closure function
   *
   * The following equation is used: \n
   * \f$ \mathbf{q} = \mathbf{\gamma}(\mathbf{y}) \f$
   *
   * \param y vector of independent joint positions
   */
  VectorXd calc_loopclosure_function(const Math::VectorNd& y);

  /** \brief Returns the loop closure Jacobian (G) from independent joint
   * position (y)
   *
   * The following equation is used: \n
   * \f$ \mathbf{G} = \frac{\partial \gamma}{\partial \mathbf{y}} \f$
   *
   * \param y vector of independent joint positions
   */
  MatrixXd calc_loopclosure_Jacobian(const Math::VectorNd& y);

  /** \brief Returns the loop closure Jacobian derivative (Gdot) from
   * independent joint position and velocity (y, yd)
   *
   * The following equation is used: \n
   * \f$ \dot{\mathbf{G}} = \frac{d\mathbf{G}}{dt} \f$
   *
   * \param y vector of independent joint positions
   * \param ydot vector of independent joint velocities
   */
  MatrixXd calc_loopclosure_Jacobiand(const Math::VectorNd& y, const Math::VectorNd& ydot);

  /** \brief Returns the loop closure bias acceleration (g) from independent
   * joint position and velocity (y, yd)
   *
   * The following equation is used: \n
   * \f$ {\mathbf{g}} = \dot{\mathbf{G}}\dot{\mathbf{y}} \f$
   *
   * \param y vector of independent joint positions
   * \param ydot vector of independent joint velocities
   */
  VectorXd calc_loopclosure_g(const Math::VectorNd& y, const Math::VectorNd& ydot);
  /** \brief Returns the loop closure Jacobian of R-3US2 leg from independent
   * joint position y, position vector of leg base joint in base frame b, vector
   * of leg endeffector joint in ee frame and rotation of leg base joint frame
   * in base frame
   *
   * \param y vector of independent joint positions
   * \param b position vector of base joint of the leg in base frame
   * \param e_ee position vector of endeffector joint of the leg in ee frame
   * \param RotMat_b rotation of base joint frame of the leg in base frame
   */
  MatrixXd compute_loopclosure_Jacobian(const Math::VectorNd y, const Math::Vector3d c,
                                        const Math::Vector3d ee_, const Math::Matrix3d RotMat_c);

  /** \brief Returns the loop closure Jacobian derivative of R-3US2_ANKLE from
   * independent joint position y, independent joint velocity ydot, position
   * vector of leg base joint in base frame b, vector of leg endeffector joint
   * in ee frame and rotation of leg base joint frame in base frame
   *
   * \param y vector of independent joint positions
   * \param ydot vector of independent joint velocities
   * \param b position vector of base joint of the leg in base frame
   * \param e_ee position vector of endeffector joint of the leg in ee frame
   * \param RotMat_b rotation of base joint frame of the leg in base frame
   */
  MatrixXd compute_loopclosure_Jacobiandot(const Math::VectorNd y, const Math::VectorNd ydot,
                                           const Math::Vector3d c, const Math::Vector3d e,
                                           const Math::Matrix3d RotMat_c);

  double wrap2pi(double x);

  /** \brief This function wrap the x values into the interval (-pi, pi]
   *
   * \param jointnames_spanningtree vector of spanning tree joint names
   * \param jointnames_active vector of actuator joint names
   */

  double choose_soln(double x, double y);

  Vector3d IGM(MatrixXd G_T_E);

  void calculate_crankpoints(Vector3d q, MatrixXd& c);
  /** \brief Calculates the crank points (ci) by the motion of the joint
     actuators
               *
               * \param input joint angles  (qx,qy,qz)
               * \param radius (r) on the crank pair points
               *  \param length of rod (l) on the centers of the cranks along
     the (x,y,z) axes of the plane.
               *
               * * The following equation is used: \n
              //~ * \f$ \mathbf{C} = (ci: 1<=i<=6)\f$
              */

  void calculate_endeffectorpoints(MatrixXd G_T_E, MatrixXd& e);
  /** \brief Calculates the six end effector position vectors (E) from the
     coordinate points of the end effector position (ei) and orinetation vectors
     (s, n, a)
              *
              * The following equation is used: \n
              //~ * \f$ \mathbf{E} = (ei: 1<=i<=6)\f$
              *
              */

  void calculate_rodlengths(MatrixXd c, MatrixXd e, VectorXd& rod_lengths);
  /** \brief This function computes the kinematic contraint equations to
     calculate the rod length (l), the distance  btw the end effector points and
     the crank points.
               *
               * \param end effector points (ei)
               * \param crank points (ci)
               *
               * * The following equation is used: \n
               //~ * \f$ \mathbf{l} = norm(ei-ci)\f$
              */

  double calculate_rodlengths_error(VectorXd rod_lengths);
  /** \brief This function minimizes the least square error btw the change in
     virtual rod length (li) and the rod length (l)  //rigidity error
               *
               * \param virtual rod length(li)
               * \param rod length(l)
               *
               * * The following equation is used: \n
               //~ * \f$ \mathbf{l} = sum(norm(ei-ci) - l)^2\f$
              */

  Vector3d calculate_endeffector_position(MatrixXd G_T_E, MatrixXd c);
  /** \brief This function calculates the end effector position
   *
   * \param end effector orientation (s, n, a)
   * \param crank point (c)
   * \param rod length (l)
   *
   */

  void solve_RIGM(MatrixXd G_T_E, Vector3d& q, Vector3d& E);
  /** \brief This function returns the solution of the Rotative inverse
   * kinematic model
   *
   * \param End effector point (E)
   * \param Joint angles (qx, qy, qz)
   *
   */

  MatrixXd RotMatrixFromAxisAngle(Vector3d axis, double angle);
  /** \brief This function generates the rotation matrix from the axes joint
   * angles
   *
   * \param End effector shift (ex, ey, ez)
   * \param Joint angles (qx, qy, qz)
   *
   */

  Vector3d sphere_intersection(Vector3d fc1, Vector3d fc2, Vector3d fc3, double r1, double r2,
                               double r3, unsigned int pos);
  /** \brief This function generates the intersection points of the rotative
   * spheres on the surface of the cranks
   *
   */
};

}  // end namespace R3US2_ANKLE

#endif  // R3US2_ANKLE
