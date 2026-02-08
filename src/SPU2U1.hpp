#ifndef SPU2U1_H
#define SPU2U1_H

#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <fstream>

#include <rbdl/rbdl.h>

#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL addon URDFReader not enabled."
#endif
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "ExplicitLoopConstraints.hpp"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using Eigen::Matrix2d;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace SPU2U1 {
/*! The geometric parameters of the 2-UPS+1U mechanism are shown in the
 * following schematic: \image html 2UPS+1U_geometry.png The topological graph
 * of the 2-UPS+1U mechanism is shown in the figure below: \image html
 * 2UPS+1U_graph.png From the above figure, one can notice that the mechanism
 * has 2 independent degrees of freedom (highlighted with green), 8 spanning
 * tree degrees of freedom (black edges), 2 x 3 = 6 cut joint degrees of freedom
 * (dashed black edges) and 2 active degrees of freedom (highlighted with red).
 * \n Following are the instructions for defining coordinate frames in the CAD
 * model for the purpose of URDF export:
 * 1. We go from base to end effector always. In order to extract all the
 * geometric properties of this mechanism, we need to define 7 unique frames in
 * the mechanism.
 * 2. For the universal joint on the constraint generator leg (passive middle
 * universal joint), define a coordinate frame (F1) where the x axis is aligned
 * with the 1st universal joint axis and y axis is aligned with the 2nd
 * universal joint axis.
 * 3. On each of the actuator legs of type UPS (universal-prismatic-spherical):
 * \n a. Define a coordinate system where x axis is the 1st universal joint axis
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
 * 3. For defining joint 3, select x axis of F2 with origin at center of
 * universal joint on the actuation leg as the joint axis.
 * 4. For defining joint 4, select y axis of F3 with origin at center of
 * universal joint on the actuation leg (y aligned with 2nd cardan axis and z
 * with the prismatic actuator axis) as the joint axis.
 * 5. For defining joint 5, select z axis of F4 at the center of spherical joint
 * as the joint axis.
 * 6. For defining joint 6, select x axis of F5 with origin at center of
 * universal joint on the actuation leg as the joint axis.
 * 7. For defining joint 7, select y axis of F6 with origin at center of
 * universal joint on the actuation leg (y aligned with 2nd cardan axis and z
 * with the prismatic actuator axis) as the joint axis.
 * 8. For defining joint 8, select z axis of F7 at the center of spherical joint
 * as the joint axis.
 *
 * Assuming you named the 8 joints of the mechanism as "J1", "J2",..,"J8", here
 * is what you will use in your submechanisms.yml file:
 *
 *   name: <popular_name_of_mechanism> \n
 *   type: "2SPU+1U" \n
 *   contextual_name: <part_of_the_robot_where_it_is_used> \n
 *   file_path: <urdf_path_of_mechanism> \n
 *   jointnames_independent: ["J1", "J2"] \n
 *   jointnames_spanningtree: ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8"]
 * \n jointnames_active: ["J5", "J8"] \n
 */
class sPu2u1 : public ExplicitLoopConstraints::ExplicitLoopConstraintSet {
public:
  /// \brief Position vector of spanning tree joint positions in zero
  /// configuration
  VectorXd Q_zero;
  /// \brief Position vector of endeffector joint 1 in EE frame
  Vector3d e1_ee;
  /// \brief Position vector of endeffector joint 2 in EE frame
  Vector3d e2_ee;
  /// \brief Position vector of base joint 1 in base frame
  Vector3d b1;
  /// \brief Position vector of base joint 2 in base frame
  Vector3d b2;
  /// \brief Rotation of base joint 1 frame in base frame
  Matrix3d RotMat_b1;
  /// \brief Rotation of base joint 2 frame in base frame
  Matrix3d RotMat_b2;

  // Constructor
  /** \brief Constructor of the 2-SPU+1U mechanism class
   *
   * \param file_path file path to URDF description
   * \param jointnames_spanningtree vector of spanning tree joint names
   * \param jointnames_active vector of actuator joint names
   */
  sPu2u1(string file_path, std::vector<string> jointnames_spanningtree,
         std::vector<string> jointnames_active);
  /// \brief Wraps an angle in the range [-pi, pi]
  double wrap2pi(double x);

  /** \brief Calculates the actuator selection matrix of size (p x n)
   *
   * \param jointnames_spanningtree vector of spanning tree joint names
   * \param jointnames_active vector of actuator joint names
   */
  void calc_permutationmatrix(std::vector<string> jointnames_spanningtree,
                              std::vector<string> jointnames_active);
  // Manually coded
  /** \brief Returns spanning tree state at position (q) level from independent
   * joint position (y) by solving the loop closure function
   *
   * The following equation is used: \n
   * \f$ \mathbf{q} = \mathbf{\gamma}(\mathbf{y}) \f$
   *
   * \param y vector of independent joint positions
   */
  VectorXd calc_loopclosure_function(const Math::VectorNd &y);

  /** \brief Returns the loop closure Jacobian (G) from independent joint
   * position (y)
   *
   * The following equation is used: \n
   * \f$ \mathbf{G} = \frac{\partial \gamma}{\partial \mathbf{y}} \f$
   *
   * \param y vector of independent joint positions
   */
  MatrixXd calc_loopclosure_Jacobian(const Math::VectorNd &y);

  /** \brief Returns the loop closure Jacobian derivative (Gdot) from
   * independent joint position and velocity (y, ydot)
   *
   * The following equation is used: \n
   * \f$ \dot{\mathbf{G}} = \frac{d\mathbf{G}}{dt} \f$
   *
   * \param y vector of independent joint positions
   * \param ydot vector of independent joint velocities
   */
  MatrixXd calc_loopclosure_Jacobiand(const Math::VectorNd &y,
                                      const Math::VectorNd &ydot);

  /** \brief Returns the loop closure bias acceleration (g) from independent
   * joint position and velocity (y, yd)
   *
   * The following equation is used: \n
   * \f$ {\mathbf{g}} = \dot{\mathbf{G}}\dot{\mathbf{y}} \f$
   *
   * \param y vector of independent joint positions
   * \param ydot vector of independent joint velocities
   */
  VectorXd calc_loopclosure_g(const Math::VectorNd &y,
                              const Math::VectorNd &ydot);

  /** \brief Returns the loop closure Jacobian of UPS leg from independent joint
   * position y and position vector of end effector joint (e_ee), position
   * vector of base joint (b) and Rotation matrix of the base joint in base
   * frame (RotMat_b)
   *
   *
   * \param y vector of independent joint positions
   * \param e_ee position vector of end effector joint in the EE frame
   * \param b position vector of base joint
   * \param Rotmat_b Rotation matrix of the base joint in base frame
   */

  MatrixXd compute_loopclosure_Jacobian_UPS_leg(const Math::VectorNd y,
                                                const Vector3d e_ee,
                                                const Vector3d b,
                                                const Matrix3d RotMat_b);

  /** \brief Returns the loop closure Jacobian derivative of UPS leg from
   * independent joint position and velocity (y, yd) and position vector of end
   * effector joint (e_ee), position vector of base joint (b) and Rotation
   * matrix of the base joint in base frame (RotMat_b)
   *
   *
   * \param y vector of independent joint positions
   * \param yd vector of independent joint velocities
   * \param e_ee position vector of end effector joint in the EE frame
   * \param b position vector of base joint
   * \param Rotmat_b Rotation matrix of the base joint in base frame
   */
  MatrixXd compute_loopclosure_Jacobiandot_UPS_leg(const Math::VectorNd y,
                                                   const Math::VectorNd yd,
                                                   const Vector3d e_ee,
                                                   const Vector3d b,
                                                   const Matrix3d RotMat_b);

  // Geometric Models(mainly needed for generating LUTs)
  /** \brief Returns the indepedent joint position (y) from actuator position
   * (u) by solving the Forward Geometric Model
   *
   *  The following equation is used: \n
   * \f$ \mathbf{y} = f(\mathbf{u})\f$
   *
   * \param u vector of active joint positions
   */
  VectorXd calc_geometricmodel_forward(const Math::VectorNd u,
                                       unsigned int max_iterations = 20,
                                       double step_tol = 1.0e-8); // numerical

  /** \brief Returns the active joint positions (u) from indepedent joint
   * position (y) by solving the Inverse Geometric Model
   *
   * The following equation is used: \n
   * \f$ \mathbf{u} = f^{-1}(\mathbf{y})\f$
   *
   * \param y vector of independent joint positions
   */
  VectorXd calc_geometricmodel_inverse(const Math::VectorNd y);

  /** \brief Returns the (2x2) Kinematic Jacobian by taking the derivative of
   * the inverse geometric model
   *
   * The following equation is used: \n
   * \f$ \mathbf{J} = \mathbf{Q} \frac{\partial \mathbf{\gamma}}{\partial
   * \mathbf{y}} \f$
   *
   * \param y vector of independent joint positions
   */
  Matrix2d compute_kinematic_Jacobian(const Math::VectorNd y);
};

} // end namespace SPU2U1

#endif // SPU2U1
