#ifndef RRPR_H
#define RRPR_H

#include <math.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
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

namespace RRPR {
/*! The geometric parameters of the 1-RRPR mechanism are shown in the following
 * schematic: \image html lambda_geometry.png The topological graph of the
 * 1-RRPR mechanism is shown in the figure below: \image html lambda_topo.png
 * From the above figure, one can notice that the mechanism has 1 independent
 * degrees of freedom (highlighted with green), 3 spanning tree degrees of
 * freedom (black edges), 1 cut joint degrees of freedom (dashed black edges)
 * and 1 active degrees of freedom (highlighted with red). \n Following are the
 * instructions for defining coordinate frames in the CAD model for the purpose
 * of URDF export:
 * 1. We go from base to end effector always. In order to extract all the
 * geometric properties of this mechanism, we need to define 3 unique frames in
 * the mechanism.
 * 2. For the end effector revolute joint, define a coordinate frame (F1) at the
 * center of the joint between the Root link and End effector link.
 * 3. Define the coordinate frame (F2) at the center of the joint between Root
 * link and the stator link of the actuator with same orientation as F1.
 * 4. Define the coordinate frame (F3) at the center of the joint between actor
 * link of the actuator and the End effector link while making sure you point
 * the z axis of the frame along the prismatic joint axis. NOTE: It is to be
 * noted that the origin of the three frames F1, F2 and F3 should lie in the
 * same plane as its a planar mechanism.
 *
 * Instructions for definiting the joints in the URDF:
 * 1. For defining joint 1, select any axis (X, Y or Z) of Frame 1 (F1) as the
 * joint axis.
 * 2. For defining joint 2, select the same axis as F1 of F2 as the joint axis.
 * 3. For defining joint 3, select z axis of F3 as the joint axis.
 *
 * Assuming you named the 3 joints of the mechanism as "J1", "J2" and "J3" here
 * is what you will use in your submechanisms.yml file:
 *
 *   name: <popular_name_of_mechanism> \n
 *   type: "rrPr" \n
 *   contextual_name: <part_of_the_robot_where_it_is_used> \n
 *   file_path: <urdf_path_of_mechanism> \n
 *   jointnames_independent: ["J1"] \n
 *   jointnames_spanningtree: ["J1", "J2", "J3"] \n
 *   jointnames_active: ["J3"] \n
 */
class rrPr : public ExplicitLoopConstraints::ExplicitLoopConstraintSet {
 protected:
  /// \brief Length of the base link (refer mechanism's schematic)
  double l1;
  /// \brief Length of the crank link (refer mechanism's schematic)
  double l2;
  /// \brief Length of the slider (refer mechanism's schematic)
  double d;
  /// \brief Spanning tree joint position state at zero configuration
  VectorXd Q_zero;

 public:
  // Constructor
  /** \brief Constructor of the rrPr mechanism class
   *
   * \param file_path file path to submechanism urdf or lua description
   * \param jointnames_spanningtree vector of spanning tree joint names
   * \param jointnames_active vector of actuator joint names
   */
  rrPr(string file_path, std::vector<string> jointnames_spanningtree,
       std::vector<string> jointnames_active);
  /// \brief Wraps an angle in the range [-pi, pi]
  double wrap2pi(double x);

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

  /** \brief Returns the actuator selection matrix of size (p x n)
   *
   * \param jointnames_spanningtree vector of spanning tree joint names
   * \param jointnames_active vector of actuator joint names
   */
  void calc_permutationmatrix(std::vector<string> jointnames_spanningtree,
                              std::vector<string> jointnames_active);
};

}  // namespace RRPR

#endif  // Rsup
