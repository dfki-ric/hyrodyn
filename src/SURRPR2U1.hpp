/*
 * HyRoDyn - Hybrid Robot Dynamics
 * Copyright (c) 2018-2020 Shivesh Kumar <shivesh.kumar@dfki.de>
 * This submechanism library is contributed by Shivesh Kumar/Christoph Stoeffler
 * <shivesh.kumar@dfki.de, christoph.stoeffler@dfki.de>. Licensed under the zlib
 * license. See LICENSE for more details.
 */

#ifndef SURRPR2U1_H
#define SURRPR2U1_H

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

namespace SURRPR2U1 {
/*! The geometric parameters of the 2SURRPR+1U mechanism are shown in the
 * following schematic: \image html 2SURRPR+1U_graph.png The topological graph
 * of the 2-UPS+1U mechanism is shown in the figure below: \image html
 * 2SURRPR+1U_graph.png From the above figure, one can notice that the mechanism
 * has 2 independent degrees of freedom (highlighted with green), 16 spanning
 * tree degrees of freedom (black edges), 2 x 2 = 4 cut joint degrees of freedom
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
 * 3. To be completed...
 *
 * Assuming you named the 8 joints of the mechanism as "J1", "J2",..,"J16", here
 * is what you will use in your submechanisms.yml file:
 *
 *   name: <popular_name_of_mechanism> \n
 *   type: "2SURRPR+1U" \n
 *   contextual_name: <part_of_the_robot_where_it_is_used> \n
 *   file_path: <urdf_path_of_mechanism> \n
 *   jointnames_independent: ["J1", "J2"] \n
 *   jointnames_spanningtree: ["J1", "J2", "J3", "J4", "J5", "J6", "J7", "J8",
 * "J9", "J10", "11", "J12", "J13", "J14", "J15", "J16"] \n jointnames_active:
 * ["J12", "J15"] \n
 */

class surrPr2u1 : public ExplicitLoopConstraints::ExplicitLoopConstraintSet {

protected:
  // Physical parameters of the mechanism

  /// \brief Spanning tree joint position state at zero configuration
  VectorXd Q_zero;
  /// \brief Spanning tree joints degrees of freedom of the mechanism
  unsigned int dof_spanningtree = 16;
  /// \brief Independent joints degrees of freedom of the mechanism
  unsigned int dof_independent = 2;
  /// \brief Active joints degrees of freedom
  unsigned int dof_active = 2;

  /// \brief Base attachment points
  double b1x, b1y, b1z, b2x, b2y, b2z;
  /// \brief End effector attachment points
  double e1x, e1y, e1z, e2x, e2y, e2z;
  /// \brief Crank attachment points
  double c1x, c1y, c1z, c2x, c2y, c2z;
  /// \brief Sphere-circle intersection points
  double k1x, k1y, k1z, k2x, k2y, k2z;
  /// \brief Crank radius
  double r1, r2;
  /// \brief Length of the studs
  double l1, l2;
  double s1x, s1z, s2x, s2z;

  /// \brief Normal vector of the crank circle plane
  double n1x, n1y, n1z, n2x, n2y, n2z;
  /// \brief Offset between actual crank circle plane and lifted plane where the
  /// sphere formed by US stud intersects the virtual circle (refer figure)
  double h1, h2;

public:
  // Constructor
  /** \brief Constructor of the surrPr2u1 mechanism class
   *
   * \param file_path file path to submechanism urdf or lua description
   * \param jointnames_spanningtree vector of spanning tree joint names
   * \param jointnames_independent vector of independent joint names
   */
  surrPr2u1(string file_path, std::vector<string> jointnames_spanningtree,
            std::vector<string> jointnames_independent);

  surrPr2u1(Vector3d b1, Vector3d b2, Vector3d c1, Vector3d c2, Vector3d e1_ee,
            Vector3d e2_ee, Vector3d k1_zero, Vector3d k2_zero);

  // Contains the symbolic code
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
   * independent joint position and velocity (y, yd)
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

  VectorXd calc_geometricmodel_inverse(const Math::VectorNd y);

  VectorXd calc_geometricmodel_forward(const Math::VectorNd u,
                                       unsigned int max_iterations = 20,
                                       double step_tol = 1.0e-8); // numerical

  Matrix2d compute_kinematic_Jacobian(const Math::VectorNd y);

  /// \brief Print all the physical parameters of the mechanism
  void print_physical_parameters();
};

} // end namespace SURRPR2U1

#endif // SURRPR2U1
