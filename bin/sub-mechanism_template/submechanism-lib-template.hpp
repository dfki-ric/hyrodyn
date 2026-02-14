/*
 * HyRoDyn - Hybrid Robot Dynamics
 * Copyright (c) 2018-2020 Shivesh Kumar <shivesh.kumar@dfki.de>
 * This submechanism library is contributed by lib_author_name <lib_author_email>.
 * Licensed under the zlib license. See LICENSE for more details.
 */

#ifndef mech_type_cap_H
#define mech_type_cap_H

#include <addons/urdfreader/urdfreader.h>
#include <math.h>
#include <rbdl/rbdl.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>

include_in_hyrodyn #include "ExplicitLoopConstraints.hpp"

    using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace mech_type_cap {
/*! The geometric parameters of the mech_type_small mechanism are shown in the following schematic:
 *  \image html mechanism_geometry_schematic_filename
 */
class mech_type_small include_in_hyrodyn
    : public ExplicitLoopConstraints::ExplicitLoopConstraintSet {
 protected:
  // Physical parameters of the mechanism

  /// \brief Spanning tree joint position state at zero configuration
  VectorXd Q_zero;
  /// \brief Spanning tree joints degrees of freedom of the mechanism
  unsigned int dof_spanningtree = st_dof;
  /// \brief Independent joints degrees of freedom of the mechanism
  unsigned int dof_independent = ind_dof;

 public:
  // Constructor
  /** \brief Constructor of the mech_type_small mechanism class
   *
   * \param file_path file path to submechanism urdf or lua description
   * \param jointnames_spanningtree vector of spanning tree joint names
   * \param jointnames_independent vector of independent joint names
   */
  mech_type_small(string file_path, std::vector<string> jointnames_spanningtree,
                  std::vector<string> jointnames_independent);

  // Contains the symbolic code
  /** \brief Returns spanning tree state at position (q) level from independent joint position (y)
   * by solving the loop closure function
   *
   * The following equation is used: \n
   * \f$ \mathbf{q} = \mathbf{\gamma}(\mathbf{y}) \f$
   *
   * \param y vector of independent joint positions
   */
  VectorXd calc_loopclosure_function(const Math::VectorNd& y);

  /** \brief Returns the loop closure Jacobian (G) from independent joint position (y)
   *
   * The following equation is used: \n
   * \f$ \mathbf{G} = \frac{\partial \gamma}{\partial \mathbf{y}} \f$
   *
   * \param y vector of independent joint positions
   */
  MatrixXd calc_loopclosure_Jacobian(const Math::VectorNd& y);

  /** \brief Returns the loop closure Jacobian derivative (Gdot) from independent joint position and
   * velocity (y, yd)
   *
   * The following equation is used: \n
   * \f$ \dot{\mathbf{G}} = \frac{d\mathbf{G}}{dt} \f$
   *
   * \param y vector of independent joint positions
   * \param ydot vector of independent joint velocities
   */
  MatrixXd calc_loopclosure_Jacobiand(const Math::VectorNd& y, const Math::VectorNd& ydot);

  /** \brief Returns the loop closure bias acceleration (g) from independent joint position and
   * velocity (y, yd)
   *
   * The following equation is used: \n
   * \f$ {\mathbf{g}} = \dot{\mathbf{G}}\dot{\mathbf{y}} \f$
   *
   * \param y vector of independent joint positions
   * \param ydot vector of independent joint velocities
   */
  VectorXd calc_loopclosure_g(const Math::VectorNd& y, const Math::VectorNd& ydot);
};

}  // end namespace mech_type_cap

#endif  // mech_type_cap
