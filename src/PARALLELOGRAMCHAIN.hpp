#ifndef PARALLELOGRAMCHAIN_H
#define PARALLELOGRAMCHAIN_H

#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <fstream>

#include <rbdl/rbdl.h>

#include <rbdl/addons/urdfreader/urdfreader.h>

#include "ExplicitLoopConstraints.hpp"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using Eigen::MatrixXd;
// using Eigen::Vector3d;
using Eigen::VectorXd;

namespace PARALLELOGRAMCHAIN {
/*! Parallelograms or parallelogram-chains can be modeled in HyRoDyn using the
 * concept of mimic joints in the URDF file. \image html
 * double_parallelogram_geometry.png A code snippet of mimic joint tag
 * definition is shown below: \n <joint name=PASSIVE_JNAME type=JTYPE > \n
 *     <parent link=PARENT /> \n
 *     <child link=CHILD /> \n
 *     <origin xyz=COORDS rpy=ANGLES /> \n
 *     <axis xyz=DIRECTION /> \n
 *     <mimic joint=ACTIVE_JNAME multiplier=m offset=b /> \n
 * </joint> \n
 * HyRoDyn will collect the multiplier and offset values from the URDF and
 * transfers them to the loop closure function of the mechanism. \n \f$
 * \mathbf{q} = \mathbf{M} \mathbf{y} + \mathbf{B} \f$ \n \f$ \dot{\mathbf{q}} =
 * \mathbf{M} \dot{\mathbf{y}} \f$ \n \f$ \ddot{\mathbf{q}} = \mathbf{M}
 * \ddot{\mathbf{y}} \f$ \n where M is the multiplier matrix containing m (+1,
 * -1, 0 as its entries) and B is the offset vector containing b (0s or angular
 * offset, in rad) involved with the active joint frame.
 */
class parallelogramchain
    : public ExplicitLoopConstraints::ExplicitLoopConstraintSet {
public:
  /// \brief Vector of offset values
  VectorXd offset;
  /// \brief Loop closure Jacobian matrix
  MatrixXd G;
  /// \brief Actuator Jacobian matrix
  MatrixXd Gu;
  /// \brief Loop closure Jacobian matrix derivative
  MatrixXd Gdot;

  // Constructor
  /** \brief Constructor of the parallelogramchain mechanism class
   *
   * \param file_path file path to submechanism urdf
   * \param jointnames_spanningtree vector of spanning tree joint names
   * \param jointnames_active vector of actuator joint names
   */
  parallelogramchain(string file_path,
                     std::vector<string> jointnames_spanningtree,
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
};

} // end namespace PARALLELOGRAMCHAIN

#endif // PARALLELOGRAMCHAIN
