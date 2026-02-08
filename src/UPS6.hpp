#ifndef UPS6_H
#define UPS6_H

#include <math.h>
#include <rbdl/rbdl.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>

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

namespace UPS6 {
/*! The geometric parameters of the 6-UPS mechanism are shown in the following
 * schematic: \image html 6UPS_geometry.png
 */
class uPs6 : public ExplicitLoopConstraints::ExplicitLoopConstraintSet {
 public:
  /// \brief RBDL Model of the parallel mechanism of type 6-UPS
  Model m;

  // mechanism parameters

  /// \brief Position vector of endeffector joint 1 in ee frame
  Vector3d e1_ee;
  /// \brief Position vector of endeffector joint 2 in ee frame
  Vector3d e2_ee;
  /// \brief Position vector of endeffector joint 3 in ee frame
  Vector3d e3_ee;
  /// \brief Position vector of endeffector joint 4 in ee frame
  Vector3d e4_ee;
  /// \brief Position vector of endeffector joint 5 in ee frame
  Vector3d e5_ee;
  /// \brief Position vector of endeffector joint 6 in ee frame
  Vector3d e6_ee;

  /// \brief Position vector of base joint 1 in base frame
  Vector3d b1;
  /// \brief Position vector of base joint 2 in base frame
  Vector3d b2;
  /// \brief Position vector of base joint 3 in base frame
  Vector3d b3;
  /// \brief Position vector of base joint 4 in base frame
  Vector3d b4;
  /// \brief Position vector of base joint 5 in base frame
  Vector3d b5;
  /// \brief Position vector of base joint 6 in base frame
  Vector3d b6;

  /// \brief Rotation of base joint 1 frame in base frame
  Matrix3d RotMat_b1;
  /// \brief Rotation of base joint 2 frame in base frame
  Matrix3d RotMat_b2;
  /// \brief Rotation of base joint 3 frame in base frame
  Matrix3d RotMat_b3;
  /// \brief Rotation of base joint 4 frame in base frame
  Matrix3d RotMat_b4;
  /// \brief Rotation of base joint 5 frame in base frame
  Matrix3d RotMat_b5;
  /// \brief Rotation of base joint 6 frame in base frame
  Matrix3d RotMat_b6;

  /// \brief Transformation matrix of EE in zero configuration
  Eigen::Matrix4d TransMat_zero;

  /// \brief Vector of independent joint positions in the zero configuration
  VectorXd y_zero;

  /// \brief Degree of freedom of active joints (p)
  unsigned int dof_active;
  /// \brief Degree of freedom of spanning tree (n)
  unsigned int dof_spanningtree;
  /// \brief Vector of spanning tree joint positions in zero configuration
  VectorXd Q_zero;

  // Constructor
  /** \brief Constructor of the UPS6 mechanism class
   *
   * \param file_path file path to URDF or lua description
   * \param jointnames_spanningtree vector of spanning tree joint names
   * \param jointnames_active vector of actuator joint names
   */
  uPs6(string file_path, std::vector<string> jointnames_spanningtree,
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

  /** \brief Returns the loop closure Jacobian of 6-UPS leg from independent
   * joint position y, position vector of leg base joint in base frame b, vector
   * of leg endeffector joint in ee frame and rotation of leg base joint frame
   * in base frame
   *
   * \param y vector of independent joint positions
   * \param b position vector of base joint of the leg in base frame
   * \param e_ee position vector of endeffector joint of the leg in ee frame
   * \param RotMat_b rotation of base joint frame of the leg in base frame
   */
  MatrixXd compute_loopclosure_Jacobian_UPS6(const Math::VectorNd y, const Math::Vector3d b,
                                             const Math::Vector3d e_ee,
                                             const Math::Matrix3d RotMat_b);

  /** \brief Returns the loop closure Jacobian derivative of 6-UPS leg from
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
  MatrixXd compute_loopclosure_Jacobiandot_UPS6(const Math::VectorNd y, const Math::VectorNd ydot,
                                                const Math::Vector3d b, const Math::Vector3d e_ee,
                                                const Math::Matrix3d RotMat_b);
};

}  // end namespace UPS6

#endif  // UPS6
