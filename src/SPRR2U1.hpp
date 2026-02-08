#ifndef SPRR2U1_H
#define SPRR2U1_H

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

namespace SPRR2U1 {
/*! The geometric parameters of the 2-SPRR+1U mechanism are shown in the
 * following schematic: \image html 2SPRR+1U_geometry.png
 */
class sPrr2u1 : public ExplicitLoopConstraints::ExplicitLoopConstraintSet {
public:
  /// \brief RBDL Model of the 2SPRR+1U mechanism
  Model m;

  // mechanism parameters
  /// \brief x coordinate of the foot attachment point 1
  double f1x;
  /// \brief y coordinate of the foot attachment point 1
  double f1y;
  /// \brief z coordinate of the foot attachment point 1
  double f1z;
  /// \brief x coordinate of the foot attachment point 2
  double f2x;
  /// \brief y coordinate of the foot attachment point 2
  double f2y;
  /// \brief z coordinate of the foot attachment point 2
  double f2z;
  /// \brief x coordinate of the shank attachment point 1
  double s1x;
  /// \brief y coordinate of the shank attachment point 1
  double s1y;
  /// \brief z coordinate of the shank attachment point 1
  double s1z;
  /// \brief x coordinate of the shank attachment point 2
  double s2x;
  /// \brief y coordinate of the shank attachment point 2
  double s2y;
  /// \brief z coordinate of the shank attachment point 2
  double s2z;
  /// \brief length of the offset link
  double r;

  /// \brief Degree of freedom of active joints (p)
  unsigned int dof_active;
  /// \brief Degree of freedom of spanning tree (n)
  unsigned int dof_spanningtree;

  /// \brief RotMat_s1 Rotation matrix of the frame attached at shank attachment
  /// point 1 in the base frame
  Matrix3d RotMat_s1;
  /// \brief RotMat_s2 Rotation matrix of the frame attached at shank attachment
  /// point 2 in the base frame
  Matrix3d RotMat_s2;
  /// \brief RotMat_f1 Rotation matrix of the frame attached at foot attachment
  /// point 1 in the base frame
  Matrix3d RotMat_f1;
  /// \brief RotMat_f2 Rotation matrix of the frame attached at foot attachment
  /// point 2 in the base frame
  Matrix3d RotMat_f2;
  /// \brief Vector of spanning tree position state in zero configuration
  VectorXd Q_zero;
  /// \brief Position vector of frame attached at foot attachment point 1 in ee
  /// frame
  Vector3d f1_ee;
  /// \brief Position vector of frame attached at foot attachment point 2 in ee
  /// frame
  Vector3d f2_ee;
  /// \brief Position vector of frame attached at shank attachment point 1 in
  /// base frame
  Vector3d s1;
  /// \brief Position vector of frame attached at shank attachment point 2 in
  /// base frame
  Vector3d s2;
  /// \brief Joint axis vector of the intermediate offset joint 1 in ee frame
  Vector3d i1_ee;
  /// \brief Joint axis vector of the intermediate offset joint 2 in ee frame
  Vector3d i2_ee;
  /// \brief length of the offset link 1
  double r1;
  /// \brief length of the offset link 2
  double r2;
  /// \brief Vector of f1k1 in zero configuration
  Vector3d f1k1_zero;
  /// \brief Vector of f2k2 in zero configuration
  Vector3d f2k2_zero;

  // Constructor
  /** \brief Constructor of the sPrr2u1  mechanism class
   *
   * \param file_path file path to lua description
   * \param jointnames_spanningtree vector of spanning tree joint names
   * \param jointnames_active vector of actuator joint names
   */
  sPrr2u1(string file_path, std::vector<string> jointnames_spanningtree,
          std::vector<string> jointnames_active);

  /** \brief Returns the actuator selection matrix of size (p x n)
   *
   * \param jointnames_spanningtree vector of spanning tree joint names
   * \param jointnames_active vector of actuator joint names
   */
  void calc_permutationmatrix(std::vector<string> jointnames_spanningtree,
                              std::vector<string> jointnames_active);
  /// \brief Wraps an angle in the range [-pi, pi]
  double wrap2pi(double x);
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

  /** \brief Returns the loop closure Jacobian of UP leg from independent joint
   * position y,vector of i_ee, r. vector of f_ee, s and rotation of RotMat_s
   *
   * The following equation is used: \n
   *
   *
   * \param y vector of independent joint positions
   * \param i_ee Joint axis vector of the intermediate offset joint in ee frame
   * \param r length of the intermediate offset link
   * \param f_ee Position vector of frame attached at foot attachment point in
   * ee frame \param s Position vector of frame attached at shank attachment
   * point in base frame \param RotMat_s Rotation matrix of the frame attached
   * at shank attachment point in the base frame
   */
  MatrixXd compute_loopclosure_Jacobian_UP_leg(
      const Math::VectorNd y, const Vector3d i_ee, const double r,
      const Vector3d f_ee, const Vector3d s, const Matrix3d RotMat_s);

  /** \brief Returns the loop closure Jacobian derivative of UP leg from
   * independent joint position y, independent joint velocity yd, vector of
   * i_ee, r. vector of f_ee, s and rotation of RotMat_s
   *
   * The following equation is used: \n
   *
   *
   * \param y vector of independent joint positions
   * \param yd vector of independent joint velocities
   * \param i_ee Joint axis vector of the intermediate offset joint in ee frame
   * \param r length of the intermediate offset link
   * \param f_ee Position vector of frame attached at foot attachment point in
   * ee frame \param s Position vector of frame attached at shank attachment
   * point in base frame \param RotMat_s Rotation matrix of the frame attached
   * at shank attachment point in the base frame
   */
  MatrixXd compute_loopclosure_Jacobiandot_UP_leg(
      const Math::VectorNd y, const Math::VectorNd yd, const Vector3d i_ee,
      const double r, const Vector3d f_ee, const Vector3d s,
      const Matrix3d RotMat_s);

  // Geometric Models

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
                                       double step_tol = 1.0e-8);

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

  /** \brief Returns the (2x2) Kinematic Jacobian derivative by taking the
   * derivative of the kinematic Jacobian
   *
   * The following equation is used: \n
   * \f$ \dot{\mathbf{J}} = \frac{d\mathbf{J}}{dt} \f$
   *
   * \param y vector of independent joint positions
   */
  Matrix2d compute_kinematic_Jacobiandot(const Math::VectorNd y,
                                         const Math::VectorNd yd);
};

} // end namespace SPRR2U1

#endif // SPRR2U1
