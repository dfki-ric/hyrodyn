#ifndef EXPLICITLOOPCONSTRAINTS_H
#define EXPLICITLOOPCONSTRAINTS_H

#include <math.h>
#include <rbdl/rbdl.h>
#include <stdio.h>

#include <Eigen/Dense>
#include <iostream>

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace ExplicitLoopConstraints {
// pure Abstract Class
class ExplicitLoopConstraintSet {
 public:
  /// \brief Degree of freedom of active joints (p)
  unsigned int dof_active;
  /// \brief Degree of freedom of spanning tree (n)
  unsigned int dof_spanningtree;
  /// \brief Degree of freedom of independent joints + free-flyer joints (m +
  /// m_floatingbase)
  unsigned int dof_independent;
  /// \brief Degree of freedom of independent robot (m)
  unsigned int dof_independent_robot;

  /// \brief Actuator selection matrix of size (p x n)
  MatrixXd permutation_matrix;
  /// \brief Indepedent joints of the robot selection matrix of size (m -
  /// floating_dof x m)
  MatrixXd permutation_matrix2;

  ExplicitLoopConstraintSet();
  /** \brief Returns spanning tree state at position (q) level from independent
   * joint position (y) by solving the loop closure function
   *
   * The following equation is used: \n
   * \f$ \mathbf{q} = \mathbf{\gamma}(\mathbf{y}) \f$
   *
   * \param y vector of independent joint positions
   */
  virtual VectorXd calc_loopclosure_function(const Math::VectorNd& y);
  /** \brief Returns the loop closure Jacobian (G) from independent joint
   * position (y)
   *
   * The following equation is used: \n
   * \f$ \mathbf{G} = \frac{\partial \gamma}{\partial \mathbf{y}} \f$
   *
   * \param y vector of independent joint positions
   */
  virtual MatrixXd calc_loopclosure_Jacobian(const Math::VectorNd& y);
  /** \brief Returns the loop closure Jacobian derivative (Gdot) from
   * independent joint position and velocity (y, yd)
   *
   * The following equation is used: \n
   * \f$ \dot{\mathbf{G}} = \frac{d\mathbf{G}}{dt} \f$
   *
   * \param y vector of independent joint positions
   * \param ydot vector of independent joint velocities
   */
  virtual MatrixXd calc_loopclosure_Jacobiand(const Math::VectorNd& y, const Math::VectorNd& yd);

  /** \brief Returns the loop closure bias acceleration (g) from independent
   * joint position and velocity (y, yd)
   *
   * The following equation is used: \n
   * \f$ {\mathbf{g}} = \dot{\mathbf{G}}\dot{\mathbf{y}} \f$
   *
   * \param y vector of independent joint positions
   * \param ydot vector of independent joint velocities
   */
  virtual VectorXd calc_loopclosure_g(const Math::VectorNd& y, const Math::VectorNd& yd);

  /// \brief Returns the degree of freedom of spanning tree
  unsigned int get_dof_spanningtree() { return dof_spanningtree; };
  /// \brief Returns the degree of freedom of independent joints + free-flyer
  /// joints
  unsigned int get_dof_independent() { return dof_independent; };
  /// \brief Returns the degree of freedom of independent robot
  unsigned int get_dof_independent_robot() { return dof_independent_robot; };
  /// \brief Returns the degree of freedom of active joints
  unsigned int get_dof_active() { return dof_active; };
  /// \brief Returns the degree of freedom of floating base joint
  unsigned int get_dof_floatingbase() { return dof_independent - dof_independent_robot; };

  /// \brief Returns Actuator selection matrix of size (p x n)
  const MatrixXd& get_permutation_matrix() { return permutation_matrix; };
  /// \brief Returns Indepedent joints of the robot selection matrix of size (m
  /// - floating_dof x m)
  const MatrixXd& get_permutation_matrix2() { return permutation_matrix2; };
  /// \brief Explicit Loop Constraint Set
  virtual ~ExplicitLoopConstraintSet(){};
};

}  // namespace ExplicitLoopConstraints

#endif  // EXPLICITLOOPCONSTRAINTS
