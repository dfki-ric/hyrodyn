#ifndef NUMERICALLOOPCONSTRAINTS_H
#define NUMERICALLOOPCONSTRAINTS_H

// Author: Rohit Kumar
#include <math.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/rbdl.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <Eigen/Dense>
#include <Eigen/QR>
#include <fstream>
#include <iostream>

#include "ExplicitLoopConstraints.hpp"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace NUMERICALLOOPCONSTRAINTS {
/// \brief Structure of constraint axes paramaters
struct Constraint_dof {
  /// \brief name of the constraint
  string name;
  /// \brief Spatial vector of the constraint axis in the order [wx wy wz vx vy
  /// vz] where w = (wx, wy, wz) denote the angular velocity and v = (vx, vy,
  /// vz) denote the linear velocity
  SpatialVector axis;
  /// \brief Stabilization parameter to calculate implicit acceleration level
  /// loop closure constraints
  double baumg_stab_param;
  /// \brief Print constraints information
  void print_constraint_name_and_axis() {
    cout << "Name : " << name << endl;
    cout << "Axis : " << axis.transpose() << endl;
    cout << "Baumgarte stabilization parameter : " << baumg_stab_param << endl;
  }
};
/// \brief Structure of Loop closure constraints
struct Loop_constraints {
  /// \brief Cut joint frame
  string jointname_cut;
  /// \brief Predecessor body frame
  string pred_body;
  /// \brief Successor body frame
  string succ_body;
  /// \brief Vector of constraint axes paramteres
  std::vector<Constraint_dof> constraint_axes;

  /// \brief Print the loop constraint details
  void print_loop_constraints_details() {
    cout << "\e[1m"
         << "<-------Loop Constraints Details------->"
         << "\e[0m" << endl;
    cout << "Cut joint Name: " << jointname_cut << endl;
    cout << "Predecessor body of the cut joint: " << pred_body << endl;
    cout << "Successor body of the cut joint: " << succ_body << endl;
    cout << "\e[1m"
         << "<---Constraint Axes ----> "
         << "\e[0m" << endl;

    for (uint j = 0; j < constraint_axes.size(); j++) {
      cout << "Constraint " << j + 1 << endl;
      constraint_axes[j].print_constraint_name_and_axis();
      cout << " <--------------------> " << endl;
    }
  }
};

class NumericalLoopConstraints : public ExplicitLoopConstraints::ExplicitLoopConstraintSet {
 private:
  /// \brief Implicit constraint Jacobian matrix
  MatrixNd K;
  /// \brief Explicit constraint Jacobian matrix
  MatrixXd G;
  /// \brief Internal state of explicit constraints matrix
  MatrixXd internal_G;
  /// \brief Implicit constraint accleration vector
  VectorXd k;
  /// \brief Explicit constraint bias acceleration vector
  VectorXd g;
  /// \brief Selection matrix for independent joints
  MatrixNd independent_joints_selection_matrix;
  /// \brief Selection matrix for dependent joints
  MatrixNd dependent_joints_selection_matrix;
  /// \brief Independent joint matrix from Implicit constraint Jacobian matrix
  MatrixXd Ki;
  /// \brief Dependent joint matrix from Implicit constraint Jacobian matrix
  MatrixXd Kd;
  /// \brief Independent joint matrix from Explicit constraint Jacobian matrix
  MatrixXd Gd;
  /// \brief Dependent joint matrix from Explicit constraint Jacobian matrix
  MatrixXd Gi;
  /// \brief Independent joints bias acceleration vector
  VectorXd gi;
  /// \brief Dependent joint bias acceleration vector
  VectorXd gd;
  /// \brief Model of the system
  Model m;
  /// \brief Loop closure Constraint set
  ConstraintSet cs;
  /// \brief Vector of generalised joint positions
  VectorNd Q;
  /// \brief Internal state of independent joints vector
  VectorNd internal_y;
  /// \brief Vector of generalised joint veocities
  VectorNd QDot;
  /// \brief Vector of initial guess for the generalized positions of the joints
  VectorNd QInit;
  /// \brief weighting coefficients for the different joint positions.
  VectorNd weights;

 public:
  // Constructor
  /** \brief Constructor of the NumericalLoopConstraints class
   *
   * \param file_path file path to submechanism urdf or lua description
   * \param jointnames_spanningtree vector of spanning tree joint names
   * \param jointnames_active vector of actuator joint names
   * \param jointnames_independent vector of independent joint names
   * \param loop_contraints_set vector of LoopConstraints structure
   */
  NumericalLoopConstraints(const string& file_path,
                           const std::vector<string>& jointnames_spanningtree,
                           const std::vector<string>& jointnames_independent,
                           const std::vector<string>& jointnames_active,
                           const std::vector<Loop_constraints>& loop_contraints_set,
                           bool verbose = false);

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

  /** \brief Returns the loop closure Jacobian (G) from implicit constraint
   * Jacobian matrix (K)
   *
   * \param K Implicit constraint Jacobian matrix
   */
  MatrixXd calc_G_from_K(MatrixNd& K);

  /** \brief Returns the implicit loop closure acceleration numerically
   *
   * The following equation is used: \n
   * \f$ {\mathbf{k}} = -\dot{\mathbf{K}}\dot{\mathbf{q}} \f$
   *
   * \param m Model of the system
   * \param CS the constraint set for which the error should be computed
   * \param Q  vector of the generalized joint positions
   * \param QDot  vector of the generalized joint velocities
   */
  VectorXd calc_k(Model& m, const Math::VectorNd& Q, const Math::VectorNd& QDot, ConstraintSet& CS);

  /** \brief Returns the loop closure bias acceleration (g) from implicit
   * implicit loop closure acceleration(k) and implicit constraint Jacobian
   * matrix (K)
   *
   * \param K implicit constraint Jacobian matrix
   * \param k implicit loop closure acceleration
   */
  VectorXd calc_g_from_k(MatrixNd& K, VectorXd& k);

  /** \brief Returns the independent and dependent joints selection matrices
   *
   * \param jointnames_spanningtree vector of spanning tree joint names
   * \param jointnames_independent vector of independent joint names
   */
  void calc_selection_matrices(const std::vector<string>& jointnames_spanningtree,
                               const std::vector<string>& jointnames_independent);

  /** \brief Returns the actuator selection matrix of size (p x n)
   *
   * \param jointnames_spanningtree vector of spanning tree joint names
   * \param jointnames_active vector of actuator joint names
   */
  void calc_permutationmatrix(const std::vector<string>& jointnames_spanningtree,
                              const std::vector<string>& jointnames_active);
};
}  // namespace NUMERICALLOOPCONSTRAINTS

#endif  // NumericalLoopConstraitnts
