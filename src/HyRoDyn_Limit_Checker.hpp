#ifndef HYRODYN_LIMIT_CHECKER_H
#define HYRODYN_LIMIT_CHECKER_H

#include <math.h>
#include <stdio.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

#include <addons/urdfreader/urdfreader.h>

#include "robot_model_hyrodyn.hpp"

using namespace hyrodyn;

class HyRoDyn_Limit_Checker {
 protected:
  /// \brief vector of independent joint position limits (minimum)
  VectorXd y_min;
  /// \brief vector of independent joint position limits (maximum)
  VectorXd y_max;
  /// \brief vector of independent joint velocity limits (minimum)
  VectorXd yd_min;
  /// \brief vector of independent joint velocity limits (maximum)
  VectorXd yd_max;
  /// \brief vector of independent joint acceleration limits (minimum)
  VectorXd ydd_min;
  /// \brief vector of independent joint acceleration limits (maximum)
  VectorXd ydd_max;
  /// \brief vector of independent joint effort limits (minimum)
  VectorXd Tau_independent_min;
  /// \brief vector of independent joint effort limits (maximum)
  VectorXd Tau_independent_max;

  /// \brief vector of spanning tree joint position limits (minimum)
  VectorXd q_min;
  /// \brief vector of spanning tree joint position limits (maximum)
  VectorXd q_max;
  /// \brief vector of spanning tree joint velocity limits (minimum)
  VectorXd qd_min;
  /// \brief vector of spanning tree joint velocity limits (maximum)
  VectorXd qd_max;
  /// \brief vector of spanning tree joint acceleration limits (minimum)
  VectorXd qdd_min;
  /// \brief vector of spanning tree joint acceleration limits (maximum)
  VectorXd qdd_max;
  /// \brief vector of spanning tree joint effort limits (minimum)
  VectorXd Tau_spanningtree_min;
  /// \brief vector of spanning tree joint effort limits (maximum)
  VectorXd Tau_spanningtree_max;

  /// \brief vector of active joint position limits (minimum)
  VectorXd u_min;
  /// \brief vector of active joint position limits (maximum)
  VectorXd u_max;
  /// \brief vector of active joint velocity limits (minimum)
  VectorXd ud_min;
  /// \brief vector of active joint velocity limits (maximum)
  VectorXd ud_max;
  /// \brief vector of active joint acceleration limits (minimum)
  VectorXd udd_min;
  /// \brief vector of active joint acceleration limits (maximum)
  VectorXd udd_max;
  /// \brief vector of active joint effort limits (minimum)
  VectorXd Tau_actuated_min;
  /// \brief vector of active joint effort limits (maximum)
  VectorXd Tau_actuated_max;

  /// \brief Robot Model in HyRoDyn
  RobotModel_HyRoDyn* robot_model;

 public:
  /// \brief vector of joint indices of independent joints vector where the
  /// position limit is violated
  std::vector<unsigned int> input_pos_limit_violation_report;
  /// \brief vector of joint indices of independent joints vector where the
  /// velocity limit is violated
  std::vector<unsigned int> input_vel_limit_violation_report;
  /// \brief vector of joint indices of spanning tree joints vector where the
  /// position limit is violated
  std::vector<unsigned int> cspace_pos_limit_violation_report;
  /// \brief vector of joint indices of spanning tree joints vector where the
  /// velocity limit is violated
  std::vector<unsigned int> cspace_vel_limit_violation_report;
  /// \brief vector of joint indices of active joints vector where the position
  /// limit is violated
  std::vector<unsigned int> actuator_pos_limit_violation_report;
  /// \brief vector of joint indices of active joints vector where the velocity
  /// limit is violated
  std::vector<unsigned int> actuator_vel_limit_violation_report;
  /// \brief vector of joint indices of active joints vector where the effort
  /// limit is violated
  std::vector<unsigned int> actuator_effort_limit_violation_report;

  /// \brief Constructor of the HyRoDyn_Limit_Checker class
  /*!
    \param filepath_urdf string file path to URDF.
    \param filepath_submechanisms string file path to submechanisms yml
  */
  HyRoDyn_Limit_Checker(string filepath_urdf, string filepath_submechanisms,
                        bool verbose = false);  // constructor

  /** \brief Compares two eigen vectors element-wise (returns true if vector
   * values are within min-max limits, otherwise false)
   *
   * \param vec vector to be comapred
   * \param min vector of minimum allowed values
   * \param max vector of maximum allowed values
   * \param index vector of the indices where the check failed (output)
   */
  bool compare_eigen_vectors(VectorXd vec, VectorXd min, VectorXd max,
                             std::vector<unsigned int>& index);

  /** \brief Performs a hierarchical point-wise capability check (returns true
   * if capability is feasible, otherwise false)
   *
   * \param y vector of independent joint positions
   * \param yd vector of independent joint velocities
   * \param ydd vector of independent joint accelerations \n
   * NOTE: If the function returns false i.e. the capability check fails, the
   * exact reason of the failure is stored into the limit violation report
   * variables of the class.
   */
  bool hierarchical_capability_check(const VectorXd y, const VectorXd yd, const VectorXd ydd);
};

#endif  // HyRoDyn_Limit_Checker
