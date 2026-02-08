#ifndef HYRODYN_UTILS_H
#define HYRODYN_UTILS_H

#include <math.h>
#include <stdio.h>
#include <sys/stat.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

#include "robot_model_hyrodyn.hpp"

using namespace hyrodyn;

namespace HyRoDyn_Utils {
class HyRoDyn_Utils {
 protected:
  /// \brief boolean input is limits in independent joint space
  bool input_is_independent_joints;
  /// \brief boolean input is limits in active joint space
  bool input_is_active_joints;
  /// \brief Number of steps for discretization of independent joint space or
  /// actuation space per joint
  int num_steps = 5;

  /// \brief Name of the robot model
  string robot_name;
  /// \brief Number of input joints
  int num_input_joints;
  /// \brief Vector of minimum joint positions possible
  VectorXd pos_min;
  /// \brief Vector of maximum joint positions possible
  VectorXd pos_max;
  /// \brief Vector of joint positions resolution
  VectorXd pos_resolution;

  /// \brief Vector of minimum joint velocities possible
  VectorXd vel_min;
  /// \brief Vector of maximum joint velocities possible
  VectorXd vel_max;
  /// \brief Vector of joint velocity resolution
  VectorXd vel_resolution;

  /// \brief Matrix representation of discretized input position space (each row
  /// of this matrix is a discretized point in input space)
  MatrixXd discretized_input_matrix;
  /// \brief Matrix representation of discretized input velocity space (each row
  /// of this matrix is a discretized point in input space)
  MatrixXd discretized_input_vel_matrix;
  /// \brief Vector of input joints (extracted from yaml file)
  std::vector<std::string> jointnames_input;

  /// \brief Robot model in HyRoDyn
  RobotModel_HyRoDyn* robot_model;
  /// \brief Parses yaml file to extract the information about input joints and
  /// their limits
  void initFromYaml(string filepath);
  /// \brief Discretizes the input joint space of the robot
  MatrixXd discretize_input_space(int num_steps, int num_input_joints, VectorXd min, VectorXd max,
                                  VectorXd resolution);

 public:
  /// \brief Constructor for the HyRoDyn_Utils class
  HyRoDyn_Utils(string filepath_urdf, string filepath_submechanisms, string filepath_jointlimits,
                int num_steps_per_joint = 5);  // constructor

  /// \brief Generates the full configuration space (all the joints in the
  /// spanning tree) of the mechanism
  void log_sysstate_q();
  /// \brief Generates the actuation space of the mechanism
  void log_actuatorstate_u();
  /// \brief Generates the actuation forces/torques of the mechanism
  void log_actuatorforces_Tau();
  /// \brief Generates the independent joint space of the mechanism
  void log_independentjointstate_y();  // forward model: from actuator space to
                                       // independent joint space

  //! Generates the workspace of the mechanism in SE(3)
  /*!
    \param body_name Body name defined in the URDF file for which workspace
    should be generated.
  */
  void log_forwardkinematics_x(string body_name);

  /// \brief Checks if the directory exists. If not creates the directory at the
  /// specified path \param dir_path Path of the directory to be checked or
  /// created
  void is_directory_existing(const char* dir_path);

  /*	void log_sysstate_qdot(const Math::VectorNd q, const Math::VectorNd
     qdot); void log_sysstate_qddot(const Math::VectorNd q, const Math::VectorNd
     qdot, const Math::VectorNd qddot);
  */
};

}  // end namespace HyRoDyn_Utils

#endif  // HyRoDyn_Utils
