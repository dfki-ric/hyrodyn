#include <rbdl/rbdl.h>

#include <iostream>

#include "HyRoDyn_Limit_Checker.hpp"
#include "HyRoDyn_Utils.hpp"
#include "robot_model_hyrodyn.hpp"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
int main(int argc, char** argv) {
  hyrodyn::RobotModel_HyRoDyn rh5;
  rh5.welcome();

  std::string filepath_urdf = "robot/rh5v2/submechanisms/RH5v2.urdf";
  std::string filepath_submechanisms = "robot/rh5v2/submechanisms/submechanisms.yml";

  rh5.load_robotmodel(filepath_urdf, filepath_submechanisms);

  rh5.calculate_system_state();

  std::cout << " System state: " << rh5.Q.transpose() << std::endl;

  return 0;
}
