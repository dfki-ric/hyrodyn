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

  std::string filepath_urdf =
      "/home/dfki.uni-bremen.de/rkumar/hyrodyn_release/hyrodyn/data/hybrid/"
      "rh5v2/submechanisms/RH5v2.urdf";
  std::string filepath_submechanisms =
      "/home/dfki.uni-bremen.de/rkumar/hyrodyn_release/hyrodyn/"
      "data/hybrid/rh5v2/submechanisms/submechanisms.yml";

  /* string filepath_urdf =
"/home/rohit/hyrodyn_dev/control/hyrodyn/data/hybrid/rh5/arm/urdf/ALElbow.urdf";
string filepath_submechanisms =
"/home/rohit/hyrodyn_dev/control/hyrodyn/data/hybrid/rh5/arm/urdf/submechanisms.yml";
*/

  /* string filepath_urdf =
  "/home/rohit/hyrodyn_dev/control/hyrodyn/data/hybrid/simplified-full-urdf-develop/submechanisms/Body.urdf";
  string filepath_submechanisms =
  "/home/rohit/hyrodyn_dev/control/hyrodyn/data/hybrid/simplified-full-urdf-develop/submechanisms/body_submechanism.yml";
*/

  rh5.load_robotmodel(filepath_urdf, filepath_submechanisms);

  rh5.calculate_system_state();

  // std::cout << " System state: " << rh5.Q << std::endl;
  // std::cout << " Tot energy: " << rh5.calculate_total_energy() << std::endl;

  return 0;
}
