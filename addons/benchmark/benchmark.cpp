#include <yaml-cpp/yaml.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>

#include "HyRoDyn_Utils.hpp"
#include "robot_model_hyrodyn.hpp"

using VectorNd = Eigen::VectorXd;

struct TimerResult {
  double system_state{0.0};
  double inverse_dynamics{0.0};
  double forward_dynamics{0.0};
};

class RandomGenerator {
 public:
  RandomGenerator(double min, double max) : engine(std::random_device{}()), dist(min, max) {}

  double operator()() { return dist(engine); }

 private:
  std::mt19937 engine;
  std::uniform_real_distribution<double> dist;
};

void readJointLimitsFromYaml(const std::string& filepath, VectorNd& pos_min, VectorNd& pos_max,
                             VectorNd& vel_min, VectorNd& vel_max) {
  YAML::Node doc = YAML::LoadFile(filepath);

  if (!doc["limits"])
    throw std::invalid_argument("Invalid joint limit YAML file: " + filepath);

  const auto names = doc["limits"]["names"];
  const auto elements = doc["limits"]["elements"];

  if (names.size() != elements.size())
    throw std::invalid_argument("Mismatch between joint names and elements");

  size_t n = names.size();

  pos_min = VectorNd::Zero(n);
  pos_max = VectorNd::Zero(n);
  vel_min = VectorNd::Zero(n);
  vel_max = VectorNd::Zero(n);

  for (size_t i = 0; i < n; ++i) {
    pos_max(i) = elements[i]["max"]["position"].as<double>();
    pos_min(i) = elements[i]["min"]["position"].as<double>();
    vel_max(i) = elements[i]["max"]["speed"].as<double>();

    try {
      vel_min(i) = elements[i]["min"]["speed"].as<double>();
    } catch (...) {
      vel_min(i) = -vel_max(i);
    }
  }
}

void fillRandom(Eigen::VectorXd& v, RandomGenerator& rng) {
  for (int i = 0; i < v.size(); ++i) v(i) = rng();
}

int main(int argc, char** argv) {
  hyrodyn::RobotModel_HyRoDyn robot;

  const std::string robot_urdf = "robot/rh5v2/submechanisms/submechanisms_reduced.urdf";

  const std::string robot_submechanisms = "robot/rh5v2/submechanisms/submechanisms_reduced.yml";

  robot.load_robotmodel(robot_urdf, robot_submechanisms);

  size_t dof = robot.jointnames_independent.size();

  Eigen::VectorXd y = Eigen::VectorXd::Zero(dof);
  Eigen::VectorXd yd = Eigen::VectorXd::Zero(dof);
  Eigen::VectorXd ydd = Eigen::VectorXd::Zero(dof);

  const int num_calls = 100000;

  const double q_min = -15.0 * M_PI / 180.0;
  const double q_max = 15.0 * M_PI / 180.0;

  RandomGenerator rng(q_min, q_max);

  TimerResult timings;

  for (int k = 0; k < num_calls; ++k) {
    fillRandom(y, rng);
    fillRandom(yd, rng);
    fillRandom(ydd, rng);

    robot.y = y;
    robot.yd = yd;
    robot.ydd = ydd;

    auto start = std::chrono::high_resolution_clock::now();
    robot.calculate_system_state();
    auto end = std::chrono::high_resolution_clock::now();

    timings.system_state += std::chrono::duration<double>(end - start).count();

    fillRandom(y, rng);
    fillRandom(yd, rng);
    fillRandom(ydd, rng);

    robot.y = y;
    robot.yd = yd;
    robot.ydd = ydd;

    start = std::chrono::high_resolution_clock::now();
    robot.calculate_inverse_dynamics();
    end = std::chrono::high_resolution_clock::now();

    timings.inverse_dynamics += std::chrono::duration<double>(end - start).count();

    fillRandom(y, rng);
    fillRandom(yd, rng);
    fillRandom(ydd, rng);

    robot.y = y;
    robot.yd = yd;
    robot.ydd = ydd;

    start = std::chrono::high_resolution_clock::now();
    robot.calculate_forward_dynamics();
    end = std::chrono::high_resolution_clock::now();

    timings.forward_dynamics += std::chrono::duration<double>(end - start).count();
  }

  std::cout << std::scientific << std::setprecision(6);

  std::cout << "\nRobot DOF Information\n";
  std::cout << "---------------------\n";
  std::cout << "Spanning Tree DOF : " << robot.spanningtree_dof << "\n";
  std::cout << "Independent DOF   : " << robot.independent_dof << "\n";
  std::cout << "Active DOF        : " << robot.active_dof << "\n\n";

  std::cout << "Benchmark Results\n";
  std::cout << "-----------------\n";
  std::cout << "Number of calls: " << num_calls << "\n\n";

  std::cout << "Total Time (s)\n";
  std::cout << "  System State     : " << timings.system_state << "\n";
  std::cout << "  Inverse Dynamics : " << timings.inverse_dynamics << "\n";
  std::cout << "  Forward Dynamics : " << timings.forward_dynamics << "\n\n";

  std::cout << "Average Time per Call (s)\n";
  std::cout << "  System State     : " << timings.system_state / num_calls << "\n";
  std::cout << "  Inverse Dynamics : " << timings.inverse_dynamics / num_calls << "\n";
  std::cout << "  Forward Dynamics : " << timings.forward_dynamics / num_calls << "\n";

  return 0;
}