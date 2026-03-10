#include <gtest/gtest.h>

#include <robot_model_hyrodyn.hpp>

using namespace hyrodyn;

const std::string filepath_urdf = "robot/leg/urdf/leg.urdf";
const std::string filepath_submechanisms = "robot/leg/urdf/submechanisms.yml";

const std::string filepath_urdf_floatingbase = "robot/lower_body/urdf/leg_floatingbase.urdf";
const std::string filepath_submechanisms_floatingbase =
    "robot/lower_body/urdf/leg_submechanisms_floatingbase.yml";

const std::string filepath_urdf_numerical = "robot/rh5v2/submechanisms/submechanisms_reduced.urdf";
const std::string filepath_submechanisms_analytical =
    "robot/rh5v2/submechanisms/submechanisms_reduced.yml";
const std::string filepath_submechanisms_numerical =
    "robot/rh5v2/submechanisms/submechanisms_reduced_numerical.yml";
const std::string filepath_jointlimits = "robot/rh5v2/submechanisms/joint_limits_independent.yml";

const double epsilon = 1.0e-8;
const double epsilon_relaxed = 1.0e-5;

/* -------------------------------------------------------------------------- */

TEST(Hyrodyn, ForwardInverseGeometricModelConsistency) {
  RobotModel_HyRoDyn robot;
  robot.load_robotmodel(filepath_urdf, filepath_submechanisms);

  VectorXd y = VectorNd::Zero(robot.jointnames_independent.size());
  y(0) = 0.1;
  robot.y = y;

  robot.calculate_forward_kinematics("LLAnklePitch_Link");
  robot.pose_input.push_back(robot.pose);

  std::vector<std::string> body_names = {"LLAnklePitch_Link"};
  robot.calculate_inverse_kinematics(body_names);

  ASSERT_TRUE((robot.y - y).norm() <= epsilon) << "Input y:\n"
                                               << y << "\nOutput y:\n"
                                               << robot.y << "\nError = " << (robot.y - y).norm();
}

/* -------------------------------------------------------------------------- */

TEST(Hyrodyn, ForwardInverseGeometricModelMultipleBodiesConsistency) {
  RobotModel_HyRoDyn robot;
  robot.load_robotmodel(filepath_urdf, filepath_submechanisms);

  VectorXd y = VectorNd::Zero(robot.jointnames_independent.size());
  y(0) = 0.1;
  robot.y = y;

  std::vector<std::string> body_names = {"LLAnklePitch_Link", "LLKnee_Link"};

  robot.calculate_forward_kinematics_multiple_bodies(body_names);
  robot.pose_input = robot.poses;

  robot.y.setZero();
  robot.calculate_inverse_kinematics(body_names);

  ASSERT_TRUE((robot.y - y).norm() <= epsilon) << "Input y:\n"
                                               << y << "\nOutput y:\n"
                                               << robot.y << "\nError = " << (robot.y - y).norm();
}

/* -------------------------------------------------------------------------- */

TEST(Hyrodyn, ForwardKinematicsMultipleBodiesConsistency) {
  RobotModel_HyRoDyn robot;
  robot.load_robotmodel(filepath_urdf, filepath_submechanisms);

  robot.y.setOnes();
  robot.yd.setOnes();
  robot.ydd.setOnes();

  std::vector<std::string> body_names = {"LLAnklePitch_Link", "LLKnee_Link", "LLHip3_Link"};

  robot.calculate_forward_kinematics_multiple_bodies(body_names);

  for (size_t i = 0; i < body_names.size(); ++i) {
    robot.calculate_forward_kinematics(body_names[i]);

    ASSERT_TRUE((robot.pose - robot.poses[i]).norm() <= epsilon);
    ASSERT_TRUE((robot.twist - robot.twists[i]).norm() <= epsilon);
    ASSERT_TRUE((robot.spatial_acceleration - robot.spatial_accelerations[i]).norm() <= epsilon);
  }
}

/* -------------------------------------------------------------------------- */

TEST(Hyrodyn, InverseSystemStateZeroConfig) {
  RobotModel_HyRoDyn robot;
  robot.load_robotmodel(filepath_urdf, filepath_submechanisms);

  robot.y.setZero();
  robot.yd.setZero();
  robot.ydd.setZero();

  robot.calculate_system_state();

  ASSERT_TRUE(robot.Q.isZero(epsilon));
  ASSERT_TRUE(robot.QDot.isZero(epsilon));
  ASSERT_TRUE(robot.QDDot.isZero(epsilon));
}

/* -------------------------------------------------------------------------- */

TEST(Hyrodyn, ForwardSystemStateZeroConfig) {
  RobotModel_HyRoDyn robot;
  robot.load_robotmodel(filepath_urdf, filepath_submechanisms);

  robot.u.setZero();
  robot.ud.setZero();
  robot.udd.setZero();

  robot.calculate_forward_system_state();

  ASSERT_TRUE(robot.Q.isZero(epsilon));
  ASSERT_TRUE(robot.QDot.isZero(epsilon));
  ASSERT_TRUE(robot.QDDot.isZero(epsilon));
}

/* -------------------------------------------------------------------------- */

TEST(Hyrodyn, ForwardInverseSystemStateConsistency) {
  RobotModel_HyRoDyn robot;
  robot.load_robotmodel(filepath_urdf, filepath_submechanisms);

  robot.y.setZero();
  robot.yd.setZero();
  robot.ydd.setZero();

  robot.y(0) = 0.1;
  robot.yd(0) = 0.1;
  robot.ydd(0) = 0.1;

  robot.calculate_system_state();
  VectorXd Q_inv = robot.Q;
  VectorXd Qd_inv = robot.QDot;
  VectorXd Qdd_inv = robot.QDDot;

  robot.calculate_forward_system_state();

  ASSERT_TRUE((robot.Q - Q_inv).norm() <= epsilon);
  ASSERT_TRUE((robot.QDot - Qd_inv).norm() <= epsilon);
  ASSERT_TRUE((robot.QDDot - Qdd_inv).norm() <= epsilon);
}

/* -------------------------------------------------------------------------- */

TEST(Hyrodyn, ForwardInverseDynamicModelConsistency) {
  RobotModel_HyRoDyn robot;
  robot.load_robotmodel(filepath_urdf, filepath_submechanisms);

  robot.y.setZero();
  robot.yd.setZero();
  robot.ydd.setZero();

  robot.calculate_inverse_dynamics();
  VectorXd tau = robot.Tau_actuated;

  robot.Tau_actuated = tau;
  robot.calculate_forward_dynamics();

  ASSERT_TRUE(robot.ydd.isZero(epsilon));
}
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */

TEST(Hyrodyn, SubmechanismNumericalVsAnalyticalConsistency) {
  RobotModel_HyRoDyn robot_analytical;
  RobotModel_HyRoDyn robot_numerical;

  robot_analytical.load_robotmodel(filepath_urdf_numerical, filepath_submechanisms_analytical);

  robot_numerical.load_robotmodel(filepath_urdf_numerical, filepath_submechanisms_numerical);

  ASSERT_EQ(robot_analytical.jointnames_independent.size(),
            robot_numerical.jointnames_independent.size());

  size_t n = robot_analytical.jointnames_independent.size();

  VectorXd y(n);
  VectorXd yd(n);
  VectorXd ydd(n);

  YAML::Node yaml = YAML::LoadFile(filepath_jointlimits);
  YAML::Node names = yaml["limits"]["names"];
  YAML::Node elements = yaml["limits"]["elements"];

  std::unordered_map<std::string, std::pair<double, double>> limits_map;

  for (size_t i = 0; i < names.size(); ++i) {
    std::string name = names[i].as<std::string>();

    double min_pos = elements[i]["min"]["position"].as<double>();
    double max_pos = elements[i]["max"]["position"].as<double>();

    limits_map[name] = {min_pos, max_pos};
  }

  for (size_t i = 0; i < n; ++i) {
    const std::string& joint = robot_analytical.jointnames_independent[i];

    ASSERT_TRUE(limits_map.count(joint)) << "Joint " << joint << " not found in joint_limits.yml";

    double lower = limits_map[joint].first - 0.2;
    double upper = limits_map[joint].second + 0.2;

    double r = static_cast<double>(rand()) / RAND_MAX;
    y(i) = lower + r * (upper - lower);

    yd(i) = 2.0 * (static_cast<double>(rand()) / RAND_MAX) - 1.0;
    ydd(i) = 2.0 * (static_cast<double>(rand()) / RAND_MAX) - 1.0;
  }

  robot_analytical.y = y;
  robot_analytical.yd = yd;
  robot_analytical.ydd = ydd;

  robot_numerical.y = y;
  robot_numerical.yd = yd;
  robot_numerical.ydd = ydd;

  robot_analytical.calculate_system_state();
  robot_numerical.calculate_system_state();

  ASSERT_TRUE((robot_analytical.Q - robot_numerical.Q).norm() <= epsilon_relaxed)
      << "Q mismatch\nAnalytical:\n"
      << robot_analytical.Q.transpose() << "\nNumerical:\n"
      << robot_numerical.Q.transpose();

  ASSERT_TRUE((robot_analytical.QDot - robot_numerical.QDot).norm() <= epsilon_relaxed)
      << "QDot mismatch\nAnalytical:\n"
      << robot_analytical.QDot.transpose() << "\nNumerical:\n"
      << robot_numerical.QDot.transpose();

  ASSERT_TRUE((robot_analytical.QDDot - robot_numerical.QDDot).norm() <= epsilon_relaxed)
      << "QDDot mismatch\nAnalytical:\n"
      << robot_analytical.QDDot.transpose() << "\nNumerical:\n"
      << robot_numerical.QDDot.transpose();
}
/* -------------------------------------------------------------------------- */

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
