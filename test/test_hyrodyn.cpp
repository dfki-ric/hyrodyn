#include <gtest/gtest.h>
#include <robot_model_hyrodyn.hpp>

using namespace hyrodyn;

const std::string filepath_urdf = "robot/leg/urdf/leg.urdf";
const std::string filepath_submechanisms = "robot/leg/urdf/submechanisms.yml";
const std::string filepath_jointlimits = "robot/leg/urdf/joint_limits.yml";

const std::string filepath_urdf_floatingbase =
    "robot/lower_body/urdf/leg_floatingbase.urdf";
const std::string filepath_submechanisms_floatingbase =
    "robot/lower_body/urdf/leg_submechanisms_floatingbase.yml";

const double epsilon = 1.0e-8;
const double epsilon_relaxed = 1.0e-5;

/* -------------------------------------------------------------------------- */

TEST(Hyrodyn, ForwardInverseGeometricModelConsistency)
{
    RobotModel_HyRoDyn robot;
    robot.load_robotmodel(filepath_urdf, filepath_submechanisms);

    VectorXd y = VectorNd::Zero(robot.jointnames_independent.size());
    y(0) = 0.1;
    robot.y = y;

    robot.calculate_forward_kinematics("LLAnklePitch_Link");
    robot.pose_input.push_back(robot.pose);

    std::vector<std::string> body_names = {"LLAnklePitch_Link"};
    robot.calculate_inverse_kinematics(body_names);

    ASSERT_TRUE((robot.y - y).norm() <= epsilon)
        << "Input y:\n" << y
        << "\nOutput y:\n" << robot.y
        << "\nError = " << (robot.y - y).norm();
}

/* -------------------------------------------------------------------------- */

TEST(Hyrodyn, ForwardInverseGeometricModelMultipleBodiesConsistency)
{
    RobotModel_HyRoDyn robot;
    robot.load_robotmodel(filepath_urdf, filepath_submechanisms);

    VectorXd y = VectorNd::Zero(robot.jointnames_independent.size());
    y(0) = 0.1;
    robot.y = y;

    std::vector<std::string> body_names = {
        "LLAnklePitch_Link",
        "LLKnee_Link"
    };

    robot.calculate_forward_kinematics_multiple_bodies(body_names);
    robot.pose_input = robot.poses;

    robot.y.setZero();
    robot.calculate_inverse_kinematics(body_names);

    ASSERT_TRUE((robot.y - y).norm() <= epsilon)
        << "Input y:\n" << y
        << "\nOutput y:\n" << robot.y
        << "\nError = " << (robot.y - y).norm();
}

/* -------------------------------------------------------------------------- */

TEST(Hyrodyn, ForwardKinematicsMultipleBodiesConsistency)
{
    RobotModel_HyRoDyn robot;
    robot.load_robotmodel(filepath_urdf, filepath_submechanisms);

    robot.y.setOnes();
    robot.yd.setOnes();
    robot.ydd.setOnes();

    std::vector<std::string> body_names = {
        "LLAnklePitch_Link",
        "LLKnee_Link",
        "LLHip3_Link"
    };

    robot.calculate_forward_kinematics_multiple_bodies(body_names);

    for (size_t i = 0; i < body_names.size(); ++i)
    {
        robot.calculate_forward_kinematics(body_names[i]);

        ASSERT_TRUE((robot.pose - robot.poses[i]).norm() <= epsilon);
        ASSERT_TRUE((robot.twist - robot.twists[i]).norm() <= epsilon);
        ASSERT_TRUE(
            (robot.spatial_acceleration - robot.spatial_accelerations[i]).norm()
            <= epsilon);
    }
}

/* -------------------------------------------------------------------------- */

TEST(Hyrodyn, InverseSystemStateZeroConfig)
{
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

TEST(Hyrodyn, ForwardSystemStateZeroConfig)
{
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

TEST(Hyrodyn, ForwardInverseSystemStateConsistency)
{
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

TEST(Hyrodyn, ForwardInverseDynamicModelConsistency)
{
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

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
