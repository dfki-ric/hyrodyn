#include "robot_model_hyrodyn.hpp"

#include <boost/algorithm/string.hpp>
#include <iostream>

using namespace std;
using namespace hyrodyn;

void RobotModel_HyRoDyn::welcome() {
  cout << "You successfully compiled and executed HyRoDyn Project. Welcome!" << endl;

  rbdl_check_api_version(RBDL_API_VERSION);
}

Matrix3d getRotMatfromRPY(const Vector3d& rpy) {
  // Returns the Rotation matrix (3x3) by applying Euler angles in the order:
  // roll[0], pitch[1], yaw[2]
  double roll = rpy(0);
  double pitch = rpy(1);
  double yaw = rpy(2);

  Matrix3d R_z, R_y, R_x;
  R_z << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;
  R_y << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch);
  R_x << 1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll);

  return R_z * R_y * R_x;
}

Vector3d getEulerAngles(const Quaternion& orientation) {
  // Returns the Euler angles in the order: roll[0], pitch[1], yaw[2]
  // Inspired from Determining yaw, pitch, and roll from a rotation matrix
  // (http://planning.cs.uiuc.edu/node103.html) and Pose.cpp from base-types
  const Eigen::Matrix3d m = orientation.toMatrix();
  // cout<<"Input RotMat from Quat: \n"<<m<<endl;
  double x = Eigen::Vector2d(m.coeff(2, 2), m.coeff(2, 1)).norm();  // x = sqrt(r33^2 + r32^2)
  Vector3d res(0, ::atan2(-m.coeff(2, 0), x), 0);                   // pitch = atan2(-r31, x)
  if (x > Eigen::NumTraits<double>::dummy_precision()) {
    res[2] = ::atan2(m.coeff(1, 0), m.coeff(0, 0));  // yaw = atan2(r21, r11)
    res[0] = ::atan2(m.coeff(2, 1), m.coeff(2, 2));  // roll = atan2(r32, r33)
  } else {
    res[2] = 0;
    res[0] = (m.coeff(2, 0) > 0 ? 1 : -1) * ::atan2(-m.coeff(0, 1), m.coeff(1, 1));
  }

  // cout<<"Output RotMat from RPY: \n"<<getRotMat(res)<<endl;
  res = -res;  // this produces correct derivatives of floating base coordinates
               // and lower error in inverse dynamics (not clear, why?)
  return res;
}

Vector3d getEulerAnglesDerivative(const Quaternion& orientation, const Vector3d& omega) {
  // Returns the time derivative of Euler angles in the order: yaw[0], pitch[1],
  // roll[2]

  Eigen::Vector3d rpy = getEulerAngles(orientation);
  double roll = rpy(0);
  double pitch = rpy(1);
  double yaw = rpy(2);

  Eigen::Matrix3d m;
  // Method 1: this formulation is motivated from body fixed angular velocity,
  // see https://davidbrown3.github.io/2017-07-25/EulerAngles/
  m << cos(yaw) / cos(pitch), -sin(yaw) / cos(pitch), 0, sin(yaw), cos(yaw), 0,
      -cos(yaw) * sin(pitch) / cos(pitch), sin(yaw) * sin(pitch) / cos(pitch), 1;
  // m = -m; // correction term  (it produces correct derivatives of the
  // floating base coordinates as well but error in inverse dynamics is high)

  /*	// Method 2: Inspired from Differential rotations
  (http://planning.cs.uiuc.edu/node690.html), corrected by Khalil's book m <<
  cos(yaw)/cos(pitch), sin(yaw)/cos(pitch), 0, -sin(yaw),           cos(yaw), 0,
           cos(yaw)*sin(pitch)/cos(pitch), sin(yaw)*sin(pitch)/cos(pitch), 1;
  */
  // rpy_dot = A * omega, where A is the Jacobian that maps the angular
  // velocity(omega) to RPY angular rates
  Vector3d rpy_dot = m * omega;

  return rpy_dot;
}

Vector3d getEulerAnglesSecondDerivative(const Quaternion& orientation, const Vector3d& omega,
                                        const Vector3d& omega_dot) {
  // Returns the 2nd time derivative of Euler angles in the order: yaw[0],
  // pitch[1], roll[2]
  // 1. Extract RPY angles from Quaternion representation
  Eigen::Vector3d rpy = getEulerAngles(orientation);
  double roll = rpy(0);
  double pitch = rpy(1);
  double yaw = rpy(2);
  // 2. Extract RPY angle rates from Quaternion and angular velocity
  Vector3d rpy_dot = getEulerAnglesDerivative(orientation, omega);
  double roll_dot = rpy_dot(0);
  double pitch_dot = rpy_dot(1);
  double yaw_dot = rpy_dot(2);

  // 3. Extract RPY 2nd derivative (symbolic derivative derived in MATLAB
  // symbolic toolbox of Method 1)
  double t2 = cos(pitch);
  double t3 = cos(yaw);
  double t4 = sin(pitch);
  double t5 = sin(yaw);
  double t6 = t4 * t4;
  double t7 = 1.0 / t2;
  double t8 = t7 * t7;
  Matrix3d m;
  m(0, 0) = -t5 * t7 * yaw_dot + pitch_dot * t3 * t4 * t8;
  m(0, 1) = -t3 * t7 * yaw_dot - pitch_dot * t4 * t5 * t8;
  m(0, 2) = 0.0;
  m(1, 0) = t3 * yaw_dot;
  m(1, 1) = -t5 * yaw_dot;
  m(1, 2) = 0.0;
  m(2, 0) = -pitch_dot * t3 - pitch_dot * t3 * t6 * t8 + t4 * t5 * t7 * yaw_dot;
  m(2, 1) = pitch_dot * t5 + pitch_dot * t5 * t6 * t8 + t3 * t4 * t7 * yaw_dot;
  m(2, 2) = 0.0;
  // m = -m;	// correction term (it produces correct derivatives of the
  // floating base coordinates as well but error in inverse dynamics is high)

  /*
  // additional sign changes as per lecture notes (Method 2)
  m(0,1) = -m(0,1);
  m(1,0) = -m(1,0);
  m(2,0) = -m(2,0);
  */
  // rpy_ddot = A * omega_dot + Adot * omega
  Vector3d rpy_ddot = getEulerAnglesDerivative(orientation, omega_dot) + m * omega;

  return rpy_ddot;
}

void RobotModel_HyRoDyn::update_all_independent_coordinates() {
  // This function should only be called for floating base robots.
  if (floating_base_robot) {
    // assign the floating base position vector
    y(0) = floating_robot_pose(0);
    y(1) = floating_robot_pose(1);
    y(2) = floating_robot_pose(2);

    // assign the floating base orientation vector (order in RBDL: x, y, z, w)
    Quaternion orientation(floating_robot_pose(3), floating_robot_pose(4), floating_robot_pose(5),
                           floating_robot_pose(6));
    // extract the roll-pitch-yaw angles
    Vector3d rpy = getEulerAngles(orientation);
    // assign the floating base orientation angles
    y(3) = rpy(0);
    y(4) = rpy(1);
    y(5) = rpy(2);

    // assign the linear velocity (note: linear vel is assigned to last three
    // elements of a twist vector)
    yd(0) = floating_robot_twist(3);
    yd(1) = floating_robot_twist(4);
    yd(2) = floating_robot_twist(5);
    Vector3d omega(floating_robot_twist(0), floating_robot_twist(1), floating_robot_twist(2));
    Vector3d rpy_dot = getEulerAnglesDerivative(orientation, omega);
    // note: order change
    yd(3) = rpy_dot(0);
    yd(4) = rpy_dot(1);
    yd(5) = rpy_dot(2);

    // assign the linear acceleration (note: linear accn is assigned to last
    // three elements of a spatial accn vector)
    ydd(0) = floating_robot_accn(3);
    ydd(1) = floating_robot_accn(4);
    ydd(2) = floating_robot_accn(5);
    Vector3d omega_dot(floating_robot_accn(0), floating_robot_accn(1), floating_robot_accn(2));
    Vector3d rpy_ddot = getEulerAnglesSecondDerivative(orientation, omega, omega_dot);
    // note: order change
    ydd(3) = rpy_ddot(0);
    ydd(4) = rpy_ddot(1);
    ydd(5) = rpy_ddot(2);

    // assign the rest of the indepdendent robot states
    y.tail(y_robot.size()) = y_robot;
    yd.tail(yd_robot.size()) = yd_robot;
    ydd.tail(ydd_robot.size()) = ydd_robot;
  }
}
/*
VectorXd RobotModel_HyRoDyn::getIndependentVelocities(const VectorXd&
floating_robot_pose, const SpatialVector& floating_robot_twist, const VectorXd&
yd_robot)
{
        VectorXd ydot_full(yd_robot.size()+6);

        // assign the floating base position vector
        ydot_full(0) = floating_robot_twist(0);
        ydot_full(1) = floating_robot_twist(1);
        ydot_full(2) = floating_robot_twist(2);
        // assign the floating base orientation vector (order in RBDL: x, y, z,
w) Quaternion orientation( floating_robot_pose(3), floating_robot_pose(4),
floating_robot_pose(5), floating_robot_pose(6));
        // extract the roll-pitch-yaw angles time derivatives
        Vector3d omega(floating_robot_twist(3), floating_robot_twist(4),
floating_robot_twist(5)); Vector3d rpy_dot =
getEulerAnglesDerivative(orientation, omega);
        // assign the orientation angle derivatives to the floating joint
velocities ydot_full(3) = rpy_dot(0); ydot_full(4) = rpy_dot(1); ydot_full(5) =
rpy_dot(2);
        // assign the rest of the indepdendent robot states
        cout<<yd_robot<<endl;
        ydot_full.tail(yd_robot.size()) = yd_robot;

    return ydot_full;
}
*/
void RobotModel_HyRoDyn::load_submechanisms_yaml(string filepath, bool verbose) {
  YAML::Node doc = YAML::LoadFile(filepath.c_str());

  std::size_t last_sep_pos = filepath.find_last_of('/');
  std::string yaml_dir;

  // find_last_of returns npos if no separator is found
  if (last_sep_pos == std::string::npos) {
    yaml_dir = "";
  } else {
    yaml_dir = filepath.substr(0, last_sep_pos) + '/';
  }

  if (not doc["submechanisms"]) {
    throw std::invalid_argument("Yaml parsing error: File " + filepath +
                                " either doesn't exist or is not a valid submechanisms file");
  }

  const YAML::Node& submechanisms_node = doc["submechanisms"];
  cout << "Number of submechanisms found: " << submechanisms_node.size() << endl;

  if (submechanisms_node.size() == 0) {
    cerr << "Submechanisms definition empty. Aborting. Please define the "
            "submechanisms properly."
         << endl;
    abort();
  }

  for (uint i = 0; i < submechanisms_node.size(); i++) {
    AssembleinHyRoDyn::submechanism submech;
    std::vector<string> _jointnames_independent, _jointnames_spanningtree, _jointnames_active,
        _jointnames;

    if (submechanisms_node[i]["name"]) {
      submech.name = submechanisms_node[i]["name"].as<std::string>();
    } else {
      submech.name = "Not Provided.";
    }

    submech.type = submechanisms_node[i]["type"].as<std::string>();

    if (submechanisms_node[i]["contextual_name"]) {
      submech.contextual_name = submechanisms_node[i]["contextual_name"].as<std::string>();
    } else {
      cerr << "Contextual name of the submechanism at i = " << i << " is not provided. Aborting!"
           << endl;
      abort();
    }

    if (submechanisms_node[i]["file_path"]) {
      submech.file_path = yaml_dir + submechanisms_node[i]["file_path"].as<std::string>();
    } else {
      submech.file_path = "";
    }

    _jointnames_spanningtree =
        submechanisms_node[i]["jointnames_spanningtree"].as<std::vector<std::string>>();
    submech.jointnames_spanningtree = _jointnames_spanningtree;

    _jointnames_active = submechanisms_node[i]["jointnames_active"].as<std::vector<std::string>>();
    submech.jointnames_active = _jointnames_active;

    _jointnames_independent =
        submechanisms_node[i]["jointnames_independent"].as<std::vector<std::string>>();
    submech.jointnames_independent = _jointnames_independent;

    if (submechanisms_node[i]["jointnames"]) {
      _jointnames = submechanisms_node[i]["jointnames"].as<std::vector<std::string>>();
      submech.jointnames = _jointnames;
    }

    // if active joint name list is not empty i.e. it is not a free joint,
    // independent joints defined belongs to the robot.
    if (!submech.jointnames_active.empty())
      submech.jointnames_independent_robot = _jointnames_independent;
    else {
      cout << "Free flyer joint detected at submechanism index: " << i << endl;
      floating_base_robot = true;
      if (i != 0) {
        cerr << "Free flyer joint is allowed only at index 0." << endl;
        abort();
      }
    }
    // jointnames vector respecting modular link enumeration scheme
    for (unsigned char i = 0; i < _jointnames_spanningtree.size(); i++)
      jointnames_spanningtree.push_back(_jointnames_spanningtree[i]);

    for (unsigned char i = 0; i < _jointnames_active.size(); i++)
      jointnames_active.push_back(_jointnames_active[i]);

    for (unsigned char i = 0; i < _jointnames_independent.size(); i++)
      jointnames_independent.push_back(_jointnames_independent[i]);

    for (unsigned char i = 0; i < _jointnames.size(); i++) jointnames.push_back(_jointnames[i]);

    if (submech.type.compare("NUMERICAL") == 0) {
      if (submechanisms_node[i]["loop_constraints"]) {
        // Parse loop constraints in the submechanisms.yml file
        const YAML::Node& loop_constraint_node = submechanisms_node[i]["loop_constraints"];

        for (uint j = 0; j < loop_constraint_node.size(); j++) {
          // Added by Rohit Kumar
          NUMERICALLOOPCONSTRAINTS::Loop_constraints lcs;
          if (loop_constraint_node[j]["cut_joint"]) {
            lcs.jointname_cut = loop_constraint_node[j]["cut_joint"].as<std::string>();
          } else {
            cerr << "Name of the cut joint is not provided. Aborting!" << endl;
            abort();
          }

          if (loop_constraint_node[j]["predecessor_body"]) {
            lcs.pred_body = loop_constraint_node[j]["predecessor_body"].as<std::string>();
          } else {
            cerr << "Name of the predecessor body is not provided. Aborting!" << endl;
            abort();
          }

          if (loop_constraint_node[j]["successor_body"]) {
            lcs.succ_body = loop_constraint_node[j]["successor_body"].as<std::string>();
          } else {
            cerr << "Name of the predecessor body is not provided. Aborting!" << endl;
            abort();
          }

          const YAML::Node& constraint_dof_node = loop_constraint_node[j]["constraint_axes"];
          for (uint k = 0; k < constraint_dof_node.size(); k++) {
            NUMERICALLOOPCONSTRAINTS::Constraint_dof cons_dof;
            std::vector<float> spatial_vector;

            if (constraint_dof_node[k]["name"]) {
              cons_dof.name = constraint_dof_node[k]["name"].as<std::string>();
            } else {
              cons_dof.name = "Not provided";
            }
            if (constraint_dof_node[k]["axis"]) {
              spatial_vector = constraint_dof_node[k]["axis"].as<std::vector<float>>();
              cons_dof.axis =
                  SpatialVector(spatial_vector[0], spatial_vector[1], spatial_vector[2],
                                spatial_vector[3], spatial_vector[4], spatial_vector[5]);
            } else {
              cerr << "Constraint axis missing. Please check the input." << endl;
              abort();
            }
            if (constraint_dof_node[k]["baumgarte_stabilization_parameter"]) {
              cons_dof.baumg_stab_param =
                  constraint_dof_node[k]["baumgarte_stabilization_parameter"].as<double>();
            } else {
              cons_dof.baumg_stab_param = 0.00001;
            }
            lcs.constraint_axes.push_back(cons_dof);
          }
          submech.loop_constraints_submech.push_back(lcs);
        }
      } else {
        cout << "Loop contraints not provided. Serial submechanism is "
                "considered."
             << endl;
      }
    }
    assembly.push_back(submech);
  }

  // Parse the exoskeletons in the submechanisms.yml file

  const YAML::Node& exoskeletons_node = doc["exoskeletons"];

  cout << "Number of exoskeletons found: " << exoskeletons_node.size() << endl;

  for (uint i = 0; i < exoskeletons_node.size(); i++) {
    AssembleinHyRoDyn::exoskeleton exo;
    std::vector<string> _jointnames_dependent, _jointnames_spanningtree, _jointnames;

    if (exoskeletons_node[i]["name"]) {
      exo.name = exoskeletons_node[i]["name"].as<std::string>();
    } else {
      cerr << "Name of the exoskeleton is not provided. Aborting!" << endl;
      abort();
    }

    exo.around = exoskeletons_node[i]["around"].as<std::string>();

    if (exoskeletons_node[i]["file_path"]) {
      exo.file_path = yaml_dir + exoskeletons_node[i]["file_path"].as<std::string>();
    } else {
      cerr << "File path of the exoskeleton is not provided. Aborting!" << endl;
      abort();
    }

    _jointnames_spanningtree =
        exoskeletons_node[i]["jointnames_spanningtree"].as<std::vector<std::string>>();
    exo.jointnames_spanningtree = _jointnames_spanningtree;

    _jointnames_dependent =
        exoskeletons_node[i]["jointnames_dependent"].as<std::vector<std::string>>();
    exo.jointnames_dependent = _jointnames_dependent;

    if (exoskeletons_node[i]["jointnames"]) {
      _jointnames = exoskeletons_node[i]["jointnames"].as<std::vector<std::string>>();
      exo.jointnames = _jointnames;
    }

    // jointnames vector respecting modular link enumeration scheme
    for (unsigned char i = 0; i < _jointnames_spanningtree.size(); i++)
      jointnames_spanningtree.push_back(_jointnames_spanningtree[i]);

    for (unsigned char i = 0; i < _jointnames.size(); i++) jointnames.push_back(_jointnames[i]);

    exteriors.push_back(exo);
  }

  // jointnames_spanningtree = jointnames;

  if (verbose) {
    // Print the submechanism details
    for (unsigned int i = 0; i < assembly.size(); i++) assembly[i].print_submechanism_details();

    // Print the exoskeleton details
    for (unsigned int i = 0; i < exteriors.size(); i++) exteriors[i].print_exoskeleton_details();

    cout << "Modularly enumerated Joint Names in the Spanning Tree: " << endl;
    cout << "Size: " << jointnames_spanningtree.size() << endl;
    for (unsigned int i = 0; i < jointnames_spanningtree.size(); i++)
      cout << jointnames_spanningtree[i] << endl;

    cout << "Modularly enumerated Active Joint Names: " << endl;
    cout << "Size: " << jointnames_active.size() << endl;
    for (unsigned int i = 0; i < jointnames_active.size(); i++)
      cout << jointnames_active[i] << endl;

    cout << "Modularly enumerated Independent Joint Names: " << endl;
    cout << "Size: " << jointnames_independent.size() << endl;
    for (unsigned int i = 0; i < jointnames_independent.size(); i++)
      cout << jointnames_independent[i] << endl;

    // if jointnames is not defined in the submechanisms, assign the spanning
    // tree joint names to the jointnames and use the hyrodyn fixed joint
    // processing in the rbdl parser (Backward Compatibility).
    if (jointnames.empty()) {
      cout << "jointnames vector including the fixed joints is not initialized "
              "in the submechanisms file. HyRoDyn will attempt automatic fixed "
              "joint processing!"
           << endl;
      jointnames = jointnames_spanningtree;
    } else {
      cout << "Modularly enumerated Joint Names (including fixed joints): " << endl;
      cout << "Size: " << jointnames.size() << endl;
      for (unsigned int i = 0; i < jointnames.size(); i++) cout << jointnames[i] << endl;
    }
  }
}

void RobotModel_HyRoDyn::set_gravity_vector(Vector3d gravity_vector) { m.gravity = gravity_vector; }

void RobotModel_HyRoDyn::calculate_system_state() {
  HyRoDyn::calc_sysstate_q(m, *elcs, y, Q);
  // HyRoDyn::calc_actuatorstate_u (m, *elcs, y, u);
  u = elcs->get_permutation_matrix() * Q;
  // cout<<"q: "<<endl<<Q.transpose()<<endl;
  // std::cout << "u for system: " << endl << u.transpose() <<endl;

  // HyRoDyn::calc_sysstate_qdot (m, *elcs, y, yd, QDot);
  // HyRoDyn::calc_actuatorstate_udot (m, *elcs, y, yd, ud);
  MatrixXd G(elcs->get_dof_spanningtree(), elcs->get_dof_independent());
  G = elcs->calc_loopclosure_Jacobian(y);
  QDot = G * yd;
  ud = elcs->get_permutation_matrix() * QDot;
  // cout<<"qd: "<<endl<<QDot.transpose()<<endl;
  // std::cout << "ud for system: " << endl << ud.transpose() <<endl;

  // HyRoDyn::calc_sysstate_qddot (m, *elcs, y, yd, ydd, QDDot);
  // HyRoDyn::calc_actuatorstate_uddot (m, *elcs, y, yd, ydd, udd);

  VectorXd g(elcs->get_dof_spanningtree());
  g = elcs->calc_loopclosure_g(y, yd);
  QDDot = G * ydd + g;

  udd = elcs->get_permutation_matrix() * QDDot;
  // cout<<"qdd: "<<endl<<QDDot.transpose()<<endl;
  // std::cout << "udd for system: " << endl << udd.transpose() <<endl;
}

void RobotModel_HyRoDyn::calculate_condition_number(string body_name, bool for_point_Jacobian) {
  const char* bodyname = body_name.c_str();
  unsigned int dim = 6;

  if (for_point_Jacobian)
    dim = 3;

  MatrixXd Gu;
  Gu.setZero(elcs->get_dof_active(), elcs->get_dof_active());

  MatrixNd G(elcs->get_dof_spanningtree(), elcs->get_dof_independent());
  G = elcs->calc_loopclosure_Jacobian(y);

  Gu = elcs->get_permutation_matrix() * G;

  MatrixXd J;
  J.setZero(dim, elcs->get_dof_spanningtree());

  if (!for_point_Jacobian)
    HyRoDyn::calc_spatial_jacobian(m, *elcs, y, J, bodyname);
  else
    HyRoDyn::calc_point_jacobian(m, *elcs, y, J, bodyname);

  MatrixXd JG;
  JG.setZero(dim, elcs->get_dof_independent());

  JG = J * G;

  MatrixXd JGGu;
  JGGu.setZero(dim, elcs->get_dof_active());

  JGGu = JG * Gu.inverse();

  Eigen::JacobiSVD<MatrixXd> svd1(JG);

  double cond1 = svd1.singularValues()(0) / svd1.singularValues()(svd1.singularValues().size() - 1);
  inv_cond1 = 1.0 / cond1;
  //	cout<<"Inverse of Condition Number 1: "<<inv_cond1<<endl;

  Eigen::JacobiSVD<MatrixXd> svd2(JGGu);

  double cond2 = svd2.singularValues()(0) / svd2.singularValues()(svd2.singularValues().size() - 1);
  inv_cond2 = 1.0 / cond2;
  //	cout<<"Inverse of Condition Number 2: "<<inv_cond2<<endl;
}

void RobotModel_HyRoDyn::calculate_condition_number_loop_closure() {
  MatrixXd Gu;
  Gu.setZero(elcs->get_dof_active(), elcs->get_dof_active());

  MatrixNd G(elcs->get_dof_spanningtree(),
             elcs->get_dof_independent());  // (n x m) matrix
  G = elcs->calc_loopclosure_Jacobian(y);   // qdot = G*ydot

  Gu = elcs->get_permutation_matrix() * G;

  MatrixXd GGu;
  GGu.setZero(elcs->get_dof_spanningtree(),
              elcs->get_dof_active());  // (n x p) matrix

  GGu = G * Gu.inverse();  // ydot = Gu*udot

  Eigen::JacobiSVD<MatrixXd> svd1(G);

  double cond1 = svd1.singularValues()(0) / svd1.singularValues()(svd1.singularValues().size() - 1);
  inv_cond_loop_closure_independent_jointspace = 1.0 / cond1;
  //	cout<<"Inverse of condition number of G:
  //"<<inv_cond_loop_closure_independent_jointspace<<endl;

  Eigen::JacobiSVD<MatrixXd> svd2(GGu);

  double cond2 = svd2.singularValues()(0) / svd2.singularValues()(svd2.singularValues().size() - 1);
  inv_cond_loop_closure_active_jointspace = 1.0 / cond2;
  //	cout<<"Inverse of condition number of G*Gu_inv:
  //"<<inv_cond_loop_closure_active_jointspace<<endl;
}

void RobotModel_HyRoDyn::calculate_forward_system_state() {
  // forward system state means actuator state to full system state

  // compute forward position kinematics
  // HyRoDyn::calc_independentjointstate_y (m, *elcs, u, y);
  // compute forward velocities
  // HyRoDyn::calc_independentjointstate_ydot (m, *elcs, u, ud, y, yd);
  // compute forward acceleration
  HyRoDyn::calc_independentjointstate_yddot(m, *elcs, u, ud, udd, y, yd,
                                            ydd);  // computes y, yd, ydd (faster way)
  HyRoDyn::calc_sysstate_q(m, *elcs, y, Q);
}

void RobotModel_HyRoDyn::calculate_inverse_dynamics() {
  Tau_actuated = HyRoDyn::calc_dynamicmodel_inverse(m, *elcs, y, yd, ydd);
  // std::cout << "Tau actuated for system: " << Tau_actuated.transpose() <<
  // std::endl;
}

void RobotModel_HyRoDyn::calculate_inverse_dynamics_including_floatingbase() {
  Tau_actuated_floatingbase =
      HyRoDyn::calc_dynamicmodel_inverse_including_floatingbase(m, *elcs, y, yd, ydd);
  // std::cout << "Tau actuated for system: " << Tau_actuated.transpose() <<
  // std::endl;
}

void RobotModel_HyRoDyn::calculate_inverse_dynamics_independentjointspace() {
  Tau_independentjointspace =
      HyRoDyn::calc_dynamicmodel_inverse_independentjointspace(m, *elcs, y, yd, ydd);
  // std::cout << "Tau independent for system: " <<
  // Tau_independentjointspace.transpose() << std::endl;
}

void RobotModel_HyRoDyn::calculate_forward_dynamics() {
  ydd = HyRoDyn::calc_dynamicmodel_forward(m, *elcs, y, yd, Tau_actuated);

  // VectorXd g(elcs->get_dof_spanningtree());
  // g = elcs->calc_loopclosure_g(y, yd);
  // QDDot = elcs->calc_loopclosure_Jacobian(y) * ydd + g;
  // std::cout << "Joint acceleration for system: " <<
  //   QDDot.transpose() << std::endl;

  // QDot = elcs->calc_loopclosure_Jacobian(y) * yd;
  // std::cout << "Joint velocities for system: " <<
  //   QDot.transpose() << std::endl;

  // Q = elcs->calc_loopclosure_function(y);
  // if (Q[0] > 1.57) {
  //   std::cout << "Joint positions for system: " << Q.transpose() <<
  //   std::endl;
  // }
  // if (Q[0] < -1.57) {
  //   std::cout << "Joint positions for system: " << Q.transpose() <<
  //   std::endl;
  // }
  // G = elcs->calc_loopclosure_Jacobian(y);
  // QDot = G * yd;
  // ud = elcs->get_permutation_matrix() * QDot;
  // // cout<<"qd: "<<endl<<QDot.transpose()<<endl;
  // // std::cout << "ud for system: " << endl << ud.transpose() <<endl;

  // // HyRoDyn::calc_sysstate_qddot (m, *elcs, y, yd, ydd, QDDot);
  // // HyRoDyn::calc_actuatorstate_uddot (m, *elcs, y, yd, ydd, udd);

  // VectorXd g(elcs->get_dof_spanningtree());
  // g = elcs->calc_loopclosure_g(y, yd);
  // QDDot = G * ydd + g;
  // std::cout << "Independent joint acceleration for system: " <<
  // ydd.transpose() << std::endl;
}

void RobotModel_HyRoDyn::calculate_mass_interia_matrix() {
  // compute the mass-inertia matrix projected to independent joint space
  HyRoDyn::calc_mass_interia_matrix(m, *elcs, y, H);
}

void RobotModel_HyRoDyn::calculate_mass_interia_matrix_actuation_space() {
  // compute the mass-inertia matrix projected to actuation space
  HyRoDyn::calc_mass_interia_matrix_actuation_space(m, *elcs, y, Hu);
}

void RobotModel_HyRoDyn::calculate_mass_interia_matrix_actuation_space_including_floatingbase() {
  // compute the mass-inertia matrix projected to actuation space
  HyRoDyn::calc_mass_interia_matrix_actuation_space_including_floating_base(m, *elcs, y, Hufb);
}

void RobotModel_HyRoDyn::calculate_nle_actuation_space() {
  // compute the nonlinear effects (Coriolis + gravity) projected to actuation space
  HyRoDyn::calc_nonlinear_effects_actuation_space(m, *elcs, y, yd, Cu);
}

void RobotModel_HyRoDyn::calculate_simplified_inverse_dynamics() {
  MatrixXd Gu;
  Gu.setZero(elcs->get_dof_active(), elcs->get_dof_active());

  HyRoDyn::calc_actuator_jacobian(m, *elcs, y, Gu);

  // u = Gu_inv.transpose() * Tau_y_r
  Tau_actuated = (Gu.transpose())
                     .colPivHouseholderQr()
                     .solve(elcs->get_permutation_matrix2() * Tau_independentjointspace_input);
}

void RobotModel_HyRoDyn::calculate_inverse_statics() {
  Tau_actuated_ext = HyRoDyn::calc_staticmodel_inverse(m, *elcs, y, wrench_points, f_ext,
                                                       wrench_resolution, wrench_interaction);
  // std::cout << "Tau actuated external for system: " <<
  // Tau_actuated_ext.transpose() << std::endl;
}

void RobotModel_HyRoDyn::calculate_forward_kinematics(string body_name) {
  const char* bodyname = body_name.c_str();

  pose = HyRoDyn::calc_geometricmodel_forward(m, *elcs, y, bodyname);

  //    std::cout << "Pose of shank link(robot model hyrodyn): " <<
  //    pose.transpose() << std::endl;

  twist = HyRoDyn::calc_kinematicmodel_forward(m, *elcs, y, yd, bodyname);

  //    std::cout << "Twist of shank link: " << twist.transpose() << std::endl;

  spatial_acceleration =
      HyRoDyn::calc_secondorder_kinematicmodel_forward(m, *elcs, y, yd, ydd, bodyname);

  //    std::cout << "Spatial Acceleration of shank link: " <<
  //    spatial_acceleration.transpose() << std::endl;
}

void RobotModel_HyRoDyn::calculate_forward_kinematics_multiple_bodies(
    std::vector<string> body_names) {
  poses = HyRoDyn::calc_geometricmodel_forward(m, *elcs, y, body_names);

  twists = HyRoDyn::calc_kinematicmodel_forward(m, *elcs, y, yd, body_names);

  spatial_accelerations =
      HyRoDyn::calc_secondorder_kinematicmodel_forward(m, *elcs, y, yd, ydd, body_names);
}

void RobotModel_HyRoDyn::calculate_spatial_acceleration_bias(string body_name) {
  const char* bodyname = body_name.c_str();
  spatial_acceleration_bias = HyRoDyn::calc_secondorder_kinematicmodel_forward(
      m, *elcs, y, yd, VectorNd::Zero(jointnames_independent.size()), bodyname);
}

void RobotModel_HyRoDyn::calculate_inverse_kinematics(std::vector<string> body_names) {
  //	const char* bodyname = body_name.c_str();

  HyRoDyn::calc_geometricmodel_inverse(m, *elcs, pose_input, body_names, y, com_input,
                                       ik_error_tolerance);
}

void RobotModel_HyRoDyn::calculate_com_properties() {
  HyRoDyn::calc_com_properties(m, *elcs, y, yd, &ydd, mass, com, &com_vel, &com_acc,
                               &com_angularmomentum, &com_angularmomentum_derivative);
}

void RobotModel_HyRoDyn::calculate_com_jacobian() { HyRoDyn::calc_com_jacobian(m, *elcs, y, Jcom); }

void RobotModel_HyRoDyn::calculate_body_jacobian(string body_name) {
  const char* bodyname = body_name.c_str();

  HyRoDyn::calc_body_jacobian_independent_joint_space(m, *elcs, y, Jb, bodyname);
}

void RobotModel_HyRoDyn::calculate_space_jacobian(string body_name) {
  const char* bodyname = body_name.c_str();

  HyRoDyn::calc_spatial_jacobian_independent_joint_space(m, *elcs, y, Js, bodyname);
}

void RobotModel_HyRoDyn::calculate_body_jacobian_actuation_space(string body_name) {
  const char* bodyname = body_name.c_str();

  HyRoDyn::calc_body_jacobian_actuation_space(m, *elcs, y, Jbu, bodyname);
}

void RobotModel_HyRoDyn::calculate_space_jacobian_actuation_space(string body_name) {
  const char* bodyname = body_name.c_str();

  HyRoDyn::calc_spatial_jacobian_actuation_space(m, *elcs, y, Jsu, bodyname);
}

void RobotModel_HyRoDyn::calculate_body_jacobian_actuation_space_including_floatingbase(
    string body_name) {
  const char* bodyname = body_name.c_str();

  HyRoDyn::calc_body_jacobian_actuation_space_including_floating_base(m, *elcs, y, Jbufb, bodyname);
}

void RobotModel_HyRoDyn::calculate_space_jacobian_actuation_space_including_floatingbase(
    string body_name) {
  const char* bodyname = body_name.c_str();

  HyRoDyn::calc_spatial_jacobian_actuation_space_including_floating_base(m, *elcs, y, Jsufb,
                                                                         bodyname);
}

void RobotModel_HyRoDyn::calculate_zmp() {
  HyRoDyn::calc_zero_moment_point(m, *elcs, y, yd, ydd, contact_surface_normal,
                                  contact_surface_point, &zmp);
}

void RobotModel_HyRoDyn::calculate_constrained_inverse_dynamics() {
  HyRoDyn::calc_constrained_dynamicmodel_inverse(m, *elcs, y, yd, twist, f_ext[0],
                                                 wrench_points[0]);
}

void RobotModel_HyRoDyn::calculate_adjoint_transformation(string body_name) {
  const char* bodyname = body_name.c_str();

  adjoint_transformation_base = HyRoDyn::calc_adjoint_transformation(m, *elcs, y, bodyname);
}

double RobotModel_HyRoDyn::calculate_kinetic_energy() {
  return HyRoDyn::calc_kinetic_energy(m, Q, QDot);
}

double RobotModel_HyRoDyn::calculate_potential_energy() {
  return HyRoDyn::calc_potential_energy(m, Q);
}

double RobotModel_HyRoDyn::calculate_total_energy() {
  return calculate_kinetic_energy() + calculate_potential_energy();
}

void RobotModel_HyRoDyn::load_robotmodel(string filepath_urdf, string filepath_submechanisms,
                                         bool verbose) {
  load_submechanisms_yaml(filepath_submechanisms, verbose);

  if (!Addons::URDFReadFromFileWithModularity(filepath_urdf.c_str(), &m, jointnames, false,
                                              false)) {
    std::cerr << "Error loading robot model from urdf" << std::endl;
    abort();
  }
  // cout << "Full spanning tree URDF loaded successfully!" << endl;

  elcs = (new AssembleinHyRoDyn::SubmechanismsAssembly(assembly, exteriors, verbose));

  spanningtree_dof = jointnames_spanningtree.size();
  active_dof = jointnames_active.size();
  independent_dof = jointnames_independent.size();
  floatingbase_dof = independent_dof - elcs->get_dof_independent_robot();

  Q = VectorNd::Zero(spanningtree_dof);
  QDot = VectorNd::Zero(spanningtree_dof);
  QDDot = VectorNd::Zero(spanningtree_dof);

  double mass;
  Vector3d com;
  Utils::CalcCenterOfMass(m, Q, QDot, NULL, mass, com);

  y_zero = VectorNd::Zero(independent_dof);
  VectorNd yd_zero = VectorNd::Zero(independent_dof);
  VectorNd ydd_zero = VectorNd::Zero(independent_dof);
  VectorNd Tau_actuated_zero = VectorNd::Zero(active_dof);

  if (verbose) {
    cout << "Active DoF count: " << active_dof << endl;
    cout << "Independent DoF count: " << independent_dof << endl;

    cout << "Model DoF overview:" << Utils::GetModelDOFOverview(m) << endl;
    cout << "Model Hierarchy overview:" << Utils::GetModelHierarchy(m) << endl;
    cout << "Named Body Origins overview:" << Utils::GetNamedBodyOriginsOverview(m) << endl;
    cout << "Gravity: " << endl << m.gravity << endl;
    cout << "Spanning tree DoF count: " << m.dof_count << endl;
    cout << "Total moving mass :" << mass << " Overall COM of the moving bodies:" << com.transpose()
         << endl;

    cout << "Number of submechanisms in the robot: "
         << dynamic_cast<AssembleinHyRoDyn::SubmechanismsAssembly*>(elcs)->assembly.size() << endl;
  }

  for (unsigned int i = 0;
       i < dynamic_cast<AssembleinHyRoDyn::SubmechanismsAssembly*>(elcs)->assembly.size(); i++) {
    submechanism_active_dof_distribution.push_back(
        dynamic_cast<AssembleinHyRoDyn::SubmechanismsAssembly*>(elcs)
            ->assembly[i]
            .jointnames_active.size());
    submechanism_spanningtree_dof_distribution.push_back(
        dynamic_cast<AssembleinHyRoDyn::SubmechanismsAssembly*>(elcs)
            ->assembly[i]
            .jointnames_spanningtree.size());
    submechanism_independent_dof_distribution.push_back(
        dynamic_cast<AssembleinHyRoDyn::SubmechanismsAssembly*>(elcs)
            ->assembly[i]
            .jointnames_independent.size());
  }

  cout << "Gamma: " << endl << elcs->calc_loopclosure_function(y_zero).transpose() << endl;

  cout << "G: " << endl << elcs->calc_loopclosure_Jacobian(y_zero) << endl;

  cout << "g: " << endl << elcs->calc_loopclosure_g(y_zero, yd_zero).transpose() << endl;

  Tau_actuated_zero = HyRoDyn::calc_dynamicmodel_inverse(m, *elcs, y_zero, yd_zero, ydd_zero);
  std::cout << "Tau actuated for system: " << Tau_actuated_zero.transpose() << std::endl;
  if (!floating_base_robot)
    std::cout << "ydd for system: " << endl
              << HyRoDyn::calc_dynamicmodel_forward(m, *elcs, y_zero, yd_zero, Tau_actuated_zero)
                     .transpose()
              << endl;

  y = VectorNd::Zero(independent_dof);
  yd = VectorNd::Zero(independent_dof);
  ydd = VectorNd::Zero(independent_dof);
  Tau_independentjointspace = VectorNd::Zero(independent_dof);

  u = VectorNd::Zero(active_dof);
  ud = VectorNd::Zero(active_dof);
  udd = VectorNd::Zero(active_dof);
  Tau_actuated = VectorNd::Zero(active_dof);
  Tau_actuated_ext = VectorNd::Zero(active_dof);

  y_robot = VectorNd::Zero(active_dof);
  yd_robot = VectorNd::Zero(active_dof);
  ydd_robot = VectorNd::Zero(active_dof);
  floating_robot_pose = VectorNd::Zero(7);

  Jb = MatrixNd::Zero(6, independent_dof);
  Js = MatrixNd::Zero(6, independent_dof);
  Jbu = MatrixNd::Zero(6, active_dof);
  Jsu = MatrixNd::Zero(6, active_dof);
  Jbufb = MatrixNd::Zero(6, floatingbase_dof + active_dof);
  Jsufb = MatrixNd::Zero(6, floatingbase_dof + active_dof);

  Jcom = MatrixNd::Zero(3, independent_dof);
  H = MatrixNd::Zero(independent_dof, independent_dof);
  Hu = MatrixNd::Zero(active_dof, active_dof);
  Hufb = MatrixNd::Zero(floatingbase_dof + active_dof, floatingbase_dof + active_dof);
  Cu = VectorNd::Zero(active_dof);

  HyRoDyn::calc_actuatorstate_u(m, *elcs, y_zero, u);
  std::cout << "u for system: " << endl << u.transpose() << endl;

  HyRoDyn::calc_actuatorstate_udot(m, *elcs, y_zero, yd_zero, ud);
  std::cout << "ud for system: " << endl << ud.transpose() << endl;

  HyRoDyn::calc_actuatorstate_uddot(m, *elcs, y_zero, yd_zero, ydd_zero, ud);
  std::cout << "udd for system: " << endl << udd.transpose() << endl;

  pose = VectorNd::Zero(7);
  std::cout << "pose(should be zero vector): " << endl << pose.transpose() << endl;
}

void RobotModel_HyRoDyn::load_filepathes_from_smurf(string filepath_smurf, string& filepath_urdf,
                                                    string& filepath_submechanisms) {
  YAML::Node smurf = YAML::LoadFile(filepath_smurf.c_str());

  std::size_t last_sep_pos = filepath_smurf.find_last_of('/');
  std::string yaml_dir;

  // find_last_of returns npos if no separator is found
  if (last_sep_pos == std::string::npos) {
    yaml_dir = "";
  } else {
    yaml_dir = filepath_smurf.substr(0, last_sep_pos) + '/';
  }

  if (not smurf["files"]) {
    throw std::invalid_argument("Yaml parsing error: File " + filepath_smurf +
                                " is no valid SMURF file");
  }

  const YAML::Node& files = smurf["files"];

  for (uint i = 0; i < files.size(); i++) {
    string ending = ".urdf";
    string lc_file = files[i].as<std::string>();
    boost::to_lower(lc_file);
    if (filepath_urdf.empty() && lc_file.length() >= ending.length() &&
        (0 == lc_file.compare(lc_file.length() - ending.length(), ending.length(), ending))) {
      filepath_urdf = yaml_dir + files[i].as<std::string>();
    } else if (filepath_submechanisms.empty()) {
      try {
        YAML::Node doc = YAML::LoadFile(yaml_dir + files[i].as<std::string>().c_str());
        if (not doc["submechanisms"]) {
          continue;
        } else {
          filepath_submechanisms = yaml_dir + files[i].as<std::string>();
        }
      } catch (const char* message) {
      }
    }
  }

  if (filepath_urdf.empty()) {
    cerr << "SMURF without URDF given!" << endl;
    abort();
  }
  if (filepath_submechanisms.empty()) {
    cerr << "SMURF without submechanisms given!" << endl;
    abort();
  }
}

void RobotModel_HyRoDyn::load_smurf(string filepath_smurf) {
  string filepath_urdf;
  string filepath_submechanisms;
  load_filepathes_from_smurf(filepath_smurf, filepath_urdf, filepath_submechanisms);
  this->load_robotmodel(filepath_urdf, filepath_submechanisms);
}
