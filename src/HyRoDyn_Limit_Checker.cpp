#include "HyRoDyn_Limit_Checker.hpp"

HyRoDyn_Limit_Checker::HyRoDyn_Limit_Checker(string filepath_urdf,
                                             string filepath_submechanisms) {

  robot_model = new RobotModel_HyRoDyn;

  // creates the robot model in hyrodyn
  robot_model->load_robotmodel(filepath_urdf, filepath_submechanisms);

  VectorXd vel_limit;
  VectorXd effort_limit;

  if (!Addons::URDFReadJointLimits(filepath_urdf.c_str(),
                                   robot_model->jointnames_spanningtree, q_max,
                                   q_min, vel_limit, effort_limit)) {
    std::cerr << "Error loading robot model from urdf" << std::endl;
    abort();
  }

  // assign velocity limits for the spanning tree joints
  qd_min = -vel_limit;
  qd_max = vel_limit;

  // assign torque/force limits for the spanning tree joints
  Tau_spanningtree_min = -effort_limit;
  Tau_spanningtree_max = effort_limit;

  // Extract position, velocity and torque limits for independent joints
  y_min.setZero(robot_model->jointnames_independent.size());
  y_max.setZero(robot_model->jointnames_independent.size());
  yd_min.setZero(robot_model->jointnames_independent.size());
  yd_max.setZero(robot_model->jointnames_independent.size());
  Tau_independent_min.setZero(robot_model->jointnames_independent.size());
  Tau_independent_max.setZero(robot_model->jointnames_independent.size());

  cout << "Spanning Tree Joint Limits Data: " << endl;
  for (unsigned int i = 0; i < robot_model->jointnames_spanningtree.size();
       i++) {
    cout << "Joint Name: " << robot_model->jointnames_spanningtree[i] << endl;
    cout << "Pos min: " << q_min(i) << " Pos max: " << q_max(i) << endl;
    cout << "Vel min: " << qd_min(i) << " Vel max: " << qd_max(i) << endl;
    cout << "Tau min: " << Tau_spanningtree_min(i)
         << " Tau max: " << Tau_spanningtree_max(i) << endl;
  }

  cout << "Independent Joint Limits Data: " << endl;
  for (unsigned int i = 0; i < robot_model->jointnames_independent.size();
       i++) {
    cout << "Joint Name: " << robot_model->jointnames_independent[i] << endl;
    auto it = std::find(robot_model->jointnames_spanningtree.begin(),
                        robot_model->jointnames_spanningtree.end(),
                        robot_model->jointnames_independent[i]);
    unsigned int index =
        std::distance(robot_model->jointnames_spanningtree.begin(), it);
    // extract independent joint position limits
    y_min(i) = q_min(index);
    y_max(i) = q_max(index);
    cout << "Pos min: " << y_min(i) << " Pos max: " << y_max(i) << endl;
    // extract independent joint velocity limits
    yd_min(i) = qd_min(index);
    yd_max(i) = qd_max(index);
    cout << "Vel min: " << yd_min(i) << " Vel max: " << yd_max(i) << endl;
    // extract independent joint force/torque limits
    Tau_independent_min(i) = Tau_spanningtree_min(index);
    Tau_independent_max(i) = Tau_spanningtree_max(index);
    cout << "Tau min: " << Tau_independent_min(i)
         << " Tau max: " << Tau_independent_max(i) << endl;
  }

  // Extract position, velocity and torque limits for active joints
  u_min.setZero(robot_model->jointnames_active.size());
  u_max.setZero(robot_model->jointnames_active.size());
  ud_min.setZero(robot_model->jointnames_active.size());
  ud_max.setZero(robot_model->jointnames_active.size());
  Tau_actuated_min.setZero(robot_model->jointnames_active.size());
  Tau_actuated_max.setZero(robot_model->jointnames_active.size());

  cout << "Actuator Limits Data: " << endl;
  for (unsigned int i = 0; i < robot_model->jointnames_active.size(); i++) {
    cout << "Actuator Name: " << robot_model->jointnames_active[i] << endl;
    auto it = std::find(robot_model->jointnames_spanningtree.begin(),
                        robot_model->jointnames_spanningtree.end(),
                        robot_model->jointnames_active[i]);
    unsigned int index =
        std::distance(robot_model->jointnames_spanningtree.begin(), it);
    // extract active joint position limits
    u_min(i) = q_min(index);
    u_max(i) = q_max(index);
    cout << "Pos min: " << u_min(i) << " Pos max: " << u_max(i) << endl;
    // extract active joint velocity limits
    ud_min(i) = qd_min(index);
    ud_max(i) = qd_max(index);
    cout << "Vel min: " << ud_min(i) << " Vel max: " << ud_max(i) << endl;
    // extract active joint force/torque limits
    Tau_actuated_min(i) = Tau_spanningtree_min(index);
    Tau_actuated_max(i) = Tau_spanningtree_max(index);
    cout << "Tau min: " << Tau_actuated_min(i)
         << " Tau max: " << Tau_actuated_max(i) << endl;
  }
}

bool HyRoDyn_Limit_Checker::compare_eigen_vectors(
    const VectorXd vec, const VectorXd min, const VectorXd max,
    std::vector<unsigned int> &index) {

  for (unsigned int i = 0; i < vec.size(); i++) {
    if (vec(i) < min(i)) {
      cerr << "Joint limit violation at index = " << i << " Value = " << vec(i)
           << ", Min = " << min(i) << ", Max = " << max(i) << endl;
      index.push_back(i);
    } else if (vec(i) > max(i)) {
      cerr << "Joint limit violation at index = " << i << " Value = " << vec(i)
           << ", Min = " << min(i) << ", Max = " << max(i) << endl;
      index.push_back(i);
    }
  }

  if (index.size() == 0)
    return false;
  else
    return true;
}

bool HyRoDyn_Limit_Checker::hierarchical_capability_check(const VectorXd y,
                                                          const VectorXd yd,
                                                          const VectorXd ydd) {

  bool violation_flag = false;

  // 1. Input feasiblity
  if (compare_eigen_vectors(y, y_min, y_max,
                            input_pos_limit_violation_report)) {
    violation_flag = true;
    cout << "The input motion violates the independent joints position limits."
         << endl;
    cout << "Affected joints include: " << endl;
    for (unsigned int i = 0; i < input_pos_limit_violation_report.size(); i++)
      cout << robot_model
                  ->jointnames_independent[input_pos_limit_violation_report[i]]
           << endl;
  }

  // 2. Feasiblity on Input velocity
  if (compare_eigen_vectors(yd, yd_min, yd_max,
                            input_vel_limit_violation_report)) {
    violation_flag = true;
    cout << "The input motion violates the independent joints velocity limits."
         << endl;
    cout << "Affected joints include: " << endl;
    for (unsigned int i = 0; i < input_vel_limit_violation_report.size(); i++)
      cout << robot_model
                  ->jointnames_independent[input_vel_limit_violation_report[i]]
           << endl;
  }

  // Input the motion to hyrodyn robot model
  robot_model->y = y;
  robot_model->yd = yd;
  robot_model->ydd = ydd;

  // Compute the complete system state
  robot_model->calculate_system_state();

  // Compute the inverse dynamics
  robot_model->calculate_inverse_dynamics();

  // Extract the output state from the hyrodyn robot model and perform the
  // capability check
  // 3. C-space feasiblity
  if (compare_eigen_vectors(robot_model->Q, q_min, q_max,
                            cspace_pos_limit_violation_report)) {
    violation_flag = true;
    cout << "The input motion violates the c-space position limits." << endl;
    cout << "Affected joints include: " << endl;
    for (unsigned int i = 0; i < cspace_pos_limit_violation_report.size(); i++)
      cout
          << robot_model
                 ->jointnames_spanningtree[cspace_pos_limit_violation_report[i]]
          << endl;
  }
  // 4. Feasiblity on Tangent Bundle of c-space
  if (compare_eigen_vectors(robot_model->QDot, qd_min, qd_max,
                            cspace_vel_limit_violation_report)) {
    violation_flag = true;
    cout << "The input motion violates the c-space velocity limits." << endl;
    cout << "Affected joints include: " << endl;
    for (unsigned int i = 0; i < cspace_vel_limit_violation_report.size(); i++)
      cout
          << robot_model
                 ->jointnames_spanningtree[cspace_vel_limit_violation_report[i]]
          << endl;
  }
  // 5. Feasiblity in actuation space (position level)
  if (compare_eigen_vectors(robot_model->u, u_min, u_max,
                            actuator_pos_limit_violation_report)) {
    violation_flag = true;
    cout << "The input motion violates the actuator position limits." << endl;
    cout << "Affected joints include: " << endl;
    for (unsigned int i = 0; i < actuator_pos_limit_violation_report.size();
         i++)
      cout << robot_model
                  ->jointnames_active[actuator_pos_limit_violation_report[i]]
           << endl;
  }
  // 6. Feasiblity in actuation space (velocity level)
  if (compare_eigen_vectors(robot_model->ud, ud_min, ud_max,
                            actuator_vel_limit_violation_report)) {
    violation_flag = true;
    cout << "The input motion violates the actuator velocity limits." << endl;
    cout << "Affected joints include: " << endl;
    for (unsigned int i = 0; i < actuator_vel_limit_violation_report.size();
         i++)
      cout << robot_model
                  ->jointnames_active[actuator_vel_limit_violation_report[i]]
           << endl;
  }
  // 7. Feasiblity in actuation space (effort level)
  if (compare_eigen_vectors(robot_model->Tau_actuated, Tau_actuated_min,
                            Tau_actuated_max,
                            actuator_effort_limit_violation_report)) {
    violation_flag = true;
    cout << "The input motion violates the actuator torque limits." << endl;
    cout << "Affected joints include: " << endl;
    for (unsigned int i = 0; i < actuator_effort_limit_violation_report.size();
         i++)
      cout << robot_model
                  ->jointnames_active[actuator_effort_limit_violation_report[i]]
           << endl;
  }

  if (violation_flag) {
    cout << "Point-wise capability not feasible!" << endl;
    return false;
  } else
    return true;
}
