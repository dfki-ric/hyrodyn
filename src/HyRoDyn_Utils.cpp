#include "HyRoDyn_Utils.hpp"

namespace HyRoDyn_Utils {

void HyRoDyn_Utils::initFromYaml(string filepath) {
  YAML::Node doc = YAML::LoadFile(filepath);

  try {
    doc["limits"];
  } catch (std::exception e) {
    std::stringstream ss;
    ss << "Yaml parsing error: File " << filepath
       << " either doesn't exist or is not a valid joint limits file"
       << std::endl;
    throw std::invalid_argument(ss.str());
  }

  const YAML::Node &names_node = doc["limits"]["names"];
  const YAML::Node &elements_node = doc["limits"]["elements"];

  if (elements_node.size() != names_node.size()) {
    throw std::invalid_argument(
        "Invalid yaml file. Size of names is different than size of elements");
  }

  num_input_joints = names_node.size();
  pos_min = VectorNd::Zero(num_input_joints);
  pos_max = VectorNd::Zero(num_input_joints);
  vel_min = VectorNd::Zero(num_input_joints);
  vel_max = VectorNd::Zero(num_input_joints);

  for (uint i = 0; i < names_node.size(); i++) {
    jointnames_input.push_back(names_node[i].as<std::string>());

    pos_max(i) = elements_node[i]["max"]["position"].as<double>();
    pos_min(i) = elements_node[i]["min"]["position"].as<double>();
    vel_max(i) = elements_node[i]["max"]["speed"].as<float>();
    try {
      vel_min(i) = elements_node[i]["min"]["speed"].as<float>();
    } catch (...) {
      vel_min(i) = -vel_max(i);
    }
  }
}

std::string SplitRobotName(const std::string &str) {
  std::string file, path;
  //  std::cout << "Splitting: " << str << '\n';
  std::size_t found = str.find_last_of("/\\");
  file = str.substr(found + 1);
  path = str.substr(0, found);
  //  std::cout << " path: " << path << '\n';
  //  std::cout << " file: " << file << '\n';
  size_t lastindex = file.find_last_of(".");
  string modelname = file.substr(0, lastindex);

  return modelname;
}

HyRoDyn_Utils::HyRoDyn_Utils(string filepath_urdf,
                             string filepath_submechanisms,
                             string filepath_jointlimits,
                             int num_steps_per_joint) {
  robot_model = new RobotModel_HyRoDyn;
  num_steps = num_steps_per_joint;

  // creates the robot model in hyrodyn
  robot_model->load_robotmodel(filepath_urdf, filepath_submechanisms);
  robot_name = SplitRobotName(filepath_urdf);
  cout << "Robot Name: " << robot_name << endl;
  cout << "Number of steps per joint: " << num_steps << endl;

  // reads the limits of the input joints
  initFromYaml(filepath_jointlimits);
  pos_resolution = (pos_max - pos_min) / num_steps;
  vel_resolution = (vel_max - vel_min) / num_steps;
  cout << "Input Joint Limits: " << endl;
  for (unsigned int i = 0; i < jointnames_input.size(); i++) {
    cout << "Joint Name: " << jointnames_input[i] << " Pos Min: " << pos_min(i)
         << " Pos Max: " << pos_max(i) << " Resolution: " << pos_resolution[i]
         << endl;
    if (pos_min(i) > pos_max(i)) {
      cerr << "pos_min is greater than pos_max for joint name: "
           << jointnames_input[i] << endl;
      abort();
    }
  }

  assert(jointnames_input.size() == robot_model->jointnames_independent.size());
  assert(jointnames_input.size() == robot_model->jointnames_active.size());

  // sorted joint names variables for comparison(thats the sole purpose of
  // sorting joint names)
  std::vector<string> sorted_jointnames_input, sorted_jointnames_independent,
      sorted_jointnames_active;
  sorted_jointnames_input = jointnames_input;
  sorted_jointnames_independent = robot_model->jointnames_independent;
  sorted_jointnames_active = robot_model->jointnames_active;

  std::sort(sorted_jointnames_input.begin(), sorted_jointnames_input.end());
  std::sort(sorted_jointnames_independent.begin(),
            sorted_jointnames_independent.end());
  std::sort(sorted_jointnames_active.begin(), sorted_jointnames_active.end());

  input_is_independent_joints =
      std::equal(sorted_jointnames_input.begin(), sorted_jointnames_input.end(),
                 sorted_jointnames_independent.begin());
  input_is_active_joints =
      std::equal(sorted_jointnames_input.begin(), sorted_jointnames_input.end(),
                 sorted_jointnames_active.begin());
  cout << "input_is_independent_joints: " << input_is_independent_joints
       << endl;
  cout << "input_is_active_joints: " << input_is_active_joints << endl;

  if (input_is_independent_joints == input_is_active_joints)
    cout << "Independent joint space and active joint space are the same for "
            "this robot"
         << endl;

  // discretize_input_space();
  discretized_input_matrix = discretize_input_space(
      num_steps, num_input_joints, pos_min, pos_max, pos_resolution);
  // discretized_input_vel_matrix = discretize_input_space(num_steps,
  // num_input_joints, vel_min, vel_max, vel_resolution);
  //	cout<<"Matrix input: "<<discretized_input_matrix<<endl;
}

//
//
MatrixXd HyRoDyn_Utils::discretize_input_space(int num_steps,
                                               int num_input_joints,
                                               VectorXd min, VectorXd max,
                                               VectorXd resolution) {
  MatrixXd discretized_input_matrix;
  discretized_input_matrix.setZero(pow(num_steps, num_input_joints),
                                   num_input_joints);
  std::vector<double> jv;

  for (uint k = 0; k < num_input_joints; k++) {
    jv.push_back(min[k]);
  }

  int i = 0;

  while (true) {
    // Compute discretized points in a matrix
    for (uint k = 0; k < num_input_joints; k++)
      discretized_input_matrix(i, k) = jv[k];
    i = i + 1;

    // Iterate and check for overflow
    bool overflow = true; // Always inc first element
    for (uint k = 0; k < num_input_joints; k++) {
      // if overflow
      if (overflow) {
        jv[k] += resolution[k];
      }

      // check if overflow
      if (jv[k] >= (max[k] - 1e-12)) {
        //            if(jv[k] >=
        //            (pos_min[k]+num_steps*pos_resolution[k])-0.001) {
        jv[k] = min[k];
        overflow = true;
      } else {
        overflow = false;
      }
    }

    if (overflow) {
      break;
    }
  }
  return discretized_input_matrix;
}

void HyRoDyn_Utils::log_sysstate_q() {
  ofstream file("../results/" + robot_name + "_sysstate_q.csv");

  for (std::size_t i = 0; i < robot_model->jointnames_spanningtree.size();
       ++i) {
    if (i != robot_model->jointnames_spanningtree.size() - 1)
      file << "q_" + robot_model->jointnames_spanningtree[i] << ",";
    else
      file << "q_" + robot_model->jointnames_spanningtree[i];
  }
  file << ","
       << "inv_cond_loop_closure_independent_jointspace"
       << ","
       << "inv_cond_loop_closure_active_jointspace";
  file << endl;

  for (uint i = 0; i < pow(num_steps, num_input_joints); i++) {
    if (input_is_independent_joints) {
      // compute the full system state via the inverse model
      robot_model->y = discretized_input_matrix.row(i);
      robot_model->calculate_system_state();
      // compute the condition number of the loop closure jacobian in
      // independent joint space and active joint space
      robot_model->calculate_condition_number_loop_closure();
    } else {
      // compute the full system state via the forward model
      robot_model->u = discretized_input_matrix.row(i);
      robot_model->calculate_forward_system_state();
    }
    for (std::size_t i = 0; i < robot_model->jointnames_spanningtree.size();
         ++i) {
      if (i != robot_model->jointnames_spanningtree.size() - 1)
        file << robot_model->Q[i] << ",";
      else
        file << robot_model->Q[i];
    }
    file << "," << robot_model->inv_cond_loop_closure_independent_jointspace
         << "," << robot_model->inv_cond_loop_closure_active_jointspace;
    file << endl;
  }

  file.close();
}

void HyRoDyn_Utils::log_actuatorstate_u() {
  ofstream file("../results/" + robot_name + "_actuatorstate_u.csv");

  for (std::size_t i = 0; i < robot_model->jointnames_active.size(); ++i) {
    if (i != robot_model->jointnames_active.size() - 1)
      file << "u_" + robot_model->jointnames_active[i] << ",";
    else
      file << "u_" + robot_model->jointnames_active[i];
  }

  file << endl;

  if (!input_is_active_joints) {
    // log actuator state from the inverse model when the input is not active
    // joint space
    for (uint i = 0; i < pow(num_steps, num_input_joints); i++) {
      robot_model->y = discretized_input_matrix.row(i);
      robot_model->calculate_system_state();
      for (std::size_t i = 0; i < robot_model->jointnames_active.size(); ++i) {
        if (i != robot_model->jointnames_active.size() - 1)
          file << robot_model->u[i] << ",";
        else
          file << robot_model->u[i];
      }
      file << endl;
    }
  } else {
    // log actuator state from the discretized input space when the input is
    // active joint space
    for (uint i = 0; i < pow(num_steps, num_input_joints); i++) {
      for (std::size_t j = 0; j < robot_model->jointnames_active.size(); ++j) {
        if (j != robot_model->jointnames_active.size() - 1)
          file << discretized_input_matrix.row(i)(j) << ",";
        else
          file << discretized_input_matrix.row(i)(j);
      }
      file << endl;
    }
  }

  file.close();
}

void HyRoDyn_Utils::log_actuatorforces_Tau() {
  // File in which the results will be stored
  ofstream file("../results/" + robot_name + "_actuatortorques_Tau.csv");

  // Create header for the output file, for every active joint a column will be
  // added
  for (std::size_t i = 0; i < robot_model->jointnames_active.size(); ++i) {
    if (i != robot_model->jointnames_active.size() - 1)
      file << "Tau_" + robot_model->jointnames_active[i] << ", ";
    else
      file << "Tau_" + robot_model->jointnames_active[i];
  }
  file << endl;

  for (uint i = 0; i < pow(num_steps, num_input_joints); i++) {

    if (input_is_independent_joints)
      robot_model->y =
          discretized_input_matrix.row(i); // y is the independent joint space
    else {
      // compute the full system state via the forward model
      robot_model->u = discretized_input_matrix.row(i);
      robot_model
          ->calculate_forward_system_state(); // this will implicitly update the
                                              // independent joint space y
    }

    robot_model->calculate_inverse_dynamics();

    // Inverse kinematic output is stored in the column of the actuated joint
    for (std::size_t i = 0; i < robot_model->jointnames_active.size(); ++i) {
      if (i != robot_model->jointnames_active.size() - 1)
        file << robot_model->Tau_actuated[i] << ", ";
      else
        file << robot_model->Tau_actuated[i];
    }
    file << endl;
  }

  file.close();
}

void HyRoDyn_Utils::log_independentjointstate_y() {
  ofstream file("../results/" + robot_name + "_independentjointstate_y.csv");

  for (std::size_t i = 0; i < robot_model->jointnames_independent.size(); ++i) {
    if (i != robot_model->jointnames_independent.size() - 1)
      file << "y_" + robot_model->jointnames_independent[i] << ",";
    else
      file << "y_" + robot_model->jointnames_independent[i];
  }
  file << endl;

  if (!input_is_independent_joints) {
    for (uint i = 0; i < pow(num_steps, num_input_joints); i++) {
      robot_model->u = discretized_input_matrix.row(i);
      robot_model->calculate_forward_system_state();
      for (std::size_t i = 0; i < robot_model->jointnames_independent.size();
           ++i) {
        if (i != robot_model->jointnames_independent.size() - 1)
          file << robot_model->y[i] << ",";
        else
          file << robot_model->y[i];
      }
      file << endl;
    }
  } else {
    for (uint i = 0; i < pow(num_steps, num_input_joints); i++) {
      for (std::size_t j = 0; j < robot_model->jointnames_independent.size();
           ++j) {
        if (j != robot_model->jointnames_independent.size() - 1)
          file << discretized_input_matrix.row(i)(j) << ",";
        else
          file << discretized_input_matrix.row(i)(j);
      }
      file << endl;
    }
  }

  file.close();
}

void HyRoDyn_Utils::log_forwardkinematics_x(string body_name) {
  const char *dir_path = "../results/";

  is_directory_existing(dir_path);
  // ofstream file(dir_path + robot_name + "_forwardkinematics_x.csv");
  ofstream file(dir_path + body_name + "_forwardkinematics_x.csv");

  file.exceptions(ofstream::failbit | ofstream::badbit);

  // prepare the header for the output file
  file << "x"
       << ", "
       << "y"
       << ", "
       << "z"
       << ", "
       << "qw"
       << ", "
       << "qx"
       << ", "
       << "qy"
       << ", "
       << "qz"
       << ", "
       << "cond1_inv"
       << ", "
       << "cond2_inv";
  file << endl;

  for (uint i = 0; i < pow(num_steps, num_input_joints); i++) {

    if (input_is_independent_joints)
      robot_model->y =
          discretized_input_matrix.row(i); // y is the independent joint space
    else {
      // compute the full system state via the forward model
      robot_model->u = discretized_input_matrix.row(i);
      robot_model
          ->calculate_forward_system_state(); // this will implicitly update the
                                              // independent joint space y
    }

    robot_model->calculate_forward_kinematics(body_name);

    if (num_input_joints < 6)
      robot_model->calculate_condition_number(
          body_name, true); // condition number is computed for point jacobian
    else
      robot_model->calculate_condition_number(
          body_name); // condition number is computed for full jacobian
    for (std::size_t i = 0; i < 7; ++i) {
      file << robot_model->pose[i] << ", ";
    }
    file << robot_model->inv_cond1 << ", " << robot_model->inv_cond2;
    file << endl;
  }

  file.close();
  std::cout << "FK log saved!" << std::endl;
}

void HyRoDyn_Utils::is_directory_existing(const char *dir_path) {
  struct stat sb;

  // checking if the results directory exists
  if (stat(dir_path, &sb) != 0) {
    // Create a directory
    try {
      mkdir(dir_path, 0777);
    } catch (...) {
      cerr << "Directory not created" << endl;
    }
  }
}

} // namespace HyRoDyn_Utils
