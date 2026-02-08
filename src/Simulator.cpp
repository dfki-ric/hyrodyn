#include "Simulator.hpp"
using namespace hyrodyn;

Simulator::Simulator(string filepath_urdf, string filepath_submechanisms,
                     string integrator_choice) {

  // Setup the hyrodyn robot model
  robot_plant.load_robotmodel(filepath_urdf, filepath_submechanisms);

  if (integrator_choice == "runge_kutta")
    integrator_type = runge_kutta;
  else if (integrator_choice == "euler")
    integrator_type = euler;
  else {
    cerr << "Integrator choice is not available. Simulator will abort!" << endl;
    abort();
  }

  dof = robot_plant.independent_dof;

  state_dim = 2 * dof;
}

void Simulator::write_simulation_data_into_csv(const char *filename) {

  ofstream output_file(filename, ios::trunc | ios::out);

  if (!output_file) {
    cerr << "Error: could not reset file " << filename << "." << endl;
    abort();
  }

  output_file << "time"
              << ",";
  for (std::size_t i = 0; i < robot_plant.jointnames_spanningtree.size(); ++i) {
    if (i != robot_plant.jointnames_spanningtree.size() - 1)
      output_file << "q_" + robot_plant.jointnames_spanningtree[i] << ",";
    else
      output_file << "q_" + robot_plant.jointnames_spanningtree[i];
  }
  output_file << endl;

  assert(pos_vector.size() == time_vector.size());

  for (unsigned int i = 0; i < pos_vector.size(); i++) {

    output_file << time_vector[i] << ",";

    for (unsigned int j = 0; j < robot_plant.jointnames_spanningtree.size();
         j++) {
      output_file << pos_vector[i][j];

      if (j != pos_vector[i].size() - 1)
        output_file << ",";
    }
    output_file << std::endl;
  }

  output_file.close();
}

VectorNd Simulator::rhs(double t, const VectorNd &y) {

  // We build an ODE in generalized coordinates a.k.a independent joint space in
  // hyrodyn

  VectorNd res(VectorNd::Zero(state_dim));

  for (int i = 0; i < dof; i++) {
    robot_plant.y[i] = y[i];
    robot_plant.yd[i] = y[i + dof];
  }

  // Compute the forward dynamics (results stored in robot_plant.ydd)
  robot_plant.calculate_forward_dynamics();

  for (int i = 0; i < dof; i++) {
    res[i] = robot_plant.yd[i];
    res[i + dof] = robot_plant.ydd[i];
  }

  return res;
}

VectorNd Simulator::runge_integrator(const double t, const double h,
                                     const VectorNd &y) {

  VectorNd k1 = rhs(t, y);
  VectorNd k2 = rhs(t + 0.5 * h, y + 0.5 * h * k1);
  VectorNd k3 = rhs(t + 0.5 * h, y + 0.5 * h * k2);
  VectorNd k4 = rhs(t + h, y + h * k3);

  return 1 / 6. * (k1 + 2. * k2 + 2. * k3 + k4);
}

VectorNd Simulator::euler_integrator(const double t, const double h,
                                     const VectorNd &y) {
  return rhs(t, y);
}

VectorNd Simulator::simulate_over_period(const double t0, const VectorNd &y0,
                                         const double tf, const double h,
                                         bool dump_sim_output_into_csv) {

  double t = t0;
  VectorNd y = y0;

  while (t <= tf) {

    switch (integrator_type) {

    case euler:
      y = y + h * euler_integrator(t, h, y);

    case runge_kutta:
      y = y + h * runge_integrator(t, h, y);
    }

    // y = y + h * integrator (t, y, h, rhs);
    t = t + h;

    robot_plant.calculate_system_state();
    time_vector.push_back(t);
    pos_vector.push_back(robot_plant.Q);
  }

  if (dump_sim_output_into_csv)
    write_simulation_data_into_csv("animation.csv");

  return y;
}

void Simulator::simulate(const double h) {

  switch (integrator_type) {

  case euler:
    current_state =
        current_state + h * euler_integrator(current_time, h, current_state);

  case runge_kutta:
    current_state =
        current_state + h * runge_integrator(current_time, h, current_state);
  }

  current_time = current_time + h;

  robot_plant.calculate_system_state();
}
