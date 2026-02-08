#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <cmath>
#include <fstream>
#include <iostream>

#include "robot_model_hyrodyn.hpp"

using namespace std;

namespace hyrodyn {
class Simulator {
  /// \brief Available integration methods
  enum integrator { euler, runge_kutta };
  /// \brief Type of integration scheme that should be used
  integrator integrator_type;
  /// \brief Vector of simulation time
  std::vector<double> time_vector;
  /// \brief Vector of generalized positions
  std::vector<VectorNd> pos_vector;

  //! Right hand side of the ordinary differential equation ydot = f(t,y)
  /*!
    \param t time
    \param y state vector comprising of position and velocity variables of the
    generalized coordinates
  */
  VectorNd rhs(double t, const VectorNd &y);
  //! Fourth order Runge Kutta Integrator
  /*!
    \param t time
    \param h step size
    \param y state vector comprising of position and velocity variables of the
    generalized coordinates	\n NOTE: The method is a fourth-order method,
    meaning that the local truncation error is on the order of O(h^5), while the
    total accumulated error is order O(h^4).
  */
  VectorNd runge_integrator(const double t, const double h, const VectorNd &y);
  //! First Order Newton-Euler Integrator
  /*!
    \param t time
    \param h step size
    \param y state vector comprising of position and velocity variables of the
    generalized coordinates	\n NOTE: The accuracy of the Euler method is
    limited and frequently its solutions are unstable.
  */
  VectorNd euler_integrator(const double t, const double h, const VectorNd &y);

  //! Writes the simulation output in a csv file so that it can be played or
  //! plotted in external applications
  /*!
    \param filename Name of the csv file where the simulation output should be
    dumped
  */
  void write_simulation_data_into_csv(const char *filename);

public:
  /// \brief Dimension of the state vector (two times the dof)
  unsigned int state_dim;

  /// \brief Independent degrees of freedom of the robot
  unsigned int dof;

  /// \brief current simulation time defaults to zero
  double current_time = 0.0;

  /// \brief current state (generalized positions + velocities) vector
  VectorNd current_state;

  /// \brief hyrodyn robot model which provides the plant dynamics
  hyrodyn::RobotModel_HyRoDyn robot_plant;

  //! Constructor of the simulator class
  /*!
    \param filepath_urdf file path of the URDF to build the hyrodyn robot model
    \param filepath_submechanisms file path of the submechanisms YAML file to
    build the hyrodyn robot model \param integrator_choice choice of integration
    scheme
  */
  Simulator(string filepath_urdf, string filepath_submechanisms,
            string integrator_choice = "runge_kutta");

  //! Simulate the plant dynamics over a certain time period tf starting at t0
  /*!
    \param t0 simulation start time
    \param y0 initial state vector
    \param tf simulation end time
    \param h simulation step size
    \param dump_sim_output_into_csv flag to decide whether to dump the
    simulation output to a csv file
  */
  VectorNd simulate_over_period(const double t0, const VectorNd &y0,
                                const double tf = 1.0, const double h = 0.001,
                                const bool dump_sim_output_into_csv = false);

  //! Simulate the plant dynamics instantenously to the next step
  /*!
    \param h simulation step size
  */
  void simulate(const double h = 0.001);
};

} // namespace hyrodyn

#endif // HYRODYN
