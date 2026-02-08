/*
 * HyRoDyn - Hybrid Robot Dynamics
 * Copyright (c) 2018-2020 Shivesh Kumar <shivesh.kumar@dfki.de>
 * This submechanism library is contributed by Shivesh Kumar
 * <shivesh.kumar@dfki.de>. Licensed under the zlib license. See LICENSE for
 * more details.
 */

#include "TRANSMISSION.hpp"

namespace TRANSMISSION {

// Constructor
transmission::transmission(string file_path,
                           std::vector<string> jointnames_spanningtree,
                           std::vector<string> jointnames_independent) {

  dof_spanningtree = jointnames_spanningtree.size();
  dof_independent = jointnames_independent.size();

  const char *ext;
  ext = strrchr(file_path.c_str(), '.');
  if (!strcmp(ext, ".urdf") || !strcmp(ext, ".URDF") ||
      !strcmp(ext, ".robot")) {
    if (!Addons::URDFReadLoopClosureFunctionTransmission(
            file_path.c_str(), G, offset, jointnames_independent,
            jointnames_spanningtree)) {
      std::cerr << "Error loading urdf model" << std::endl;
      abort();
    }
  } else if (!strcmp(ext, ".lua")) {
    std::cerr << "Lua models are not supported for parallelogram chains. "
                 "Please provide a URDF file with mimic joints defined."
              << std::endl;
  } else {
    std::cerr << "Unknown file type: Accepted file types is .urdf" << endl;
    abort();
  }

  Gdot.setZero(dof_spanningtree, dof_independent);

  //	cout<<"G matrix: \n"<<G<<endl;
  //	cout<<"offset vector: \n"<<offset<<endl;
  //	cout<<"Gdot matrix: \n"<<Gdot<<endl;
}

VectorXd transmission::calc_loopclosure_function(const Math::VectorNd &y) {
  // This function calculates the loop closure function Gamma which is a vector
  // of size (n x 1).

  VectorXd Q(dof_spanningtree);

  Q = G * y + offset;

  return Q;
}

MatrixXd transmission::calc_loopclosure_Jacobian(const Math::VectorNd &y) {
  // This function calculates the loop closure Jacobian matrix G which is a
  // matrix of size (n x m).

  return G;
}

MatrixXd transmission::calc_loopclosure_Jacobiand(const Math::VectorNd &y,
                                                  const Math::VectorNd &ydot) {
  // This function calculates the 1st order time derivative of loop closure
  // Jacobian Gdot which is a matrix of size (n x m).

  return Gdot;
}

VectorXd transmission::calc_loopclosure_g(const Math::VectorNd &y,
                                          const Math::VectorNd &ydot) {
  // This function calculates the loop closure bias acceleration g= Gdot*ydot

  return calc_loopclosure_Jacobiand(y, ydot) * ydot;
}

} // namespace TRANSMISSION
