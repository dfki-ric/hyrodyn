/*
 * HyRoDyn - Hybrid Robot Dynamics
 * Copyright (c) 2018-2020 Shivesh Kumar <shivesh.kumar@dfki.de>
 * This submechanism library is contributed by lib_author_name <lib_author_email>.
 * Licensed under the zlib license. See LICENSE for more details.
 */

#include "mech_type_cap.hpp"

namespace mech_type_cap {

// Constructor
mech_type_small::mech_type_small(string file_path, std::vector<string> jointnames_spanningtree,
                                 std::vector<string> jointnames_independent) {
  Model m;

  const char* ext;
  ext = strrchr(file_path.c_str(), '.');
  if (!strcmp(ext, ".urdf") || !strcmp(ext, ".URDF") || !strcmp(ext, ".robot")) {
    if (!Addons::URDFReadFromFileWithModularity(file_path.c_str(), &m, jointnames_spanningtree,
                                                false)) {
      std::cerr << "Error loading urdf model" << std::endl;
      abort();
    }
  }
}
else {
  std::cerr << "Unknown file type: Accepted file types are .urdf or .lua" << endl;
  abort();
}

assert(dof_independent == jointnames_independent.size());
assert(dof_spanningtree == m.dof_count);

// Put the mechanism in its zero configuration
VectorNd Q(VectorNd::Zero(m.dof_count));
UpdateKinematicsCustom(m, &Q, NULL, NULL);

// Extract the physical parameters of the mechanism from the RBDL model

// Comment the line below in case you derived the analytical solutions in relative coordinates.
Q_zero = VectorNd::Zero(m.dof_count);
cout << "Q_zero: " << Q_zero.transpose() << endl;

}  // namespace mech_type_cap

VectorXd mech_type_small::calc_loopclosure_function(const Math::VectorNd& y) {
  // This function calculates the loop closure function Gamma which is a vector of size (n x 1).

  VectorXd Q(dof_spanningtree);

  // Put your symbolically generated code here!

  // Comment the line below in case you derived the analytical solutions in relative coordinates.
  Q = Q - Q_zero;

  return Q;
}

MatrixXd mech_type_small::calc_loopclosure_Jacobian(const Math::VectorNd& y) {
  // This function calculates the loop closure Jacobian matrix G which is a matrix of size (n x m).

  MatrixXd G(dof_spanningtree, dof_independent);

  // Put your symbolically generated code here!

  return G;
}

MatrixXd mech_type_small::calc_loopclosure_Jacobiand(const Math::VectorNd& y,
                                                     const Math::VectorNd& ydot) {
  // This function calculates the 1st order time derivative of loop closure Jacobian Gdot which is a
  // matrix of size (n x m).

  MatrixXd Gdot(dof_spanningtree, dof_independent);

  // Put your symbolically generated code here!

  return Gdot;
}

VectorXd mech_type_small::calc_loopclosure_g(const Math::VectorNd& y, const Math::VectorNd& ydot) {
  // This function calculates the loop closure bias acceleration g= Gdot*ydot

  return calc_loopclosure_Jacobiand(y, ydot) * ydot;
}
}
