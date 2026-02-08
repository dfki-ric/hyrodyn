#include "EXTERIOR.hpp"

namespace EXTERIOR {

// Constructor
exterior::exterior(string file_path,
                   std::vector<string> jointnames_spanningtree,
                   std::vector<string> jointnames_dependent) {

  unsigned int dof_dependent = jointnames_dependent.size();
  dof_spanningtree = jointnames_spanningtree.size();

  Gdot.setZero(dof_spanningtree, dof_dependent);

  const char *ext;
  ext = strrchr(file_path.c_str(), '.');
  if (!strcmp(ext, ".urdf") || !strcmp(ext, ".URDF") ||
      !strcmp(ext, ".robot")) {
    //	cout<<"Input file is URDF"<<endl;
    if (!Addons::URDFReadLoopClosureFunctionExterior(
            file_path.c_str(), G, offset, jointnames_dependent,
            jointnames_spanningtree)) {
      std::cerr << "Error loading urdf model" << std::endl;
      abort();
    }
  }

  else if (!strcmp(ext, ".lua")) {
    std::cerr << "Lua models are not supported for parallelogram chains. "
                 "Please provide a URDF file with mimic joints defined."
              << std::endl;
  } else {
    std::cerr << "Unknown file type: Accepted file types is .urdf" << endl;
    abort();
  }
  //	cout<<"G:\n"<<G<<endl;
  //	cout<<"offset:\n"<<offset<<endl;
}

VectorXd exterior::calc_loopclosure_function(const Math::VectorNd &y) {
  // This function calculates the loop closure functions gamma
  // for the mechanism

  VectorXd Q(dof_spanningtree);

  Q = G * y + offset;

  return Q;
}

MatrixXd exterior::calc_loopclosure_Jacobian(const Math::VectorNd &y) {
  // This function calculates the loop closure Jacobian G
  // for the mechanism

  return G;
}

MatrixXd exterior::calc_loopclosure_Jacobiand(const Math::VectorNd &y,
                                              const Math::VectorNd &ydot) {
  // This function calculates the loop closure Jacobiand Gdot
  // for the mechanism

  return Gdot;
}

VectorXd exterior::calc_loopclosure_g(const Math::VectorNd &y,
                                      const Math::VectorNd &ydot) {
  // This function calculates the loop closure bias acceleration g= Gdot*ydot
  // for the mechanism

  return calc_loopclosure_Jacobiand(y, ydot) * ydot;
}

} // namespace EXTERIOR
