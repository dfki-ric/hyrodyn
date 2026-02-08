#include "PARALLELOGRAMCHAIN.hpp"

namespace PARALLELOGRAMCHAIN {

// Constructor
parallelogramchain::parallelogramchain(string file_path,
                                       std::vector<string> jointnames_spanningtree,
                                       std::vector<string> jointnames_active) {
  dof_active = jointnames_active.size();
  dof_spanningtree = jointnames_spanningtree.size();

  std::vector<string> activejoints, treejoints;

  //	G.setZero(dof_spanningtree, dof_active);
  Gdot.setZero(dof_spanningtree, dof_active);

  const char* ext;
  ext = strrchr(file_path.c_str(), '.');
  if (!strcmp(ext, ".urdf") || !strcmp(ext, ".URDF") || !strcmp(ext, ".robot")) {
    //	cout<<"Input file is URDF"<<endl;
    if (!Addons::URDFReadLoopClosureFunction(file_path.c_str(), G, offset, Gu, activejoints,
                                             treejoints)) {
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

  //  cout<<"Matrix G:"<<endl<<G<<endl;
  //  cout<<"Offset:"<<endl<<offset<<endl;
  //  cout<<"Matrix Gu:"<<endl<<Gu<<endl;
}

VectorXd parallelogramchain::calc_loopclosure_function(const Math::VectorNd& y) {
  // This function calculates the loop closure functions gamma
  // for the mechanism

  VectorXd Q(dof_spanningtree);

  Q = G * y + offset;

  return Q;
}

MatrixXd parallelogramchain::calc_loopclosure_Jacobian(const Math::VectorNd& y) {
  // This function calculates the loop closure Jacobian G
  // for the mechanism

  return G;
}

MatrixXd parallelogramchain::calc_loopclosure_Jacobiand(const Math::VectorNd& y,
                                                        const Math::VectorNd& ydot) {
  // This function calculates the loop closure Jacobiand Gdot
  // for the mechanism

  return Gdot;
}

VectorXd parallelogramchain::calc_loopclosure_g(const Math::VectorNd& y,
                                                const Math::VectorNd& ydot) {
  // This function calculates the loop closure bias acceleration g= Gdot*ydot
  // for the mechanism

  return calc_loopclosure_Jacobiand(y, ydot) * ydot;
}

}  // namespace PARALLELOGRAMCHAIN
