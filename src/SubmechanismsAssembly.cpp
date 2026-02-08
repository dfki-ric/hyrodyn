#include "SubmechanismsAssembly.hpp"

#include <cstdlib>

namespace AssembleinHyRoDyn {
/*
std::string getexepath()
{
  char result[ PATH_MAX ];
  ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
  return std::string( result, (count > 0) ? count : 0 );
}
*/
// std::string SplitFilename (const std::string& str)
// {
//   std::string file, path;
// //  std::cout << "Splitting: " << str << '\n';
//   std::size_t found = str.find_last_of("/\\");
//   file = str.substr(found+1);
//   path = str.substr(0,found);
// //  std::cout << " path: " << path << '\n';
// //  std::cout << " file: " << file << '\n';
//   return path;
// }

// constructor
SubmechanismsAssembly::SubmechanismsAssembly(
    std::vector<submechanism> to_assemble,
    std::vector<exoskeleton> to_externally_attach) {
  // Add the name of the parallel submechanisms that are known to the solver
  available_parallelsubmechanisms.push_back("rrPr");
  available_parallelsubmechanisms.push_back("2SPRR+1U");
  available_parallelsubmechanisms.push_back("3RUS2");
  available_parallelsubmechanisms.push_back("3RUS2_ANKLE");
  available_parallelsubmechanisms.push_back("2SPU+1U");
  available_parallelsubmechanisms.push_back("6UPS");
  available_parallelsubmechanisms.push_back("2SURRPR+1U");
  available_parallelsubmechanisms.push_back("PARALLELOGRAMCHAIN");
  available_parallelsubmechanisms.push_back("TRANSMISSION");
  available_parallelsubmechanisms.push_back("NUMERICAL");
  // calculate the absolute file path wrt which submechanisms file should be
  // specified.
  // path = SplitFilename (getexepath());

  /*
  // calculate the rock env file path wrt which the submechanisms file paths
  should be specified. const char *rock_evn_path =
  std::getenv("AUTOPROJ_CURRENT_ROOT"); if(rock_evn_path != nullptr) path =
  rock_evn_path; else{ cerr<<"Autoproj environment path not defined. Make sure
  you do: source env.sh before calling HyRoDyn!"<<endl; abort();
  }


  // Override absolute path if env variable is available
  // This is useful when using hyrodyn from python.
  const char *pythonbindings_path = std::getenv("HYRODYN_BASE");
  if(pythonbindings_path != nullptr) {
          path = pythonbindings_path;
  }
  */

  assembly = to_assemble;
  exteriors = to_externally_attach;

  std::vector<string> jointnames_spanningtree;
  std::vector<string> jointnames_active;
  std::vector<string> jointnames_independent;
  std::vector<string> jointnames_independent_robot;

  // Prepare the joint names vector of spanning tree joints
  for (unsigned int i = 0; i < assembly.size(); i++)
    for (unsigned int j = 0; j < assembly[i].jointnames_spanningtree.size();
         j++)
      jointnames_spanningtree.push_back(assembly[i].jointnames_spanningtree[j]);
  // additionally extract the spanning tree joints of any exoskeleton mechanism
  // to the mechanism
  for (unsigned int i = 0; i < exteriors.size(); i++)
    for (unsigned int j = 0; j < exteriors[i].jointnames_spanningtree.size();
         j++)
      jointnames_spanningtree.push_back(
          exteriors[i].jointnames_spanningtree[j]);

  // Prepare the joint names vector of active joints
  for (unsigned int i = 0; i < assembly.size(); i++)
    for (unsigned int j = 0; j < assembly[i].jointnames_active.size(); j++)
      jointnames_active.push_back(assembly[i].jointnames_active[j]);

  // Prepare the joint names vector of independent joints
  for (unsigned int i = 0; i < assembly.size(); i++)
    for (unsigned int j = 0; j < assembly[i].jointnames_independent.size(); j++)
      jointnames_independent.push_back(assembly[i].jointnames_independent[j]);

  // Prepare the joint names vector of independent joints in the robot (i.e.
  // excluding free flyer joint)
  for (unsigned int i = 0; i < assembly.size(); i++)
    for (unsigned int j = 0;
         j < assembly[i].jointnames_independent_robot.size(); j++)
      jointnames_independent_robot.push_back(
          assembly[i].jointnames_independent_robot[j]);

  // calculate the permutation matrix Q1 (p x n) for the whole assembly
  calc_permutationmatrix(jointnames_spanningtree, jointnames_active);
  // calculate the permutation matrix Q2 (p x m) for the whole assembly
  calc_permutationmatrix2(jointnames_independent, jointnames_independent_robot);

  dof_active = 0;
  dof_spanningtree = 0;
  dof_independent = 0;
  dof_independent_robot = 0;

  for (unsigned int i = 0; i < assembly.size(); i++)
    dof_spanningtree =
        dof_spanningtree + assembly[i].jointnames_spanningtree.size();
  //	cout<<"size of spanning tree: "<<dof_spanningtree<<endl;

  for (unsigned int i = 0; i < exteriors.size(); i++)
    dof_spanningtree =
        dof_spanningtree + exteriors[i].jointnames_spanningtree.size();

  for (unsigned int i = 0; i < assembly.size(); i++)
    dof_active = dof_active + assembly[i].jointnames_active.size();
  //	cout<<"number of active joints: " << dof_active << endl;

  for (unsigned int i = 0; i < assembly.size(); i++)
    dof_independent =
        dof_independent + assembly[i].jointnames_independent.size();
  //	cout<<"number of independent joints: " << dof_independent << endl;

  for (unsigned int i = 0; i < assembly.size(); i++)
    dof_independent_robot =
        dof_independent_robot + assembly[i].jointnames_independent_robot.size();
  //	cout<<"number of independent joints belonging to the robot: " <<
  //dof_independent_robot << endl;

  for (unsigned int i = 0; i < assembly.size(); i++) {
    cout << "Loading submechanism URDF at index " << i << endl;
    if (std::find(available_parallelsubmechanisms.begin(),
                  available_parallelsubmechanisms.end(),
                  assembly[i].type) != available_parallelsubmechanisms.end()) {
      if (assembly[i].file_path == "") {
        cerr << "file_path has to be defined for submechanisms that are not "
                "serial ones"
             << endl;
        abort();
      }
      if (assembly[i].type == "rrPr")
        submechanism_constraint_set.push_back(new RRPR::rrPr(
            assembly[i].file_path, assembly[i].jointnames_spanningtree,
            assembly[i].jointnames_active));
      else if (assembly[i].type == "2SPU+1U")
        submechanism_constraint_set.push_back(new SPU2U1::sPu2u1(
            assembly[i].file_path, assembly[i].jointnames_spanningtree,
            assembly[i].jointnames_active));
      else if (assembly[i].type == "3RUS2")
        submechanism_constraint_set.push_back(new R3US2::R3us2(
            assembly[i].file_path, assembly[i].jointnames_spanningtree,
            assembly[i].jointnames_active));
      else if (assembly[i].type == "3RUS2_ANKLE")
        submechanism_constraint_set.push_back(new R3US2_ANKLE::r3us2_ankle(
            assembly[i].file_path, assembly[i].jointnames_spanningtree,
            assembly[i].jointnames_active));
      else if (assembly[i].type == "2SPRR+1U")
        submechanism_constraint_set.push_back(new SPRR2U1::sPrr2u1(
            assembly[i].file_path, assembly[i].jointnames_spanningtree,
            assembly[i].jointnames_active));
      else if (assembly[i].type == "2SURRPR+1U")
        submechanism_constraint_set.push_back(new SURRPR2U1::surrPr2u1(
            assembly[i].file_path, assembly[i].jointnames_spanningtree,
            assembly[i].jointnames_active));
      else if (assembly[i].type == "6UPS")
        submechanism_constraint_set.push_back(new UPS6::uPs6(
            assembly[i].file_path, assembly[i].jointnames_spanningtree,
            assembly[i].jointnames_active));
      else if (assembly[i].type == "PARALLELOGRAMCHAIN")
        submechanism_constraint_set.push_back(
            new PARALLELOGRAMCHAIN::parallelogramchain(
                assembly[i].file_path, assembly[i].jointnames_spanningtree,
                assembly[i].jointnames_active));
      else if (assembly[i].type == "TRANSMISSION")
        submechanism_constraint_set.push_back(new TRANSMISSION::transmission(
            assembly[i].file_path, assembly[i].jointnames_spanningtree,
            assembly[i].jointnames_independent));

      else if (assembly[i].type == "NUMERICAL") {

        cout << "Let's do it." << endl;
        submechanism_constraint_set.push_back(
            new NUMERICALLOOPCONSTRAINTS::NumericalLoopConstraints(
                assembly[i].file_path, assembly[i].jointnames_spanningtree,
                assembly[i].jointnames_independent,
                assembly[i].jointnames_active,
                assembly[i].loop_constraints_submech));
      }

      else {
        cerr << "Submechanism type not found. Aborting" << endl;
        abort();
      }
    } else
      submechanism_constraint_set.push_back(
          new ExplicitLoopConstraints::ExplicitLoopConstraintSet);
  }

  for (unsigned int i = 0; i < exteriors.size(); i++)
    exterior_constraint_set.push_back(new EXTERIOR::exterior(
        exteriors[i].file_path, exteriors[i].jointnames_spanningtree,
        exteriors[i].jointnames_dependent));
}

void SubmechanismsAssembly::calc_permutationmatrix(
    std::vector<string> jointnames_spanningtree,
    std::vector<string> jointnames_active) {
  /*
  This permutation matrix maps the spanning tree joints to active joints.
  * u = Q * q = Q * Gamma(y)
  */
  permutation_matrix.setZero(jointnames_active.size(),
                             jointnames_spanningtree.size());
  for (unsigned int i = 0; i < jointnames_active.size(); i++) {
    for (unsigned int j = 0; j < jointnames_spanningtree.size(); j++) {
      if (jointnames_spanningtree[j] == jointnames_active[i])
        //		Q(i,j) = 1;
        permutation_matrix(i, j) = 1;
    }
  }
  //	cout<<"Permutation matrix, Q = "<<endl<<permutation_matrix<<endl;
  //	cout<<"Permutation matrix, QTQ =
  //"<<endl<<permutation_matrix*permutation_matrix.transpose()<<endl;
}

void SubmechanismsAssembly::calc_permutationmatrix2(
    std::vector<string> jointnames_independent,
    std::vector<string> jointnames_independent_robot) {
  /*
  This permutation matrix( m - floating_dof x m) maps the independent joints
  including floating base coordinates(y = [yf yr]) to independent joints (yr).
  * yr = Q * y
  */

  permutation_matrix2.setZero(jointnames_independent_robot.size(),
                              jointnames_independent.size());
  for (unsigned int i = 0; i < jointnames_independent_robot.size(); i++) {
    for (unsigned int j = 0; j < jointnames_independent.size(); j++) {
      if (jointnames_independent[j] == jointnames_independent_robot[i])
        permutation_matrix2(i, j) = 1;
    }
  }

  //	permutation_matrix2.setZero(jointnames_independent.size() - 6,
  //jointnames_independent.size());

  //	permutation_matrix2.block(0,6,jointnames_independent.size() - 6,
  //jointnames_independent.size() - 6) =
  //MatrixNd::Identity(jointnames_independent.size() - 6,
  //jointnames_independent.size() - 6);

  //	cout<<"Permutation matrix, Q = "<<endl<<permutation_matrix2<<endl;
}

VectorXd
SubmechanismsAssembly::calc_loopclosure_function(const Math::VectorNd &y) {

  VectorXd Gamma = VectorXd::Zero(dof_spanningtree);

  unsigned int k = 0; // iterator on independent dofs of the full assembly
  unsigned int j = 0; // iterator on spanning tree of the full assembly

  for (unsigned int i = 0; i < assembly.size(); i++) {
    VectorXd y_submechanism =
        y.segment(k, assembly[i].jointnames_independent.size());
    // put the value of loop closure function from the submechanism into the
    // relevant segment of vector Gamma
    Gamma.segment(j, assembly[i].jointnames_spanningtree.size()) =
        submechanism_constraint_set[i]->calc_loopclosure_function(
            y_submechanism);
    k = k + assembly[i].jointnames_independent.size();
    j = j + assembly[i].jointnames_spanningtree.size();
  }

  k = 0; // set to zero to iterate on the independent dofs again
  for (unsigned int i = 0; i < exteriors.size(); i++) {
    VectorXd y_submechanism =
        y.segment(k, exteriors[i].jointnames_dependent.size());
    // put the value of loop closure function from the submechanism into the
    // relevant segment of vector Gamma
    Gamma.segment(j, exteriors[i].jointnames_spanningtree.size()) =
        exterior_constraint_set[i]->calc_loopclosure_function(y_submechanism);
    k = k + exteriors[i].jointnames_dependent.size();
    j = j + exteriors[i].jointnames_spanningtree.size();
  }

  return Gamma;
}

MatrixXd
SubmechanismsAssembly::calc_loopclosure_Jacobian(const Math::VectorNd &y) {

  MatrixXd G;
  G.setZero(dof_spanningtree, dof_independent);
  unsigned int k = 0; // iterator on spanning tree of the full assembly
  unsigned int j = 0; // iterator on independent dofs of the full assembly

  for (unsigned int i = 0; i < assembly.size(); i++) {
    // select a segment of vectorxd y which corresponds to the length of active
    // joints in that particular submechanism
    VectorXd y_submechanism =
        y.segment(j, assembly[i].jointnames_independent.size());
    // put the value of loop closure Jacobian from the submechanism into the
    // relevant block of matrix G
    G.block(k, j, assembly[i].jointnames_spanningtree.size(),
            assembly[i].jointnames_independent.size()) =
        submechanism_constraint_set[i]->calc_loopclosure_Jacobian(
            y_submechanism);
    k = k + assembly[i].jointnames_spanningtree.size();
    j = j + assembly[i].jointnames_independent.size();
  }

  j = 0; // set to zero to iterate on the independent dofs again
  for (unsigned int i = 0; i < exteriors.size(); i++) {
    // select a segment of vectorxd y which corresponds to the length of active
    // joints in that particular submechanism
    VectorXd y_submechanism =
        y.segment(j, exteriors[i].jointnames_dependent.size());
    // put the value of loop closure Jacobian from the submechanism into the
    // relevant block of matrix G
    G.block(k, j, exteriors[i].jointnames_spanningtree.size(),
            exteriors[i].jointnames_dependent.size()) =
        exterior_constraint_set[i]->calc_loopclosure_Jacobian(y_submechanism);
    k = k + exteriors[i].jointnames_spanningtree.size();
    j = j + exteriors[i].jointnames_dependent.size();
  }

  return G;
}

MatrixXd
SubmechanismsAssembly::calc_loopclosure_Jacobiand(const Math::VectorNd &y,
                                                  const Math::VectorNd &ydot) {

  MatrixXd Gdot;
  Gdot.setZero(dof_spanningtree, dof_independent);
  unsigned int k = 0; // iterator on spanning tree of the full assembly
  unsigned int j = 0; // iterator on independent dofs of the full assembly

  for (unsigned int i = 0; i < assembly.size(); i++) {
    // select a segment of vectorxd y which corresponds to the length of active
    // joints in that particular submechanism
    VectorXd y_submechanism =
        y.segment(j, assembly[i].jointnames_independent.size());
    VectorXd yd_submechanism =
        ydot.segment(j, assembly[i].jointnames_independent.size());
    // put the value of loop closure Jacobian from the submechanism into the
    // relevant block of matrix Gdot
    Gdot.block(k, j, assembly[i].jointnames_spanningtree.size(),
               assembly[i].jointnames_independent.size()) =
        submechanism_constraint_set[i]->calc_loopclosure_Jacobiand(
            y_submechanism, yd_submechanism);
    k = k + assembly[i].jointnames_spanningtree.size();
    j = j + assembly[i].jointnames_independent.size();
  }

  j = 0; // set to zero to iterate on the independent dofs again
  for (unsigned int i = 0; i < exteriors.size(); i++) {
    // select a segment of vectorxd y which corresponds to the length of active
    // joints in that particular submechanism
    VectorXd y_submechanism =
        y.segment(j, exteriors[i].jointnames_dependent.size());
    VectorXd yd_submechanism =
        ydot.segment(j, exteriors[i].jointnames_dependent.size());
    // put the value of loop closure Jacobian from the submechanism into the
    // relevant block of matrix Gdot
    Gdot.block(k, j, exteriors[i].jointnames_spanningtree.size(),
               exteriors[i].jointnames_dependent.size()) =
        exterior_constraint_set[i]->calc_loopclosure_Jacobiand(y_submechanism,
                                                               yd_submechanism);
    k = k + exteriors[i].jointnames_spanningtree.size();
    j = j + exteriors[i].jointnames_dependent.size();
  }

  return Gdot;
}

VectorXd SubmechanismsAssembly::calc_loopclosure_g(const Math::VectorNd &y,
                                                   const Math::VectorNd &ydot) {
  // This function calculates the loop closure bias acceleration g= Gdot*ydot

  VectorXd loop_g = VectorXd::Zero(dof_spanningtree);

  unsigned int k = 0; // iterator on independent dofs of the full assembly
  unsigned int j = 0; // iterator on spanning tree of the full assembly

  for (unsigned int i = 0; i < assembly.size(); i++) {
    VectorXd y_submechanism =
        y.segment(k, assembly[i].jointnames_independent.size());
    VectorXd yd_submechanism =
        ydot.segment(k, assembly[i].jointnames_independent.size());
    // put the value of loop closure function from the submechanism into the
    // relevant segment of vector Gamma
    loop_g.segment(j, assembly[i].jointnames_spanningtree.size()) =
        submechanism_constraint_set[i]->calc_loopclosure_g(y_submechanism,
                                                           yd_submechanism);
    k = k + assembly[i].jointnames_independent.size();
    j = j + assembly[i].jointnames_spanningtree.size();
  }

  k = 0; // set to zero to iterate on the independent dofs again
  for (unsigned int i = 0; i < exteriors.size(); i++) {
    VectorXd y_submechanism =
        y.segment(k, exteriors[i].jointnames_dependent.size());
    VectorXd yd_submechanism =
        ydot.segment(k, exteriors[i].jointnames_dependent.size());
    // put the value of loop closure function from the submechanism into the
    // relevant segment of vector Gamma
    loop_g.segment(j, exteriors[i].jointnames_spanningtree.size()) =
        exterior_constraint_set[i]->calc_loopclosure_g(y_submechanism,
                                                       yd_submechanism);
    k = k + exteriors[i].jointnames_dependent.size();
    j = j + exteriors[i].jointnames_spanningtree.size();
  }

  return loop_g;
}

} // namespace AssembleinHyRoDyn
