#include "NumericalLoopConstraints.hpp"

#include <ostream>

namespace NUMERICALLOOPCONSTRAINTS {
// Constructor
NumericalLoopConstraints::NumericalLoopConstraints(
    const string& file_path, const std::vector<string>& jointnames_spanningtree,
    const std::vector<string>& jointnames_independent, const std::vector<string>& jointnames_active,
    const std::vector<Loop_constraints>& loop_constraints_set, bool verbose) {
  const char* ext;
  ext = strrchr(file_path.c_str(), '.');

  if (!strcmp(ext, ".urdf") || !strcmp(ext, ".URDF") || !strcmp(ext, ".robot")) {
    //	cout<<"Input file is URDF"<<endl;
    if (!Addons::URDFReadFromFileWithModularity(file_path.c_str(), &m, jointnames_spanningtree,
                                                false)) {
      std::cerr << "Error loading urdf model" << std::endl;
      abort();
    }
  } else {
    std::cerr << "Unknown file type: Accepted file types is .urdf" << endl;
    abort();
  }

  // Declarations
  unsigned int pred_body_id, succ_body_id, cut_joint_id;
  Vector3d pred_body_pos, succ_body_pos, cut_joint_pos;
  Matrix3d rot_pred, rot_succ, rot_identity, rot_cut;
  SpatialTransform T0_p, T0_s, T0_k, Tp_k, Ts_k;
  Matrix3d rot_pk, rot_sk;
  Vector3d pos_pk, pos_sk;

  dof_spanningtree = jointnames_spanningtree.size();

  Q = VectorNd::Zero(m.dof_count);
  QDot = VectorNd::Zero(m.dof_count);
  UpdateKinematicsCustom(m, &Q, NULL, NULL);

  // Binding constraints to the model.
  for (uint i = 0; i < loop_constraints_set.size(); i++) {
    pred_body_id = m.GetBodyId(loop_constraints_set[i].pred_body.c_str());
    succ_body_id = m.GetBodyId(loop_constraints_set[i].succ_body.c_str());

    // Getting the cut joint id  ///Check for find function
    for (uint j = 0; j < jointnames_spanningtree.size(); j++)
      if (jointnames_spanningtree[j] == loop_constraints_set[i].jointname_cut)
        cut_joint_id = j + 1;

    // Setting the transformation matrix from root to predecessor and successor
    // body
    pred_body_pos = CalcBodyToBaseCoordinates(m, Q, pred_body_id, Vector3d(0., 0., 0.), false);
    succ_body_pos = CalcBodyToBaseCoordinates(m, Q, succ_body_id, Vector3d(0., 0., 0.), false);
    // cut_joint_pos = CalcBodyToBaseCoordinates(m, Q, cut_joint_id,
    //                                           Vector3d(0., 0., 0.), false);

    rot_pred = CalcBodyWorldOrientation(m, Q, pred_body_id).transpose();
    rot_succ = CalcBodyWorldOrientation(m, Q, succ_body_id).transpose();
    // rot_cut = CalcBodyWorldOrientation(m, Q, cut_joint_id).transpose();

    cut_joint_pos = pred_body_pos;
    rot_cut = rot_pred;
    T0_p = SpatialTransform(rot_pred, pred_body_pos);
    T0_s = SpatialTransform(rot_succ, succ_body_pos);
    T0_k = SpatialTransform(rot_cut, cut_joint_pos);

    // Predecessor and Successor Transform wrt to Cut joint
    rot_pk = T0_p.E.inverse() * T0_k.E;
    rot_sk = T0_s.E.inverse() * T0_k.E;
    pos_pk = T0_p.E.inverse() * (cut_joint_pos - pred_body_pos);
    pos_sk = T0_s.E.inverse() * (cut_joint_pos - succ_body_pos);

    Tp_k = SpatialTransform(rot_pk, pos_pk);
    Ts_k = SpatialTransform(rot_sk, pos_sk);

    for (uint j = 0; j < loop_constraints_set[i].constraint_axes.size(); j++) {
      cs.AddLoopConstraint(pred_body_id, succ_body_id, Tp_k, Ts_k,
                           loop_constraints_set[i].constraint_axes[j].axis, false,
                           loop_constraints_set[i].constraint_axes[j].baumg_stab_param,
                           loop_constraints_set[i].constraint_axes[j].name.c_str());
    }
  }
  cs.Bind(m);

  // Set the matrices to zero
  K = MatrixNd::Zero(cs.size(), m.dof_count);
  Kd = MatrixXd::Zero(dof_spanningtree - jointnames_independent.size(),
                      dof_spanningtree - jointnames_independent.size());
  Ki = MatrixXd::Zero(dof_spanningtree - jointnames_independent.size(),
                      jointnames_independent.size());
  Gd = MatrixXd::Zero(dof_spanningtree - jointnames_independent.size(),
                      jointnames_independent.size());
  Gi = MatrixNd::Identity(jointnames_independent.size(), jointnames_independent.size());
  gi = VectorXd::Zero(jointnames_independent.size());
  gd = VectorXd::Zero(dof_spanningtree - jointnames_independent.size());
  G = MatrixXd(dof_spanningtree, jointnames_independent.size());
  g = VectorXd(dof_spanningtree);
  k = VectorXd(cs.size());
  internal_G = MatrixXd(dof_spanningtree, jointnames_independent.size());
  internal_y = VectorXd(jointnames_independent.size());

  // Set up independent and dependent joints selection matrices
  calc_selection_matrices(jointnames_spanningtree, jointnames_independent);
  calc_permutationmatrix(jointnames_spanningtree, jointnames_active);

  QInit = VectorNd::Zero(m.dof_count);
  weights = VectorNd::Zero(m.dof_count);
  weights = independent_joints_selection_matrix.transpose() *
            VectorXd::Ones(jointnames_independent.size());

  // Check if intial position is zero or not.
  bool assembly_success;
  assembly_success = CalcAssemblyQ(m, QInit, cs, Q, weights);
  if (!assembly_success) {
    cerr << "WARNING: computation of assembly Q was not successful." << endl;
    abort();
  }
  // Test in zero position.
  VectorNd err(VectorNd::Zero(cs.size()));
  CalcConstraintsPositionError(m, Q, cs, err);
  if (verbose) {
    cout << "DOF: " << m.dof_count << endl;
    cout << "Constraint set size: " << cs.size() << endl;
    cout << "Initial Q: " << Q.transpose() << endl;
    cout << "Initial position error: " << err.transpose() << endl;
    cout << "Numerical Constructor executed successfully" << endl;
  }
}

VectorXd NumericalLoopConstraints::calc_loopclosure_function(const ::VectorNd& y) {
  bool succ;
  QInit = independent_joints_selection_matrix.transpose() * y +
          dependent_joints_selection_matrix.transpose() * dependent_joints_selection_matrix * Q;
  succ = CalcAssemblyQ(m, QInit, cs, Q, weights);
  if (succ) {
    internal_y = y;
    return Q;
  } else {
    VectorNd err(VectorNd::Zero(cs.size()));
    CalcConstraintsPositionError(m, Q, cs, err);
    cout << "Position error: " << err.transpose() << "\nQ: \n" << Q << endl;
    cerr << "y : " << y.transpose() << endl;
    throw std::runtime_error(
        "Assembly of Q was not succeessfull through numerical approach. The "
        "initial condition can be too far from the desired one.");
    // return Q;
  }
}

MatrixXd NumericalLoopConstraints::calc_loopclosure_Jacobian(const ::VectorNd& y) {
  if (internal_y.isApprox(y)) {
    CalcConstraintsJacobian(m, Q, cs, K);
  } else {
    Q = calc_loopclosure_function(y);
    CalcConstraintsJacobian(m, Q, cs, K);
  }
  G = calc_G_from_K(K);
  internal_G = G;
  return G;
}

VectorXd NumericalLoopConstraints::calc_loopclosure_g(const ::VectorNd& y, const ::VectorNd& ydot) {
  if (G.isApprox(internal_G)) {
    Q = calc_loopclosure_function(y);
    QDot = G * ydot;
  } else {
    G = calc_loopclosure_Jacobian(y);
    QDot = G * ydot;
  }
  k = calc_k(m, Q, QDot, cs);
  return calc_g_from_k(K, k);
}

MatrixXd NumericalLoopConstraints::calc_G_from_K(MatrixNd& K) {
  Kd = K * dependent_joints_selection_matrix.transpose();
  Ki = K * independent_joints_selection_matrix.transpose();

  Gd = -Kd.colPivHouseholderQr().solve(Ki);
  // G << Gi,Gd;
  G = independent_joints_selection_matrix.transpose() * Gi +
      dependent_joints_selection_matrix.transpose() * Gd;

  return G;
}

VectorXd NumericalLoopConstraints::calc_g_from_k(MatrixNd& K, VectorXd& k) {
  gd = Kd.colPivHouseholderQr().solve(k);
  // g << gi,gd;
  g = independent_joints_selection_matrix.transpose() * gi +
      dependent_joints_selection_matrix.transpose() * gd;
  return g;
}

VectorXd NumericalLoopConstraints::calc_k(Model& m, const Math::VectorNd& Q,
                                          const Math::VectorNd& QDot, ConstraintSet& CS) {
  // Code for computing k=-\dot(K)*\dot(q)
  Math::MatrixNd Gdummy;
  for (unsigned int k = 0; k < CS.loopConstraints.size(); ++k) {
    CS.loopConstraints[k]->calcGamma(m, 0., Q, QDot, Gdummy, CS.gamma, CS.cache, true);
  }
  return CS.gamma;
}

void NumericalLoopConstraints::calc_selection_matrices(
    const std::vector<string>& jointnames_spanningtree,
    const std::vector<string>& jointnames_independent) {
  independent_joints_selection_matrix.setZero(jointnames_independent.size(),
                                              jointnames_spanningtree.size());
  unsigned int n_row_dependent = jointnames_spanningtree.size() - jointnames_independent.size();
  dependent_joints_selection_matrix.setZero(n_row_dependent, jointnames_spanningtree.size());

  std::vector<int> independent_joints_index(jointnames_independent.size());
  std::vector<string> dependent_joints;

  // Loop for building independent_joints_selection_matrix

  for (unsigned int i = 0; i < jointnames_independent.size(); i++)
    for (unsigned int j = 0; j < jointnames_spanningtree.size(); j++) {
      if (jointnames_spanningtree[j] == jointnames_independent[i]) {
        independent_joints_selection_matrix(i, j) = 1;
        independent_joints_index[i] = j;
      }
    }

  for (uint i = 0; i < jointnames_spanningtree.size(); i++) {
    bool present = false;
    for (unsigned int j = 0; j < jointnames_independent.size(); j++)
      if (i == independent_joints_index[j])
        present = true;
    if (!present)
      dependent_joints.push_back(jointnames_spanningtree[i]);
  }

  for (unsigned int i = 0; i < n_row_dependent; i++)
    for (unsigned int j = 0; j < jointnames_spanningtree.size(); j++) {
      if (jointnames_spanningtree[j] == dependent_joints[i]) {
        dependent_joints_selection_matrix(i, j) = 1;
      }
    }
}

void NumericalLoopConstraints::calc_permutationmatrix(
    const std::vector<string>& jointnames_spanningtree,
    const std::vector<string>& jointnames_active) {
  permutation_matrix.setZero(jointnames_active.size(), jointnames_spanningtree.size());

  for (unsigned int i = 0; i < jointnames_active.size(); i++) {
    for (unsigned int j = 0; j < jointnames_spanningtree.size(); j++) {
      if (jointnames_spanningtree[j] == jointnames_active[i])
        permutation_matrix(i, j) = 1;
    }
  }
}

}  // namespace NUMERICALLOOPCONSTRAINTS
