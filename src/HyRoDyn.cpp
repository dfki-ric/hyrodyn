#include "HyRoDyn.hpp"

#include <ostream>

namespace HyRoDyn {

double wrap2pi(double x) {
  // This function wrap the x values into the interval (-pi, pi]
  // wrapx = x + n pi with n integer such that wrapx \in (-pi, pi]

  double wrapx;
  wrapx = atan2(sin(x), cos(x));
  //    cout<<"wrapx = "<<wrapx<<endl;
  return wrapx;
}

void calc_sysstate_q(Model &model, ExplicitLoopConstraintSet &elcs,
                     const Math::VectorNd &y, Math::VectorNd &q) {
  // This function calculates the position state of spanning tree in generalized
  // coordinates depending on the state of active joint positions

  q = elcs.calc_loopclosure_function(y);
}

void calc_actuatorstate_u(Model &model, ExplicitLoopConstraintSet &elcs,
                          const Math::VectorNd &y, Math::VectorNd &u) {
  // This function calculates the position state of active joints depending on
  // the state of independent joint positions

  VectorNd q = elcs.calc_loopclosure_function(y);

  u = elcs.get_permutation_matrix() * q;
}

void calc_independentjointstate_y(Model &model, ExplicitLoopConstraintSet &elcs,
                                  const Math::VectorNd &u, Math::VectorNd &y) {
  // This function calculates the position state of independent joints depending
  // on the state of active joint positions. This is kind of forward geometric
  // model in joint space.

  if (elcs.get_dof_independent() != elcs.get_dof_independent_robot()) {
    cerr << "Independent dof defined in urdf: " << elcs.get_dof_independent()
         << ", Independent dof belonging to the robot: "
         << elcs.get_dof_independent_robot() << endl;
    cerr << "This function is not supported for floating base robots! Please "
            "note that in these cases actuator state does not define "
            "independent joint state as the system is underactuated."
         << endl;
    abort();
  }

  unsigned int max_iterations = 20;
  double step_tol = 1.0e-08;

  VectorNd u_i(VectorNd::Zero(elcs.get_dof_active()));
  // VectorNd y_i(VectorNd::Zero(elcs.get_dof_active()));
  VectorNd du(VectorNd::Zero(elcs.get_dof_active()));
  VectorNd dy(VectorNd::Zero(elcs.get_dof_active()));

  VectorNd y_i = y;  // use the current value of y as initial guess

  MatrixXd Gu;
  Gu.setZero(elcs.get_dof_active(), elcs.get_dof_active());

  MatrixXd G(elcs.get_dof_spanningtree(), elcs.get_dof_independent());

  unsigned int i = 0;

  while (1) {
    // calculate actuator state u_i from y estimated
    calc_actuatorstate_u(model, elcs, y_i, u_i);

    du = u - u_i;

    // cout<<"error_norm:"<<du.norm()<<endl;

    if (du.norm() < step_tol) break;

    G = elcs.calc_loopclosure_Jacobian(y_i);

    Gu = elcs.get_permutation_matrix() * G;

    dy = (Gu).colPivHouseholderQr().solve(du);  //	dy = Gu_inv*du;

    y_i = y_i + dy;

    i = i + 1;
    if (i == max_iterations) {
      cerr << "FGM solution not found. Max iterations reached" << endl;
      break;
    }
  }

  for (int i = 0; i < elcs.get_dof_independent(); i++) y_i(i) = wrap2pi(y_i(i));

  y = y_i;
}

void calc_sysstate_qdot(Model &model, ExplicitLoopConstraintSet &elcs,
                        const Math::VectorNd &y, const Math::VectorNd &yd,
                        Math::VectorNd &qd) {
  // This function calculates the velocity state of spanning tree in generalized
  // coordinates depending on the state of active joint positions

  MatrixXd G(elcs.get_dof_spanningtree(), elcs.get_dof_active());
  G = elcs.calc_loopclosure_Jacobian(y);

  qd = G * yd;
}

void calc_actuatorstate_udot(Model &model, ExplicitLoopConstraintSet &elcs,
                             const Math::VectorNd &y, const Math::VectorNd &yd,
                             Math::VectorNd &ud) {
  // This function calculates the velocity state of active joints depending on
  // the state of independent joints

  MatrixXd G(elcs.get_dof_spanningtree(), elcs.get_dof_active());
  G = elcs.calc_loopclosure_Jacobian(y);

  ud = elcs.get_permutation_matrix() * G * yd;
}

void calc_independentjointstate_ydot(Model &model,
                                     ExplicitLoopConstraintSet &elcs,
                                     const Math::VectorNd &u,
                                     const Math::VectorNd &ud,
                                     Math::VectorNd &y, Math::VectorNd &yd) {
  // This function calculates the velocity state of independent joints depending
  // on the state of velocity in active joints

  if (elcs.get_dof_independent() != elcs.get_dof_independent_robot()) {
    cerr << "Independent dof defined in urdf: " << elcs.get_dof_independent()
         << ", Independent dof belonging to the robot: "
         << elcs.get_dof_independent_robot() << endl;
    cerr << "This function is not supported for floating base robots! Please "
            "note that in these cases actuator state does not define "
            "independent joint state as the system is underactuated."
         << endl;
    abort();
  }

  // VectorNd y(VectorNd::Zero(elcs.get_dof_active()));

  MatrixXd Gu;
  Gu.setZero(elcs.get_dof_active(), elcs.get_dof_active());

  MatrixXd G(elcs.get_dof_spanningtree(), elcs.get_dof_active());

  // compute the independent joint state from actuator state
  calc_independentjointstate_y(model, elcs, u, y);

  // Theory: ud = Gu(y)*yd or yd = Gu_inv*ud
  G = elcs.calc_loopclosure_Jacobian(y);
  Gu = elcs.get_permutation_matrix() * G;

  yd = (Gu).colPivHouseholderQr().solve(ud);  //	yd = Gu_inv*ud
}

void calc_sysstate_qddot(Model &model, ExplicitLoopConstraintSet &elcs,
                         const Math::VectorNd &y, const Math::VectorNd &yd,
                         const Math::VectorNd &ydd, Math::VectorNd &qdd) {
  // This function calculates the acceleration state of spanning tree in
  // generalized coordinates depending on the state of active joint positions

  MatrixNd G(elcs.get_dof_spanningtree(), elcs.get_dof_active());
  G = elcs.calc_loopclosure_Jacobian(y);

  /* MatrixNd Gdot(elcs.get_dof_spanningtree(), elcs.get_dof_active());
  Gdot = elcs.calc_loopclosure_Jacobiand(y, yd); */

  // g = Gdot * ydot
  VectorNd g(elcs.get_dof_spanningtree());
  g = elcs.calc_loopclosure_g(y, yd);

  qdd = G * ydd + g;
}

void calc_actuatorstate_uddot(Model &model, ExplicitLoopConstraintSet &elcs,
                              const Math::VectorNd &y, const Math::VectorNd &yd,
                              const Math::VectorNd &ydd, Math::VectorNd &udd) {
  // This function calculates the acceleration state of active joints depending
  // on the state of independent joints

  MatrixNd G(elcs.get_dof_spanningtree(), elcs.get_dof_active());
  G = elcs.calc_loopclosure_Jacobian(y);

  /* MatrixNd Gdot(elcs.get_dof_spanningtree(), elcs.get_dof_active());
  Gdot = elcs.calc_loopclosure_Jacobiand(y, yd); */

  VectorNd g(elcs.get_dof_spanningtree());
  g = elcs.calc_loopclosure_g(y, yd);
  udd = elcs.get_permutation_matrix() * (G * ydd + g);
}

void calc_independentjointstate_yddot(
    Model &model, ExplicitLoopConstraintSet &elcs, const Math::VectorNd &u,
    const Math::VectorNd &ud, const Math::VectorNd &udd, Math::VectorNd &y,
    Math::VectorNd &yd, Math::VectorNd &ydd) {
  // This function calculates the acceleration state of independent joints
  // depending on the state of acceleration in active joints

  if (elcs.get_dof_independent() != elcs.get_dof_independent_robot()) {
    cerr << "Independent dof defined in urdf: " << elcs.get_dof_independent()
         << ", Independent dof belonging to the robot: "
         << elcs.get_dof_independent_robot() << endl;
    cerr << "This function is not supported for floating base robots! Please "
            "note that in these cases actuator state does not define "
            "independent joint state as the system is underactuated."
         << endl;
    abort();
  }

  // VectorNd y(VectorNd::Zero(elcs.get_dof_active())),
  // yd(VectorNd::Zero(elcs.get_dof_active()));

  MatrixXd Gu;
  Gu.setZero(elcs.get_dof_active(), elcs.get_dof_active());

  MatrixXd G(elcs.get_dof_spanningtree(), elcs.get_dof_active());

  // compute the independent joint state from actuator state
  calc_independentjointstate_y(model, elcs, u, y);

  // Theory: udd = Gu(y)*ydd + Q*Gdot*ydot or ydd = Gu_inv*(udd - Q*Gdot*ydot)
  // Compute ydot
  G = elcs.calc_loopclosure_Jacobian(y);
  Gu = elcs.get_permutation_matrix() * G;
  yd = (Gu).colPivHouseholderQr().solve(ud);  //	yd = Gu_inv*ud
  // Compute yddot
  /* MatrixNd Gdot(elcs.get_dof_spanningtree(), elcs.get_dof_active());
  Gdot = elcs.calc_loopclosure_Jacobiand(y, yd); */

  VectorNd g(elcs.get_dof_spanningtree());
  g = elcs.calc_loopclosure_g(y, yd);

  ydd =
      (Gu).colPivHouseholderQr().solve(udd - elcs.get_permutation_matrix() * g);
}

std::vector<Math::VectorNd> calc_geometricmodel_forward(
    Model &model, ExplicitLoopConstraintSet &elcs, const Math::VectorNd &y,
    const std::vector<string> body_names) {
  // This function calculates the pose (position + orientation) of an input
  // bodies depending on the state of independent joint positions

  std::vector<Math::VectorNd> poses;

  // Update the position state of the spanning tree
  Math::VectorNd q(model.dof_count);
  calc_sysstate_q(model, elcs, y, q);

  for (unsigned int i = 0; i < body_names.size(); i++) {
    string body_name = body_names[i];

    Math::VectorNd pose(VectorNd::Zero(7));

    Math::Vector3d position;
    Quaternion orientation;

    unsigned int body_id = model.GetBodyId(body_name.c_str());

    if (model.IsBodyId(body_id)) {
      Vector3d point_position;
      point_position.setZero();

      // point_position(0) = 0.15; point_position(1) = 0; point_position(2) =
      // -0.07;	//test rh5 point_position(0) = 0.0170421582; point_position(1) =
      // -0.3916003704; point_position(2) = 0.0983130783; //recupera arm

      position = CalcBodyToBaseCoordinates(model, q, body_id, point_position);

      Matrix3d rot_mat;
      rot_mat.setIdentity(3, 3);
      rot_mat = CalcBodyWorldOrientation(model, q, body_id);
      orientation = orientation.fromMatrix(rot_mat);

      for (int i = 0; i < 3; i++) pose(i) = position(i);

      for (int i = 3; i < 7; i++) pose(i) = orientation(i - 3);

      poses.push_back(pose);

    } else {
      cerr << "calc_geometricmodel_forward: Body Name = " << body_name
           << " provided is not valid." << endl;
      abort();
    }
  }

  return poses;
}

Math::VectorNd calc_geometricmodel_forward(Model &model,
                                           ExplicitLoopConstraintSet &elcs,
                                           const Math::VectorNd &y,
                                           const char *body_name) {
  // This function calculates the pose (position + orientation) of an input body
  // depending on the state of independent joint positions

  Math::VectorNd pose(VectorNd::Zero(7));

  Math::Vector3d position;
  Quaternion orientation;

  unsigned int body_id = model.GetBodyId(body_name);
  if (model.IsBodyId(body_id)) {
    Math::VectorNd q(model.dof_count);
    calc_sysstate_q(model, elcs, y, q);

    Vector3d point_position;
    point_position.setZero();

    // point_position(0) = 0.15; point_position(1) = 0; point_position(2) =
    // -0.07;	//test rh5 point_position(0) = 0.0170421582; point_position(1) =
    // -0.3916003704; point_position(2) = 0.0983130783; //recupera arm

    position = CalcBodyToBaseCoordinates(model, q, body_id, point_position);

    Matrix3d rot_mat;
    rot_mat.setIdentity(3, 3);
    rot_mat = CalcBodyWorldOrientation(model, q, body_id);
    orientation = orientation.fromMatrix(rot_mat);

    for (int i = 0; i < 3; i++) pose(i) = position(i);

    for (int i = 3; i < 7; i++) pose(i) = orientation(i - 3);
  } else {
    cerr << "calc_geometricmodel_forward: Body Name provided is not valid."
         << endl;
    abort();
  }
  return pose;
}

void calc_com_jacobian_full(Model &model, const Math::VectorNd &q,
                            Math::MatrixNd &Jcom, Math::Vector3d *com) {
  // This function computes the full COM Jacobian of size (3xn) and optionally
  // the com position in base coordinates NOTE: this function should ideally
  // belong to RBDL and should be moved there!

  Jcom.setZero(3, model.dof_count);  // initiate a zero matrix
  double total_mass = 0.0;

  for (unsigned int i = 1; i < model.mBodies.size();
       i++) {  // iterate over the moving bodies except base link (numbered 0 in
               // the graph)

    const Body &body = model.mBodies.at(i);

    Math::MatrixNd com_jac_body_i;

    com_jac_body_i.setZero(3, model.dof_count);

    CalcPointJacobian(model, q, i, body.mCenterOfMass, com_jac_body_i, true);

    Jcom = Jcom + body.mMass * com_jac_body_i;

    total_mass = total_mass + body.mMass;

    if (com)
      *com = *com + body.mMass * CalcBodyToBaseCoordinates(model, q, i,
                                                           body.mCenterOfMass);
  }

  if (com) *com = *com / total_mass;

  Jcom = (1 / total_mass) * Jcom;
}

Vector3d CalcAngularVelocityfromMatrix(const Matrix3d &RotMat) {
  double tol = 1e-12;

  Vector3d l =
      Vector3d(RotMat(2, 1) - RotMat(1, 2), RotMat(0, 2) - RotMat(2, 0),
               RotMat(1, 0) - RotMat(0, 1));
  if (l.norm() > tol) {
    double preFactor = atan2(l.norm(), (RotMat.trace() - 1.0)) / l.norm();
    return preFactor * l;
  } else if ((RotMat(0, 0) > 0 && RotMat(1, 1) > 0 && RotMat(2, 2) > 0) ||
             l.norm() < tol) {
    return Vector3dZero;
  } else {
    double PI = atan(1) * 4.0;
    return Vector3d(PI / 2 * (RotMat(0, 0) + 1.0),
                    PI / 2 * (RotMat(1, 1) + 1.0),
                    PI / 2 * (RotMat(2, 2) + 1.0));
  }
}

bool IK_CS(Model &model, ExplicitLoopConstraintSet &elcs,
           const Math::VectorNd &Yinit, InverseKinematicsConstraintSet &CS,
           Math::VectorNd &Yres) {
  //  assert (Qinit.size() == model.q_size);
  //  assert (Qres.size() == Qinit.size());

  CS.J = MatrixNd::Zero(CS.num_constraints, model.qdot_size);
  CS.e = VectorNd::Zero(CS.num_constraints);

  Yres = Yinit;
  VectorNd Qres;

  for (CS.num_steps = 0; CS.num_steps < CS.max_steps; CS.num_steps++) {
    // convert Yres to Qres
    Qres = elcs.calc_loopclosure_function(Yres);
    // cout << "Q " << Qres.transpose() << std::endl;
    UpdateKinematicsCustom(model, &Qres, NULL, NULL);

    for (unsigned int k = 0; k < CS.body_ids.size(); k++) {
      CS.G = MatrixNd::Zero(6, model.qdot_size);
      CalcPointJacobian6D(model, Qres, CS.body_ids[k], CS.body_points[k], CS.G,
                          false);
      Vector3d point_base = CalcBodyToBaseCoordinates(
          model, Qres, CS.body_ids[k], CS.body_points[k], false);
      Matrix3d R = CalcBodyWorldOrientation(model, Qres, CS.body_ids[k], false);
      Vector3d angular_velocity =
          R.transpose() * CalcAngularVelocityfromMatrix(
                              R * CS.target_orientations[k].transpose());
      // For COM IK
      Vector3d com;
      MatrixNd Jcom;
      calc_com_jacobian_full(model, Qres, Jcom, &com);

      // assign offsets and Jacobians
      if (CS.constraint_type[k] ==
          InverseKinematicsConstraintSet::ConstraintTypeFull) {
        for (unsigned int i = 0; i < 3; i++) {
          unsigned int row = CS.constraint_row_index[k] + i;
          CS.e[row + 3] = CS.target_positions[k][i] - point_base[i];
          CS.e[row] = angular_velocity[i];
          for (unsigned int j = 0; j < model.qdot_size; j++) {
            CS.J(row + 3, j) = CS.G(i + 3, j);
            CS.J(row, j) = CS.G(i, j);
          }
        }
      } else if (CS.constraint_type[k] ==
                 InverseKinematicsConstraintSet::ConstraintTypeOrientation) {
        for (unsigned int i = 0; i < 3; i++) {
          unsigned int row = CS.constraint_row_index[k] + i;
          CS.e[row] = angular_velocity[i];
          for (unsigned int j = 0; j < model.qdot_size; j++) {
            CS.J(row, j) = CS.G(i, j);
          }
        }
      } else if (CS.constraint_type[k] ==
                 InverseKinematicsConstraintSet::ConstraintTypePosition) {
        for (unsigned int i = 0; i < 3; i++) {
          unsigned int row = CS.constraint_row_index[k] + i;
          CS.e[row] = CS.target_positions[k][i] - point_base[i];
          for (unsigned int j = 0; j < model.qdot_size; j++) {
            CS.J(row, j) = CS.G(i + 3, j);
          }
        }
      } else if (CS.constraint_type[k] ==
                 InverseKinematicsConstraintSet::ConstraintTypeCOM) {
        for (unsigned int i = 0; i < 3; i++) {
          unsigned int row = CS.constraint_row_index[k] + i;
          CS.e[row] = CS.target_positions[k][i] - com[i];
          for (unsigned int j = 0; j < model.qdot_size; j++) {
            CS.J(row, j) = Jcom(i, j);
          }
        }
      } else {
        assert(false && !"Invalid inverse kinematics constraint");
      }
    }

    LOG << "J = " << CS.J << std::endl;
    LOG << "e = " << CS.e.transpose() << std::endl;
    CS.error_norm = CS.e.norm();

    // abort if we are getting "close"
    if (CS.error_norm < CS.step_tol) {
      cout << "Reached target close enough after " << CS.num_steps << " steps"
           << std::endl;
      LOG << "Reached target close enough after " << CS.num_steps << " steps"
          << std::endl;
      return true;
    }

    double Ek = 0.;

    for (size_t ei = 0; ei < CS.e.size(); ei++) {
      Ek += CS.e[ei] * CS.e[ei] * 0.5;
    }

    MatrixNd temp = CS.J * elcs.calc_loopclosure_Jacobian(Yres);

    VectorNd ek = temp.transpose() * CS.e;
    MatrixNd Wn = MatrixNd::Zero(Yres.size(), Yres.size());

    assert(ek.size() == Yres.size());

    for (size_t wi = 0; wi < Yres.size(); wi++) {
      Wn(wi, wi) = ek[wi] * ek[wi] * 0.5 + CS.lambda;
      //      Wn(wi, wi) = Ek + 1.0e-3;
    }

    MatrixNd A = temp.transpose() * temp + Wn;
    VectorNd delta_theta =
        A.colPivHouseholderQr().solve(temp.transpose() * CS.e);

    Yres = Yres + delta_theta;
    if (delta_theta.norm() < CS.step_tol) {
      cout << "reached convergence after " << CS.num_steps << " steps"
           << std::endl;
      LOG << "reached convergence after " << CS.num_steps << " steps"
          << std::endl;
      return true;
    }
  }

  return false;
}

void calc_geometricmodel_inverse(Model &model, ExplicitLoopConstraintSet &elcs,
                                 const std::vector<Math::VectorNd> &pose,
                                 const std::vector<string> body_name,
                                 Math::VectorNd &y, Vector3d com_input,
                                 double error_tolerance) {
  // This function calculates the state of independent joint positions depending
  // on the pose (position + orientation) of an input body

  if (pose.size() != body_name.size()) {
    cerr << "Size of input pose vector: " << pose.size()
         << ", Size of body name vector: " << body_name.size() << endl;
    cerr << "Size of the input pose vector should be equal to the size of body "
            "name vector."
         << endl;
    abort();
  }

  InverseKinematicsConstraintSet cs;
  cs.step_tol = error_tolerance;  // set the IK numerical error tolerance

  Vector3d base_point_position;
  for (unsigned int i = 0; i < 3; i++) base_point_position(i) = 0.0;

  for (unsigned int i = 0; i < pose.size(); i++) {
    VectorNd pose_element =
        pose[i];  // extract the element from the pose vector

    Vector3d position;
    Vector4d orientation_quat(pose_element(3), pose_element(4), pose_element(5),
                              pose_element(6));

    for (unsigned int i = 0; i < 3; i++) position(i) = pose_element(i);

    Matrix3d G_R_E = Quaternion(pose_element(3), pose_element(4),
                                pose_element(5), pose_element(6))
                         .toMatrix();

    unsigned int body_id = model.GetBodyId(body_name[i].c_str());

    if (!model.IsBodyId(body_id)) {
      cerr << "calc_geometricmodel_inverse: Body Name provided is not valid."
           << endl;
      abort();
    }

    if (orientation_quat.norm() ==
        0.)  // if orientation quat vector norm is zero (which means quaternion
             // vector is invalid), position only IK will be solved.
      cs.AddPointConstraint(body_id, base_point_position, position);
    else if (position.norm() == 0.)  // if position norm is zero, orientation
                                     // only IK will be solved.
      cs.AddOrientationConstraint(body_id, G_R_E);
    else  // otherwise, full IK (position+orientation) will be solved
      cs.AddFullConstraint(body_id, base_point_position, position, G_R_E);

    // Quaternion q;

    // cout<<"IGM input for body name:"<<body_name[i]<<" X =
    // "<<position.transpose()<<" q = "<<q.fromMatrix (G_R_E).transpose<<endl;

    cout << "IGM input for body name:" << body_name[i]
         << " X = " << pose_element.transpose() << endl;
  }
  // Add COM constraint
  if (!com_input.isZero()) {
    cs.AddCOMConstraint(com_input);
    cout << "IGM input for COM:" << com_input.transpose() << endl;
  }

  VectorNd YInit(VectorNd::Zero(elcs.get_dof_independent())),
      Yres(VectorNd::Zero(elcs.get_dof_independent()));

  YInit = y;  // init the IK with last valid config of independent joint state

  if (IK_CS(model, elcs, YInit, cs, Yres)) {
    cout << "IGM: Solved" << endl;
    for (int i = 0; i < elcs.get_dof_independent(); i++)
      Yres(i) = wrap2pi(Yres(i));
    y = Yres;  // store the new IK result
  } else {
    cout << "IGM: Could not find a solution. Last independent joint state will "
            "not be updated."
         << endl;
  }

  std::cout << "IGM Output Yres (indepdendent_dof): " << Yres.transpose()
            << std::endl;
}

VectorNd calc_constrained_dynamicmodel_inverse(
    Model &model, ExplicitLoopConstraintSet &elcs, const Math::VectorNd &y,
    const Math::VectorNd &yd, const Math::SpatialVector &xdd,
    const Math::SpatialVector &f_ext, const string body_name) {
  unsigned int body_id = model.GetBodyId(body_name.c_str());

  if (!model.IsBodyId(body_id)) {
    cerr << "calc_constrained_dynamicmodel_inverse: Body Name provided is not "
            "valid."
         << endl;
    abort();
  }

  VectorXd ydd(elcs.get_dof_independent());

  // NOTE: Be careful that MatrixXd variables like H matrix are set to zero
  // before used.
  MatrixXd H;  // mass matrix
  H.setZero(model.dof_count, model.dof_count);
  VectorXd C(model.dof_count);  // coriolis and gravity terms

  MatrixXd Gu;
  Gu.setZero(elcs.get_dof_active(), elcs.get_dof_active());

  VectorXd q(model.dof_count);
  calc_sysstate_q(model, elcs, y, q);

  VectorXd qd(model.dof_count);
  calc_sysstate_qdot(model, elcs, y, yd, qd);

  MatrixXd G(elcs.get_dof_spanningtree(), elcs.get_dof_independent());
  G = elcs.calc_loopclosure_Jacobian(y);
  Gu = elcs.get_permutation_matrix() * G *
       elcs.get_permutation_matrix2().transpose();

  // VectorXd g(elcs.get_dof_spanningtree());
  // g = elcs.calc_loopclosure_Jacobiand(y, yd) * yd;

  VectorNd g(elcs.get_dof_spanningtree());
  g = elcs.calc_loopclosure_g(y, yd);

  // Compute the bias force
  NonlinearEffects(model, q, qd, C);
  // Compute the mass-inertia matrix with CRBA in O(n) time
  CompositeRigidBodyAlgorithm(model, q, H);

  MatrixXd H_y;  // mass matrix
  H_y.setZero(elcs.get_dof_independent(), elcs.get_dof_independent());
  H_y = G.transpose() * H *
        G;  // compute the mass matrix projected in the independent joint space

  VectorXd C_y = VectorXd::Zero(elcs.get_dof_independent());
  C_y = G.transpose() *
        (C + H * g);  // compute the bias forces in independent joint space

  MatrixXd point_Jacobian;
  Math::Vector3d point_position;
  point_position.setZero();
  point_Jacobian.setZero(6, model.dof_count);
  CalcPointJacobian6D(
      model, q, body_id, point_position,
      point_Jacobian);  // compute point jacobian 6D for this point (6 x n) in
                        // base coordinate system

  MatrixXd
      Jacobian_y;  // jacobian in independent joint space, J(y): ydot -> Xdot
  Jacobian_y.setZero(6, elcs.get_dof_independent());  // size is (6 x m)
  Jacobian_y = point_Jacobian * G;

  unsigned int floating_base_dof =
      elcs.get_dof_independent() - elcs.get_dof_independent_robot();
  MatrixXd selection_matrix;
  selection_matrix.setZero(elcs.get_dof_independent(),
                           elcs.get_dof_independent());
  selection_matrix.block(floating_base_dof, floating_base_dof,
                         elcs.get_dof_independent_robot(),
                         elcs.get_dof_independent_robot()) =
      MatrixXd::Identity(elcs.get_dof_independent_robot(),
                         elcs.get_dof_independent_robot());

  MatrixXd Z;
  Z.setZero(elcs.get_dof_independent() + 6, 2 * elcs.get_dof_independent());
  Z.block(0, 0, elcs.get_dof_independent(), elcs.get_dof_independent()) = H_y;
  Z.block(elcs.get_dof_independent(), 0, 6, elcs.get_dof_independent()) =
      Jacobian_y;
  Z.block(0, elcs.get_dof_independent(), elcs.get_dof_independent(),
          elcs.get_dof_independent()) = selection_matrix;
  cout << "Z: " << endl << Z << endl;

  SpatialVector Jacobiandot_y_dot_yd =
      Jacobian_y * g + CalcPointAcceleration6D(model, q, qd,
                                               VectorXd::Zero(model.dof_count),
                                               body_id, point_position);

  VectorXd f = VectorXd::Zero(6 + elcs.get_dof_independent());
  f.head(elcs.get_dof_independent()) = -C_y + Jacobian_y.transpose() * f_ext;
  f.tail(6) = xdd - Jacobiandot_y_dot_yd;

  double regularizer = 0.01;

  // Solve the constrained inverse dynamics
  VectorXd qdd_Tau_y_vector =
      (Z.transpose() * Z +
       regularizer * MatrixXd::Identity(2 * elcs.get_dof_independent(),
                                        2 * elcs.get_dof_independent()))
          .colPivHouseholderQr()
          .solve(Z.transpose() * f);
  cout << "qdd_Tau_y_vector: " << qdd_Tau_y_vector.transpose() << endl;

  // Extract the generalized forces
  VectorXd Tau_independentjointspace =
      qdd_Tau_y_vector.tail(elcs.get_dof_independent());
  cout << "Tau_independentjointspace: " << Tau_independentjointspace.transpose()
       << endl;

  // Convert the generalized forces into actuator forces
  VectorXd Tau_actuated =
      (Gu.transpose())
          .colPivHouseholderQr()
          .solve(elcs.get_permutation_matrix2() * Tau_independentjointspace);
  cout << "Tau_actuated: " << Tau_actuated.transpose() << endl;

  return Tau_actuated;
}

void calc_mass_interia_matrix(Model &model, ExplicitLoopConstraintSet &elcs,
                              const Math::VectorNd &y, Math::MatrixNd &H) {
  // NOTE: Be careful that MatrixXd variables like H matrix are set to zero
  // before used.
  MatrixXd H_q;  // mass-interia matrix in spanning tree space
  H_q.setZero(model.dof_count, model.dof_count);

  VectorXd q(model.dof_count);
  calc_sysstate_q(model, elcs, y, q);

  MatrixXd G(elcs.get_dof_spanningtree(), elcs.get_dof_independent());
  G = elcs.calc_loopclosure_Jacobian(y);

  // Compute the mass-inertia matrix with CRBA in O(nd) time
  CompositeRigidBodyAlgorithm(model, q, H_q);

  H.setZero(elcs.get_dof_independent(), elcs.get_dof_independent());
  H = G.transpose() * H_q *
      G;  // compute the mass matrix projected in the independent joint space
}

void calc_mass_interia_matrix_actuation_space(Model &model,
                                              ExplicitLoopConstraintSet &elcs,
                                              const Math::VectorNd &y,
                                              Math::MatrixNd &Hu) {
  // Compute the full system state from y
  VectorXd q(model.dof_count);
  calc_sysstate_q(model, elcs, y, q);

  // Compute the loop closure jacobian
  MatrixXd G(elcs.get_dof_spanningtree(), elcs.get_dof_independent());
  G = elcs.calc_loopclosure_Jacobian(y);

  // Compute the actuator jocobian Gu
  MatrixXd Gu;
  Gu.setZero(elcs.get_dof_active(), elcs.get_dof_active());
  Gu = elcs.get_permutation_matrix() * G *
       elcs.get_permutation_matrix2().transpose();

  // Compute the mass-inertia matrix with CRBA in O(nd) time
  MatrixXd H_q;  // mass-interia matrix in spanning tree space
  H_q.setZero(model.dof_count, model.dof_count);
  CompositeRigidBodyAlgorithm(model, q, H_q);

  Hu.setZero(elcs.get_dof_active(), elcs.get_dof_active());
  Hu = Gu.inverse().transpose() * G.transpose() * H_q * G *
       Gu.inverse();  // compute the mass matrix projected in the independent
                      // joint space
}

void calc_mass_interia_matrix_actuation_space_including_floating_base(
    Model &model, ExplicitLoopConstraintSet &elcs, const Math::VectorNd &y,
    Math::MatrixNd &Hufb) {
  // Compute the full system state from y
  VectorXd q(model.dof_count);
  calc_sysstate_q(model, elcs, y, q);

  // Compute the loop closure jacobian
  MatrixXd G(elcs.get_dof_spanningtree(), elcs.get_dof_independent());
  G = elcs.calc_loopclosure_Jacobian(y);

  // Compute the actuator jocobian Gu
  MatrixXd Gu;
  Gu.setZero(elcs.get_dof_active(), elcs.get_dof_active());
  Gu = elcs.get_permutation_matrix() * G *
       elcs.get_permutation_matrix2().transpose();

  // Compute the mass-inertia matrix with CRBA in O(nd) time
  MatrixXd H_q;  // mass-interia matrix in spanning tree space
  H_q.setZero(model.dof_count, model.dof_count);
  CompositeRigidBodyAlgorithm(model, q, H_q);

  MatrixXd Hy;  // mass-interia matrix in independent joint space
  Hy.setZero(elcs.get_dof_independent(), elcs.get_dof_independent());
  Hy = G.transpose() * H_q * G;

  Hufb.setZero(elcs.get_dof_floatingbase() + elcs.get_dof_active(),
               elcs.get_dof_floatingbase() + elcs.get_dof_active());
  // top left block
  Hufb.topLeftCorner(elcs.get_dof_floatingbase(), elcs.get_dof_floatingbase()) =
      Hy.topLeftCorner(elcs.get_dof_floatingbase(),
                       elcs.get_dof_floatingbase());
  // top right block
  Hufb.topRightCorner(elcs.get_dof_floatingbase(),
                      elcs.get_dof_independent_robot()) =
      Hy.topRightCorner(elcs.get_dof_floatingbase(),
                        elcs.get_dof_independent_robot()) *
      Gu.inverse();
  // bottom left block
  Hufb.bottomLeftCorner(elcs.get_dof_independent_robot(),
                        elcs.get_dof_floatingbase()) =
      Gu.inverse().transpose() *
      Hy.bottomLeftCorner(elcs.get_dof_independent_robot(),
                          elcs.get_dof_floatingbase());
  // bottom right block
  Hufb.bottomRightCorner(elcs.get_dof_independent_robot(),
                         elcs.get_dof_independent_robot()) =
      Gu.inverse().transpose() *
      Hy.bottomRightCorner(elcs.get_dof_independent_robot(),
                           elcs.get_dof_independent_robot()) *
      Gu.inverse();
}

SpatialVector calc_kinematicmodel_forward(Model &model,
                                          ExplicitLoopConstraintSet &elcs,
                                          const Math::VectorNd &y,
                                          const Math::VectorNd &yd,
                                          const char *body_name) {
  // This function calculates the twist (angular + linear velocity) of an input
  // body depending on the state of independent joint positions and velocities

  SpatialVector twist;

  unsigned int body_id = model.GetBodyId(body_name);
  if (model.IsBodyId(body_id)) {
    // calculate generalized positions(q)
    Math::VectorNd q(model.dof_count);
    calc_sysstate_q(model, elcs, y, q);

    // calculate generalized velocities(qd)
    Math::VectorNd qd(model.dof_count);
    MatrixXd G(elcs.get_dof_spanningtree(), elcs.get_dof_independent());
    G = elcs.calc_loopclosure_Jacobian(y);
    qd = G * yd;

    Vector3d point_position;
    point_position.setZero();

    // calculate twist
    twist = CalcPointVelocity6D(model, q, qd, body_id, point_position);

    return twist;
  } else {
    cerr << "calc_kinematicmodel_forward: Body Name provided is not valid."
         << endl;
    abort();
  }
}

std::vector<SpatialVector> calc_kinematicmodel_forward(
    Model &model, ExplicitLoopConstraintSet &elcs, const Math::VectorNd &y,
    const Math::VectorNd &yd, std::vector<string> body_names) {
  // This function calculates the twists (angular + linear velocity) of multiple
  // input bodies depending on the state of independent joint positions and
  // velocities

  std::vector<SpatialVector> twists;

  // calculate generalized positions(q)
  Math::VectorNd q(model.dof_count);
  calc_sysstate_q(model, elcs, y, q);

  // calculate generalized velocities(qd)
  Math::VectorNd qd(model.dof_count);
  MatrixXd G(elcs.get_dof_spanningtree(), elcs.get_dof_independent());
  G = elcs.calc_loopclosure_Jacobian(y);
  qd = G * yd;

  Vector3d point_position;
  point_position.setZero();

  for (unsigned int i = 0; i < body_names.size(); i++) {
    string body_name = body_names[i];
    unsigned int body_id = model.GetBodyId(body_name.c_str());
    if (model.IsBodyId(body_id)) {
      // calculate twist and push it to the twists vector
      twists.push_back(
          CalcPointVelocity6D(model, q, qd, body_id, point_position));

    } else {
      cerr << "calc_kinematicmodel_forward: Body Name = " << body_name
           << " provided is not valid." << endl;
      abort();
    }
  }

  return twists;
}

SpatialVector calc_secondorder_kinematicmodel_forward(
    Model &model, ExplicitLoopConstraintSet &elcs, const Math::VectorNd &y,
    const Math::VectorNd &yd, const Math::VectorNd &ydd,
    const char *body_name) {
  // This function calculates the spatial acceleration (angular + linear
  // acceleration) of an input body depending on the state of independent joint
  // positions, velocities and acceleration

  SpatialVector spatial_acceleration;

  unsigned int body_id = model.GetBodyId(body_name);
  if (model.IsBodyId(body_id)) {
    // calculate generalized positions(q)
    Math::VectorNd q(model.dof_count);
    calc_sysstate_q(model, elcs, y, q);

    // calculate generalized velocities(qd)
    Math::VectorNd qd(model.dof_count);
    MatrixXd G(elcs.get_dof_spanningtree(), elcs.get_dof_independent());
    G = elcs.calc_loopclosure_Jacobian(y);
    qd = G * yd;

    // calculate generalized accelerations(qdd)
    Math::VectorNd qdd(model.dof_count);
    /* MatrixNd Gdot(elcs.get_dof_spanningtree(), elcs.get_dof_active());
    Gdot = elcs.calc_loopclosure_Jacobiand(y, yd); */
    VectorNd g(elcs.get_dof_spanningtree());
    g = elcs.calc_loopclosure_g(y, yd);
    qdd = G * ydd + g;

    Vector3d point_position;
    point_position.setZero();

    // calculate twist derivative in space coordinates (hybrid representation of
    // twist)
    spatial_acceleration =
        CalcPointAcceleration6D(model, q, qd, qdd, body_id, point_position);

    return spatial_acceleration;
  } else {
    cerr << "calc_secondorder_kinematicmodel_forward: Body Name provided is "
            "not valid."
         << endl;
    abort();
  }
}

std::vector<SpatialVector> calc_secondorder_kinematicmodel_forward(
    Model &model, ExplicitLoopConstraintSet &elcs, const Math::VectorNd &y,
    const Math::VectorNd &yd, const Math::VectorNd &ydd,
    std::vector<string> body_names) {
  // This function calculates the spatial accelerations (angular + linear
  // acceleration) of multiple input bodies depending on the state of
  // independent joint positions, velocities and acceleration

  std::vector<SpatialVector> spatial_accelerations;

  // calculate generalized positions(q)
  Math::VectorNd q(model.dof_count);
  calc_sysstate_q(model, elcs, y, q);

  // calculate generalized velocities(qd)
  Math::VectorNd qd(model.dof_count);
  MatrixXd G(elcs.get_dof_spanningtree(), elcs.get_dof_independent());
  G = elcs.calc_loopclosure_Jacobian(y);
  qd = G * yd;

  // calculate generalized accelerations(qdd)
  Math::VectorNd qdd(model.dof_count);
  MatrixNd Gdot(elcs.get_dof_spanningtree(), elcs.get_dof_active());
  Gdot = elcs.calc_loopclosure_Jacobiand(y, yd);
  qdd = G * ydd + Gdot * yd;

  Vector3d point_position;
  point_position.setZero();

  for (unsigned int i = 0; i < body_names.size(); i++) {
    string body_name = body_names[i];
    unsigned int body_id = model.GetBodyId(body_name.c_str());
    if (model.IsBodyId(body_id)) {
      // calculate twist derivative in space coordinates (hybrid representation
      // of twist) and push it into the vector
      spatial_accelerations.push_back(
          CalcPointAcceleration6D(model, q, qd, qdd, body_id, point_position));
    } else {
      cerr << "calc_secondorder_kinematicmodel_forward: Body Name = "
           << body_name << " provided is not valid." << endl;
      abort();
    }
  }
  return spatial_accelerations;
}

void calc_actuator_jacobian(Model &model, ExplicitLoopConstraintSet &elcs,
                            const Math::VectorNd &y, Math::MatrixNd &Gu) {
  Gu.setZero(elcs.get_dof_active(), elcs.get_dof_active());

  MatrixXd G(elcs.get_dof_spanningtree(), elcs.get_dof_independent());
  G = elcs.calc_loopclosure_Jacobian(y);

  Gu = elcs.get_permutation_matrix() * G *
       elcs.get_permutation_matrix2().transpose();
}

void calc_spatial_jacobian(Model &model, ExplicitLoopConstraintSet &elcs,
                           const Math::VectorNd &y, Math::MatrixNd &J,
                           const char *body_name) {
  // This function computes the spatial Jacobian (6 x n) of a body depending on
  // the state of position of independent joints.
  J.setZero(6, elcs.get_dof_spanningtree());
  unsigned int body_id = model.GetBodyId(body_name);
  if (model.IsBodyId(body_id)) {
    // calculate generalized positions(q)
    Math::VectorNd q(model.dof_count);
    calc_sysstate_q(model, elcs, y, q);
    Vector3d point_position;
    point_position.setZero();
    // point_position(0) = 0.15; point_position(1) = 0; point_position(2) =
    // -0.07;	//test point_position(0) = 0.0170421582; point_position(1) =
    // -0.3916003704; point_position(2) = 0.0983130783; //recupera arm
    CalcPointJacobian6D(model, q, body_id, point_position, J);
  } else {
    cerr << "calc_spatial_jacobian: Body Name provided is not valid." << endl;
    abort();
  }
}

void calc_spatial_jacobian_independent_joint_space(
    Model &model, ExplicitLoopConstraintSet &elcs, const Math::VectorNd &y,
    Math::MatrixNd &J, const char *body_name) {
  // This function computes the spatial Jacobian (6 x m) of a body depending on
  // the state of position of independent joints projected to independent joint
  // space.
  J.setZero(6, elcs.get_dof_independent());
  MatrixXd J_full = MatrixXd::Zero(6, elcs.get_dof_spanningtree());
  // compute the full spatial Jacobian
  calc_spatial_jacobian(model, elcs, y, J_full, body_name);
  MatrixXd G =
      MatrixXd::Zero(elcs.get_dof_spanningtree(), elcs.get_dof_independent());
  G = elcs.calc_loopclosure_Jacobian(y);  // compute the loop closure jacobian
  J = J_full * G;  // projection in independent joint space
}

void calc_spatial_jacobian_actuation_space(Model &model,
                                           ExplicitLoopConstraintSet &elcs,
                                           const Math::VectorNd &y,
                                           Math::MatrixNd &J,
                                           const char *body_name) {
  // This function computes the spatial Jacobian (6 x p) of a body depending on
  // the state of position of independent joints projected to actuation space.
  J.setZero(6, elcs.get_dof_active());
  MatrixXd J_full = MatrixXd::Zero(6, elcs.get_dof_spanningtree());
  // compute the full spatial Jacobian
  calc_spatial_jacobian(model, elcs, y, J_full, body_name);
  MatrixXd G =
      MatrixXd::Zero(elcs.get_dof_spanningtree(), elcs.get_dof_independent());
  G = elcs.calc_loopclosure_Jacobian(y);  // compute the loop closure jacobian
  MatrixXd Gu = MatrixXd::Zero(elcs.get_dof_active(), elcs.get_dof_active());
  calc_actuator_jacobian(model, elcs, y, Gu);  // actuator jacobian
  J = J_full * G * (Gu.inverse());             // projection in actuation space
}

void calc_spatial_jacobian_actuation_space_including_floating_base(
    Model &model, ExplicitLoopConstraintSet &elcs, const Math::VectorNd &y,
    Math::MatrixNd &J, const char *body_name) {
  // This function computes the spatial Jacobian (6 x floating_dof+p) of a body
  // depending on the state of position of independent joints projected to
  // actuation space (but includes floating base joints).
  J.setZero(6, elcs.get_dof_floatingbase() + elcs.get_dof_active());
  Math::MatrixNd Jy;
  Jy.setZero(6, elcs.get_dof_independent());
  calc_spatial_jacobian_independent_joint_space(
      model, elcs, y, Jy,
      body_name);  // spatial jacobian projected to independent joint space
  MatrixXd Gu = MatrixXd::Zero(elcs.get_dof_active(), elcs.get_dof_active());
  calc_actuator_jacobian(model, elcs, y, Gu);  // actuator jacobian
  J << Jy.leftCols(elcs.get_dof_floatingbase()),
      Jy.rightCols(elcs.get_dof_independent_robot()) *
          (Gu.inverse());  // assumes independent dof of robot equals active dof
}

void calc_body_jacobian(Model &model, ExplicitLoopConstraintSet &elcs,
                        const Math::VectorNd &y, Math::MatrixNd &J,
                        const char *body_name) {
  // This function computes the body Jacobian (6 x n) of a body depending on the
  // state of position of independent joints.
  J.setZero(6, elcs.get_dof_spanningtree());
  unsigned int body_id = model.GetBodyId(body_name);
  if (model.IsBodyId(body_id)) {
    // calculate full spanning tree positions(q)
    Math::VectorNd q(model.dof_count);
    calc_sysstate_q(model, elcs, y, q);
    CalcBodySpatialJacobian(model, q, body_id, J);
  } else {
    cerr << "calc_body_jacobian: Body Name provided is not valid." << endl;
    abort();
  }
}

void calc_body_jacobian_independent_joint_space(Model &model,
                                                ExplicitLoopConstraintSet &elcs,
                                                const Math::VectorNd &y,
                                                Math::MatrixNd &J,
                                                const char *body_name) {
  // This function computes the body Jacobian (6 x m) of a body depending on the
  // state of position of independent joints projected to independent joint
  // space.
  J.setZero(6, elcs.get_dof_independent());
  MatrixXd J_full = MatrixXd::Zero(6, elcs.get_dof_spanningtree());
  // compute the full spatial Jacobian
  calc_body_jacobian(model, elcs, y, J_full, body_name);
  MatrixXd G =
      MatrixXd::Zero(elcs.get_dof_spanningtree(), elcs.get_dof_independent());
  G = elcs.calc_loopclosure_Jacobian(y);  // compute the loop closure jacobian
  J = J_full * G;  // projection in independent joint space
}

void calc_body_jacobian_actuation_space(Model &model,
                                        ExplicitLoopConstraintSet &elcs,
                                        const Math::VectorNd &y,
                                        Math::MatrixNd &J,
                                        const char *body_name) {
  // This function computes the body Jacobian (6 x p) of a body depending on the
  // state of position of independent joints projected to actuation space.
  J.setZero(6, elcs.get_dof_independent());
  MatrixXd J_full = MatrixXd::Zero(6, elcs.get_dof_spanningtree());
  // compute the full spatial Jacobian
  calc_body_jacobian(model, elcs, y, J_full, body_name);
  MatrixXd G =
      MatrixXd::Zero(elcs.get_dof_spanningtree(), elcs.get_dof_independent());
  G = elcs.calc_loopclosure_Jacobian(y);  // compute the loop closure jacobian
  MatrixXd Gu = MatrixXd::Zero(elcs.get_dof_active(), elcs.get_dof_active());
  calc_actuator_jacobian(model, elcs, y, Gu);  // actuator jacobian
  J = J_full * G * (Gu.inverse());             // projection in actuation space
}

void calc_body_jacobian_actuation_space_including_floating_base(
    Model &model, ExplicitLoopConstraintSet &elcs, const Math::VectorNd &y,
    Math::MatrixNd &J, const char *body_name) {
  // This function computes the body Jacobian (6 x floating_dof+p) of a body
  // depending on the state of position of independent joints projected to
  // actuation space.
  J.setZero(6, elcs.get_dof_floatingbase() + elcs.get_dof_active());
  Math::MatrixNd Jy;
  Jy.setZero(6, elcs.get_dof_independent());
  calc_body_jacobian_independent_joint_space(
      model, elcs, y, Jy,
      body_name);  // body jacobian projected to independent joint space
  MatrixXd Gu = MatrixXd::Zero(elcs.get_dof_active(), elcs.get_dof_active());
  calc_actuator_jacobian(model, elcs, y, Gu);  // actuator jacobian
  J << Jy.leftCols(elcs.get_dof_floatingbase()),
      Jy.rightCols(elcs.get_dof_independent_robot()) *
          (Gu.inverse());  // assumes independent dof of robot equals active dof
}

void calc_point_jacobian(Model &model, ExplicitLoopConstraintSet &elcs,
                         const Math::VectorNd &y, Math::MatrixNd &J,
                         const char *body_name) {
  // This function computes the point Jacobian (3 x n) of a body depending on
  // the state of position of independent joints.

  unsigned int body_id = model.GetBodyId(body_name);
  if (model.IsBodyId(body_id)) {
    // calculate generalized positions(q)
    Math::VectorNd q(model.dof_count);
    calc_sysstate_q(model, elcs, y, q);
    Vector3d point_position;
    point_position.setZero();
    // point_position(0) = 0.15; point_position(1) = 0; point_position(2) =
    // -0.07;	//test point_position(0) = 0.0170421582; point_position(1) =
    // -0.3916003704; point_position(2) = 0.0983130783; //recupera arm
    CalcPointJacobian(model, q, body_id, point_position, J);
  } else {
    cerr << "calc_point_jacobian: Body Name provided is not valid." << endl;
    abort();
  }
}

void calc_com_jacobian(Model &model, ExplicitLoopConstraintSet &elcs,
                       const Math::VectorNd &y, Math::MatrixNd &Jcom) {
  Jcom.setZero(3, elcs.get_dof_independent());

  Math::VectorNd q(model.dof_count);
  calc_sysstate_q(model, elcs, y,
                  q);  // compute full system state from
                       // independent joint state (position only)

  MatrixXd G(elcs.get_dof_spanningtree(), elcs.get_dof_independent());
  G = elcs.calc_loopclosure_Jacobian(y);  // compute the loop closure jacobian

  MatrixXd J;
  J.setZero(3, model.dof_count);  // initiate a zero matrix

  calc_com_jacobian_full(model, q, J, NULL);

  Jcom = J * G;  // projection in independent joint space
}

Math::SpatialTransform calc_adjoint_transformation(
    Model &model, ExplicitLoopConstraintSet &elcs, const Math::VectorNd &y,
    const char *body_name) {
  // This function computes the adjoint transformation (6 x 6) of a body in base
  // coordinates depending on the state of position of independent joints.

  unsigned int body_id = model.GetBodyId(body_name);
  if (!model.IsBodyId(body_id)) {
    cerr << "calc_adjoint_transformation: Body Name provided is not valid."
         << endl;
    abort();
  }
  Math::VectorNd q(model.dof_count);
  calc_sysstate_q(model, elcs, y,
                  q);  // compute full state of the spanning tree
                       // in order to update robot kinematics

  Matrix3d rot_mat;
  rot_mat.setIdentity(3, 3);
  rot_mat = CalcBodyWorldOrientation(
      model, q,
      body_id);  // NOTE: Rot matrix in rbdl is transposed! i.e. it represents
                 // the rotation of a body in body coordinates

  Math::SpatialTransform adjoint_transformation(
      rot_mat.transpose(), Vector3d::Zero());  // adjoint transformation of the
                                               // body in base coordinates

  return adjoint_transformation;
}

VectorNd calc_dynamicmodel_inverse(Model &model,
                                   ExplicitLoopConstraintSet &elcs,
                                   const Math::VectorNd &y,
                                   const Math::VectorNd &yd,
                                   const Math::VectorNd &ydd) {
  // This function calculates the actuator forces (or torques) depending on
  // the state of independent joint positions, velocities and accelerations

  if (model.dof_count == elcs.get_dof_spanningtree()) {
    // To be transferred to closed loop mechanism model
    MatrixXd Gu;  // Gu_inv;
    Gu.setZero(elcs.get_dof_active(), elcs.get_dof_active());
    // Gu_inv.setZero(elcs.get_dof_active(), elcs.get_dof_active());

    VectorXd Tau_spanningtree(model.dof_count);
    VectorXd Tau_actuated(elcs.get_dof_active());

    VectorXd q(model.dof_count);
    calc_sysstate_q(model, elcs, y, q);

    MatrixXd G(elcs.get_dof_spanningtree(), elcs.get_dof_independent());
    G = elcs.calc_loopclosure_Jacobian(y);

    VectorXd qd(model.dof_count);
    qd = G * yd;

    VectorNd g(elcs.get_dof_spanningtree());
    g = elcs.calc_loopclosure_g(y, yd);

    VectorXd qdd(model.dof_count);
    qdd = G * ydd + g;

    InverseDynamics(model, q, qd, qdd, Tau_spanningtree);
    /*
    Gu = elcs.get_permutation_matrix() * G;		// old code: works for
fixed base
    // To solve a system of equations, Ax = b, its better to use x =
A.colPivHouseholderQr().solve(b) instead of computing an explicit inverse
//		Gu_inv = Gu.inverse();
//		Tau_actuated = Gu_inv.transpose() * G.transpose() *
Tau_spanningtree; Tau_actuated =
(Gu.transpose()).colPivHouseholderQr().solve(G.transpose() * Tau_spanningtree);
// old code: works for fixed base
    */
    // Gu matrix = Q1 * G * Q2^T
    // Solving equation: Gu^T * Tau_actuator = Q2 * G^T * Tau_spanningtree
    // To solve a system of equations, Ax = b, its better to use x =
    // A.colPivHouseholderQr().solve(b) instead of computing an explicit inverse
    Gu = elcs.get_permutation_matrix() * G *
         elcs.get_permutation_matrix2().transpose();

    Tau_actuated = (Gu.transpose())
                       .colPivHouseholderQr()
                       .solve(elcs.get_permutation_matrix2() * G.transpose() *
                              Tau_spanningtree);

    return Tau_actuated;

  } else {
    cerr << "Model and analytical constraint sets are not consistent in "
            "dof_spanningtree size."
         << endl;
    cerr << "Dof count: " << model.dof_count
         << "\nSpanning tree: " << elcs.get_dof_spanningtree() << endl;
    abort();
  }
}

VectorNd calc_dynamicmodel_inverse_including_floatingbase(
    Model &model, ExplicitLoopConstraintSet &elcs, const Math::VectorNd &y,
    const Math::VectorNd &yd, const Math::VectorNd &ydd) {
  // This function calculates the actuator forces (or torques) depending on
  // the state of independent joint positions, velocities and accelerations

  if (model.dof_count == elcs.get_dof_spanningtree()) {
    // To be transferred to closed loop mechanism model
    MatrixXd Gu;  // Gu_inv;
    Gu.setZero(elcs.get_dof_active(), elcs.get_dof_active());

    VectorXd Tau_spanningtree(model.dof_count);
    VectorXd Tau_actuated(elcs.get_dof_floatingbase() + elcs.get_dof_active());

    VectorXd q(model.dof_count);
    calc_sysstate_q(model, elcs, y, q);

    MatrixXd G(elcs.get_dof_spanningtree(), elcs.get_dof_independent());
    G = elcs.calc_loopclosure_Jacobian(y);

    VectorXd qd(model.dof_count);
    qd = G * yd;

    VectorXd qdd(model.dof_count);

    VectorNd g(elcs.get_dof_spanningtree());
    g = elcs.calc_loopclosure_g(y, yd);

    qdd = G * ydd + g;

    InverseDynamics(model, q, qd, qdd, Tau_spanningtree);

    // Gu matrix = Q1 * G * Q2^T
    // Solving equation: Gu^T * Tau_actuator = Q2 * G^T * Tau_spanningtree
    // To solve a system of equations, Ax = b, its better to use x =
    // A.colPivHouseholderQr().solve(b) instead of computing an explicit inverse
    Gu = elcs.get_permutation_matrix() * G *
         elcs.get_permutation_matrix2().transpose();

    Tau_actuated.head(elcs.get_dof_floatingbase()) =
        Tau_spanningtree.head(elcs.get_dof_floatingbase());
    Tau_actuated.tail(elcs.get_dof_active()) =
        (Gu.transpose())
            .colPivHouseholderQr()
            .solve(elcs.get_permutation_matrix2() * G.transpose() *
                   Tau_spanningtree);

    return Tau_actuated;

  } else {
    cerr << "Model and analytical constraint sets are not consistent in "
            "dof_spanningtree size."
         << endl;
    abort();
  }
}

VectorNd calc_dynamicmodel_inverse_independentjointspace(
    Model &model, ExplicitLoopConstraintSet &elcs, const Math::VectorNd &y,
    const Math::VectorNd &yd, const Math::VectorNd &ydd) {
  // This function calculates the actuator forces (or torques) depending on
  // the state of active joint positions, velocities and accelerations

  if (model.dof_count == elcs.get_dof_spanningtree()) {
    // To be transferred to closed loop mechanism model
    MatrixXd Gu;  // Gu_inv;
    Gu.setZero(elcs.get_dof_active(), elcs.get_dof_active());
    // Gu_inv.setZero(elcs.get_dof_active(), elcs.get_dof_active());

    VectorXd Tau_spanningtree(model.dof_count);
    VectorXd Tau_independentjointspace(elcs.get_dof_active());

    VectorXd q(model.dof_count);
    calc_sysstate_q(model, elcs, y, q);

    MatrixXd G(elcs.get_dof_spanningtree(), elcs.get_dof_independent());
    G = elcs.calc_loopclosure_Jacobian(y);

    VectorXd qd(model.dof_count);
    qd = G * yd;

    VectorXd qdd(model.dof_count);
    /* MatrixXd Gdot(elcs.get_dof_spanningtree(), elcs.get_dof_independent());
    Gdot = elcs.calc_loopclosure_Jacobiand(y, yd); */

    VectorNd g(elcs.get_dof_spanningtree());
    g = elcs.calc_loopclosure_g(y, yd);
    qdd = G * ydd + g;

    InverseDynamics(model, q, qd, qdd, Tau_spanningtree);

    Tau_independentjointspace = G.transpose() * Tau_spanningtree;

    return Tau_independentjointspace;

  } else {
    cerr << "Model and analytical constraint sets are not consistent in "
            "dof_spanningtree size."
         << endl;
    abort();
  }
}

VectorNd calc_dynamicmodel_forward(Model &model,
                                   ExplicitLoopConstraintSet &elcs,
                                   const Math::VectorNd &y,
                                   const Math::VectorNd &yd,
                                   const VectorXd &Tau_actuated) {
  if (elcs.get_dof_independent() != elcs.get_dof_independent_robot()) {
    cerr << "Independent dof defined in urdf: " << elcs.get_dof_independent()
         << ", Independent dof belonging to the robot: "
         << elcs.get_dof_independent_robot() << endl;
    cerr << "This function is not supported for floating base robots! Please "
            "note that in these cases actuator state does not define "
            "independent joint state as the system is underactuated."
         << endl;
    abort();
  }

  if (model.dof_count == elcs.get_dof_spanningtree()) {
    VectorXd ydd(elcs.get_dof_independent());

    if (elcs.get_dof_spanningtree() == elcs.get_dof_independent() &&
        elcs.get_dof_spanningtree() == elcs.get_dof_active()) {
      // The robot is a serial or tree type mechanism (n = p = m). Hence, we
      // will exploit O(n) articulated body algorithm from Featherstone.
      ForwardDynamics(model, y, yd, Tau_actuated,
                      ydd);  // stores the result in ydd
    } else {
      // NOTE: Be careful that MatrixXd variables like H matrix are set to zero
      // before used.
      MatrixXd H;  // mass matrix
      H.setZero(model.dof_count, model.dof_count);
      VectorXd C(model.dof_count);  // coriolis and gravity terms

      MatrixXd Gu, Gu_inv;
      Gu.setZero(elcs.get_dof_active(), elcs.get_dof_active());
      Gu_inv.setZero(elcs.get_dof_active(), elcs.get_dof_active());

      VectorXd q(model.dof_count);
      calc_sysstate_q(model, elcs, y, q);
      // std::cout<< "Q: \n" << q.transpose() << std::endl;
      VectorXd qd(model.dof_count);
      calc_sysstate_qdot(model, elcs, y, yd, qd);

      MatrixXd G(elcs.get_dof_spanningtree(), elcs.get_dof_active());
      G = elcs.calc_loopclosure_Jacobian(y);
      Gu = elcs.get_permutation_matrix() * G;

      /* VectorXd g(elcs.get_dof_spanningtree());
      g = elcs.calc_loopclosure_Jacobiand(y, yd) * yd; */

      VectorNd g(elcs.get_dof_spanningtree());
      g = elcs.calc_loopclosure_g(y, yd);

      // Compute the bias force
      NonlinearEffects(model, q, qd, C);
      // Compute the mass-inertia matrix with CRBA in O(n) time
      CompositeRigidBodyAlgorithm(model, q, H);
      /*
      // SVD of the mass-inertia matrix
      Eigen::JacobiSVD<MatrixXd> svd(H);
      cout << "Mass-inertia matrix, H(q):" << endl << H << endl;
      cout << "Condition number of H(q): " << svd.singularValues()(0) /
      svd.singularValues()(svd.singularValues().size()-1)<<endl; cout << "Rank
      of H(q): " << svd.rank()<<endl; cout << "Its singular values are:" << endl
      << svd.singularValues() << endl; cout << "Its left singular vectors are
      the columns of the thin U matrix:" << endl << svd.matrixU() << endl; cout
      << "Its right singular vectors are the columns of the thin V matrix:" <<
      endl << svd.matrixV() << endl;

      // Eigen value decomposition of H
      Eigen::SelfAdjointEigenSolver<MatrixXd> eigensolver(H);
      if (eigensolver.info() != Eigen::Success) abort();
      cout << "The eigenvalues of H are:\n" << eigensolver.eigenvalues() <<
      endl; cout << "Here's a matrix whose columns are eigenvectors of H \n"
              << "corresponding to these eigenvalues:\n"
              << eigensolver.eigenvectors() << endl;

      // SVD of the mass-inertia matrix
      Eigen::JacobiSVD<MatrixXd> svdn(G.transpose()*H*G);
      cout << "Mass-inertia matrix, H(y):" << endl << G.transpose()*H*G << endl;
      cout << "Condition number of H(y): " << svdn.singularValues()(0) /
      svdn.singularValues()(svdn.singularValues().size()-1)<<endl; cout << "Rank
      of H(y): " << svdn.rank()<<endl; cout << "Its singular values are:" <<
      endl << svdn.singularValues() << endl; cout << "Its left singular vectors
      are the columns of the thin U matrix:" << endl << svdn.matrixU() << endl;
      cout << "Its right singular vectors are the columns of the thin V matrix:"
      << endl << svdn.matrixV() << endl;
      */

      // Compute the independent joint accelerations
      // ydd = (G.transpose()*H*G).inverse()*(Gu.transpose()*Tau_actuated -
      // G.transpose()*C - G.transpose()*H*g);

      // ydd =
      // (G.transpose()*H*G).colPivHouseholderQr().solve(Gu.transpose()*Tau_actuated
      // - G.transpose()*C - G.transpose()*H*g); //working version
      ydd = (G.transpose() * H * G)
                .llt()
                .solve(Gu.transpose() * Tau_actuated - G.transpose() * C -
                       G.transpose() * H * g);  // LLT Decomposition
      // std::cout<< "Ydd: \n" << ydd.transpose() << std::endl;
      // ydd =
      // (G.transpose()*H*G).inverse()*(elcs.get_permutation_matrix2().transpose()*Gu.transpose()*Tau_actuated
      // - G.transpose()*C - G.transpose()*H*g);
      /*
       * test that shows that forward dynamics is not really solvable in the
      case of floating base robot when only actuator forces are known! VectorXd
      Tau_spanningtree(model.dof_count); InverseDynamics (model, q, qd, qd,
      Tau_spanningtree); cout<<"G.transpose()*Tau_spanningtree:
      "<<G.transpose()*Tau_spanningtree<<endl;
      cout<<"elcs.get_permutation_matrix2().transpose()*Gu.transpose()*Tau_actuated:
      "<<elcs.get_permutation_matrix2().transpose()*Gu.transpose()*Tau_actuated<<endl;
      ydd = (G.transpose()*H*G).inverse()*(G.transpose()*Tau_spanningtree -
      G.transpose()*C - G.transpose()*H*g);
      */
      // ydd =
      // (elcs.get_permutation_matrix2()*(G.transpose()*H*G)).inverse()*((elcs.get_permutation_matrix()
      // * G *
      // elcs.get_permutation_matrix2().transpose()).transpose()*Tau_actuated -
      // elcs.get_permutation_matrix2()*G.transpose()*C -
      // elcs.get_permutation_matrix2()*G.transpose()*H*g);
    }
    return ydd;
  } else {
    cerr << "Model and analytical constraint sets are not consistent in "
            "dof_spanningtree size."
         << endl;
    abort();
  }
}

void calc_com_properties(Model &model, ExplicitLoopConstraintSet &elcs,
                         const Math::VectorNd &y, const Math::VectorNd &yd,
                         const Math::VectorNd *ydd, double &mass,
                         Math::Vector3d &com, Math::Vector3d *com_velocity,
                         Math::Vector3d *com_acceleration,
                         Math::Vector3d *angular_momentum,
                         Math::Vector3d *change_of_angular_momentum) {
  // calculate positions(q)
  Math::VectorNd q(model.dof_count);
  calc_sysstate_q(model, elcs, y, q);

  // calculate velocities(qd)
  Math::VectorNd qd(model.dof_count);
  calc_sysstate_qdot(model, elcs, y, yd, qd);

  // calculate accelerations(qdd)
  Math::VectorNd qdd(model.dof_count);
  if (ydd != NULL &&
      (change_of_angular_momentum != NULL || com_acceleration != NULL))
    calc_sysstate_qddot(model, elcs, y, yd, *ydd, qdd);

  if (com_velocity && com_acceleration && angular_momentum &&
      change_of_angular_momentum)
    CalcCenterOfMass(model, q, qd, &qdd, mass, com, com_velocity,
                     com_acceleration, angular_momentum,
                     change_of_angular_momentum);  // compute everything
  else if (com_velocity && com_acceleration && angular_momentum)
    CalcCenterOfMass(
        model, q, qd, &qdd, mass, com, com_velocity, com_acceleration,
        angular_momentum);  // calculate mass, com, linear velocity,
                            // linear acceleration, angular momentum
  else if (com_velocity && com_acceleration)
    CalcCenterOfMass(model, q, qd, &qdd, mass, com, com_velocity,
                     com_acceleration);  // calculate mass, com, linear
                                         // velocity, linear acceleration
  else if (com_velocity && angular_momentum)
    CalcCenterOfMass(model, q, qd, &qdd, mass, com, com_velocity, NULL,
                     angular_momentum);  // calculate mass, com, linear
                                         // velocity, angular momentum
  else if (com_velocity)
    CalcCenterOfMass(model, q, qd, NULL, mass, com,
                     com_velocity);  // calculate mass, com, linear velocity
  else
    CalcCenterOfMass(
        model, q, qd, NULL, mass,
        com);  // calculate total mass and center of mass of the system only
}

void calc_zero_moment_point(Model &model, ExplicitLoopConstraintSet &elcs,
                            const Math::VectorNd &y, const Math::VectorNd &yd,
                            const Math::VectorNd &ydd,
                            const Math::Vector3d &surface_normal,
                            const Math::Vector3d &surface_point,
                            Math::Vector3d *zmp) {
  // calculate positions(q)
  Math::VectorNd q(model.dof_count);
  calc_sysstate_q(model, elcs, y, q);

  // calculate velocities(qd)
  Math::VectorNd qd(model.dof_count);
  calc_sysstate_qdot(model, elcs, y, yd, qd);

  // calculate accelerations(qdd)
  Math::VectorNd qdd(model.dof_count);
  calc_sysstate_qddot(model, elcs, y, yd, ydd, qdd);

  // Compute the zero moment point
  CalcZeroMomentPoint(model, q, qd, qdd, zmp, surface_normal, surface_point);
  // cout<<"ZMP: "<<*zmp<<endl;
}
VectorNd calc_staticmodel_inverse(Model &model, ExplicitLoopConstraintSet &elcs,
                                  const Math::VectorNd &y,
                                  const std::vector<string> wrench_points,
                                  const std::vector<Math::SpatialVector> f_ext,
                                  const std::vector<bool> wrench_resolution,
                                  const std::vector<bool> wrench_interaction) {
  if (wrench_points.size() != f_ext.size() ||
      wrench_resolution.size() != f_ext.size() ||
      wrench_points.size() != wrench_resolution.size() ||
      wrench_points.size() != wrench_interaction.size()) {
    cerr << "wrench_points size: " << wrench_points.size()
         << ", f_ext size: " << f_ext.size()
         << ", wrench_resolution size: " << wrench_resolution.size()
         << ", wrench_interaction size: " << wrench_interaction.size() << endl;
    cerr << "Size mismatch!" << endl;
    abort();
  } else {
    VectorNd Tau_actuated_ext(VectorNd::Zero(elcs.get_dof_active()));
    VectorNd wrench_independentjointspace(
        VectorNd::Zero(elcs.get_dof_independent()));
    VectorNd wrench_independentjointspace_robot(
        VectorNd::Zero(elcs.get_dof_independent_robot()));
    MatrixXd wrench_point_Jacobian;
    Math::Vector3d point_position;
    point_position.setZero();
    VectorNd q(VectorNd::Zero(model.dof_count));
    calc_sysstate_q(model, elcs, y,
                    q);  // compute full position state of the spanning tree
    MatrixXd Gu;
    Gu.setZero(elcs.get_dof_active(), elcs.get_dof_active());
    MatrixXd G(elcs.get_dof_spanningtree(), elcs.get_dof_active());
    G = elcs.calc_loopclosure_Jacobian(y);  // compute loop closure jacobian
    Gu = elcs.get_permutation_matrix() * G *
         elcs.get_permutation_matrix2()
             .transpose();  // compute actuator jacobian

    for (unsigned int i = 0; i < wrench_points.size(); i++) {
      unsigned int body_id = model.GetBodyId(
          wrench_points[i]
              .c_str());  // find body_id of the force application points

      if (model.IsBodyId(body_id)) {
        wrench_point_Jacobian.setZero(
            6, model.dof_count);  // set this matrix every time to zero
        if (wrench_resolution[i])
          CalcBodySpatialJacobian(
              model, q, body_id,
              wrench_point_Jacobian);  // compute jacobian (6 x n) in body
                                       // coordinate system
        else
          CalcPointJacobian6D(
              model, q, body_id, point_position,
              wrench_point_Jacobian);  // compute point jacobian 6D for this
                                       // point (6 x n) in base coordinate
                                       // system

        wrench_independentjointspace =
            (wrench_point_Jacobian * G).transpose() * f_ext[i];

        wrench_independentjointspace_robot =
            elcs.get_permutation_matrix2() * wrench_independentjointspace;
        if (wrench_interaction[i])
          Tau_actuated_ext =
              Tau_actuated_ext - (Gu.transpose())
                                     .colPivHouseholderQr()
                                     .solve(wrench_independentjointspace_robot);
        else
          Tau_actuated_ext =
              Tau_actuated_ext + (Gu.transpose())
                                     .colPivHouseholderQr()
                                     .solve(wrench_independentjointspace_robot);
        // std::cout << "i = "<< i << " Tau actuated external for system: " <<
        // Tau_actuated_ext.transpose() << std::endl;
      } else {
        cerr << "calc_staticmodel_inverse: Body Name provided is not valid."
             << endl;
        abort();
      }
    }

    return Tau_actuated_ext;
  }
}

SpatialVector calc_staticmodel_forward(
    Model &model, ExplicitLoopConstraintSet &elcs, const Math::VectorNd &y,
    const Math::VectorNd &Tau_actuated_measured, const string FTsensor_link,
    const bool wrench_resolution) {
  if (elcs.get_dof_independent() != elcs.get_dof_independent_robot()) {
    cerr << "Independent dof defined in urdf: " << elcs.get_dof_independent()
         << ", Independent dof belonging to the robot: "
         << elcs.get_dof_independent_robot() << endl;
    cerr << "This function is not supported for floating base robots!" << endl;
    abort();
  }

  if (elcs.get_dof_independent_robot() != 6) {
    cerr << "Independent dof belonging to the robot: "
         << elcs.get_dof_independent_robot() << endl;
    cerr << "This function is not supported for robots with mobility less than "
            "or greater than 6! Please note that in these cases the measured "
            "actuator forces do not define 6 dof wrench vector."
         << endl;
    abort();
  }

  SpatialVector wrench;

  VectorNd actforce_independentjointspace_robot(
      VectorNd::Zero(elcs.get_dof_independent_robot()));

  MatrixXd wrench_point_Jacobian;  // (6 x n) Jacobian matrix
  Math::Vector3d point_position;
  point_position.setZero();

  VectorNd q(VectorNd::Zero(model.dof_count));
  calc_sysstate_q(model, elcs, y,
                  q);  // compute full position state of the spanning tree

  MatrixXd G(elcs.get_dof_spanningtree(), elcs.get_dof_active());
  G = elcs.calc_loopclosure_Jacobian(y);  // compute loop closure jacobian

  MatrixXd Gu;
  Gu.setZero(elcs.get_dof_active(), elcs.get_dof_active());
  Gu = elcs.get_permutation_matrix() * G *
       elcs.get_permutation_matrix2().transpose();  // compute actuator jacobian
  actforce_independentjointspace_robot =
      Gu.transpose() *
      Tau_actuated_measured;  // force in independent joint space

  unsigned int body_id = model.GetBodyId(
      FTsensor_link.c_str());  // find body_id of the force application points
  if (model.IsBodyId(body_id)) {
    wrench_point_Jacobian.setZero(
        6, model.dof_count);  // set this matrix every time to zero
    if (wrench_resolution)
      CalcBodySpatialJacobian(
          model, q, body_id,
          wrench_point_Jacobian);  // compute jacobian (6 x n) in body
                                   // coordinate system
    else
      CalcPointJacobian6D(
          model, q, body_id, point_position,
          wrench_point_Jacobian);  // compute point jacobian 6D for this point
                                   // (6 x n) in base coordinate system

    // wrench_point_Jacobian*G is square and invertible only for 6 dof robots.
    wrench = ((wrench_point_Jacobian * G).transpose())
                 .colPivHouseholderQr()
                 .solve(actforce_independentjointspace_robot);
    return wrench;
  } else {
    cerr << "calc_staticmodel_forward: Body Name provided is not valid."
         << endl;
    abort();
  }
}

double calc_kinetic_energy(RigidBodyDynamics::Model &model, Eigen::VectorXd q,
                           Eigen::VectorXd qdot) {
  // Compute mass matrix
  // Eigen::MatrixXd M = Eigen::MatrixXd::Zero(model.dof_count,
  // model.dof_count); RigidBodyDynamics::CompositeRigidBodyAlgorithm(model, q,
  // M, true);

  double kin_energy =
      RigidBodyDynamics::Utils::CalcKineticEnergy(model, q, qdot, true);
  // Kinetic energy = 0.5 * qdot^T * M * qdot
  // return 0.5 * qdot.transpose() * M * qdot;
  return kin_energy;
}

double calc_potential_energy(RigidBodyDynamics::Model &model,
                             Eigen::VectorXd q) {
  double V = RigidBodyDynamics::Utils::CalcPotentialEnergy(model, q, true);
  // Eigen::Vector3d g = model.gravity; // typically [0, 0, -9.81]

  // for (unsigned int i = 1; i < model.mBodies.size(); i++) {
  //   double m = model.mBodies[i].mMass;
  //   RigidBodyDynamics::Math::Vector3d com =
  //       RigidBodyDynamics::CalcBodyToBaseCoordinates(
  //           model, q, i, model.mBodies[i].mCenterOfMass, true);
  //   V += -m * g.dot(com); // negative because gravity points down
  // }
  return V;
}

double calc_total_energy(RigidBodyDynamics::Model &model, Eigen::VectorXd q,
                         Eigen::VectorXd qdot) {
  return calc_kinetic_energy(model, q, qdot) + calc_potential_energy(model, q);
}

}  // namespace HyRoDyn
