/*
 * HyRoDyn - Hybrid Robot Dynamics
 * Copyright (c) 2018-2020 Shivesh Kumar <shivesh.kumar@dfki.de>
 * This submechanism library is contributed by Ibrahim Tijjani
 * <ibrahim.tijjani@dfki.de>. Licensed under the zlib license. See LICENSE for
 * more details.
 */

#include "R3US2_ANKLE.hpp"

namespace R3US2_ANKLE {
Eigen::Matrix4d BuildTransMat(const Math::Matrix3d rot,
                              const Math::Vector3d trans) {
  Eigen::Matrix4d TransMat = Eigen::Matrix4d::Zero();

  TransMat.block(0, 0, 3, 3) = rot;
  TransMat(0, 3) = trans(0);
  TransMat(1, 3) = trans(1);
  TransMat(2, 3) = trans(2);
  TransMat(3, 3) = 1;

  return TransMat;
  // cout<<"Transformation matrix:"<<TransMat<<endl;
}

Eigen::Vector4d BuildTransVec(const Math::Vector3d pos) {
  Eigen::Vector4d TransVec = Eigen::Vector4d::Zero();

  TransVec(0) = pos(0);
  TransVec(1) = pos(1);
  TransVec(2) = pos(2);
  TransVec(3) = 1;

  return TransVec;
  // cout<<"Translation Vector:"<<TransVec<<endl;
}

Eigen::Matrix4d invertTransMat(const Eigen::Matrix4d TransMat) {
  Eigen::Matrix4d invTransMat = Eigen::Matrix4d::Zero();
  Eigen::Matrix3d rot = TransMat.block(0, 0, 3, 3);
  Eigen::Vector3d pos = TransMat.col(3).head(3);

  invTransMat.block(0, 0, 3, 3) = rot.transpose();
  invTransMat.block(0, 3, 3, 1) = -rot.transpose() * pos;
  invTransMat(3, 3) = 1;

  return invTransMat;
  // cout<<"Inverse Transformation matrix:"<<invTransMat<<endl;
}

// Constructor
r3us2_ankle::r3us2_ankle(string file_path,
                         std::vector<string> jointnames_spanningtree,
                         std::vector<string> jointnames_independent) {

  // Model m;

  const char *ext;
  ext = strrchr(file_path.c_str(), '.');
  if (!strcmp(ext, ".urdf") || !strcmp(ext, ".URDF") ||
      !strcmp(ext, ".robot")) {
    if (!Addons::URDFReadFromFileWithModularity(
            file_path.c_str(), &m, jointnames_spanningtree, false)) {
      std::cerr << "Error loading urdf model" << std::endl;
      abort();
    }
  } else {
    std::cerr << "Unknown file type: Accepted file types are .urdf or .lua"
              << endl;
    abort();
  }

  assert(dof_independent == jointnames_independent.size());
  assert(dof_spanningtree == m.dof_count);

  // Put the mechanism in its zero configuration
  VectorNd Q(VectorNd::Zero(m.dof_count));
  UpdateKinematicsCustom(m, &Q, NULL, NULL);

  dof_active = 3;
  dof_spanningtree = m.dof_count;

  // Extract the physical parameters of the mechanism from the RBDL model

  // calculate G_T_E (position and orientation) w.r.t global frame
  G_T_E_zero.setIdentity(4, 4);

  // calculate body coordinates of the ee points at the fixed ee frame
  e1_ee = CalcBodyToBaseCoordinates(m, Q, 3, Vector3d(0., -0.035, 0.), false);
  e2_ee = CalcBodyToBaseCoordinates(m, Q, 3, Vector3d(0., 0.035, 0.), false);
  e3_ee = CalcBodyToBaseCoordinates(m, Q, 3, Vector3d(0., 0., -0.035), false);
  e4_ee = CalcBodyToBaseCoordinates(m, Q, 3, Vector3d(0., 0., 0.035), false);
  e5_ee = CalcBodyToBaseCoordinates(m, Q, 3, Vector3d(-0.035, 0., 0.), false);
  e6_ee = CalcBodyToBaseCoordinates(m, Q, 3, Vector3d(0.035, 0., 0.), false);
  cout << "e1_ee " << e1_ee.transpose() << endl;
  cout << "e2_ee " << e2_ee.transpose() << endl;
  cout << "e3_ee " << e3_ee.transpose() << endl;
  cout << "e4_ee " << e4_ee.transpose() << endl;
  cout << "e5_ee " << e5_ee.transpose() << endl;
  cout << "e6_ee " << e6_ee.transpose() << endl;

  // calculate body coordinates from motor 1 to motor 3
  Vector3d motor_1, motor_2, motor_3;
  Vector3d lever_1_roll, lever_2_roll, lever_3_roll, lever_4_roll, lever_5_roll,
      lever_6_roll;
  Vector3d c1, c2, c3, c4, c5, c6;

  motor_1 = CalcBodyToBaseCoordinates(m, Q, 7, Vector3d(0., 0., 0.), false);
  lever_1_roll =
      CalcBodyToBaseCoordinates(m, Q, 8, Vector3d(0., 0., 0.), false);
  c1 = CalcBodyToBaseCoordinates(m, Q, 9, Vector3d(0., 0., 0.),
                                 false); // lever 1 pitch
  lever_2_roll =
      CalcBodyToBaseCoordinates(m, Q, 10, Vector3d(0., 0., 0.), false);
  c2 = CalcBodyToBaseCoordinates(m, Q, 11, Vector3d(0., 0., 0.),
                                 false); // lever 2 pitch
  motor_2 = CalcBodyToBaseCoordinates(m, Q, 12, Vector3d(0., 0., 0.), false);
  lever_3_roll =
      CalcBodyToBaseCoordinates(m, Q, 13, Vector3d(0., 0., 0.), false);
  c3 = CalcBodyToBaseCoordinates(m, Q, 14, Vector3d(0., 0., 0.),
                                 false); // lever 3 pitch
  lever_4_roll =
      CalcBodyToBaseCoordinates(m, Q, 15, Vector3d(0., 0., 0.), false);
  c4 = CalcBodyToBaseCoordinates(m, Q, 16, Vector3d(0., 0., 0.),
                                 false); // lever 4 pitch
  motor_3 = CalcBodyToBaseCoordinates(m, Q, 17, Vector3d(0., 0., 0.), false);
  lever_5_roll =
      CalcBodyToBaseCoordinates(m, Q, 18, Vector3d(0., 0., 0.), false);
  c5 = CalcBodyToBaseCoordinates(m, Q, 19, Vector3d(0., 0., 0.),
                                 false); // lever 5 pitch
  lever_6_roll =
      CalcBodyToBaseCoordinates(m, Q, 20, Vector3d(0., 0., 0.), false);
  c6 = CalcBodyToBaseCoordinates(m, Q, 21, Vector3d(0., 0., 0.),
                                 false); // lever 6 pitch

  Vector3d Position_EE = CalcBodyToBaseCoordinates(
      m, Q, 3, Vector3d(0., 0., 0.), false); // use body-ID 1, 2, or 3
  Matrix3d RotMat_EE = CalcBodyWorldOrientation(m, Q, 3.).transpose();
  cout << "endeff coord position :\n " << Position_EE << endl;
  cout << "endeff rotation :\n " << RotMat_EE << endl;

  // calculate transformation from the G frame to the E-frame (G_T_E)
  G_T_E_zero = BuildTransMat(RotMat_EE, Position_EE);
  cout << "Global-endeff Transformation: \n"
       << G_T_E_zero << endl; // at zero config

  // caluclate transformation from the G-frame to the taskspace-frame (G_T_T)
  Vector3d Position_T = CalcBodyToBaseCoordinates(
      m, Q, 6, Vector3d(0., 0., 0.), false); // use body-ID 4, 5, or 6
  Matrix3d RotMat_T = CalcBodyWorldOrientation(m, Q, 6.).transpose();
  cout << "Task space coord position :\n " << Position_T << endl;
  cout << "Task space rotation :\n " << RotMat_T << endl;

  G_T_T_zero = BuildTransMat(RotMat_T, Position_T);
  cout << "Global-taskspace-Transformation: \n"
       << G_T_T_zero << endl; // at zero config

  // calculate radius d of the sphere around the ee points and radius r around
  // the crank points
  r = (c1 - c2).norm() / 2;
  d = (e1_ee - e2_ee).norm() / 2;
  l = (c1 - e1_ee).norm();
  cout << "radius d:" << d << endl;
  cout << "radius r:" << r << endl;
  cout << "rod length l:" << l << endl;

  tolerance = 0.00001;
  Vector3d q, ee;
  solve_RIGM(G_T_E_zero, q, ee);
  cout << "input joint angles: \n" << q << endl;
  cout << "endeff point position: \n" << ee << endl;

  // calculate end effector orientation (fixed rotation matrices) of the ci
  // frames i.e crank points
  RotMat_c1 = CalcBodyWorldOrientation(m, Q, 9.);
  RotMat_c2 = CalcBodyWorldOrientation(m, Q, 11.);
  RotMat_c3 = CalcBodyWorldOrientation(m, Q, 14.);
  RotMat_c4 = CalcBodyWorldOrientation(m, Q, 16.);
  RotMat_c5 = CalcBodyWorldOrientation(m, Q, 19.);
  RotMat_c6 = CalcBodyWorldOrientation(m, Q, 21.);

  // cout<<"RotMat c1 "<<RotMat_c1.transpose()<<endl;

  // Comment the line below in case you derived the analytical solutions in
  // relative coordinates.
  Q_zero =
      VectorNd::Zero(m.dof_count); // zero config. vector of the spanning tree
  VectorNd y =
      VectorNd::Zero(dof_active); // zero config. vector of the active joints
  Q_zero = calc_loopclosure_function(y);
  cout << "Q_zero: " << Q_zero.transpose()
       << endl; // prints zero config. transpose of the spanning tree
}

VectorXd r3us2_ankle::calc_loopclosure_function(const Math::VectorNd &y) {
  // This function calculates the loop closure function Gamma which is a vector
  // of size (n x 1).

  VectorXd Q(dof_spanningtree);
  // VectorXd Q = VectorNd::Zero(dof_spanningtree);

  // Put your symbolically generated code here!
  // compute transformation from task space to task space_prime (T_T_Tprime) and
  // extract the rotation part
  // compute rotation matrix from the inputs: roll, pitch, yaw angles of the EE-
  // frame
  Matrix3d RotMat_x_in, RotMat_y_in, RotMat_z_in, RotMat_in;
  Eigen::Matrix4d T_T_Tprime = Eigen::Matrix4d::Zero();
  Eigen::Matrix4d G_T_Tprime = Eigen::Matrix4d::Zero();
  Eigen::Matrix4d EE_TransMat_new = Eigen::Matrix4d::Zero();

  double roll_in, pitch_in, yaw_in;
  roll_in = y(0);  // roll
  pitch_in = y(1); // pitch
  yaw_in = y(2);   // yaw

  RotMat_x_in << 1, 0, 0, 0, cos(roll_in), -sin(roll_in), 0, sin(roll_in),
      cos(roll_in);

  RotMat_y_in << cos(pitch_in), 0, sin(pitch_in), 0, 1, 0, -sin(pitch_in), 0,
      cos(pitch_in);

  RotMat_z_in << cos(yaw_in), -sin(yaw_in), 0, sin(yaw_in), cos(yaw_in), 0, 0,
      0, 1;

  RotMat_in = RotMat_x_in * RotMat_y_in * RotMat_z_in;
  // cout<< "rotation matrix :\n"<< RotMat_in<< endl;

  T_T_Tprime.block(0, 0, 3, 3) = RotMat_in;
  T_T_Tprime(0, 3) = 0;
  T_T_Tprime(1, 3) = 0;
  T_T_Tprime(2, 3) = 0;
  T_T_Tprime(3, 3) = 1;
  // cout<< "Taskspace-frame transformation matrix :\n"<<T_T_Tprime<<endl;

  // Compute the transformation from the global frame to the task space-prime
  // frame (G_T_Tprime)
  RotMat_Tprime = CalcBodyWorldOrientation(m, Q, 6.).transpose();
  Vector3d PosVec_Tprime =
      CalcBodyToBaseCoordinates(m, Q, 6, Vector3d(0., 0., 0.), false);
  // cout<<"Task space RotMat :\n"<<RotMat_Tprime<<endl;

  G_T_Tprime.block(0, 0, 3, 3) = RotMat_Tprime;
  G_T_Tprime.block(0, 3, 3, 1) = PosVec_Tprime;
  G_T_Tprime(3, 3) = 1;
  // cout<< "Global to Taskspace-prime frame transformation
  // :\n"<<G_T_Tprime<<endl;

  // caluclate transformation from the E-prime frame to the taskspace-frame
  // (Eprime_T_T) or E_T_Tprime
  Eprime_T_T = invertTransMat(G_T_E_zero) * (G_T_Tprime);
  // cout<<"endeff-prime Transformation: \n"<<Eprime_T_T<<endl;

  // compute transformation matrix from E frame to the Eprime frame (E_T_Eprime)
  Eigen::Matrix4d E_T_Eprime = invertTransMat(G_T_E_zero) *
                               (G_T_Tprime)*invertTransMat(T_T_Tprime) *
                               invertTransMat(Eprime_T_T);
  // cout<<"End eff transformation to end eff-prime:\n"<<E_T_Eprime<<endl;

  // compute joint angles q & end effector coord. position ee, from the E-frame
  // TransMat
  Vector3d q, ee;
  ee = Vector3d(0., 0., 0.);
  Eigen::Matrix4d RIGM_Mat = E_T_Eprime;
  RIGM_Mat.block(0, 3, 3, 1) = Vector3d(0., 0., 0.);
  // std::cout<<"RIGM MATRIX : \n" << RIGM_Mat <<std::endl;
  solve_RIGM(RIGM_Mat, q, ee);
  // cout<<"input joint angles :\n"<<q<<endl;
  // cout<<"ee_point position :\n"<<ee<<endl;
  // cout<<"End eff transformation to end eff-prime:\n"<<E_T_Eprime<<endl;
  MatrixXd c = MatrixXd::Zero(3, 6);
  MatrixXd e = MatrixXd::Zero(3, 6);
  VectorXd rod_lengths = VectorXd::Zero(6);

  // compute the states of the crank points ci from q
  calculate_crankpoints(q, c);
  // c << c1,c2,c3,c4,c5,c6;
  cout << "crank points: \n" << c << endl;

  // Build a new E-transformation matrix from global frame w.r.t E-frame
  EE_TransMat_new = E_T_Eprime;
  EE_TransMat_new.block(0, 3, 3, 1) = ee;
  // cout<<"new ee matrix:\n"<<EE_TransMat_new<<endl;

  // compute end effector points ei from EE_TransMat_new
  calculate_endeffectorpoints(EE_TransMat_new, e);
  // e<<e1_ee,e2_ee,e3_ee,e4_ee,e5_ee,e6_ee;
  cout << "end effector points: \n" << e << endl;

  // calculate 3x1 vectors for leg direction
  Vector3d c2e2 = e.col(0) - c.col(0);
  Vector3d c1e1 = e.col(1) - c.col(1);
  Vector3d c6e6 = e.col(2) - c.col(2);
  Vector3d c5e5 = e.col(3) - c.col(3);
  Vector3d c4e4 = e.col(4) - c.col(4);
  Vector3d c3e3 = e.col(5) - c.col(5);
  // cout<<"c1e1 before: "<<c1e1.transpose()<<endl;

  // length of line segments joining end effector points and crank points
  double l1 = (c1e1).norm();
  double l2 = (c2e2).norm();
  double l3 = (c3e3).norm();
  double l4 = (c4e4).norm();
  double l5 = (c5e5).norm();
  double l6 = (c6e6).norm();

  cout << "l1 value:" << l1 << endl;
  cout << "l2 value:" << l2 << endl;
  cout << "l3 value:" << l3 << endl;
  cout << "l4 value:" << l4 << endl;
  cout << "l5 value:" << l5 << endl;
  cout << "l6 value:" << l6 << endl;

  Q(0) = ee(0); // ee_position_x
  Q(1) = ee(1); // ee_position_y
  Q(2) = ee(2); // ee_position_z
  Q(3) = y(0);  // roll
  Q(4) = y(1);  // pitch
  Q(5) = y(2);  // yaw
  Q(6) = q(0);  // motor 1
  Q(11) = q(1); // motor 2
  Q(16) = q(2); // motor 3
  // cout<<"q : "<<-q.transpose()<<endl;

  // calculate end effector orientation (fixed rotation matrices) of the ci
  // frames i.e crank points
  RotMat_c1 = CalcBodyWorldOrientation(m, Q, 9.);
  RotMat_c2 = CalcBodyWorldOrientation(m, Q, 11.);
  RotMat_c3 = CalcBodyWorldOrientation(m, Q, 14.);
  RotMat_c4 = CalcBodyWorldOrientation(m, Q, 16.);
  RotMat_c5 = CalcBodyWorldOrientation(m, Q, 19.);
  RotMat_c6 = CalcBodyWorldOrientation(m, Q, 21.);

  Matrix3d RotMat_c1n = RotMat_c1 * G_T_E_zero.block(0, 0, 3, 3);
  Matrix3d RotMat_c2n = RotMat_c2 * G_T_E_zero.block(0, 0, 3, 3);
  Matrix3d RotMat_c3n = RotMat_c3 * G_T_E_zero.block(0, 0, 3, 3);
  Matrix3d RotMat_c4n = RotMat_c4 * G_T_E_zero.block(0, 0, 3, 3);
  Matrix3d RotMat_c5n = RotMat_c5 * G_T_E_zero.block(0, 0, 3, 3);
  Matrix3d RotMat_c6n = RotMat_c6 * G_T_E_zero.block(0, 0, 3, 3);

  // transform the rodlength vector to spherical joint frame
  c1e1 = RotMat_c1n * c1e1;
  c2e2 = RotMat_c2n * c2e2;
  c3e3 = RotMat_c3n * c3e3;
  c4e4 = RotMat_c4n * c4e4;
  c5e5 = RotMat_c5n * c5e5;
  c6e6 = RotMat_c6n * c6e6;

  // leg parameters(roll_u, pitch_u), extract rotation angles
  double roll_u1 = atan2(c1e1(1), c1e1(2));
  double pitch_u1 = acos(c1e1(0) / l1);

  double roll_u2 = atan2(c2e2(1), c2e2(2));
  double pitch_u2 = acos(c2e2(0) / l2);

  double roll_u3 = atan2(c3e3(1), c3e3(2));
  double pitch_u3 = acos(c3e3(0) / l3);

  double roll_u4 = atan2(c4e4(1), c4e4(2));
  double pitch_u4 = acos(c4e4(0) / l4);

  double roll_u5 = atan2(c5e5(1), c5e5(2));
  double pitch_u5 = acos(c5e5(0) / l5);

  double roll_u6 = atan2(c6e6(1), c6e6(2));
  double pitch_u6 = acos(c6e6(0) / l6);
  // cout<<"pitch 6 "<<pitch_u6<<endl;

  Q(7) = roll_u1;  // lever 1 roll
  Q(8) = pitch_u1; // lever 1 pitch

  Q(9) = roll_u2;   // lever 2 roll
  Q(10) = pitch_u2; // lever 2 pitch

  Q(12) = roll_u3;  // lever 3 roll
  Q(13) = pitch_u3; // lever 3 pitch
  Q(14) = roll_u4;  // lever 4 roll
  Q(15) = pitch_u4; // lever 4 pitch

  Q(17) = roll_u5;  // lever 5 roll
  Q(18) = pitch_u5; // lever 5 pitch
  Q(19) = roll_u6;  // lever 6 roll
  Q(20) = pitch_u6; // lever 6 pitch

  // Comment the line below in case you derived the analytical solutions in
  // relative coordinates.
  Q = Q - Q_zero;
  cout << "Q wrt zero position: " << Q.transpose() << endl;

  return Q;
}

MatrixXd r3us2_ankle::calc_loopclosure_Jacobian(const Math::VectorNd &y) {
  // This function calculates the loop closure Jacobian matrix G which is a
  // matrix of size (n x m).

  MatrixXd G;
  G.setZero(dof_spanningtree, dof_independent);

  // Put your symbolically generated code here!

  double qx = y(0);
  double qy = y(1);
  double qz = y(2);

  double t2 = cos(qx);
  double t3 = cos(qy);
  double t4 = cos(qz);
  double t5 = sin(qx);
  double t6 = sin(qy);
  double t7 = sin(qz);
  double t8 = G_T_Tprime(0, 3) * d;
  double t9 = G_T_Tprime(1, 3) * d;
  double t10 = G_T_Tprime(2, 3) * d;
  double t11 = d * l;
  double t12 = -t11;
  double t13 = t8 + t12;
  double t14 = t9 + t12;
  double t15 = t10 + t12;

  G(0, 0) = G_T_Tprime(0, 1) * d;

  G(0, 1) = G_T_Tprime(1, 1) * d - r * cos(qx);
  G(0, 2) = G_T_Tprime(2, 1) * d - r * sin(qx);
  // cout<<"I am here" <<endl;
  G(1, 0) = G_T_Tprime(1, 1) * t15 - G_T_Tprime(2, 1) * t9;
  // cout<<"I am here werr" <<endl;
  G(1, 1) = -G_T_Tprime(0, 1) * t15 + G_T_Tprime(2, 1) * t8;
  G(1, 2) = G_T_Tprime(0, 1) * t9 - G_T_Tprime(1, 1) * t8;
  G(2, 0) = -G_T_Tprime(1, 3) * r * t5 + G_T_Tprime(2, 3) * r * t2 - l * r * t2;
  G(2, 1) = 0;
  G(2, 2) = 0;
  G(3, 0) = G_T_Tprime(0, 2) * d - r * sin(qz);
  G(3, 1) = G_T_Tprime(1, 2) * d;
  G(3, 2) = G_T_Tprime(2, 2) * d - r * cos(qz);
  G(4, 0) = G_T_Tprime(1, 2) * t10 - G_T_Tprime(2, 2) * t9;
  G(4, 1) = -G_T_Tprime(0, 2) * t10 + G_T_Tprime(2, 2) * t13;
  G(4, 2) = G_T_Tprime(0, 2) * t9 - G_T_Tprime(1, 2) * t13;
  G(5, 0) = 0;
  G(5, 1) = G_T_Tprime(0, 3) * r * t4 - G_T_Tprime(2, 3) * r * t7 - l * r * t4;
  G(5, 2) = 0;
  G(6, 0) = G_T_Tprime(0, 0) * d - r * cos(qy);
  G(6, 1) = G_T_Tprime(1, 0) * d - r * sin(qy);
  G(6, 2) = G_T_Tprime(2, 0) * d;
  G(7, 0) = G_T_Tprime(1, 0) * t10 - G_T_Tprime(2, 0) * t14;
  G(7, 1) = -G_T_Tprime(0, 0) * t10 + G_T_Tprime(2, 0) * t8;
  G(7, 2) = G_T_Tprime(0, 0) * t14 - G_T_Tprime(1, 0) * t8;
  G(8, 0) = 0;
  G(8, 1) = 0;
  G(8, 2) = -G_T_Tprime(0, 3) * r * t6 + G_T_Tprime(1, 3) * r * t3 - l * r * t3;

  cout << "Loop closure jacobian, G: \n" << G << endl;

  return G;
}

MatrixXd r3us2_ankle::calc_loopclosure_Jacobiand(const Math::VectorNd &y,
                                                 const Math::VectorNd &ydot) {
  // This function calculates the 1st order time derivative of loop closure
  // Jacobian Gdot which is a matrix of size (n x m).

  MatrixXd Gdot(dof_spanningtree, dof_independent);
  Gdot.setZero(dof_spanningtree, dof_independent);

  // Put your symbolically generated code here!
  double qx = y(0);
  double qy = y(1);
  double qz = y(2);

  double t2 = cos(qx);
  double t3 = cos(qy);
  double t4 = cos(qz);
  double t5 = sin(qx);
  double t6 = sin(qy);
  double t7 = sin(qz);

  Gdot(0, 0) = 0;
  Gdot(0, 1) = -t2;
  Gdot(0, 2) = -t5;
  Gdot(1, 0) = 0;
  Gdot(1, 1) = 0;
  Gdot(1, 2) = 0;
  Gdot(2, 0) = G_T_Tprime(2, 3) * t2 - G_T_Tprime(1, 3) * t5 - l * t2;
  Gdot(2, 1) = 0;
  Gdot(2, 2) = 0;
  Gdot(3, 0) = -t7;
  Gdot(3, 1) = 0;
  Gdot(3, 2) = -t4;
  Gdot(4, 0) = 0;
  Gdot(4, 1) = 0;
  Gdot(4, 2) = 0;
  Gdot(5, 0) = 0;
  Gdot(5, 1) = G_T_Tprime(0, 3) * t4 - G_T_Tprime(2, 3) * t7 - l * t4;
  Gdot(5, 2) = 0;
  Gdot(6, 0) = -t3;
  Gdot(6, 1) = -t6;
  Gdot(6, 2) = 0;
  Gdot(7, 0) = 0;
  Gdot(7, 1) = 0;
  Gdot(7, 2) = 0;
  Gdot(8, 0) = 0;
  Gdot(8, 1) = 0;
  Gdot(8, 2) = G_T_Tprime(1, 3) * t3 - G_T_Tprime(0, 3) * t6 - l * t3;

  cout << "Loop closure jacobiand, Gdot: \n" << Gdot << endl;

  return Gdot;
}

VectorXd r3us2_ankle::calc_loopclosure_g(const Math::VectorNd &y,
                                         const Math::VectorNd &ydot) {
  // This function calculates the loop closure bias acceleration g= Gdot*ydot

  return calc_loopclosure_Jacobiand(y, ydot) * ydot;
}
double r3us2_ankle::wrap2pi(double x) {
  // This function wrap the x values into the interval (-pi, pi]
  // wrapx = x + n pi with n integer such that wrapx \in (-pi, pi]

  double wrapx;
  wrapx = atan2(sin(x), cos(x));
  //    cout<<"wrapx = "<<wrapx<<endl;
  return wrapx;
}

double r3us2_ankle::choose_soln(double x, double y) {
  double soln;
  if (x <= PI / 2 && x >= -PI / 2)
    soln = x;
  else if (y <= PI / 2 && y >= -PI / 2)
    soln = y;
  else
    cout << "No feasible solution" << x << y << endl;
  return soln;
}

Vector3d r3us2_ankle::IGM(MatrixXd G_T_E) {
  Vector3d q;

  double qx, qy, qz; // Input joint angles
  double qx_1, qy_1, qz_1, qx_2, qy_2,
      qz_2;          // 2 solutions of Input joint angles
  double Ex, Ey, Ez; // End effector cooridnates w.r.t G (global frame)
  double sx, sy, sz; // dcm 1st column (unit vector along Ex axis)
  double nx, ny, nz; // dcm 2nd column (unit vector along Ey axis)
  double ax, ay, az; // dcm 3rd column (unit vector along Ez axis)

  sx = G_T_E(0, 0);
  sy = G_T_E(1, 0);
  sz = G_T_E(2, 0);
  nx = G_T_E(0, 1);
  ny = G_T_E(1, 1);
  nz = G_T_E(2, 1);
  ax = G_T_E(0, 2);
  ay = G_T_E(1, 2);
  az = G_T_E(2, 2);
  Ex = G_T_E(0, 3);
  Ey = G_T_E(1, 3);
  Ez = G_T_E(2, 3);

  // Equations of IGM
  qx_1 =
      2 * atan2(-r * (Ez - l) +
                    sqrt(pow(r * Ey, 2) + pow(r * (Ez - l), 2) -
                         pow(d * (l * nz - (Ex * nx + Ey * ny + Ez * nz)), 2)),
                d * (l * nz - (Ex * nx + Ey * ny + Ez * nz)) - r * Ey);
  qx_2 =
      2 * atan2(-r * (Ez - l) -
                    sqrt(pow(r * Ey, 2) + pow(r * (Ez - l), 2) -
                         pow(d * (l * nz - (Ex * nx + Ey * ny + Ez * nz)), 2)),
                d * (l * nz - (Ex * nx + Ey * ny + Ez * nz)) - r * Ey);
  qx_1 = wrap2pi(qx_1);
  qx_2 = wrap2pi(qx_2);

  qy_1 =
      2 * atan2(-r * (Ex - l) +
                    sqrt(pow(r * Ez, 2) + pow(r * (Ex - l), 2) -
                         pow(d * (l * ax - (Ex * ax + Ey * ay + Ez * az)), 2)),
                d * (l * ax - (Ex * ax + Ey * ay + Ez * az)) - r * Ez);
  qy_2 =
      2 * atan2(-r * (Ex - l) -
                    sqrt(pow(r * Ez, 2) + pow(r * (Ex - l), 2) -
                         pow(d * (l * ax - (Ex * ax + Ey * ay + Ez * az)), 2)),
                d * (l * ax - (Ex * ax + Ey * ay + Ez * az)) - r * Ez);
  qy_1 = wrap2pi(qy_1);
  qy_2 = wrap2pi(qy_2);

  qz_1 =
      2 * atan2(-r * (Ey - l) +
                    sqrt(pow(r * Ex, 2) + pow(r * (Ey - l), 2) -
                         pow(d * (l * sy - (Ex * sx + Ey * sy + Ez * sz)), 2)),
                d * (l * sy - (Ex * sx + Ey * sy + Ez * sz)) - r * Ex);
  qz_2 =
      2 * atan2(-r * (Ey - l) -
                    sqrt(pow(r * Ex, 2) + pow(r * (Ey - l), 2) -
                         pow(d * (l * sy - (Ex * sx + Ey * sy + Ez * sz)), 2)),
                d * (l * sy - (Ex * sx + Ey * sy + Ez * sz)) - r * Ex);
  qz_1 = wrap2pi(qz_1);
  qz_2 = wrap2pi(qz_2);

  // Choose the soln between -PI/2 to +PI/2
  qx = choose_soln(qx_1, qx_2);
  qy = choose_soln(qy_1, qy_2);
  qz = choose_soln(qz_1, qz_2);

  q(0) = qx;
  q(1) = qy;
  q(2) = qz;

  return q;
}

void r3us2_ankle::calculate_crankpoints(Vector3d q, MatrixXd &c) {

  double qx, qy, qz;
  qx = q(0);
  qy = q(1);
  qz = q(2);

  // Centers of the 3 circles
  Vector3d bx, by, bz;

  bx(0) = 0;
  bx(1) = 0;
  bx(2) = l;

  by(0) = l;
  by(1) = 0;
  by(2) = 0;

  bz(0) = 0;
  bz(1) = l;
  bz(2) = 0;
  // A general point on 3 circles w.r.t to bx, by, bz
  Vector3d cx, cy, cz;

  cx(0) = 0;
  cx(1) = r * cos(qx);
  cx(2) = r * sin(qx);

  cy(0) = r * sin(qy);
  cy(1) = 0;
  cy(2) = r * cos(qy);

  cz(0) = r * cos(qz);
  cz(1) = r * sin(qz);
  cz(2) = 0;

  // six points on rotative circles
  Vector3d c1, c2, c3, c4, c5, c6;

  c1 = bx + cx;
  c2 = bx - cx;

  c3 = bz + cz;
  c4 = bz - cz;

  c5 = by + cy;
  c6 = by - cy;

  //        cout<<"Crank points:"<<endl;
  //        cout<<"c1 = "<<c1.transpose()<<"\nc2 = "<<c2.transpose()<<"\nc3 =
  //        "<<c3.transpose()<<"\nc4 = "<<c4.transpose()<<"\nc5 =
  //        "<<c5.transpose()<<"\nc6 = "<<c6.transpose()<<endl;

  c.col(0) = c1;
  c.col(1) = c2;
  c.col(2) = c3;
  c.col(3) = c4;
  c.col(4) = c5;
  c.col(5) = c6;
}

void r3us2_ankle::calculate_endeffectorpoints(MatrixXd G_T_E, MatrixXd &e) {

  Vector3d s, n, a, E;
  Vector3d e1, e2, e3, e4, e5, e6;

  s(0) = G_T_E(0, 0);
  s(1) = G_T_E(1, 0);
  s(2) = G_T_E(2, 0);

  n(0) = G_T_E(0, 1);
  n(1) = G_T_E(1, 1);
  n(2) = G_T_E(2, 1);

  a(0) = G_T_E(0, 2);
  a(1) = G_T_E(1, 2);
  a(2) = G_T_E(2, 2);

  E(0) = G_T_E(0, 3);
  E(1) = G_T_E(1, 3);
  E(2) = G_T_E(2, 3);

  // Calculation of position of 6 points(ei) on the end-effector
  e1 = E + d * n;
  e2 = E - d * n;
  e3 = E + d * s;
  e4 = E - d * s;
  e5 = E + d * a;
  e6 = E - d * a;

  //    cout<<"End effector points:"<<endl;
  //    cout<<"e1 = "<<e1.transpose()<<"\ne2 = "<<e2.transpose()<<"\ne3 =
  //    "<<e3.transpose()<<"\ne4 = "<<e4.transpose()<<"\ne5 =
  //    "<<e5.transpose()<<"\ne6 = "<<e6.transpose()<<endl;

  e.col(0) = e1;
  e.col(1) = e2;
  e.col(2) = e3;
  e.col(3) = e4;
  e.col(4) = e5;
  e.col(5) = e6;
}

void r3us2_ankle::calculate_rodlengths(MatrixXd c, MatrixXd e,
                                       VectorXd &rod_lengths) {
  for (unsigned int i = 0; i <= 5; i++)
    rod_lengths(i) = (e.col(i) - c.col(i)).norm();
}

double r3us2_ankle::calculate_rodlengths_error(VectorXd rod_lengths) {
  double error = 0.0;
  for (unsigned int i = 0; i <= 5; i++)
    error = error + pow(rod_lengths(i) - l, 2);
  return error;
}

Vector3d r3us2_ankle::calculate_endeffector_position(MatrixXd G_T_E,
                                                     MatrixXd c) {
  Vector3d s, n, a;
  Vector3d E;

  s(0) = G_T_E(0, 0);
  s(1) = G_T_E(1, 0);
  s(2) = G_T_E(2, 0);

  n(0) = G_T_E(0, 1);
  n(1) = G_T_E(1, 1);
  n(2) = G_T_E(2, 1);

  a(0) = G_T_E(0, 2);
  a(1) = G_T_E(1, 2);
  a(2) = G_T_E(2, 2);

  Vector3d fc1, fc2, fc3;
  double r1, r2, r3;
  unsigned int pos;

  fc3 = -(d * n - c.col(0));
  fc2 = -(d * s - c.col(2));
  fc1 = -(d * a - c.col(4));

  r1 = l;
  r2 = l;
  r3 = l;

  Vector3d soln1, soln2;
  soln1 = sphere_intersection(fc1, fc2, fc3, r1, r2, r3, 0);
  soln2 = sphere_intersection(fc1, fc2, fc3, r1, r2, r3, 1);

  if (soln1(0) > -d && soln1(0) < d && soln1(1) > -d && soln1(1) < d &&
      soln1(2) > -d && soln1(2) < d) {
    E(0) = soln1(0);
    E(1) = soln1(1);
    E(2) = soln1(2);
  } else {
    E(0) = soln2(0);
    E(1) = soln2(1);
    E(2) = soln2(2);
  }
  // cout<<"Intersection point"<<E<<endl;
  return E;
}

void r3us2_ankle::solve_RIGM(MatrixXd G_T_E, Vector3d &q, Vector3d &E) {

  // Vector3d q, E;
  MatrixXd c(3, 6), e(3, 6);
  VectorXd rod_lengths(6);
  double error;

  while (1) {
    q = IGM(G_T_E);
    // cout<<"qx = "<<q(0)<<" qy = "<<q(1)<<" qz = "<<q(2)<<endl;    G_T_E,
    // Vector3d &q, Vector3d &ee

    calculate_crankpoints(q, c);

    calculate_endeffectorpoints(G_T_E, e);

    calculate_rodlengths(c, e, rod_lengths);
    // cout<<"Six Rod lengths"<<rod_lengths<<endl;
    error = calculate_rodlengths_error(rod_lengths);
    // cout<< "Error: " << error<<endl;

    if (error < tolerance)
      break;
    else {
      E = calculate_endeffector_position(G_T_E, c);
      G_T_E(0, 3) = E(0);
      G_T_E(1, 3) = E(1);
      G_T_E(2, 3) = E(2);
    }
    // cout<<"End Effector shift: Ex = "<<G_T_E(0,3)<<" Ey = "<<G_T_E(1,3)<<" Ez
    // = "<<G_T_E(2,3)<<endl;
  }
}

MatrixXd r3us2_ankle::RotMatrixFromAxisAngle(Vector3d axis, double angle) {
  MatrixXd R;
  R.setIdentity(4, 4);

  double c = cos(angle);
  double s = sin(angle);
  double t = 1.0 - c;
  //  if axis is not already normalised then uncomment this
  double magnitude = axis.norm();
  if (magnitude == 0)
    cout << "Error: Axis norm zero";
  axis(0) /= magnitude;
  axis(1) /= magnitude;
  axis(2) /= magnitude;

  R(0, 0) = c + axis(0) * axis(0) * t;
  R(1, 1) = c + axis(1) * axis(1) * t;
  R(2, 2) = c + axis(2) * axis(2) * t;

  double tmp1 = axis(0) * axis(1) * t;
  double tmp2 = axis(2) * s;
  R(1, 0) = tmp1 + tmp2;
  R(0, 1) = tmp1 - tmp2;
  tmp1 = axis(0) * axis(2) * t;
  tmp2 = axis(1) * s;
  R(2, 0) = tmp1 - tmp2;
  R(0, 2) = tmp1 + tmp2;
  tmp1 = axis(1) * axis(2) * t;
  tmp2 = axis(0) * s;
  R(2, 1) = tmp1 + tmp2;
  R(1, 2) = tmp1 - tmp2;

  return R;
}

Vector3d r3us2_ankle::sphere_intersection(Vector3d fc1, Vector3d fc2,
                                          Vector3d fc3, double r1, double r2,
                                          double r3, unsigned int pos) {
  Vector3d result;
  result.setZero();
  double x1, x2, x3, y1, y2, y3, z1, z2, z3;
  double a, b, c;

  x1 = fc1(0);
  y1 = fc1(1);
  z1 = fc1(2);
  x2 = fc2(0);
  y2 = fc2(1);
  z2 = fc2(2);
  x3 = fc3(0);
  y3 = fc3(1);
  z3 = fc3(2);

  x2 = x2 - x1;
  y2 = y2 - y1;
  z2 = z2 - z1;
  x3 = x3 - x1;
  y3 = y3 - y1;
  z3 = z3 - z1;

    a =
    (((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((
    (((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((
    (((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((
    (((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((
    (((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((((
    (((((((((((((((((((((((((((((((((((((((((16.0 * (y2 * y2) * z3 * (y3 * y3) *
    z2 * x3 * (r1 * r1) * x2 - 4.0 * pow(y2, 3.0) * z3 * y3 * z2
    * x3 * (r1 * r1) * x2) + 4.0 * pow(y2, 3.0) * z3 * y3 * z2 *
    x3 * x2 * (r3 * r3)) - 4.0 * y2 * pow(y3, 3.0) * z2 * x2 *
    z3 * (r1 * r1) * x3) + 4.0 * y2 * pow(y3, 3.0) * z2 * x2 *
    z3 * (r2 * r2) * x3) + 16.0 * z2 * (x3 * x3) * (x2 * x2) * (r1 * r1) * y3 *
    y2 * z3) - 4.0 * z2 * pow(x3, 3.0) * x2 * (r1 * r1) * y3 *
    y2 * z3) + 4.0 * z2 * pow(x3, 3.0) * x2 * y2 * z3 * (r2 * r2)
    * y3) - 4.0 * pow(x2, 3.0) * z3 * x3 * y2 * (r1 * r1) * z2 *
    y3) + 4.0 * pow(x2, 3.0) * z3 * x3 * y2 * (r3 * r3) * z2 *
    y3) - 4.0 * y2 * z3 * pow(z2, 3.0) * y3 * x3 * (r1 * r1) *
    x2) + 4.0 * y2 * z3 * pow(z2, 3.0) * y3 * x3 * x2 * (r3 * r3))
    + 8.0 * y2 * (z3 * z3) * (z2 * z2) * y3 * x2 * (r1 * r1) * x3) - 4.0 * y2 *
    (z3 * z3) * (z2 * z2) * y3 * x2 * (r2 * r2) * x3) + 4.0 * (x2 * x2) * (y3 *
    y3) * z2 * (y2 * y2) * z3 * (x3 * x3)) - 4.0 * pow(x2, 4.0) *
    (z3 * z3) * (x3 * x3) * y3 * y2) - 2.0 * (z2 * z2) * pow(x3,
    4.0) * (x2 * x2) * (y2 * y2)) + 2.0 * (z2 * z2) * pow(x3,
    3.0) * (y2 * y2) * (r1 * r1) * x2) - 2.0 * (z2 * z2) * pow
    (x3, 3.0) * (y2 * y2) * x2 * (r3 * r3)) + 2.0 * (z2 * z2) *
    pow(x3, 5.0) * x2 * y2 * y3) + 2.0 * pow(x2,
    5.0) * (z3 * z3) * x3 * y3 * y2) + 2.0 * pow(z2, 3.0) * (y3 *
    y3) * (y2 * y2) * z3 * (x3 * x3)) + 2.0 * pow(z2, 3.0) * (y3
    * y3) * (x2 * x2) * z3 * (x3 * x3)) + 2.0 * pow(z2, 3.0) *
    (y3 * y3) * (x2 * x2) * z3 * (r1 * r1)) - 2.0 * pow(z2, 3.0)
    * (y3 * y3) * (x2 * x2) * z3 * (r3 * r3)) + 2.0 * (x2 * x2) *
    pow(y3, 4.0) * z2 * (y2 * y2) * z3) - 4.0 * (x2 * x2) * (y3 *
    y3) * (z2 * z2) * (x3 * x3) * (y2 * y2)) + 2.0 * pow(x2, 4.0)
    * (y3 * y3) * z2 * z3 * (x3 * x3)) - 2.0 * (x2 * x2) * pow
    (y3, 3.0) * (z2 * z2) * y2 * (x3 * x3)) + 2.0 * (x2 * x2) *
    pow(y3, 3.0) * (z2 * z2) * y2 * (z3 * z3)) + 2.0 *
    pow(x2, 3.0) * (y3 * y3) * (z2 * z2) * x3 * (z3 * z3)) + 2.0
    * (x2 * x2) * (y3 * y3) * z2 * (y2 * y2) * pow(z3, 3.0)) +
    2.0 * (x2 * x2) * pow(y3, 3.0) * (z2 * z2) * y2 * (r1 * r1))
    + 2.0 * (x2 * x2) * (y3 * y3) * (z2 * z2) * (x3 * x3) * (r2 * r2)) + 2.0 *
    pow(x2, 4.0) * (y3 * y3) * z2 * z3 * (r1 * r1)) - 2.0 *
    pow(x2, 4.0) * (y3 * y3) * z2 * z3 * (r3 * r3)) - 2.0 * (x2 *
    x2) * pow(y3, 3.0) * (z2 * z2) * y2 * (r3 * r3)) + 2.0 *
    pow(x2, 3.0) * (y3 * y3) * (z2 * z2) * x3 * (r1 * r1)) - 2.0
    * pow(x2, 3.0) * (y3 * y3) * (z2 * z2) * x3 * (r3 * r3)) +
    2.0 * pow(y2, 4.0) * z3 * (x3 * x3) * (y3 * y3) * z2) + 2.0 *
    (y2 * y2) * z3 * pow(x3, 4.0) * z2 * (x2 * x2)) - 4.0 * (y2 *
    y2) * (z3 * z3) * (x3 * x3) * (x2 * x2) * (y3 * y3)) + 2.0 *
    pow(y2, 3.0) * (z3 * z3) * (x3 * x3) * (z2 * z2) * y3) + 2.0
    * (y2 * y2) * (z3 * z3) * pow(x3, 3.0) * x2 * (z2 * z2)) -
    2.0 * pow(y2, 3.0) * (z3 * z3) * (x3 * x3) * (x2 * x2) * y3)
    + 2.0 * pow(y2, 3.0) * (z3 * z3) * (x3 * x3) * (r1 * r1) *
    y3) + 2.0 * (y2 * y2) * z3 * pow(x3, 4.0) * z2 * (r1 * r1))
    - 2.0 * (y2 * y2) * z3 * pow(x3, 4.0) * z2 * (r2 * r2)) +
    2.0 * (y2 * y2) * (z3 * z3) * (x3 * x3) * (x2 * x2) * (r3 * r3)) - 2.0 *
    pow(y2, 3.0) * (z3 * z3) * (x3 * x3) * (r2 * r2) * y3) + 2.0
    * (y2 * y2) * (z3 * z3) * pow(x3, 3.0) * x2 * (r1 * r1)) -
    2.0 * (y2 * y2) * (z3 * z3) * pow(x3, 3.0) * x2 * (r2 * r2))
    - 2.0 * (y2 * y2) * (z3 * z3) * (y3 * y3) * pow(x2, 3.0) *
    x3) - 4.0 * pow(y2, 4.0) * (z3 * z3) * (y3 * y3) * x2 * x3)
    + 2.0 * (y2 * y2) * (z3 * z3) * (y3 * y3) * (x2 * x2) * (r3 * r3)) + 4.0 *
    pow(y2, 3.0) * (z3 * z3) * y3 * pow(x2, 3.0)
    * x3) + 2.0 * pow(y2, 5.0) * (z3 * z3) * y3 * x2 * x3) + 4.0
    * y2 * pow(y3, 3.0) * (z2 * z2) * pow(x3,
    3.0) * x2) + 2.0 * y2 * pow(y3, 5.0) * (z2 * z2) * x3 * x2)
    - 2.0 * (y2 * y2) * (y3 * y3) * (z2 * z2) * pow(x3, 3.0) *
    x2) - 4.0 * (y2 * y2) * pow(y3, 4.0) * (z2 * z2) * x3 * x2)
    + 2.0 * (y2 * y2) * (y3 * y3) * (z2 * z2) * (x3 * x3) * (r2 * r2)) - 4.0 *
    (z2 * z2) * pow(x3, 4.0) * (x2 * x2) * y2 * y3) + 2.0 * z2 *
    (x3 * x3) * (x2 * x2) * (y2 * y2) * pow(z3, 3.0)) + 2.0 * z2
    * (x3 * x3) * pow(y2, 4.0) * z3 * (r1 * r1)) + 2.0 * (z2 *
    z2) * (x3 * x3) * pow(y2, 3.0) * (r1 * r1) * y3) - 2.0 * z2 *
    (x3 * x3) * pow(y2, 4.0) * z3 * (r3 * r3)) - 2.0 * (z2 * z2)
    * (x3 * x3) * pow(y2, 3.0) * (r3 * r3) * y3) -
    pow(z2, 4.0) * (y3 * y3) * (x3 * x3) * (x2 * x2)) -
    pow(z2, 4.0) * (y3 * y3) * (x3 * x3) * (y2 * y2)) + 2.0 *
    pow(z2, 3.0) * pow(y3, 4.0) * (x2 * x2) * z3)
    + 2.0 * pow(z2, 3.0) * (y3 * y3) * (x2 * x2) *
    pow(z3, 3.0)) + 2.0 * (x2 * x2) * pow(y3,
    5.0) * (z2 * z2) * y2) - 2.0 * (x2 * x2) * pow(y3, 4.0) *
    (z2 * z2) * (y2 * y2)) - 2.0 * pow(x2, 4.0) * (y3 * y3) *
    (z2 * z2) * (x3 * x3)) + 2.0 * pow(x2, 3.0) * (y3 * y3) *
    (z2 * z2) * pow(x3, 3.0)) + 2.0 * pow(x2,
    4.0) * pow(y3, 4.0) * z2 * z3) + 2.0 * pow
    (x2, 3.0) * pow(y3, 4.0) * (z2 * z2) * x3) + 2.0 * (x2 * x2)
    * pow(y3, 4.0) * (z2 * z2) * (r2 * r2)) - 2.0 *
    pow(y2, 4.0) * (z3 * z3) * (x3 * x3) * (y3 * y3)) + 2.0 *
    pow(y2, 5.0) * (z3 * z3) * (x3 * x3) * y3) + 2.0 *
    pow(y2, 4.0) * z3 * pow(x3, 4.0) * z2) + 2.0
    * (y2 * y2) * (z3 * z3) * pow(x3, 3.0) * pow
    (x2, 3.0)) - 2.0 * (y2 * y2) * (z3 * z3) * pow(x3, 4.0) *
    (x2 * x2)) + 2.0 * pow(y2, 4.0) * (z3 * z3) *
    pow(x3, 3.0) * x2) + 2.0 * (y2 * y2) * z3 *
    pow(x3, 4.0) * pow(z2, 3.0)) - y2 * y2 *
    pow(z3, 4.0) * (x3 * x3) * (x2 * x2)) + 2.0 *
    pow(y2, 4.0) * (z3 * z3) * (x3 * x3) * (r3 * r3)) - 2.0 *
    (y2 * y2) * (z3 * z3) * pow(y3, 4.0) * (x2 * x2)) + 2.0 *
    pow(y2, 3.0) * (z3 * z3) * pow(y3, 3.0) *
    (x2 * x2)) - y2 * y2 * pow(z3, 4.0) * (y3 * y3) * (x2 * x2))
    - pow(y2, 4.0) * (z3 * z3) * (y3 * y3) * (x2 * x2)) + 2.0 *
    pow(y2, 3.0) * pow(y3, 3.0) * (z2 * z2) *
    (x3 * x3)) - y2 * y2 * pow(y3, 4.0) * (z2 * z2) * (x3 * x3))
    - 2.0 * pow(y2, 4.0) * (y3 * y3) * (z2 * z2) * (x3 * x3)) -
    pow(z2, 4.0) * pow(y3, 4.0) * (x2 * x2)) -
    2.0 * pow(x2, 4.0) * pow(y3, 4.0) * (z2 * z2))
    - 2.0 * pow(y2, 4.0) * (z3 * z3) * pow(x3,
    4.0)) - pow(y2, 4.0) * pow(z3, 4.0) * (x3 *
    x3)) - 2.0 * (z2 * z2) * pow(x3, 4.0) * pow
    (y2, 4.0)) - pow(z2, 4.0) * pow(x3, 4.0) *
    (y2 * y2)) - 2.0 * pow(x2, 4.0) * (z3 * z3) *
    pow(y3, 4.0)) - pow(x2, 4.0) *
    pow(z3, 4.0) * (y3 * y3)) - z3 * z3 * pow(y2,
    6.0) * (x3 * x3)) - pow(x3, 6.0) * (z2 * z2) * (y2 * y2)) -
    z3 * z3 * pow(x2, 6.0) * (y3 * y3)) - 2.0 *
    pow(y2, 4.0) * pow(x3, 4.0) * (y3 * y3)) +
    2.0 * pow(y2, 4.0) * pow(x3, 4.0) * (r3 * r3))
    + 2.0 * pow(y2, 5.0) * pow(x3, 4.0) * y3) -
    pow(y2, 4.0) * (x3 * x3) * pow(r1, 4.0)) -
    pow(y2, 4.0) * (x3 * x3) * pow(y3, 4.0)) +
    2.0 * pow(y2, 5.0) * (x3 * x3) * pow(y3, 3.0))
    - pow(y2, 4.0) * (x3 * x3) * pow(r3, 4.0)) -
    pow(y2, 6.0) * (x3 * x3) * (y3 * y3)) - y2 * y2 *
    pow(x3, 4.0) * pow(r1, 4.0)) - y2 * y2 *
    pow(x3, 4.0) * pow(x2, 4.0)) + 2.0 * (y2 *
    y2) * pow(x3, 5.0) * pow(x2, 3.0)) - y2 * y2
    * pow(x3, 4.0) * pow(r2, 4.0)) - y2 * y2 *
    pow(x3, 6.0) * (x2 * x2)) - 2.0 * pow(y2,
    4.0) * pow(x3, 4.0) * (x2 * x2)) + 2.0 * pow
    (y2, 4.0) * pow(x3, 4.0) * (r2 * r2)) + 2.0 *
    pow(y2, 4.0) * pow(x3, 5.0) * x2) + 2.0 *
    pow(x2, 4.0) * pow(y3, 5.0) * y2) - 2.0 *
    pow(x2, 4.0) * pow(y3, 4.0) * (y2 * y2)) +
    2.0 * pow(x2, 4.0) * pow(y3, 4.0) * (r2 * r2))
    - x2 * x2 * pow(y3, 4.0) * pow(r1, 4.0)) -
    x2 * x2 * pow(y3, 6.0) * (y2 * y2)) + 2.0 * (x2 * x2) *
    pow(y3, 5.0) * pow(y2, 3.0)) - x2 * x2 *
    pow(y3, 4.0) * pow(y2, 4.0)) - x2 * x2 *
    pow(y3, 4.0) * pow(r2, 4.0)) -
    pow(x2, 4.0) * (y3 * y3) * pow(r1, 4.0)) -
    pow(x2, 6.0) * (y3 * y3) * (x3 * x3)) + 2.0 *
    pow(x2, 5.0) * (y3 * y3) * pow(x3, 3.0)) -
    pow(x2, 4.0) * (y3 * y3) * pow(x3, 4.0)) -
    pow(x2, 4.0) * (y3 * y3) * pow(r3, 4.0)) +
    2.0 * pow(x2, 5.0) * pow(y3, 4.0) * x3) -
    2.0 * pow(x2, 4.0) * pow(y3, 4.0) * (x3 * x3))
    + 2.0 * pow(x2, 4.0) * pow(y3, 4.0) * (r3 *
    r3)) - pow(y3, 6.0) * (z2 * z2) * (x2 * x2)) -
    pow(y2, 4.0) * pow(x3, 6.0)) -
    pow(y2, 6.0) * pow(x3, 4.0)) -
    pow(x2, 6.0) * pow(y3, 4.0)) -
    pow(x2, 4.0) * pow(y3, 6.0)) - 2.0 * z2 *
    (x3 * x3) * (r1 * r1) * (y2 * y2) * z3 * (r3 * r3)) - 2.0 * z2 * (y3 * y3) *
    (r2 * r2) * (x2 * x2) * z3 * (r1 * r1)) + 2.0 * z2 * (y3 * y3) * (r2 * r2) *
    (x2 * x2) * z3 * (r3 * r3)) + 2.0 * pow(x2, 4.0) * (y3 * y3)
    * z2 * pow(z3, 3.0)) + 2.0 * y2 * pow(r1,
    4.0) * (z2 * z2) * y3 * x3 * x2) + 2.0 * (x2 * x2) * (y3 * y3) * z2 * (y2 *
    y2) * z3 * (r1 * r1)) - 8.0 * (x2 * x2) * pow(y3, 3.0) * z2 *
    (r1 * r1) * y2 * z3) - 2.0 * (x2 * x2) * (y3 * y3) * z2 * (y2 * y2) * z3 *
    (r3 * r3)) - 8.0 * pow(x2, 3.0) * (y3 * y3) * z2 * z3 * (r1 *
    r1) * x3) + 2.0 * (y2 * y2) * z3 * (x3 * x3) * (r1 * r1) * (y3 * y3) * z2) -
    8.0 * pow(y2, 3.0) * z3 * (x3 * x3) * (r1 * r1) * z2 * y3) -
    2.0 * (y2 * y2) * z3 * (x3 * x3) * z2 * (y3 * y3) * (r2 * r2)) - 8.0 * (y2 *
    y2) * z3 * pow(x3, 3.0) * z2 * (r1 * r1) * x2) - 4.0 * (y2 *
    y2) * (z3 * z3) * (y3 * y3) * x2 * (z2 * z2) * x3) - 4.0 * (y2 * y2) * (z3 *
    z3) * (y3 * y3) * x2 * (r1 * r1) * x3) + 4.0 * (y2 * y2) * (z3 * z3) * (y3 *
    y3) * x2 * (r2 * r2) * x3) - 4.0 * pow(y2, 3.0) * z3 * y3 *
    z2 * pow(x3, 3.0) * x2) - 4.0 * pow(y2, 3.0)
    * z3 * pow(y3, 3.0) * z2 * x3 * x2) - 4.0 *
    pow(y2, 3.0) * pow(z3, 3.0) * y3 * z2 * x3 *
    x2) + 4.0 * pow(y2, 3.0) * (z3 * z3) * y3 * x2 * (z2 * z2) *
    x3) - 4.0 * pow(y2, 3.0) * (z3 * z3) * y3 * x2 * (r2 * r2) *
    x3) - 4.0 * y2 * pow(y3, 3.0) * z2 * pow(x2,
    3.0) * z3 * x3) + 4.0 * y2 * pow(y3, 3.0) * (z2 * z2) * x3 *
    x2 * (z3 * z3)) - 4.0 * y2 * pow(y3, 3.0) *
    pow(z2, 3.0) * x2 * z3 * x3) - 4.0 * y2 *
    pow(y3, 3.0) * (z2 * z2) * x3 * x2 * (r3 * r3)) - 4.0 * (y2 *
    y2) * (y3 * y3) * (z2 * z2) * x3 * (r1 * r1) * x2) + 4.0 * (y2 * y2) * (y3 *
    y3) * (z2 * z2) * x3 * x2 * (r3 * r3)) - 4.0 * (z2 * z2) * (x3 * x3) * (x2 *
    x2) * y2 * (z3 * z3) * y3) + 2.0 * z2 * (x3 * x3) * (x2 * x2) * (y2 * y2) *
    z3 * (r1 * r1)) - 4.0 * (z2 * z2) * (x3 * x3) * (x2 * x2) * y2 * (r1 * r1) *
    y3) - 2.0 * z2 * (x3 * x3) * (x2 * x2) * (y2 * y2) * z3 * (r3 * r3)) + 4.0 *
    (z2 * z2) * (x3 * x3) * (x2 * x2) * y2 * (r3 * r3) * y3) - 4.0 *
    pow(z2, 3.0) * pow(x3, 3.0) * x2 * y2 * z3 *
    y3) + 4.0 * (z2 * z2) * pow(x3, 3.0) * x2 * y2 * (z3 * z3) *
    y3) - 4.0 * z2 * pow(x3, 3.0) * pow(x2, 3.0)
    * y3 * y2 * z3) - 4.0 * (z2 * z2) * pow(x3, 3.0) * x2 * y2 *
    (r3 * r3) * y3) + 4.0 * pow(x2, 3.0) * (z3 * z3) * x3 * y2 *
    (z2 * z2) * y3) - 4.0 * pow(x2, 3.0) * pow
    (z3, 3.0) * x3 * y2 * z2 * y3) - 4.0 * pow(x2, 3.0) * (z3 *
    z3) * x3 * y2 * (r2 * r2) * y3) + 2.0 * (x2 * x2) * z3 * (x3 * x3) * (r1 *
    r1) * (y3 * y3) * z2) - 4.0 * (x2 * x2) * (z3 * z3) * (x3 * x3) * (r1 * r1) *
    y3 * y2) - 2.0 * (x2 * x2) * z3 * (x3 * x3) * z2 * (y3 * y3) * (r2 * r2)) +
    4.0 * (x2 * x2) * (z3 * z3) * (x3 * x3) * y2 * (r2 * r2) * y3) - 4.0 * y2 *
    pow(z3, 3.0) * pow(z2, 3.0) * y3 * x3 * x2)
    + 2.0 * y2 * (z3 * z3) * pow(z2, 4.0) * y3 * x2 * x3) + 2.0 *
    y2 * pow(z3, 4.0) * (z2 * z2) * y3 * x3 * x2) - 2.0 * (r1 *
    r1) * (y3 * y3) * z2 * (x2 * x2) * z3 * (r3 * r3)) - 2.0 * (y2 * y2) * z3 *
    (r1 * r1) * z2 * (x3 * x3) * (r2 * r2)) + 2.0 * pow(r1, 4.0)
    * y3 * y2 * (z3 * z3) * x2 * x3) + 2.0 * (z2 * z2) * pow(x3,
    5.0) * (y2 * y2) * x2) + 2.0 * (z2 * z2) * pow(x3, 4.0) *
    pow(y2, 3.0) * y3) + 2.0 * z2 * (x3 * x3) *
    pow(y2, 4.0) * pow(z3, 3.0)) + 2.0 * (z2 *
    z2) * pow(x3, 4.0) * (y2 * y2) * (r2 * r2)) - z2 * z2 *
    pow(x3, 4.0) * (x2 * x2) * (y3 * y3)) + 2.0 *
    pow(x2, 5.0) * (z3 * z3) * x3 * (y3 * y3)) -
    pow(x2, 4.0) * (z3 * z3) * (x3 * x3) * (y2 * y2)) - 2.0 *
    pow(x2, 4.0) * (z3 * z3) * (x3 * x3) * (y3 * y3)) + 2.0 *
    pow(x2, 4.0) * (z3 * z3) * pow(y3, 3.0) * y2)
    + 2.0 * pow(x2, 4.0) * (z3 * z3) * (y3 * y3) * (r3 * r3)) -
    2.0 * (y2 * y2) * pow(x3, 4.0) * (z2 * z2) * (y3 * y3)) -
    2.0 * (z2 * z2) * (x3 * x3) * (x2 * x2) * pow(y3, 4.0)) -
    2.0 * (x2 * x2) * (z3 * z3) * pow(y2, 4.0) * (x3 * x3)) -
    2.0 * pow(x2, 4.0) * (y3 * y3) * (y2 * y2) * (z3 * z3)) +
    2.0 * (y2 * y2) * pow(z3, 3.0) * pow(z2, 3.0)
    * (x3 * x3)) - z3 * z3 * (y2 * y2) * pow(r1, 4.0) * (x3 * x3))
    - z3 * z3 * (y2 * y2) * pow(z2, 4.0) * (x3 * x3)) - z3 * z3 *
    (y2 * y2) * pow(r2, 4.0) * (x3 * x3)) - 2.0 * (z3 * z3) *
    pow(y2, 4.0) * (x3 * x3) * (z2 * z2)) + 2.0 * (z3 * z3) *
    pow(y2, 4.0) * (x3 * x3) * (r2 * r2)) - 2.0 *
    pow(x3, 4.0) * (z2 * z2) * (y2 * y2) * (z3 * z3)) + 2.0 *
    pow(x3, 4.0) * (z2 * z2) * (y2 * y2) * (r3 * r3)) - x3 * x3 *
    (z2 * z2) * (y2 * y2) * pow(r1, 4.0)) - x3 * x3 * (z2 * z2) *
    (y2 * y2) * pow(z3, 4.0)) - x3 * x3 * (z2 * z2) * (y2 * y2) *
    pow(r3, 4.0)) - 2.0 * (z3 * z3) * pow(x2,
    4.0) * (y3 * y3) * (z2 * z2)) + 2.0 * (z3 * z3) * pow(x2,
    4.0) * (y3 * y3) * (r2 * r2)) - z3 * z3 * (x2 * x2) * pow(r1,
    4.0) * (y3 * y3)) - z3 * z3 * (x2 * x2) * pow(z2, 4.0) * (y3
    * y3)) - z3 * z3 * (x2 * x2) * pow(r2, 4.0) * (y3 * y3)) +
    2.0 * pow(y2, 3.0) * pow(x3, 4.0) * (r1 * r1)
    * y3) - 2.0 * pow(y2, 3.0) * pow(x3, 4.0) *
    (x2 * x2) * y3) - 2.0 * pow(y2, 3.0) * pow
    (x3, 4.0) * (r2 * r2) * y3) + 2.0 * pow(y2, 4.0) *
    pow(x3, 3.0) * (r1 * r1) * x2) - 2.0 * pow
    (y2, 4.0) * pow(x3, 3.0) * x2 * (y3 * y3)) - 2.0 * (z2 * z2)
    * (x3 * x3) * (x2 * x2) * (y3 * y3) * (z3 * z3)) + 2.0 * (z2 * z2) * (x3 *
    x3) * (x2 * x2) * (y3 * y3) * (r3 * r3)) - 2.0 * (x2 * x2) * (z3 * z3) * (y2
    * y2) * (x3 * x3) * (z2 * z2)) + 2.0 * (x2 * x2) * (z3 * z3) * (y2 * y2) *
    (x3 * x3) * (r2 * r2)) + 2.0 * (x2 * x2) * (y3 * y3) * (y2 * y2) * (z3 * z3)
    * (r2 * r2)) + 2.0 * (y2 * y2) * pow(z3, 3.0) * z2 * (x3 *
    x3) * (r1 * r1)) - 2.0 * (y2 * y2) * pow(z3, 3.0) * z2 * (x3
    * x3) * (r2 * r2)) + 2.0 * pow(z2, 3.0) * (x3 * x3) * (y2 *
    y2) * z3 * (r1 * r1)) - 2.0 * pow(z2, 3.0) * (x3 * x3) * (y2
    * y2) * z3 * (r3 * r3)) + 2.0 * (x2 * x2) * pow(z3, 3.0) *
    (r1 * r1) * (y3 * y3) * z2) - 2.0 * (x2 * x2) * pow(z3, 3.0)
    * z2 * (y3 * y3) * (r2 * r2)) + 2.0 * pow(r1, 4.0) * (y3 *
    y3) * z2 * (x2 * x2) * z3) + 2.0 * (y2 * y2) * z3 * pow(r1,
    4.0) * z2 * (x3 * x3)) + 2.0 * (x2 * x2) * z3 * pow(y3, 4.0)
    * (r1 * r1) * z2) + 2.0 * (x2 * x2) * (z3 * z3) * pow(y3,
    3.0) * (r1 * r1) * y2) - 2.0 * (x2 * x2) * z3 * pow(y3, 4.0)
    * z2 * (r2 * r2)) - 2.0 * (x2 * x2) * (z3 * z3) * pow(y3,
    3.0) * y2 * (r2 * r2)) + 2.0 * pow(x2, 3.0) * (z3 * z3) *
    (y3 * y3) * (r1 * r1) * x3) - 2.0 * pow(x2, 3.0) * (z3 * z3)
    * (y3 * y3) * (r2 * r2) * x3) - 2.0 * (y2 * y2) * (z3 * z3) * (z2 * z2) *
    (y3 * y3) * (x2 * x2)) - 2.0 * (y2 * y2) * (x3 * x3) * (z2 * z2) * (y3 * y3)
    * (z3 * z3)) + 2.0 * (y2 * y2) * (x3 * x3) * (z2 * z2) * (y3 * y3) * (r3 *
    r3)) - 4.0 * y2 * (z3 * z3) * (z2 * z2) * y3 * x3 * x2 * (r3 * r3)) - 4.0 *
    y2 * pow(z3, 3.0) * z2 * y3 * x2 * (r1 * r1) * x3) + 4.0 *
    y2 * pow(z3, 3.0) * z2 * y3 * x2 * (r2 * r2) * x3) - 4.0 *
    pow(r1, 4.0) * y3 * y2 * z3 * z2 * x3 * x2) - 4.0 * (r1 * r1)
    * y3 * y2 * (z3 * z3) * x2 * (r2 * r2) * x3) - 4.0 * y2 * (r1 * r1) * (z2 *
    z2) * y3 * x3 * x2 * (r3 * r3)) + 4.0 * (r1 * r1) * y3 * y2 * z3 * z2 * x3 *
    x2 * (r3 * r3)) + 4.0 * y2 * (r1 * r1) * z2 * y3 * x2 * z3 * (r2 * r2) * x3)
    + 2.0 * z2 * (x3 * x3) * (r2 * r2) * (y2 * y2) * z3 * (r3 * r3)) + 2.0 * y2 *
    (z3 * z3) * pow(r2, 4.0) * y3 * x2 * x3) + 2.0 * y2 *
    pow(r3, 4.0) * (z2 * z2) * y3 * x3 * x2) + 4.0 * (y2 * y2) *
    x3 * x2 * (y3 * y3) * (r1 * r1) * (r3 * r3)) + 4.0 * (y2 * y2) * x3 * x2 *
    (y3 * y3) * (r1 * r1) * (r2 * r2)) - 4.0 * (y2 * y2) * x3 * x2 * (y3 * y3) *
    (r3 * r3) * (r2 * r2)) + 4.0 * y2 * (x3 * x3) * (x2 * x2) * y3 * (r1 * r1) *
    (r2 * r2)) + 4.0 * y2 * (x3 * x3) * (x2 * x2) * y3 * (r1 * r1) * (r3 * r3))
    - 4.0 * y2 * (x3 * x3) * (x2 * x2) * y3 * (r2 * r2) * (r3 * r3)) - 4.0 *
    pow(y2, 3.0) * x3 * x2 * y3 * (z3 * z3) * (r3 * r3)) - 4.0 *
    y2 * x3 * x2 * pow(y3, 3.0) * (r1 * r1) * (r2 * r2)) - 4.0 *
    pow(y2, 3.0) * x3 * x2 * y3 * (r1 * r1) * (r3 * r3)) - 4.0 *
    y2 * x3 * x2 * pow(y3, 3.0) * (z2 * z2) * (r2 * r2)) - 4.0 *
    y2 * x3 * pow(x2, 3.0) * y3 * (r1 * r1) * (r3 * r3)) - 4.0 *
    y2 * pow(x3, 3.0) * x2 * y3 * (r1 * r1) * (r2 * r2)) - 4.0 *
    y2 * pow(x3, 3.0) * x2 * y3 * (z2 * z2) * (r2 * r2)) - 4.0 *
    y2 * x3 * pow(x2, 3.0) * y3 * (z3 * z3) * (r3 * r3)) - 4.0 *
    (z3 * z3) * (y2 * y2) * (r1 * r1) * (x3 * x3) * (z2 * z2)) + 2.0 * (z3 * z3)
    * (y2 * y2) * (r1 * r1) * (x3 * x3) * (r2 * r2)) + 2.0 * (z3 * z3) * (y2 *
    y2) * (z2 * z2) * (x3 * x3) * (r2 * r2)) + 2.0 * (x3 * x3) * (z2 * z2) * (y2
    * y2) * (z3 * z3) * (r3 * r3)) + 2.0 * (x3 * x3) * (z2 * z2) * (y2 * y2) *
    (r1 * r1) * (r3 * r3)) - 4.0 * (z3 * z3) * (x2 * x2) * (r1 * r1) * (y3 * y3)
    * (z2 * z2)) + 2.0 * (z3 * z3) * (x2 * x2) * (r1 * r1) * (y3 * y3) * (r2 *
    r2)) + 2.0 * (z3 * z3) * (x2 * x2) * (z2 * z2) * (y3 * y3) * (r2 * r2)) -
    2.0 * pow(y2, 3.0) * (x3 * x3) * (r1 * r1) * y3 * (r3 * r3))
    - 2.0 * pow(y2, 3.0) * (x3 * x3) * (x2 * x2) * y3 * (r1 * r1))
    + 2.0 * pow(y2, 3.0) * (x3 * x3) * (x2 * x2) * y3 * (r3 * r3))
    - 2.0 * pow(y2, 3.0) * (x3 * x3) * (r1 * r1) * (r2 * r2) *
    y3) + 2.0 * pow(y2, 3.0) * (x3 * x3) * (r3 * r3) * (r2 * r2)
    * y3) - 2.0 * (y2 * y2) * pow(x3, 3.0) * (r1 * r1) * x2 *
    (r2 * r2)) - 2.0 * (y2 * y2) * pow(x3, 3.0) * (r1 * r1) * x2
    * (y3 * y3)) - 2.0 * (y2 * y2) * pow(x3, 3.0) * (r1 * r1) *
    x2 * (r3 * r3)) + 2.0 * (y2 * y2) * pow(x3, 3.0) * (r2 * r2)
    * x2 * (y3 * y3)) + 2.0 * (y2 * y2) * pow(x3, 3.0) * (r2 *
    r2) * x2 * (r3 * r3)) + 2.0 * (y2 * y2) * (x3 * x3) * (r1 * r1) * (y3 * y3) *
    (r2 * r2)) + 4.0 * (y2 * y2) * (x3 * x3) * (x2 * x2) * (y3 * y3) * (r2 * r2))
    + 2.0 * (y2 * y2) * (x3 * x3) * (r1 * r1) * (x2 * x2) * (r3 * r3)) + 4.0 *
    (y2 * y2) * (x3 * x3) * (x2 * x2) * (y3 * y3) * (r3 * r3)) - 8.0 *
    pow(y2, 3.0) * pow(x3, 3.0) * (r1 * r1) * x2
    * y3) - 2.0 * (x2 * x2) * pow(y3, 3.0) * (r1 * r1) * y2 *
    (x3 * x3)) - 2.0 * (x2 * x2) * pow(y3, 3.0) * (r1 * r1) * y2
    * (r3 * r3)) - 2.0 * (x2 * x2) * pow(y3, 3.0) * y2 * (r1 *
    r1) * (r2 * r2)) + 2.0 * (x2 * x2) * pow(y3, 3.0) * y2 * (x3
    * x3) * (r2 * r2)) + 2.0 * (x2 * x2) * pow(y3, 3.0) * y2 *
    (r3 * r3) * (r2 * r2)) - 2.0 * pow(x2, 3.0) * (y3 * y3) *
    (r1 * r1) * (y2 * y2) * x3) - 2.0 * pow(x2, 3.0) * (y3 * y3)
    * (r1 * r1) * (r2 * r2) * x3) - 2.0 * pow(x2, 3.0) * (y3 *
    y3) * (r1 * r1) * x3 * (r3 * r3)) + 2.0 * pow(x2, 3.0) * (y3
    * y3) * (y2 * y2) * x3 * (r3 * r3)) + 2.0 * pow(x2, 3.0) *
    (y3 * y3) * (r2 * r2) * x3 * (r3 * r3)) + 2.0 * (x2 * x2) * (y3 * y3) * (y2 *
    y2) * (r1 * r1) * (r3 * r3)) - 4.0 * y2 * z3 * (r2 * r2) * y3 * z2 * x3 * x2
    * (r3 * r3)) - 4.0 * pow(x2, 4.0) * (y3 * y3) * (r1 * r1) *
    (x3 * x3)) + 2.0 * pow(x2, 4.0) * (y3 * y3) * (r1 * r1) *
    (r3 * r3)) + 2.0 * pow(x2, 3.0) * (y3 * y3) * (r1 * r1) *
    pow(x3, 3.0)) + 2.0 * pow(x2, 4.0) * (y3 *
    y3) * (x3 * x3) * (r2 * r2)) - 2.0 * pow(x2, 5.0) * (y3 * y3)
    * x3 * (r3 * r3)) - 2.0 * pow(x2, 3.0) * (y3 * y3) * (r2 *
    r2) * pow(x3, 3.0)) + 2.0 * pow(x2, 4.0) *
    (y3 * y3) * (x3 * x3) * (r3 * r3)) - y3 * y3 * (z2 * z2) *
    pow(r1, 4.0) * (x2 * x2)) - y3 * y3 * (z2 * z2) * (x2 * x2) *
    pow(z3, 4.0)) - y3 * y3 * (z2 * z2) * (x2 * x2) *
    pow(r3, 4.0)) - 2.0 * pow(y3, 4.0) * (z2 *
    z2) * (x2 * x2) * (z3 * z3)) + 2.0 * pow(y3, 4.0) * (z2 * z2)
    * (x2 * x2) * (r3 * r3)) + 4.0 * pow(y2, 3.0) * x3 *
    pow(x2, 3.0) * pow(y3, 3.0)) + 4.0 *
    pow(y2, 3.0) * pow(x3, 3.0) * x2 *
    pow(y3, 3.0)) + 2.0 * y2 * x3 * pow(x2, 5.0)
    * pow(y3, 3.0)) + 2.0 * pow(y2, 3.0) *
    pow(x3, 5.0) * x2 * y3) + 2.0 * pow(y2, 3.0)
    * x3 * x2 * pow(y3, 5.0)) - 4.0 * pow(y2,
    4.0) * x3 * x2 * pow(y3, 4.0)) + 2.0 * pow
    (y2, 5.0) * x3 * x2 * pow(y3, 3.0)) + 2.0 * y2 *
    pow(x3, 3.0) * pow(x2, 5.0) * y3) - 4.0 * y2
    * pow(x3, 4.0) * pow(x2, 4.0) * y3) + 2.0 *
    pow(y2, 5.0) * pow(x3, 3.0) * x2 * y3) + 2.0
    * y2 * pow(x3, 5.0) * pow(x2, 3.0) * y3) +
    2.0 * y2 * x3 * pow(x2, 3.0) * pow(y3, 5.0))
    + 4.0 * pow(y2, 3.0) * pow(x3, 3.0) *
    pow(x2, 3.0) * y3) + 4.0 * y2 * pow(x3, 3.0)
    * pow(x2, 3.0) * pow(y3, 3.0)) - 2.0 *
    pow(y2, 4.0) * pow(x3, 3.0) * x2 * (r3 * r3))
    - 2.0 * pow(y2, 5.0) * (x3 * x3) * (r3 * r3) * y3) + 2.0 *
    pow(y2, 4.0) * (x3 * x3) * (y3 * y3) * (r2 * r2)) + 2.0 *
    pow(y2, 3.0) * (x3 * x3) * (r1 * r1) * pow
    (y3, 3.0)) + 2.0 * pow(y2, 3.0) * (x3 * x3) *
    pow(r1, 4.0) * y3) - 4.0 * pow(y2, 4.0) *
    (x3 * x3) * (r1 * r1) * (y3 * y3)) - 3.0 * pow(y2, 4.0) *
    (x3 * x3) * (x2 * x2) * (y3 * y3)) + 2.0 * pow(y2, 4.0) *
    (x3 * x3) * (r1 * r1) * (r3 * r3)) + 2.0 * pow(y2, 5.0) *
    (x3 * x3) * (r1 * r1) * y3) + 2.0 * pow(y2, 4.0) * (x3 * x3)
    * (y3 * y3) * (r3 * r3)) - 2.0 * pow(y2, 3.0) * (x3 * x3) *
    pow(y3, 3.0) * (r2 * r2)) - y2 * y2 * (x3 * x3) *
    pow(r1, 4.0) * (y3 * y3)) - 3.0 * (y2 * y2) * (x3 * x3) *
    pow(x2, 4.0) * (y3 * y3)) - y2 * y2 * (x3 * x3) *
    pow(r2, 4.0) * (y3 * y3)) - y2 * y2 * (x3 * x3) *
    pow(r1, 4.0) * (x2 * x2)) - 3.0 * (y2 * y2) * (x3 * x3) *
    (x2 * x2) * pow(y3, 4.0)) - y2 * y2 * (x3 * x3) * (x2 * x2) *
    pow(r3, 4.0)) + 2.0 * (y2 * y2) * pow(x3,
    3.0) * pow(r1, 4.0) * x2) + 2.0 * (y2 * y2) *
    pow(x3, 3.0) * (r1 * r1) * pow(x2, 3.0)) -
    4.0 * (y2 * y2) * pow(x3, 4.0) * (r1 * r1) * (x2 * x2)) +
    2.0 * (y2 * y2) * pow(x3, 4.0) * (r1 * r1) * (r2 * r2)) +
    2.0 * (y2 * y2) * pow(x3, 5.0) * (r1 * r1) * x2) + 2.0 * (y2
    * y2) * pow(x3, 4.0) * (x2 * x2) * (r2 * r2)) - 2.0 * (y2 *
    y2) * pow(x3, 3.0) * pow(x2, 3.0) * (r3 * r3))
    - 2.0 * (y2 * y2) * pow(x3, 5.0) * (r2 * r2) * x2) - 3.0 *
    (y2 * y2) * pow(x3, 4.0) * (x2 * x2) * (y3 * y3)) + 2.0 *
    (y2 * y2) * pow(x3, 4.0) * (x2 * x2) * (r3 * r3)) + 2.0 *
    pow(x2, 4.0) * pow(y3, 3.0) * y2 * (r1 * r1))
    - 2.0 * pow(x2, 4.0) * pow(y3, 3.0) * y2 *
    (x3 * x3)) - 2.0 * pow(x2, 4.0) * pow(y3,
    3.0) * y2 * (r3 * r3)) + 2.0 * pow(x2, 3.0) *
    pow(y3, 4.0) * (r1 * r1) * x3) - 2.0 * pow
    (x2, 3.0) * pow(y3, 4.0) * (y2 * y2) * x3) - 2.0 *
    pow(x2, 3.0) * pow(y3, 4.0) * (r2 * r2) * x3)
    - 2.0 * (x2 * x2) * pow(y3, 3.0) * pow(y2,
    3.0) * (r3 * r3)) + 2.0 * (x2 * x2) * pow(y3, 4.0) * (y2 *
    y2) * (r2 * r2)) + 2.0 * (x2 * x2) * pow(y3, 5.0) * (r1 * r1)
    * y2) + 2.0 * (x2 * x2) * pow(y3, 3.0) * pow
    (r1, 4.0) * y2) - 4.0 * (x2 * x2) * pow(y3, 4.0) * (r1 * r1)
    * (y2 * y2)) + 2.0 * (x2 * x2) * pow(y3, 4.0) * (r1 * r1) *
    (r2 * r2)) + 2.0 * (x2 * x2) * pow(y3, 3.0) *
    pow(y2, 3.0) * (r1 * r1)) + 2.0 * (x2 * x2) *
    pow(y3, 4.0) * (y2 * y2) * (r3 * r3)) - 2.0 * (x2 * x2) *
    pow(y3, 5.0) * y2 * (r2 * r2)) - x2 * x2 * (y3 * y3) * (y2 *
    y2) * pow(r1, 4.0)) - x2 * x2 * (y3 * y3) * (y2 * y2) *
    pow(r3, 4.0)) - x2 * x2 * (y3 * y3) * pow(r1,
    4.0) * (x3 * x3)) - x2 * x2 * (y3 * y3) * pow(r2, 4.0) * (x3
    * x3)) + 2.0 * pow(x2, 3.0) * (y3 * y3) *
    pow(r1, 4.0) * x3) + 2.0 * pow(x2, 5.0) *
    (y3 * y3) * (r1 * r1) * x3) + 2.0 * (x2 * x2) * (y3 * y3) * (r1 * r1) * (x3 *
    x3) * (r2 * r2)) - 8.0 * pow(x2, 3.0) * pow
    (y3, 3.0) * (r1 * r1) * y2 * x3) + 2.0 * (y3 * y3) * (z2 * z2) * (r1 * r1) *
    (x2 * x2) * (r3 * r3)) + 2.0 * (y3 * y3) * (z2 * z2) * (x2 * x2) * (z3 * z3)
    * (r3 * r3)) + 4.0 * pow(y2, 4.0) * x3 * x2 * (y3 * y3) *
    (r3 * r3)) + 4.0 * pow(y2, 3.0) * x3 * x2 *
    pow(y3, 3.0) * (z2 * z2)) - 4.0 * pow(y2,
    3.0) * x3 * x2 * pow(y3, 3.0) * (r2 * r2)) - 4.0 * (y2 * y2)
    * x3 * x2 * pow(y3, 4.0) * (r1 * r1)) - 4.0 * (y2 * y2) * x3
    * x2 * (y3 * y3) * pow(r1, 4.0)) + 8.0 * pow
    (y2, 3.0) * x3 * x2 * pow(y3, 3.0) * (r1 * r1)) + 4.0 * y2 *
    x3 * pow(x2, 3.0) * pow(y3, 3.0) * (z2 * z2))
    - 4.0 * y2 * x3 * pow(x2, 3.0) * pow(y3, 3.0)
    * (r2 * r2)) - 4.0 * pow(y2, 4.0) * x3 * x2 * (y3 * y3) *
    (r1 * r1)) + 4.0 * pow(y2, 3.0) * pow(x3,
    3.0) * x2 * y3 * (z3 * z3)) - 4.0 * pow(y2, 3.0) *
    pow(x3, 3.0) * x2 * y3 * (r3 * r3)) + 4.0 *
    pow(y2, 3.0) * x3 * x2 * pow(y3, 3.0) * (z3 *
    z3)) - 4.0 * pow(y2, 3.0) * x3 * x2 * pow(y3,
    3.0) * (r3 * r3)) + 4.0 * (y2 * y2) * x3 * x2 * pow(y3, 4.0)
    * (r2 * r2)) + 2.0 * y2 * x3 * x2 * pow(y3, 3.0) *
    pow(r1, 4.0)) + 2.0 * pow(y2, 3.0) * x3 * x2
    * y3 * pow(r1, 4.0)) + 2.0 * pow(y2, 3.0) *
    x3 * x2 * y3 * pow(z3, 4.0)) + 2.0 * pow(y2,
    3.0) * x3 * x2 * y3 * pow(r3, 4.0)) + 2.0 * y2 * x3 * x2 *
    pow(y3, 3.0) * pow(z2, 4.0)) + 2.0 * y2 * x3
    * x2 * pow(y3, 3.0) * pow(r2, 4.0)) + 2.0 *
    y2 * x3 * pow(x2, 3.0) * y3 * pow(r1, 4.0))
                       + 2.0 * y2 * pow(x3, 3.0) * x2 * y3 *
                       pow(r1, 4.0)) + 2.0 * y2 *
                      pow(x3, 3.0) * x2 * y3 *
                      pow(z2, 4.0)) + 2.0 * y2 *
                     pow(x3, 3.0) * x2 * y3 *
                     pow(r2, 4.0)) + 2.0 * y2 * x3 *
                    pow(x2, 3.0) * y3 * pow(z3,
    4.0)) + 2.0 * y2 * x3 * pow(x2, 3.0) * y3 *
                   pow(r3, 4.0)) - 4.0 * y2 * (x3 * x3) * (x2 *
    x2) * y3 * pow(r1, 4.0)) - 4.0 * y2 * (x3 * x3) *
                 pow(x2, 4.0) * y3 * (r1 * r1)) + 8.0 * y2 *
                pow(x3, 3.0) * pow(x2, 3.0) * y3
                * (r1 * r1)) - 4.0 * y2 * pow(x3, 4.0) * (x2 *
    x2) * y3 * (r1 * r1)) + 4.0 * y2 * pow(x3, 3.0) *
              pow(x2, 3.0) * y3 * (z2 * z2)) - 4.0 * y2 *
             pow(x3, 3.0) * pow(x2, 3.0) * y3 *
             (r2 * r2)) + 4.0 * y2 * (x3 * x3) * pow(x2, 4.0) *
            y3 * (r3 * r3)) + 4.0 * pow(y2, 3.0) *
           pow(x3, 3.0) * x2 * y3 * (z2 * z2)) - 4.0 *
          pow(y2, 3.0) * pow(x3, 3.0) * x2 * y3 *
          (r2 * r2)) + 4.0 * y2 * pow(x3, 4.0) * (x2 * x2) * y3 *
         (r2 * r2)) + 4.0 * y2 * pow(x3, 3.0) *
        pow(x2, 3.0) * y3 * (z3 * z3)) - 4.0 * y2 *
       pow(x3, 3.0) * pow(x2, 3.0) * y3 * (r3 *
        r3)) + 4.0 * y2 * x3 * pow(x2, 3.0) *
      pow(y3, 3.0) * (z3 * z3)) - 4.0 * y2 * x3 *
     pow(x2, 3.0) * pow(y3, 3.0) * (r3 * r3)) +
    16.0 * (y2 * y2) * (x3 * x3) * (x2 * x2) * (y3 * y3) * (r1 * r1);

    b = ((((((((((((((((((((((((((((((((((((((-pow(z2, 3.0) * (y3 * y3) -
                                              x2 * x2 * (y3 * y3) * z2) -
                                             y2 * y2 * z3 * (x3 * x3)) -
                                            y2 * y2 * z3 * (y3 * y3)) +
                                           pow(y2, 3.0) * z3 * y3) +
                                          y2 * pow(y3, 3.0) * z2) -
                                         y2 * y2 * (y3 * y3) * z2) -
                                        z2 * (x3 * x3) * (x2 * x2)) -
                                       z2 * (x3 * x3) * (y2 * y2)) +
                                      z2 * pow(x3, 3.0) * x2) +
                                     pow(x2, 3.0) * z3 * x3) -
                                    x2 * x2 * z3 * (x3 * x3)) -
                                   x2 * x2 * z3 * (y3 * y3)) +
                                  y2 * z3 * (z2 * z2) * y3) +
                                 y2 * (x3 * x3) * z2 * y3) +
                                y2 * (z3 * z3) * z2 * y3) +
                               z2 * x3 * x2 * (y3 * y3)) +
                              z2 * x3 * x2 * (z3 * z3)) +
                             x2 * z3 * (y2 * y2) * x3) +
                            x2 * z3 * (z2 * z2) * x3) +
                           x2 * x2 * y3 * y2 * z3) -
                          y2 * y2 * pow(z3, 3.0)) -
                         pow(z2, 3.0) * (x3 * x3)) -
                        x2 * x2 * pow(z3, 3.0)) -
                       r1 * r1 * (y3 * y3) * z2) -
                      y2 * y2 * z3 * (r1 * r1)) +
                     r1 * r1 * y3 * y2 * z3) +
                    y2 * (r1 * r1) * z2 * y3) +
                   z2 * (y3 * y3) * (r2 * r2)) -
                  z2 * (x3 * x3) * (r1 * r1)) +
                 z2 * (x3 * x3) * (r2 * r2)) -
                x2 * x2 * z3 * (r1 * r1)) +
               x2 * x2 * z3 * (r3 * r3)) +
              y2 * y2 * z3 * (r3 * r3)) -
             y2 * z3 * (r2 * r2) * y3) -
            y2 * (r3 * r3) * z2 * y3) +
           z2 * x3 * (r1 * r1) * x2) -
          z2 * x3 * x2 * (r3 * r3)) +
         x2 * z3 * (r1 * r1) * x3) -
        x2 * z3 * (r2 * r2) * x3;

    c = (((((((-2.0 * y2 * z3 * z2 * y3 - 2.0 * z2 * x3 * x2 * z3) +
              z3 * z3 * (y2 * y2)) +
             x3 * x3 * (z2 * z2)) +
            z3 * z3 * (x2 * x2)) +
           y2 * y2 * (x3 * x3)) +
          x2 * x2 * (y3 * y3)) +
         y3 * y3 * (z2 * z2)) -
        2.0 * y2 * x3 * x2 * y3;

    // cout<<"a = "<<a<<" b = "<<b<<" c = "<<c<<endl;

    if (a < 0 || c == 0) {
      cout << "c is the denominator and hence must be non-zero" << endl;
      cout << "a is under a root so must be positive" << endl;
      return result;
    }

    double za, zb, z;

    za = -0.5 * (b - sqrt(a)) / c;
    zb = -0.5 * (b + sqrt(a)) / c;

    // cout<<"za = "<<za<<" zb = "<<zb<<endl;

    if (za > zb) {
      if (pos != 0) {
        z = za;
      } else {
        z = zb;
      }
    } else if (pos != 0) {
      z = zb;
    } else {
      z = za;
    }

    a = (r1 * r1) * x2 - (r1 * r1) * x3 + (r2 * r2) * x3 - (r3 * r3) * x2 +
        x2 * (x3 * x3) - (x2 * x2) * x3 + x2 * (y3 * y3) - x3 * (y2 * y2) +
        x2 * (z3 * z3) - x3 * (z2 * z2) - x2 * z * z3 * 2.0 + x3 * z * z2 * 2.0;
    b = -2.0 * y2 * x3 + 2.0 * x2 * y3;
    // cout<<"a = "<<a<<" b = "<<b<<endl;
    if (b == 0) {
      cout << "b is the denominator in the expression" << endl;
      return result;
    }

    double x, y;

    y = a / b;

    if (x2 == 0) {
      cout << "x2 is the denominator in the expression" << endl;
      return result;
    }

    x = 0.5 *
        (r1 * r1 + x2 * x2 - 2 * y * y2 + y2 * y2 - 2 * z * z2 + z2 * z2 -
         r2 * r2) /
        x2;
    // cout<<"x = "<<x<<" y = "<<y<<endl;
    //  convert result back to global
    result(0) = x1 + x;
    result(1) = y1 + y;
    result(2) = z1 + z;

    // cout<<"Intersection point"<<result;
    return result;
}
} // namespace R3US2_ANKLE
