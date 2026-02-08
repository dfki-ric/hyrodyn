#include "UPS6.hpp"

namespace UPS6 {

Eigen::Matrix4d BuildTransMat(const Math::Matrix3d rot, const Math::Vector3d trans) {
  Eigen::Matrix4d TransMat = Eigen::Matrix4d::Zero();

  TransMat.block(0, 0, 3, 3) = rot;
  TransMat(0, 3) = trans(0);
  TransMat(1, 3) = trans(1);
  TransMat(2, 3) = trans(2);
  TransMat(3, 3) = 1;

  return TransMat;
}

Eigen::Vector4d BuildTransVec(const Math::Vector3d pos) {
  Eigen::Vector4d TransVec = Eigen::Vector4d::Zero();

  TransVec(0) = pos(0);
  TransVec(1) = pos(1);
  TransVec(2) = pos(2);
  TransVec(3) = 1;

  return TransVec;
}

Eigen::Matrix4d invertTransMat(const Eigen::Matrix4d TransMat) {
  Eigen::Matrix4d invTransMat = Eigen::Matrix4d::Zero();
  Eigen::Matrix3d rot = TransMat.block(0, 0, 3, 3);
  Eigen::Vector3d pos = TransMat.col(3).head(3);

  invTransMat.block(0, 0, 3, 3) = rot.transpose();
  invTransMat.block(0, 3, 3, 1) = -rot.transpose() * pos;
  invTransMat(3, 3) = 1;

  return invTransMat;
}

// Constructor
// uPs6::uPs6(string file_path){

// Constructor
uPs6::uPs6(string file_path, std::vector<string> jointnames_spanningtree,
           std::vector<string> jointnames_active) {
  // model parsing (remains unchanged)
  // Model m;

  const char* ext;
  ext = strrchr(file_path.c_str(), '.');
  if (!strcmp(ext, ".urdf") || !strcmp(ext, ".URDF") || !strcmp(ext, ".robot")) {
    cout << "Input file is URDF." << endl;
    if (!Addons::URDFReadFromFileWithModularity(file_path.c_str(), &m, jointnames_spanningtree,
                                                false)) {
      std::cerr << "Error loading robot model from urdf" << std::endl;
      abort();
    }
  } else {
    cout << "Unknown file type: Accepted file types are .urdf or .lua" << endl;
  }

  cout << "Model DoF overview:" << Utils::GetModelDOFOverview(m) << endl;
  cout << "Model Hierarchy overview:" << Utils::GetModelHierarchy(m) << endl;
  cout << "Named Body Origins overview:" << Utils::GetNamedBodyOriginsOverview(m) << endl;

  VectorNd Q(VectorNd::Zero(m.dof_count));
  UpdateKinematicsCustom(m, &Q, NULL, NULL);

  // end of model parsing

  e1_ee = CalcBodyToBaseCoordinates(m, Q, 9., Vector3d(0., 0., 0.), false);
  e2_ee = CalcBodyToBaseCoordinates(m, Q, 12., Vector3d(0., 0., 0.), false);
  e3_ee = CalcBodyToBaseCoordinates(m, Q, 15., Vector3d(0., 0., 0.), false);
  e4_ee = CalcBodyToBaseCoordinates(m, Q, 18., Vector3d(0., 0., 0.), false);
  e5_ee = CalcBodyToBaseCoordinates(m, Q, 21., Vector3d(0., 0., 0.), false);
  e6_ee = CalcBodyToBaseCoordinates(m, Q, 24., Vector3d(0., 0., 0.), false);

  //	Vector3d endeff_pos = CalcBodyToBaseCoordinates (m, Q,
  // m.GetBodyId("hexapod_ee_link"), Vector3d (0., 0., 0.), false);  	// 19
  //--> 24 	Eigen::Matrix3d endeff_rot = CalcBodyWorldOrientation(m, Q,
  // m.GetBodyId("hexapod_ee_link"));
  // End effector link will be body number 6.
  Vector3d endeff_pos =
      CalcBodyToBaseCoordinates(m, Q, 6., Vector3d(0., 0., 0.), false);  // 19 --> 24
  Eigen::Matrix3d endeff_rot = CalcBodyWorldOrientation(m, Q, 6.);
  cout << "endeff coord: " << endeff_pos << endl;
  cout << "endeff rotation: " << endeff_rot << endl;

  Eigen::Matrix4d TransMat_ee = BuildTransMat(endeff_rot, endeff_pos);
  // cout << "TransMat ee" << endl << TransMat_ee << endl << endl;
  // cout << "TransMat pos" << endl << BuildTransVec(e1_ee) << endl << endl;

  // cout << invertTransMat(TransMat_ee) << endl ;

  e1_ee = (invertTransMat(TransMat_ee) * BuildTransVec(e1_ee)).head(3);
  e2_ee = (invertTransMat(TransMat_ee) * BuildTransVec(e2_ee)).head(3);
  e3_ee = (invertTransMat(TransMat_ee) * BuildTransVec(e3_ee)).head(3);
  e4_ee = (invertTransMat(TransMat_ee) * BuildTransVec(e4_ee)).head(3);
  e5_ee = (invertTransMat(TransMat_ee) * BuildTransVec(e5_ee)).head(3);
  e6_ee = (invertTransMat(TransMat_ee) * BuildTransVec(e6_ee)).head(3);

  b1 = CalcBodyToBaseCoordinates(m, Q, 7., Vector3d(0., 0., 0.), false);
  b2 = CalcBodyToBaseCoordinates(m, Q, 10., Vector3d(0., 0., 0.), false);
  b3 = CalcBodyToBaseCoordinates(m, Q, 13., Vector3d(0., 0., 0.), false);
  b4 = CalcBodyToBaseCoordinates(m, Q, 16., Vector3d(0., 0., 0.), false);
  b5 = CalcBodyToBaseCoordinates(m, Q, 19., Vector3d(0., 0., 0.), false);
  b6 = CalcBodyToBaseCoordinates(m, Q, 22., Vector3d(0., 0., 0.), false);

  // printing should show the coordinates of Ujoint_ball

  RotMat_b1 = CalcBodyWorldOrientation(m, Q, 7.);
  RotMat_b2 = CalcBodyWorldOrientation(m, Q, 10.);
  RotMat_b3 = CalcBodyWorldOrientation(m, Q, 13.);
  RotMat_b4 = CalcBodyWorldOrientation(m, Q, 16.);
  RotMat_b5 = CalcBodyWorldOrientation(m, Q, 19.);
  RotMat_b6 = CalcBodyWorldOrientation(m, Q, 22.);

  dof_active = 6;
  dof_spanningtree = m.dof_count;
  cout << dof_spanningtree << endl;

  double x_zero, y_zero, z_zero;
  double roll_zero, pitch_zero, yaw_zero;
  x_zero = TransMat_ee(0, 3);  // x
  y_zero = TransMat_ee(1, 3);  // y
  z_zero = TransMat_ee(2, 3);  // z
  roll_zero = 0;               // roll
  pitch_zero = 0;              // pitch
  yaw_zero = 0;                // yaw

  Matrix3d RotMat_x_zero, RotMat_y_zero, RotMat_z_zero,
      RotMat_zero;  // zero position rotation matrix

  TransMat_zero.setZero();

  RotMat_x_zero << 1, 0, 0, 0, cos(roll_zero), -sin(roll_zero), 0, sin(roll_zero), cos(roll_zero);

  RotMat_y_zero << cos(pitch_zero), 0, sin(pitch_zero), 0, 1, 0, -sin(pitch_zero), 0,
      cos(pitch_zero);

  RotMat_z_zero << cos(yaw_zero), -sin(yaw_zero), 0, sin(yaw_zero), cos(yaw_zero), 0, 0, 0, 1;

  RotMat_zero = RotMat_x_zero * RotMat_y_zero * RotMat_z_zero;
  // cout<<"RotMat: "<<RotMat<<endl;

  roll_zero = 0;
  pitch_zero = 0;
  yaw_zero = 0;

  TransMat_zero.block(0, 0, 3, 3) = RotMat_zero;
  TransMat_zero(0, 3) = x_zero;
  TransMat_zero(1, 3) = y_zero;
  TransMat_zero(2, 3) = z_zero;
  TransMat_zero(3, 3) = 1;

  Q_zero = VectorNd::Zero(dof_spanningtree);
  VectorNd y = VectorNd::Zero(dof_active);

  Q_zero = calc_loopclosure_function(y);
  // cout << "Q_zero:" << endl << Q_zero;

  // cout << "TransMat Zero:" << endl << TransMat_zero << endl;
}

// calc_loopclosure_function(VectorNd y (6x1))
VectorXd uPs6::calc_loopclosure_function(const Math::VectorNd& y) {
  // This function calculates the loop closure functions gamma
  // for the mechanism
  //

  VectorXd Q = VectorNd::Zero(dof_spanningtree);

  Matrix3d RotMat_x_in, RotMat_y_in, RotMat_z_in, RotMat_in;
  Eigen::Matrix4d TransMat_in = Eigen::Matrix4d::Zero();

  double ex, ey, ez;
  double roll_in, pitch_in, yaw_in;
  ex = y(0);        // x
  ey = y(1);        // y
  ez = y(2);        // z
  roll_in = y(3);   // roll
  pitch_in = y(4);  // pitch
  yaw_in = y(5);    // yaw

  RotMat_x_in << 1, 0, 0, 0, cos(roll_in), -sin(roll_in), 0, sin(roll_in), cos(roll_in);

  RotMat_y_in << cos(pitch_in), 0, sin(pitch_in), 0, 1, 0, -sin(pitch_in), 0, cos(pitch_in);

  RotMat_z_in << cos(yaw_in), -sin(yaw_in), 0, sin(yaw_in), cos(yaw_in), 0, 0, 0, 1;

  RotMat_in = RotMat_x_in * RotMat_y_in * RotMat_z_in;

  TransMat_in.block(0, 0, 3, 3) = RotMat_in;
  TransMat_in(0, 3) = ex;
  TransMat_in(1, 3) = ey;
  TransMat_in(2, 3) = ez;
  TransMat_in(3, 3) = 1;

  Eigen::Matrix4d TransMat_abs;
  TransMat_abs = TransMat_zero * TransMat_in;  // TransMat_zero from constructor

  // build 4x1 vectors out of 3x1 vectors
  Vector4d v(0, 0, 0, 1);
  Vector4d e1_f, e2_f, e3_f, e4_f, e5_f, e6_f;

  e1_f = v;
  e2_f = v;
  e3_f = v;
  e4_f = v;
  e5_f = v;
  e6_f = v;

  e1_f.head(3) = e1_ee;
  e2_f.head(3) = e2_ee;
  e3_f.head(3) = e3_ee;
  e4_f.head(3) = e4_ee;
  e5_f.head(3) = e5_ee;
  e6_f.head(3) = e6_ee;

  // express six end effector points in base coordinates
  Vector4d e1 = TransMat_abs * e1_f;
  Vector4d e2 = TransMat_abs * e2_f;
  Vector4d e3 = TransMat_abs * e3_f;
  Vector4d e4 = TransMat_abs * e4_f;
  Vector4d e5 = TransMat_abs * e5_f;
  Vector4d e6 = TransMat_abs * e6_f;

  // calculate 3x1 vectors for leg direction
  Vector3d b1e1 = e1.head(3) - b1;
  Vector3d b2e2 = e2.head(3) - b2;
  Vector3d b3e3 = e3.head(3) - b3;
  Vector3d b4e4 = e4.head(3) - b4;
  Vector3d b5e5 = e5.head(3) - b5;
  Vector3d b6e6 = e6.head(3) - b6;

  // distance norm is calculated when points involved in expressed in same
  // coordinate system
  double l1 = (b1e1).norm();
  double l2 = (b2e2).norm();
  double l3 = (b3e3).norm();
  double l4 = (b4e4).norm();
  double l5 = (b5e5).norm();
  double l6 = (b6e6).norm();

  // transform the UPS leg vector to universal joint frame
  b1e1 = RotMat_b1 * b1e1;
  b2e2 = RotMat_b2 * b2e2;
  b3e3 = RotMat_b3 * b3e3;
  b4e4 = RotMat_b4 * b4e4;
  b5e5 = RotMat_b5 * b5e5;
  b6e6 = RotMat_b6 * b6e6;

  // leg parameters(roll_u, pitch_u, l)
  double roll_u1 = atan2(-b1e1(1), b1e1(2));
  double pitch_u1 = asin(b1e1(0) / l1);

  double roll_u2 = atan2(-b2e2(1), b2e2(2));
  double pitch_u2 = asin(b2e2(0) / l2);

  double roll_u3 = atan2(-b3e3(1), b3e3(2));
  double pitch_u3 = asin(b3e3(0) / l3);

  double roll_u4 = atan2(-b4e4(1), b4e4(2));
  double pitch_u4 = asin(b4e4(0) / l4);

  double roll_u5 = atan2(-b5e5(1), b5e5(2));
  double pitch_u5 = asin(b5e5(0) / l5);

  double roll_u6 = atan2(-b6e6(1), b6e6(2));
  double pitch_u6 = asin(b6e6(0) / l6);

  Q(0) = ex;        // roll
  Q(1) = ey;        // pitch
  Q(2) = ez;        // yaw
  Q(3) = roll_in;   // roll
  Q(4) = pitch_in;  // pitch
  Q(5) = yaw_in;    // yaw
  // leg 1
  Q(6) = roll_u1;   // roll
  Q(7) = pitch_u1;  // pitch
  Q(8) = l1;        // prismatic
  // cout<<"roll_u1 "<<roll_u1<<" pitch_u1 "<<pitch_u1<<" l1 "<<l1<<endl;
  //  leg 2
  Q(9) = roll_u2;    // roll
  Q(10) = pitch_u2;  // pitch
  Q(11) = l2;        // prismatic
  // cout<<"roll_u2 "<<roll_u2<<" pitch_u2 "<<pitch_u2<<" l1 "<<l2<<endl;
  //  leg 3
  Q(12) = roll_u3;   // roll
  Q(13) = pitch_u3;  // pitch
  Q(14) = l3;        // prismatic
  // cout<<"roll_u3 "<<roll_u3<<" pitch_u3 "<<pitch_u3<<" l3 "<<l3<<endl;
  //  leg 4
  Q(15) = roll_u4;   // roll
  Q(16) = pitch_u4;  // pitch
  Q(17) = l4;        // prismatic
  // cout<<"roll_u4 "<<roll_u4<<" pitch_u4 "<<pitch_u4<<" l4 "<<l4<<endl;
  //  leg 5
  Q(18) = roll_u5;   // roll
  Q(19) = pitch_u5;  // pitch
  Q(20) = l5;        // prismatic
  // cout<<"roll_u5 "<<roll_u5<<" pitch_u5 "<<pitch_u5<<" l5 "<<l5<<endl;
  //  leg 6
  Q(21) = roll_u6;   // roll
  Q(22) = pitch_u6;  // pitch
  Q(23) = l6;        // prismatic
  // cout<<"roll_u6 "<<roll_u6<<" pitch_u6 "<<pitch_u6<<" l6 "<<l6<<endl;

  Q = Q - Q_zero;  // wrt to zero position of assembly

  // cout<<"Q wrt zero position: "<<Q.transpose()<<endl;

  return Q;
}

MatrixXd uPs6::calc_loopclosure_Jacobian(const Math::VectorNd& y) {
  // This function calculates the loop closure Jacobian G
  // for the mechanism

  MatrixXd G;

  G.setZero(dof_spanningtree, dof_active);

  // loop closure jacobian for two UPS legs
  MatrixXd G1, G2, G3, G4, G5, G6;
  G1.setZero(3, dof_active);
  G2.setZero(3, dof_active);
  G3.setZero(3, dof_active);
  G4.setZero(3, dof_active);
  G5.setZero(3, dof_active);
  G6.setZero(3, dof_active);

  G1 = compute_loopclosure_Jacobian_UPS6(y, b1, e1_ee, RotMat_b1);
  // cout << "G1" << endl << G1 << endl << endl;

  G2 = compute_loopclosure_Jacobian_UPS6(y, b2, e2_ee, RotMat_b2);
  G3 = compute_loopclosure_Jacobian_UPS6(y, b3, e3_ee, RotMat_b3);
  G4 = compute_loopclosure_Jacobian_UPS6(y, b4, e4_ee, RotMat_b4);
  G5 = compute_loopclosure_Jacobian_UPS6(y, b5, e5_ee, RotMat_b5);
  G6 = compute_loopclosure_Jacobian_UPS6(y, b6, e6_ee, RotMat_b6);

  G.block(0, 0, 6, 6).setIdentity();
  G.block(6, 0, 3, 6) = G1;
  G.block(9, 0, 3, 6) = G2;
  G.block(12, 0, 3, 6) = G3;
  G.block(15, 0, 3, 6) = G4;
  G.block(18, 0, 3, 6) = G5;
  G.block(21, 0, 3, 6) = G6;

  // cout<<"Loop closure jacobian, G: \n"<<G<<endl;

  return G;
}

MatrixXd uPs6::calc_loopclosure_Jacobiand(const Math::VectorNd& y, const Math::VectorNd& ydot) {
  // This function calculates the loop closure Jacobiand Gdot
  // for the mechanism
  //
  //       /   q1(d)  \
	//  Q =  |   q1(d)  |
  //       \    d    /
  //

  MatrixXd Gdot;

  Gdot.setZero(dof_spanningtree, dof_active);

  // loop closure jacobian for two UPS legs
  MatrixXd Gdot1, Gdot2, Gdot3, Gdot4, Gdot5, Gdot6;
  Gdot1.setZero(3, dof_active);
  Gdot2.setZero(3, dof_active);
  Gdot3.setZero(3, dof_active);
  Gdot4.setZero(3, dof_active);
  Gdot5.setZero(3, dof_active);
  Gdot6.setZero(3, dof_active);

  Gdot1 = compute_loopclosure_Jacobiandot_UPS6(y, ydot, b1, e1_ee, RotMat_b1);

  Gdot2 = compute_loopclosure_Jacobiandot_UPS6(y, ydot, b2, e2_ee, RotMat_b2);
  Gdot3 = compute_loopclosure_Jacobiandot_UPS6(y, ydot, b3, e3_ee, RotMat_b3);
  Gdot4 = compute_loopclosure_Jacobiandot_UPS6(y, ydot, b4, e4_ee, RotMat_b4);
  Gdot5 = compute_loopclosure_Jacobiandot_UPS6(y, ydot, b5, e5_ee, RotMat_b5);
  Gdot6 = compute_loopclosure_Jacobiandot_UPS6(y, ydot, b6, e6_ee, RotMat_b6);

  Gdot.block(6, 0, 3, 6) = Gdot1;
  Gdot.block(9, 0, 3, 6) = Gdot2;
  Gdot.block(12, 0, 3, 6) = Gdot3;
  Gdot.block(15, 0, 3, 6) = Gdot4;
  Gdot.block(18, 0, 3, 6) = Gdot5;
  Gdot.block(21, 0, 3, 6) = Gdot6;
  // cout<<"Loop closure jacobiand, Gdot: \n"<<Gdot<<endl;

  return Gdot;
}

VectorXd uPs6::calc_loopclosure_g(const Math::VectorNd& y, const Math::VectorNd& ydot) {
  // This function calculates the loop closure bias acceleration g= Gdot*ydot
  // for the mechanism
  //
  //       /   q1(d)  \
	//  Q =  |   q1(d)  |
  //       \    d    /
  //
  return calc_loopclosure_Jacobiand(y, ydot) * ydot;
}
// compute_loopclosure_Jacobian_UPS6(endeffector coordinates y(6 x 1))
MatrixXd uPs6::compute_loopclosure_Jacobian_UPS6(const Math::VectorNd y, const Math::Vector3d b,
                                                 const Math::Vector3d e_ee,
                                                 const Math::Matrix3d RotMat_b) {
  MatrixXd J;
  J.setZero(3, 6);

  Matrix3d RotMat_x_in, RotMat_y_in, RotMat_z_in, RotMat_in;
  Eigen::Matrix4d TransMat_in = Eigen::Matrix4d::Zero();

  double roll, pitch, yaw;
  roll = y(3);   // roll
  pitch = y(4);  // pitch
  yaw = y(5);    // yaw

  RotMat_x_in << 1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll);

  RotMat_y_in << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch);

  RotMat_z_in << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;

  RotMat_in = RotMat_x_in * RotMat_y_in * RotMat_z_in;

  TransMat_in = BuildTransMat(RotMat_in, Vector3d(y(0), y(1), y(2)));

  Eigen::Matrix4d TransMat_abs;
  TransMat_abs = TransMat_zero * TransMat_in;  // TransMat_zero from constructor

  // build 4x1 vectors out of 3x1 vectors
  Vector4d e_f;

  e_f = BuildTransVec(e_ee);

  // express six end effector points in base coordinates
  Vector4d e = TransMat_abs * e_f;

  // calculate 3x1 vectors for leg direction
  Vector3d be = e.head(3) - b;

  // distance norm is calculated when points involved in expressed in same
  // coordinate system
  double l = (be).norm();

  // transform the UPS leg vector to universal joint frame
  be = RotMat_b * be;

  // leg parameters(roll_u, pitch_u, l)
  double roll_u = atan2(-be(1), be(2));
  double pitch_u = asin(be(0) / l);
  // cout<<"roll_u "<<roll_u<<" pitch_u "<<pitch_u<<" l "<<l<<endl;
  // end for

  double sx, sy, sz, nx, ny, nz, ax, ay,
      az;  // elements of rotation matrix to the s frame
  // fixed params(rotation matrix transformation to universal joint frame)
  sx = RotMat_b(0, 0);
  sy = RotMat_b(1, 0);
  sz = RotMat_b(2, 0);

  nx = RotMat_b(0, 1);
  ny = RotMat_b(1, 1);
  nz = RotMat_b(2, 1);

  ax = RotMat_b(0, 2);
  ay = RotMat_b(1, 2);
  az = RotMat_b(2, 2);

  double ex, ey, ez;
  ex = e_ee(0);  // x
  ey = e_ee(1);  // y
  ez = e_ee(2);  // z

  // inputs to the code below: ex, ey, ez, roll, pitch, yaw, roll_u, pitch_u, l
  double t2 = 1.0 / l;
  double t3 = cos(pitch_u);
  double t4 = 1.0 / t3;
  double t5 = cos(roll_u);
  double t6 = sin(roll_u);
  double t7 = cos(yaw);
  double t8 = sin(roll);
  double t9 = cos(roll);
  double t10 = sin(pitch);
  double t11 = sin(yaw);
  double t12 = cos(pitch);
  double t13 = t8 * t11;
  double t39 = t7 * t9 * t10;
  double t14 = t13 - t39;
  double t15 = ex * t14;
  double t16 = t7 * t8;
  double t17 = t9 * t10 * t11;
  double t18 = t16 + t17;
  double t19 = ey * t18;
  double t20 = ez * t9 * t12;
  double t21 = t15 + t19 + t20;
  double t22 = t9 * t11;
  double t23 = t7 * t8 * t10;
  double t24 = t22 + t23;
  double t25 = ex * t24;
  double t26 = t7 * t9;
  double t40 = t8 * t10 * t11;
  double t27 = t26 - t40;
  double t28 = ey * t27;
  double t49 = ez * t8 * t12;
  double t29 = t25 + t28 - t49;
  double t30 = ez * t9 * t10;
  double t31 = ex * t7 * t9 * t12;
  double t53 = ey * t9 * t11 * t12;
  double t32 = t30 + t31 - t53;
  double t33 = ez * t8 * t10;
  double t34 = ex * t7 * t8 * t12;
  double t54 = ey * t8 * t11 * t12;
  double t35 = t33 + t34 - t54;
  double t36 = ez * t12;
  double t37 = ey * t10 * t11;
  double t55 = ex * t7 * t10;
  double t38 = t36 + t37 - t55;
  double t41 = ey * t7 * t12;
  double t42 = ex * t11 * t12;
  double t43 = t41 + t42;
  double t44 = ex * t18;
  double t62 = ey * t14;
  double t45 = t44 - t62;
  double t46 = ex * t27;
  double t63 = ey * t24;
  double t47 = t46 - t63;
  double t48 = sin(pitch_u);
  double t50 = az * t29;
  double t71 = nz * t21;
  double t51 = t50 - t71;
  double t52 = ay * t29;
  double t56 = ny * t35;
  double t57 = sy * t38;
  double t77 = ay * t32;
  double t58 = t56 + t57 - t77;
  double t59 = nz * t35;
  double t60 = sz * t38;
  double t76 = az * t32;
  double t61 = t59 + t60 - t76;
  double t64 = ay * t45;
  double t65 = ny * t47;
  double t82 = sy * t43;
  double t66 = t64 + t65 - t82;
  double t67 = az * t45;
  double t68 = nz * t47;
  double t81 = sz * t43;
  double t69 = t67 + t68 - t81;
  double t70 = ax * t29;
  double t72 = t52 - ny * t21;
  double t73 = nx * t35;
  double t74 = sx * t38;
  double t75 = t73 + t74 - ax * t32;
  double t78 = ax * t45;
  double t79 = nx * t47;
  double t80 = t78 + t79 - sx * t43;

  J(0, 0) = -sy * t2 * t4 * t5 - sz * t2 * t4 * t6;
  J(0, 1) = -ny * t2 * t4 * t5 - nz * t2 * t4 * t6;
  J(0, 2) = -ay * t2 * t4 * t5 - az * t2 * t4 * t6;
  J(0, 3) = -t2 * t4 * t6 * t51 - t2 * t4 * t5 * t72;
  J(0, 4) = -t2 * t4 * t5 * t58 - t2 * t4 * t6 * t61;
  J(0, 5) = -t2 * t4 * t5 * t66 - t2 * t4 * t6 * t69;
  J(1, 0) = sx * t2 * t3 + sy * t2 * t6 * t48 - sz * t2 * t5 * t48;
  J(1, 1) = nx * t2 * t3 + ny * t2 * t6 * t48 - nz * t2 * t5 * t48;
  J(1, 2) = ax * t2 * t3 + ay * t2 * t6 * t48 - az * t2 * t5 * t48;
  J(1, 3) = t2 * t3 * (t70 - nx * t21) - t2 * t5 * t48 * t51 + t2 * t6 * t48 * t72;
  J(1, 4) = t2 * t3 * t75 + t2 * t6 * t48 * t58 - t2 * t5 * t48 * t61;
  J(1, 5) = t2 * t3 * t80 + t2 * t6 * t48 * t66 - t2 * t5 * t48 * t69;
  J(2, 0) = sx * t48 - sy * t3 * t6 + sz * t3 * t5;
  J(2, 1) = nx * t48 - ny * t3 * t6 + nz * t3 * t5;
  J(2, 2) = ax * t48 - ay * t3 * t6 + az * t3 * t5;
  J(2, 3) = t48 * (t70 - nx * t21) - t3 * t6 * t72 + t3 * t5 * (t50 - t71);
  J(2, 4) = t48 * t75 - t3 * t6 * t58 + t3 * t5 * t61;
  J(2, 5) = t48 * t80 - t3 * t6 * t66 + t3 * t5 * t69;

  return J;
}

MatrixXd uPs6::compute_loopclosure_Jacobiandot_UPS6(const Math::VectorNd y,
                                                    const Math::VectorNd ydot,
                                                    const Math::Vector3d b,
                                                    const Math::Vector3d e_ee,
                                                    const Math::Matrix3d RotMat_b) {
  MatrixXd J;
  J.setZero(3, dof_active);

  Matrix3d RotMat_x_in, RotMat_y_in, RotMat_z_in, RotMat_in;
  Eigen::Matrix4d TransMat_in = Eigen::Matrix4d::Zero();

  double roll, pitch, yaw;
  roll = y(3);   // roll
  pitch = y(4);  // pitch
  yaw = y(5);    // yaw

  double roll_dot, pitch_dot, yaw_dot;
  roll_dot = ydot(3);   // roll
  pitch_dot = ydot(4);  // pitch
  yaw_dot = ydot(5);    // yaw

  RotMat_x_in << 1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll);

  RotMat_y_in << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch);

  RotMat_z_in << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;

  RotMat_in = RotMat_x_in * RotMat_y_in * RotMat_z_in;

  TransMat_in = BuildTransMat(RotMat_in, Vector3d(y(0), y(1), y(2)));

  Eigen::Matrix4d TransMat_abs;
  TransMat_abs = TransMat_zero * TransMat_in;  // TransMat_zero from constructor

  // build 4x1 vectors out of 3x1 vectors
  Vector4d e_f;

  e_f = BuildTransVec(e_ee);

  // express six end effector points in base coordinates
  Vector4d e = TransMat_abs * e_f;

  // calculate 3x1 vectors for leg direction
  Vector3d be = e.head(3) - b;

  // distance norm is calculated when points involved in expressed in same
  // coordinate system
  double l = (be).norm();

  // transform the UPS leg vector to universal joint frame
  be = RotMat_b * be;

  // leg parameters(roll_u, pitch_u, l)
  double roll_u = atan2(-be(1), be(2));
  double pitch_u = asin(be(0) / l);
  // cout<<"roll_u "<<roll_u<<" pitch_u "<<pitch_u<<" l "<<l<<endl;
  // end for

  VectorXd qlegdot(3);
  qlegdot = compute_loopclosure_Jacobian_UPS6(y, b, e_ee, RotMat_b) * ydot;
  double roll_u_dot = qlegdot(0);
  double pitch_u_dot = qlegdot(1);
  double l_dot = qlegdot(2);

  double sx, sy, sz, nx, ny, nz, ax, ay,
      az;  // elements of rotation matrix to the s frame
  // fixed params(rotation matrix transformation to universal joint frame)
  sx = RotMat_b(0, 0);
  sy = RotMat_b(1, 0);
  sz = RotMat_b(2, 0);

  nx = RotMat_b(0, 1);
  ny = RotMat_b(1, 1);
  nz = RotMat_b(2, 1);

  ax = RotMat_b(0, 2);
  ay = RotMat_b(1, 2);
  az = RotMat_b(2, 2);

  double ex, ey, ez;
  ex = e_ee(0);  // x
  ey = e_ee(1);  // y
  ez = e_ee(2);  // z

  // cout << "Rotation Matrix input: " << endl << RotMat_in << endl;
  // cout << "Translation Vector input: " << endl << ex << endl << ey << endl <<
  // ez << endl; cout<<"roll_d "<<roll_dot<<" pitch_d "<<pitch_dot<<" yaw_d
  // "<<yaw_dot<<endl;

  double t2 = cos(pitch_u);
  double t3 = 1.0 / t2;
  double t4 = cos(roll_u);
  double t5 = sin(roll_u);
  double t6 = 1.0 / l;
  double t7 = sy * t4;
  double t8 = sz * t5;
  double t9 = t7 + t8;
  double t10 = 1.0 / (l * l);
  double t11 = 1.0 / (t2 * t2);
  double t12 = sin(pitch_u);
  double t13 = ny * t4;
  double t14 = nz * t5;
  double t15 = t13 + t14;
  double t16 = ay * t4;
  double t17 = az * t5;
  double t18 = t16 + t17;
  double t19 = cos(roll);
  double t20 = cos(yaw);
  double t21 = sin(roll);
  double t22 = sin(pitch);
  double t23 = sin(yaw);
  double t24 = cos(pitch);
  double t25 = roll_dot * t19 * t23;
  double t26 = t20 * t21 * yaw_dot;
  double t27 = roll_dot * t20 * t21 * t22;
  double t28 = t19 * t22 * t23 * yaw_dot;
  double t99 = pitch_dot * t19 * t20 * t24;
  double t29 = t25 + t26 + t27 + t28 - t99;
  double t30 = ex * t29;
  double t31 = roll_dot * t19 * t20;
  double t32 = pitch_dot * t19 * t23 * t24;
  double t33 = t19 * t20 * t22 * yaw_dot;
  double t100 = t21 * t23 * yaw_dot;
  double t101 = roll_dot * t21 * t22 * t23;
  double t34 = t31 + t32 + t33 - t100 - t101;
  double t35 = ey * t34;
  double t130 = ez * pitch_dot * t19 * t22;
  double t131 = ez * roll_dot * t21 * t24;
  double t36 = t30 + t35 - t130 - t131;
  double t37 = t19 * t20 * yaw_dot;
  double t38 = pitch_dot * t20 * t21 * t24;
  double t39 = roll_dot * t19 * t20 * t22;
  double t102 = roll_dot * t21 * t23;
  double t103 = t21 * t22 * t23 * yaw_dot;
  double t40 = t37 + t38 + t39 - t102 - t103;
  double t41 = roll_dot * t20 * t21;
  double t42 = t19 * t23 * yaw_dot;
  double t43 = pitch_dot * t21 * t23 * t24;
  double t44 = roll_dot * t19 * t22 * t23;
  double t45 = t20 * t21 * t22 * yaw_dot;
  double t46 = t41 + t42 + t43 + t44 + t45;
  double t47 = ez * pitch_dot * t21 * t22;
  double t132 = ex * t40;
  double t133 = ey * t46;
  double t134 = ez * roll_dot * t19 * t24;
  double t48 = t47 + t132 - t133 - t134;
  double t49 = t21 * t23;
  double t66 = t19 * t20 * t22;
  double t50 = t49 - t66;
  double t51 = ex * t50;
  double t52 = t20 * t21;
  double t53 = t19 * t22 * t23;
  double t54 = t52 + t53;
  double t55 = ey * t54;
  double t56 = ez * t19 * t24;
  double t57 = t51 + t55 + t56;
  double t58 = t19 * t23;
  double t59 = t20 * t21 * t22;
  double t60 = t58 + t59;
  double t61 = ex * t60;
  double t62 = t19 * t20;
  double t67 = t21 * t22 * t23;
  double t63 = t62 - t67;
  double t64 = ey * t63;
  double t68 = ez * t21 * t24;
  double t65 = t61 + t64 - t68;
  double t69 = az * t65;
  double t70 = ay * t65;
  double t141 = ny * t57;
  double t71 = t70 - t141;
  double t140 = nz * t57;
  double t72 = t69 - t140;
  double t73 = ey * t20 * t24 * yaw_dot;
  double t74 = ex * pitch_dot * t20 * t22;
  double t75 = ex * t23 * t24 * yaw_dot;
  double t84 = ez * pitch_dot * t24;
  double t85 = ey * pitch_dot * t22 * t23;
  double t76 = t73 + t74 + t75 - t84 - t85;
  double t77 = ez * t22;
  double t78 = ex * t20 * t24;
  double t86 = ey * t23 * t24;
  double t79 = t77 + t78 - t86;
  double t80 = ex * t22 * t23 * yaw_dot;
  double t81 = ey * pitch_dot * t23 * t24;
  double t82 = ey * t20 * t22 * yaw_dot;
  double t142 = ez * pitch_dot * t22;
  double t143 = ex * pitch_dot * t20 * t24;
  double t83 = t80 + t81 + t82 - t142 - t143;
  double t87 = ez * t24;
  double t88 = ey * t22 * t23;
  double t90 = ex * t20 * t22;
  double t89 = t87 + t88 - t90;
  double t91 = sz * t89;
  double t92 = nz * t21 * t79;
  double t98 = az * t19 * t79;
  double t93 = t91 + t92 - t98;
  double t94 = sy * t89;
  double t95 = ny * t21 * t79;
  double t97 = ay * t19 * t79;
  double t96 = t94 + t95 - t97;
  double t104 = ex * pitch_dot * t22 * t23;
  double t105 = ey * pitch_dot * t20 * t22;
  double t106 = ey * t23 * t24 * yaw_dot;
  double t157 = ex * t20 * t24 * yaw_dot;
  double t107 = t104 + t105 + t106 - t157;
  double t108 = ex * t34;
  double t158 = ey * t29;
  double t109 = t108 - t158;
  double t110 = ey * t40;
  double t111 = ex * t46;
  double t112 = t110 + t111;
  double t113 = ey * t20 * t24;
  double t114 = ex * t23 * t24;
  double t115 = t113 + t114;
  double t116 = ex * t54;
  double t120 = ey * t50;
  double t117 = t116 - t120;
  double t118 = ex * t63;
  double t122 = ey * t60;
  double t119 = t118 - t122;
  double t121 = az * t117;
  double t123 = nz * t119;
  double t129 = sz * t115;
  double t124 = t121 + t123 - t129;
  double t125 = ay * t117;
  double t126 = ny * t119;
  double t128 = sy * t115;
  double t127 = t125 + t126 - t128;
  double t135 = az * t48;
  double t168 = nz * t36;
  double t136 = t135 - t168;
  double t137 = ay * t48;
  double t138 = ax * t65;
  double t170 = nx * t57;
  double t139 = t138 - t170;
  double t144 = sx * t89;
  double t145 = nx * t21 * t79;
  double t176 = ax * t19 * t79;
  double t146 = t144 + t145 - t176;
  double t147 = sz * t83;
  double t148 = az * t19 * t76;
  double t149 = nz * roll_dot * t19 * t79;
  double t150 = az * roll_dot * t21 * t79;
  double t177 = nz * t21 * t76;
  double t151 = t147 + t148 + t149 + t150 - t177;
  double t152 = sy * t83;
  double t153 = ay * t19 * t76;
  double t154 = ny * roll_dot * t19 * t79;
  double t155 = ay * roll_dot * t21 * t79;
  double t178 = ny * t21 * t76;
  double t156 = t152 + t153 + t154 + t155 - t178;
  double t159 = sz * t107;
  double t160 = az * t109;
  double t181 = nz * t112;
  double t161 = t159 + t160 - t181;
  double t162 = sy * t107;
  double t163 = ay * t109;
  double t164 = ax * t117;
  double t165 = nx * t119;
  double t183 = sx * t115;
  double t166 = t164 + t165 - t183;
  double t167 = ax * t48;
  double t169 = t137 - ny * t36;
  double t171 = sx * t83;
  double t172 = ax * t19 * t76;
  double t173 = nx * roll_dot * t19 * t79;
  double t174 = ax * roll_dot * t21 * t79;
  double t175 = t171 + t172 + t173 + t174 - nx * t21 * t76;
  double t179 = sx * t107;
  double t180 = ax * t109;
  double t182 = t162 + t163 - ny * t112;
  J(0, 0) = t3 * t6 * (roll_u_dot * sy * t5 - roll_u_dot * sz * t4) + l_dot * t3 * t9 * t10 -
            pitch_u_dot * t6 * t9 * t11 * t12;
  J(0, 1) = t3 * t6 * (ny * roll_u_dot * t5 - nz * roll_u_dot * t4) + l_dot * t3 * t10 * t15 -
            pitch_u_dot * t6 * t11 * t12 * t15;
  J(0, 2) = t3 * t6 * (ay * roll_u_dot * t5 - az * roll_u_dot * t4) + l_dot * t3 * t10 * t18 -
            pitch_u_dot * t6 * t11 * t12 * t18;
  J(0, 3) = -t3 * t5 * t6 * t136 - t3 * t4 * t6 * t169 + l_dot * t3 * t4 * t10 * t71 +
            l_dot * t3 * t5 * t10 * t72 - roll_u_dot * t3 * t4 * t6 * t72 +
            roll_u_dot * t3 * t5 * t6 * t71 - pitch_u_dot * t4 * t6 * t11 * t12 * t71 -
            pitch_u_dot * t5 * t6 * t11 * t12 * t72;
  J(0, 4) = -t3 * t5 * t6 * t151 - t3 * t4 * t6 * t156 + l_dot * t3 * t5 * t10 * t93 +
            l_dot * t3 * t4 * t10 * t96 - roll_u_dot * t3 * t4 * t6 * t93 +
            roll_u_dot * t3 * t5 * t6 * t96 - pitch_u_dot * t5 * t6 * t11 * t12 * t93 -
            pitch_u_dot * t4 * t6 * t11 * t12 * t96;
  J(0, 5) = -t3 * t5 * t6 * t161 - t3 * t4 * t6 * t182 + l_dot * t3 * t5 * t10 * t124 +
            l_dot * t3 * t4 * t10 * t127 - roll_u_dot * t3 * t4 * t6 * t124 +
            roll_u_dot * t3 * t5 * t6 * t127 - pitch_u_dot * t5 * t6 * t11 * t12 * t124 -
            pitch_u_dot * t4 * t6 * t11 * t12 * t127;
  J(1, 0) =
      t6 * (-pitch_u_dot * sx * t12 + pitch_u_dot * sy * t2 * t5 - pitch_u_dot * sz * t2 * t4 +
            roll_u_dot * sy * t4 * t12 + roll_u_dot * sz * t5 * t12) -
      l_dot * t10 * (sx * t2 + sy * t5 * t12 - sz * t4 * t12);
  J(1, 1) =
      t6 * (-nx * pitch_u_dot * t12 + ny * pitch_u_dot * t2 * t5 - nz * pitch_u_dot * t2 * t4 +
            ny * roll_u_dot * t4 * t12 + nz * roll_u_dot * t5 * t12) -
      l_dot * t10 * (nx * t2 + ny * t5 * t12 - nz * t4 * t12);
  J(1, 2) =
      t6 * (-ax * pitch_u_dot * t12 + ay * pitch_u_dot * t2 * t5 - az * pitch_u_dot * t2 * t4 +
            ay * roll_u_dot * t4 * t12 + az * roll_u_dot * t5 * t12) -
      l_dot * t10 * (ax * t2 + ay * t5 * t12 - az * t4 * t12);
  J(1, 3) = t2 * t6 * (t167 - nx * t36) - l_dot * t2 * t10 * t139 - pitch_u_dot * t6 * t12 * t139 -
            t4 * t6 * t12 * t136 + t5 * t6 * t12 * t169 - l_dot * t5 * t10 * t12 * t71 -
            pitch_u_dot * t2 * t4 * t6 * t72 + l_dot * t4 * t10 * t12 * (t69 - t140) +
            pitch_u_dot * t2 * t5 * t6 * (t70 - t141) + roll_u_dot * t5 * t6 * t12 * (t69 - t140) +
            roll_u_dot * t4 * t6 * t12 * (t70 - t141);
  J(1, 4) = t2 * t6 * t175 - l_dot * t2 * t10 * t146 - pitch_u_dot * t6 * t12 * t146 -
            t4 * t6 * t12 * t151 + t5 * t6 * t12 * t156 + l_dot * t4 * t10 * t12 * t93 -
            l_dot * t5 * t10 * t12 * t96 - pitch_u_dot * t2 * t4 * t6 * t93 +
            pitch_u_dot * t2 * t5 * t6 * t96 + roll_u_dot * t5 * t6 * t12 * t93 +
            roll_u_dot * t4 * t6 * t12 * t96;
  J(1, 5) = t2 * t6 * (t179 + t180 - nx * t112) - l_dot * t2 * t10 * t166 -
            pitch_u_dot * t6 * t12 * t166 - t4 * t6 * t12 * t161 + t5 * t6 * t12 * t182 +
            l_dot * t4 * t10 * t12 * t124 - l_dot * t5 * t10 * t12 * t127 -
            pitch_u_dot * t2 * t4 * t6 * t124 + pitch_u_dot * t2 * t5 * t6 * t127 +
            roll_u_dot * t5 * t6 * t12 * t124 + roll_u_dot * t4 * t6 * t12 * t127;
  J(2, 0) = pitch_u_dot * sx * t2 + pitch_u_dot * sy * t5 * t12 - pitch_u_dot * sz * t4 * t12 -
            roll_u_dot * sy * t2 * t4 - roll_u_dot * sz * t2 * t5;
  J(2, 1) = nx * pitch_u_dot * t2 + ny * pitch_u_dot * t5 * t12 - nz * pitch_u_dot * t4 * t12 -
            ny * roll_u_dot * t2 * t4 - nz * roll_u_dot * t2 * t5;
  J(2, 2) = ax * pitch_u_dot * t2 + ay * pitch_u_dot * t5 * t12 - az * pitch_u_dot * t4 * t12 -
            ay * roll_u_dot * t2 * t4 - az * roll_u_dot * t2 * t5;
  J(2, 3) = t12 * (t167 - nx * t36) - t2 * t5 * t169 + pitch_u_dot * t2 * (t138 - t170) +
            t2 * t4 * (t135 - t168) - pitch_u_dot * t4 * t12 * t72 - roll_u_dot * t2 * t4 * t71 -
            roll_u_dot * t2 * t5 * t72 + pitch_u_dot * t5 * t12 * (t70 - t141);
  J(2, 4) = t12 * t175 + pitch_u_dot * t2 * t146 + t2 * t4 * t151 - t2 * t5 * t156 -
            pitch_u_dot * t4 * t12 * t93 + pitch_u_dot * t5 * t12 * t96 -
            roll_u_dot * t2 * t5 * t93 - roll_u_dot * t2 * t4 * t96;
  J(2, 5) = t12 * (t179 + t180 - nx * t112) + t2 * t4 * (t159 + t160 - t181) +
            pitch_u_dot * t2 * t166 - t2 * t5 * t182 - pitch_u_dot * t4 * t12 * t124 +
            pitch_u_dot * t5 * t12 * t127 - roll_u_dot * t2 * t5 * t124 -
            roll_u_dot * t2 * t4 * t127;

  return J;
}

}  // namespace UPS6
