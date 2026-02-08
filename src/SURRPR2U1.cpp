/*
 * HyRoDyn - Hybrid Robot Dynamics
 * Copyright (c) 2018-2020 Shivesh Kumar <shivesh.kumar@dfki.de>
 * This submechanism library is contributed by Shivesh Kumar/Christoph Stoeffler
 * <shivesh.kumar@dfki.de, christoph.stoeffler@dfki.de>. Licensed under the zlib
 * license. See LICENSE for more details.
 */

#include "SURRPR2U1.hpp"

namespace SURRPR2U1 {

// Constructor
surrPr2u1::surrPr2u1(string file_path, std::vector<string> jointnames_spanningtree,
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
  } else {
    std::cerr << "Unknown file type: Accepted file types are .urdf or .lua" << endl;
    abort();
  }

  assert(dof_independent == jointnames_independent.size());
  assert(dof_spanningtree == m.dof_count);

  // Put the mechanism in its zero configuration
  VectorNd Q(VectorNd::Zero(m.dof_count));
  UpdateKinematicsCustom(m, &Q, NULL, NULL);

  // Extract the physical parameters of the mechanism from the RBDL model

  // end effector points
  Vector3d e1_ee = CalcBodyToBaseCoordinates(m, Q, 3, Vector3d(0., 0., 0.), false);
  Vector3d e2_ee = CalcBodyToBaseCoordinates(m, Q, 7, Vector3d(0., 0., 0.), false);
  cout << "e1_ee: " << e1_ee.transpose() << ", e2_ee: " << e2_ee.transpose() << endl;

  // base points
  unsigned int body_id_b1 = m.GetBodyId("ALWrist_AL1_Link");
  unsigned int body_id_b2 = m.GetBodyId("ALWrist_AR1_Link");
  Vector3d b1 = CalcBodyToBaseCoordinates(m, Q, 11, Vector3d(0., 0., 0.), false);
  Vector3d b2 = CalcBodyToBaseCoordinates(m, Q, 13, Vector3d(0., 0., 0.), false);
  cout << "b1: " << b1.transpose() << ", b2: " << b2.transpose() << endl;
  cout << "BID b1: " << body_id_b1 << ", BID b2: " << body_id_b2 << endl;

  // crank points
  unsigned int body_id_c1 = m.GetBodyId("ALWrist_BL1_Link");
  unsigned int body_id_c2 = m.GetBodyId("ALWrist_BR1_Link");
  Vector3d c1 = CalcBodyToBaseCoordinates(m, Q, 15., Vector3d(0., 0., 0.), false);
  Vector3d c2 = CalcBodyToBaseCoordinates(m, Q, 16., Vector3d(0., 0., 0.), false);
  cout << "c1: " << c1.transpose() << ", c2: " << c2.transpose() << endl;
  cout << "BID c1: " << body_id_c1 << ", BID c2: " << body_id_c2 << endl;

  // attachment points
  unsigned int body_id_k1 = m.GetBodyId("ALWrist_ActL_Link");
  unsigned int body_id_k2 = m.GetBodyId("ALWrist_ActR_Link");
  Vector3d k1_zero = CalcBodyToBaseCoordinates(m, Q, 12., Vector3d(0., 0., 0.), false);
  Vector3d k2_zero = CalcBodyToBaseCoordinates(m, Q, 14., Vector3d(0., 0., 0.), false);
  cout << "k1_zero: " << k1_zero.transpose() << ", k2_zero: " << k2_zero.transpose() << endl;
  cout << "BID k1: " << body_id_k1 << ", BID k2: " << body_id_k2 << endl;
  /*
  for(uint i = 0; i <= m.dof_count; i++){
          cout<<"i = "<<i<<", Body Name: "<<m.GetBodyName(i)<<endl;
  }
  */
  // Call the constructor which initializes the submechanism model directly with
  // the physical parameters
  // surrPr2u1(b1, b2, c1, c2, e1_ee, e2_ee, k1_zero, k2_zero);

  b1x = b1(0);
  b1y = b1(1);
  b1z = b1(2);

  c1x = c1(0);
  c1y = c1(1);
  c1z = c1(2);

  e1x = e1_ee(0);
  e1y = e1_ee(1);
  e1z = e1_ee(2);

  k1x = k1_zero(0);
  k1y = k1_zero(1);
  k1z = k1_zero(2);

  b2x = b2(0);
  b2y = b2(1);
  b2z = b2(2);

  c2x = c2(0);
  c2y = c2(1);
  c2z = c2(2);

  e2x = e2_ee(0);
  e2y = e2_ee(1);
  e2z = e2_ee(2);

  k2x = k2_zero(0);
  k2y = k2_zero(1);
  k2z = k2_zero(2);

  Vector3d c1k1 = c1 - k1_zero;
  Vector3d c2k2 = c2 - k2_zero;

  l1 = (e1_ee - k1_zero).norm();
  l2 = (e2_ee - k2_zero).norm();
  cout << "l1 = " << l1 << " l2 = " << l2 << endl;

  r1 = c1k1.norm();
  r2 = c2k2.norm();
  cout << "r1 = " << r1 << " r2 = " << r2 << endl;

  cout << "d1_zero = " << (b1 - k1_zero).norm() << " d2_zero = " << (b2 - k2_zero).norm() << endl;

  // Normalize the zero vector of the crank ciki
  c1k1 = c1k1 / r1;
  c2k2 = c2k2 / r2;

  s1x = c1k1(0);
  s1z = c1k1(2);

  s2x = c2k2(0);
  s2z = c2k2(2);

  Vector3d c1b1 = c1 - b1;
  Vector3d c2b2 = c2 - b2;

  c1b1 = c1b1 / c1b1.norm();
  c2b2 = c2b2 / c2b2.norm();

  Vector3d n1 = c1b1.cross(c1k1);
  Vector3d n2 = c2b2.cross(c2k2);
  n1 = n1 / n1.norm();
  n2 = n2 / n2.norm();

  n1x = n1(0);
  n1y = n1(1);
  n1z = n1(2);
  n2x = n2(0);
  n2y = n2(1);
  n2z = n2(2);

  cout << "n1 vector: " << n1 << endl;
  cout << "n2 vector: " << n2 << endl;

  h1 = 0.0;
  h2 = 0.0;

  // Comment the line below in case you derived the analytical solutions in
  // relative coordinates.
  Q_zero = VectorNd::Zero(m.dof_count);
  VectorNd y(2);
  y(0) = 0.0;
  y(1) = 0.0;
  Q_zero = calc_loopclosure_function(y);
  // cout<<"Q_zero: "<<Q_zero.transpose()<<endl;
}

void surrPr2u1::print_physical_parameters() {
  cout << "Physical Parameters of the mechanism are: " << endl;

  cout << "e1x: " << e1x << ", e1y: " << e1y << ", e1z: " << e1z << endl;
  cout << "e2x: " << e2x << ", e2y: " << e2y << ", e2z: " << e2z << endl;

  cout << "b1x: " << b1x << ", b1y: " << b1y << ", b1z: " << b1z << endl;
  cout << "b2x: " << b2x << ", b2y: " << b2y << ", b2z: " << b2z << endl;

  cout << "c1x: " << c1x << ", c1y: " << c1y << ", c1z: " << c1z << endl;
  cout << "c2x: " << c2x << ", c2y: " << c2y << ", c2z: " << c2z << endl;

  cout << "e1x: " << e1x << ", e1y: " << e1y << ", e1z: " << e1z << endl;
  cout << "e2x: " << e2x << ", e2y: " << e2y << ", e2z: " << e2z << endl;

  cout << "n1x: " << n1x << ", n1y: " << n1y << ", n1z: " << n1z << endl;
  cout << "n2x: " << n2x << ", n2y: " << n2y << ", n2z: " << n2z << endl;

  cout << "r1: " << r1 << ", l1: " << l1 << ", h1: " << h1 << endl;
  cout << "r2: " << r2 << ", l2: " << l2 << ", h2: " << h2 << endl;
}
// Constructor from the physical parameters of the robot
surrPr2u1::surrPr2u1(Vector3d b1, Vector3d b2, Vector3d c1, Vector3d c2, Vector3d e1_ee,
                     Vector3d e2_ee, Vector3d k1_zero, Vector3d k2_zero) {
  b1x = b1(0);
  b1y = b1(1);
  b1z = b1(2);

  c1x = c1(0);
  c1y = c1(1);
  c1z = c1(2);

  e1x = e1_ee(0);
  e1y = e1_ee(1);
  e1z = e1_ee(2);

  k1x = k1_zero(0);
  k1y = k1_zero(1);
  k1z = k1_zero(2);

  b2x = b2(0);
  b2y = b2(1);
  b2z = b2(2);

  c2x = c2(0);
  c2y = c2(1);
  c2z = c2(2);

  e2x = e2_ee(0);
  e2y = e2_ee(1);
  e2z = e2_ee(2);

  k2x = k2_zero(0);
  k2y = k2_zero(1);
  k2z = k2_zero(2);

  Vector3d c1k1 = c1 - k1_zero;
  Vector3d c2k2 = c2 - k2_zero;

  l1 = (e1_ee - k1_zero).norm();
  l2 = (e2_ee - k2_zero).norm();
  cout << "l1 = " << l1 << " l2 = " << l2 << endl;

  r1 = c1k1.norm();
  r2 = c2k2.norm();
  cout << "r1 = " << r1 << " r2 = " << r2 << endl;

  cout << "d1_zero = " << (b1 - k1_zero).norm() << " d2_zero = " << (b2 - k2_zero).norm() << endl;

  // Normalize the zero vector of the crank ciki
  c1k1 = c1k1 / r1;
  c2k2 = c2k2 / r2;

  s1x = c1k1(0);
  s1z = c1k1(2);

  s2x = c2k2(0);
  s2z = c2k2(2);

  Vector3d c1b1 = c1 - b1;
  Vector3d c2b2 = c2 - b2;

  c1b1 = c1b1 / c1b1.norm();
  c2b2 = c2b2 / c2b2.norm();

  Vector3d n1 = c1b1.cross(c1k1);
  Vector3d n2 = c2b2.cross(c2k2);
  n1 = n1 / n1.norm();
  n2 = n2 / n2.norm();

  n1x = n1(0);
  n1y = n1(1);
  n1z = n1(2);
  n2x = n2(0);
  n2y = n2(1);
  n2z = n2(2);

  cout << "n1 vector: " << n1 << endl;
  cout << "n2 vector: " << n2 << endl;

  h1 = 0.0;
  h2 = 0.0;

  print_physical_parameters();
}

VectorXd surrPr2u1::calc_loopclosure_function(const Math::VectorNd& y) {
  // This function calculates the loop closure function Gamma which is a vector
  // of size (n x 1).

  VectorXd Q = VectorNd::Zero(dof_spanningtree);

  // Put your symbolically generated code here!

  // independent joints via identity mapping
  Q(0) = y(0);
  Q(1) = y(1);

  // active joints via symbolic inverse geometric model generated in MATLAB
  VectorXd u = calc_geometricmodel_inverse(y);
  Q(11) = u(0);
  Q(13) = u(1);

  // Comment the line below in case you derived the analytical solutions in
  // relative coordinates.
  Q = Q - Q_zero;

  return Q;
}

MatrixXd surrPr2u1::calc_loopclosure_Jacobian(const Math::VectorNd& y) {
  // This function calculates the loop closure Jacobian matrix G which is a
  // matrix of size (n x m).

  MatrixXd G;
  G.setZero(dof_spanningtree, dof_independent);

  // Put your symbolically generated code here!

  // independent joints via identity mapping
  G(0, 0) = 1.0;
  G(1, 1) = 1.0;

  // active joints via kinematic jacobian
  Matrix2d Gu = compute_kinematic_Jacobian(y);
  // for row correspoding to roll DOF
  G(11, 0) = Gu(0, 0);
  G(11, 1) = Gu(0, 1);
  // for row corresponding to pitch DOF
  G(13, 0) = Gu(1, 0);
  G(13, 1) = Gu(1, 1);

  return G;
}

MatrixXd surrPr2u1::calc_loopclosure_Jacobiand(const Math::VectorNd& y,
                                               const Math::VectorNd& ydot) {
  // This function calculates the 1st order time derivative of loop closure
  // Jacobian Gdot which is a matrix of size (n x m).

  MatrixXd Gdot(dof_spanningtree, dof_independent);
  Gdot.setZero(dof_spanningtree, dof_independent);

  // Put your symbolically generated code here!

  return Gdot;
}

VectorXd surrPr2u1::calc_loopclosure_g(const Math::VectorNd& y, const Math::VectorNd& ydot) {
  // This function calculates the loop closure bias acceleration g= Gdot*ydot

  return calc_loopclosure_Jacobiand(y, ydot) * ydot;
}

VectorXd surrPr2u1::calc_geometricmodel_forward(const Math::VectorNd u, unsigned int max_iterations,
                                                double step_tol) {
  VectorNd y(VectorNd::Zero(dof_active));
  VectorNd u_i(VectorNd::Zero(dof_active));
  VectorNd du(VectorNd::Zero(dof_active));
  VectorNd dy(VectorNd::Zero(dof_active));
  Matrix2d J = Matrix2d::Zero();

  unsigned int i = 0;

  while (1) {
    u_i = surrPr2u1::calc_geometricmodel_inverse(y);

    du = u - u_i;

    if (du.norm() < step_tol)
      break;

    J = surrPr2u1::compute_kinematic_Jacobian(y);
    dy = J.inverse() * du;
    y = y + dy;

    i = i + 1;
    if (i == max_iterations) {
      cerr << "Solution not found. Max iterations reached" << endl;
      break;
    }
  }

  // cout<<"roll="<<y(0)<<endl;
  // cout<<"pitch="<<y(1)<<endl;

  return y;
}

VectorXd surrPr2u1::calc_geometricmodel_inverse(const Math::VectorNd y) {
  double roll, pitch;
  roll = y(0);   // roll
  pitch = y(1);  // pitch

  double t2 = cos(pitch);
  double t3 = cos(roll);
  double t4 = sin(pitch);
  double t5 = sin(roll);
  double t6 = h1 * n1x;
  double t7 = h2 * n2x;
  double t8 = h1 * n1y;
  double t9 = h2 * n2y;
  double t10 = h1 * n1z;
  double t11 = h2 * n2z;
  double t12 = l1 * l1;
  double t13 = l2 * l2;
  double t14 = n1x * n1x;
  double t15 = n2x * n2x;
  double t16 = n1y * n1y;
  double t17 = n2y * n2y;
  double t18 = n1z * n1z;
  double t19 = n2z * n2z;
  double t20 = r1 * r1;
  double t21 = r2 * r2;
  double t30 = -c1x;
  double t31 = -c2x;
  double t32 = -c1z;
  double t33 = -c2z;
  double t36 = c1x / 2.0;
  double t37 = c2x / 2.0;
  double t38 = c1y / 2.0;
  double t39 = c2y / 2.0;
  double t40 = c1z / 2.0;
  double t41 = c2z / 2.0;
  double t22 = e1x * t2;
  double t23 = e2x * t2;
  double t24 = e1y * t3;
  double t25 = e2y * t3;
  double t26 = e1x * t4;
  double t27 = e2x * t4;
  double t28 = e1z * t5;
  double t29 = e2z * t5;
  double t34 = -t6;
  double t35 = -t7;
  double t42 = -t12;
  double t43 = -t13;
  double t44 = e1z * t2 * t3;
  double t45 = e2z * t2 * t3;
  double t46 = e1y * t2 * t5;
  double t47 = e2y * t2 * t5;
  double t48 = e1z * t3 * t4;
  double t49 = e2z * t3 * t4;
  double t50 = e1y * t4 * t5;
  double t51 = e2y * t4 * t5;
  double t56 = -t36;
  double t57 = -t37;
  double t58 = -t40;
  double t59 = -t41;
  double t52 = -t24;
  double t53 = -t25;
  double t54 = -t26;
  double t55 = -t27;
  double t60 = t22 / 2.0;
  double t61 = t23 / 2.0;
  double t62 = t24 / 2.0;
  double t63 = t25 / 2.0;
  double t64 = t26 / 2.0;
  double t65 = t27 / 2.0;
  double t66 = t28 / 2.0;
  double t67 = t29 / 2.0;
  double t68 = -t44;
  double t69 = -t45;
  double t70 = -t46;
  double t71 = -t47;
  double t76 = t44 / 2.0;
  double t77 = t45 / 2.0;
  double t78 = t46 / 2.0;
  double t79 = t47 / 2.0;
  double t80 = t48 / 2.0;
  double t81 = t49 / 2.0;
  double t82 = t50 / 2.0;
  double t83 = t51 / 2.0;
  double t90 = t22 + t30 + t34 + t48 + t50;
  double t91 = t23 + t31 + t35 + t49 + t51;
  double t72 = -t62;
  double t73 = -t63;
  double t74 = -t64;
  double t75 = -t65;
  double t84 = c1y + t8 + t28 + t52;
  double t85 = c2y + t9 + t29 + t53;
  double t88 = c1z + t10 + t26 + t68 + t70;
  double t89 = c2z + t11 + t27 + t69 + t71;
  double t94 = n1x * t90;
  double t95 = n2x * t91;
  double t86 = n1y * t84;
  double t87 = n2y * t85;
  double t92 = n1z * t88;
  double t93 = n2z * t89;
  double t96 = -t94;
  double t97 = -t95;
  double t98 = t86 + t92 + t96;
  double t99 = t87 + t93 + t97;
  double t100 = n1x * t98;
  double t101 = n2x * t99;
  double t102 = n1y * t98;
  double t103 = n2y * t99;
  double t104 = n1z * t98;
  double t105 = n2z * t99;
  double t106 = t98 * t98;
  double t107 = t99 * t99;
  double t108 = -t102;
  double t109 = -t103;
  double t110 = t100 / 2.0;
  double t111 = t101 / 2.0;
  double t112 = t102 / 2.0;
  double t113 = t103 / 2.0;
  double t114 = t104 / 2.0;
  double t115 = t105 / 2.0;
  double t116 = t14 * t106;
  double t117 = t15 * t107;
  double t118 = t16 * t106;
  double t119 = t17 * t107;
  double t120 = t18 * t106;
  double t121 = t19 * t107;
  double t130 = t22 + t30 + t48 + t50 + t100;
  double t131 = t23 + t31 + t49 + t51 + t101;
  double t132 = t32 + t44 + t46 + t54 + t104;
  double t133 = t33 + t45 + t47 + t55 + t105;
  double t122 = -t112;
  double t123 = -t113;
  double t124 = c1y + t28 + t52 + t108;
  double t125 = c2y + t29 + t53 + t109;
  double t134 = t130 * t130;
  double t135 = t131 * t131;
  double t136 = t132 * t132;
  double t137 = t133 * t133;
  double t142 = t20 + t42 + t116 + t118 + t120;
  double t143 = t21 + t43 + t117 + t119 + t121;
  double t126 = t124 * t124;
  double t127 = t125 * t125;
  double t138 = t134 * 2.0;
  double t139 = t135 * 2.0;
  double t140 = t136 * 2.0;
  double t141 = t137 * 2.0;
  double t128 = t126 * 2.0;
  double t129 = t127 * 2.0;
  double t144 = t128 + t138 + t140;
  double t145 = t129 + t139 + t141;
  double t146 = 1.0 / t144;
  double t147 = 1.0 / t145;
  double t148 = t124 * t142 * t146;
  double t149 = t125 * t143 * t147;
  double t150 = t130 * t142 * t146;
  double t151 = t131 * t143 * t147;
  double t152 = t132 * t142 * t146;
  double t153 = t133 * t143 * t147;
  double t154 = t38 + t66 + t72 + t122 + t148;
  double t155 = t39 + t67 + t73 + t123 + t149;
  double t160 = t56 + t60 + t80 + t82 + t110 + t150;
  double t161 = t57 + t61 + t81 + t83 + t111 + t151;
  double t164 = t58 + t74 + t76 + t78 + t114 + t152;
  double t165 = t59 + t75 + t77 + t79 + t115 + t153;
  double t156 = t154 * t154;
  double t157 = t155 * t155;
  double t162 = t160 * t160;
  double t163 = t161 * t161;
  double t166 = t164 * t164;
  double t167 = t165 * t165;
  double t158 = -t156;
  double t159 = -t157;
  double t168 = -t162;
  double t169 = -t163;
  double t170 = -t166;
  double t171 = -t167;
  double t172 = t156 + t162 + t166;
  double t173 = t157 + t163 + t167;
  double t174 = 1.0 / sqrt(t172);
  double t175 = 1.0 / sqrt(t173);
  double t176 = t20 + t158 + t168 + t170;
  double t177 = t21 + t159 + t169 + t171;
  double t178 = sqrt(t176);
  double t179 = sqrt(t177);

  double d11 = sqrt(pow(-b1z - t10 + t40 + t74 + t76 + t78 + t114 + t152 +
                            n1x * t154 * t174 * t178 + n1y * t160 * t174 * t178,
                        2.0) +
                    pow(b1y + t8 - t38 + t66 + t72 + t122 + t148 - n1x * t164 * t174 * t178 +
                            n1z * t160 * t174 * t178,
                        2.0) +
                    pow(-b1x + t34 + t36 + t60 + t80 + t82 + t110 + t150 -
                            n1y * t164 * t174 * t178 - n1z * t154 * t174 * t178,
                        2.0));

  double d21 = sqrt(pow(-b2z - t11 + t41 + t75 + t77 + t79 + t115 + t153 +
                            n2x * t155 * t175 * t179 + n2y * t161 * t175 * t179,
                        2.0) +
                    pow(b2y + t9 - t39 + t67 + t73 + t123 + t149 - n2x * t165 * t175 * t179 +
                            n2z * t161 * t175 * t179,
                        2.0) +
                    pow(-b2x + t35 + t37 + t61 + t81 + t83 + t111 + t151 -
                            n2y * t165 * t175 * t179 - n2z * t155 * t175 * t179,
                        2.0));

  VectorNd u(VectorNd::Zero(dof_active));
  u(0) = d11;
  u(1) = d21;

  return u;
}

Matrix2d surrPr2u1::compute_kinematic_Jacobian(const Math::VectorNd y) {
  // forward kinematic jacobian in terms of independent params

  Matrix2d J = Matrix2d::Zero();

  double roll, pitch;
  roll = y(0);   // roll
  pitch = y(1);  // pitch

  double t2 = cos(pitch);
  double t3 = cos(roll);
  double t4 = sin(pitch);
  double t5 = sin(roll);
  double t6 = h1 * n1x;
  double t7 = h2 * n2x;
  double t8 = h1 * n1y;
  double t9 = h2 * n2y;
  double t10 = h1 * n1z;
  double t11 = h2 * n2z;
  double t12 = l1 * l1;
  double t13 = l2 * l2;
  double t14 = n1x * n1x;
  double t15 = n2x * n2x;
  double t16 = n1y * n1y;
  double t17 = n2y * n2y;
  double t18 = n1z * n1z;
  double t19 = n2z * n2z;
  double t20 = r1 * r1;
  double t21 = r2 * r2;
  double t34 = -b1x;
  double t35 = -b2x;
  double t36 = -b1z;
  double t37 = -b2z;
  double t38 = -c1x;
  double t39 = -c2x;
  double t40 = -c1z;
  double t41 = -c2z;
  double t46 = c1x / 2.0;
  double t47 = c2x / 2.0;
  double t48 = c1y / 2.0;
  double t49 = c2y / 2.0;
  double t50 = c1z / 2.0;
  double t51 = c2z / 2.0;
  double t22 = e1x * t2;
  double t23 = e2x * t2;
  double t24 = e1y * t3;
  double t25 = e2y * t3;
  double t26 = e1z * t3;
  double t27 = e2z * t3;
  double t28 = e1x * t4;
  double t29 = e2x * t4;
  double t30 = e1y * t5;
  double t31 = e2y * t5;
  double t32 = e1z * t5;
  double t33 = e2z * t5;
  double t42 = -t6;
  double t43 = -t7;
  double t44 = -t10;
  double t45 = -t11;
  double t52 = -t12;
  double t53 = -t13;
  double t74 = -t46;
  double t75 = -t47;
  double t76 = -t48;
  double t77 = -t49;
  double t78 = -t50;
  double t79 = -t51;
  double t54 = t2 * t24;
  double t55 = t2 * t25;
  double t56 = t2 * t26;
  double t57 = t2 * t27;
  double t58 = t2 * t30;
  double t59 = t4 * t24;
  double t60 = t2 * t31;
  double t61 = t4 * t25;
  double t62 = t2 * t32;
  double t63 = t4 * t26;
  double t64 = t2 * t33;
  double t65 = t4 * t27;
  double t66 = t4 * t30;
  double t67 = t4 * t31;
  double t68 = t4 * t32;
  double t69 = t4 * t33;
  double t70 = -t24;
  double t71 = -t25;
  double t72 = -t28;
  double t73 = -t29;
  double t80 = t22 / 2.0;
  double t81 = t23 / 2.0;
  double t82 = t24 / 2.0;
  double t83 = t25 / 2.0;
  double t84 = t26 / 2.0;
  double t85 = t27 / 2.0;
  double t86 = t28 / 2.0;
  double t87 = t29 / 2.0;
  double t88 = t30 / 2.0;
  double t89 = t31 / 2.0;
  double t90 = t32 / 2.0;
  double t91 = t33 / 2.0;
  double t124 = t26 + t30;
  double t125 = t27 + t31;
  double t92 = -t54;
  double t93 = -t55;
  double t94 = -t56;
  double t95 = -t57;
  double t96 = -t58;
  double t97 = -t59;
  double t98 = -t60;
  double t99 = -t61;
  double t100 = -t62;
  double t101 = -t64;
  double t102 = -t68;
  double t103 = -t69;
  double t104 = -t82;
  double t105 = -t83;
  double t106 = -t86;
  double t107 = -t87;
  double t108 = t54 / 2.0;
  double t109 = t55 / 2.0;
  double t110 = t56 / 2.0;
  double t111 = t57 / 2.0;
  double t112 = t58 / 2.0;
  double t113 = t59 / 2.0;
  double t114 = t60 / 2.0;
  double t115 = t61 / 2.0;
  double t116 = t62 / 2.0;
  double t117 = t63 / 2.0;
  double t118 = t64 / 2.0;
  double t119 = t65 / 2.0;
  double t120 = t66 / 2.0;
  double t121 = t67 / 2.0;
  double t122 = t68 / 2.0;
  double t123 = t69 / 2.0;
  double t134 = n1y * t124;
  double t135 = n2y * t125;
  double t138 = c1y + t8 + t32 + t70;
  double t139 = c2y + t9 + t33 + t71;
  double t144 = t22 + t63 + t66;
  double t145 = t23 + t65 + t67;
  double t154 = t56 + t58 + t72;
  double t155 = t57 + t60 + t73;
  double t126 = -t108;
  double t127 = -t109;
  double t128 = -t110;
  double t129 = -t111;
  double t130 = -t112;
  double t131 = -t113;
  double t132 = -t114;
  double t133 = -t115;
  double t136 = -t134;
  double t137 = -t135;
  double t140 = t54 + t100;
  double t141 = t55 + t101;
  double t142 = t59 + t102;
  double t143 = t61 + t103;
  double t146 = n1y * t138;
  double t147 = n2y * t139;
  double t152 = n1z * t144;
  double t153 = n2z * t145;
  double t156 = n1x * t154;
  double t157 = n2x * t155;
  double t160 = c1z + t10 + t28 + t94 + t96;
  double t161 = c2z + t11 + t29 + t95 + t98;
  double t162 = t38 + t42 + t144;
  double t163 = t39 + t43 + t145;
  double t148 = n1z * t140;
  double t149 = n2z * t141;
  double t150 = n1x * t142;
  double t151 = n2x * t143;
  double t158 = -t152;
  double t159 = -t153;
  double t164 = n1z * t160;
  double t165 = n2z * t161;
  double t166 = n1x * t162;
  double t167 = n2x * t163;
  double t172 = -n1x * (t152 - t156);
  double t173 = -n2x * (t153 - t157);
  double t174 = -n1z * (t152 - t156);
  double t175 = -n2z * (t153 - t157);
  double t178 = n1x * (t152 - t156) * (-1.0 / 2.0);
  double t179 = n2x * (t153 - t157) * (-1.0 / 2.0);
  double t180 = n1y * (t152 - t156) * (-1.0 / 2.0);
  double t181 = n2y * (t153 - t157) * (-1.0 / 2.0);
  double t182 = n1z * (t152 - t156) * (-1.0 / 2.0);
  double t183 = n2z * (t153 - t157) * (-1.0 / 2.0);
  double t168 = -t166;
  double t169 = -t167;
  double t170 = t156 + t158;
  double t171 = t157 + t159;
  double t176 = t136 + t148 + t150;
  double t177 = t137 + t149 + t151;
  double t198 = t144 + t174;
  double t199 = t145 + t175;
  double t204 = t28 + t94 + t96 + t172;
  double t205 = t29 + t95 + t98 + t173;
  double t184 = n1x * t176;
  double t185 = n2x * t177;
  double t186 = n1y * t176;
  double t187 = n2y * t177;
  double t188 = n1z * t176;
  double t189 = n2z * t177;
  double t206 = t146 + t164 + t168;
  double t207 = t147 + t165 + t169;
  double t190 = t184 / 2.0;
  double t191 = t185 / 2.0;
  double t192 = t186 / 2.0;
  double t193 = t187 / 2.0;
  double t194 = t188 / 2.0;
  double t195 = t189 / 2.0;
  double t196 = t124 + t186;
  double t197 = t125 + t187;
  double t200 = t62 + t92 + t188;
  double t201 = t64 + t93 + t189;
  double t202 = t68 + t97 + t184;
  double t203 = t69 + t99 + t185;
  double t208 = n1x * t206;
  double t209 = n2x * t207;
  double t210 = n1y * t206;
  double t211 = n2y * t207;
  double t212 = n1z * t206;
  double t213 = n2z * t207;
  double t214 = t206 * t206;
  double t215 = t207 * t207;
  double t250 = t14 * t206 * (t152 - t156) * -2.0;
  double t251 = t15 * t207 * (t153 - t157) * -2.0;
  double t252 = t16 * t206 * (t152 - t156) * -2.0;
  double t253 = t17 * t207 * (t153 - t157) * -2.0;
  double t254 = t18 * t206 * (t152 - t156) * -2.0;
  double t255 = t19 * t207 * (t153 - t157) * -2.0;
  double t256 = t14 * t176 * t206 * 2.0;
  double t257 = t15 * t177 * t207 * 2.0;
  double t258 = t16 * t176 * t206 * 2.0;
  double t259 = t17 * t177 * t207 * 2.0;
  double t260 = t18 * t176 * t206 * 2.0;
  double t261 = t19 * t177 * t207 * 2.0;
  double t216 = -t210;
  double t217 = -t211;
  double t218 = t208 / 2.0;
  double t219 = t209 / 2.0;
  double t220 = t210 / 2.0;
  double t221 = t211 / 2.0;
  double t222 = t212 / 2.0;
  double t223 = t213 / 2.0;
  double t224 = t14 * t214;
  double t225 = t15 * t215;
  double t226 = t16 * t214;
  double t227 = t17 * t215;
  double t228 = t18 * t214;
  double t229 = t19 * t215;
  double t238 = t38 + t144 + t208;
  double t239 = t39 + t145 + t209;
  double t240 = t40 + t154 + t212;
  double t241 = t41 + t155 + t213;
  double t286 = t250 + t252 + t254;
  double t287 = t251 + t253 + t255;
  double t288 = t256 + t258 + t260;
  double t289 = t257 + t259 + t261;
  double t230 = -t220;
  double t231 = -t221;
  double t232 = c1y + t32 + t70 + t216;
  double t233 = c2y + t33 + t71 + t217;
  double t242 = t238 * t238;
  double t243 = t239 * t239;
  double t244 = t240 * t240;
  double t245 = t241 * t241;
  double t270 = t198 * t240 * 4.0;
  double t271 = t199 * t241 * 4.0;
  double t272 = t202 * t238 * 4.0;
  double t273 = t203 * t239 * 4.0;
  double t274 = t238 * (t154 + n1x * (t152 - t156)) * -4.0;
  double t275 = t239 * (t155 + n2x * (t153 - t157)) * -4.0;
  double t276 = t200 * t240 * 4.0;
  double t277 = t201 * t241 * 4.0;
  double t278 = t20 + t52 + t224 + t226 + t228;
  double t279 = t21 + t53 + t225 + t227 + t229;
  double t234 = t232 * t232;
  double t235 = t233 * t233;
  double t246 = t242 * 2.0;
  double t247 = t243 * 2.0;
  double t248 = t244 * 2.0;
  double t249 = t245 * 2.0;
  double t262 = n1y * t232 * (t152 - t156) * -4.0;
  double t263 = n2y * t233 * (t153 - t157) * -4.0;
  double t264 = n1y * t232 * (t152 - t156) * 4.0;
  double t265 = n2y * t233 * (t153 - t157) * 4.0;
  double t266 = t196 * t232 * 4.0;
  double t267 = t197 * t233 * 4.0;
  double t236 = t234 * 2.0;
  double t237 = t235 * 2.0;
  double t268 = -t266;
  double t269 = -t267;
  double t290 = t264 + t270 + t274;
  double t291 = t265 + t271 + t275;
  double t280 = t236 + t246 + t248;
  double t281 = t237 + t247 + t249;
  double t292 = t268 + t272 + t276;
  double t293 = t269 + t273 + t277;
  double t282 = 1.0 / t280;
  double t284 = 1.0 / t281;
  double t283 = t282 * t282;
  double t285 = t284 * t284;
  double t294 = -n1y * t278 * t282 * (t152 - t156);
  double t295 = -n2y * t279 * t284 * (t153 - t157);
  double t296 = t196 * t278 * t282;
  double t297 = t197 * t279 * t284;
  double t298 = t198 * t278 * t282;
  double t299 = t199 * t279 * t284;
  double t300 = t200 * t278 * t282;
  double t301 = t201 * t279 * t284;
  double t302 = t202 * t278 * t282;
  double t303 = t203 * t279 * t284;
  double t304 = -t278 * t282 * (t154 + n1x * (t152 - t156));
  double t305 = -t279 * t284 * (t155 + n2x * (t153 - t157));
  double t306 = t232 * t278 * t282;
  double t307 = t233 * t279 * t284;
  double t308 = t238 * t278 * t282;
  double t309 = t239 * t279 * t284;
  double t310 = t240 * t278 * t282;
  double t311 = t241 * t279 * t284;
  double t318 = -t232 * t282 *
                (t14 * t206 * (t152 - t156) * 2.0 + t16 * t206 * (t152 - t156) * 2.0 +
                 t18 * t206 * (t152 - t156) * 2.0);
  double t319 = -t233 * t284 *
                (t15 * t207 * (t153 - t157) * 2.0 + t17 * t207 * (t153 - t157) * 2.0 +
                 t19 * t207 * (t153 - t157) * 2.0);
  double t320 = t232 * t282 *
                (t14 * t206 * (t152 - t156) * 2.0 + t16 * t206 * (t152 - t156) * 2.0 +
                 t18 * t206 * (t152 - t156) * 2.0);
  double t321 = t233 * t284 *
                (t15 * t207 * (t153 - t157) * 2.0 + t17 * t207 * (t153 - t157) * 2.0 +
                 t19 * t207 * (t153 - t157) * 2.0);
  double t330 = -t238 * t282 *
                (t14 * t206 * (t152 - t156) * 2.0 + t16 * t206 * (t152 - t156) * 2.0 +
                 t18 * t206 * (t152 - t156) * 2.0);
  double t331 = -t239 * t284 *
                (t15 * t207 * (t153 - t157) * 2.0 + t17 * t207 * (t153 - t157) * 2.0 +
                 t19 * t207 * (t153 - t157) * 2.0);
  double t334 = -t240 * t282 *
                (t14 * t206 * (t152 - t156) * 2.0 + t16 * t206 * (t152 - t156) * 2.0 +
                 t18 * t206 * (t152 - t156) * 2.0);
  double t335 = -t241 * t284 *
                (t15 * t207 * (t153 - t157) * 2.0 + t17 * t207 * (t153 - t157) * 2.0 +
                 t19 * t207 * (t153 - t157) * 2.0);
  double t338 = t232 * t282 * t288;
  double t339 = t233 * t284 * t289;
  double t342 = t238 * t282 * t288;
  double t343 = t239 * t284 * t289;
  double t344 = t240 * t282 * t288;
  double t345 = t241 * t284 * t289;
  double t312 = t48 + t90 + t104 + t230 + t306;
  double t313 = t49 + t91 + t105 + t231 + t307;
  double t322 = t74 + t80 + t117 + t120 + t218 + t308;
  double t323 = t75 + t81 + t119 + t121 + t219 + t309;
  double t326 = t78 + t106 + t110 + t112 + t222 + t310;
  double t327 = t79 + t107 + t111 + t114 + t223 + t311;
  double t340 = -t338;
  double t341 = -t339;
  double t346 = t232 * t278 * t283 * t290;
  double t347 = t233 * t279 * t285 * t291;
  double t348 = t238 * t278 * t283 * t290;
  double t349 = t239 * t279 * t285 * t291;
  double t350 = t240 * t278 * t283 * t290;
  double t351 = t241 * t279 * t285 * t291;
  double t356 = t232 * t278 * t283 * t292;
  double t357 = t233 * t279 * t285 * t293;
  double t358 = t238 * t278 * t283 * t292;
  double t359 = t239 * t279 * t285 * t293;
  double t360 = t240 * t278 * t283 * t292;
  double t361 = t241 * t279 * t285 * t293;
  double t314 = t312 * t312;
  double t315 = t313 * t313;
  double t324 = t322 * t322;
  double t325 = t323 * t323;
  double t328 = t326 * t326;
  double t329 = t327 * t327;
  double t352 = -t348;
  double t353 = -t349;
  double t354 = -t350;
  double t355 = -t351;
  double t362 = -t358;
  double t363 = -t359;
  double t364 = -t360;
  double t365 = -t361;
  double t378 = t180 + t294 + t320 + t346;
  double t379 = t181 + t295 + t321 + t347;
  double t384 = t84 + t88 + t192 + t296 + t340 + t356;
  double t385 = t85 + t89 + t193 + t297 + t341 + t357;
  double t390 =
      t312 * (t318 - t346 + (n1y * (t152 - t156)) / 2.0 + n1y * t278 * t282 * (t152 - t156)) * -2.0;
  double t391 =
      t313 * (t319 - t347 + (n2y * (t153 - t157)) / 2.0 + n2y * t279 * t284 * (t153 - t157)) * -2.0;
  double t392 =
      t312 * (t318 - t346 + (n1y * (t152 - t156)) / 2.0 + n1y * t278 * t282 * (t152 - t156)) * 2.0;
  double t393 =
      t313 * (t319 - t347 + (n2y * (t153 - t157)) / 2.0 + n2y * t279 * t284 * (t153 - t157)) * 2.0;
  double t398 = t322 *
                (-t86 - t128 - t130 + t348 + (n1x * (t152 - t156)) / 2.0 +
                 t238 * t282 *
                     (t14 * t206 * (t152 - t156) * 2.0 + t16 * t206 * (t152 - t156) * 2.0 +
                      t18 * t206 * (t152 - t156) * 2.0) +
                 t278 * t282 * (t154 + n1x * (t152 - t156))) *
                -2.0;
  double t399 = t323 *
                (-t87 - t129 - t132 + t349 + (n2x * (t153 - t157)) / 2.0 +
                 t239 * t284 *
                     (t15 * t207 * (t153 - t157) * 2.0 + t17 * t207 * (t153 - t157) * 2.0 +
                      t19 * t207 * (t153 - t157) * 2.0) +
                 t279 * t284 * (t155 + n2x * (t153 - t157))) *
                -2.0;
  double t316 = -t314;
  double t317 = -t315;
  double t332 = -t324;
  double t333 = -t325;
  double t336 = -t328;
  double t337 = -t329;
  double t366 = t314 + t324 + t328;
  double t367 = t315 + t325 + t329;
  double t380 = t80 + t117 + t120 + t182 + t298 + t334 + t354;
  double t381 = t81 + t119 + t121 + t183 + t299 + t335 + t355;
  double t382 = t86 + t128 + t130 + t178 + t304 + t330 + t352;
  double t383 = t87 + t129 + t132 + t179 + t305 + t331 + t353;
  double t386 = t122 + t131 + t190 + t302 + t342 + t362;
  double t387 = t123 + t133 + t191 + t303 + t343 + t363;
  double t388 = t116 + t126 + t194 + t300 + t344 + t364;
  double t389 = t118 + t127 + t195 + t301 + t345 + t365;
  double t394 = t312 * t384 * 2.0;
  double t395 = t313 * t385 * 2.0;
  double t368 = 1.0 / sqrt(t366);
  double t370 = 1.0 / sqrt(t367);
  double t372 = t20 + t316 + t332 + t336;
  double t373 = t21 + t317 + t333 + t337;
  double t396 = -t394;
  double t397 = -t395;
  double t400 = t326 * t380 * 2.0;
  double t401 = t327 * t381 * 2.0;
  double t402 = t322 * t386 * 2.0;
  double t403 = t323 * t387 * 2.0;
  double t404 = t326 * t388 * 2.0;
  double t405 = t327 * t389 * 2.0;
  double t369 = t368 * t368 * t368;
  double t371 = t370 * t370 * t370;
  double t374 = sqrt(t372);
  double t375 = sqrt(t373);
  double t424 = t392 + t398 + t400;
  double t425 = t393 + t399 + t401;
  double t426 = t396 + t402 + t404;
  double t427 = t397 + t403 + t405;
  double t376 = 1.0 / t374;
  double t377 = 1.0 / t375;
  double t406 = n1x * t312 * t368 * t374;
  double t407 = n1z * t312 * t368 * t374;
  double t408 = n2x * t313 * t370 * t375;
  double t409 = n2z * t313 * t370 * t375;
  double t412 = n1y * t322 * t368 * t374;
  double t413 = n1z * t322 * t368 * t374;
  double t414 = n2y * t323 * t370 * t375;
  double t415 = n2z * t323 * t370 * t375;
  double t416 = n1x * t326 * t368 * t374;
  double t417 = n1y * t326 * t368 * t374;
  double t418 = n2x * t327 * t370 * t375;
  double t419 = n2y * t327 * t370 * t375;
  double t410 = -t407;
  double t411 = -t409;
  double t420 = -t416;
  double t421 = -t417;
  double t422 = -t418;
  double t423 = -t419;
  double t430 = t36 + t44 + t50 + t106 + t110 + t112 + t222 + t310 + t406 + t412;
  double t431 = t37 + t45 + t51 + t107 + t111 + t114 + t223 + t311 + t408 + t414;
  double t428 = b1y + t8 + t76 + t90 + t104 + t230 + t306 + t413 + t420;
  double t429 = b2y + t9 + t77 + t91 + t105 + t231 + t307 + t415 + t422;
  double t434 = t430 * t430;
  double t435 = t431 * t431;
  double t436 = t34 + t42 + t46 + t80 + t117 + t120 + t218 + t308 + t410 + t421;
  double t437 = t35 + t43 + t47 + t81 + t119 + t121 + t219 + t309 + t411 + t423;
  double t432 = t428 * t428;
  double t433 = t429 * t429;
  double t438 = t436 * t436;
  double t439 = t437 * t437;
  double t440 = t432 + t434 + t438;
  double t441 = t433 + t435 + t439;
  double t442 = 1.0 / sqrt(t440);
  double t443 = 1.0 / sqrt(t441);

  J(0, 0) =
      (t442 *
       (t430 *
            (t108 - t116 - t194 - t300 - t344 + t360 + n1x * t368 * t374 * t384 -
             n1y * t368 * t374 * t386 + (n1x * t312 * t369 * t374 * t426) / 2.0 +
             (n1x * t312 * t368 * t376 * t426) / 2.0 + (n1y * t322 * t369 * t374 * t426) / 2.0 +
             (n1y * t322 * t368 * t376 * t426) / 2.0) *
            2.0 +
        t428 *
            (t384 + n1x * t368 * t374 * t388 - n1z * t368 * t374 * t386 -
             (n1x * t326 * t369 * t374 * t426) / 2.0 - (n1x * t326 * t368 * t376 * t426) / 2.0 +
             (n1z * t322 * t369 * t374 * t426) / 2.0 + (n1z * t322 * t368 * t376 * t426) / 2.0) *
            2.0 -
        t436 *
            (t386 - n1y * t368 * t374 * t388 + n1z * t368 * t374 * t384 +
             (n1y * t326 * t369 * t374 * t426) / 2.0 + (n1y * t326 * t368 * t376 * t426) / 2.0 +
             (n1z * t312 * t369 * t374 * t426) / 2.0 + (n1z * t312 * t368 * t376 * t426) / 2.0) *
            2.0)) /
      2.0;
  J(0, 1) =
      (t442 *
       (t436 *
            (t382 - n1y * t368 * t374 * t380 -
             n1z * t368 * t374 *
                 (t318 - t346 + (n1y * (t152 - t156)) / 2.0 + n1y * t278 * t282 * (t152 - t156)) +
             (n1y * t326 * t369 * t374 * t424) / 2.0 + (n1y * t326 * t368 * t376 * t424) / 2.0 +
             (n1z * t312 * t369 * t374 * t424) / 2.0 + (n1z * t312 * t368 * t376 * t424) / 2.0) *
            -2.0 +
        t428 *
            (t378 + n1x * t368 * t374 * t380 +
             n1z * t368 * t374 *
                 (-t86 - t128 - t130 + t348 + (n1x * (t152 - t156)) / 2.0 +
                  t238 * t282 *
                      (t14 * t206 * (t152 - t156) * 2.0 + t16 * t206 * (t152 - t156) * 2.0 +
                       t18 * t206 * (t152 - t156) * 2.0) +
                  t278 * t282 * (t154 + n1x * (t152 - t156))) -
             (n1x * t326 * t369 * t374 * t424) / 2.0 - (n1x * t326 * t368 * t376 * t424) / 2.0 +
             (n1z * t322 * t369 * t374 * t424) / 2.0 + (n1z * t322 * t368 * t376 * t424) / 2.0) *
            2.0 +
        t430 *
            (-t80 - t117 - t120 - t298 + t350 + (n1z * (t152 - t156)) / 2.0 +
             t240 * t282 *
                 (t14 * t206 * (t152 - t156) * 2.0 + t16 * t206 * (t152 - t156) * 2.0 +
                  t18 * t206 * (t152 - t156) * 2.0) -
             n1x * t368 * t374 *
                 (t318 - t346 + (n1y * (t152 - t156)) / 2.0 + n1y * t278 * t282 * (t152 - t156)) +
             n1y * t368 * t374 *
                 (-t86 - t128 - t130 + t348 + (n1x * (t152 - t156)) / 2.0 +
                  t238 * t282 *
                      (t14 * t206 * (t152 - t156) * 2.0 + t16 * t206 * (t152 - t156) * 2.0 +
                       t18 * t206 * (t152 - t156) * 2.0) +
                  t278 * t282 * (t154 + n1x * (t152 - t156))) +
             (n1x * t312 * t369 * t374 * t424) / 2.0 + (n1x * t312 * t368 * t376 * t424) / 2.0 +
             (n1y * t322 * t369 * t374 * t424) / 2.0 + (n1y * t322 * t368 * t376 * t424) / 2.0) *
            2.0)) /
      2.0;
  J(1, 0) =
      (t443 *
       (t431 *
            (t109 - t118 - t195 - t301 - t345 + t361 + n2x * t370 * t375 * t385 -
             n2y * t370 * t375 * t387 + (n2x * t313 * t371 * t375 * t427) / 2.0 +
             (n2x * t313 * t370 * t377 * t427) / 2.0 + (n2y * t323 * t371 * t375 * t427) / 2.0 +
             (n2y * t323 * t370 * t377 * t427) / 2.0) *
            2.0 +
        t429 *
            (t385 + n2x * t370 * t375 * t389 - n2z * t370 * t375 * t387 -
             (n2x * t327 * t371 * t375 * t427) / 2.0 - (n2x * t327 * t370 * t377 * t427) / 2.0 +
             (n2z * t323 * t371 * t375 * t427) / 2.0 + (n2z * t323 * t370 * t377 * t427) / 2.0) *
            2.0 -
        t437 *
            (t387 - n2y * t370 * t375 * t389 + n2z * t370 * t375 * t385 +
             (n2y * t327 * t371 * t375 * t427) / 2.0 + (n2y * t327 * t370 * t377 * t427) / 2.0 +
             (n2z * t313 * t371 * t375 * t427) / 2.0 + (n2z * t313 * t370 * t377 * t427) / 2.0) *
            2.0)) /
      2.0;
  J(1, 1) =
      (t443 *
       (t437 *
            (t383 - n2y * t370 * t375 * t381 -
             n2z * t370 * t375 *
                 (t319 - t347 + (n2y * (t153 - t157)) / 2.0 + n2y * t279 * t284 * (t153 - t157)) +
             (n2y * t327 * t371 * t375 * t425) / 2.0 + (n2y * t327 * t370 * t377 * t425) / 2.0 +
             (n2z * t313 * t371 * t375 * t425) / 2.0 + (n2z * t313 * t370 * t377 * t425) / 2.0) *
            -2.0 +
        t429 *
            (t379 + n2x * t370 * t375 * t381 +
             n2z * t370 * t375 *
                 (-t87 - t129 - t132 + t349 + (n2x * (t153 - t157)) / 2.0 +
                  t239 * t284 *
                      (t15 * t207 * (t153 - t157) * 2.0 + t17 * t207 * (t153 - t157) * 2.0 +
                       t19 * t207 * (t153 - t157) * 2.0) +
                  t279 * t284 * (t155 + n2x * (t153 - t157))) -
             (n2x * t327 * t371 * t375 * t425) / 2.0 - (n2x * t327 * t370 * t377 * t425) / 2.0 +
             (n2z * t323 * t371 * t375 * t425) / 2.0 + (n2z * t323 * t370 * t377 * t425) / 2.0) *
            2.0 +
        t431 *
            (-t81 - t119 - t121 - t299 + t351 + (n2z * (t153 - t157)) / 2.0 +
             t241 * t284 *
                 (t15 * t207 * (t153 - t157) * 2.0 + t17 * t207 * (t153 - t157) * 2.0 +
                  t19 * t207 * (t153 - t157) * 2.0) -
             n2x * t370 * t375 *
                 (t319 - t347 + (n2y * (t153 - t157)) / 2.0 + n2y * t279 * t284 * (t153 - t157)) +
             n2y * t370 * t375 *
                 (-t87 - t129 - t132 + t349 + (n2x * (t153 - t157)) / 2.0 +
                  t239 * t284 *
                      (t15 * t207 * (t153 - t157) * 2.0 + t17 * t207 * (t153 - t157) * 2.0 +
                       t19 * t207 * (t153 - t157) * 2.0) +
                  t279 * t284 * (t155 + n2x * (t153 - t157))) +
             (n2x * t313 * t371 * t375 * t425) / 2.0 + (n2x * t313 * t370 * t377 * t425) / 2.0 +
             (n2y * t323 * t371 * t375 * t425) / 2.0 + (n2y * t323 * t370 * t377 * t425) / 2.0) *
            2.0)) /
      2.0;

  return J;
}

}  // namespace SURRPR2U1
