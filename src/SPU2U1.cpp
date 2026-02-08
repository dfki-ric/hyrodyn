#include "SPU2U1.hpp"

namespace SPU2U1 {
void sPu2u1::calc_permutationmatrix(std::vector<string> jointnames_spanningtree,
                                    std::vector<string> jointnames_active) {
  permutation_matrix.setZero(jointnames_active.size(), jointnames_spanningtree.size());

  for (unsigned int i = 0; i < jointnames_active.size(); i++) {
    for (unsigned int j = 0; j < jointnames_spanningtree.size(); j++) {
      if (jointnames_spanningtree[j] == jointnames_active[i])
        permutation_matrix(i, j) = 1;
    }
  }
}

double sPu2u1::wrap2pi(double x) {
  // This function wrap the x values into the interval (-pi, pi]
  // wrapx = x + n pi with n integer such that wrapx \in (-pi, pi]

  double wrapx;
  wrapx = atan2(sin(x), cos(x));
  //    cout<<"wrapx = "<<wrapx<<endl;
  return wrapx;
}

// Constructor
sPu2u1::sPu2u1(string file_path, std::vector<string> jointnames_spanningtree,
               std::vector<string> jointnames_active) {
  Model m;

  const char* ext;
  ext = strrchr(file_path.c_str(), '.');
  if (!strcmp(ext, ".urdf") || !strcmp(ext, ".URDF") || !strcmp(ext, ".robot")) {
    // cout<<"Input file is URDF"<<endl;
    /*
            if (!Addons::URDFReadFromFile (file_path.c_str(), &m, false)) {
                    std::cerr << "Error loading model .urdf" << std::endl;
                    abort();
            }
            */

    if (!Addons::URDFReadFromFileWithModularity(file_path.c_str(), &m, jointnames_spanningtree,
                                                false)) {
      std::cerr << "Error loading robot model from urdf" << std::endl;
      abort();
    }

  } else {
    cout << "Unknown file type: Accepted file types are .urdf or .lua" << endl;
  }

  VectorNd Q(VectorNd::Zero(m.dof_count));
  UpdateKinematicsCustom(m, &Q, NULL, NULL);

  /*
  // Calculate the fixed rotation matrices of first bi frames
  RotMat_b1 = CalcBodyWorldOrientation(m, Q, 1.);
  RotMat_b2 = CalcBodyWorldOrientation(m, Q, 4.);
  // Calculate the position vectors to bi frames
  b1 = CalcBodyToBaseCoordinates (m, Q, 1., Vector3d (0., 0., 0.), false);
  b2 = CalcBodyToBaseCoordinates (m, Q, 4., Vector3d (0., 0., 0.), false);
  // Calculate the positiob vector to ei frame in end effector frame
  e1_ee = CalcBodyToBaseCoordinates (m, Q, 3., Vector3d (0., 0., 0.), false);
  e2_ee = CalcBodyToBaseCoordinates (m, Q, 6., Vector3d (0., 0., 0.), false);
  */

  // Calculate the fixed rotation matrices of first bi frames
  RotMat_b1 = CalcBodyWorldOrientation(m, Q, 3.);
  RotMat_b2 = CalcBodyWorldOrientation(m, Q, 6.);
  // Calculate the position vectors to bi frames
  b1 = CalcBodyToBaseCoordinates(m, Q, 3., Vector3d(0., 0., 0.), false);
  b2 = CalcBodyToBaseCoordinates(m, Q, 6., Vector3d(0., 0., 0.), false);
  // Calculate the positiob vector to ei frame in end effector frame
  e1_ee = CalcBodyToBaseCoordinates(m, Q, 5., Vector3d(0., 0., 0.), false);
  e2_ee = CalcBodyToBaseCoordinates(m, Q, 8., Vector3d(0., 0., 0.), false);

  cout << "Model DoF overview:" << Utils::GetModelDOFOverview(m) << endl;
  cout << "Model Hierarchy overview:" << Utils::GetModelHierarchy(m) << endl;
  cout << "Named Body Origins overview:" << Utils::GetNamedBodyOriginsOverview(m) << endl;

  dof_active = 2;
  dof_spanningtree = m.dof_count;

  Q_zero = VectorNd::Zero(m.dof_count);
  VectorNd y(2);
  y(0) = 0.0;
  y(1) = 0.0;
  Q_zero = calc_loopclosure_function(y);
  // cout<<"Q_zero in constructor: "<<Q_zero.transpose()<<endl;
  calc_permutationmatrix(jointnames_spanningtree, jointnames_active);
  /*
      double mass;
      Vector3d com;
      Utils::CalcCenterOfMass(m, Q, Q, mass, com);
      cout<<"Submechanism mass :"<<mass<<" Submechanism COM
     :"<<com.transpose()<<endl;
      */
}

VectorXd sPu2u1::calc_loopclosure_function(const Math::VectorNd& y) {
  // This function calculates the loop closure functions gamma
  // for the mechanism

  VectorXd Q(VectorNd::Zero(8));

  Matrix3d RotMat_x, RotMat_y, RotMat;

  double roll, pitch;
  roll = y(0);   // roll
  pitch = y(1);  // pitch

  RotMat_x << 1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll);

  RotMat_y << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch);

  RotMat = RotMat_x * RotMat_y;
  // cout<<"RotMat: "<<RotMat<<endl;

  Vector3d e1, e2;
  e1 = RotMat * e1_ee;
  e2 = RotMat * e2_ee;

  Vector3d b1e1 = e1 - b1;
  Vector3d b2e2 = e2 - b2;

  // transform the UPS leg vector to universal joint frame
  b1e1 = RotMat_b1 * b1e1;
  b2e2 = RotMat_b2 * b2e2;

  double d1 = (e1 - b1).norm();
  double d2 = (e2 - b2).norm();

  // cout<<"actor length d1: "<<d1<<" d2: "<<d2<<endl;

  double roll_u1, pitch_u1;
  roll_u1 = atan2(-b1e1(1), b1e1(2));
  pitch_u1 = asin(b1e1(0) / d1);

  // cout <<"roll universal_1 = "<<roll_u1<<"pitch_universal_1 =
  // "<<pitch_u1<<endl;

  double roll_u2, pitch_u2;
  roll_u2 = atan2(-b2e2(1), b2e2(2));
  pitch_u2 = asin(b2e2(0) / d2);

  // cout <<"roll universal_2 = "<<roll_u2<<"pitch_universal_2 =
  // "<<pitch_u2<<endl;
  /*
  Q(0) = roll_u1;	// roll
  Q(1) = pitch_u1;	// pitch
  Q(2) = d1;	// theta 1
  Q(3) = roll_u2;	// theta 2
  Q(4) = pitch_u2;
  Q(5) = d2;
  Q(6) = y(0);
  Q(7) = y(1);	// actor 2 length
  */

  Q(0) = y(0);
  Q(1) = y(1);      // actor 2 length
  Q(2) = roll_u1;   // roll
  Q(3) = pitch_u1;  // pitch
  Q(4) = d1;        // theta 1
  Q(5) = roll_u2;   // theta 2
  Q(6) = pitch_u2;
  Q(7) = d2;

  Q = Q - Q_zero;  // wrt to zero position of assembly

  return Q;
}

MatrixXd sPu2u1::calc_loopclosure_Jacobian(const Math::VectorNd& y) {
  // This function calculates the loop closure Jacobian G
  // for the mechanism

  MatrixXd G;
  G.setZero(8, 2);

  // loop closure jacobian for two UPS legs
  MatrixXd G1, G2;
  G1.setZero(3, 2);
  G2.setZero(3, 2);

  G1 = compute_loopclosure_Jacobian_UPS_leg(y, e1_ee, b1, RotMat_b1);
  G2 = compute_loopclosure_Jacobian_UPS_leg(y, e2_ee, b2, RotMat_b2);
  /*
  G.block(0,0,3,2) = G1;
  G.block(3,0,3,2) = G2;
  G(6,0) = 1.0;
  G(7,1) = 1.0;
  */

  G(0, 0) = 1.0;
  G(1, 1) = 1.0;
  G.block(2, 0, 3, 2) = G1;
  G.block(5, 0, 3, 2) = G2;

  // cout<<"Loop closure jacobian, G: \n"<<G<<endl;

  return G;
}

MatrixXd sPu2u1::calc_loopclosure_Jacobiand(const Math::VectorNd& y, const Math::VectorNd& ydot) {
  // This function calculates the loop closure Jacobiand Gdot
  // for the mechanism
  //
  //       /   q1(d)  \
	//  Q =  |   q1(d)  |
  //       \    d    /
  //

  MatrixXd Gdot;
  Gdot.setZero(8, 2);

  // loop closure jacobian for two UPS legs
  MatrixXd G1dot, G2dot;
  G1dot.setZero(3, 2);
  G2dot.setZero(3, 2);

  G1dot = compute_loopclosure_Jacobiandot_UPS_leg(y, ydot, e1_ee, b1, RotMat_b1);
  G2dot = compute_loopclosure_Jacobiandot_UPS_leg(y, ydot, e2_ee, b2, RotMat_b2);
  /*
  Gdot.block(0,0,3,2) = G1dot;
  Gdot.block(3,0,3,2) = G2dot;
  */
  Gdot.block(2, 0, 3, 2) = G1dot;
  Gdot.block(5, 0, 3, 2) = G2dot;
  // cout<<"Loop closure jacobian dot, Gdot: \n"<<Gdot<<endl;

  return Gdot;
}

VectorXd sPu2u1::calc_loopclosure_g(const Math::VectorNd& y, const Math::VectorNd& ydot) {
  // This function calculates the loop closure bias acceleration g= Gdot*ydot
  // for the mechanism
  //
  //       /   q1(d)  \
	//  Q =  |   q1(d)  |
  //       \    d    /
  //
  return calc_loopclosure_Jacobiand(y, ydot) * ydot;
}

VectorXd sPu2u1::calc_geometricmodel_forward(const Math::VectorNd u, unsigned int max_iterations,
                                             double step_tol) {
  VectorNd y(VectorNd::Zero(dof_active));
  VectorNd u_i(VectorNd::Zero(dof_active));
  VectorNd du(VectorNd::Zero(dof_active));
  VectorNd dy(VectorNd::Zero(dof_active));
  Matrix2d J = Matrix2d::Zero();

  unsigned int i = 0;

  while (1) {
    u_i = sPu2u1::calc_geometricmodel_inverse(y);

    du = u - u_i;

    if (du.norm() < step_tol)
      break;

    J = sPu2u1::compute_kinematic_Jacobian(y);
    dy = J * du;
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

VectorXd sPu2u1::calc_geometricmodel_inverse(const Math::VectorNd y) {
  double roll, pitch;
  roll = y(0);   // roll
  pitch = y(1);  // pitch

  Matrix3d RotMat_x, RotMat_y, RotMat;

  RotMat_x << 1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll);

  RotMat_y << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch);

  RotMat = RotMat_x * RotMat_y;

  Vector3d e1, e2;
  e1 = RotMat * e1_ee;
  e2 = RotMat * e2_ee;

  Vector3d b1e1 = e1 - b1;
  Vector3d b2e2 = e2 - b2;

  double d1 = (e1 - b1).norm();
  double d2 = (e2 - b2).norm();

  // cout<<"actor length d1: "<<d1<<" d2: "<<d2<<endl;

  VectorNd u(VectorNd::Zero(dof_active));
  u(0) = d1;
  u(1) = d2;

  return u;
}

Matrix2d sPu2u1::compute_kinematic_Jacobian(const Math::VectorNd y) {
  // forward kinematic jacobian in terms of independent params

  Matrix2d J = Matrix2d::Zero();

  double roll, pitch;
  roll = y(0);   // roll
  pitch = y(1);  // pitch

  double b1x, b1y, b1z, e1x, e1y, e1z;
  // point b1 (in base frame) and e1(in end effector frame)
  b1x = b1(0);
  b1y = b1(1);
  b1z = b1(2);

  e1x = e1_ee(0);
  e1y = e1_ee(1);
  e1z = e1_ee(2);

  double b2x, b2y, b2z, e2x, e2y, e2z;
  // point b2 (in base frame) and e2(in end effector frame)
  b2x = b2(0);
  b2y = b2(1);
  b2z = b2(2);

  e2x = e2_ee(0);
  e2y = e2_ee(1);
  e2z = e2_ee(2);

  double t2 = cos(roll);
  double t4 = cos(pitch);
  double t5 = sin(pitch);
  double t7 = sin(roll);
  double t35 = e1y * t7;
  double t36 = e1z * t2 * t4;
  double t37 = e1x * t2 * t5;
  double t38 = b1z - t35 - t36 + t37;
  double t3 = fabs(t38);
  double t30 = e1x * t4;
  double t31 = e1z * t5;
  double t32 = -b1x + t30 + t31;
  double t6 = fabs(t32);
  double t26 = e1y * t2;
  double t27 = e1z * t4 * t7;
  double t28 = e1x * t5 * t7;
  double t29 = b1y - t26 + t27 - t28;
  double t8 = fabs(t29);
  double t9 = e2x * t4;
  double t10 = e2z * t5;
  double t11 = -b2x + t9 + t10;
  double t12 = fabs(t11);
  double t13 = (t11 / fabs(t11));
  double t14 = e2x * t2 * t5;
  double t22 = e2y * t7;
  double t23 = e2z * t2 * t4;
  double t15 = b2z + t14 - t22 - t23;
  double t16 = e2z * t4 * t7;
  double t18 = e2y * t2;
  double t19 = e2x * t5 * t7;
  double t17 = b2y + t16 - t18 - t19;
  double t20 = fabs(t17);
  double t21 = (t17 / fabs(t17));
  double t24 = fabs(t15);
  double t25 = (t15 / fabs(t15));
  double t33 = (t29 / fabs(t29));
  double t34 = (t32 / fabs(t32));
  double t39 = (t38 / fabs(t38));
  double t40 = t2 * t2;
  double t41 = t4 * t4;
  double t42 = t5 * t5;
  double t43 = t7 * t7;
  double t44 = e1y * e2z * t4 * t7 * t8 * t12 * t13 * t33;
  double t45 = e1x * e2y * t5 * t6 * t7 * t20 * t21 * t34;
  double t46 = e1x * e2y * t3 * t4 * t24 * t25 * t39 * t40;
  double t47 = e2y * e1z * t3 * t5 * t24 * t25 * t39 * t40;
  double t48 = e1z * e2z * t3 * t7 * t12 * t13 * t39 * t41;
  double t49 = e1x * e2x * t3 * t7 * t12 * t13 * t39 * t42;
  double t50 = e1z * e2z * t2 * t8 * t12 * t13 * t33 * t41;
  double t51 = e1x * e2x * t2 * t8 * t12 * t13 * t33 * t42;
  double t52 = e1x * e2y * t4 * t8 * t20 * t21 * t33 * t43;
  double t53 = e2y * e1z * t5 * t8 * t20 * t21 * t33 * t43;
  double t54 = e2x * e1z * t8 * t24 * t25 * t33 * t40 * t41;
  double t55 = e1x * e2z * t8 * t24 * t25 * t33 * t41 * t43;
  double t56 = e2x * e1z * t3 * t20 * t21 * t39 * t40 * t42;
  double t57 = e1x * e2z * t3 * t20 * t21 * t39 * t42 * t43;
  double t58 = e2y * e1z * t2 * t4 * t6 * t24 * t25 * t34;
  double t59 = e2x * e1y * t2 * t3 * t5 * t12 * t13 * t39;
  double t60 = e2x * e1z * t2 * t3 * t7 * t24 * t25 * t39 * t41;
  double t61 = e2x * e1z * t2 * t3 * t7 * t24 * t25 * t39 * t42;
  double t62 = e1x * e2x * t3 * t4 * t5 * t20 * t21 * t39 * t40;
  double t63 = e1z * e2z * t4 * t5 * t8 * t24 * t25 * t33 * t40;
  double t64 = e1x * e2x * t3 * t4 * t5 * t20 * t21 * t39 * t43;
  double t65 = e1z * e2z * t4 * t5 * t8 * t24 * t25 * t33 * t43;
  double t66 = e1x * e2z * t2 * t7 * t8 * t20 * t21 * t33 * t41;
  double t67 = e1x * e2z * t2 * t7 * t8 * t20 * t21 * t33 * t42;
  double t68 = e2x * e1y * t2 * t3 * t4 * t7 * t20 * t21 * t39;
  double t69 = e2x * e1y * t2 * t4 * t7 * t8 * t24 * t25 * t33;
  double t70 = e1x * e2z * t4 * t5 * t6 * t7 * t24 * t25 * t34;
  double t71 = e2x * e1z * t4 * t5 * t6 * t7 * t24 * t25 * t34;
  double t72 = e1y * e2z * t2 * t3 * t5 * t7 * t20 * t21 * t39;
  double t73 = e1y * e2z * t2 * t5 * t7 * t8 * t24 * t25 * t33;
  double t74 = e1x * e2z * t2 * t4 * t5 * t6 * t20 * t21 * t34;
  double t75 = e2x * e1z * t2 * t4 * t5 * t6 * t20 * t21 * t34;
  double t83 = e2y * e1z * t4 * t6 * t7 * t20 * t21 * t34;
  double t84 = e2x * e1y * t5 * t7 * t8 * t12 * t13 * t33;
  double t85 = e2x * e1y * t3 * t4 * t24 * t25 * t39 * t40;
  double t86 = e1y * e2z * t3 * t5 * t24 * t25 * t39 * t40;
  double t87 = e1z * e2z * t6 * t7 * t24 * t25 * t34 * t41;
  double t88 = e1x * e2x * t6 * t7 * t24 * t25 * t34 * t42;
  double t89 = e1z * e2z * t2 * t6 * t20 * t21 * t34 * t41;
  double t90 = e1x * e2x * t2 * t6 * t20 * t21 * t34 * t42;
  double t91 = e2x * e1y * t4 * t8 * t20 * t21 * t33 * t43;
  double t92 = e1y * e2z * t5 * t8 * t20 * t21 * t33 * t43;
  double t93 = e1x * e2z * t3 * t20 * t21 * t39 * t40 * t41;
  double t94 = e1x * e2z * t8 * t24 * t25 * t33 * t40 * t42;
  double t95 = e2x * e1z * t3 * t20 * t21 * t39 * t41 * t43;
  double t96 = e2x * e1z * t8 * t24 * t25 * t33 * t42 * t43;
  double t97 = e1y * e2z * t2 * t3 * t4 * t12 * t13 * t39;
  double t98 = e1x * e2y * t2 * t5 * t6 * t24 * t25 * t34;
  double t99 = e1x * e2z * t2 * t3 * t7 * t24 * t25 * t39 * t41;
  double t100 = e1x * e2z * t2 * t3 * t7 * t24 * t25 * t39 * t42;
  double t101 = e1x * e2x * t4 * t5 * t8 * t24 * t25 * t33 * t40;
  double t102 = e1z * e2z * t3 * t4 * t5 * t20 * t21 * t39 * t40;
  double t103 = e1x * e2x * t4 * t5 * t8 * t24 * t25 * t33 * t43;
  double t104 = e1z * e2z * t3 * t4 * t5 * t20 * t21 * t39 * t43;
  double t105 = e2x * e1z * t2 * t7 * t8 * t20 * t21 * t33 * t41;
  double t106 = e2x * e1z * t2 * t7 * t8 * t20 * t21 * t33 * t42;
  double t107 = e1x * e2y * t2 * t3 * t4 * t7 * t20 * t21 * t39;
  double t108 = e1x * e2y * t2 * t4 * t7 * t8 * t24 * t25 * t33;
  double t109 = e1x * e2z * t3 * t4 * t5 * t7 * t12 * t13 * t39;
  double t110 = e2x * e1z * t3 * t4 * t5 * t7 * t12 * t13 * t39;
  double t111 = e2y * e1z * t2 * t3 * t5 * t7 * t20 * t21 * t39;
  double t112 = e2y * e1z * t2 * t5 * t7 * t8 * t24 * t25 * t33;
  double t113 = e1x * e2z * t2 * t4 * t5 * t8 * t12 * t13 * t33;
  double t114 = e2x * e1z * t2 * t4 * t5 * t8 * t12 * t13 * t33;
  double t76 = t44 + t45 + t46 + t47 + t48 + t49 + t50 + t51 + t52 + t53 + t54 + t55 + t56 + t57 +
               t58 + t59 + t60 + t61 + t62 + t63 + t64 + t65 + t66 + t67 + t68 + t69 + t70 + t71 +
               t72 + t73 + t74 + t75 - t83 - t84 - t85 - t86 - t87 - t88 - t89 - t90 - t91 - t92 -
               t93 - t94 - t95 - t96 - t97 - t98 - t99 - t100 - t101 - t102 - t103 - t104 - t105 -
               t106 - t107 - t108 - t109 - t110 - t111 - t112 - t113 - t114;
  double t77 = 1.0 / t76;
  double t78 = t3 * t3;
  double t79 = t6 * t6;
  double t80 = t8 * t8;
  double t81 = t78 + t79 + t80;
  double t82 = sqrt(t81);
  double t115 = t24 * t24;
  double t116 = t12 * t12;
  double t117 = t20 * t20;
  double t118 = t115 + t116 + t117;
  double t119 = sqrt(t118);

  J(0, 0) = -t77 * t82 *
            (e2x * t5 * t12 * t13 - e2z * t4 * t12 * t13 + e2x * t4 * t7 * t20 * t21 -
             e2x * t2 * t4 * t24 * t25 + e2z * t5 * t7 * t20 * t21 - e2z * t2 * t5 * t24 * t25);
  J(0, 1) = t77 * t119 *
            (e1x * t5 * t6 * t34 - e1z * t4 * t6 * t34 - e1x * t2 * t3 * t4 * t39 +
             e1x * t4 * t7 * t8 * t33 - e1z * t2 * t3 * t5 * t39 + e1z * t5 * t7 * t8 * t33);
  J(1, 0) = -t77 * t82 *
            (e2y * t7 * t20 * t21 - e2y * t2 * t24 * t25 - e2x * t2 * t5 * t20 * t21 -
             e2x * t5 * t7 * t24 * t25 + e2z * t2 * t4 * t20 * t21 + e2z * t4 * t7 * t24 * t25);
  J(1, 1) = -t77 * t119 *
            (e1y * t2 * t3 * t39 - e1y * t7 * t8 * t33 + e1x * t2 * t5 * t8 * t33 +
             e1x * t3 * t5 * t7 * t39 - e1z * t2 * t4 * t8 * t33 - e1z * t3 * t4 * t7 * t39);

  return J;
}

MatrixXd sPu2u1::compute_loopclosure_Jacobian_UPS_leg(const Math::VectorNd y, const Vector3d e_ee,
                                                      const Vector3d b, const Matrix3d RotMat_b) {
  MatrixXd J;
  J.setZero(3, 2);

  double bx, by, bz, ex, ey, ez;
  double sx, sy, sz, nx, ny, nz, ax, ay,
      az;                     // elements of rotation matrix to the universal joint frame
  double roll, pitch;         // independent joints
  double roll_u, pitch_u, l;  // parameters for UPS leg

  // independent params
  roll = y(0);
  pitch = y(1);

  // point b (in base frame) and e(in end effector frame)
  bx = b(0);
  by = b(1);
  bz = b(2);

  ex = e_ee(0);
  ey = e_ee(1);
  ez = e_ee(2);

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

  // solving the leg geometric model first
  Matrix3d RotMat_x, RotMat_y, RotMat;

  RotMat_x << 1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll);

  RotMat_y << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch);

  RotMat = RotMat_x * RotMat_y;

  Vector3d e = RotMat * e_ee;

  Vector3d be = e - b;

  // transform the UPS leg vector to universal joint frame
  be = RotMat_b * be;

  // leg parameters
  l = (e - b).norm();
  roll_u = atan2(-be(1), be(2));
  pitch_u = asin(be(0) / l);
  // cout <<"roll universal = "<<roll_u<<"pitch_universal = "<<pitch_u<<"l =
  // "<<l<<endl;

  double t2 = cos(roll);
  double t3 = cos(pitch);
  double t4 = sin(roll);
  double t5 = sin(pitch);
  double t6 = 1.0 / l;
  double t7 = cos(pitch_u);
  double t8 = 1.0 / t7;
  double t9 = ey * t4;
  double t10 = ez * t2 * t3;
  double t25 = ex * t2 * t5;
  double t11 = t9 + t10 - t25;
  double t12 = ey * t2;
  double t13 = ex * t4 * t5;
  double t26 = ez * t3 * t4;
  double t14 = t12 + t13 - t26;
  double t15 = cos(roll_u);
  double t16 = sin(roll_u);
  double t17 = ex * t2 * t3;
  double t18 = ez * t2 * t5;
  double t19 = t17 + t18;
  double t20 = ex * t3 * t4;
  double t21 = ez * t4 * t5;
  double t22 = t20 + t21;
  double t23 = ex * t5;
  double t31 = ez * t3;
  double t24 = t23 - t31;
  double t27 = az * t14;
  double t39 = nz * t11;
  double t28 = t27 - t39;
  double t29 = sin(pitch_u);
  double t30 = ay * t14;
  double t32 = ay * t19;
  double t33 = sy * t24;
  double t44 = ny * t22;
  double t34 = t32 + t33 - t44;
  double t35 = az * t19;
  double t36 = sz * t24;
  double t37 = ax * t14;
  double t38 = t30 - ny * t11;
  double t40 = ax * t19;
  double t41 = sx * t24;
  double t42 = t40 + t41 - nx * t22;
  double t43 = t35 + t36 - nz * t22;

  J(0, 0) = -t6 * t8 * t16 * t28 - t6 * t8 * t15 * t38;
  J(0, 1) = t6 * t8 * t15 * t34 + t6 * t8 * t16 * t43;
  J(1, 0) = t6 * t7 * (t37 - nx * t11) - t6 * t15 * t28 * t29 + t6 * t16 * t29 * t38;
  J(1, 1) = -t6 * t7 * t42 - t6 * t16 * t29 * t34 + t6 * t15 * t29 * t43;
  J(2, 0) = t29 * (t37 - nx * t11) - t7 * t16 * t38 + t7 * t15 * (t27 - t39);
  J(2, 1) = -t29 * t42 + t7 * t16 * (t32 + t33 - t44) - t7 * t15 * t43;

  return J;
}

MatrixXd sPu2u1::compute_loopclosure_Jacobiandot_UPS_leg(const Math::VectorNd y,
                                                         const Math::VectorNd yd,
                                                         const Vector3d e_ee, const Vector3d b,
                                                         const Matrix3d RotMat_b) {
  MatrixXd Gdot;
  Gdot.setZero(3, 2);

  double bx, by, bz, ex, ey, ez;
  double sx, sy, sz, nx, ny, nz, ax, ay,
      az;                     // elements of rotation matrix to the universal joint frame
  double roll, pitch;         // independent joints
  double roll_u, pitch_u, l;  // parameters for UPS leg

  // independent params
  roll = y(0);
  pitch = y(1);

  // point b (in base frame) and e(in end effector frame)
  bx = b(0);
  by = b(1);
  bz = b(2);

  ex = e_ee(0);
  ey = e_ee(1);
  ez = e_ee(2);

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

  // solving the leg geometric model first
  Matrix3d RotMat_x, RotMat_y, RotMat;

  RotMat_x << 1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll);

  RotMat_y << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch);

  RotMat = RotMat_x * RotMat_y;

  Vector3d e = RotMat * e_ee;

  Vector3d be = e - b;

  // transform the UPS leg vector to universal joint frame
  be = RotMat_b * be;

  // leg parameters
  l = (e - b).norm();
  roll_u = atan2(-be(1), be(2));
  pitch_u = asin(be(0) / l);
  // cout <<"roll universal = "<<roll_u<<"pitch_universal = "<<pitch_u<<"l =
  // "<<l<<endl;

  Vector3d qd = compute_loopclosure_Jacobian_UPS_leg(y, e_ee, b, RotMat_b) * yd;
  double roll_u_dot = qd(0);
  double pitch_u_dot = qd(1);
  double l_dot = qd(2);

  // roll and pitch velocities
  double roll_dot = yd(0);
  double pitch_dot = yd(1);

  double t2 = cos(roll);
  double t3 = sin(pitch);
  double t4 = cos(pitch);
  double t5 = sin(roll);
  double t6 = 1.0 / l;
  double t7 = cos(pitch_u);
  double t8 = 1.0 / t7;
  double t9 = ex * pitch_dot * t2 * t4;
  double t10 = ez * pitch_dot * t2 * t3;
  double t11 = ez * roll_dot * t4 * t5;
  double t62 = ey * roll_dot * t2;
  double t63 = ex * roll_dot * t3 * t5;
  double t12 = t9 + t10 + t11 - t62 - t63;
  double t13 = ex * pitch_dot * t4 * t5;
  double t14 = ex * roll_dot * t2 * t3;
  double t15 = ez * pitch_dot * t3 * t5;
  double t64 = ey * roll_dot * t5;
  double t65 = ez * roll_dot * t2 * t4;
  double t16 = t13 + t14 + t15 - t64 - t65;
  double t17 = cos(roll_u);
  double t18 = ey * t5;
  double t19 = ez * t2 * t4;
  double t26 = ex * t2 * t3;
  double t20 = t18 + t19 - t26;
  double t21 = ey * t2;
  double t22 = ex * t3 * t5;
  double t27 = ez * t4 * t5;
  double t23 = t21 + t22 - t27;
  double t24 = 1.0 / (l * l);
  double t25 = sin(roll_u);
  double t28 = az * t23;
  double t29 = ay * t23;
  double t75 = ny * t20;
  double t30 = t29 - t75;
  double t31 = 1.0 / (t7 * t7);
  double t32 = sin(pitch_u);
  double t74 = nz * t20;
  double t33 = t28 - t74;
  double t34 = ex * pitch_dot * t4;
  double t35 = ez * pitch_dot * t3;
  double t36 = t34 + t35;
  double t37 = ez * roll_dot * t3 * t5;
  double t38 = ex * pitch_dot * t2 * t3;
  double t39 = ex * roll_dot * t4 * t5;
  double t76 = ez * pitch_dot * t2 * t4;
  double t40 = t37 + t38 + t39 - t76;
  double t41 = ex * roll_dot * t2 * t4;
  double t42 = ez * pitch_dot * t4 * t5;
  double t43 = ez * roll_dot * t2 * t3;
  double t77 = ex * pitch_dot * t3 * t5;
  double t44 = t41 + t42 + t43 - t77;
  double t45 = ex * t2 * t4;
  double t46 = ez * t2 * t3;
  double t47 = t45 + t46;
  double t48 = ex * t4 * t5;
  double t49 = ez * t3 * t5;
  double t50 = t48 + t49;
  double t51 = ex * t3;
  double t54 = ez * t4;
  double t52 = t51 - t54;
  double t53 = az * t47;
  double t55 = sz * t52;
  double t61 = nz * t50;
  double t56 = t53 + t55 - t61;
  double t57 = ay * t47;
  double t58 = sy * t52;
  double t60 = ny * t50;
  double t59 = t57 + t58 - t60;
  double t66 = nz * t12;
  double t67 = az * t16;
  double t68 = t66 + t67;
  double t69 = ny * t12;
  double t70 = ay * t16;
  double t71 = t69 + t70;
  double t72 = ax * t23;
  double t89 = nx * t20;
  double t73 = t72 - t89;
  double t78 = az * t40;
  double t79 = nz * t44;
  double t93 = sz * t36;
  double t80 = t78 + t79 - t93;
  double t81 = ay * t40;
  double t82 = ny * t44;
  double t94 = sy * t36;
  double t83 = t81 + t82 - t94;
  double t84 = ax * t47;
  double t85 = sx * t52;
  double t86 = nx * t12;
  double t87 = ax * t16;
  double t88 = t86 + t87;
  double t90 = ax * t40;
  double t91 = nx * t44;
  double t92 = t90 + t91 - sx * t36;
  double t95 = t84 + t85 - nx * t50;

  Gdot(0, 0) = -t6 * t8 * t17 * t71 - t6 * t8 * t25 * t68 + l_dot * t8 * t17 * t24 * t30 +
               l_dot * t8 * t24 * t25 * t33 - roll_u_dot * t6 * t8 * t17 * t33 +
               roll_u_dot * t6 * t8 * t25 * t30 - pitch_u_dot * t6 * t17 * t30 * t31 * t32 -
               pitch_u_dot * t6 * t25 * t31 * t32 * t33;
  Gdot(0, 1) = -t6 * t8 * t17 * t83 - t6 * t8 * t25 * t80 - l_dot * t8 * t17 * t24 * t59 -
               l_dot * t8 * t24 * t25 * t56 + roll_u_dot * t6 * t8 * t17 * t56 -
               roll_u_dot * t6 * t8 * t25 * t59 +
               pitch_u_dot * t6 * t17 * t31 * t32 * (t57 + t58 - t60) +
               pitch_u_dot * t6 * t25 * t31 * t32 * (t53 + t55 - t61);
  Gdot(1, 0) =
      t6 * t7 * t88 - l_dot * t7 * t24 * t73 - pitch_u_dot * t6 * t32 * t73 - t6 * t17 * t32 * t68 +
      t6 * t25 * t32 * t71 - l_dot * t24 * t25 * t30 * t32 - pitch_u_dot * t6 * t7 * t17 * t33 +
      l_dot * t17 * t24 * t32 * (t28 - t74) + pitch_u_dot * t6 * t7 * t25 * (t29 - t75) +
      roll_u_dot * t6 * t17 * t32 * (t29 - t75) + roll_u_dot * t6 * t25 * t32 * (t28 - t74);
  Gdot(1, 1) = t6 * t7 * t92 + l_dot * t7 * t24 * t95 + pitch_u_dot * t6 * t32 * t95 -
               t6 * t17 * t32 * t80 + t6 * t25 * t32 * t83 +
               l_dot * t24 * t25 * t32 * (t57 + t58 - t60) +
               pitch_u_dot * t6 * t7 * t17 * (t53 + t55 - t61) - l_dot * t17 * t24 * t32 * t56 -
               pitch_u_dot * t6 * t7 * t25 * t59 - roll_u_dot * t6 * t17 * t32 * t59 -
               roll_u_dot * t6 * t25 * t32 * t56;
  Gdot(2, 0) = t32 * t88 + t7 * t17 * t68 - t7 * t25 * t71 + pitch_u_dot * t7 * (t72 - t89) -
               pitch_u_dot * t17 * t32 * t33 - roll_u_dot * t7 * t17 * t30 -
               roll_u_dot * t7 * t25 * t33 + pitch_u_dot * t25 * t32 * (t29 - t75);
  Gdot(2, 1) = t32 * t92 - pitch_u_dot * t7 * t95 + t7 * t17 * t80 - t7 * t25 * t83 -
               pitch_u_dot * t25 * t32 * t59 + pitch_u_dot * t17 * t32 * (t53 + t55 - t61) +
               roll_u_dot * t7 * t17 * (t57 + t58 - t60) +
               roll_u_dot * t7 * t25 * (t53 + t55 - t61);

  /*

          double t2 = cos(roll);
          double t3 = sin(pitch);
          double t4 = cos(pitch);
          double t5 = sin(roll);
          double t6 = 1.0/l;
          double t7 = cos(pitch_u);
          double t8 = 1.0/t7;
          double t9 = ex*pitch_dot*t2*t4;
          double t10 = ez*pitch_dot*t2*t3;
          double t11 = ez*roll_dot*t4*t5;
          double t30 = ey*roll_dot*t2;
          double t31 = ex*roll_dot*t3*t5;
          double t12 = t9+t10+t11-t30-t31;
          double t13 = ex*pitch_dot*t4*t5;
          double t14 = ex*roll_dot*t2*t3;
          double t15 = ez*pitch_dot*t3*t5;
          double t32 = ey*roll_dot*t5;
          double t33 = ez*roll_dot*t2*t4;
          double t16 = t13+t14+t15-t32-t33;
          double t17 = cos(roll_u);
          double t18 = sin(roll_u);
          double t19 = ex*pitch_dot*t4;
          double t20 = ez*pitch_dot*t3;
          double t21 = t19+t20;
          double t22 = ez*roll_dot*t3*t5;
          double t23 = ex*pitch_dot*t2*t3;
          double t24 = ex*roll_dot*t4*t5;
          double t41 = ez*pitch_dot*t2*t4;
          double t25 = t22+t23+t24-t41;
          double t26 = ex*roll_dot*t2*t4;
          double t27 = ez*pitch_dot*t4*t5;
          double t28 = ez*roll_dot*t2*t3;
          double t42 = ex*pitch_dot*t3*t5;
          double t29 = t26+t27+t28-t42;
          double t34 = nz*t12;
          double t35 = az*t16;
          double t36 = t34+t35;
          double t37 = sin(pitch_u);
          double t38 = ny*t12;
          double t39 = ay*t16;
          double t40 = t38+t39;
          double t43 = az*t25;
          double t44 = nz*t29;
          double t55 = sz*t21;
          double t45 = t43+t44-t55;
          double t46 = ay*t25;
          double t47 = ny*t29;
          double t56 = sy*t21;
          double t48 = t46+t47-t56;
          double t49 = nx*t12;
          double t50 = ax*t16;
          double t51 = t49+t50;
          double t52 = ax*t25;
          double t53 = nx*t29;
          double t54 = t52+t53-sx*t21;

          Gdot(0,0) = -t6*t8*t18*t36-t6*t8*t17*t40;
          Gdot(0,1) = -t6*t8*t18*t45-t6*t8*t17*t48;
          Gdot(1,0) = t6*t7*t51-t6*t17*t36*t37+t6*t18*t37*t40;
          Gdot(1,1) = t6*t7*t54-t6*t17*t37*t45+t6*t18*t37*t48;
          Gdot(2,0) = t37*t51+t7*t17*t36-t7*t18*t40;
          Gdot(2,1) = t37*t54+t7*t17*t45-t7*t18*t48;
  */
  return Gdot;
}

}  // namespace SPU2U1
