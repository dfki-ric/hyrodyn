#include "SPRR2U1.hpp"

namespace SPRR2U1 {
void sPrr2u1::calc_permutationmatrix(
    std::vector<string> jointnames_spanningtree,
    std::vector<string> jointnames_active) {

  permutation_matrix.setZero(jointnames_active.size(),
                             jointnames_spanningtree.size());

  for (unsigned int i = 0; i < jointnames_active.size(); i++) {
    for (unsigned int j = 0; j < jointnames_spanningtree.size(); j++) {
      if (jointnames_spanningtree[j] == jointnames_active[i])
        permutation_matrix(i, j) = 1;
    }
  }
}

double sPrr2u1::wrap2pi(double x) {
  // This function wrap the x values into the interval (-pi, pi]
  // wrapx = x + n pi with n integer such that wrapx \in (-pi, pi]

  double wrapx;
  wrapx = atan2(sin(x), cos(x));
  //    cout<<"wrapx = "<<wrapx<<endl;
  return wrapx;
}

// Constructor
sPrr2u1::sPrr2u1(string file_path, std::vector<string> jointnames_spanningtree,
                 std::vector<string> jointnames_active) {

  // Model m;

  const char *ext;
  ext = strrchr(file_path.c_str(), '.');
  if (!strcmp(ext, ".urdf") || !strcmp(ext, ".URDF") ||
      !strcmp(ext, ".robot")) {
    // cout<<"Input file is URDF"<<endl;
    if (!Addons::URDFReadFromFile(file_path.c_str(), &m, false)) {
      std::cerr << "Error loading model rh5_knee.urdf" << std::endl;
      abort();
    }

  } else {
    cout << "Unknown file type: Accepted file types are .urdf or .lua" << endl;
  }

  //	cout<<"Model DoF overview:"<<Utils::GetModelDOFOverview(m)<<endl;
  //	cout<<"Model Hierarchy overview:"<<Utils::GetModelHierarchy(m)<<endl;
  //	cout<<"Named Body Origins
  //overview:"<<Utils::GetNamedBodyOriginsOverview(m)<<endl;

  VectorNd Q(VectorNd::Zero(m.dof_count));
  UpdateKinematicsCustom(m, &Q, NULL, NULL);

  f1_ee = CalcBodyToBaseCoordinates(m, Q, 3., Vector3d(0., 0., 0.), false);
  f2_ee = CalcBodyToBaseCoordinates(m, Q, 4., Vector3d(0., 0., 0.), false);
  //	cout<<"f1_ee: "<<f1_ee.transpose()<<endl;
  //	cout<<"f2_ee: "<<f2_ee.transpose()<<endl;

  f1x = f1_ee(0);
  f1y = f1_ee(1);
  f1z = f1_ee(2);
  f2x = f2_ee(0);
  f2y = f2_ee(1);
  f2z = f2_ee(2);

  s1 = CalcBodyToBaseCoordinates(m, Q, 5., Vector3d(0., 0., 0.), false);
  s2 = CalcBodyToBaseCoordinates(m, Q, 8., Vector3d(0., 0., 0.), false);
  //	cout<<"s1: "<<s1.transpose()<<endl;
  //	cout<<"s2: "<<s2.transpose()<<endl;

  s1x = s1(0);
  s1y = s1(1);
  s1z = s1(2);
  s2x = s2(0);
  s2y = s2(1);
  s2z = s2(2);

  Vector3d k1 =
      CalcBodyToBaseCoordinates(m, Q, 7., Vector3d(0., 0., 0.), false);
  Vector3d k2 =
      CalcBodyToBaseCoordinates(m, Q, 10., Vector3d(0., 0., 0.), false);
  //	cout<<"k1: "<<k1.transpose()<<endl;
  //	cout<<"k2: "<<k2.transpose()<<endl;

  r1 = (f1_ee - k1).norm();
  r2 = (f2_ee - k2).norm();

  //	cout<<"r1="<<r1<<endl;
  //	cout<<"r2="<<r2<<endl;

  f1k1_zero = -(f1_ee - k1) / (f1_ee - k1).norm();
  f2k2_zero = -(f2_ee - k2) / (f2_ee - k2).norm();

  RotMat_s1 = CalcBodyWorldOrientation(m, Q, 5.);
  RotMat_s2 = CalcBodyWorldOrientation(m, Q, 8.);
  //	cout<<"RotMat_s1: "<< RotMat_s1 <<endl;
  //	cout<<"RotMat_s2: "<< RotMat_s2 <<endl;

  RotMat_f1 = CalcBodyWorldOrientation(m, Q, 3.);
  RotMat_f2 = CalcBodyWorldOrientation(m, Q, 4.);
  //	cout<<"RotMat_f1: "<<RotMat_f1<<endl;
  //	cout<<"RotMat_f2: "<<RotMat_f2<<endl;

  i1_ee << RotMat_f1(0, 0), RotMat_f1(1, 0), RotMat_f1(2, 0);
  i2_ee << RotMat_f2(0, 0), RotMat_f2(1, 0), RotMat_f2(2, 0);

  dof_active = 2;
  dof_spanningtree = m.dof_count;

  Q_zero = VectorNd::Zero(m.dof_count);
  VectorNd y(2);
  y(0) = 0.0;
  y(1) = 0.0;
  Q_zero = calc_loopclosure_function(y);
}

VectorXd sPrr2u1::calc_loopclosure_function(const Math::VectorNd &y) {
  // This function calculates the loop closure functions gamma
  // for the mechanism
  //
  //       /   q1(d)  \
	//  Q =  |   q1(d)  |
  //       \    d    /
  //
  VectorXd Q(VectorNd::Zero(10));
  Matrix3d RotMat_x, RotMat_y, RotMat;

  double roll, pitch;
  roll = y(0);  // roll
  pitch = y(1); // pitch

  Q(0) = y(0); // roll
  Q(1) = y(1); // pitch

  RotMat_x << 1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll);

  RotMat_y << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch);

  RotMat = RotMat_x * RotMat_y;
  //	cout<<"RotMat: "<<RotMat<<endl;

  Vector3d f1, f2;
  f1 = RotMat * f1_ee;
  f2 = RotMat * f2_ee;
  //	cout<<"f1: "<<f1.transpose()<<endl;
  //	cout<<"f2: "<<f2.transpose()<<endl;

  Vector3d i1, i2;
  i1 = RotMat * i1_ee;
  i2 = RotMat * i2_ee;
  //	cout<<"i1: "<<i1.transpose()<<endl;
  //	cout<<"i2: "<<i2.transpose()<<endl;

  Vector3d delta1 = s1 - f1;
  Vector3d delta2 = s2 - f2;

  double d1 =
      sqrt(pow(i1.dot(delta1), 2) + pow((i1.cross(delta1)).norm() - r1, 2));
  double d2 =
      sqrt(pow(i2.dot(delta2), 2) + pow((i2.cross(delta2)).norm() - r2, 2));
  //	cout<<"d1: "<<d1<<endl;
  //	cout<<"d2: "<<d2<<endl;

  Vector3d f1k1_unit = (delta1 - i1.dot(delta1) * i1);
  Vector3d f2k2_unit = (delta2 - i2.dot(delta2) * i2);

  Vector3d k1 = f1 + r1 * f1k1_unit / f1k1_unit.norm();
  Vector3d k2 = f2 + r2 * f2k2_unit / f2k2_unit.norm();
  //	cout<<"k1: "<<k1.transpose()<<endl;
  //	cout<<"k2: "<<k2.transpose()<<endl;

  // f1k1_unit = -(f1-k1);
  // f2k2_unit = -(f2-k2);

  Matrix3d RotMat_f1n = CalcBodyWorldOrientation(m, Q, 3.);
  Matrix3d RotMat_f2n = CalcBodyWorldOrientation(m, Q, 4.);
  //	cout<<"RotMat_f1n: "<<RotMat_f1n<<endl;
  //	cout<<"RotMat_f2n: "<<RotMat_f2n<<endl;

  //	cout<<"RotMat_f1 cal: "<<RotMat*RotMat_f1<<endl;
  //	cout<<"RotMat_f2 cal: "<<RotMat*RotMat_f2<<endl;

  f1k1_unit = RotMat_f1n * f1k1_unit;
  f2k2_unit = RotMat_f2n * f2k2_unit;
  //	cout<<"f1k1 unit in f1 frame: "<<f1k1_unit.transpose()<<endl;
  //	cout<<"f2k2 unit in f2 frame: "<<f2k2_unit.transpose()<<endl;

  double theta1 = atan2(-f1k1_unit(1), f1k1_unit(2));
  double theta2 = atan2(-f2k2_unit(1), f2k2_unit(2));
  //	cout<<"theta1 new:"<<theta1<<endl;
  //	cout<<"theta2 new:"<<theta2<<endl;

  /*
          f1k1_unit = f1k1_unit/f1k1_unit.norm();
          f2k2_unit = f2k2_unit/f2k2_unit.norm();
          cout<<"f1k1 unit: "<<f1k1_unit.transpose()<<endl;
          cout<<"f2k2 unit: "<<f2k2_unit.transpose()<<endl;

          double costheta1 = f1k1_zero.dot(f1k1_unit);
          double sintheta1 = sqrt(1-pow(costheta1,2));
          double theta1 = atan2(sintheta1, costheta1);

          // computing directed angle between 2 vectors
      cout<<"theta1 matlab = "<<atan2( (f1k1_zero.cross(f1k1_unit)).norm() ,
     f1k1_unit.dot(f1k1_zero) )<<endl; cout<<"theta2 matlab = "<<atan2(
     (f2k2_zero.cross(f2k2_unit)).norm() , f2k2_unit.dot(f2k2_zero) )<<endl;

          double costheta2 = f2k2_zero.dot(f2k2_unit);
          double sintheta2 = sqrt(1-pow(costheta2,2));
          double theta2 = atan2(sintheta2, costheta2);
          cout <<"theta_1 = "<<theta1<<"theta_2 = "<<theta2<<endl;
          */
  // express kisi in si frame
  Vector3d k1s1 = RotMat_s1 * (k1 - s1);
  Vector3d k2s2 = RotMat_s2 * (k2 - s2);

  double X_s1 = k1s1(0);
  double Y_s1 = k1s1(1);
  double Z_s1 = k1s1(2);

  // double roll_s1 = atan2(Y_s1, sqrt(pow(X_s1,2) + pow(Z_s1,2)));
  // double pitch_s1 = atan2(X_s1/cos(roll_s1), Z_s1/cos(roll_s1));
  double roll_s1 = atan2(-Y_s1, Z_s1);
  double sin_pitch_s1 = X_s1 / d1;
  double pitch_s1 = atan2(sin_pitch_s1, sqrt(1 - pow(sin_pitch_s1, 2)));

  // double roll_s1 = atan2(-Y_s1, Z_s1);
  // double pitch_s1 = asin(X_s1/d1);

  //	cout <<"roll universal_1 = "<<roll_s1<<"pitch_universal_1 =
  //"<<pitch_s1<<endl;

  double X_s2 = k2s2(0);
  double Y_s2 = k2s2(1);
  double Z_s2 = k2s2(2);

  // double roll_s2 = atan2(Y_s2, sqrt(pow(X_s2,2) + pow(Z_s2,2)));
  // double pitch_s2 = atan2(X_s2/cos(roll_s2), Z_s2/cos(roll_s2));
  double roll_s2 = atan2(-Y_s2, Z_s2);
  double sin_pitch_s2 = X_s2 / d2;
  double pitch_s2 = atan2(sin_pitch_s2, sqrt(1 - pow(sin_pitch_s2, 2)));
  //	cout <<"roll universal_2 = "<<roll_s2<<"pitch_universal_2 =
  //"<<pitch_s2<<endl;

  Q(2) = theta1; // theta 1
  Q(3) = theta2; // theta 2
  Q(4) = roll_s1;
  Q(5) = pitch_s1;
  Q(6) = d1; // actor 1 length
  Q(7) = roll_s2;
  Q(8) = pitch_s2;
  Q(9) = d2; // actor 2 length

  Q = Q - Q_zero; // wrt to zero position of assembly
                  //	cout<<"Q vector: \n"<<Q.transpose()<<endl;
  return Q;
}

MatrixXd sPrr2u1::calc_loopclosure_Jacobian(const Math::VectorNd &y) {
  // This function calculates the loop closure Jacobian G
  // for the mechanism

  MatrixXd G;
  G.setZero(10, 2);

  // loop closure jacobian for two UPS legs
  MatrixXd G1, G2;
  G1.setZero(3, 2);
  G2.setZero(3, 2);

  G1 = compute_loopclosure_Jacobian_UP_leg(y, i1_ee, r1, f1_ee, s1, RotMat_s1);
  G2 = compute_loopclosure_Jacobian_UP_leg(y, i2_ee, r2, f2_ee, s2, RotMat_s2);

  G(0, 0) = 1.0;
  G(1, 1) = 1.0;
  G.block(4, 0, 3, 2) = G1;
  G.block(7, 0, 3, 2) = G2;

  //	cout<<"Loop closure jacobian, G: \n"<<G<<endl;

  return G;
}

MatrixXd sPrr2u1::calc_loopclosure_Jacobiand(const Math::VectorNd &y,
                                             const Math::VectorNd &ydot) {
  // This function calculates the loop closure Jacobiand Gdot
  // for the mechanism
  //
  //       /   q1(d)  \
	//  Q =  |   q1(d)  |
  //       \    d    /
  //

  MatrixXd Gdot;
  Gdot.setZero(10, 2);

  // loop closure jacobian for two UPS legs
  MatrixXd G1dot, G2dot;
  G1dot.setZero(3, 2);
  G2dot.setZero(3, 2);

  G1dot = compute_loopclosure_Jacobiandot_UP_leg(y, ydot, i1_ee, r1, f1_ee, s1,
                                                 RotMat_s1);
  G2dot = compute_loopclosure_Jacobiandot_UP_leg(y, ydot, i2_ee, r2, f2_ee, s2,
                                                 RotMat_s2);

  Gdot.block(4, 0, 3, 2) = G1dot;
  Gdot.block(7, 0, 3, 2) = G2dot;

  return Gdot;
}

VectorXd sPrr2u1::calc_loopclosure_g(const Math::VectorNd &y,
                                     const Math::VectorNd &ydot) {
  // This function calculates the loop closure bias acceleration g= Gdot*ydot
  // for the mechanism
  //
  //       /   q1(d)  \
	//  Q =  |   q1(d)  |
  //       \    d    /
  //

  return calc_loopclosure_Jacobiand(y, ydot) * ydot;
}

VectorXd sPrr2u1::calc_geometricmodel_forward(const Math::VectorNd u,
                                              unsigned int max_iterations,
                                              double step_tol) {

  VectorNd y(VectorNd::Zero(dof_active));
  VectorNd u_i(VectorNd::Zero(dof_active));
  VectorNd du(VectorNd::Zero(dof_active));
  VectorNd dy(VectorNd::Zero(dof_active));
  Matrix2d J = Matrix2d::Zero();

  unsigned int i = 0;

  while (1) {
    // cout<<"y:"<<y<<endl;
    u_i = sPrr2u1::calc_geometricmodel_inverse(y);
    // cout<<"u_i:"<<u_i<<endl;
    du = u - u_i;
    // cout<<"du:"<<du<<endl;
    if (du.norm() < step_tol)
      break;

    J = sPrr2u1::compute_kinematic_Jacobian(y);
    dy = J.inverse() * du; // It takes too much time to compute symbolic inverse
                           // of the Jacobian matrix in MATLAB.
    y = y + dy;

    i = i + 1;
    if (i == 20) {
      cout << "Max iterations reached" << endl;
      abort();
      break;
    }
  }

  //	cout<<"roll="<<y(0)<<endl;
  //	cout<<"pitch="<<y(1)<<endl;

  return y;
}

VectorXd sPrr2u1::calc_geometricmodel_inverse(const Math::VectorNd y) {

  double roll, pitch;
  roll = y(0);
  pitch = y(1);

  Matrix3d RotMat_x, RotMat_y, RotMat;

  RotMat_x << 1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll);

  RotMat_y << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch);

  RotMat = RotMat_x * RotMat_y;
  // cout<<"RotMat: "<<RotMat<<endl;

  Vector3d f1, f2;
  f1 = RotMat * f1_ee;
  f2 = RotMat * f2_ee;
  // cout<<"f1: "<<f1.transpose()<<endl;
  // cout<<"f2: "<<f2.transpose()<<endl;

  Vector3d i1, i2;
  i1 = RotMat * i1_ee;
  i2 = RotMat * i2_ee;
  // cout<<"i1: "<<i1.transpose()<<endl;
  // cout<<"i2: "<<i2.transpose()<<endl;

  Vector3d delta1 = s1 - f1;
  Vector3d delta2 = s2 - f2;

  double d1 =
      sqrt(pow(i1.dot(delta1), 2) + pow((i1.cross(delta1)).norm() - r1, 2));
  double d2 =
      sqrt(pow(i2.dot(delta2), 2) + pow((i2.cross(delta2)).norm() - r2, 2));
  //	cout<<"d1: "<<d1<<endl;
  //	cout<<"d2: "<<d2<<endl;

  VectorNd u(VectorNd::Zero(dof_active));
  u(0) = d1;
  u(1) = d2;

  return u;
}

Matrix2d sPrr2u1::compute_kinematic_Jacobian(const Math::VectorNd y) {

  Matrix2d J = Matrix2d::Zero();

  double roll = y(0);
  double pitch = y(1);

  double t2 = sin(pitch);
  double t3 = cos(roll);
  double t4 = sin(roll);
  double t5 = cos(pitch);
  double t9 = f1z * t3;
  double t10 = s1z * t5;
  double t11 = f1y * t4 * t5;
  double t12 = s1x * t2 * t3;
  double t6 = t9 - t10 + t11 - t12;
  double t14 = s1y * t5;
  double t15 = f1z * t4;
  double t16 = f1y * t3 * t5;
  double t17 = s1x * t2 * t4;
  double t7 = t14 + t15 - t16 - t17;
  double t20 = s1y * t3;
  double t21 = s1z * t4;
  double t8 = -f1y + t20 + t21;
  double t13 = t6 * t6;
  double t18 = t7 * t7;
  double t19 = t2 * t2;
  double t22 = t8 * t8;
  double t23 = t19 * t22;
  double t24 = t13 + t18 + t23;
  double t25 = t5 * t5;
  double t26 = f1z * f1z;
  double t27 = f1y * f1y;
  double t28 = t3 * t3;
  double t29 = t4 * t4;
  double t30 = s1y * s1y;
  double t31 = s1x * s1x;
  double t32 = s1z * s1z;
  double t33 = t26 * t28;
  double t34 = t25 * t30;
  double t35 = t25 * t32;
  double t36 = t19 * t27;
  double t37 = t26 * t29;
  double t38 = t25 * t27 * t28;
  double t39 = t25 * t27 * t29;
  double t40 = t19 * t28 * t31;
  double t41 = t19 * t28 * t30;
  double t42 = t19 * t29 * t31;
  double t43 = t19 * t29 * t32;
  double t44 = f1z * s1y * t4 * t5 * 2.0;
  double t45 = s1z * t2 * t3;
  double t46 = f1x + t45 - s1x * t5 - s1y * t2 * t4;
  double t47 = sqrt(t24);
  double t48 = r - t47;
  double t49 = 1.0 / sqrt(t24);
  double t50 = s1y * s1z * t3 * t4 * t19 * 2.0;
  double t51 = s1x * s1z * t2 * t3 * t5 * 2.0;
  double t59 = f1y * s1y * t3 * t25 * 2.0;
  double t60 = f1y * s1y * t3 * t19 * 2.0;
  double t61 = f1z * s1x * t2 * t28 * 2.0;
  double t62 = f1y * s1z * t4 * t25 * 2.0;
  double t63 = f1z * s1x * t2 * t29 * 2.0;
  double t64 = f1y * s1z * t4 * t19 * 2.0;
  double t65 = f1z * s1z * t3 * t5 * 2.0;
  double t52 = t33 + t34 + t35 + t36 + t37 + t38 + t39 + t40 + t41 + t42 + t43 +
               t44 + t50 + t51 - t59 - t60 - t61 - t62 - t63 - t64 - t65 -
               s1x * s1y * t2 * t4 * t5 * 2.0;
  double t53 = sqrt(t52);
  double t54 = t25 * t31;
  double t55 = f1x * f1x;
  double t56 = r * r;
  double t57 = t19 * t28 * t32;
  double t58 = t19 * t29 * t30;
  double t66 = f1x * s1z * t2 * t3 * 2.0;
  double t70 = f2z * t3;
  double t71 = s2z * t5;
  double t72 = f2y * t4 * t5;
  double t73 = s2x * t2 * t3;
  double t67 = t70 - t71 + t72 - t73;
  double t75 = s2y * t5;
  double t76 = f2z * t4;
  double t77 = f2y * t3 * t5;
  double t78 = s2x * t2 * t4;
  double t68 = t75 + t76 - t77 - t78;
  double t80 = s2y * t3;
  double t81 = s2z * t4;
  double t69 = -f2y + t80 + t81;
  double t74 = t67 * t67;
  double t79 = t68 * t68;
  double t82 = t69 * t69;
  double t83 = t19 * t82;
  double t84 = t74 + t79 + t83;
  double t85 = f2z * f2z;
  double t86 = f2y * f2y;
  double t87 = s2y * s2y;
  double t88 = s2x * s2x;
  double t89 = s2z * s2z;
  double t90 = t28 * t85;
  double t91 = t25 * t87;
  double t92 = t25 * t89;
  double t93 = t19 * t86;
  double t94 = t29 * t85;
  double t95 = t25 * t28 * t86;
  double t96 = t25 * t29 * t86;
  double t97 = t19 * t28 * t88;
  double t98 = t19 * t28 * t87;
  double t99 = t19 * t29 * t88;
  double t100 = t19 * t29 * t89;
  double t101 = f2z * s2y * t4 * t5 * 2.0;
  double t102 = s2z * t2 * t3;
  double t103 = f2x + t102 - s2x * t5 - s2y * t2 * t4;
  double t104 = sqrt(t84);
  double t105 = r - t104;
  double t106 = 1.0 / sqrt(t84);
  double t107 = s2y * s2z * t3 * t4 * t19 * 2.0;
  double t108 = s2x * s2z * t2 * t3 * t5 * 2.0;
  double t115 = f2y * s2y * t3 * t25 * 2.0;
  double t116 = f2y * s2y * t3 * t19 * 2.0;
  double t117 = f2z * s2x * t2 * t28 * 2.0;
  double t118 = f2y * s2z * t4 * t25 * 2.0;
  double t119 = f2z * s2x * t2 * t29 * 2.0;
  double t120 = f2y * s2z * t4 * t19 * 2.0;
  double t121 = f2z * s2z * t3 * t5 * 2.0;
  double t109 = t90 + t91 + t92 + t93 + t94 + t95 + t96 + t97 + t98 + t99 +
                t100 + t101 + t107 + t108 - t115 - t116 - t117 - t118 - t119 -
                t120 - t121 - s2x * s2y * t2 * t4 * t5 * 2.0;
  double t110 = sqrt(t109);
  double t111 = t25 * t88;
  double t112 = f2x * f2x;
  double t113 = t19 * t28 * t89;
  double t114 = t19 * t29 * t87;
  double t122 = f2x * s2z * t2 * t3 * 2.0;

  J(0, 0) = (t46 * (s1y * t2 * t3 + s1z * t2 * t4) * 2.0 +
             t48 * t49 *
                 (t7 * (t9 + t11 - t12) * 2.0 + t6 * (-t15 + t16 + t17) * 2.0 -
                  t8 * t19 * (s1y * t4 - s1z * t3) * 2.0)) *
            1.0 /
            sqrt(t33 + t34 + t35 + t36 + t37 + t38 + t39 + t40 + t41 + t42 +
                 t43 + t44 + t54 + t55 + t56 + t57 + t58 + t66 - r * t53 * 2.0 -
                 f1x * s1x * t5 * 2.0 - f1x * s1y * t2 * t4 * 2.0 -
                 f1y * s1y * t3 * t19 * 2.0 - f1y * s1y * t3 * t25 * 2.0 -
                 f1z * s1x * t2 * t28 * 2.0 - f1z * s1x * t2 * t29 * 2.0 -
                 f1y * s1z * t4 * t19 * 2.0 - f1y * s1z * t4 * t25 * 2.0 -
                 f1z * s1z * t3 * t5 * 2.0) *
            (-1.0 / 2.0);
  J(0, 1) = (t46 * (s1x * t2 - s1y * t4 * t5 + s1z * t3 * t5) * 2.0 +
             t48 * t49 *
                 (t7 * (s1y * t2 - f1y * t2 * t3 + s1x * t4 * t5) * 2.0 +
                  t6 * (-s1z * t2 + f1y * t2 * t4 + s1x * t3 * t5) * 2.0 -
                  t2 * t5 * t22 * 2.0)) *
            1.0 /
            sqrt(t33 + t34 + t35 + t36 + t37 + t38 + t39 + t40 + t41 + t42 +
                 t43 + t44 + t54 + t55 + t56 + t57 + t58 - t59 - t60 - t61 -
                 t62 - t63 - t64 - t65 + t66 - r * t53 * 2.0 -
                 f1x * s1x * t5 * 2.0 - f1x * s1y * t2 * t4 * 2.0) *
            (1.0 / 2.0);
  J(1, 0) =
      (t103 * (s2y * t2 * t3 + s2z * t2 * t4) * 2.0 +
       t105 * t106 *
           (t68 * (t70 + t72 - t73) * 2.0 + t67 * (-t76 + t77 + t78) * 2.0 -
            t19 * t69 * (s2y * t4 - s2z * t3) * 2.0)) *
      1.0 /
      sqrt(t56 + t90 + t91 + t92 + t93 + t94 + t95 + t96 + t97 + t98 + t99 +
           t100 + t101 + t111 + t112 + t113 + t114 + t122 - r * t110 * 2.0 -
           f2x * s2x * t5 * 2.0 - f2x * s2y * t2 * t4 * 2.0 -
           f2y * s2y * t3 * t19 * 2.0 - f2y * s2y * t3 * t25 * 2.0 -
           f2z * s2x * t2 * t28 * 2.0 - f2z * s2x * t2 * t29 * 2.0 -
           f2y * s2z * t4 * t19 * 2.0 - f2y * s2z * t4 * t25 * 2.0 -
           f2z * s2z * t3 * t5 * 2.0) *
      (-1.0 / 2.0);
  J(1, 1) = (t103 * (s2x * t2 - s2y * t4 * t5 + s2z * t3 * t5) * 2.0 +
             t105 * t106 *
                 (t68 * (s2y * t2 - f2y * t2 * t3 + s2x * t4 * t5) * 2.0 +
                  t67 * (-s2z * t2 + f2y * t2 * t4 + s2x * t3 * t5) * 2.0 -
                  t2 * t5 * t82 * 2.0)) *
            1.0 /
            sqrt(t56 + t90 + t91 + t92 + t93 + t94 + t95 + t96 + t97 + t98 +
                 t99 + t100 + t101 + t111 + t112 + t113 + t114 - t115 - t116 -
                 t117 - t118 - t119 - t120 - t121 + t122 - r * t110 * 2.0 -
                 f2x * s2x * t5 * 2.0 - f2x * s2y * t2 * t4 * 2.0) *
            (1.0 / 2.0);

  return J;
}

Matrix2d sPrr2u1::compute_kinematic_Jacobiandot(const Math::VectorNd y,
                                                const Math::VectorNd yd) {
  Matrix2d Jdot = Matrix2d::Zero();

  double roll = y(0);
  double pitch = y(1);

  // roll and pitch velocities
  double roll_dot = yd(0);
  double pitch_dot = yd(1);

  double t2 = cos(roll);
  double t3 = cos(pitch);
  double t4 = sin(roll);
  double t5 = sin(pitch);
  double t6 = f1z * roll_dot * t2;
  double t7 = f1y * roll_dot * t4;
  double t8 = t6 + t7;
  double t9 = f1z * t4;
  double t11 = f1y * t2;
  double t10 = s1y + t9 - t11;
  double t12 = f1z * t2;
  double t13 = f1y * t4;
  double t19 = s1z * t3;
  double t20 = s1x * t5;
  double t14 = t12 + t13 - t19 - t20;
  double t15 = t3 * t3;
  double t16 = t5 * t5;
  double t17 = t12 + t13;
  double t18 = t10 * t10;
  double t21 = s1x * s1x;
  double t22 = s1y * s1y;
  double t23 = s1z * s1z;
  double t24 = f1z * f1z;
  double t25 = t2 * t2;
  double t26 = t24 * t25;
  double t27 = t15 * t22;
  double t28 = t15 * t23;
  double t29 = f1y * f1y;
  double t30 = t4 * t4;
  double t31 = t29 * t30;
  double t32 = t16 * t21;
  double t33 = t16 * t22;
  double t34 = t15 * t25 * t29;
  double t35 = t16 * t25 * t29;
  double t36 = t15 * t24 * t30;
  double t37 = t16 * t24 * t30;
  double t38 = f1z * s1y * t4 * t15 * 2.0;
  double t39 = f1z * s1y * t4 * t16 * 2.0;
  double t40 = f1y * f1z * t2 * t4 * 2.0;
  double t41 = t15 * t18;
  double t42 = t16 * t18;
  double t43 = t14 * t14;
  double t44 = t41 + t42 + t43;
  double t45 = f1y * roll_dot * t2;
  double t46 = pitch_dot * s1z * t5;
  double t49 = f1z * roll_dot * t4;
  double t69 = pitch_dot * s1x * t3;
  double t47 = t45 + t46 - t49 - t69;
  double t48 = t9 - t11;
  double t50 = t45 - t49;
  double t51 = t15 * t21;
  double t52 = t16 * t23;
  double t53 = s1x * s1z * t3 * t5 * 2.0;
  double t59 = f1y * s1x * t4 * t5 * 2.0;
  double t60 = f1y * s1y * t2 * t15 * 2.0;
  double t61 = f1y * s1y * t2 * t16 * 2.0;
  double t62 = f1z * s1z * t2 * t3 * 2.0;
  double t63 = f1z * s1x * t2 * t5 * 2.0;
  double t64 = f1y * s1z * t3 * t4 * 2.0;
  double t65 = f1y * f1z * t2 * t4 * t15 * 2.0;
  double t66 = f1y * f1z * t2 * t4 * t16 * 2.0;
  double t54 = t26 + t27 + t28 + t31 + t32 + t33 + t34 + t35 + t36 + t37 + t38 +
               t39 + t40 + t53 - t59 - t60 - t61 - t62 - t63 - t64 - t65 - t66;
  double t55 = sqrt(t54);
  double t56 = f1x * f1x;
  double t57 = r * r;
  double t58 = f1x * s1z * t5 * 2.0;
  double t67 = sqrt(t44);
  double t68 = r - t67;
  double t70 = t14 * t47 * 2.0;
  double t71 = t8 * t10 * t15 * 2.0;
  double t72 = t8 * t10 * t16 * 2.0;
  double t73 = t70 + t71 + t72;
  double t74 = t10 * t15 * t17 * 2.0;
  double t75 = t10 * t16 * t17 * 2.0;
  double t79 = t14 * t48 * 2.0;
  double t76 = t74 + t75 - t79;
  double t95 = r * t55 * 2.0;
  double t96 = f1x * s1x * t3 * 2.0;
  double t77 = t26 + t27 + t28 + t31 + t32 + t33 + t34 + t35 + t36 + t37 + t38 +
               t39 + t40 + t51 + t52 + t56 + t57 + t58 - t59 - t60 - t61 - t62 -
               t63 - t64 - t65 - t66 - t95 - t96;
  double t78 = 1.0 / sqrt(t77);
  double t80 = 1.0 / sqrt(t44);
  double t81 = roll_dot * t2 * t4 * t29 * 2.0;
  double t82 = f1y * f1z * roll_dot * t25 * 2.0;
  double t83 = f1y * f1z * roll_dot * t15 * t30 * 2.0;
  double t84 = f1y * f1z * roll_dot * t16 * t30 * 2.0;
  double t85 = f1z * pitch_dot * s1z * t2 * t5 * 2.0;
  double t86 = f1z * roll_dot * s1z * t3 * t4 * 2.0;
  double t87 = f1y * pitch_dot * s1z * t4 * t5 * 2.0;
  double t88 = f1z * roll_dot * s1x * t4 * t5 * 2.0;
  double t89 = roll_dot * t2 * t4 * t15 * t24 * 2.0;
  double t90 = roll_dot * t2 * t4 * t16 * t24 * 2.0;
  double t91 = f1z * roll_dot * s1y * t2 * t15 * 2.0;
  double t92 = f1y * roll_dot * s1y * t4 * t15 * 2.0;
  double t93 = f1z * roll_dot * s1y * t2 * t16 * 2.0;
  double t94 = f1y * roll_dot * s1y * t4 * t16 * 2.0;
  double t97 = s1z * t5;
  double t98 = 1.0 / t44;
  double t103 = s1x * t3;
  double t99 = t97 - t103;
  double t100 = pitch_dot * s1z * t3;
  double t101 = pitch_dot * s1x * t5;
  double t102 = t100 + t101;
  double t104 = 1.0 / pow(t44, 3.0 / 2.0);
  double t105 = t19 + t20;
  double t106 = 1.0 / sqrt(t54);
  double t107 = pitch_dot * t3 * t5 * t21 * 2.0;
  double t108 = pitch_dot * s1x * s1z * t15 * 2.0;
  double t110 = roll_dot * t2 * t4 * t24 * 2.0;
  double t113 = f1y * f1z * roll_dot * t30 * 2.0;
  double t114 = f1y * f1z * roll_dot * t15 * t25 * 2.0;
  double t115 = f1y * f1z * roll_dot * t16 * t25 * 2.0;
  double t116 = f1z * pitch_dot * s1x * t2 * t3 * 2.0;
  double t117 = f1y * roll_dot * s1z * t2 * t3 * 2.0;
  double t118 = f1y * pitch_dot * s1x * t3 * t4 * 2.0;
  double t119 = f1y * roll_dot * s1x * t2 * t5 * 2.0;
  double t120 = roll_dot * t2 * t4 * t15 * t29 * 2.0;
  double t121 = roll_dot * t2 * t4 * t16 * t29 * 2.0;
  double t109 = t81 + t82 + t83 + t84 + t85 + t86 + t87 + t88 + t89 + t90 +
                t91 + t92 + t93 + t94 + t107 + t108 - t110 - t113 - t114 -
                t115 - t116 - t117 - t118 - t119 - t120 - t121 -
                pitch_dot * s1x * s1z * t16 * 2.0 -
                pitch_dot * t3 * t5 * t23 * 2.0;
  double t111 = f1x * pitch_dot * s1z * t3 * 2.0;
  double t112 = f1x * pitch_dot * s1x * t5 * 2.0;
  double t122 = 1.0 / pow(t77, 3.0 / 2.0);
  double t123 = f2z * roll_dot * t2;
  double t124 = f2y * roll_dot * t4;
  double t125 = t123 + t124;
  double t126 = f2z * t4;
  double t128 = f2y * t2;
  double t127 = s2y + t126 - t128;
  double t129 = f2z * t2;
  double t130 = f2y * t4;
  double t134 = s2z * t3;
  double t135 = s2x * t5;
  double t131 = t129 + t130 - t134 - t135;
  double t132 = t129 + t130;
  double t133 = t127 * t127;
  double t136 = s2x * s2x;
  double t137 = s2y * s2y;
  double t138 = s2z * s2z;
  double t139 = f2z * f2z;
  double t140 = t25 * t139;
  double t141 = t15 * t137;
  double t142 = t15 * t138;
  double t143 = f2y * f2y;
  double t144 = t30 * t143;
  double t145 = t16 * t136;
  double t146 = t16 * t137;
  double t147 = t15 * t25 * t143;
  double t148 = t16 * t25 * t143;
  double t149 = t15 * t30 * t139;
  double t150 = t16 * t30 * t139;
  double t151 = f2z * s2y * t4 * t15 * 2.0;
  double t152 = f2z * s2y * t4 * t16 * 2.0;
  double t153 = f2y * f2z * t2 * t4 * 2.0;
  double t154 = t15 * t133;
  double t155 = t16 * t133;
  double t156 = t131 * t131;
  double t157 = t154 + t155 + t156;
  double t158 = f2y * roll_dot * t2;
  double t159 = pitch_dot * s2z * t5;
  double t162 = f2z * roll_dot * t4;
  double t181 = pitch_dot * s2x * t3;
  double t160 = t158 + t159 - t162 - t181;
  double t161 = t126 - t128;
  double t163 = t158 - t162;
  double t164 = t15 * t136;
  double t165 = t16 * t138;
  double t166 = s2x * s2z * t3 * t5 * 2.0;
  double t171 = f2y * s2x * t4 * t5 * 2.0;
  double t172 = f2y * s2y * t2 * t15 * 2.0;
  double t173 = f2y * s2y * t2 * t16 * 2.0;
  double t174 = f2z * s2z * t2 * t3 * 2.0;
  double t175 = f2z * s2x * t2 * t5 * 2.0;
  double t176 = f2y * s2z * t3 * t4 * 2.0;
  double t177 = f2y * f2z * t2 * t4 * t15 * 2.0;
  double t178 = f2y * f2z * t2 * t4 * t16 * 2.0;
  double t167 = t140 + t141 + t142 + t144 + t145 + t146 + t147 + t148 + t149 +
                t150 + t151 + t152 + t153 + t166 - t171 - t172 - t173 - t174 -
                t175 - t176 - t177 - t178;
  double t168 = sqrt(t167);
  double t169 = f2x * f2x;
  double t170 = f2x * s2z * t5 * 2.0;
  double t179 = sqrt(t157);
  double t180 = r - t179;
  double t182 = t131 * t160 * 2.0;
  double t183 = t15 * t125 * t127 * 2.0;
  double t184 = t16 * t125 * t127 * 2.0;
  double t185 = t182 + t183 + t184;
  double t186 = t15 * t127 * t132 * 2.0;
  double t187 = t16 * t127 * t132 * 2.0;
  double t191 = t131 * t161 * 2.0;
  double t188 = t186 + t187 - t191;
  double t207 = r * t168 * 2.0;
  double t208 = f2x * s2x * t3 * 2.0;
  double t189 = t57 + t140 + t141 + t142 + t144 + t145 + t146 + t147 + t148 +
                t149 + t150 + t151 + t152 + t153 + t164 + t165 + t169 + t170 -
                t171 - t172 - t173 - t174 - t175 - t176 - t177 - t178 - t207 -
                t208;
  double t190 = 1.0 / sqrt(t189);
  double t192 = 1.0 / sqrt(t157);
  double t193 = roll_dot * t2 * t4 * t143 * 2.0;
  double t194 = f2y * f2z * roll_dot * t25 * 2.0;
  double t195 = f2y * f2z * roll_dot * t15 * t30 * 2.0;
  double t196 = f2y * f2z * roll_dot * t16 * t30 * 2.0;
  double t197 = f2z * pitch_dot * s2z * t2 * t5 * 2.0;
  double t198 = f2z * roll_dot * s2z * t3 * t4 * 2.0;
  double t199 = f2y * pitch_dot * s2z * t4 * t5 * 2.0;
  double t200 = f2z * roll_dot * s2x * t4 * t5 * 2.0;
  double t201 = roll_dot * t2 * t4 * t15 * t139 * 2.0;
  double t202 = roll_dot * t2 * t4 * t16 * t139 * 2.0;
  double t203 = f2z * roll_dot * s2y * t2 * t15 * 2.0;
  double t204 = f2y * roll_dot * s2y * t4 * t15 * 2.0;
  double t205 = f2z * roll_dot * s2y * t2 * t16 * 2.0;
  double t206 = f2y * roll_dot * s2y * t4 * t16 * 2.0;
  double t209 = s2z * t5;
  double t210 = 1.0 / t157;
  double t215 = s2x * t3;
  double t211 = t209 - t215;
  double t212 = pitch_dot * s2z * t3;
  double t213 = pitch_dot * s2x * t5;
  double t214 = t212 + t213;
  double t216 = 1.0 / pow(t157, 3.0 / 2.0);
  double t217 = t134 + t135;
  double t218 = 1.0 / sqrt(t167);
  double t219 = pitch_dot * t3 * t5 * t136 * 2.0;
  double t220 = pitch_dot * s2x * s2z * t15 * 2.0;
  double t222 = roll_dot * t2 * t4 * t139 * 2.0;
  double t225 = f2y * f2z * roll_dot * t30 * 2.0;
  double t226 = f2y * f2z * roll_dot * t15 * t25 * 2.0;
  double t227 = f2y * f2z * roll_dot * t16 * t25 * 2.0;
  double t228 = f2z * pitch_dot * s2x * t2 * t3 * 2.0;
  double t229 = f2y * roll_dot * s2z * t2 * t3 * 2.0;
  double t230 = f2y * pitch_dot * s2x * t3 * t4 * 2.0;
  double t231 = f2y * roll_dot * s2x * t2 * t5 * 2.0;
  double t232 = roll_dot * t2 * t4 * t15 * t143 * 2.0;
  double t233 = roll_dot * t2 * t4 * t16 * t143 * 2.0;
  double t221 = t193 + t194 + t195 + t196 + t197 + t198 + t199 + t200 + t201 +
                t202 + t203 + t204 + t205 + t206 + t219 + t220 - t222 - t225 -
                t226 - t227 - t228 - t229 - t230 - t231 - t232 - t233 -
                pitch_dot * s2x * s2z * t16 * 2.0 -
                pitch_dot * t3 * t5 * t138 * 2.0;
  double t223 = f2x * pitch_dot * s2z * t3 * 2.0;
  double t224 = f2x * pitch_dot * s2x * t5 * 2.0;
  double t234 = 1.0 / pow(t189, 3.0 / 2.0);

  Jdot(0, 0) =
      (t73 * t76 * t98 * 1.0 /
       sqrt(t26 + t27 + t28 + t31 + t32 + t33 + t34 + t35 + t36 + t37 + t38 +
            t39 + t40 + t51 + t52 + t56 + t57 + t58 - r * t55 * 2.0 -
            f1x * s1x * t3 * 2.0 - f1y * s1x * t4 * t5 * 2.0 -
            f1z * s1x * t2 * t5 * 2.0 - f1y * s1y * t2 * t15 * 2.0 -
            f1y * s1y * t2 * t16 * 2.0 - f1y * s1z * t3 * t4 * 2.0 -
            f1z * s1z * t2 * t3 * 2.0 - f1y * f1z * t2 * t4 * t15 * 2.0 -
            f1y * f1z * t2 * t4 * t16 * 2.0)) /
          4.0 -
      (t68 * t78 * t80 *
       (t8 * t14 * -2.0 - t47 * t48 * 2.0 + t8 * t15 * t17 * 2.0 +
        t8 * t16 * t17 * 2.0 + t10 * t15 * t50 * 2.0 + t10 * t16 * t50 * 2.0)) /
          2.0 +
      (t68 * t76 * t80 * t122 *
       (t81 + t82 + t83 + t84 + t85 + t86 + t87 + t88 + t89 + t90 + t91 + t92 +
        t93 + t94 + t111 + t112 - r * t106 * t109 -
        f1y * f1z * roll_dot * t30 * 2.0 - roll_dot * t2 * t4 * t24 * 2.0 -
        f1y * f1z * roll_dot * t15 * t25 * 2.0 -
        f1y * f1z * roll_dot * t16 * t25 * 2.0 -
        f1y * pitch_dot * s1x * t3 * t4 * 2.0 -
        f1z * pitch_dot * s1x * t2 * t3 * 2.0 -
        f1y * roll_dot * s1x * t2 * t5 * 2.0 -
        f1y * roll_dot * s1z * t2 * t3 * 2.0 -
        roll_dot * t2 * t4 * t15 * t29 * 2.0 -
        roll_dot * t2 * t4 * t16 * t29 * 2.0)) /
          4.0 +
      (t68 * t73 * t76 * t78 * t104) / 4.0;
  Jdot(0, 1) =
      (t78 *
       (t102 * t105 * 2.0 - (t46 - t69) * (f1x + t97 - s1x * t3) * 2.0 -
        t14 * t68 * t80 * t102 * 2.0 + t14 * t73 * t98 * t99 -
        t47 * t68 * t80 * t99 * 2.0 + t14 * t68 * t73 * t104 * (t97 - t103))) /
          2.0 -
      (t122 * (t105 * (f1x + t97 - t103) * 2.0 - t14 * t68 * t80 * t99 * 2.0) *
       (t81 + t82 + t83 + t84 + t85 + t86 + t87 + t88 + t89 + t90 + t91 + t92 +
        t93 + t94 - t110 + t111 + t112 - t113 - t114 - t115 - t116 - t117 -
        t118 - t119 - t120 - t121 - r * t106 * t109)) /
          4.0;
  Jdot(1, 0) =
      (t185 * t188 * t210 * 1.0 /
       sqrt(t57 + t140 + t141 + t142 + t144 + t145 + t146 + t147 + t148 + t149 +
            t150 + t151 + t152 + t153 + t164 + t165 + t169 + t170 -
            r * t168 * 2.0 - f2x * s2x * t3 * 2.0 - f2y * s2x * t4 * t5 * 2.0 -
            f2z * s2x * t2 * t5 * 2.0 - f2y * s2y * t2 * t15 * 2.0 -
            f2y * s2y * t2 * t16 * 2.0 - f2y * s2z * t3 * t4 * 2.0 -
            f2z * s2z * t2 * t3 * 2.0 - f2y * f2z * t2 * t4 * t15 * 2.0 -
            f2y * f2z * t2 * t4 * t16 * 2.0)) /
          4.0 -
      (t180 * t190 * t192 *
       (t125 * t131 * -2.0 - t160 * t161 * 2.0 + t15 * t125 * t132 * 2.0 +
        t16 * t125 * t132 * 2.0 + t15 * t127 * t163 * 2.0 +
        t16 * t127 * t163 * 2.0)) /
          2.0 +
      (t180 * t188 * t192 * t234 *
       (t193 + t194 + t195 + t196 + t197 + t198 + t199 + t200 + t201 + t202 +
        t203 + t204 + t205 + t206 + t223 + t224 - r * t218 * t221 -
        f2y * f2z * roll_dot * t30 * 2.0 - roll_dot * t2 * t4 * t139 * 2.0 -
        f2y * f2z * roll_dot * t15 * t25 * 2.0 -
        f2y * f2z * roll_dot * t16 * t25 * 2.0 -
        f2y * pitch_dot * s2x * t3 * t4 * 2.0 -
        f2z * pitch_dot * s2x * t2 * t3 * 2.0 -
        f2y * roll_dot * s2x * t2 * t5 * 2.0 -
        f2y * roll_dot * s2z * t2 * t3 * 2.0 -
        roll_dot * t2 * t4 * t15 * t143 * 2.0 -
        roll_dot * t2 * t4 * t16 * t143 * 2.0)) /
          4.0 +
      (t180 * t185 * t188 * t190 * t216) / 4.0;
  Jdot(1, 1) =
      (t190 *
       (t214 * t217 * 2.0 - (t159 - t181) * (f2x + t209 - s2x * t3) * 2.0 -
        t131 * t180 * t192 * t214 * 2.0 + t131 * t185 * t210 * t211 -
        t160 * t180 * t192 * t211 * 2.0 +
        t131 * t180 * t185 * t216 * (t209 - t215))) /
          2.0 -
      (t234 *
       (t217 * (f2x + t209 - t215) * 2.0 - t131 * t180 * t192 * t211 * 2.0) *
       (t193 + t194 + t195 + t196 + t197 + t198 + t199 + t200 + t201 + t202 +
        t203 + t204 + t205 + t206 - t222 + t223 + t224 - t225 - t226 - t227 -
        t228 - t229 - t230 - t231 - t232 - t233 - r * t218 * t221)) /
          4.0;

  return Jdot;
}

MatrixXd sPrr2u1::compute_loopclosure_Jacobian_UP_leg(
    const Math::VectorNd y, const Vector3d i_ee, const double r,
    const Vector3d f_ee, const Vector3d s, const Matrix3d RotMat_s) {
  MatrixXd J;
  J.setZero(3, 2);

  Matrix3d RotMat_x, RotMat_y, RotMat;

  double roll, pitch;
  roll = y(0);  // roll
  pitch = y(1); // pitch

  RotMat_x << 1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll);

  RotMat_y << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch);

  RotMat = RotMat_x * RotMat_y;
  //	cout<<"RotMat: "<<RotMat<<endl;

  Vector3d f;
  f = RotMat * f_ee;
  //	cout<<"f: "<<f.transpose()<<endl;

  Vector3d i;
  i = RotMat * i_ee;
  //	cout<<"i1: "<<i.transpose()<<endl;

  Vector3d delta = s - f;

  double l = sqrt(pow(i.dot(delta), 2) + pow((i.cross(delta)).norm() - r, 2));
  //	cout<<"d1: "<<l<<endl;

  Vector3d fk_unit = (delta - i.dot(delta) * i);

  Vector3d k = f + r * fk_unit / fk_unit.norm();
  //	cout<<"k1: "<<k.transpose()<<endl;

  // express kisi in si frame
  Vector3d ks = RotMat_s * (k - s);

  double X_s = ks(0);
  double Y_s = ks(1);
  double Z_s = ks(2);

  double roll_u = atan2(-Y_s, Z_s);
  double sin_pitch_s = X_s / l;
  double pitch_u = atan2(sin_pitch_s, sqrt(1 - pow(sin_pitch_s, 2)));

  double sx, sy, sz, nx, ny, nz, ax, ay,
      az; // elements of rotation matrix to the s frame
  // fixed params(rotation matrix transformation to shank joint frame)
  sx = RotMat_s(0, 0);
  sy = RotMat_s(1, 0);
  sz = RotMat_s(2, 0);

  nx = RotMat_s(0, 1);
  ny = RotMat_s(1, 1);
  nz = RotMat_s(2, 1);

  ax = RotMat_s(0, 2);
  ay = RotMat_s(1, 2);
  az = RotMat_s(2, 2);

  double shx, shy, shz, fx, fy, fz;

  // shank point s (in base frame) and f(in end effector frame)
  shx = s(0);
  shy = s(1);
  shz = s(2);

  fx = f_ee(0);
  fy = f_ee(1);
  fz = f_ee(2);

  // generated code from matlab
  double t2 = cos(pitch);
  double t3 = sin(pitch);
  double t4 = cos(roll);
  double t5 = sin(roll);
  double t7 = t2 * t2;
  double t27 = shx * t7;
  double t28 = fz * t3;
  double t29 = shz * t2 * t3 * t4;
  double t30 = shy * t2 * t3 * t5;
  double t6 = -shx + t27 + t28 - t29 + t30;
  double t8 = t4 * t4;
  double t11 = shz * t8;
  double t12 = fy * t5;
  double t13 = shz * t7 * t8;
  double t14 = fz * t2 * t4;
  double t15 = shy * t4 * t5;
  double t16 = shx * t2 * t3 * t4;
  double t17 = shy * t4 * t5 * t7;
  double t9 = shz - t11 - t12 + t13 - t14 + t15 + t16 - t17;
  double t18 = shy * t8;
  double t19 = fy * t4;
  double t20 = shy * t7 * t8;
  double t22 = fz * t2 * t5;
  double t23 = shx * t2 * t3 * t5;
  double t31 = shy * t7;
  double t32 = shz * t4 * t5;
  double t33 = shz * t4 * t5 * t7;
  double t10 = -t18 + t19 + t20 - t22 + t23 - t31 - t32 + t33;
  double t21 = t5 * t5;
  double t24 = shy * t2 * t3 * t4;
  double t25 = shz * t2 * t3 * t5;
  double t26 = t24 + t25;
  double t34 = t6 * t6;
  double t35 = t9 * t9;
  double t36 = t18 - t19 - t20 + t22 - t23 + t31 + t32 - t33;
  double t37 = t36 * t36;
  double t38 = t34 + t35 + t37;
  double t39 = shz * t7 * t21;
  double t40 = shy * t4 * t5 * t7 * 2.0;
  double t46 = shz * t21;
  double t47 = shy * t4 * t5 * 2.0;
  double t41 = t11 + t12 - t13 + t14 - t16 + t39 + t40 - t46 - t47;
  double t42 = shy * t21;
  double t43 = shz * t4 * t5 * t7 * 2.0;
  double t52 = shy * t7 * t21;
  double t53 = shz * t4 * t5 * 2.0;
  double t44 = -t18 + t19 + t20 - t22 + t23 + t42 + t43 - t52 - t53;
  double t45 = t6 * t26 * 2.0;
  double t48 = t36 * t41 * 2.0;
  double t54 = t9 * t44 * 2.0;
  double t49 = t45 + t48 - t54;
  double t50 = 1.0 / pow(t38, 3.0 / 2.0);
  double t51 = 1.0 / sqrt(t38);
  double t55 = 1.0 / l;
  double t56 = cos(pitch_u);
  double t57 = 1.0 / t56;
  double t58 = r * t36 * t49 * t50 * (1.0 / 2.0);
  double t99 = fx * t3 * t4;
  double t100 = r * t41 * t51;
  double t59 = t12 + t14 + t58 - t99 - t100;
  double t60 = r * t44 * t51;
  double t61 = r * t9 * t49 * t50 * (1.0 / 2.0);
  double t101 = fx * t3 * t5;
  double t62 = -t19 + t22 + t60 + t61 - t101;
  double t63 = cos(roll_u);
  double t64 = fz * t3 * t4;
  double t65 = t3 * t3;
  double t66 = shx * t4 * t7;
  double t67 = shy * t2 * t3 * t4 * t5 * 2.0;
  double t81 = shx * t4 * t65;
  double t82 = shz * t2 * t3 * t8 * 2.0;
  double t68 = t64 + t66 + t67 - t81 - t82;
  double t69 = shy * t2 * t3 * 2.0;
  double t70 = fz * t3 * t5;
  double t71 = shx * t5 * t7;
  double t78 = shx * t5 * t65;
  double t79 = shy * t2 * t3 * t8 * 2.0;
  double t80 = shz * t2 * t3 * t4 * t5 * 2.0;
  double t72 = t69 + t70 + t71 - t78 - t79 - t80;
  double t73 = fz * t2;
  double t74 = shy * t5 * t7;
  double t75 = shz * t4 * t65;
  double t85 = shx * t2 * t3 * 2.0;
  double t86 = shz * t4 * t7;
  double t87 = shy * t5 * t65;
  double t76 = t73 + t74 + t75 - t85 - t86 - t87;
  double t77 = t6 * t76 * 2.0;
  double t83 = t9 * t68 * 2.0;
  double t88 = t36 * t72 * 2.0;
  double t84 = t77 + t83 - t88;
  double t89 = sin(roll_u);
  double t90 = fx * t2 * t4;
  double t91 = r * t9 * t50 * t84 * (1.0 / 2.0);
  double t110 = r * t51 * t68;
  double t92 = t64 + t90 + t91 - t110;
  double t93 = fx * t2 * t5;
  double t111 = r * t51 * t72;
  double t112 = r * t36 * t50 * t84 * (1.0 / 2.0);
  double t94 = t70 + t93 - t111 - t112;
  double t95 = r * t6 * t50 * t84 * (1.0 / 2.0);
  double t113 = fx * t3;
  double t114 = r * t51 * t76;
  double t96 = t73 + t95 - t113 - t114;
  double t97 = r * t26 * t51;
  double t102 = r * t6 * t49 * t50 * (1.0 / 2.0);
  double t98 = t97 - t102;
  double t103 = sz * t98;
  double t104 = nz * t59;
  double t105 = az * t62;
  double t106 = t103 + t104 + t105;
  double t107 = sin(pitch_u);
  double t108 = ny * t59;
  double t109 = ay * t62;
  double t115 = nz * t94;
  double t116 = sz * t96;
  double t128 = az * t92;
  double t117 = t115 + t116 - t128;
  double t118 = ny * t94;
  double t119 = sy * t96;
  double t120 = sx * t98;
  double t121 = nx * t59;
  double t122 = ax * t62;
  double t123 = t120 + t121 + t122;
  double t124 = sy * t98;
  double t125 = t108 + t109 + t124;
  double t126 = nx * t94;
  double t127 = sx * t96;
  double t129 = t118 + t119 - ay * t92;

  J(0, 0) = t55 * t57 * t63 *
                (t108 + t109 +
                 sy * (r * t26 * 1.0 / sqrt(t34 + t35 + t10 * t10) -
                       r * t6 * t49 * t50 * (1.0 / 2.0))) +
            t55 * t57 * t89 * t106;
  J(0, 1) = -t55 * t57 * t63 * t129 - t55 * t57 * t89 * t117;
  J(1, 0) =
      -t55 * t56 * t123 + t55 * t63 * t106 * t107 - t55 * t89 * t107 * t125;
  J(1, 1) = t55 * t56 * (t126 + t127 - ax * t92) - t55 * t63 * t107 * t117 +
            t55 * t89 * t107 * t129;
  J(2, 0) = -t107 * t123 - t56 * t63 * t106 + t56 * t89 * t125;
  J(2, 1) = t107 * (t126 + t127 - ax * t92) + t56 * t63 * (t115 + t116 - t128) -
            t56 * t89 * t129;

  return J;
}

MatrixXd sPrr2u1::compute_loopclosure_Jacobiandot_UP_leg(
    const Math::VectorNd y, const Math::VectorNd yd, const Vector3d i_ee,
    const double r, const Vector3d f_ee, const Vector3d s,
    const Matrix3d RotMat_s) {
  MatrixXd J;
  J.setZero(3, 2);

  Matrix3d RotMat_x, RotMat_y, RotMat;

  double roll, pitch;
  roll = y(0);  // roll
  pitch = y(1); // pitch

  RotMat_x << 1, 0, 0, 0, cos(roll), -sin(roll), 0, sin(roll), cos(roll);

  RotMat_y << cos(pitch), 0, sin(pitch), 0, 1, 0, -sin(pitch), 0, cos(pitch);

  RotMat = RotMat_x * RotMat_y;
  //	cout<<"RotMat: "<<RotMat<<endl;

  Vector3d f;
  f = RotMat * f_ee;
  //	cout<<"f: "<<f.transpose()<<endl;

  Vector3d i;
  i = RotMat * i_ee;
  //	cout<<"i1: "<<i.transpose()<<endl;

  Vector3d delta = s - f;

  double l = sqrt(pow(i.dot(delta), 2) + pow((i.cross(delta)).norm() - r, 2));
  //	cout<<"d1: "<<l<<endl;

  Vector3d fk_unit = (delta - i.dot(delta) * i);

  Vector3d k = f + r * fk_unit / fk_unit.norm();
  //	cout<<"k1: "<<k.transpose()<<endl;

  // express kisi in si frame
  Vector3d ks = RotMat_s * (k - s);

  double X_s = ks(0);
  double Y_s = ks(1);
  double Z_s = ks(2);

  double roll_u = atan2(-Y_s, Z_s);
  double sin_pitch_s = X_s / l;
  double pitch_u = atan2(sin_pitch_s, sqrt(1 - pow(sin_pitch_s, 2)));

  double sx, sy, sz, nx, ny, nz, ax, ay,
      az; // elements of rotation matrix to the s frame
  // fixed params(rotation matrix transformation to shank joint frame)
  sx = RotMat_s(0, 0);
  sy = RotMat_s(1, 0);
  sz = RotMat_s(2, 0);

  nx = RotMat_s(0, 1);
  ny = RotMat_s(1, 1);
  nz = RotMat_s(2, 1);

  ax = RotMat_s(0, 2);
  ay = RotMat_s(1, 2);
  az = RotMat_s(2, 2);

  double shx, shy, shz, fx, fy, fz;

  // shank point s (in base frame) and f(in end effector frame)
  shx = s(0);
  shy = s(1);
  shz = s(2);

  fx = f_ee(0);
  fy = f_ee(1);
  fz = f_ee(2);

  // roll and pitch velocities
  double roll_dot = yd(0);
  double pitch_dot = yd(1);

  Vector3d qd =
      compute_loopclosure_Jacobian_UP_leg(y, i_ee, r, f_ee, s, RotMat_s) * yd;

  double roll_u_dot = qd(0);
  double pitch_u_dot = qd(1);
  double l_dot = qd(2);

  double t2 = cos(pitch);
  double t3 = cos(roll);
  double t4 = sin(pitch);
  double t6 = sin(roll);
  double t7 = t2 * t2;
  double t13 = shx * t7;
  double t14 = fz * t4;
  double t15 = shz * t2 * t3 * t4;
  double t16 = shy * t2 * t4 * t6;
  double t5 = -shx + t13 + t14 - t15 + t16;
  double t8 = t3 * t3;
  double t18 = shz * t8;
  double t19 = fy * t6;
  double t20 = shz * t7 * t8;
  double t21 = fz * t2 * t3;
  double t22 = shy * t3 * t6;
  double t23 = shx * t2 * t3 * t4;
  double t24 = shy * t3 * t6 * t7;
  double t9 = shz - t18 - t19 + t20 - t21 + t22 + t23 - t24;
  double t26 = shy * t7;
  double t27 = shy * t8;
  double t28 = fy * t3;
  double t29 = shy * t7 * t8;
  double t30 = fz * t2 * t6;
  double t31 = shz * t3 * t6;
  double t32 = shx * t2 * t4 * t6;
  double t33 = shz * t3 * t6 * t7;
  double t10 = -t26 - t27 + t28 + t29 - t30 - t31 + t32 + t33;
  double t11 = fy * roll_dot * t3;
  double t12 = t6 * t6;
  double t17 = t5 * t5;
  double t25 = t9 * t9;
  double t34 = t26 + t27 - t28 - t29 + t30 + t31 - t32 - t33;
  double t35 = t4 * t4;
  double t36 = roll_dot * shy * t8 * 2.0;
  double t37 = roll_dot * shz * t3 * t6 * 4.0;
  double t38 = pitch_dot * shx * t3 * t7;
  double t39 = roll_dot * shy * t7 * t12 * 2.0;
  double t40 = fz * pitch_dot * t3 * t4;
  double t41 = fz * roll_dot * t2 * t6;
  double t42 = pitch_dot * shz * t2 * t4 * t12 * 2.0;
  double t43 = pitch_dot * shy * t2 * t3 * t4 * t6 * 4.0;
  double t45 = pitch_dot * shx * t3 * t35;
  double t46 = pitch_dot * shz * t2 * t4 * t8 * 2.0;
  double t47 = roll_dot * shx * t2 * t4 * t6;
  double t115 = roll_dot * shy * t12 * 2.0;
  double t116 = roll_dot * shy * t7 * t8 * 2.0;
  double t117 = roll_dot * shz * t3 * t6 * t7 * 4.0;
  double t44 = -t11 + t36 + t37 + t38 + t39 + t40 + t41 + t42 + t43 - t45 -
               t46 - t47 - t115 - t116 - t117;
  double t48 = fy * roll_dot * t6;
  double t49 = pitch_dot * shx * t6 * t35;
  double t50 = fz * roll_dot * t2 * t3;
  double t51 = pitch_dot * shy * t2 * t4 * t8 * 2.0;
  double t52 = roll_dot * shz * t8;
  double t53 = roll_dot * shz * t7 * t12;
  double t54 = roll_dot * shy * t3 * t6 * t7 * 2.0;
  double t55 = pitch_dot * shz * t2 * t3 * t4 * t6 * 2.0;
  double t81 = roll_dot * shz * t12;
  double t82 = roll_dot * shy * t3 * t6 * 2.0;
  double t83 = pitch_dot * shx * t6 * t7;
  double t84 = roll_dot * shz * t7 * t8;
  double t85 = pitch_dot * shy * t2 * t4 * 2.0;
  double t86 = fz * pitch_dot * t4 * t6;
  double t87 = roll_dot * shx * t2 * t3 * t4;
  double t56 = t48 + t49 + t50 + t51 + t52 + t53 + t54 + t55 - t81 - t82 - t83 -
               t84 - t85 - t86 - t87;
  double t57 = fz * pitch_dot * t2;
  double t58 = pitch_dot * shy * t6 * t7;
  double t59 = pitch_dot * shz * t3 * t35;
  double t60 = roll_dot * shy * t2 * t3 * t4;
  double t61 = roll_dot * shz * t2 * t4 * t6;
  double t89 = pitch_dot * shz * t3 * t7;
  double t90 = pitch_dot * shy * t6 * t35;
  double t91 = pitch_dot * shx * t2 * t4 * 2.0;
  double t62 = t57 + t58 + t59 + t60 + t61 - t89 - t90 - t91;
  double t63 = roll_dot * shy * t12;
  double t64 = roll_dot * shy * t7 * t8;
  double t65 = roll_dot * shz * t3 * t6 * t7 * 2.0;
  double t93 = roll_dot * shy * t8;
  double t94 = roll_dot * shz * t3 * t6 * 2.0;
  double t95 = roll_dot * shy * t7 * t12;
  double t96 = pitch_dot * shy * t2 * t3 * t4 * t6 * 2.0;
  double t66 = t11 - t38 - t40 - t41 + t45 + t46 + t47 + t63 + t64 + t65 - t93 -
               t94 - t95 - t96;
  double t67 = t34 * t34;
  double t68 = t17 + t25 + t67;
  double t69 = 1.0 / pow(t68, 3.0 / 2.0);
  double t70 = shz * t7 * t12;
  double t71 = shy * t3 * t6 * t7 * 2.0;
  double t79 = shz * t12;
  double t80 = shy * t3 * t6 * 2.0;
  double t72 = t18 + t19 - t20 + t21 - t23 + t70 + t71 - t79 - t80;
  double t73 = shy * t12;
  double t74 = shz * t3 * t6 * t7 * 2.0;
  double t98 = shy * t7 * t12;
  double t99 = shz * t3 * t6 * 2.0;
  double t75 = -t27 + t28 + t29 - t30 + t32 + t73 + t74 - t98 - t99;
  double t76 = shy * t2 * t3 * t4;
  double t77 = shz * t2 * t4 * t6;
  double t78 = t76 + t77;
  double t88 = t34 * t56 * 2.0;
  double t92 = t5 * t62 * 2.0;
  double t107 = t9 * t66 * 2.0;
  double t97 = t88 + t92 - t107;
  double t100 = t5 * t78 * 2.0;
  double t101 = t34 * t72 * 2.0;
  double t125 = t9 * t75 * 2.0;
  double t102 = t100 + t101 - t125;
  double t103 = pitch_dot * shy * t3 * t7;
  double t104 = pitch_dot * shz * t6 * t7;
  double t105 = roll_dot * shz * t2 * t3 * t4;
  double t118 = pitch_dot * shy * t3 * t35;
  double t119 = pitch_dot * shz * t6 * t35;
  double t120 = roll_dot * shy * t2 * t4 * t6;
  double t106 = t103 + t104 + t105 - t118 - t119 - t120;
  double t108 = roll_dot * shz * t8 * 2.0;
  double t109 = roll_dot * shz * t7 * t12 * 2.0;
  double t110 = roll_dot * shy * t3 * t6 * t7 * 4.0;
  double t111 = pitch_dot * shz * t2 * t3 * t4 * t6 * 4.0;
  double t128 = roll_dot * shz * t12 * 2.0;
  double t129 = roll_dot * shy * t3 * t6 * 4.0;
  double t130 = roll_dot * shz * t7 * t8 * 2.0;
  double t131 = pitch_dot * shy * t2 * t4 * t12 * 2.0;
  double t112 = t48 + t49 + t50 + t51 - t83 - t86 - t87 + t108 + t109 + t110 +
                t111 - t128 - t129 - t130 - t131;
  double t113 = t9 * t112 * 2.0;
  double t114 = t62 * t78 * 2.0;
  double t121 = t5 * t106 * 2.0;
  double t122 = t66 * t75 * 2.0;
  double t123 = t56 * t72 * 2.0;
  double t132 = t34 * t44 * 2.0;
  double t124 = t113 + t114 + t121 + t122 + t123 - t132;
  double t126 = 1.0 / pow(t68, 5.0 / 2.0);
  double t127 = 1.0 / sqrt(t68);
  double t133 = 1.0 / l;
  double t134 = cos(pitch_u);
  double t135 = 1.0 / t134;
  double t136 = fx * roll_dot * t4 * t6;
  double t137 = r * t34 * t69 * t124 * (1.0 / 2.0);
  double t138 = r * t69 * t72 * t97 * (1.0 / 2.0);
  double t139 = r * t56 * t69 * t102 * (1.0 / 2.0);
  double t140 = r * t69 * t78 * t97 * (1.0 / 2.0);
  double t141 = r * t5 * t69 * t124 * (1.0 / 2.0);
  double t142 = r * t62 * t69 * t102 * (1.0 / 2.0);
  double t279 = r * t106 * t127;
  double t280 = r * t5 * t97 * t102 * t126 * (3.0 / 4.0);
  double t143 = t140 + t141 + t142 - t279 - t280;
  double t144 = r * t112 * t127;
  double t145 = r * t66 * t69 * t102 * (1.0 / 2.0);
  double t146 = r * t69 * t75 * t97 * (1.0 / 2.0);
  double t147 = fx * pitch_dot * t2 * t6;
  double t148 = fx * roll_dot * t3 * t4;
  double t149 = r * t9 * t97 * t102 * t126 * (3.0 / 4.0);
  double t281 = r * t9 * t69 * t124 * (1.0 / 2.0);
  double t150 =
      -t48 - t50 + t86 + t144 + t145 + t146 + t147 + t148 + t149 - t281;
  double t151 = sin(roll_u);
  double t152 = r * t78 * t127;
  double t161 = r * t5 * t69 * t102 * (1.0 / 2.0);
  double t153 = t152 - t161;
  double t154 = r * t34 * t69 * t102 * (1.0 / 2.0);
  double t163 = fx * t3 * t4;
  double t164 = r * t72 * t127;
  double t155 = t19 + t21 + t154 - t163 - t164;
  double t156 = r * t75 * t127;
  double t157 = r * t9 * t69 * t102 * (1.0 / 2.0);
  double t166 = fx * t4 * t6;
  double t158 = -t28 + t30 + t156 + t157 - t166;
  double t159 = 1.0 / (l * l);
  double t160 = cos(roll_u);
  double t162 = sy * t153;
  double t165 = ny * t155;
  double t167 = ay * t158;
  double t168 = t162 + t165 + t167;
  double t169 = sz * t153;
  double t170 = nz * t155;
  double t171 = az * t158;
  double t172 = t169 + t170 + t171;
  double t173 = 1.0 / (t134 * t134);
  double t174 = sin(pitch_u);
  double t175 = fz * t2;
  double t176 = shy * t6 * t7;
  double t177 = shz * t3 * t35;
  double t196 = shx * t2 * t4 * 2.0;
  double t197 = shz * t3 * t7;
  double t198 = shy * t6 * t35;
  double t178 = t175 + t176 + t177 - t196 - t197 - t198;
  double t179 = fz * roll_dot * t4 * t6;
  double t180 = roll_dot * shx * t6 * t35;
  double t181 = pitch_dot * shz * t8 * t35 * 2.0;
  double t182 = fz * pitch_dot * t2 * t3;
  double t183 = pitch_dot * shy * t3 * t6 * t7 * 2.0;
  double t184 = roll_dot * shy * t2 * t4 * t8 * 2.0;
  double t185 = roll_dot * shz * t2 * t3 * t4 * t6 * 4.0;
  double t186 = shy * t2 * t4 * 2.0;
  double t187 = fz * t4 * t6;
  double t188 = shx * t6 * t7;
  double t200 = shx * t6 * t35;
  double t201 = shy * t2 * t4 * t8 * 2.0;
  double t202 = shz * t2 * t3 * t4 * t6 * 2.0;
  double t189 = t186 + t187 + t188 - t200 - t201 - t202;
  double t190 = fz * t3 * t4;
  double t191 = shx * t3 * t7;
  double t192 = shy * t2 * t3 * t4 * t6 * 2.0;
  double t194 = shx * t3 * t35;
  double t195 = shz * t2 * t4 * t8 * 2.0;
  double t193 = t190 + t191 + t192 - t194 - t195;
  double t199 = t5 * t178 * 2.0;
  double t203 = t9 * t193 * 2.0;
  double t238 = t34 * t189 * 2.0;
  double t204 = t199 + t203 - t238;
  double t205 = pitch_dot * shy * t7 * 2.0;
  double t206 = roll_dot * shx * t3 * t7;
  double t207 = pitch_dot * shy * t8 * t35 * 2.0;
  double t208 = fz * pitch_dot * t2 * t6;
  double t209 = fz * roll_dot * t3 * t4;
  double t210 = pitch_dot * shz * t3 * t6 * t35 * 2.0;
  double t211 = roll_dot * shz * t2 * t4 * t12 * 2.0;
  double t212 = roll_dot * shy * t2 * t3 * t4 * t6 * 4.0;
  double t230 = pitch_dot * shy * t35 * 2.0;
  double t231 = roll_dot * shx * t3 * t35;
  double t232 = pitch_dot * shy * t7 * t8 * 2.0;
  double t233 = pitch_dot * shz * t3 * t6 * t7 * 2.0;
  double t234 = roll_dot * shz * t2 * t4 * t8 * 2.0;
  double t235 = pitch_dot * shx * t2 * t4 * t6 * 4.0;
  double t213 = t205 + t206 + t207 + t208 + t209 + t210 + t211 + t212 - t230 -
                t231 - t232 - t233 - t234 - t235;
  double t214 = roll_dot * shx * t6 * t7;
  double t215 = pitch_dot * shz * t7 * t8 * 2.0;
  double t216 = pitch_dot * shy * t3 * t6 * t35 * 2.0;
  double t217 = roll_dot * shy * t2 * t4 * t12 * 2.0;
  double t218 = pitch_dot * shx * t2 * t3 * t4 * 4.0;
  double t219 = t179 - t180 - t181 - t182 - t183 - t184 - t185 + t214 + t215 +
                t216 + t217 + t218;
  double t220 = t9 * t219 * 2.0;
  double t221 = t56 * t189 * 2.0;
  double t222 = t66 * t193 * 2.0;
  double t223 = fz * pitch_dot * t4;
  double t224 = pitch_dot * shx * t7 * 2.0;
  double t225 = roll_dot * shy * t3 * t35;
  double t226 = roll_dot * shz * t6 * t35;
  double t227 = pitch_dot * shy * t2 * t4 * t6 * 4.0;
  double t239 = pitch_dot * shx * t35 * 2.0;
  double t240 = roll_dot * shy * t3 * t7;
  double t241 = roll_dot * shz * t6 * t7;
  double t242 = pitch_dot * shz * t2 * t3 * t4 * 4.0;
  double t228 = t223 + t224 + t225 + t226 + t227 - t239 - t240 - t241 - t242;
  double t229 = t5 * t228 * 2.0;
  double t236 = t34 * t213 * 2.0;
  double t243 = t62 * t178 * 2.0;
  double t237 = t220 + t221 + t222 + t229 + t236 - t243;
  double t244 = r * t66 * t69 * t204 * (1.0 / 2.0);
  double t245 = r * t9 * t69 * t237 * (1.0 / 2.0);
  double t246 = fx * pitch_dot * t3 * t4;
  double t247 = fx * roll_dot * t2 * t6;
  double t248 = r * t9 * t97 * t126 * t204 * (3.0 / 4.0);
  double t293 = r * t127 * t219;
  double t294 = r * t69 * t97 * t193 * (1.0 / 2.0);
  double t249 = t179 - t182 + t244 + t245 + t246 + t247 + t248 - t293 - t294;
  double t250 = r * t34 * t69 * t237 * (1.0 / 2.0);
  double t251 = r * t69 * t97 * t189 * (1.0 / 2.0);
  double t252 = fx * roll_dot * t2 * t3;
  double t253 = r * t34 * t97 * t126 * t204 * (3.0 / 4.0);
  double t295 = r * t127 * t213;
  double t296 = r * t56 * t69 * t204 * (1.0 / 2.0);
  double t297 = fx * pitch_dot * t4 * t6;
  double t254 = t208 + t209 + t250 + t251 + t252 + t253 - t295 - t296 - t297;
  double t255 = fx * pitch_dot * t2;
  double t256 = r * t5 * t69 * t237 * (1.0 / 2.0);
  double t257 = r * t5 * t97 * t126 * t204 * (3.0 / 4.0);
  double t298 = r * t127 * t228;
  double t299 = r * t69 * t97 * t178 * (1.0 / 2.0);
  double t300 = r * t62 * t69 * t204 * (1.0 / 2.0);
  double t258 = t223 + t255 + t256 + t257 - t298 - t299 - t300;
  double t259 = fx * t2 * t3;
  double t260 = r * t9 * t69 * t204 * (1.0 / 2.0);
  double t266 = r * t127 * t193;
  double t261 = t190 + t259 + t260 - t266;
  double t262 = fx * t2 * t6;
  double t267 = r * t127 * t189;
  double t268 = r * t34 * t69 * t204 * (1.0 / 2.0);
  double t263 = t187 + t262 - t267 - t268;
  double t264 = r * t5 * t69 * t204 * (1.0 / 2.0);
  double t270 = fx * t4;
  double t271 = r * t127 * t178;
  double t265 = t175 + t264 - t270 - t271;
  double t269 = nz * t263;
  double t272 = sz * t265;
  double t273 = ny * t263;
  double t274 = sy * t265;
  double t311 = ay * t261;
  double t275 = t273 + t274 - t311;
  double t310 = az * t261;
  double t276 = t269 + t272 - t310;
  double t277 = r * t44 * t127;
  double t282 = fx * pitch_dot * t2 * t3;
  double t283 = r * t34 * t97 * t102 * t126 * (3.0 / 4.0);
  double t278 =
      t11 - t40 - t41 + t136 + t137 + t138 + t139 + t277 - t282 - t283;
  double t284 = sz * t143;
  double t285 = az * t150;
  double t315 = nz * t278;
  double t286 = t284 + t285 - t315;
  double t287 = sy * t143;
  double t288 = ay * t150;
  double t289 = sx * t153;
  double t290 = nx * t155;
  double t291 = ax * t158;
  double t292 = t289 + t290 + t291;
  double t301 = nx * t263;
  double t302 = sx * t265;
  double t320 = ax * t261;
  double t303 = t301 + t302 - t320;
  double t304 = az * t249;
  double t305 = nz * t254;
  double t321 = sz * t258;
  double t306 = t304 + t305 - t321;
  double t307 = ay * t249;
  double t308 = ny * t254;
  double t322 = sy * t258;
  double t309 = t307 + t308 - t322;
  double t312 = sx * t143;
  double t313 = ax * t150;
  double t314 = t312 + t313 - nx * t278;
  double t316 = t287 + t288 - ny * t278;
  double t317 = ax * t249;
  double t318 = nx * t254;
  double t319 = t317 + t318 - sx * t258;

  J(0, 0) = -t133 * t135 * t151 * t286 -
            t133 * t135 * t160 *
                (t287 + t288 -
                 ny * (t11 - t40 - t41 + t136 + t137 + t138 + t139 +
                       r * t44 * 1.0 / sqrt(t17 + t25 + t10 * t10) -
                       fx * pitch_dot * t2 * t3 -
                       r * t34 * t97 * t102 * t126 * (3.0 / 4.0))) -
            l_dot * t135 * t151 * t159 * t172 -
            l_dot * t135 * t159 * t160 * t168 -
            roll_u_dot * t133 * t135 * t151 * t168 +
            roll_u_dot * t133 * t135 * t160 * t172 +
            pitch_u_dot * t133 * t151 * t172 * t173 * t174 +
            pitch_u_dot * t133 * t160 * t168 * t173 * t174;
  J(0, 1) = -t133 * t135 * t151 * t306 - t133 * t135 * t160 * t309 +
            l_dot * t135 * t151 * t159 * t276 +
            l_dot * t135 * t159 * t160 * t275 +
            roll_u_dot * t133 * t135 * t151 * t275 -
            roll_u_dot * t133 * t135 * t160 * t276 -
            pitch_u_dot * t133 * t151 * t173 * t174 * t276 -
            pitch_u_dot * t133 * t160 * t173 * t174 * t275;
  J(1, 0) = t133 * t134 * t314 + l_dot * t134 * t159 * t292 +
            pitch_u_dot * t133 * t174 * t292 - t133 * t160 * t174 * t286 +
            t133 * t151 * t174 * t316 + l_dot * t151 * t159 * t168 * t174 -
            l_dot * t159 * t160 * t172 * t174 -
            pitch_u_dot * t133 * t134 * t151 * t168 +
            pitch_u_dot * t133 * t134 * t160 * t172 -
            roll_u_dot * t133 * t151 * t172 * t174 -
            roll_u_dot * t133 * t160 * t168 * t174;
  J(1, 1) = t133 * t134 * t319 - l_dot * t134 * t159 * t303 -
            pitch_u_dot * t133 * t174 * t303 + t133 * t151 * t174 * t309 -
            t133 * t160 * t174 * t306 +
            l_dot * t159 * t160 * t174 * (t269 + t272 - t310) +
            pitch_u_dot * t133 * t134 * t151 * (t273 + t274 - t311) +
            roll_u_dot * t133 * t151 * t174 * (t269 + t272 - t310) +
            roll_u_dot * t133 * t160 * t174 * (t273 + t274 - t311) -
            l_dot * t151 * t159 * t174 * t275 -
            pitch_u_dot * t133 * t134 * t160 * t276;
  J(2, 0) = t174 * t314 - pitch_u_dot * t134 * t292 + t134 * t160 * t286 -
            t134 * t151 * t316 - pitch_u_dot * t151 * t168 * t174 +
            pitch_u_dot * t160 * t172 * t174 + roll_u_dot * t134 * t151 * t172 +
            roll_u_dot * t134 * t160 * t168;
  J(2, 1) = t174 * t319 + pitch_u_dot * t134 * (t301 + t302 - t320) -
            t134 * t151 * t309 + t134 * t160 * t306 -
            pitch_u_dot * t160 * t174 * t276 - roll_u_dot * t134 * t151 * t276 -
            roll_u_dot * t134 * t160 * t275 +
            pitch_u_dot * t151 * t174 * (t273 + t274 - t311);

  return J;
}

} // namespace SPRR2U1
