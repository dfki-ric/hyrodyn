#include "RRPR.hpp"

namespace RRPR {

void rrPr::calc_permutationmatrix(std::vector<string> jointnames_spanningtree,
                                  std::vector<string> jointnames_active) {
  // MatrixXd Q;
  // Q.setZero(jointnames_active.size(), jointnames_spanningtree.size());
  permutation_matrix.setZero(jointnames_active.size(), jointnames_spanningtree.size());

  for (unsigned int i = 0; i < jointnames_active.size(); i++) {
    for (unsigned int j = 0; j < jointnames_spanningtree.size(); j++) {
      if (jointnames_spanningtree[j] == jointnames_active[i])
        // Q(i,j) = 1;
        permutation_matrix(i, j) = 1;
    }
  }
  //	cout<<"Permutation matrix, Q = "<<endl<<Q<<endl;

  // set_permutation_matrix(Q);
}

// Constructor
rrPr::rrPr(string file_path, std::vector<string> jointnames_spanningtree,
           std::vector<string> jointnames_active) {
  Model m;

  const char* ext;
  ext = strrchr(file_path.c_str(), '.');
  if (!strcmp(ext, ".urdf") || !strcmp(ext, ".URDF") || !strcmp(ext, ".robot")) {
    //	cout<<"Input file is URDF"<<endl;
    //		if (!Addons::URDFReadFromFile (file_path.c_str(), &m, false)) {
    if (!Addons::URDFReadFromFileWithModularity(file_path.c_str(), &m, jointnames_spanningtree,
                                                false)) {  // done to ensure that urdf is parsed
                                                           // according to correct numbering scheme
      std::cerr << "Error loading urdf model" << std::endl;
      abort();
    }
  } else {
    std::cerr << "Unknown file type: Accepted file types are .urdf or .lua" << endl;
    abort();
  }

  VectorNd Q(VectorNd::Zero(m.dof_count));
  UpdateKinematicsCustom(m, &Q, NULL, NULL);

  Vector3d root_position = CalcBodyToBaseCoordinates(m, Q, 0., Vector3d(0., 0., 0.), false);
  //	cout<<"root pos: "<<root_position.transpose()<<endl;
  Vector3d ee_position =
      CalcBodyToBaseCoordinates(m, Q, 1., Vector3d(0., 0., 0.), false);  // B1 frame
  //	cout<<"ee pos: "<<ee_position.transpose()<<endl;
  Vector3d b2_position =
      CalcBodyToBaseCoordinates(m, Q, 2., Vector3d(0., 0., 0.),
                                false);  // B2 frame (In case of RH5, Stator-knee or Actor-hip2)
                                         //	cout<<"b2 pos: "<<b2_position.transpose()<<endl;
  Vector3d b3_position =
      CalcBodyToBaseCoordinates(m, Q, 3., Vector3d(0., 0., 0.),
                                false);  // B3 frame (In case of RH5, Stator-hip2 or Actor-knee)
                                         //	cout<<"b3 pos: "<<b3_position.transpose()<<endl;

  /* // Old code
  l1 = (b2_position - root_position).norm();
  l2 = (b3_position - root_position).norm();
  d = (b3_position - b2_position).norm();
  */

  // Its better to compute l1 and l2 wrt to ee_position as sometimes root frame
  // might not be coincident with the ee frame.
  l1 = (b2_position - ee_position).norm();
  l2 = (b3_position - ee_position).norm();
  d = (b3_position - b2_position).norm();

  // cout<<"Lambda mechanism: Design parameters: l1 = "<<l1<<" l2 = "<<l2<<" d =
  // "<<d<<endl;

  // cout<<"Model DoF overview:"<<Utils::GetModelDOFOverview(m)<<endl;
  // cout<<"Model Hierarchy overview:"<<Utils::GetModelHierarchy(m)<<endl;
  // cout<<"Named Body Origins
  // overview:"<<Utils::GetNamedBodyOriginsOverview(m)<<endl;

  /*
      // hip dimensions in RH5 Humanoid (please update these variables in lua
  file accordingly) l1 = 114.02/1000; l2 = 364.97/1000; d = 297.40/1000;
      */

  /*
  // knee dimensions in RH5 Humanoid (please update these variables in lua file
accordingly) l1 = 340.82/1000; l2 = 85.0/1000; d = 273.41/1000;
  */

  dof_active = 1;
  dof_spanningtree = m.dof_count;

  //	set_dofs(m.dof_count, 1);

  Q_zero = VectorNd::Zero(m.dof_count);

  double cos_theta1 = (l1 * l1 + l2 * l2 - d * d) / (2 * l1 * l2);
  double sin_theta1 = sqrt(1 - cos_theta1 * cos_theta1);
  double theta1 = atan2(sin_theta1, cos_theta1);

  double cos_theta2 = (l1 * l1 + d * d - l2 * l2) / (2 * l1 * d);
  double sin_theta2 = sqrt(1 - cos_theta2 * cos_theta2);
  double theta2 = M_PI - atan2(sin_theta2, cos_theta2);

  double d = sqrt(l1 * l1 + l2 * l2 - 2 * l1 * l2 * cos(theta1));

  // (Hack starts) NOTE: In future, find a way to detect the signed angle and
  // plane of observation automatically.
  if (abs(d - 0.2974) <= 0.00001) {
    cout << "Hip detected" << endl;
    Q_zero(0) = -theta1;
    Q_zero(1) = -theta2;
    Q_zero(2) = d;
  } else {
    cout << "Hip not detected" << endl;
    Q_zero(0) = theta1;
    Q_zero(1) = theta2;
    Q_zero(2) = d;
  }
  // Hack ends!

  /*
  Q_zero(0) = theta1;
  Q_zero(1) = theta2;
  Q_zero(2) = d;
  */
  cout << "Q_zero: " << Q_zero.transpose() << endl;

  // Calculate the permutation matrix (m x n) between active dof and spanning
  // tree dof
  calc_permutationmatrix(jointnames_spanningtree, jointnames_active);
}

VectorXd rrPr::calc_loopclosure_function(const Math::VectorNd& y) {
  // This function calculates the loop closure functions gamma
  // for the mechanism
  //
  //       /   theta1      \
	//  Q =  |   q2(theta1)  |
  //       \    d(theta1)  /
  //

  VectorXd Q(3);

  double theta1 = y(0) + Q_zero(0);

  double cos_theta2 = (l2 * cos(theta1) - l1) / d;
  double sin_theta2 = (l2 * sin(theta1)) / d;
  double theta2 = atan2(sin_theta2, cos_theta2);

  double d = sqrt(l1 * l1 + l2 * l2 - 2 * l1 * l2 * cos(theta1));

  Q(0) = theta1;
  Q(1) = theta2;
  Q(2) = d;

  //	cout<<"Q absolute: "<<Q.transpose()<<endl;
  Q = Q - Q_zero;

  return Q;
}

MatrixXd rrPr::calc_loopclosure_Jacobian(const Math::VectorNd& y) {
  // This function calculates the loop closure Jacobian G
  // for the mechanism
  //
  //       /   theta1      \
	//  Q =  |   q2(theta1)  |
  //       \    d(theta1)  /
  //

  double theta1 = y(0) + Q_zero(0);

  MatrixXd G(3, 1);

  double d = sqrt(l1 * l1 + l2 * l2 - 2 * l1 * l2 * cos(theta1));

  G(0, 0) = 1.0;
  G(1, 0) = (l2 * l2 - l1 * l2 * cos(theta1)) / (d * d);
  G(2, 0) = (l1 * l2 * sin(theta1)) / d;

  return G;
}

MatrixXd rrPr::calc_loopclosure_Jacobiand(const Math::VectorNd& y, const Math::VectorNd& ydot) {
  // This function calculates the loop closure Jacobiand Gdot
  // for the mechanism
  //
  //       /   theta1      \
	//  Q =  |   q2(theta1)  |
  //       \    d(theta1)  /
  //

  double theta1 = y(0) + Q_zero(0);
  double theta1_dot = ydot(0);
  double d = sqrt(l1 * l1 + l2 * l2 - 2 * l1 * l2 * cos(theta1));

  MatrixXd Gdot(3, 1);
  double u2 = (l1 * l2 * sin(theta1)) / d;

  Gdot(0, 0) = 0.0;
  Gdot(1, 0) = (((l1 * l1 - l2 * l2) * l1 * l2 * sin(theta1)) * theta1_dot) / pow(d, 4);
  Gdot(2, 0) = ((l1 * l2 * cos(theta1) - u2 * u2) * theta1_dot) / d;

  return Gdot;
}

VectorXd rrPr::calc_loopclosure_g(const Math::VectorNd& y, const Math::VectorNd& ydot) {
  // This function calculates the loop closure bias acceleration g= Gdot*ydot
  // for the mechanism
  //
  //       /   theta1      \
	//  Q =  |   q2(theta1)  |
  //       \    d(theta1)  /
  //
  return calc_loopclosure_Jacobiand(y, ydot) * ydot;
}

}  // namespace RRPR
