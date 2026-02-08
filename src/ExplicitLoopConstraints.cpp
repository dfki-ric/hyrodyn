#include "ExplicitLoopConstraints.hpp"

namespace ExplicitLoopConstraints {

ExplicitLoopConstraintSet ::ExplicitLoopConstraintSet()
    : dof_active(0), dof_spanningtree(0) {}

VectorXd
ExplicitLoopConstraintSet::calc_loopclosure_function(const Math::VectorNd &y) {
  // VectorXd q = y;
  // return q;
  return y;
}

MatrixXd
ExplicitLoopConstraintSet::calc_loopclosure_Jacobian(const Math::VectorNd &y) {
  MatrixXd G;
  G.setIdentity(y.size(), y.size());
  return G;
}

MatrixXd ExplicitLoopConstraintSet::calc_loopclosure_Jacobiand(
    const Math::VectorNd &y, const Math::VectorNd &yd) {
  MatrixXd Gdot;
  Gdot.setZero(y.size(), y.size());
  return Gdot;
}

VectorXd
ExplicitLoopConstraintSet::calc_loopclosure_g(const Math::VectorNd &y,
                                              const Math::VectorNd &yd) {
  VectorXd g = VectorXd::Zero(y.size());
  return g;
}
/*
void ExplicitLoopConstraintSet::set_dofs(unsigned int dof_spanningtree, unsigned
int dof_active){}

void ExplicitLoopConstraintSet::set_permutation_matrix(MatrixXd
permutation_matrix){}

unsigned int ExplicitLoopConstraintSet::get_dof_spanningtree(){
        return dof_spanningtree;
}

unsigned int ExplicitLoopConstraintSet::get_dof_active(){
                return dof_active;
}

MatrixXd ExplicitLoopConstraintSet::get_permutation_matrix(){
                return permutation_matrix;
}
*/
} // namespace ExplicitLoopConstraints
