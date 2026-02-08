#ifndef EXTERIOR_H
#define EXTERIOR_H

#include <math.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <rbdl/rbdl.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>

#include "ExplicitLoopConstraints.hpp"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using Eigen::MatrixXd;
// using Eigen::Vector3d;
using Eigen::VectorXd;

namespace EXTERIOR {
/*! Exoskeletons to series-parallel hybrid mechanisms can be modeled in HyRoDyn
 * using the concept of mimic joints in the URDF file. \image html
 * exo_with_human.png A code snippet of mimic joint tag definition is shown
 * below: \n <joint name=PASSIVE_JNAME type=JTYPE > \n <parent link=PARENT /> \n
 *     <child link=CHILD /> \n
 *     <origin xyz=COORDS rpy=ANGLES /> \n
 *     <axis xyz=DIRECTION /> \n
 *     <mimic joint=DEPENDENT_JNAME multiplier=m offset=b /> \n
 * </joint> \n
 * HyRoDyn will collect the multiplier and offset values from the URDF and
 * transfers them to the loop closure function (LCF) of the exoskeleton. \n \f$
 * \mathbf{q} = \mathbf{M} \mathbf{y} + \mathbf{B} \f$ \n \f$ \dot{\mathbf{q}} =
 * \mathbf{M} \dot{\mathbf{y}} \f$ \n \f$ \ddot{\mathbf{q}} = \mathbf{M}
 * \ddot{\mathbf{y}} \f$ \n where M is the multiplier matrix containing m (+1,
 * -1, 0 as its entries) and B is the offset vector containing b (0s or angular
 * offset, in rad) involved with the active joint frame. \n The LCF of the
 * exoskeletons differ in the sense that they can not be arranged in a block
 * diagonal sense like LCF of the normal submechanism definition in HyRoDyn. \n
 * Assuming you named the 4 human joints as "right_human_shoulder_joint0",
 * "right_human_shoulder_joint1", "right_human_shoulder_joint2",
 * "right_human_elbow_joint" and 4 exoskeleton joints as
 * "right_arm_shoulder_joint0", "right_arm_shoulder_joint1",
 * "right_arm_shoulder_joint2", "right_arm_elbow_joint" here is what you will
 * use in your submechanisms.yml file: exoskeletons: \n
 *
 * - name: "human_rightarm" \n
 *   around: "right_exo_arm" \n
 *   file_path: "RDAWRH.urdf" \n
 *   jointnames_dependent: ["right_arm_shoulder_joint0",
 * "right_arm_shoulder_joint1", "right_arm_shoulder_joint2",
 * "right_arm_elbow_joint"] \n jointnames_spanningtree:
 * ["right_human_shoulder_joint0", "right_human_shoulder_joint1",
 * "right_human_shoulder_joint2", "right_human_elbow_joint"] \n
 *
 *
 * For more examples, see data/hybrid/recupera/dual_arm_exo_with_human.
 */
class exterior : public ExplicitLoopConstraints::ExplicitLoopConstraintSet {
 public:
  /// \brief Vector of offset values
  VectorXd offset;
  /// \brief Loop closure Jacobian matrix
  MatrixXd G;
  /// \brief Actuator Jacobian matrix
  MatrixXd Gu;
  /// \brief Loop closure Jacobian matrix derivative
  MatrixXd Gdot;

  // Constructor
  /** \brief Constructor of the exterior mechanism class
   *
   * \param file_path file path to submechanism urdf
   * \param jointnames_spanningtree vector of spanning tree joint names
   * \param jointnames_active vector of actuator joint names
   */
  exterior(string file_path, std::vector<string> jointnames_spanningtree,
           std::vector<string> jointnames_active);

  // Manually coded
  /** \brief Returns spanning tree state at position (q) level from independent
   * joint position (y) by solving the loop closure function
   *
   * The following equation is used: \n
   * \f$ \mathbf{q} = \mathbf{\gamma}(\mathbf{y}) \f$
   *
   * \param y vector of independent joint positions
   */
  VectorXd calc_loopclosure_function(const Math::VectorNd& y);

  /** \brief Returns the loop closure Jacobian (G) from independent joint
   * position (y)
   *
   * The following equation is used: \n
   * \f$ \mathbf{G} = \frac{\partial \gamma}{\partial \mathbf{y}} \f$
   *
   * \param y vector of independent joint positions
   */
  MatrixXd calc_loopclosure_Jacobian(const Math::VectorNd& y);

  /** \brief Returns the loop closure Jacobian derivative (Gdot) from
   * independent joint position and velocity (y, ydot)
   *
   * The following equation is used: \n
   * \f$ \dot{\mathbf{G}} = \frac{d\mathbf{G}}{dt} \f$
   *
   * \param y vector of independent joint positions
   * \param ydot vector of independent joint velocities
   */
  MatrixXd calc_loopclosure_Jacobiand(const Math::VectorNd& y, const Math::VectorNd& ydot);

  /** \brief Returns the loop closure bias acceleration (g) from independent
   * joint position and velocity (y, yd)
   *
   * The following equation is used: \n
   * \f$ {\mathbf{g}} = \dot{\mathbf{G}}\dot{\mathbf{y}} \f$
   *
   * \param y vector of independent joint positions
   * \param ydot vector of independent joint velocities
   */
  VectorXd calc_loopclosure_g(const Math::VectorNd& y, const Math::VectorNd& ydot);
};

}  // end namespace EXTERIOR

#endif  // EXTERIOR
