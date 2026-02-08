/*
 * HyRoDyn - Hybrid Robot Dynamics
 * Copyright (c) 2018-2020 Shivesh Kumar <shivesh.kumar@dfki.de>
 * This submechanism library is contributed by Shivesh Kumar
 * <shivesh.kumar@dfki.de>. Licensed under the zlib license. See LICENSE for
 * more details.
 */

#ifndef TRANSMISSION_H
#define TRANSMISSION_H

#include <Eigen/Dense>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <fstream>

#include <rbdl/rbdl.h>

#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL addon URDFReader not enabled."
#endif
#include <rbdl/addons/urdfreader/urdfreader.h>

#include "ExplicitLoopConstraints.hpp"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using Eigen::MatrixXd;
using Eigen::VectorXd;

namespace TRANSMISSION {
/*! Transmissions can be modeled in HyRoDyn using the concept of custom modified
 * concept of transmission in the URDF file. The idea is to provide a mapping
 * between spanning tree joints(which includes actuators and any other passive
 * joints) and independent joints using depends_on joint tag. \n \image html
 * differential_transmission.png Below example shows how one can model a
 * differential transmission using the MultiJointLinearTransmission interface in
 * HyRoDyn. The equations of differential transmission are given by: \n \f$
 * \theta_1 = R + P \f$ \n \f$ \theta_2 = R - P \f$ \n \f$ \theta_3 = Y \f$ \n
 * where R,P,Y denote the roll, pitch and yaw angles respectively and theta1,
 * theta2, theta3 denote the three motor angles. A code snippet of how one could
 * model a 3 DOF differential transmission is shown below: \n <transmission
 * name="Head"> \n <type>MultiJointLinearTransmission</type> \n <joint
 * name="Head_Act1"> \n <depends_on joint="HeadRoll" multiplier="1.0" offset="0"
 * /> \n <depends_on joint="HeadPitch" multiplier="1.0" offset="0" /> \n
 *   </joint> \n
 *   <joint name="Head_Act2"> \n
 *     <depends_on joint="HeadRoll" multiplier="1.0" offset="0" /> \n
 *     <depends_on joint="HeadPitch" multiplier="-1.0" offset="0" /> \n
 *   </joint> \n
 *   <joint name="Head_Act3"> \n
 *     <depends_on joint="HeadYaw" multiplier="1.0" offset="0" /> \n
 *   </joint> \n
 * </transmission> \n
 * HyRoDyn will collect the multiplier and offset values from the URDF
 * Transmission definition and transfers them to the loop closure function of
 * the mechanism. \n \f$ \mathbf{q} = \mathbf{M} \mathbf{y} + \mathbf{B} \f$ \n
 * \f$ \dot{\mathbf{q}} = \mathbf{M} \dot{\mathbf{y}} \f$ \n
 * \f$ \ddot{\mathbf{q}} = \mathbf{M} \ddot{\mathbf{y}} \f$ \n
 * where M is the multiplier matrix containing m (+1, -1, 0 as its entries) and
 * B is the offset vector containing b (0s or angular offset, in rad) involved
 * with the independent joint frame. \n NOTE: In order to implement this
 * feature, standard transmission parser of the URDF was modified and locally
 * integrated in RBDL addon so that we don't have any dependency to ros_control.
 * The standard ROS transmission examples are not relevant and not supported in
 * HyRoDyn.
 */

class transmission : public ExplicitLoopConstraints::ExplicitLoopConstraintSet {

protected:
  /// \brief Vector of offset values
  VectorXd offset;
  /// \brief Loop closure Jacobian matrix
  MatrixXd G;
  /// \brief Loop closure Jacobian matrix derivative
  MatrixXd Gdot;
  /// \brief Spanning tree joints degrees of freedom of the mechanism
  unsigned int dof_spanningtree;
  /// \brief Independent joints degrees of freedom of the mechanism
  unsigned int dof_independent;

public:
  // Constructor
  /** \brief Constructor of the transmission mechanism class
   *
   * \param file_path file path to submechanism urdf or lua description
   * \param jointnames_spanningtree vector of spanning tree joint names
   * \param jointnames_independent vector of independent joint names
   */
  transmission(string file_path, std::vector<string> jointnames_spanningtree,
               std::vector<string> jointnames_independent);

  // Contains the symbolic code
  /** \brief Returns spanning tree state at position (q) level from independent
   * joint position (y) by solving the loop closure function
   *
   * The following equation is used: \n
   * \f$ \mathbf{q} = \mathbf{\gamma}(\mathbf{y}) \f$
   *
   * \param y vector of independent joint positions
   */
  VectorXd calc_loopclosure_function(const Math::VectorNd &y);

  /** \brief Returns the loop closure Jacobian (G) from independent joint
   * position (y)
   *
   * The following equation is used: \n
   * \f$ \mathbf{G} = \frac{\partial \gamma}{\partial \mathbf{y}} \f$
   *
   * \param y vector of independent joint positions
   */
  MatrixXd calc_loopclosure_Jacobian(const Math::VectorNd &y);

  /** \brief Returns the loop closure Jacobian derivative (Gdot) from
   * independent joint position and velocity (y, yd)
   *
   * The following equation is used: \n
   * \f$ \dot{\mathbf{G}} = \frac{d\mathbf{G}}{dt} \f$
   *
   * \param y vector of independent joint positions
   * \param ydot vector of independent joint velocities
   */
  MatrixXd calc_loopclosure_Jacobiand(const Math::VectorNd &y,
                                      const Math::VectorNd &ydot);

  /** \brief Returns the loop closure bias acceleration (g) from independent
   * joint position and velocity (y, yd)
   *
   * The following equation is used: \n
   * \f$ {\mathbf{g}} = \dot{\mathbf{G}}\dot{\mathbf{y}} \f$
   *
   * \param y vector of independent joint positions
   * \param ydot vector of independent joint velocities
   */
  VectorXd calc_loopclosure_g(const Math::VectorNd &y,
                              const Math::VectorNd &ydot);
};

} // end namespace TRANSMISSION

#endif // TRANSMISSION
