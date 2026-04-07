#ifndef HYRODYN_H
#define HYRODYN_H

#include <math.h>
#include <rbdl/rbdl.h>
#include <stdio.h>

#include <Eigen/Dense>
#include <iostream>

#include "ExplicitLoopConstraints.hpp"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics::Utils;
using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace ExplicitLoopConstraints;

namespace HyRoDyn {

/** \brief Calculate system state at position (q) level from independent joint
 * position (y)
 *
 * The following equation is used: \n
 * \f$ \mathbf{q} = \gamma(\mathbf{y}) \f$
 *
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param y vector of independent joint positions
 * \param q vector of spanning tree positions (output)
 */
void calc_sysstate_q(Model& model, ExplicitLoopConstraintSet& elcs, const Math::VectorNd& y,
                     Math::VectorNd& q);

/** \brief Calculate system state at velocity (qdot) level from independent
 * joint position (y) and velocity (yd)
 *
 * The following equation is used: \n
 * \f$ \mathbf{\dot{q}} = \mathbf{G}\mathbf{\dot{y}} \f$
 *
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param y vector of independent joint positions
 * \param yd vector of independent joint velocities
 * \param qd vector of spanning tree velocities (output)
 */
void calc_sysstate_qdot(Model& model, ExplicitLoopConstraintSet& elcs, const Math::VectorNd& y,
                        const Math::VectorNd& yd, Math::VectorNd& qd);

/** \brief Calculate system state at acceleration (qddot) level from independent
 * joint position (y), velocity (yd) and acceleration (ydd)
 *
 * The following equation is used: \n
 * \f$ \mathbf{\ddot{q}} = \mathbf{G}\mathbf{\ddot{y}} +
 * \mathbf{\dot{G}}\dot{\mathbf{y}}\f$
 *
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param y vector of independent joint positions
 * \param yd vector of independent joint velocities
 * \param ydd vector of independent joint accelerations
 * \param qdd vector of spanning tree accelerations (output)
 */
void calc_sysstate_qddot(Model& model, ExplicitLoopConstraintSet& elcs, const Math::VectorNd& y,
                         const Math::VectorNd& yd, const Math::VectorNd& ydd, Math::VectorNd& qdd);

/** \brief Calculate actuator state at position (u) level from independent joint
 * position (y)
 *
 * The following equation is used: \n
 * \f$ \mathbf{u} = \mathbf{Q} \mathbf{q} \f$
 *
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param q vector of spanning tree positions
 * \param u vector of actuator positions (output)
 */
void calc_actuatorstate_u(Model& model, ExplicitLoopConstraintSet& elcs, const Math::VectorNd& y,
                          Math::VectorNd& u);

/** \brief Calculate actuator state at velocity (udot) level from independent
 * joint position (y) and velocity (yd)
 *
 * The following equation is used: \n
 * \f$ \mathbf{\dot{u}} = \mathbf{Q} \mathbf{\dot{q}} \f$
 *
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param q vector of spanning tree positions
 * \param qd vector of spanning tree velocities
 * \param ud vector of actuator velocities (output)
 */
void calc_actuatorstate_udot(Model& model, ExplicitLoopConstraintSet& elcs, const Math::VectorNd& y,
                             const Math::VectorNd& yd, Math::VectorNd& ud);

/** \brief Calculate actuator state at acceleration (uddot) level from
 * independent joint position (y), velocity (yd) and acceleration (ydd)
 *
 * The following equation is used: \n
 * \f$ \mathbf{\ddot{u}} = \mathbf{Q} \mathbf{\ddot{q}} \f$
 *
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param q vector of spannning tree positions
 * \param qd vector of spanning tree velocities
 * \param qdd vector of spanning tree accelerations
 * \param udd vector of actuator accelerations (output)
 */
void calc_actuatorstate_uddot(Model& model, ExplicitLoopConstraintSet& elcs,
                              const Math::VectorNd& y, const Math::VectorNd& yd,
                              const Math::VectorNd& ydd, Math::VectorNd& udd);

/** \brief Calculate independent joint state at position (y) level from actuator
 * position (u)
 *
 * The following equation is used: \n
 * \f$ \mathbf{u} = \mathbf{Q} \gamma(\mathbf{y}) \f$
 *
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param u vector of actuator positions
 * \param y vector of independent joint positions (output)
 */
void calc_independentjointstate_y(Model& model, ExplicitLoopConstraintSet& elcs,
                                  const Math::VectorNd& u, Math::VectorNd& y);

/** \brief Calculate independent joint state at velocity (ydot) level from
 * actuator position (u) and velocity (ud)
 *
 * The following equation is used: \n
 * \f$ \mathbf{\dot{u}} = \mathbf{Q} \mathbf{G} \mathbf{\dot{y}} \f$
 *
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param u vector of actuator positions
 * \param ud vector of actuator velocities
 * \param y vector of independent joint position (additional output, used for
 * initial guess) \param yd vector of independent joint velocities (output)
 */
void calc_independentjointstate_ydot(Model& model, ExplicitLoopConstraintSet& elcs,
                                     const Math::VectorNd& u, const Math::VectorNd& ud,
                                     Math::VectorNd& y, Math::VectorNd& yd);

/** \brief Calculate independent joint state at acceleration (yddot) level from
 * actuator position (u), velocity (ud) and acceleration (udd)
 *
 * The following equation is used: \n
 * \f$ \mathbf{\ddot{u}} = \mathbf{Q} (\mathbf{G} \mathbf{\dot{y}} +
 * \dot{\mathbf{G}} \dot{\mathbf{y}})\f$
 *
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param u vector of actuator positions
 * \param ud vector of actuator velocities
 * \param udd vector of actuator accelerations
 *  \param y vector of independent joint position (additional output, also used
 * for warm start) \param yd vector of independent joint velocity (additional
 * output) \param ydd vector of independent joint accelerations (main output)
 */
void calc_independentjointstate_yddot(Model& model, ExplicitLoopConstraintSet& elcs,
                                      const Math::VectorNd& u, const Math::VectorNd& ud,
                                      const Math::VectorNd& udd, Math::VectorNd& y,
                                      Math::VectorNd& yd, Math::VectorNd& ydd);

/** \brief Returns the poses of n bodies (n being the size of body_names vector)
 * from input indepedent joint position (y) by solving the Forward Geometric
 * Model
 *
 *  The following equation is used: \n
 * \f$ \mathbf{X}_i = f_i(\mathbf{y}) \quad \forall \quad i \in [1,\ldots, n]\f$
 *
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param y vector of independent joint positions
 * \param body_names vector of Body frames (Link names) for which Forward
 * Geometric Model should be calculated
 */
std::vector<Math::VectorNd> calc_geometricmodel_forward(Model& model,
                                                        ExplicitLoopConstraintSet& elcs,
                                                        const Math::VectorNd& y,
                                                        const std::vector<string> body_names);

/** \brief Returns the pose of a body from input indepedent joint position (y)
 * by solving the Forward Geometric Model
 *
 *  The following equation is used: \n
 * \f$ \mathbf{X} = f(\mathbf{y})\f$
 *
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param y vector of independent joint positions
 * \param body_name Body frame (Link name) for which Forward Geometric Model
 * should be calculated \param X pose of body_name
 */
Math::VectorNd calc_geometricmodel_forward(Model& model, ExplicitLoopConstraintSet& elcs,
                                           const Math::VectorNd& y, const char* body_name);

/** \brief Calculates the Inverse Geometric Model
 *
 * The following equation is used: \n
 * \f$ \mathbf{y} = f^{-1}(\mathbf{X})\f$
 *
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param pose vector of poses
 * \param body_name Body frame (Link name) for which Inverse Geometric Model
 * should be calculated \param y indepedent joint position (output) \param
 * com_input input position vector of center of mass (defaults to zero vector
 * which is treated as invalid COM constraint and disregarded) \param
 * error_tolerance error tolerance for the numerical IK (defaults to 1e-8) NOTE:
 * This function initializes with the last known valid state of the independent
 * joint state. If the IGM is solved successfully, the joint state is updated
 * otherwise not. Make sure it is set to zero if you call this function for the
 * first time.
 */
void calc_geometricmodel_inverse(Model& model, ExplicitLoopConstraintSet& elcs,
                                 const std::vector<Math::VectorNd>& pose,
                                 const std::vector<string> body_name, Math::VectorNd& y,
                                 Vector3d com_input = Vector3d::Zero(),
                                 double error_tolerance = 1e-8);

/** \brief Returns the twist of a body (6-D vector for which the first three
 * elements are the angular velocity and the last three elements the linear
 * velocity in the global reference system a.k.a hybrid representation of the
 * twist) from input indepedent joint position (y) and velocity (yd) by solving
 * the Forward Kinematic Model
 *
 * The following equation is used: \n
 * \f$ \dot{\mathbf{X}} = \mathbf{J}\mathbf{G}\mathbf{\dot{y}}\f$
 *
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param y vector of independent joint positions
 * \param yd vector of independent joint velocities
 * \param body_name Body frame (Link name) for which Forward Kinematics should
 * be calculated
 */
SpatialVector calc_kinematicmodel_forward(Model& model, ExplicitLoopConstraintSet& elcs,
                                          const Math::VectorNd& y, const Math::VectorNd& yd,
                                          const char* body_name);

/** \brief Returns the vector of twists of multiple bodies (each twist being a
 * 6-D vector for which the first three elements are the angular velocity and
 * the last three elements the linear velocity in the global reference system
 * a.k.a hybrid representation of the twist) from input indepedent joint
 * position (y) and velocity (yd) by solving the Forward Kinematic Model
 *
 * The following equation is used: \n
 * \f$ \dot{\mathbf{X}}_i = \mathbf{J}_i\mathbf{G}\mathbf{\dot{y}} \quad \forall
 * \quad i \in [1,\ldots, n]\f$
 *
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param y vector of independent joint positions
 * \param yd vector of independent joint velocities
 * \param body_names vector of Body frames (Link names) for which Forward
 * Geometric Model should be calculated
 */
std::vector<SpatialVector> calc_kinematicmodel_forward(Model& model,
                                                       ExplicitLoopConstraintSet& elcs,
                                                       const Math::VectorNd& y,
                                                       const Math::VectorNd& yd,
                                                       std::vector<string> body_names);

/** \brief Returns the spatial acceleration (time derivative of twist) of a body
 * (6-D vector for which the first three elements are the angular acceleration
 * and the last three elements the linear acceleration in the global reference
 * system a.k.a hybrid representation) from input indepedent joint position (y),
 * velocity (yd) and acceleration (ydd) by solving the 2nd order Forward
 * Kinematic Model
 *
 * The following equation is used: \n
 * \f$ \ddot{\mathbf{X}} = \mathbf{J}\mathbf{G}\mathbf{\ddot{y}} +
 * (\mathbf{\dot{J}}\mathbf{G} + \mathbf{J}\mathbf{\dot{G}})\mathbf{\ddot{y}}
 * \f$
 *
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param y vector of independent joint positions
 * \param yd vector of independent joint velocities
 * \param ydd vector of independent joint acceleration
 * \param body_name Body frame (Link name) for which Forward Kinematics should
 * be calculated
 */
SpatialVector calc_secondorder_kinematicmodel_forward(Model& model, ExplicitLoopConstraintSet& elcs,
                                                      const Math::VectorNd& y,
                                                      const Math::VectorNd& yd,
                                                      const Math::VectorNd& ydd,
                                                      const char* body_name);

/** \brief Returns the vector of spatial accelerations (time derivative of
 * twist) of multiple bodies (each spatial acceleration being a 6-D vector for
 * which the first three elements are the angular acceleration and the last
 * three elements the linear acceleration in the global reference system a.k.a
 * hybrid representation) from input indepedent joint position (y), velocity
 * (yd) and acceleration (ydd) by solving the 2nd order Forward Kinematic Model
 *
 * The following equation is used: \n
 * \f$ \ddot{\mathbf{X}}_i = \mathbf{J}_i\mathbf{G}\mathbf{\ddot{y}} +
 * (\mathbf{\dot{J}}_i\mathbf{G} +
 * \mathbf{J}_i\mathbf{\dot{G}})\mathbf{\ddot{y}} \quad \forall \quad i \in
 * [1,\ldots, n]\f$
 *
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param y vector of independent joint positions
 * \param yd vector of independent joint velocities
 * \param ydd vector of independent joint acceleration
 * \param body_names vector of Body frames (Link names) for which Forward
 * Kinematics should be calculated
 */
std::vector<SpatialVector> calc_secondorder_kinematicmodel_forward(
    Model& model, ExplicitLoopConstraintSet& elcs, const Math::VectorNd& y,
    const Math::VectorNd& yd, const Math::VectorNd& ydd, std::vector<string> body_names);

/** \brief Calculate full spatial Jacobian of size (6xn)
 *
 *  This Jacobian maps the spanning tree joint velocities (qdot) to twist in
 * spatial representation: \n \f$ \mathbf{V}_s = \mathbf{J}_s(\mathbf{y})
 * \mathbf{\dot{q}}\f$ \n \n NOTE: The spatial representation of twist inside
 * HyRoDyn differs from Modern Robotics Textbook by Lynch and Park in the sense
 * that linear velocity is the actual time derivative of the position of the
 * origin of body frame in base coordinates (See Section 3.3.2 in Chapter 3).
 * This has also been referred to as hybrid representation of the twist in the
 * literature. \param model RBDL model \param elcs Explicit Loop Constraints Set
 * \param y vector of independent joint positions
 * \param J Jacobian matrix (output)
 * \param body_name Body frame (Link name) for which Jacobian should be
 * calculated
 */
void calc_spatial_jacobian(Model& model, ExplicitLoopConstraintSet& elcs, const Math::VectorNd& y,
                           Math::MatrixNd& J, const char* body_name);

/** \brief Calculate full body Jacobian of size (6xn)
 *
 *  This Jacobian maps the spanning tree joint velocities (qdot) to twist in
 * body representation: \n \f$ \mathbf{V}_b = \mathbf{J}_b(\mathbf{y})
 * \mathbf{\dot{q}}\f$
 *
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param y vector of independent joint positions
 * \param J Body Jacobian matrix (output)
 * \param body_name Body frame (Link name) for which Jacobian should be
 * calculated
 */
void calc_body_jacobian(Model& model, ExplicitLoopConstraintSet& elcs, const Math::VectorNd& y,
                        Math::MatrixNd& J, const char* body_name);

/** \brief Calculate spatial Jacobian of size (6xm) projected to independent
 * joint space
 *
 *  This Jacobian maps the independent joint velocities (ydot) to twist in
 * spatial representation: \n \f$ \mathbf{V}_s = \mathbf{J}_s(\mathbf{y})
 * \mathbf{\dot{y}}\f$ \n \n NOTE: The spatial representation of twist inside
 * HyRoDyn differs from Modern Robotics Textbook by Lynch and Park in the sense
 * that linear velocity is the actual time derivative of the position of the
 * origin of body frame in base coordinates (See Section 3.3.2 in Chapter 3).
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param y vector of independent joint positions
 * \param J Jacobian matrix (output)
 * \param body_name Body frame (Link name) for which Jacobian should be
 * calculated
 */
void calc_spatial_jacobian_independent_joint_space(Model& model, ExplicitLoopConstraintSet& elcs,
                                                   const Math::VectorNd& y, Math::MatrixNd& J,
                                                   const char* body_name);

/** \brief Calculate body Jacobian of size (6xn) projected to independent joint
 * space
 *
 *  This Jacobian maps the spanning tree joint velocities (ydot) to twist in
 * spatial representation: \n \f$ \mathbf{V}_b = \mathbf{J}_b(\mathbf{y})
 * \mathbf{\dot{y}}\f$
 *
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param y vector of independent joint positions
 * \param J Body Jacobian matrix (output)
 * \param body_name Body frame (Link name) for which Jacobian should be
 * calculated
 */
void calc_body_jacobian_independent_joint_space(Model& model, ExplicitLoopConstraintSet& elcs,
                                                const Math::VectorNd& y, Math::MatrixNd& J,
                                                const char* body_name);

/** \brief Calculate spatial Jacobian of size (6xp) projected to actuation space
 *
 *  This Jacobian maps the independent joint velocities (ydot) to twist in
 * spatial representation: \n \f$ \mathbf{V}_s = \mathbf{J}_s(\mathbf{u})
 * \mathbf{\dot{u}}\f$ \n \n NOTE: The spatial representation of twist inside
 * HyRoDyn differs from Modern Robotics Textbook by Lynch and Park in the sense
 * that linear velocity is the actual time derivative of the position of the
 * origin of body frame in base coordinates (See Section 3.3.2 in Chapter 3).
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param y vector of independent joint positions
 * \param J Jacobian matrix (output)
 * \param body_name Body frame (Link name) for which Jacobian should be
 * calculated
 */
void calc_spatial_jacobian_actuation_space(Model& model, ExplicitLoopConstraintSet& elcs,
                                           const Math::VectorNd& y, Math::MatrixNd& J,
                                           const char* body_name);

/** \brief Calculate body Jacobian of size (6xp) projected to actuation space
 *
 *  This Jacobian maps the spanning tree joint velocities (ydot) to twist in
 * spatial representation: \n \f$ \mathbf{V}_b = \mathbf{J}_b(\mathbf{u})
 * \mathbf{\dot{u}}\f$
 *
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param y vector of independent joint positions
 * \param J Body Jacobian matrix (output)
 * \param body_name Body frame (Link name) for which Jacobian should be
 * calculated
 */
void calc_body_jacobian_actuation_space(Model& model, ExplicitLoopConstraintSet& elcs,
                                        const Math::VectorNd& y, Math::MatrixNd& J,
                                        const char* body_name);

/** \brief Calculate spatial Jacobian of size (6 x (6+p)) projected to actuation
 * space including the 6 DOF floating base joint
 *
 *  This Jacobian maps the independent joint velocities (ydot) to twist in
 * spatial representation: \n \f$ \mathbf{V}_s = \mathbf{J}_s(\mathbf{u})
 * \mathbf{\dot{u}}\f$ \n \n NOTE: The spatial representation of twist inside
 * HyRoDyn differs from Modern Robotics Textbook by Lynch and Park in the sense
 * that linear velocity is the actual time derivative of the position of the
 * origin of body frame in base coordinates (See Section 3.3.2 in Chapter 3).
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param y vector of independent joint positions
 * \param J Jacobian matrix (output)
 * \param body_name Body frame (Link name) for which Jacobian should be
 * calculated NOTE: This Jacobian (output of this function) is kind of
 * juxtaposition because the floating base coordinates are not a part of
 * actuation space. Nevertheless, it might be useful for applications in whole
 * body control or optimal control formulations where you take care of finding
 * consistent contact wrenches that take care of the underactuation.
 */
void calc_spatial_jacobian_actuation_space_including_floating_base(Model& model,
                                                                   ExplicitLoopConstraintSet& elcs,
                                                                   const Math::VectorNd& y,
                                                                   Math::MatrixNd& J,
                                                                   const char* body_name);

/** \brief Calculate body Jacobian of size (6 x (6+p)) projected to actuation
 * space including the 6 DOF floating base joint
 *
 *  This Jacobian maps the spanning tree joint velocities (ydot) to twist in
 * spatial representation: \n \f$ \mathbf{V}_b = \mathbf{J}_b(\mathbf{u})
 * \mathbf{\dot{u}}\f$
 *
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param y vector of independent joint positions
 * \param J Body Jacobian matrix (output)
 * \param body_name Body frame (Link name) for which Jacobian should be
 * calculated NOTE: This Jacobian (output of this function) is kind of
 * juxtaposition because the floating base coordinates are not a part of
 * actuation space. Nevertheless, it might be useful for applications in whole
 * body control or optimal control formulations where you take care of finding
 * consistent contact wrenches that take care of the underactuation.
 */
void calc_body_jacobian_actuation_space_including_floating_base(Model& model,
                                                                ExplicitLoopConstraintSet& elcs,
                                                                const Math::VectorNd& y,
                                                                Math::MatrixNd& J,
                                                                const char* body_name);

/** \brief Calculate point Jacobian of size (3xm)
 *
 *  This Jacobian maps the independent joint velocities (ydot) to linear
 * velocity of the body in base coordinates: \n \f$ \mathbf{v}_s =
 * \mathbf{J}_s(\mathbf{y}) \mathbf{\dot{y}}\f$
 *
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param y vector of independent joint positions
 * \param J Jacobian matrix (output)
 * \param body_name Body frame (Link name) for which Jacobian should be
 * calculated
 */
void calc_point_jacobian(Model& model, ExplicitLoopConstraintSet& elcs, const Math::VectorNd& y,
                         Math::MatrixNd& J, const char* body_name);

/** \brief Calculate center of mass Jacobian
 *
 *  This Jacobian maps the independent joint velocities (ydot) to linear
 * velocity of com in base coordinates: \n \f$ \mathbf{v}_c =
 * \mathbf{J}_c(\mathbf{y}) \mathbf{\dot{y}}\f$
 *
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param y vector of independent joint positions
 * \param Jcom Center of Mass Jacobian matrix (output)
 */
void calc_com_jacobian(Model& model, ExplicitLoopConstraintSet& elcs, const Math::VectorNd& y,
                       Math::MatrixNd& Jcom);

/** \brief Returns the (special) adjoint transformation of a body from input
 * indepedent joint position (y) in base coordinates
 *
 *  The following equation is used: \n
 * \f$ ^s\mathbf{X}_b = \left( \begin{array}{cc} ^s\mathbf{R}_b & \mathbf{0}
 * \\ \mathbf{0} & ^s\mathbf{R}_b \end{array} \right) \f$ \n where \f$
 * ^s\mathbf{X}_b \f$ is the adjoint transformation of a body in space fixed
 * frame or base coordinates \n \f$ ^s\mathbf{R}_b \f$ is the rotation matrix of
 * a body in space fixed frame or base coordinates \n NOTE: The adjoint matrix
 * expression inside HyRoDyn is different from the usual textbook expressions.
 * This is because of the special treatment of spatial Jacobian inside RBDL
 * which is more practical i.e. when it is multiplied by a body twist, it
 * provides a spatial twist such that linear velocity is the time derivative of
 * the origin of body frame in base coordinates. For the usual adjoint
 * transformation, please use RBDL's built-in type SpatialTransform (const
 * Matrix3d &rotation, const Vector3d &translation). \n Here are some
 * applications on how you can use this adjoint transformation: \n
 * 1. Transform body twist into space twist using \f$ \mathbf{V}_s =
 * ^s\mathbf{X}_b \mathbf{V}_b \f$ \n
 * 2. Transform body jacobian into space jacobian using \f$ \mathbf{J}_s =
 * ^s\mathbf{X}_b \mathbf{J}_b \f$ \n
 * 3. Transform body wrench into space wrench using \f$ \mathbf{W}_s =
 * ^s\mathbf{X}_b^* \mathbf{W}_b \f$ where \f$ ^s\mathbf{X}_b^* \f$ is the
 * adjoint representation is dual space given by \f$ ^s\mathbf{X}_b^* =
 * ^s\mathbf{X}_b^{-T} \f$ \n To compute \f$ ^s\mathbf{X}_b^* \f$, one can use
 * adjoint_transformation.toMatrixAdjoint() which is a built-in feature for
 * computing adjoint transformation in its dual space in RBDL for the type
 * SpatialTransformation. \param model RBDL model \param elcs Explicit Loop
 * Constraints Set \param y vector of independent joint positions \param
 * body_name Body frame (Link name) for which Forward Geometric Model should be
 * calculated
 */
Math::SpatialTransform calc_adjoint_transformation(Model& model, ExplicitLoopConstraintSet& elcs,
                                                   const Math::VectorNd& y, const char* body_name);

/** \brief Calculate actuator Jacobian of size (pxm) which maps robot's
 * independent joint velocities to actuator velocities
 *
 * The following equations are used: \n
 * \f$ \mathbf{G} = \frac{\partial \gamma}{\partial \mathbf{y}} \f$ \n
 * \f$ \mathbf{G_u} = \mathbf{Q}\mathbf{G}  \f$
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param y vector of independent joint positions
 * \param Gu actuator Jacobian matrix (output)
 */
void calc_actuator_jacobian(Model& model, ExplicitLoopConstraintSet& elcs, const Math::VectorNd& y,
                            Math::MatrixNd& Gu);

/** \brief Returns the independent joint accelerations (ydd) from the input
 * actuator force (Tau_actuated) to a robot and its state (y, yd) by solving the
 * forward dynamic model
 *
 * The following equations are used:
 * \f$ \mathbf{q} = \gamma(\mathbf{y}) \f$ \n
 * \f$ \mathbf{\dot{q}} = \mathbf{G}\mathbf{\dot{y}} \f$ \n
 * \f$ \mathbf{g} = \dot{\mathbf{G}}\mathbf{\dot{y}} \f$ \n
 * \f$ \mathbf{G_u} = \mathbf{Q}\mathbf{G}  \f$ \n
 * \f$ \mathbf{\tau} = \mathbf{C}(\mathbf{q},\mathbf{\dot{q}}) \f$ \n
 * \f$ \mathbf{H} = CRBA(\mathbf{q}) \f$ \n
 * \f$ \mathbf{\ddot{y}} = (\mathbf{G}^T
 * \mathbf{H}\mathbf{G})^{-1}\mathbf{G}^T_u \mathbf{\tau}_u -
 * \mathbf{G}^T\mathbf{C} - \mathbf{G}^T \mathbf{H}\mathbf{g} \f$ \param model
 * RBDL model \param elcs Explicit Loop Constraints Set \param y vector of
 * independent joint positions \param yd vector of independent joint velocities
 * \param Tau_actuated the actuated torque
 */
VectorNd calc_dynamicmodel_forward(Model& model, ExplicitLoopConstraintSet& elcs,
                                   const Math::VectorNd& y, const Math::VectorNd& yd,
                                   const VectorXd& Tau_actuated);

/** \brief Compute the mass-interia matrix (m x m) of the robot projected in
 * independent joint space
 *
 * The following equations are used: \n
 * \f$ \mathbf{q} = \gamma(\mathbf{y}) \f$ \n
 * \f$ \mathbf{H} = CRBA(\mathbf{q}) \f$ \n
 * \f$ \mathbf{H}_y = \mathbf{G}^T \mathbf{H}\mathbf{G} \f$
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param y vector of independent joint positions
 * \param H mass-interia matrix of the robot projected in independent joint
 * space (output)
 */
void calc_mass_interia_matrix(Model& model, ExplicitLoopConstraintSet& elcs,
                              const Math::VectorNd& y, Math::MatrixNd& H);

/** \brief Compute the mass-interia matrix (p x p) of the robot projected in
 * actuation space
 *
 * The following equations are used: \n
 * \f$ \mathbf{y} = NewtonRaphson(\mathbf{u}) \f$ \n
 * \f$ \mathbf{q} = \gamma(\mathbf{y}) \f$ \n
 * \f$ \mathbf{H} = CRBA(\mathbf{q}) \f$ \n
 * \f$ \mathbf{H}_u = \mathbf{G}_u^{-T} \mathbf{G}^T \mathbf{H}\mathbf{G}
 * \mathbf{G}_u^{-1}\f$ \param model RBDL model \param elcs Explicit Loop
 * Constraints Set \param y vector of independent joint positions \param Hu
 * mass-interia matrix of the robot projected in actuation space (output)
 */
void calc_mass_interia_matrix_actuation_space(Model& model, ExplicitLoopConstraintSet& elcs,
                                              const Math::VectorNd& y, Math::MatrixNd& Hu);

/** \brief Compute the mass-interia matrix (p+floating_dof x p+floating_dof) of
 * the robot projected in actuation space including the floating base
 *
 * The following equations are used: \n
 * \f$ \mathbf{y} = NewtonRaphson(\mathbf{u}) \f$ \n
 * \f$ \mathbf{q} = \gamma(\mathbf{y}) \f$ \n
 * \f$ \mathbf{H} = CRBA(\mathbf{q}) \f$ \n
 * \f$ \mathbf{H}_u = \mathbf{G}_u^{-T} \mathbf{G}^T \mathbf{H}\mathbf{G}
 * \mathbf{G}_u^{-1}\f$ \param model RBDL model \param elcs Explicit Loop
 * Constraints Set \param y vector of independent joint positions \param Hufb
 * mass-interia matrix of the robot projected in actuation space including the
 * floating base (output) NOTE: This mass-inertia matrix (output of this
 * function) is kind of juxtaposition because the floating base coordinates are
 * not a part of actuation space. Nevertheless, it might be useful for
 * applications in whole body control or optimal control formulations where you
 * take care of finding consistent contact wrenches that take care of the
 * underactuation.
 */
void calc_mass_interia_matrix_actuation_space_including_floating_base(
    Model& model, ExplicitLoopConstraintSet& elcs, const Math::VectorNd& y, Math::MatrixNd& Hufb);

/** \brief Compute the nonlinear effects vector (p x 1) of the robot projected
 * in actuation space
 *
 * The following equations are used: \n
 * \f$ \mathbf{q} = \gamma(\mathbf{y}) \f$ \n
 * \f$ \mathbf{C} = NonlinearEffects(\mathbf{q}, \mathbf{\dot{q}}) \f$ \n
 * \f$ \mathbf{H} = CRBA(\mathbf{q}) \f$ \n
 * \f$ \mathbf{g} = \dot{\mathbf{G}}\mathbf{\dot{y}} \f$ \n
 * \f$ \mathbf{C}_u = \mathbf{G}_u^{-T} \mathbf{G}^T (\mathbf{C} +
 * \mathbf{H}\mathbf{g}) \f$ \param model RBDL model \param elcs Explicit Loop
 * Constraints Set \param y vector of independent joint positions \param yd
 * vector of independent joint velocities \param Cu nonlinear effects vector of
 * the robot projected in actuation space (output)
 */
void calc_nonlinear_effects_actuation_space(Model& model, ExplicitLoopConstraintSet& elcs,
                                            const Math::VectorNd& y, const Math::VectorNd& yd,
                                            Math::VectorNd& Cu);

/** \brief Returns the actuator forces from the input motion (y, yd, ydd) to a
 * robot by solving the inverse dynamic model
 *
 * The following equations are used: \n
 * \f$ \mathbf{q} = \gamma(\mathbf{y}) \f$  \n
 * \f$ \mathbf{\dot{q}} = \mathbf{G}\mathbf{\dot{y}} \f$ \n
 * \f$ \mathbf{G_u} = \mathbf{Q}\mathbf{G}  \f$ \n
 * \f$ \mathbf{\ddot{q}} = \mathbf{G}\mathbf{\ddot{y}} + \mathbf{g} \f$  \n
 * where \f$ \mathbf{g} = \dot{\mathbf{G}}\mathbf{\dot{y}} \f$ \n
 * \f$ \mathbf{\tau} = \mathbf{M}(\mathbf{q})\mathbf{\ddot{q}} +
 * \mathbf{C}(\mathbf{q},\mathbf{\dot{q}}) \f$ \n \f$ \mathbf{\tau_y} =
 * \mathbf{G}^T \mathbf{\tau} \f$ \n \f$ \mathbf{\tau}_u =
 * \mathbf{G}^{-1}_u\mathbf{G}^T\mathbf{\tau} \f$ \param model RBDL model \param
 * elcs Explicit Loop Constraints Set \param y vector of independent joint
 * positions \param yd vector of independent joint velocities \param ydd vector
 * of independent joint accelerations
 */
VectorNd calc_dynamicmodel_inverse(Model& model, ExplicitLoopConstraintSet& elcs,
                                   const Math::VectorNd& y, const Math::VectorNd& yd,
                                   const Math::VectorNd& ydd);

/** \brief Returns the actuator forces from the input motion (y, yd, ydd) to a
 * robot by solving the inverse dynamic model including the floating base
 *
 * The following equations are used: \n
 * \f$ \mathbf{q} = \gamma(\mathbf{y}) \f$  \n
 * \f$ \mathbf{\dot{q}} = \mathbf{G}\mathbf{\dot{y}} \f$ \n
 * \f$ \mathbf{G_u} = \mathbf{Q}\mathbf{G}  \f$ \n
 * \f$ \mathbf{\ddot{q}} = \mathbf{G}\mathbf{\ddot{y}} + \mathbf{g} \f$  \n
 * where \f$ \mathbf{g} = \dot{\mathbf{G}}\mathbf{\dot{y}} \f$ \n
 * \f$ \mathbf{\tau} = \mathbf{M}(\mathbf{q})\mathbf{\ddot{q}} +
 * \mathbf{C}(\mathbf{q},\mathbf{\dot{q}}) \f$ \n \f$ \mathbf{\tau_y} =
 * \mathbf{G}^T \mathbf{\tau} \f$ \n \f$ \mathbf{\tau}_u =
 * \mathbf{G}^{-1}_u\mathbf{G}^T\mathbf{\tau} \f$ \param model RBDL model \param
 * elcs Explicit Loop Constraints Set \param y vector of independent joint
 * positions \param yd vector of independent joint velocities \param ydd vector
 * of independent joint accelerations NOTE: The output of this function is kind
 * of juxtaposition because the floating base coordinates are not a part of
 * actuation space. Nevertheless, it might be useful for applications in whole
 * body control or optimal control formulations where you take care of finding
 * consistent contact wrenches that take care of the underactuation.
 */
VectorNd calc_dynamicmodel_inverse_including_floatingbase(Model& model,
                                                          ExplicitLoopConstraintSet& elcs,
                                                          const Math::VectorNd& y,
                                                          const Math::VectorNd& yd,
                                                          const Math::VectorNd& ydd);

/** \brief Returns the generalized forces(indepedent joint space forces/torques)
 * from the input motion (y, yd, ydd) to a robot by solving the inverse dynamic
 * model
 *
 * The following equations are used: \n
 * \f$ \mathbf{q} = \gamma(\mathbf{y}) \f$  \n
 * \f$ \mathbf{\dot{q}} = \mathbf{G}\mathbf{\dot{y}} \f$ \n
 * \f$ \mathbf{\ddot{q}} = \mathbf{G}\mathbf{\ddot{y}} + \mathbf{g} \f$  \n
 * where \f$ \mathbf{g} = \dot{\mathbf{G}}\mathbf{\dot{y}} \f$ \n
 * \f$ \mathbf{\tau} = \mathbf{M}(\mathbf{q})\mathbf{\ddot{q}} +
 * \mathbf{C}(\mathbf{q},\mathbf{\dot{q}}) \f$ \n \f$ \mathbf{\tau_y} =
 * \mathbf{G}^T \mathbf{\tau} \f$ \n \param model RBDL model \param elcs
 * Explicit Loop Constraints Set \param y vector of independent joint positions
 * \param yd vector of independent joint velocities
 * \param ydd vector of independent joint accelerations
 */
VectorNd calc_dynamicmodel_inverse_independentjointspace(Model& model,
                                                         ExplicitLoopConstraintSet& elcs,
                                                         const Math::VectorNd& y,
                                                         const Math::VectorNd& yd,
                                                         const Math::VectorNd& ydd);

/** \brief Returns the actuator forces from the input motion (y, yd) in
 * independent joint space and operational space (xdd) to a robot by solving the
 * inverse dynamic model
 *
 * The following equations are used: \n
 * \f$ \mathbf{q} = \gamma(\mathbf{y}) \f$  \n
 * \f$ \mathbf{\dot{q}} = \mathbf{G}\mathbf{\dot{y}} \f$ \n
 * \f$ \mathbf{G_u} = \mathbf{Q}\mathbf{G}  \f$ \n
 * \f$ \mathbf{\ddot{q}} = \mathbf{G}\mathbf{\ddot{y}} + \mathbf{g} \f$  \n
 * where \f$ \mathbf{g} = \dot{\mathbf{G}}\mathbf{\dot{y}} \f$ \n
 * \f$ \mathbf{\tau} = \mathbf{M}(\mathbf{q})\mathbf{\ddot{q}} +
 * \mathbf{C}(\mathbf{q},\mathbf{\dot{q}}) \f$ \n \f$ \mathbf{\tau_y} =
 * \mathbf{G}^T \mathbf{\tau} \f$ \n \f$ \mathbf{\tau}_u =
 * \mathbf{G}^{-1}_u\mathbf{G}^T\mathbf{\tau} \f$ \param model RBDL model \param
 * elcs Explicit Loop Constraints Set \param y vector of independent joint
 * positions \param yd vector of independent joint velocities \param xdd vector
 * of spatial(task space) accelerations
 */
VectorNd calc_constrained_dynamicmodel_inverse(Model& model, ExplicitLoopConstraintSet& elcs,
                                               const Math::VectorNd& y, const Math::VectorNd& yd,
                                               const Math::SpatialVector& xdd,
                                               const Math::SpatialVector& f_ext,
                                               const string body_name);

/** \brief Returns the actuator forces from the wrenches applied on a robot by
 * solving the inverse static model
 *
 * The following equations are used: \n
 * \f$ \mathbf{\tau}_{ext} = (\mathbf{J}\mathbf{G})^T F_{ext} \f$
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param y vector of independent joint positions
 * \param FTsensor_links vector of link names where the external wrench is
 * applied or measured \param f_ext vector of external wrenches (of type
 * SpatialVector) \param wrench_resolution vector of wrench resolution (of type
 * boolean), true = wrench resolved in body frame and false = wrench resolved in
 * base frame \param wrench_interaction vector of wrench interaction (of type
 * boolean), true = resistive and false = assistive
 */
VectorNd calc_staticmodel_inverse(Model& model, ExplicitLoopConstraintSet& elcs,
                                  const Math::VectorNd& y, const std::vector<string> FTsensor_links,
                                  const std::vector<Math::SpatialVector> f_ext,
                                  const std::vector<bool> wrench_resolution,
                                  const std::vector<bool> wrench_interaction);

/** \brief Returns the wrench applied on a point on the robot from the given
 * actuator forces by solving the forward static model
 *
 * The following equations are used: \n
 * \f$ F_{ext} = (\mathbf{J}\mathbf{G})^{-T} \mathbf{\tau}_{ext} \f$
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param y vector of independent joint positions
 * \param Tau_actuated_ext vector of measured actuator forces from which the
 * wrench must be computed \param FTsensor_link link name where the external
 * wrench is applied or measured \param wrench_resolution wrench resolution flag
 * (of type boolean), true = wrench resolved in body frame and false = wrench
 * resolved in base frame \n NOTE: This function works only for 6-dof robots.
 * Otherwise the problem is not well-posed.
 */
SpatialVector calc_staticmodel_forward(Model& model, ExplicitLoopConstraintSet& elcs,
                                       const Math::VectorNd& y,
                                       const Math::VectorNd& Tau_actuated_ext,
                                       const string FTsensor_link, const bool wrench_resolution);

/** \brief Compute mass, center of mass, its linear velocity and acceleration,
 * Angular momentum and its derivative etc. from independent joint state (y, yd,
 * ydd)
 *
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param y vector of independent joint positions
 * \param yd vector of independent joint velocities
 * \param ydd vector of independent joint accelerations
 * \param mass total mass of the system (output)
 * \param com 3d position vector of center of mass (output)
 * \param com_velocity 3d linear velocity vector of center of mass (defaults to
 * NULL) (output) \param com_acceleration 3d linear acceleration vector of
 * center of mass (defaults to NULL) (output) \param angular_momentum 3d angular
 * momentum vector(defaults to NULL) (output) \param change_of_angular_momentum
 * 3d angular momentum derivative vector(defaults to NULL) (output)
 */
void calc_com_properties(Model& model, ExplicitLoopConstraintSet& elcs, const Math::VectorNd& y,
                         const Math::VectorNd& yd, const Math::VectorNd* ydd, double& mass,
                         Math::Vector3d& com, Math::Vector3d* com_velocity = NULL,
                         Math::Vector3d* com_acceleration = NULL,
                         Math::Vector3d* angular_momentum = NULL,
                         Math::Vector3d* change_of_angular_momentum = NULL);

/** \brief Compute the Zero Moment Point (ZMP) from independent joint state (y,
 * yd, ydd) and contact surface plane definition (surface_point, surface_normal)
 *
 * \param model RBDL model
 * \param elcs Explicit Loop Constraints Set
 * \param y vector of independent joint positions
 * \param yd vector of independent joint velocities
 * \param ydd vector of independent joint accelerations
 * \param surface_normal unit normal vector of the contact surface plane
 * \param surface_point point on the contact surface plane
 * \param zmp position vector of the zero moment point projected on the contact
 * surface plane in base coordinates (output)
 */
void calc_zero_moment_point(Model& model, ExplicitLoopConstraintSet& elcs, const Math::VectorNd& y,
                            const Math::VectorNd& yd, const Math::VectorNd& ydd,
                            const Math::Vector3d& surface_normal,
                            const Math::Vector3d& surface_point, Math::Vector3d* zmp);

/**
 * @brief Calculates kinetic energy of the system
 * @param model       RBDL model
 * @param loop_closures Set of explicit loop constraints
 * @param q           Joint positions (tree coords)
 * @param qdot        Joint velocities (tree coords)
 * @return scalar kinetic energy
 */
double calc_kinetic_energy(RigidBodyDynamics::Model& model, Eigen::VectorXd q,
                           Eigen::VectorXd qdot);

/**
 * @brief Calculates potential energy of the system (gravity only)
 * @param model       RBDL model
 * @param q           Joint positions (tree coords)
 * @return scalar potential energy
 */
double calc_potential_energy(RigidBodyDynamics::Model& model, Eigen::VectorXd q);

/**
 * @brief Calculates total system energy (kinetic + potential)
 */
double calc_total_energy(RigidBodyDynamics::Model& model, Eigen::VectorXd q, Eigen::VectorXd qdot);

}  // namespace HyRoDyn

#endif  // HYRODYN
