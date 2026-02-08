#ifndef SUBMECHANISMSASSEMBLY_H
#define SUBMECHANISMSASSEMBLY_H

#include <algorithm>
#include <fstream>
#include <iostream>
#include <limits.h>
#include <string.h>
#include <unistd.h>

#include <rbdl/rbdl.h>

#ifndef RBDL_BUILD_ADDON_URDFREADER
#error "Error: RBDL addon URDFReader not enabled."
#endif
#include <rbdl/addons/urdfreader/urdfreader.h>

#include <math.h>

#include "ExplicitLoopConstraints.hpp"
#include "HyRoDyn.hpp"
#include "SubmechanismsAssembly.hpp"

// Parallel submechanism headers
#include "PARALLELOGRAMCHAIN.hpp"
#include "R3US2.hpp"
#include "R3US2_ANKLE.hpp"
#include "RRPR.hpp"
#include "SPRR2U1.hpp"
#include "SPU2U1.hpp"
#include "SURRPR2U1.hpp"
#include "TRANSMISSION.hpp"
#include "UPS6.hpp"
// Exterior exoskeleton header
#include "EXTERIOR.hpp"
// Numerical loop constraint header
#include "NumericalLoopConstraints.hpp"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace std;
using Eigen::MatrixXd;

// Parallel submechanism namespaces
using namespace RRPR;
using namespace SPU2U1;
using namespace R3US2;
using namespace R3US2_ANKLE;
using namespace SPRR2U1;
using namespace UPS6;
using namespace PARALLELOGRAMCHAIN;
using namespace TRANSMISSION;
using namespace EXTERIOR;
// using namespace NUMERICALLOOPCONSTRAINTS;
/*
NOTE: While adding new submechanism libraries, please be sure that
the methods in the class have the exact same function arguments.
E.g. calc_loopclosure_function(const Math::VectorNd &y)
A stupid mistake can be that you are missing & operator before y.
*/

namespace AssembleinHyRoDyn {

// Struct for storing a submechanism
struct submechanism {

  /// \brief Reflects topology of the mechanism
  string type;
  /// \brief Reflects a common name from literature (e.g. stewart platform)
  string name;
  /// \brief Name in an application context (e.g. knee lambda or hip2 lambda)
  string contextual_name;
  /// \brief URDF or lua file path of the submechanism (basically for extraction
  /// of geometric parameters)
  string file_path;

  /// \brief Vector of independent + free-flyer joint names
  std::vector<string> jointnames_independent;
  /// \brief Vector of spanning tree joint names
  std::vector<string> jointnames_spanningtree;
  /// \brief Vector of active joint names
  std::vector<string> jointnames_active;
  /// \brief Vector of independent robot joint names
  std::vector<string> jointnames_independent_robot;
  /// \brief Vector of all joint names including fixed joints
  std::vector<string> jointnames;

  /// \brief Vector of loop constraints
  std::vector<NUMERICALLOOPCONSTRAINTS::Loop_constraints>
      loop_constraints_submech;

  /// \brief Print the submechanism details
  void print_submechanism_details() {
    cout << "=========Submechanism Details=======" << endl;
    cout << "Type: " << type << endl;
    cout << "Popular Name: " << name << endl;
    cout << "Name in the application context: " << contextual_name << endl;
    cout << "Loaded file path: " << file_path << endl;

    cout << "Joint(s) in spanning tree: " << endl;
    for (unsigned int j = 0; j < jointnames_spanningtree.size(); j++)
      cout << jointnames_spanningtree[j] << endl;

    cout << "Spanning Tree DOF: " << jointnames_spanningtree.size() << endl;

    cout << "Independent Joint(s): " << endl;
    for (unsigned int j = 0; j < jointnames_independent.size(); j++)
      cout << jointnames_independent[j] << endl;
    cout << "Independent DOF: " << jointnames_independent.size() << endl;

    cout << "Independent Joint(s) beloging to the robot: " << endl;
    for (unsigned int j = 0; j < jointnames_independent_robot.size(); j++)
      cout << jointnames_independent_robot[j] << endl;

    cout << "Independent DOF belonging to robot (excluding free-flyer joint, "
            "if defined): "
         << jointnames_independent_robot.size() << endl;

    cout << "Active Joint(s): " << endl;
    for (unsigned int j = 0; j < jointnames_active.size(); j++)
      cout << jointnames_active[j] << endl;

    cout << "All the Joint(s) in the submechanism (including fixed joints): "
         << endl;
    for (unsigned int j = 0; j < jointnames.size(); j++)
      cout << jointnames[j] << endl;

    // cout <<"=========Loop Constraints Details======="<<endl;
    for (uint i = 0; i < loop_constraints_submech.size(); i++) {
      cout << "\e[1m"
           << "Loop constraint " << i + 1 << "\e[1m" << endl;
      loop_constraints_submech[i].print_loop_constraints_details();
    }
    cout << "Total number of joints in the submechanisms file (including fixed "
            "joints): "
         << jointnames.size() << endl;
    cout << "====================================" << endl;
  }
};

// Struct for storing a exoskeleton to the mechanism
struct exoskeleton {

  /// \brief Reflects a common name from literature (e.g. stewart platform)
  string name;
  /// \brief Name of the submechanism around which the exoskeleton is defined
  /// (defaults to the full robot when not defined)
  string around;
  /// \brief URDF or lua file path of the submechanism (basically for extraction
  /// of geometric parameters)
  string file_path;

  /// \brief Vector of joint names on which spanning tree joints are dependent
  /// upon
  std::vector<string> jointnames_dependent;
  /// \brief Vector of spanning tree joint names
  std::vector<string> jointnames_spanningtree;
  /// \brief Vector of all joint names including fixed joints
  std::vector<string> jointnames;

  /// \brief Print the exoskeleton details
  void print_exoskeleton_details() {
    cout << "=========Exoskeleton Details=======" << endl;
    cout << "Name: " << name << endl;
    cout << "Joint(s) in spanning tree: " << endl;
    cout << "Loaded file path: " << file_path << endl;
    for (unsigned int j = 0; j < jointnames_spanningtree.size(); j++)
      cout << jointnames_spanningtree[j] << endl;
    cout << "Dependent Joint(s): " << endl;
    for (unsigned int j = 0; j < jointnames_dependent.size(); j++)
      cout << jointnames_dependent[j] << endl;
    cout << "All the Joint(s) in the exo (including fixed joints): " << endl;
    for (unsigned int j = 0; j < jointnames.size(); j++)
      cout << jointnames[j] << endl;
    cout << "====================================" << endl;
  }
};

class SubmechanismsAssembly
    : public ExplicitLoopConstraints::ExplicitLoopConstraintSet {
public:
  /// \brief Vector of available parallel submechanisms
  std::vector<string> available_parallelsubmechanisms;
  /// \brief Vector of submechanisms
  std::vector<submechanism> assembly;
  /// \brief Vector of exoskeletons
  std::vector<exoskeleton> exteriors;
  /// \brief Vector of Explicit Loop Constraint Set for different submechanisms
  std::vector<ExplicitLoopConstraints::ExplicitLoopConstraintSet *>
      submechanism_constraint_set;
  /// \brief Vector of Explicit Loop Constraint Set for different exoskeletons
  std::vector<ExplicitLoopConstraints::ExplicitLoopConstraintSet *>
      exterior_constraint_set;
  /// \brief Environment path variable
  std::string path;

  //! Constructor of the SubmechanismsAssembly class
  SubmechanismsAssembly(std::vector<submechanism> to_assemble,
                        std::vector<exoskeleton> to_externally_attach);
  // Interface for abstract class i.e. ExplicitLoopConstraintSet
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

  /** \brief Returns the actuator selection matrix of size (p x n)
   *
   * \param jointnames_spanningtree vector of spanning tree joint names
   * \param jointnames_active vector of actuator joint names
   */
  void calc_permutationmatrix(std::vector<string> jointnames_spanningtree,
                              std::vector<string> jointnames_active);
  /** \brief Returns the indepedent joints of the robot selection matrix of size
   * (m - floating_dof x m)
   *
   * \param jointnames_independent vector of independent joint names including
   * the free floating joint \param jointnames_independent_robot vector of
   * independent joint names inside the robot
   */
  void
  calc_permutationmatrix2(std::vector<string> jointnames_independent,
                          std::vector<string> jointnames_independent_robot);
  /// \brief Explicit Loop Constraint Set of the full series-parallel hybrid
  /// system
  ExplicitLoopConstraints::ExplicitLoopConstraintSet obj;
};

} // namespace AssembleinHyRoDyn

#endif // SUBMECHANISMSASSEMBLY
