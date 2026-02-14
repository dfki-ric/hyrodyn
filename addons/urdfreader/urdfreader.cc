#include "urdfreader.h"

#include <assert.h>
#include <rbdl/rbdl.h>

#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stack>

#include "transmission_parser.h"

#ifdef RBDL_USE_ROS_URDF_LIBRARY
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <boost/shared_ptr.hpp>

typedef urdf::LinkSharedPtr LinkPtr;
typedef const urdf::LinkConstSharedPtr ConstLinkPtr;
typedef urdf::JointSharedPtr JointPtr;
typedef urdf::ModelInterfaceSharedPtr ModelPtr;
typedef urdf::Joint UrdfJointType;

#define LINKMAP links_
#define JOINTMAP joints_
#define PARENT_TRANSFORM parent_to_joint_origin_transform
#define RPY getRPY
#else
#include <urdf/joint.h>
#include <urdf/link.h>
#include <urdf/model.h>

typedef std::shared_ptr<urdf::Link> LinkPtr;
typedef std::shared_ptr<urdf::Link> ConstLinkPtr;
typedef std::shared_ptr<urdf::Joint> JointPtr;
typedef std::shared_ptr<urdf::UrdfModel> ModelPtr;
typedef urdf::JointType UrdfJointType;

#define LINKMAP link_map
#define JOINTMAP joint_map
#define PARENT_TRANSFORM parent_to_joint_transform
#define RPY getRpy
#endif

using namespace std;
using namespace transmission_interface;

namespace RigidBodyDynamics {

namespace Addons {

using namespace Math;
using namespace Errors;

typedef vector<LinkPtr> URDFLinkVector;
typedef vector<JointPtr> URDFJointVector;
typedef map<string, LinkPtr> URDFLinkMap;
typedef map<string, JointPtr> URDFJointMap;

// =============================================================================

std::string get_model_xml_string_from_file(const char* filename) {
  ifstream model_file(filename);
  if (!model_file) {
    ostringstream error_msg;
    error_msg << "Error opening file '" << filename << "'." << endl;
    throw RBDLFileParseError(error_msg.str());
  }

  // reserve memory for the contents of the file
  string model_xml_string;
  model_file.seekg(0, std::ios::end);
  model_xml_string.reserve(model_file.tellg());
  model_file.seekg(0, std::ios::beg);
  model_xml_string.assign((std::istreambuf_iterator<char>(model_file)),
                          std::istreambuf_iterator<char>());

  model_file.close();
  return model_xml_string;
}

Joint get_rbdl_joint(const JointPtr& urdf_joint) {
  Joint rbdl_joint;
  if (urdf_joint->type == UrdfJointType::REVOLUTE ||
      urdf_joint->type == UrdfJointType::CONTINUOUS) {
    rbdl_joint = Joint(
        SpatialVector(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z, 0., 0., 0.));
  } else if (urdf_joint->type == UrdfJointType::PRISMATIC) {
    rbdl_joint = Joint(
        SpatialVector(0., 0., 0., urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z));
  } else if (urdf_joint->type == UrdfJointType::FIXED) {
    rbdl_joint = Joint(JointTypeFixed);
  } else if (urdf_joint->type == UrdfJointType::FLOATING) {
    // todo: what order of DoF should be used?
    rbdl_joint =
        Joint(SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.),
              SpatialVector(0., 0., 0., 0., 0., 1.), SpatialVector(1., 0., 0., 0., 0., 0.),
              SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(0., 0., 1., 0., 0., 0.));
  } else if (urdf_joint->type == UrdfJointType::PLANAR) {
    // todo: which two directions should be used that are perpendicular
    // to the specified axis?
    ostringstream error_msg;
    error_msg << "Error while processing joint '" << urdf_joint->name
              << "': planar joints not yet supported!" << endl;
    throw RBDLFileParseError(error_msg.str());
  }
  return rbdl_joint;
}

Body get_rbdl_body(ConstLinkPtr& urdf_link, bool is_root_link) {
  // assemble the body
  urdf::Vector3 link_inertial_rpy_temp;
  Vector3d link_inertial_position;
  Vector3d link_inertial_rpy;
  Matrix3d link_inertial_inertia = Matrix3d::Zero();
  double link_inertial_mass = 0.;

  // but only if we actually have inertial data
#ifdef RBDL_USE_ROS_URDF_LIBRARY
  if (urdf_link->inertial) {
    auto I = urdf_link->inertial;
#else
  if (urdf_link->inertial.has_value()) {
    auto I = &urdf_link->inertial.value();
#endif
    link_inertial_mass = I->mass;

    link_inertial_position.set(I->origin.position.x, I->origin.position.y, I->origin.position.z);

    if (!is_root_link) {
      I->origin.rotation.RPY(link_inertial_rpy_temp.x, link_inertial_rpy_temp.y,
                             link_inertial_rpy_temp.z);
      link_inertial_rpy.set(link_inertial_rpy_temp.x, link_inertial_rpy_temp.y,
                            link_inertial_rpy_temp.z);
    }

    link_inertial_inertia(0, 0) = I->ixx;
    link_inertial_inertia(0, 1) = I->ixy;
    link_inertial_inertia(0, 2) = I->ixz;

    link_inertial_inertia(1, 0) = I->ixy;
    link_inertial_inertia(1, 1) = I->iyy;
    link_inertial_inertia(1, 2) = I->iyz;

    link_inertial_inertia(2, 0) = I->ixz;
    link_inertial_inertia(2, 1) = I->iyz;
    link_inertial_inertia(2, 2) = I->izz;

    if (is_root_link) {
      if (link_inertial_mass == 0. && (I->ixx != 0 || I->ixy != 0 || I->ixz != 0 || I->iyy != 0 ||
                                       I->iyz != 0 || I->izz != 0)) {
        std::ostringstream error_msg;
        error_msg << "Error creating rbdl model! Urdf root link (" << urdf_link->name
                  << ") has inertial but no mass!";
        throw RBDLFileParseError(error_msg.str());
      }
    } else {
      if (link_inertial_rpy_temp.x != 0 || link_inertial_rpy_temp.y != 0 ||
          link_inertial_rpy_temp.z != 0) {
        ostringstream error_msg;
        error_msg << "Error while processing body '" << urdf_link->name
                  << "': rotation of body frames not yet supported."
                  << " Please rotate the joint frame instead." << endl;
        throw RBDLFileParseError(error_msg.str());
      }
    }
  }

  Body rbdl_body = Body(link_inertial_mass, link_inertial_position, link_inertial_inertia);

  return rbdl_body;
}

void add_joints_to_rbdl_model(Model* rbdl_model, const URDFLinkMap& link_map,
                              const URDFJointMap& joint_map, const vector<string>& joint_names,
                              bool verbose) {
  unsigned int j;
  for (j = 0; j < joint_names.size(); j++) {
    JointPtr urdf_joint = joint_map.at(joint_names.at(j));
    LinkPtr urdf_parent = link_map.at(urdf_joint->parent_link_name);
    LinkPtr urdf_child = link_map.at(urdf_joint->child_link_name);

    // determine where to add the current joint and child body
    unsigned int rbdl_parent_id = 0;

    rbdl_parent_id = rbdl_model->GetBodyId(urdf_parent->name.c_str());

    if (rbdl_parent_id == std::numeric_limits<unsigned int>::max()) {
      ostringstream error_msg;
      error_msg << "Error while processing joint '" << urdf_joint->name << "': parent link '"
                << urdf_parent->name << "' could not be found." << endl;
      throw RBDLFileParseError(error_msg.str());
    }

    // create the joint
    Joint rbdl_joint = get_rbdl_joint(urdf_joint);

    // compute the joint transformation
    // Temp variable used for compatability between urdf functions
    // and potential Casadi symbolics.
    urdf::Vector3 joint_rpy_temp;

    Vector3d joint_rpy;
    Vector3d joint_translation;
    urdf_joint->PARENT_TRANSFORM.rotation.RPY(joint_rpy_temp.x, joint_rpy_temp.y, joint_rpy_temp.z);
    joint_rpy.set(joint_rpy_temp.x, joint_rpy_temp.y, joint_rpy_temp.z);
    joint_translation.set(urdf_joint->PARENT_TRANSFORM.position.x,
                          urdf_joint->PARENT_TRANSFORM.position.y,
                          urdf_joint->PARENT_TRANSFORM.position.z);
    SpatialTransform rbdl_joint_frame =
        Xrotx(joint_rpy[0]) * Xroty(joint_rpy[1]) * Xrotz(joint_rpy[2]) * Xtrans(joint_translation);
    // rbdl_joint_frame = Xtrans(joint_translation);

    // assemble the body
    Body rbdl_body = get_rbdl_body(urdf_child, false);

    if (rbdl_model->mBodyNameMap.find(urdf_child->name) != rbdl_model->mBodyNameMap.end()) {
      if (verbose) {
        cout << "+ Skipping Add Body: " << urdf_child->name << endl;
      }
      continue;
    }

    if (verbose) {
      cout << "+ Adding Body: " << urdf_child->name << endl;
      cout << "  parent_id  : " << rbdl_parent_id << endl;
      cout << "  joint names: " << joint_names[j] << endl;
      cout << "  joint frame: " << rbdl_joint_frame << endl;
      cout << "  joint dofs : " << rbdl_joint.mDoFCount << endl;
      for (unsigned int j = 0; j < rbdl_joint.mDoFCount; j++) {
        cout << "    " << j << ": " << rbdl_joint.mJointAxes[j].transpose() << endl;
      }
      cout << "  body inertia: " << endl << rbdl_body.mInertia << endl;
      cout << "  body mass   : " << rbdl_body.mMass << endl;
      cout << "  body name   : " << urdf_child->name << endl;
    }

    if (urdf_joint->type == UrdfJointType::FLOATING) {
      Matrix3d zero_matrix = Matrix3d::Zero();
      Body null_body(0., Vector3d::Zero(), zero_matrix);
      Joint joint_txtytz(JointTypeTranslationXYZ);
      string trans_body_name = urdf_child->name + "_Translate";
      rbdl_model->AddBody(rbdl_parent_id, rbdl_joint_frame, joint_txtytz, null_body,
                          trans_body_name);

      Joint joint_euler_zyx(JointTypeEulerXYZ);
      rbdl_model->AppendBody(SpatialTransform(), joint_euler_zyx, rbdl_body, urdf_child->name);
    } else {
      rbdl_model->AddBody(rbdl_parent_id, rbdl_joint_frame, rbdl_joint, rbdl_body,
                          urdf_child->name);
    }
  }
}

void construct_model(Model* rbdl_model, ModelPtr urdf_model, const string& root_link,
                     bool floating_base, bool verbose) {
  LinkPtr urdf_root_link;

  URDFLinkMap link_map = urdf_model->LINKMAP;
  URDFJointMap joint_map = urdf_model->JOINTMAP;

  vector<string> joint_names;

  // Holds the links that we are processing in our depth first traversal
  // with the top element being the current link.
  stack<LinkPtr> link_stack;
  // Holds the child joint index of the current link
  stack<int> joint_index_stack;

  // Check if the parsed root link is a valid one or not
  if (link_map.count(root_link) == 0) {
    ostringstream error_msg;
    error_msg << "Error the parsed root link: '" << root_link << "' could not be found." << endl;
    throw RBDLFileParseError(error_msg.str());
  }
  // add the bodies in a depth-first order of the model tree
  link_stack.push(link_map[root_link]);

  // add the root body
  ConstLinkPtr root = urdf_model->getLink(root_link);
  Body root_link_body = get_rbdl_body(root, true);

  Joint root_joint(JointTypeFixed);
  if (floating_base) {
    root_joint = JointTypeFloatingBase;
  }

  SpatialTransform root_joint_frame = SpatialTransform();

  if (verbose) {
    cout << "+ Adding Root Body " << endl;
    cout << "  joint frame: " << root_joint_frame << endl;
    if (floating_base) {
      cout << "  joint type : floating" << endl;
    } else {
      cout << "  joint type : fixed" << endl;
    }
    cout << "  body inertia: " << endl << root_link_body.mInertia << endl;
    cout << "  body mass   : " << root_link_body.mMass << endl;
    cout << "  body name   : " << root->name << endl;
  }

  rbdl_model->AppendBody(root_joint_frame, root_joint, root_link_body, root->name);

  // depth first traversal: push the first child onto our joint_index_stack
  joint_index_stack.push(0);

  while (link_stack.size() > 0) {
    LinkPtr cur_link = link_stack.top();

    unsigned int joint_idx = joint_index_stack.top();

    // Add any child bodies and increment current joint index if we still
    // have child joints to process.
    if (joint_idx < cur_link->child_joints.size()) {
      JointPtr cur_joint = cur_link->child_joints[joint_idx];

      // increment joint index
      joint_index_stack.pop();
      joint_index_stack.push(joint_idx + 1);

      link_stack.push(link_map[cur_joint->child_link_name]);
      joint_index_stack.push(0);

      if (verbose) {
        for (unsigned int i = 1; i < joint_index_stack.size() - 1; i++) {
          cout << "  ";
        }
        cout << "joint '" << cur_joint->name << "' child link '" << link_stack.top()->name
             << "' type = " << cur_joint->type << endl;
      }

      joint_names.push_back(cur_joint->name);
    } else {
      link_stack.pop();
      joint_index_stack.pop();
    }
  }

  add_joints_to_rbdl_model(rbdl_model, link_map, joint_map, joint_names, verbose);
}
// =============================================================================

void construct_partial_model(Model* rbdl_model, ModelPtr urdf_model, const string& root_link,
                             const vector<string>& tip_links, bool floating_base, bool verbose) {
  LinkPtr urdf_root_link;

  URDFLinkMap link_map = urdf_model->LINKMAP;
  URDFJointMap joint_map = urdf_model->JOINTMAP;

  // Holds the links that we are processing in our depth first traversal
  // with the top element being the current link.
  stack<LinkPtr> link_stack;
  // Holds the child joint index of the current link
  stack<int> joint_index_stack;

  // Check if the parsed root link and tip links are valid or not
  if (link_map.count(root_link) == 0) {
    ostringstream error_msg;
    error_msg << "Error the parsed root link: '" << root_link << "' could not be found." << endl;
    throw RBDLFileParseError(error_msg.str());
  }
  if (tip_links.empty()) {
    ostringstream error_msg;
    error_msg << "Error the parsed tip links cannot be empty!" << endl;
    throw RBDLFileParseError(error_msg.str());
  }
  // add the bodies in a depth-first order of the model tree
  link_stack.push(link_map[root_link]);

  // add the root body
  ConstLinkPtr root = urdf_model->getLink(root_link);
  Body root_link_body = get_rbdl_body(root, true);

  Joint root_joint(JointTypeFixed);
  if (floating_base) {
    root_joint = JointTypeFloatingBase;
  }

  SpatialTransform root_joint_frame = SpatialTransform();

  if (verbose) {
    cout << "+ Adding Root Body " << endl;
    cout << "  joint frame: " << root_joint_frame << endl;
    if (floating_base) {
      cout << "  joint type : floating" << endl;
    } else {
      cout << "  joint type : fixed" << endl;
    }
    cout << "  body inertia: " << endl << root_link_body.mInertia << endl;
    cout << "  body mass   : " << root_link_body.mMass << endl;
    cout << "  body name   : " << root->name << endl;
  }

  rbdl_model->AppendBody(root_joint_frame, root_joint, root_link_body, root->name);

  // depth first traversal: push the first child onto our joint_index_stack
  joint_index_stack.push(0);

  for (const std::string& tip_link : tip_links) {
    vector<string> joint_names;
    vector<string> local_joint_names;
    string parent_link = tip_link;
    vector<string> links_verbose;
    if (link_map.count(tip_link) == 0) {
      ostringstream error_msg;
      error_msg << "Error while processing tip link '" << tip_link
                << "': tip link could not be found in the model." << endl;
      throw RBDLFileParseError(error_msg.str());
    }
    while (parent_link.compare(root_link) != 0) {
      ostringstream verbose_string;
      if (!(link_map[parent_link] && link_map[parent_link]->getParent() &&
            link_map[parent_link]->parent_joint)) {
        ostringstream error_msg;
        error_msg << "Error while processing tip link '" << tip_link
                  << "' as reached root link is '" << parent_link
                  << "', couldn't find desired root link '" << root_link << "' in the tree" << endl;
        throw RBDLFileParseError(error_msg.str());
      } else {
        local_joint_names.push_back(link_map[parent_link]->parent_joint->name);
      }
      parent_link = link_map[parent_link]->getParent()->name;
      if (verbose && link_map[parent_link]->parent_joint) {
        verbose_string << "joint '" << link_map[parent_link]->parent_joint->name << "' child link '"
                       << link_map[parent_link]->parent_joint->child_link_name
                       << "' type = " << link_map[parent_link]->parent_joint->type << endl;
        links_verbose.push_back(verbose_string.str());
      }
    }
    reverse(local_joint_names.begin(), local_joint_names.end());
    reverse(links_verbose.begin(), links_verbose.end());
    joint_names.insert(joint_names.end(), local_joint_names.begin(), local_joint_names.end());
    if (verbose) {
      for (unsigned int i = 0; i < links_verbose.size(); i++) {
        for (unsigned int j = 0; j < i; j++) {
          cout << "  ";
        }
        cout << links_verbose[i];
      }
    }
    add_joints_to_rbdl_model(rbdl_model, link_map, joint_map, joint_names, verbose);
  }
}

RBDL_ADDON_DLLAPI bool URDFReadFromFile(const char* filename, Model* model, bool floating_base,
                                        bool verbose) {
  const string model_xml_string = get_model_xml_string_from_file(filename);

  return URDFReadFromString(model_xml_string.c_str(), model, floating_base, verbose);
}

// =============================================================================

RBDL_ADDON_DLLAPI bool URDFReadFromString(const char* model_xml_string, Model* model,
                                          bool floating_base, bool verbose) {
  assert(model);

#ifdef RBDL_USE_ROS_URDF_LIBRARY
  ModelPtr urdf_model = urdf::parseURDF(model_xml_string);
#else
  ModelPtr urdf_model = urdf::UrdfModel::fromUrdfStr(model_xml_string);
#endif

  construct_model(model, urdf_model, urdf_model->getRoot()->name, floating_base, verbose);

  model->gravity.set(0., 0., -9.81);

  return true;
}

RBDL_ADDON_DLLAPI bool PartialURDFReadFromFile(const char* filename, Model* model,
                                               const std::string& root_link,
                                               const std::vector<std::string>& tip_links,
                                               bool floating_base, bool verbose) {
  const string model_xml_string = get_model_xml_string_from_file(filename);

  return PartialURDFReadFromString(model_xml_string.c_str(), model, root_link, tip_links,
                                   floating_base, verbose);
}

RBDL_ADDON_DLLAPI bool PartialURDFReadFromString(const char* model_xml_string, Model* model,
                                                 const std::string& root_link,
                                                 const std::vector<std::string>& tip_links,
                                                 bool floating_base, bool verbose) {
  assert(model);

#ifdef RBDL_USE_ROS_URDF_LIBRARY
  ModelPtr urdf_model = urdf::parseURDF(model_xml_string);
#else
  ModelPtr urdf_model = urdf::UrdfModel::fromUrdfStr(model_xml_string);
#endif

  if (tip_links.empty()) {
    construct_model(model, urdf_model, root_link, floating_base, verbose);
  } else {
    construct_partial_model(model, urdf_model, root_link, tip_links, floating_base, verbose);
  }

  model->gravity.set(0., 0., -9.81);
  return true;
}
// Finished as same

/*
RBDL_ADDON_DLLAPI bool URDFReadLoopClosureFunction(
    const char* filename, MatrixN_t& G, VectorN_t& offset, MatrixN_t& Gu,
    const std::vector<std::string>& actuated_joint_names,
    const std::vector<std::string>& tree_joint_names) {
  ifstream model_file(filename);
  if (!model_file) {
    ostringstream error_msg;
    error_msg << "Error opening file '" << filename << "'." << endl;
    throw RBDLFileParseError(error_msg.str());
  }

  // reserve memory for the contents of the file
  string model_xml_string;
  model_file.seekg(0, std::ios::end);
  model_xml_string.reserve(model_file.tellg());
  model_file.seekg(0, std::ios::beg);
  model_xml_string.assign((std::istreambuf_iterator<char>(model_file)),
                          std::istreambuf_iterator<char>());
  model_file.close();

  // Parse URDF
  ModelPtr urdf_model = urdf::parseURDF(model_xml_string);

  LinkPtr urdf_root_link;

  URDFLinkMap link_map;
  link_map = urdf_model->links_;

  URDFJointMap joint_map;
  joint_map = urdf_model->joints_;

  vector<string> joint_names;

  stack<LinkPtr> link_stack;
  stack<int> joint_index_stack;

  // add the bodies in a depth-first order of the model tree
  link_stack.push(link_map[(urdf_model->getRoot()->name)]);

  // add the root body
  ConstLinkPtr& root = urdf_model->getRoot();

  // depth first traversal: push the first child onto our joint_index_stack
  joint_index_stack.push(0);

  while (link_stack.size() > 0) {
    LinkPtr cur_link = link_stack.top();
    unsigned int joint_idx = joint_index_stack.top();

    if (joint_idx < cur_link->child_joints.size()) {
      JointPtr cur_joint = cur_link->child_joints[joint_idx];

      // increment joint index
      joint_index_stack.pop();
      joint_index_stack.push(joint_idx + 1);

      link_stack.push(link_map[cur_joint->child_link_name]);
      joint_index_stack.push(0);

      joint_names.push_back(cur_joint->name);
    } else {
      link_stack.pop();
      joint_index_stack.pop();
    }
  }

  // Collect all tree joint names and actuated joint names
  // vector<string> tree_joint_names;
  // vector<string> actuated_joint_names;
  unsigned int y, z;

  for (z = 0; z < joint_names.size(); z++) {
    JointPtr urdf_joint = joint_map[joint_names[z]];
    if (urdf_joint->type == urdf::Joint::FLOATING) {
      cout << "Floating base systems are not supported yet." << endl;
      return false;
    }
    if (urdf_joint->type == urdf::Joint::PLANAR) {
      cout << "Planar joints are not yet supported in RBDL URDF parsing." << endl;
      return false;
    }
    if (!(urdf_joint->type == urdf::Joint::FIXED)) {
      tree_joint_names.push_back(joint_names[z]);
      if (!(urdf_joint->mimic)) {
        actuated_joint_names.push_back(joint_names[z]);
      }
    }
  }

  // Allocate size and set zeros
  G.setZero(tree_joint_names.size(), actuated_joint_names.size());
  Gu.setZero(actuated_joint_names.size(), actuated_joint_names.size());
  offset.setZero(tree_joint_names.size());

  // Check if there are mimic joints in the spanning tree
  if (tree_joint_names.size() == actuated_joint_names.size()) {
    cout << "No mimic joints found. Calling this function has no meaning!" << endl;
    return false;
  }

  // Build matrix G and vector offset
  for (y = 0; y < actuated_joint_names.size(); y++) {
    for (z = 0; z < tree_joint_names.size(); z++) {
      if (actuated_joint_names[y] == tree_joint_names[z]) {
        G(z, y) = 1;
      }
      JointPtr urdf_tree_joint = joint_map[tree_joint_names[z]];
      if (urdf_tree_joint->mimic) {
        if (urdf_tree_joint->mimic->joint_name == actuated_joint_names[y])
          G(z, y) = urdf_tree_joint->mimic->multiplier;
        offset(z) = urdf_tree_joint->mimic->offset;
      }
    }
  }

  // Build matrix Gu
  for (y = 0; y < actuated_joint_names.size(); y++) {
    for (z = 0; z < tree_joint_names.size(); z++) {
      if (actuated_joint_names[y] == tree_joint_names[z]) {
        Gu.row(y) = G.row(z);
      }
    }
  }

  return true;
}

RBDL_ADDON_DLLAPI bool URDFReadLoopClosureFunctionExterior(
    const char* filename, MatrixN_t& G, VectorN_t& offset,
    const std::vector<std::string>& actuated_joint_names,
    const std::vector<std::string>& tree_joint_names) {
  ifstream model_file(filename);
  if (!model_file) {
    ostringstream error_msg;
    error_msg << "Error opening file '" << filename << "'." << endl;
    throw RBDLFileParseError(error_msg.str());
  }

  // reserve memory for the contents of the file
  string model_xml_string;
  model_file.seekg(0, std::ios::end);
  model_xml_string.reserve(model_file.tellg());
  model_file.seekg(0, std::ios::beg);
  model_xml_string.assign((std::istreambuf_iterator<char>(model_file)),
                          std::istreambuf_iterator<char>());
  model_file.close();

  // Parse URDF
  ModelPtr urdf_model = urdf::parseURDF(model_xml_string);

  LinkPtr urdf_root_link;

  URDFLinkMap link_map;
  link_map = urdf_model->links_;

  URDFJointMap joint_map;
  joint_map = urdf_model->joints_;

  vector<string> joint_names;

  stack<LinkPtr> link_stack;
  stack<int> joint_index_stack;

  // add the bodies in a depth-first order of the model tree
  link_stack.push(link_map[(urdf_model->getRoot()->name)]);

  // add the root body
  ConstLinkPtr& root = urdf_model->getRoot();

  // depth first traversal: push the first child onto our joint_index_stack
  joint_index_stack.push(0);

  while (link_stack.size() > 0) {
    LinkPtr cur_link = link_stack.top();
    unsigned int joint_idx = joint_index_stack.top();

    if (joint_idx < cur_link->child_joints.size()) {
      JointPtr cur_joint = cur_link->child_joints[joint_idx];

      // increment joint index
      joint_index_stack.pop();
      joint_index_stack.push(joint_idx + 1);

      link_stack.push(link_map[cur_joint->child_link_name]);
      joint_index_stack.push(0);

      joint_names.push_back(cur_joint->name);
    } else {
      link_stack.pop();
      joint_index_stack.pop();
    }
  }

  // Collect all tree joint names and actuated joint names
  // vector<string> tree_joint_names;
  // vector<string> actuated_joint_names;
  unsigned int y, z;

  // Allocate size and set zeros
  G.setZero(tree_joint_names.size(), actuated_joint_names.size());
  offset.setZero(tree_joint_names.size());

  // Build matrix G and vector offset
  for (y = 0; y < actuated_joint_names.size(); y++) {
    for (z = 0; z < tree_joint_names.size(); z++) {
      JointPtr urdf_tree_joint = joint_map[tree_joint_names[z]];
      if (urdf_tree_joint->mimic) {
        if (urdf_tree_joint->mimic->joint_name == actuated_joint_names[y])
          G(z, y) = urdf_tree_joint->mimic->multiplier;
        offset(z) = urdf_tree_joint->mimic->offset;
      }
    }
  }

  return true;
}

RBDL_ADDON_DLLAPI bool URDFReadJointLimits(
    const char* filename, const std::vector<std::string>& joint_names_respecting_modularity,
    VectorN_t& q_max, VectorN_t& q_min, VectorN_t& vel_limit, VectorN_t& effort_limit) {
  ifstream model_file(filename);
  if (!model_file) {
    ostringstream error_msg;
    error_msg << "Error opening file '" << filename << "'." << endl;
    throw RBDLFileParseError(error_msg.str());
  }

  // reserve memory for the contents of the file
  string model_xml_string;
  model_file.seekg(0, std::ios::end);
  model_xml_string.reserve(model_file.tellg());
  model_file.seekg(0, std::ios::beg);
  model_xml_string.assign((std::istreambuf_iterator<char>(model_file)),
                          std::istreambuf_iterator<char>());
  model_file.close();

  // Parse URDF
  ModelPtr urdf_model = urdf::parseURDF(model_xml_string);

  URDFJointMap joint_map;
  joint_map = urdf_model->joints_;

  unsigned int y = 0;

  // Allocate size and set zeros
  q_max.setZero(joint_names_respecting_modularity.size());
  q_min.setZero(joint_names_respecting_modularity.size());
  vel_limit.setZero(joint_names_respecting_modularity.size());
  effort_limit.setZero(joint_names_respecting_modularity.size());

  for (unsigned int z = 0; z < joint_names_respecting_modularity.size(); z++) {
    JointPtr urdf_joint = joint_map[joint_names_respecting_modularity[z]];
    if (urdf_joint->type == urdf::Joint::FLOATING) {
      cout << "Floating base systems are not supported yet." << endl;
      return false;
    }
    if (urdf_joint->type == urdf::Joint::PLANAR) {
      cout << "Planar joints are not yet supported in RBDL URDF parsing." << endl;
      return false;
    }
    if (!(urdf_joint->type == urdf::Joint::FIXED)) {
      q_min(y) = urdf_joint->limits->lower;
      q_max(y) = urdf_joint->limits->upper;
      vel_limit(y) = urdf_joint->limits->velocity;
      effort_limit(y) = urdf_joint->limits->effort;
      y++;
    }
  }
  return true;
}

bool construct_model_with_modularity(Model* rbdl_model, ModelPtr urdf_model,
                                     std::vector<std::string> joint_names_respecting_modularity,
                                     bool floating_base, bool verbose) {
  LinkPtr urdf_root_link;

  URDFLinkMap link_map;
  link_map = urdf_model->links_;

  URDFJointMap joint_map;
  joint_map = urdf_model->joints_;

  vector<string> joint_names;

  // Holds the links that we are processing in our depth first traversal with the top element being
  // the current link.
  stack<LinkPtr> link_stack;
  // Holds the child joint index of the current link
  stack<int> joint_index_stack;

  // add the bodies in a depth-first order of the model tree
  link_stack.push(link_map[(urdf_model->getRoot()->name)]);

  // add the root body
  ConstLinkPtr& root = urdf_model->getRoot();
  Vector3d root_inertial_rpy = Vector3d::Zero();
  Vector3d root_inertial_position = Vector3d::Zero();
  Matrix3d root_inertial_inertia = Matrix3d::Zero();
  double root_inertial_mass;
  if (root->inertial) {
    root_inertial_mass = root->inertial->mass;

    root_inertial_position.set(root->inertial->origin.position.x, root->inertial->origin.position.y,
                               root->inertial->origin.position.z);

    root_inertial_inertia(0, 0) = root->inertial->ixx;
    root_inertial_inertia(0, 1) = root->inertial->ixy;
    root_inertial_inertia(0, 2) = root->inertial->ixz;

    root_inertial_inertia(1, 0) = root->inertial->ixy;
    root_inertial_inertia(1, 1) = root->inertial->iyy;
    root_inertial_inertia(1, 2) = root->inertial->iyz;

    root_inertial_inertia(2, 0) = root->inertial->ixz;
    root_inertial_inertia(2, 1) = root->inertial->iyz;
    root_inertial_inertia(2, 2) = root->inertial->izz;

    root->inertial->origin.rotation.getRPY(root_inertial_rpy[0], root_inertial_rpy[1],
                                           root_inertial_rpy[2]);

    Body root_link = Body(root_inertial_mass, root_inertial_position, root_inertial_inertia);

    Joint root_joint(JointTypeFixed);
    if (floating_base) {
      root_joint = JointTypeFloatingBase;
    }

    SpatialTransform root_joint_frame = SpatialTransform();

    if (verbose) {
      cout << "+ Adding Root Body " << endl;
      cout << "  joint frame: " << root_joint_frame << endl;
      if (floating_base) {
        cout << "  joint type : floating" << endl;
      } else {
        cout << "  joint type : fixed" << endl;
      }
      cout << "  body inertia: " << endl << root_link.mInertia << endl;
      cout << "  body mass   : " << root_link.mMass << endl;
      cout << "  body name   : " << root->name << endl;
    }

    rbdl_model->AppendBody(root_joint_frame, root_joint, root_link, root->name);
  }

  // depth first traversal: push the first child onto our joint_index_stack
  joint_index_stack.push(0);

  while (link_stack.size() > 0) {
    LinkPtr cur_link = link_stack.top();

    unsigned int joint_idx = joint_index_stack.top();

    // Add any child bodies and increment current joint index if we still have child joints to
    // process.
    if (joint_idx < cur_link->child_joints.size()) {
      JointPtr cur_joint = cur_link->child_joints[joint_idx];

      // increment joint index
      joint_index_stack.pop();
      joint_index_stack.push(joint_idx + 1);

      link_stack.push(link_map[cur_joint->child_link_name]);
      joint_index_stack.push(0);

      if (verbose) {
        for (unsigned int i = 1; i < joint_index_stack.size() - 1; i++) {
          cout << "  ";
        }
        cout << "joint '" << cur_joint->name << "' child link '" << link_stack.top()->name
             << "' type = " << cur_joint->type << endl;
      }

      joint_names.push_back(cur_joint->name);
    } else {
      link_stack.pop();
      joint_index_stack.pop();
    }
  }

  // HyRoDyn stuff starts here
  if (verbose) {
    cout << "Joint names from spanning tree in URDF: " << endl;
    for (unsigned int j = 0; j < joint_names.size(); j++)
      cout << "j: " << j << " " << joint_names[j] << endl;
  }

  // if there is a fixed joint found, insert it into the joint names respecting modularity vector so
  // that user does not have to define in submechanisms.yml file

  // Fixed joint processing is used when joint_names_respecting_modularity vector (joint names
  // provided by the submechanism definition) size is different from joint_names vector (joint names
  // extracted from the URDF) size.
  if (joint_names_respecting_modularity.size() != joint_names.size()) {
    if (verbose) {
      cout
          << "joint_names_respecting_modularity vector (joint names provided by the submechanism "
             "definition) size = "
          << joint_names_respecting_modularity.size()
          << " is different from joint_names vector (joint names extracted from the URDF) size = "
          << joint_names.size()
          << ". Hence, automatic fixed joint processing of HyRoDyn will be used. Handle with care. "
             "May not always work!"
          << endl;
    }

    unsigned int jn = 0;

    // Case 1: fixed joint group (minimum size 1) attached to root link
    JointPtr urdf_joint = joint_map[joint_names[jn]];
    if (urdf_joint->type == urdf::Joint::FIXED) {
      std::vector<string> fixed_jointnames_root;
      if (verbose) {
        cout << "root fixed joint found: " << joint_names[jn] << endl;
      }
      fixed_jointnames_root.push_back(joint_names[jn]);
      for (unsigned int kn = 1; kn < joint_names.size(); kn++) {
        JointPtr urdf_joint = joint_map[joint_names[kn]];
        if (urdf_joint->type == urdf::Joint::FIXED) {
          if (verbose) {
            cout << "root fixed joint found in group: " << joint_names[kn] << endl;
          }
          fixed_jointnames_root.push_back(joint_names[kn]);
        } else
          break;
      }
      // insert the fixed joints at root in the modular joint names vector
      joint_names_respecting_modularity.insert(joint_names_respecting_modularity.begin(),
                                               fixed_jointnames_root.begin(),
                                               fixed_jointnames_root.end());
      jn = jn + fixed_jointnames_root.size();
    }
    //

    // Case 2: fixed joint group (minimum size 1) found in between
    while (jn < joint_names.size()) {
    here:
      if (verbose) {
        cout << "jn = " << jn << endl;
      }
      JointPtr urdf_joint = joint_map[joint_names[jn]];
      if (urdf_joint->type == urdf::Joint::FIXED) {
        std::vector<string> fixed_jointnames_inbetween;
        if (verbose) {
          cout << "in-between fixed joint found: " << joint_names[jn] << endl;
        }
        fixed_jointnames_inbetween.push_back(joint_names[jn]);
        // Add further fixed joints into the block
        for (unsigned int kn = jn + 1; kn < joint_names.size(); kn++) {
          JointPtr urdf_joint = joint_map[joint_names[kn]];
          if (urdf_joint->type == urdf::Joint::FIXED) {
            if (verbose) {
              cout << "in-between fixed joint found in group: " << joint_names[kn] << endl;
            }
            fixed_jointnames_inbetween.push_back(joint_names[kn]);
          } else
            break;
        }

        // idea: find predecessor joint and insert the fixed joints in-between vector in the modular
        // joint names vector string predecessor_joint_name = joint_names[jn-1];	// trivial
        // thought, may not always work
        string predecessor_link_name = urdf_joint->parent_link_name;
        LinkPtr predecessor_link = link_map[predecessor_link_name];
        string predecessor_joint_name;
        if (predecessor_link_name != urdf_model->getRoot()->name) {
          if (verbose) {
            cout << "Predecessor link is not root link" << endl;
          }
          predecessor_joint_name = predecessor_link->parent_joint->name;
        } else {
          if (verbose) {
            cout << "Predecessor link is a root link!" << endl;
          }
          joint_names_respecting_modularity.insert(joint_names_respecting_modularity.begin(),
                                                   fixed_jointnames_inbetween.begin(),
                                                   fixed_jointnames_inbetween.end());
          jn = jn + fixed_jointnames_inbetween.size();
          goto here;
        }
        bool successor_joint_flag = false;
        string successor_joint_name;

        if ((jn + fixed_jointnames_inbetween.size()) < joint_names.size()) {
          successor_joint_name = joint_names[jn + fixed_jointnames_inbetween.size()];
          if (verbose) {
            cout << "predecessor joint name " << predecessor_joint_name << ", successor joint name "
                 << successor_joint_name << endl;
          }
          successor_joint_flag = true;
        } else {
          if (verbose) {
            cout << "predecessor joint name " << predecessor_joint_name
                 << ", successor joint does not exist " << endl;
          }
        }

        unsigned int index_predecessor = 0, index_successor = 0;

        if (std::find(joint_names_respecting_modularity.begin(),
                      joint_names_respecting_modularity.end(),
                      predecessor_joint_name) != joint_names_respecting_modularity.end()) {
          auto it_p = std::find(joint_names_respecting_modularity.begin(),
                                joint_names_respecting_modularity.end(), predecessor_joint_name);
          auto index_p = std::distance(joint_names_respecting_modularity.begin(), it_p);
          if (verbose) {
            cout << "predecessor joint with name " << predecessor_joint_name
                 << " to the fixed joint group found at " << index_p << endl;
          }
          index_predecessor = index_p;
        }

        if (std::find(joint_names_respecting_modularity.begin(),
                      joint_names_respecting_modularity.end(),
                      successor_joint_name) != joint_names_respecting_modularity.end()) {
          auto it_s = std::find(joint_names_respecting_modularity.begin(),
                                joint_names_respecting_modularity.end(), successor_joint_name);
          auto index_s = std::distance(joint_names_respecting_modularity.begin(), it_s);
          if (verbose) {
            cout << "successor joint with name " << successor_joint_name
                 << " to the fixed joint group found at " << index_s << endl;
          }
          index_successor = index_s;
        }
        if (index_predecessor < index_successor) {
          joint_names_respecting_modularity.insert(
              joint_names_respecting_modularity.begin() + index_successor,
              fixed_jointnames_inbetween.begin(), fixed_jointnames_inbetween.end());
          if (verbose) {
            cout << "Fixed joint successfully inserted after successor joint name" << endl;
          }
        } else {
          joint_names_respecting_modularity.insert(
              joint_names_respecting_modularity.begin() + index_predecessor + 1,
              fixed_jointnames_inbetween.begin(), fixed_jointnames_inbetween.end());
          if (verbose) {
            cout << "Fixed joint successfully inserted after predecessor joint name" << endl;
          }
        }
        jn = jn + fixed_jointnames_inbetween.size();
      } else
        jn = jn + 1;
    }
  }

  //	std::sort(joint_names.begin(), joint_names.end());
  if (verbose) {
    cout << "Joint names respecting modularity with fixed joints: " << endl;
    for (unsigned int j = 0; j < joint_names_respecting_modularity.size(); j++)
      cout << "j: " << j << " " << joint_names_respecting_modularity[j] << endl;
  }
  // sorted joint names variables for comparison(thats the sole purpose of sorting joint names)
  std::vector<string> sorted_jointnames, sorted_jointnames_spanningtree;

  sorted_jointnames = joint_names;
  std::sort(sorted_jointnames.begin(), sorted_jointnames.end());
  if (verbose) {
    cout << "Joint names from URDF after sorting for comparison" << endl;
    for (unsigned int j = 0; j < sorted_jointnames.size(); j++)
      cout << "j: " << j << " " << sorted_jointnames[j] << endl;
  }
  sorted_jointnames_spanningtree = joint_names_respecting_modularity;
  std::sort(sorted_jointnames_spanningtree.begin(), sorted_jointnames_spanningtree.end());
  if (verbose) {
    cout << "Joint names from submechanisms file with fixed joints after sorting for comparison"
         << endl;
    for (unsigned int j = 0; j < sorted_jointnames_spanningtree.size(); j++)
      cout << "j: " << j << " " << sorted_jointnames_spanningtree[j] << endl;
  }
  bool is_equal = false;
  if (sorted_jointnames.size() == sorted_jointnames_spanningtree.size()) {
    is_equal = std::equal(sorted_jointnames.begin(), sorted_jointnames.end(),
                          sorted_jointnames_spanningtree.begin());
    if (is_equal) {
      if (verbose) {
        cout << "Joint names respecting modularity and joint names from URDF are consistent. RBDL "
                "model will now respect the modular numbering scheme."
             << endl;
      }
      joint_names = joint_names_respecting_modularity;
    } else {
      ostringstream error_msg;
      error_msg
          << "Names mismatch: Joint names respecting modularity and joint names from URDF are "
             "inconsistent"
          << endl;
      error_msg
          << "Sorted joint names parsed from URDF and regular numbering scheme for comparison:"
          << endl;
      for (unsigned int j = 0; j < joint_names.size(); j++) {
        error_msg << "index: " << j << " " << sorted_jointnames[j] << ", "
                  << sorted_jointnames_spanningtree[j] << endl;
        if (sorted_jointnames[j] != sorted_jointnames_spanningtree[j])
          error_msg << "Mismatch here!" << endl;
      }

      error_msg << "Aborting..." << endl;
      throw RBDLFileParseError(error_msg.str());
    }
  } else {
    ostringstream error_msg;
    error_msg << "Size of joint names in URDF: " << joint_names.size() << endl;
    error_msg << "Size of modular joint names provided: "
              << joint_names_respecting_modularity.size() << endl;
    error_msg << "Size mismatch: Joint names respecting modularity and joint names from URDF are "
                 "inconsistent"
              << endl;
    throw RBDLFileParseError(error_msg.str());
  }
  if (verbose) {
    cout << "Joint names respecting modularity: " << endl;
    for (unsigned int j = 0; j < joint_names.size(); j++)
      cout << "j: " << j << " " << joint_names[j] << endl;
  }
  // HyRoDyn stuff ends here

  unsigned int j;
  for (j = 0; j < joint_names.size(); j++) {
    JointPtr urdf_joint = joint_map[joint_names[j]];
    LinkPtr urdf_parent = link_map[urdf_joint->parent_link_name];
    LinkPtr urdf_child = link_map[urdf_joint->child_link_name];

    // determine where to add the current joint and child body
    unsigned int rbdl_parent_id = 0;
    rbdl_parent_id = rbdl_model->GetBodyId(urdf_parent->name.c_str());

    if (rbdl_parent_id == std::numeric_limits<unsigned int>::max()) {
      // TIP: if you enter this for the root link of the robot, make sure the root link had inertial
      // properties defined (all zeros are also fine)
      ostringstream error_msg;
      error_msg << "Error while processing joint '" << urdf_joint->name << "': parent link '"
                << urdf_parent->name << "' could not be found." << endl;
      throw RBDLFileParseError(error_msg.str());
    }

    // cout << "joint: " << urdf_joint->name << "\tparent = " << urdf_parent->name << " child = " <<
    // urdf_child->name << " parent_id = " << rbdl_parent_id << endl;

    // create the joint
    Joint rbdl_joint;
    if (urdf_joint->type == urdf::Joint::REVOLUTE || urdf_joint->type == urdf::Joint::CONTINUOUS) {
      SpatialVector axis(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z, 0., 0., 0.);
      if (fabs(axis.norm() - 1.0) > 1.0e-2) {
        throw std::runtime_error("Joint axis of joint" + joint_names[j] +
                                 " is not unit. Fix your URDF file.");
      } else if (fabs(axis.norm() - 1.0) > 1.0e-8) {
        std::cerr << "Warning: joint axis is not unit! It will be normalized automatically."
                  << std::endl;
        axis /= axis.norm();
      }
      rbdl_joint = Joint(axis);
      //      cout<<"urdf joint limits: "<<urdf_joint->limits->lower<<endl;
    } else if (urdf_joint->type == urdf::Joint::PRISMATIC) {
      SpatialVector axis(0., 0., 0., urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z);
      if (fabs(axis.norm() - 1.0) > 1.0e-2) {
        throw std::runtime_error("Joint axis of joint" + joint_names[j] +
                                 " is not unit. Fix your URDF file.");
      } else if (fabs(axis.norm() - 1.0) > 1.0e-8) {
        std::cerr << "Warning: joint axis is not unit! It will be normalized automatically."
                  << std::endl;
        axis /= axis.norm();
      }
      rbdl_joint = Joint(axis);
    } else if (urdf_joint->type == urdf::Joint::FIXED) {
      rbdl_joint = Joint(JointTypeFixed);
    } else if (urdf_joint->type == urdf::Joint::FLOATING) {
      // todo: what order of DoF should be used?
      rbdl_joint =
          Joint(SpatialVector(0., 0., 0., 1., 0., 0.), SpatialVector(0., 0., 0., 0., 1., 0.),
                SpatialVector(0., 0., 0., 0., 0., 1.), SpatialVector(1., 0., 0., 0., 0., 0.),
                SpatialVector(0., 1., 0., 0., 0., 0.), SpatialVector(0., 0., 1., 0., 0., 0.));
    } else if (urdf_joint->type == urdf::Joint::PLANAR) {
      // todo: which two directions should be used that are perpendicular
      // to the specified axis?
      cerr << "Error while processing joint '" << urdf_joint->name
           << "': planar joints not yet supported!" << endl;
      return false;
    }

    // compute the joint transformation
    Vector3d joint_rpy;
    Vector3d joint_translation;
    urdf_joint->parent_to_joint_origin_transform.rotation.getRPY(joint_rpy[0], joint_rpy[1],
                                                                 joint_rpy[2]);
    joint_translation.set(urdf_joint->parent_to_joint_origin_transform.position.x,
                          urdf_joint->parent_to_joint_origin_transform.position.y,
                          urdf_joint->parent_to_joint_origin_transform.position.z);
    SpatialTransform rbdl_joint_frame =
        Xrot(joint_rpy[0], Vector3d(1., 0., 0.)) * Xrot(joint_rpy[1], Vector3d(0., 1., 0.)) *
        Xrot(joint_rpy[2], Vector3d(0., 0., 1.)) * Xtrans(Vector3d(joint_translation));

    // assemble the body
    Vector3d link_inertial_position = Vector3d::Zero();
    Vector3d link_inertial_rpy = Vector3d::Zero();
    Matrix3d link_inertial_inertia = Matrix3d::Zero();
    double link_inertial_mass = 0.;

    // but only if we actually have inertial data
    if (urdf_child->inertial) {
      link_inertial_mass = urdf_child->inertial->mass;

      link_inertial_position.set(urdf_child->inertial->origin.position.x,
                                 urdf_child->inertial->origin.position.y,
                                 urdf_child->inertial->origin.position.z);
      urdf_child->inertial->origin.rotation.getRPY(link_inertial_rpy[0], link_inertial_rpy[1],
                                                   link_inertial_rpy[2]);

      link_inertial_inertia(0, 0) = urdf_child->inertial->ixx;
      link_inertial_inertia(0, 1) = urdf_child->inertial->ixy;
      link_inertial_inertia(0, 2) = urdf_child->inertial->ixz;

      link_inertial_inertia(1, 0) = urdf_child->inertial->ixy;
      link_inertial_inertia(1, 1) = urdf_child->inertial->iyy;
      link_inertial_inertia(1, 2) = urdf_child->inertial->iyz;

      link_inertial_inertia(2, 0) = urdf_child->inertial->ixz;
      link_inertial_inertia(2, 1) = urdf_child->inertial->iyz;
      link_inertial_inertia(2, 2) = urdf_child->inertial->izz;

      if (link_inertial_rpy != Vector3d(0., 0., 0.)) {
        cerr << "Error while processing body '" << urdf_child->name
             << "': rotation of body frames not yet supported. Please rotate the joint frame "
                "instead."
             << endl;
        return false;
      }
    }

    Body rbdl_body = Body(link_inertial_mass, link_inertial_position, link_inertial_inertia);

    if (verbose) {
      cout << "+ Adding Body: " << urdf_child->name << endl;
      cout << "  parent_id  : " << rbdl_parent_id << endl;
      cout << "  joint frame: " << rbdl_joint_frame << endl;
      cout << "  joint dofs : " << rbdl_joint.mDoFCount << endl;
      for (unsigned int j = 0; j < rbdl_joint.mDoFCount; j++) {
        cout << "    " << j << ": " << rbdl_joint.mJointAxes[j].transpose() << endl;
      }
      cout << "  body inertia: " << endl << rbdl_body.mInertia << endl;
      cout << "  body mass   : " << rbdl_body.mMass << endl;
      cout << "  body name   : " << urdf_child->name << endl;
    }

    if (urdf_joint->type == urdf::Joint::FLOATING) {
      Matrix3d zero_matrix = Matrix3d::Zero();
      Body null_body(0., Vector3d::Zero(3), zero_matrix);
      Joint joint_txtytz(JointTypeTranslationXYZ);
      string trans_body_name = urdf_child->name + "_Translate";
      rbdl_model->AddBody(rbdl_parent_id, rbdl_joint_frame, joint_txtytz, null_body,
                          trans_body_name);

      Joint joint_euler_zyx(JointTypeEulerXYZ);
      rbdl_model->AppendBody(SpatialTransform(), joint_euler_zyx, rbdl_body, urdf_child->name);
    } else {
      rbdl_model->AddBody(rbdl_parent_id, rbdl_joint_frame, rbdl_joint, rbdl_body,
                          urdf_child->name);
    }
  }
  cout << "URDF parsed successfully" << endl;
  return true;
}
*/
/*
RBDL_ADDON_DLLAPI bool URDFReadFromFileWithModularity(const char* filename, Model* model,
                                                      const std::vector<std::string>& joint_names,
                                                      bool floating_base, bool verbose = false) {
  ifstream model_file(filename);
  if (!model_file) {
    ostringstream error_msg;
    error_msg << "Error opening file '" << filename << "'." << endl;
    throw RBDLFileParseError(error_msg.str());
  }

  // reserve memory for the contents of the file
  string model_xml_string;
  model_file.seekg(0, std::ios::end);
  model_xml_string.reserve(model_file.tellg());
  model_file.seekg(0, std::ios::beg);
  model_xml_string.assign((std::istreambuf_iterator<char>(model_file)),
                          std::istreambuf_iterator<char>());

  model_file.close();

  return URDFReadFromStringWithModularity(model_xml_string.c_str(), model, joint_names,
                                          floating_base, verbose);
}

RBDL_ADDON_DLLAPI bool URDFReadFromStringWithModularity(const char* model_xml_string, Model* model,
                                                        const std::vector<std::string>& joint_names,
                                                        bool floating_base, bool verbose = false) {
  assert(model);

  ModelPtr urdf_model = urdf::parseURDF(model_xml_string);

  if (!construct_model_with_modularity(model, urdf_model, joint_names, floating_base, verbose)) {
    ostringstream error_msg;
    error_msg << "Error constructing model from urdf file." << endl;
    throw RBDLFileParseError(error_msg.str());
  }

  model->gravity.set(0., 0., -9.81);

  return true;
}
*/

// Transmission LCF
bool URDFReadLoopClosureFunctionTransmission(
    const char* filename, MatrixN_t& G, VectorN_t& offset,
    const std::vector<std::string>& independent_joint_names,
    const std::vector<std::string>& tree_joint_names, bool verbose) {
  ifstream model_file(filename);
  if (!model_file) {
    ostringstream error_msg;
    error_msg << "Error opening file '" << filename << "'." << endl;
    throw RBDLFileParseError(error_msg.str());
  }
  // reserve memory for the contents of the file
  string model_xml_string;
  model_file.seekg(0, std::ios::end);
  model_xml_string.reserve(model_file.tellg());
  model_file.seekg(0, std::ios::beg);
  model_xml_string.assign((std::istreambuf_iterator<char>(model_file)),
                          std::istreambuf_iterator<char>());
  model_file.close();

  std::vector<TransmissionInfo> transmissions;
  TransmissionParser parser_obj;
  parser_obj.Parse(model_xml_string, transmissions);
  if (verbose) {
    cout << "No. of transmissions: " << transmissions.size() << endl;
    for (unsigned int i = 0; i < transmissions.size(); i++) {
      cout << "Name: " << transmissions[i].name_ << endl;
      cout << "Type: " << transmissions[i].type_ << endl;
      cout << "No. of joints: " << transmissions[i].joints_.size() << endl;
      for (unsigned int j = 0; j < transmissions[i].joints_.size(); j++) {
        cout << "Name: " << transmissions[i].joints_[j].name_ << endl;
        cout << "No. of joints on which it depends: "
             << transmissions[i].joints_[j].dependent_joints_.size() << endl;
        for (unsigned int k = 0; k < transmissions[i].joints_[j].dependent_joints_.size(); k++) {
          cout << "Depending Joint Name: " << transmissions[i].joints_[j].dependent_joints_[k].name
               << ", multiplier: " << transmissions[i].joints_[j].dependent_joints_[k].multiplier
               << ", offset: " << transmissions[i].joints_[j].dependent_joints_[k].offset << endl;
        }
      }
    }
  }

  // Allocate sizes and set zeros
  G.setZero(tree_joint_names.size(), independent_joint_names.size());
  offset.setZero(tree_joint_names.size());

  // Check if spanning tree joints size equals to actuated joints size
  if (tree_joint_names.size() == independent_joint_names.size()) {
    cerr << "Number of tree joints equals to number of actuators. Calling this "
            "function has no meaning! Note: Spanning tree joints should always "
            "contain the active joints."
         << endl;
    return false;
  }

  // Build matrix G and vector offset
  for (uint y = 0; y < independent_joint_names.size(); y++) {
    for (uint z = 0; z < tree_joint_names.size(); z++) {
      if (independent_joint_names[y] == tree_joint_names[z]) {
        G(z, y) = 1;
      }
      for (unsigned int i = 0; i < transmissions.size(); i++) {
        for (unsigned int j = 0; j < transmissions[i].joints_.size(); j++) {
          if (transmissions[i].joints_[j].name_ ==
              tree_joint_names[z])  // when act names or passive names
          {
            for (unsigned int k = 0; k < transmissions[i].joints_[j].dependent_joints_.size();
                 k++) {
              if (transmissions[i].joints_[j].dependent_joints_[k].name ==
                  independent_joint_names[y]) {
                G(z, y) = transmissions[i].joints_[j].dependent_joints_[k].multiplier;
                offset(z) = transmissions[i].joints_[j].dependent_joints_[k].offset;
              }
            }
          }
        }
      }
    }
  }

  return true;
}

}  // namespace Addons

}  // namespace RigidBodyDynamics
