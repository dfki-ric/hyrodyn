#include <rbdl/rbdl.h>

#include "urdfreader.h"
#include "transmission_parser.h"

#include <assert.h>
#include <iostream>
#include <fstream>
#include <map>
#include <stack>
#include <stdexcept>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#if USE_BOOST
  #include <boost/shared_ptr.hpp>

  typedef boost::shared_ptr<urdf::Link> LinkPtr;
  typedef const boost::shared_ptr<const urdf::Link> ConstLinkPtr;
  typedef boost::shared_ptr<urdf::Joint> JointPtr;
  typedef boost::shared_ptr<urdf::ModelInterface> ModelPtr;
#else
  #include <memory>

  typedef std::shared_ptr<urdf::Link> LinkPtr;
  typedef const std::shared_ptr<const urdf::Link> ConstLinkPtr;
  typedef std::shared_ptr<urdf::Joint> JointPtr;
  typedef std::shared_ptr<urdf::ModelInterface> ModelPtr;
#endif


using namespace std;

using namespace transmission_interface;

namespace RigidBodyDynamics {

namespace Addons {

using namespace Math;

typedef vector<LinkPtr> URDFLinkVector;
typedef vector<JointPtr> URDFJointVector;
typedef map<string, LinkPtr > URDFLinkMap;
typedef map<string, JointPtr > URDFJointMap;
// Transmission LCF
bool URDFReadLoopClosureFunctionTransmission (const char* filename, MatrixN_t &G, VectorN_t &offset, std::vector<std::string> &independent_joint_names, std::vector<std::string> &tree_joint_names)
{
	ifstream model_file (filename);
	if (!model_file) {
		cerr << "Error opening file '" << filename << "'." << endl;
		abort();
	}

	// reserve memory for the contents of the file
	string model_xml_string;
	model_file.seekg(0, std::ios::end);
	model_xml_string.reserve(model_file.tellg());
	model_file.seekg(0, std::ios::beg);
	model_xml_string.assign((std::istreambuf_iterator<char>(model_file)), std::istreambuf_iterator<char>());
	model_file.close();
	
	std::vector<TransmissionInfo> transmissions;
	TransmissionParser parser_obj;
	parser_obj.parse(model_xml_string, transmissions);
	cout<<"No. of transmissions: "<<transmissions.size()<<endl;
	for(unsigned int i = 0; i<transmissions.size();i++)
	{
		cout<<"Name: "<<transmissions[i].name_<<endl;
		cout<<"Type: "<<transmissions[i].type_<<endl;
		cout<<"No. of joints: "<<transmissions[i].joints_.size()<<endl;
		for(unsigned int j = 0; j<transmissions[i].joints_.size();j++)
		{
		cout<<"Name: "<<transmissions[i].joints_[j].name_<<endl;
		cout<<"No. of joints on which it depends: "<<transmissions[i].joints_[j].dependent_joints_.size()<<endl;
		for(unsigned int k = 0; k<transmissions[i].joints_[j].dependent_joints_.size();k++)
		{
		cout<<"Depending Joint Name: "<<transmissions[i].joints_[j].dependent_joints_[k].name<<
		", multiplier: "<<transmissions[i].joints_[j].dependent_joints_[k].multiplier<<
		", offset: "<<transmissions[i].joints_[j].dependent_joints_[k].offset<<endl;
		}		
		}			
	}
	
	// Allocate sizes and set zeros
	G.setZero(tree_joint_names.size(),independent_joint_names.size());
	offset.setZero(tree_joint_names.size());	

	// Check if spanning tree joints size equals to actuated joints size
	if(tree_joint_names.size()==independent_joint_names.size()){
	cout<<"Number of tree joints equals to number of actuators. Calling this function has no meaning! Note: Spanning tree joints should always contain the active joints."<<endl;
	return false;
	}

	// Build matrix G and vector offset
	for (uint y = 0; y < independent_joint_names.size(); y++) {
		for (uint z = 0; z < tree_joint_names.size(); z++) {
			if(independent_joint_names[y]==tree_joint_names[z]){
			G(z,y) = 1;
			}
			for(unsigned int i = 0; i<transmissions.size();i++)
			{
				for(unsigned int j = 0; j<transmissions[i].joints_.size();j++)
				{
				if(transmissions[i].joints_[j].name_ == tree_joint_names[z])	// when act names or passive names 
				{
					for(unsigned int k = 0; k<transmissions[i].joints_[j].dependent_joints_.size();k++)
					{
						if(transmissions[i].joints_[j].dependent_joints_[k].name == independent_joint_names[y]){
							G(z,y) = transmissions[i].joints_[j].dependent_joints_[k].multiplier;
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
//
bool construct_model (Model* rbdl_model, ModelPtr urdf_model, bool floating_base, bool verbose) {
  LinkPtr urdf_root_link;

  URDFLinkMap link_map;
  link_map = urdf_model->links_;

  URDFJointMap joint_map;
  joint_map = urdf_model->joints_;

  vector<string> joint_names;

  // Holds the links that we are processing in our depth first traversal with the top element being the current link.
  stack<LinkPtr > link_stack;
  // Holds the child joint index of the current link
  stack<int> joint_index_stack;

  // add the bodies in a depth-first order of the model tree
  link_stack.push (link_map[(urdf_model->getRoot()->name)]);

  // add the root body
  ConstLinkPtr& root = urdf_model->getRoot ();
  Vector3d root_inertial_rpy = Vector3d::Zero();
  Vector3d root_inertial_position = Vector3d::Zero();
  Matrix3d root_inertial_inertia = Matrix3d::Zero();
  double root_inertial_mass;

  if (root->inertial) {
    root_inertial_mass = root->inertial->mass;

    root_inertial_position.set (
        root->inertial->origin.position.x,
        root->inertial->origin.position.y,
        root->inertial->origin.position.z);

    root_inertial_inertia(0,0) = root->inertial->ixx;
    root_inertial_inertia(0,1) = root->inertial->ixy;
    root_inertial_inertia(0,2) = root->inertial->ixz;

    root_inertial_inertia(1,0) = root->inertial->ixy;
    root_inertial_inertia(1,1) = root->inertial->iyy;
    root_inertial_inertia(1,2) = root->inertial->iyz;

    root_inertial_inertia(2,0) = root->inertial->ixz;
    root_inertial_inertia(2,1) = root->inertial->iyz;
    root_inertial_inertia(2,2) = root->inertial->izz;

    root->inertial->origin.rotation.getRPY (root_inertial_rpy[0], root_inertial_rpy[1], root_inertial_rpy[2]);

    Body root_link = Body (root_inertial_mass,
        root_inertial_position,
        root_inertial_inertia);

    Joint root_joint (JointTypeFixed);
    if (floating_base) {
      root_joint = JointTypeFloatingBase;
    }

    SpatialTransform root_joint_frame = SpatialTransform ();

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

    rbdl_model->AppendBody(root_joint_frame,
        root_joint,
        root_link,
        root->name);
  }

  // depth first traversal: push the first child onto our joint_index_stack
  joint_index_stack.push(0);

  while (link_stack.size() > 0) {
    LinkPtr cur_link = link_stack.top();

    unsigned int joint_idx = joint_index_stack.top();

    // Add any child bodies and increment current joint index if we still have child joints to process.
    if (joint_idx < cur_link->child_joints.size()) {
      JointPtr cur_joint = cur_link->child_joints[joint_idx];

      // increment joint index
      joint_index_stack.pop();
      joint_index_stack.push (joint_idx + 1);

      link_stack.push (link_map[cur_joint->child_link_name]);
      joint_index_stack.push(0);

      if (verbose) {
        for (unsigned int i = 1; i < joint_index_stack.size() - 1; i++) {
          cout << "  ";
        }
        cout << "joint '" << cur_joint->name << "' child link '" << link_stack.top()->name << "' type = " << cur_joint->type << endl;
      }

      joint_names.push_back(cur_joint->name);
    } else {
      link_stack.pop();
      joint_index_stack.pop();
    }
  }

  unsigned int j;
  for (j = 0; j < joint_names.size(); j++) {
    JointPtr urdf_joint = joint_map[joint_names[j]];
    LinkPtr urdf_parent = link_map[urdf_joint->parent_link_name];
    LinkPtr urdf_child = link_map[urdf_joint->child_link_name];

    // determine where to add the current joint and child body
    unsigned int rbdl_parent_id = 0;
    rbdl_parent_id = rbdl_model->GetBodyId (urdf_parent->name.c_str());

    if (rbdl_parent_id == std::numeric_limits<unsigned int>::max()){
      cerr << "Error while processing joint '" << urdf_joint->name
        << "': parent link '" << urdf_parent->name
        << "' could not be found." << endl;      
      abort();
    }

    //cout << "joint: " << urdf_joint->name << "\tparent = " << urdf_parent->name << " child = " << urdf_child->name << " parent_id = " << rbdl_parent_id << endl;

    // create the joint
    Joint rbdl_joint;
    if (urdf_joint->type == urdf::Joint::REVOLUTE || urdf_joint->type == urdf::Joint::CONTINUOUS) {
      rbdl_joint = Joint (SpatialVector (urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z, 0., 0., 0.));
    } else if (urdf_joint->type == urdf::Joint::PRISMATIC) {
      rbdl_joint = Joint (SpatialVector (0., 0., 0., urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z));
    } else if (urdf_joint->type == urdf::Joint::FIXED) {
      rbdl_joint = Joint (JointTypeFixed);
    } else if (urdf_joint->type == urdf::Joint::FLOATING) {
      // todo: what order of DoF should be used?
      rbdl_joint = Joint (
          SpatialVector (0., 0., 0., 1., 0., 0.),
          SpatialVector (0., 0., 0., 0., 1., 0.),
          SpatialVector (0., 0., 0., 0., 0., 1.),
          SpatialVector (1., 0., 0., 0., 0., 0.),
          SpatialVector (0., 1., 0., 0., 0., 0.),
          SpatialVector (0., 0., 1., 0., 0., 0.));
    } else if (urdf_joint->type == urdf::Joint::PLANAR) {
      // todo: which two directions should be used that are perpendicular
      // to the specified axis?
      cerr << "Error while processing joint '" << urdf_joint->name << "': planar joints not yet supported!" << endl;
      return false;
    }

    // compute the joint transformation
    Vector3d joint_rpy;
    Vector3d joint_translation;
    urdf_joint->parent_to_joint_origin_transform.rotation.getRPY (joint_rpy[0], joint_rpy[1], joint_rpy[2]);
    joint_translation.set (
        urdf_joint->parent_to_joint_origin_transform.position.x,
        urdf_joint->parent_to_joint_origin_transform.position.y,
        urdf_joint->parent_to_joint_origin_transform.position.z
        );
    SpatialTransform rbdl_joint_frame =
          Xrot (joint_rpy[0], Vector3d (1., 0., 0.))
        * Xrot (joint_rpy[1], Vector3d (0., 1., 0.))
        * Xrot (joint_rpy[2], Vector3d (0., 0., 1.))
        * Xtrans (Vector3d (
              joint_translation
              ));

    // assemble the body
    Vector3d link_inertial_position = Vector3d::Zero();
    Vector3d link_inertial_rpy = Vector3d::Zero();
    Matrix3d link_inertial_inertia = Matrix3d::Zero();
    double link_inertial_mass = 0.;

    // but only if we actually have inertial data
    if (urdf_child->inertial) {
      link_inertial_mass = urdf_child->inertial->mass;

      link_inertial_position.set (
          urdf_child->inertial->origin.position.x,
          urdf_child->inertial->origin.position.y,
          urdf_child->inertial->origin.position.z
          );
      urdf_child->inertial->origin.rotation.getRPY (link_inertial_rpy[0], link_inertial_rpy[1], link_inertial_rpy[2]);

      link_inertial_inertia(0,0) = urdf_child->inertial->ixx;
      link_inertial_inertia(0,1) = urdf_child->inertial->ixy;
      link_inertial_inertia(0,2) = urdf_child->inertial->ixz;

      link_inertial_inertia(1,0) = urdf_child->inertial->ixy;
      link_inertial_inertia(1,1) = urdf_child->inertial->iyy;
      link_inertial_inertia(1,2) = urdf_child->inertial->iyz;

      link_inertial_inertia(2,0) = urdf_child->inertial->ixz;
      link_inertial_inertia(2,1) = urdf_child->inertial->iyz;
      link_inertial_inertia(2,2) = urdf_child->inertial->izz;

      if (link_inertial_rpy != Vector3d (0., 0., 0.)) {
        cerr << "Error while processing body '" << urdf_child->name << "': rotation of body frames not yet supported. Please rotate the joint frame instead." << endl;
        return false;
      }
    }

    Body rbdl_body = Body (link_inertial_mass, link_inertial_position, link_inertial_inertia);

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
      Body null_body (0., Vector3d::Zero(3), zero_matrix);
      Joint joint_txtytz(JointTypeTranslationXYZ);
      string trans_body_name = urdf_child->name + "_Translate";
      rbdl_model->AddBody (rbdl_parent_id, rbdl_joint_frame, joint_txtytz, null_body, trans_body_name);

      Joint joint_euler_zyx (JointTypeEulerXYZ);
      rbdl_model->AppendBody (SpatialTransform(), joint_euler_zyx, rbdl_body, urdf_child->name);
    } else {
      rbdl_model->AddBody (rbdl_parent_id, rbdl_joint_frame, rbdl_joint, rbdl_body, urdf_child->name);
    }
  }

  return true;
}

RBDL_DLLAPI bool URDFReadFromFile (const char* filename, Model* model, bool floating_base, bool verbose) {
  ifstream model_file (filename);
  if (!model_file) {
    cerr << "Error opening file '" << filename << "'." << endl;
    abort();
  }

  // reserve memory for the contents of the file
  string model_xml_string;
  model_file.seekg(0, std::ios::end);
  model_xml_string.reserve(model_file.tellg());
  model_file.seekg(0, std::ios::beg);
  model_xml_string.assign((std::istreambuf_iterator<char>(model_file)), std::istreambuf_iterator<char>());

  model_file.close();

  return URDFReadFromString (model_xml_string.c_str(), model, floating_base, verbose);
}

RBDL_DLLAPI bool URDFReadFromString (const char* model_xml_string, Model* model, bool floating_base, bool verbose) {
  assert (model);

  ModelPtr urdf_model = urdf::parseURDF (model_xml_string);

  if (!construct_model (model, urdf_model, floating_base, verbose)) {
    cerr << "Error constructing model from urdf file." << endl;
    return false;
  }

  model->gravity.set (0., 0., -9.81);

  return true;
}

RBDL_DLLAPI bool URDFReadLoopClosureFunction (const char* filename, MatrixN_t &G, VectorN_t &offset, MatrixN_t &Gu, std::vector<std::string> &actuated_joint_names, std::vector<std::string> &tree_joint_names) {

	ifstream model_file (filename);
	if (!model_file) {
		cerr << "Error opening file '" << filename << "'." << endl;
		abort();
	}

	// reserve memory for the contents of the file
	string model_xml_string;
	model_file.seekg(0, std::ios::end);
	model_xml_string.reserve(model_file.tellg());
	model_file.seekg(0, std::ios::beg);
	model_xml_string.assign((std::istreambuf_iterator<char>(model_file)), std::istreambuf_iterator<char>());
	model_file.close();

	// Parse URDF
	ModelPtr urdf_model = urdf::parseURDF (model_xml_string);

	LinkPtr urdf_root_link;

	URDFLinkMap link_map;
	link_map = urdf_model->links_;

	URDFJointMap joint_map;
	joint_map = urdf_model->joints_;

	vector<string> joint_names;

	stack<LinkPtr > link_stack;
	stack<int> joint_index_stack;

	// add the bodies in a depth-first order of the model tree
	link_stack.push (link_map[(urdf_model->getRoot()->name)]);

	// add the root body
	ConstLinkPtr& root = urdf_model->getRoot ();

	// depth first traversal: push the first child onto our joint_index_stack
	joint_index_stack.push(0);


	while (link_stack.size() > 0) {
		LinkPtr cur_link = link_stack.top();
		unsigned int joint_idx = joint_index_stack.top();

		if (joint_idx < cur_link->child_joints.size()) {
			JointPtr cur_joint = cur_link->child_joints[joint_idx];

			// increment joint index
			joint_index_stack.pop();
			joint_index_stack.push (joint_idx + 1);

			link_stack.push (link_map[cur_joint->child_link_name]);
			joint_index_stack.push(0);

			joint_names.push_back(cur_joint->name);
		} else {
			link_stack.pop();
			joint_index_stack.pop();
		}
	}

	// Collect all tree joint names and actuated joint names
	//vector<string> tree_joint_names;
	//vector<string> actuated_joint_names;
	unsigned int y,z;

	for (z = 0; z < joint_names.size(); z++) {
		JointPtr urdf_joint = joint_map[joint_names[z]];
		if(urdf_joint->type == urdf::Joint::FLOATING){
		cout<<"Floating base systems are not supported yet."<<endl;
		return false;
		}
		if(urdf_joint->type == urdf::Joint::PLANAR){
		cout<<"Planar joints are not yet supported in RBDL URDF parsing."<<endl;
		return false;
		}
		if(!(urdf_joint->type == urdf::Joint::FIXED)){
			tree_joint_names.push_back(joint_names[z]);
			if(!(urdf_joint->mimic)){
				actuated_joint_names.push_back(joint_names[z]);
			}
		}
	}

	// Allocate size and set zeros
	G.setZero(tree_joint_names.size(),actuated_joint_names.size());
	Gu.setZero(actuated_joint_names.size(),actuated_joint_names.size());
	offset.setZero(tree_joint_names.size());

	// Check if there are mimic joints in the spanning tree
	if(tree_joint_names.size()==actuated_joint_names.size()){
	cout<<"No mimic joints found. Calling this function has no meaning!"<<endl;
	return false;
	}

	// Build matrix G and vector offset
	for (y = 0; y < actuated_joint_names.size(); y++) {
		for (z = 0; z < tree_joint_names.size(); z++) {
			if(actuated_joint_names[y]==tree_joint_names[z]){
			G(z,y) = 1;
			}
			JointPtr urdf_tree_joint = joint_map[tree_joint_names[z]];
			if(urdf_tree_joint->mimic){
				if(urdf_tree_joint->mimic->joint_name == actuated_joint_names[y])
					G(z,y) = urdf_tree_joint->mimic->multiplier;
					offset(z) = urdf_tree_joint->mimic->offset;
			}
		}
	}

	// Build matrix Gu
	for (y = 0; y < actuated_joint_names.size(); y++) {
		for (z = 0; z < tree_joint_names.size(); z++) {
			if(actuated_joint_names[y]==tree_joint_names[z]){
			Gu.row(y) = G.row(z);
			}
		}
	}

	return true;
}

RBDL_DLLAPI bool URDFReadLoopClosureFunctionExterior (const char* filename, MatrixN_t &G, VectorN_t &offset, std::vector<std::string> actuated_joint_names, std::vector<std::string> tree_joint_names) {

	ifstream model_file (filename);
	if (!model_file) {
		cerr << "Error opening file '" << filename << "'." << endl;
		abort();
	}

	// reserve memory for the contents of the file
	string model_xml_string;
	model_file.seekg(0, std::ios::end);
	model_xml_string.reserve(model_file.tellg());
	model_file.seekg(0, std::ios::beg);
	model_xml_string.assign((std::istreambuf_iterator<char>(model_file)), std::istreambuf_iterator<char>());
	model_file.close();

	// Parse URDF
	ModelPtr urdf_model = urdf::parseURDF (model_xml_string);

	LinkPtr urdf_root_link;

	URDFLinkMap link_map;
	link_map = urdf_model->links_;

	URDFJointMap joint_map;
	joint_map = urdf_model->joints_;

	vector<string> joint_names;

	stack<LinkPtr > link_stack;
	stack<int> joint_index_stack;

	// add the bodies in a depth-first order of the model tree
	link_stack.push (link_map[(urdf_model->getRoot()->name)]);

	// add the root body
	ConstLinkPtr& root = urdf_model->getRoot ();

	// depth first traversal: push the first child onto our joint_index_stack
	joint_index_stack.push(0);


	while (link_stack.size() > 0) {
		LinkPtr cur_link = link_stack.top();
		unsigned int joint_idx = joint_index_stack.top();

		if (joint_idx < cur_link->child_joints.size()) {
			JointPtr cur_joint = cur_link->child_joints[joint_idx];

			// increment joint index
			joint_index_stack.pop();
			joint_index_stack.push (joint_idx + 1);

			link_stack.push (link_map[cur_joint->child_link_name]);
			joint_index_stack.push(0);

			joint_names.push_back(cur_joint->name);
		} else {
			link_stack.pop();
			joint_index_stack.pop();
		}
	}

	// Collect all tree joint names and actuated joint names
	//vector<string> tree_joint_names;
	//vector<string> actuated_joint_names;
	unsigned int y,z;

	// Allocate size and set zeros
	G.setZero(tree_joint_names.size(),actuated_joint_names.size());
	offset.setZero(tree_joint_names.size());

	// Build matrix G and vector offset
	for (y = 0; y < actuated_joint_names.size(); y++) {
		for (z = 0; z < tree_joint_names.size(); z++) {
			JointPtr urdf_tree_joint = joint_map[tree_joint_names[z]];
			if(urdf_tree_joint->mimic){
				if(urdf_tree_joint->mimic->joint_name == actuated_joint_names[y])
					G(z,y) = urdf_tree_joint->mimic->multiplier;
					offset(z) = urdf_tree_joint->mimic->offset;
			}
		}
	}

	return true;
}

RBDL_DLLAPI bool URDFReadJointLimits (const char* filename, std::vector<std::string> joint_names_respecting_modularity, VectorN_t &q_max, VectorN_t &q_min, VectorN_t &vel_limit, VectorN_t &effort_limit) {

	ifstream model_file (filename);
	if (!model_file) {
		cerr << "Error opening file '" << filename << "'." << endl;
		abort();
	}

	// reserve memory for the contents of the file
	string model_xml_string;
	model_file.seekg(0, std::ios::end);
	model_xml_string.reserve(model_file.tellg());
	model_file.seekg(0, std::ios::beg);
	model_xml_string.assign((std::istreambuf_iterator<char>(model_file)), std::istreambuf_iterator<char>());
	model_file.close();

	// Parse URDF
	ModelPtr urdf_model = urdf::parseURDF (model_xml_string);

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
		if(urdf_joint->type == urdf::Joint::FLOATING){
		cout<<"Floating base systems are not supported yet."<<endl;
		return false;
		}
		if(urdf_joint->type == urdf::Joint::PLANAR){
		cout<<"Planar joints are not yet supported in RBDL URDF parsing."<<endl;
		return false;
		}
		if(!(urdf_joint->type == urdf::Joint::FIXED)){
		q_min(y) = urdf_joint->limits->lower;
		q_max(y) = urdf_joint->limits->upper;
		vel_limit(y) = urdf_joint->limits->velocity;
		effort_limit(y) = urdf_joint->limits->effort;
		y++;
		}
	}

	/* // Working approach
	ifstream model_file (filename);
	if (!model_file) {
		cerr << "Error opening file '" << filename << "'." << endl;
		abort();
	}

	// reserve memory for the contents of the file
	string model_xml_string;
	model_file.seekg(0, std::ios::end);
	model_xml_string.reserve(model_file.tellg());
	model_file.seekg(0, std::ios::beg);
	model_xml_string.assign((std::istreambuf_iterator<char>(model_file)), std::istreambuf_iterator<char>());
	model_file.close();

	// Parse URDF
	ModelPtr urdf_model = urdf::parseURDF (model_xml_string);

	LinkPtr urdf_root_link;

	URDFLinkMap link_map;
	link_map = urdf_model->links_;

	URDFJointMap joint_map;
	joint_map = urdf_model->joints_;

	vector<string> joint_names;

	stack<LinkPtr > link_stack;
	stack<int> joint_index_stack;

	// add the bodies in a depth-first order of the model tree
	link_stack.push (link_map[(urdf_model->getRoot()->name)]);

	// add the root body
	ConstLinkPtr& root = urdf_model->getRoot ();

	// depth first traversal: push the first child onto our joint_index_stack
	joint_index_stack.push(0);

	while (link_stack.size() > 0) {
		LinkPtr cur_link = link_stack.top();
		unsigned int joint_idx = joint_index_stack.top();

		if (joint_idx < cur_link->child_joints.size()) {
			JointPtr cur_joint = cur_link->child_joints[joint_idx];

			// increment joint index
			joint_index_stack.pop();
			joint_index_stack.push (joint_idx + 1);

			link_stack.push (link_map[cur_joint->child_link_name]);
			joint_index_stack.push(0);

			joint_names.push_back(cur_joint->name);
		} else {
			link_stack.pop();
			joint_index_stack.pop();
		}
	}

	unsigned int y = 0;
	std::vector<string> tree_joint_names;

	for (unsigned int z = 0; z < joint_names.size(); z++) {
		JointPtr urdf_joint = joint_map[joint_names[z]];
		if(urdf_joint->type == urdf::Joint::FLOATING){
		cout<<"Floating base systems are not supported yet."<<endl;
		return false;
		}
		if(urdf_joint->type == urdf::Joint::PLANAR){
		cout<<"Planar joints are not yet supported in RBDL URDF parsing."<<endl;
		return false;
		}
		if(!(urdf_joint->type == urdf::Joint::FIXED)){
			tree_joint_names.push_back(joint_names[z]);
		}
	}

	// Allocate size and set zeros
	q_max.setZero(tree_joint_names.size());
	q_min.setZero(tree_joint_names.size());
	vel_limit.setZero(tree_joint_names.size());
	effort_limit.setZero(tree_joint_names.size());

	for (unsigned int y = 0; y < tree_joint_names.size(); y++) {
		JointPtr urdf_tree_joint = joint_map[tree_joint_names[y]];
		q_min(y) = urdf_tree_joint->limits->lower;
		q_max(y) = urdf_tree_joint->limits->upper;
		vel_limit(y) = urdf_tree_joint->limits->velocity;
		effort_limit(y) = urdf_tree_joint->limits->effort;
	}
	*/
	return true;
}



bool construct_model_with_modularity (Model* rbdl_model, ModelPtr urdf_model, std::vector<std::string> joint_names_respecting_modularity, bool floating_base, bool verbose) {

  LinkPtr urdf_root_link;

  URDFLinkMap link_map;
  link_map = urdf_model->links_;

  URDFJointMap joint_map;
  joint_map = urdf_model->joints_;

  vector<string> joint_names;

  // Holds the links that we are processing in our depth first traversal with the top element being the current link.
  stack<LinkPtr > link_stack;
  // Holds the child joint index of the current link
  stack<int> joint_index_stack;

  // add the bodies in a depth-first order of the model tree
  link_stack.push (link_map[(urdf_model->getRoot()->name)]);

  // add the root body
  ConstLinkPtr& root = urdf_model->getRoot ();
  Vector3d root_inertial_rpy = Vector3d::Zero();
  Vector3d root_inertial_position = Vector3d::Zero();
  Matrix3d root_inertial_inertia = Matrix3d::Zero();
  double root_inertial_mass;
  if (root->inertial) {
    root_inertial_mass = root->inertial->mass;

    root_inertial_position.set (
        root->inertial->origin.position.x,
        root->inertial->origin.position.y,
        root->inertial->origin.position.z);

    root_inertial_inertia(0,0) = root->inertial->ixx;
    root_inertial_inertia(0,1) = root->inertial->ixy;
    root_inertial_inertia(0,2) = root->inertial->ixz;

    root_inertial_inertia(1,0) = root->inertial->ixy;
    root_inertial_inertia(1,1) = root->inertial->iyy;
    root_inertial_inertia(1,2) = root->inertial->iyz;

    root_inertial_inertia(2,0) = root->inertial->ixz;
    root_inertial_inertia(2,1) = root->inertial->iyz;
    root_inertial_inertia(2,2) = root->inertial->izz;

    root->inertial->origin.rotation.getRPY (root_inertial_rpy[0], root_inertial_rpy[1], root_inertial_rpy[2]);

    Body root_link = Body (root_inertial_mass,
        root_inertial_position,
        root_inertial_inertia);

    Joint root_joint (JointTypeFixed);
    if (floating_base) {
      root_joint = JointTypeFloatingBase;
    }

    SpatialTransform root_joint_frame = SpatialTransform ();

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

    rbdl_model->AppendBody(root_joint_frame,
        root_joint,
        root_link,
        root->name);
  }

  // depth first traversal: push the first child onto our joint_index_stack
  joint_index_stack.push(0);

  while (link_stack.size() > 0) {
    LinkPtr cur_link = link_stack.top();

    unsigned int joint_idx = joint_index_stack.top();

    // Add any child bodies and increment current joint index if we still have child joints to process.
    if (joint_idx < cur_link->child_joints.size()) {
      JointPtr cur_joint = cur_link->child_joints[joint_idx];

      // increment joint index
      joint_index_stack.pop();
      joint_index_stack.push (joint_idx + 1);

      link_stack.push (link_map[cur_joint->child_link_name]);
      joint_index_stack.push(0);

      if (verbose) {
        for (unsigned int i = 1; i < joint_index_stack.size() - 1; i++) {
          cout << "  ";
        }
        cout << "joint '" << cur_joint->name << "' child link '" << link_stack.top()->name << "' type = " << cur_joint->type << endl;
      }

      joint_names.push_back(cur_joint->name);
    } else {
      link_stack.pop();
      joint_index_stack.pop();
    }
  }

// HyRoDyn stuff starts here
	cout<<"Joint names from spanning tree in URDF: "<<endl;
	for (unsigned int j = 0; j < joint_names.size(); j++)
		cout<<"j: "<<j<<" "<<joint_names[j]<<endl;

	// if there is a fixed joint found, insert it into the joint names respecting modularity vector so that user
	// does not have to define in submechanisms.yml file

	/*
	// idea: find the indices between which fixed joint is contained and insert into joint_names_respecting_modularity accordingly.
	std::vector<string> fixed_jointnames;
	for (unsigned int j = 0; j < joint_names.size(); j++) {
		JointPtr urdf_joint = joint_map[joint_names[j]];
		if (urdf_joint->type == urdf::Joint::FIXED) {
			fixed_jointnames.push_back(joint_names[j]);
			cout<<"fixed joint found: "<<joint_names[j]<<endl;
			if (std::find(joint_names_respecting_modularity.begin(), joint_names_respecting_modularity.end(), joint_names[j+1]) != joint_names_respecting_modularity.end())
			{
			  auto it = std::find(joint_names_respecting_modularity.begin(), joint_names_respecting_modularity.end(), joint_names[j+1]);
			  auto index = std::distance(joint_names_respecting_modularity.begin(), it);
			  cout<<"successor joint with name "<<joint_names[j+1]<<" to fixed joint found at "<<index<<endl;
			  joint_names_respecting_modularity.insert(joint_names_respecting_modularity.begin()+index, joint_names[j]);
			}
			else
			joint_names_respecting_modularity.insert(joint_names_respecting_modularity.begin()+j, joint_names[j]);	// for fixed transformation to ee link

			//if (std::find(joint_names_respecting_modularity.begin(), joint_names_respecting_modularity.end(), joint_names[j-1]) != joint_names_respecting_modularity.end())
			//{
			  //auto it = std::find(joint_names_respecting_modularity.begin(), joint_names_respecting_modularity.end(), joint_names[j-1]);
			  //auto index = std::distance(joint_names_respecting_modularity.begin(), it);
			  //cout<<"predecessor joint with name "<<joint_names[j-1]<<" to fixed joint found at "<<index<<endl;
			  ////joint_names_respecting_modularity.insert(joint_names_respecting_modularity.begin()+index, joint_names[j]);
			//}
		}
	}
	*/

	// Fixed joint processing is used when joint_names_respecting_modularity vector (joint names provided by the submechanism definition) 
	// size is different from joint_names vector (joint names extracted from the URDF) size.
	if (joint_names_respecting_modularity.size()!=joint_names.size()){
		cout<<"joint_names_respecting_modularity vector (joint names provided by the submechanism definition) size = " << joint_names_respecting_modularity.size() 
		<< " is different from joint_names vector (joint names extracted from the URDF) size = "<< joint_names.size() <<". Hence, automatic fixed joint processing of HyRoDyn will be used. Handle with care. May not always work!"<<endl;
		unsigned int jn = 0;

		// Case 1: fixed joint group (minimum size 1) attached to root link
		JointPtr urdf_joint = joint_map[joint_names[jn]];
		if (urdf_joint->type == urdf::Joint::FIXED) {
		std::vector<string> fixed_jointnames_root;
		cout<<"root fixed joint found: "<<joint_names[jn]<<endl;
		fixed_jointnames_root.push_back(joint_names[jn]);
		for (unsigned int kn = 1; kn < joint_names.size(); kn++){
		JointPtr urdf_joint = joint_map[joint_names[kn]];
		if (urdf_joint->type == urdf::Joint::FIXED) {
			cout<<"root fixed joint found in group: "<<joint_names[kn]<<endl;
			fixed_jointnames_root.push_back(joint_names[kn]);
		}
		else
		break;
		}
		// insert the fixed joints at root in the modular joint names vector
		joint_names_respecting_modularity.insert(joint_names_respecting_modularity.begin(), fixed_jointnames_root.begin(), fixed_jointnames_root.end());
		jn = jn + fixed_jointnames_root.size();
		}
		//

		// Case 2: fixed joint group (minimum size 1) found in between
		while(jn<joint_names.size()){
			here:
			cout<<"jn = "<<jn<<endl;
			JointPtr urdf_joint = joint_map[joint_names[jn]];
			if (urdf_joint->type == urdf::Joint::FIXED) {
				std::vector<string> fixed_jointnames_inbetween;
				cout<<"in-between fixed joint found: "<<joint_names[jn]<<endl;
				fixed_jointnames_inbetween.push_back(joint_names[jn]);
				// Add further fixed joints into the block
				for (unsigned int kn = jn+1; kn < joint_names.size(); kn++){
				JointPtr urdf_joint = joint_map[joint_names[kn]];
				if (urdf_joint->type == urdf::Joint::FIXED) {
					cout<<"in-between fixed joint found in group: "<<joint_names[kn]<<endl;
					fixed_jointnames_inbetween.push_back(joint_names[kn]);
				}
				else
				break;
				}

				// idea: find predecessor joint and insert the fixed joints in-between vector in the modular joint names vector
				// string predecessor_joint_name = joint_names[jn-1];	// trivial thought, may not always work
				string predecessor_link_name = urdf_joint->parent_link_name;
				LinkPtr predecessor_link = link_map[predecessor_link_name];
				string predecessor_joint_name;
				if(predecessor_link_name!=urdf_model->getRoot()->name){
					cout<<"Predecessor link is not root link"<<endl;
					predecessor_joint_name = predecessor_link->parent_joint->name;
				}
				else{
					cout<<"Predecessor link is a root link!"<<endl;
					joint_names_respecting_modularity.insert(joint_names_respecting_modularity.begin(), fixed_jointnames_inbetween.begin(), fixed_jointnames_inbetween.end());
					jn = jn + fixed_jointnames_inbetween.size();
					goto here;
				}
				bool successor_joint_flag = false;
				string successor_joint_name;

				if((jn + fixed_jointnames_inbetween.size()) < joint_names.size()){
					successor_joint_name = joint_names[ jn + fixed_jointnames_inbetween.size()];
					cout<<"predecessor joint name "<<predecessor_joint_name<<", successor joint name "<<successor_joint_name<<endl;
					successor_joint_flag = true;
				}else
					cout<<"predecessor joint name "<<predecessor_joint_name<<", successor joint does not exist "<<endl;
				/*
				cout<<"Joint names respecting modularity with fixed joints(not final): "<<endl;
				for (unsigned int j = 0; j < joint_names_respecting_modularity.size(); j++)
					cout<<"j: "<<j<<" "<<joint_names_respecting_modularity[j]<<endl;
				*/							
				unsigned int index_predecessor=0, index_successor=0;
					
				if (std::find(joint_names_respecting_modularity.begin(), joint_names_respecting_modularity.end(), predecessor_joint_name) != joint_names_respecting_modularity.end())
				{
				  auto it_p = std::find(joint_names_respecting_modularity.begin(), joint_names_respecting_modularity.end(), predecessor_joint_name);
				  auto index_p = std::distance(joint_names_respecting_modularity.begin(), it_p);
				  cout<<"predecessor joint with name "<<predecessor_joint_name<<" to the fixed joint group found at "<<index_p<<endl;
				  index_predecessor = index_p;
				}

				if(std::find(joint_names_respecting_modularity.begin(), joint_names_respecting_modularity.end(), successor_joint_name) != joint_names_respecting_modularity.end())
				{
				  auto it_s = std::find(joint_names_respecting_modularity.begin(), joint_names_respecting_modularity.end(), successor_joint_name);
				  auto index_s = std::distance(joint_names_respecting_modularity.begin(), it_s);
				  cout<<"successor joint with name "<<successor_joint_name<<" to the fixed joint group found at "<<index_s<<endl;
				  index_successor = index_s;
				}
	/*
				cout<<"jn="<<jn<<"index_predecessor"<<index_predecessor<<"joint names respecting modularity size"<<joint_names_respecting_modularity.size()<<endl;

				if(index_predecessor>jn){
					joint_names_respecting_modularity.insert(joint_names_respecting_modularity.begin()+index_predecessor+1, fixed_jointnames_inbetween.begin(), fixed_jointnames_inbetween.end());
					cout<<"Fixed joint successfully inserted after predecessor joint name"<<endl;
					temp.insert(temp.end(), fixed_jointnames_inbetween.begin(), fixed_jointnames_inbetween.end());
				} else{
				  joint_names_respecting_modularity.insert(joint_names_respecting_modularity.begin()+index_successor, fixed_jointnames_inbetween.begin(), fixed_jointnames_inbetween.end());
				  cout<<"Fixed joint successfully inserted after successor joint name"<<endl;
					temp.insert(temp.end(), fixed_jointnames_inbetween.begin(), fixed_jointnames_inbetween.end());
				}
				*/								
				if(index_predecessor<index_successor){
				joint_names_respecting_modularity.insert(joint_names_respecting_modularity.begin()+index_successor, fixed_jointnames_inbetween.begin(), fixed_jointnames_inbetween.end());
				  cout<<"Fixed joint successfully inserted after successor joint name"<<endl;
				}else{
				joint_names_respecting_modularity.insert(joint_names_respecting_modularity.begin()+index_predecessor+1, fixed_jointnames_inbetween.begin(), fixed_jointnames_inbetween.end());
				cout<<"Fixed joint successfully inserted after predecessor joint name"<<endl;
				}
				jn = jn + fixed_jointnames_inbetween.size();
			}
			else
			jn = jn + 1;
		}
	}					
	
//	std::sort(joint_names.begin(), joint_names.end());
	cout<<"Joint names respecting modularity with fixed joints: "<<endl;
	for (unsigned int j = 0; j < joint_names_respecting_modularity.size(); j++)
		cout<<"j: "<<j<<" "<<joint_names_respecting_modularity[j]<<endl;

	// sorted joint names variables for comparison(thats the sole purpose of sorting joint names)
	std::vector<string> sorted_jointnames, sorted_jointnames_spanningtree;

	sorted_jointnames = joint_names;
    std::sort(sorted_jointnames.begin(), sorted_jointnames.end());
    cout<<"Joint names from URDF after sorting for comparison"<<endl;
	for (unsigned int j = 0; j < sorted_jointnames.size(); j++)
		cout<<"j: "<<j<<" "<<sorted_jointnames[j]<<endl;
		
	sorted_jointnames_spanningtree = joint_names_respecting_modularity;
    std::sort(sorted_jointnames_spanningtree.begin(), sorted_jointnames_spanningtree.end());
    cout<<"Joint names from submechanisms file with fixed joints after sorting for comparison"<<endl;
	for (unsigned int j = 0; j < sorted_jointnames_spanningtree.size(); j++)
		cout<<"j: "<<j<<" "<<sorted_jointnames_spanningtree[j]<<endl;
		
	bool is_equal = false;
	if(sorted_jointnames.size() == sorted_jointnames_spanningtree.size()){
	  is_equal = std::equal(sorted_jointnames.begin(), sorted_jointnames.end(), sorted_jointnames_spanningtree.begin());
	  if(is_equal){
			cout<<"Joint names respecting modularity and joint names from URDF are consistent. RBDL model will now respect the modular numbering scheme."<<endl;
			joint_names = joint_names_respecting_modularity;
	  } else{
			cout << "Names mismatch: Joint names respecting modularity and joint names from URDF are inconsistent"<< endl;
			cout << "Sorted joint names parsed from URDF and regular numbering scheme for comparison:"<<endl;
			for (unsigned int j = 0; j < joint_names.size(); j++){
				cout << "index: " << j << " " << sorted_jointnames[j]<<", "<<sorted_jointnames_spanningtree[j]<<endl;
				if(sorted_jointnames[j]!=sorted_jointnames_spanningtree[j])
					cout<<"Mismatch here!"<<endl;
			}
			cerr << "Aborting..." << endl;
			abort();
	  }
	}
	else{
		cout<<"Size of joint names in URDF: "<<joint_names.size()<<endl;
		cout<<"Size of modular joint names provided: "<<joint_names_respecting_modularity.size()<<endl;
		cerr << "Size mismatch: Joint names respecting modularity and joint names from URDF are inconsistent"<< endl;
		abort();
	}

	cout<<"Joint names respecting modularity: "<<endl;
	for (unsigned int j = 0; j < joint_names.size(); j++)
		cout<<"j: "<<j<<" "<<joint_names[j]<<endl;

// HyRoDyn stuff ends here

  unsigned int j;
  for (j = 0; j < joint_names.size(); j++) {
    JointPtr urdf_joint = joint_map[joint_names[j]];
    LinkPtr urdf_parent = link_map[urdf_joint->parent_link_name];
    LinkPtr urdf_child = link_map[urdf_joint->child_link_name];

    // determine where to add the current joint and child body
    unsigned int rbdl_parent_id = 0;
    rbdl_parent_id = rbdl_model->GetBodyId (urdf_parent->name.c_str());

    if (rbdl_parent_id == std::numeric_limits<unsigned int>::max()){
	// TIP: if you enter this for the root link of the robot, make sure the root link had inertial properties defined (all zeros are also fine)
      cerr << "Error while processing joint '" << urdf_joint->name
        << "': parent link '" << urdf_parent->name
        << "' could not be found." << endl;
      abort();
    }

    //cout << "joint: " << urdf_joint->name << "\tparent = " << urdf_parent->name << " child = " << urdf_child->name << " parent_id = " << rbdl_parent_id << endl;

    // create the joint
    Joint rbdl_joint;
    if (urdf_joint->type == urdf::Joint::REVOLUTE || urdf_joint->type == urdf::Joint::CONTINUOUS) {
      SpatialVector axis(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z, 0., 0., 0.);
      if (fabs(axis.norm() - 1.0) > 1.0e-2) {
        throw std::runtime_error("Joint axis of joint" + joint_names[j] + " is not unit. Fix your URDF file.");
      } else if (fabs(axis.norm() - 1.0) > 1.0e-8) {
        std::cerr << "Warning: joint axis is not unit! It will be normalized automatically." << std::endl;
        axis /= axis.norm();
      }
      rbdl_joint = Joint (axis);
//      cout<<"urdf joint limits: "<<urdf_joint->limits->lower<<endl;
    } else if (urdf_joint->type == urdf::Joint::PRISMATIC) {
      SpatialVector axis(0., 0., 0., urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z);
      if (fabs(axis.norm() - 1.0) > 1.0e-2) {
        throw std::runtime_error("Joint axis of joint" + joint_names[j] + " is not unit. Fix your URDF file.");
      } else if (fabs(axis.norm() - 1.0) > 1.0e-8) {
        std::cerr << "Warning: joint axis is not unit! It will be normalized automatically." << std::endl;
        axis /= axis.norm();
      }
      rbdl_joint = Joint (axis);
    } else if (urdf_joint->type == urdf::Joint::FIXED) {
      rbdl_joint = Joint (JointTypeFixed);
    } else if (urdf_joint->type == urdf::Joint::FLOATING) {
      // todo: what order of DoF should be used?
      rbdl_joint = Joint (
          SpatialVector (0., 0., 0., 1., 0., 0.),
          SpatialVector (0., 0., 0., 0., 1., 0.),
          SpatialVector (0., 0., 0., 0., 0., 1.),
          SpatialVector (1., 0., 0., 0., 0., 0.),
          SpatialVector (0., 1., 0., 0., 0., 0.),
          SpatialVector (0., 0., 1., 0., 0., 0.));
    } else if (urdf_joint->type == urdf::Joint::PLANAR) {
      // todo: which two directions should be used that are perpendicular
      // to the specified axis?
      cerr << "Error while processing joint '" << urdf_joint->name << "': planar joints not yet supported!" << endl;
      return false;
    }

    // compute the joint transformation
    Vector3d joint_rpy;
    Vector3d joint_translation;
    urdf_joint->parent_to_joint_origin_transform.rotation.getRPY (joint_rpy[0], joint_rpy[1], joint_rpy[2]);
    joint_translation.set (
        urdf_joint->parent_to_joint_origin_transform.position.x,
        urdf_joint->parent_to_joint_origin_transform.position.y,
        urdf_joint->parent_to_joint_origin_transform.position.z
        );
    SpatialTransform rbdl_joint_frame =
          Xrot (joint_rpy[0], Vector3d (1., 0., 0.))
        * Xrot (joint_rpy[1], Vector3d (0., 1., 0.))
        * Xrot (joint_rpy[2], Vector3d (0., 0., 1.))
        * Xtrans (Vector3d (
              joint_translation
              ));

    // assemble the body
    Vector3d link_inertial_position = Vector3d::Zero();
    Vector3d link_inertial_rpy = Vector3d::Zero();
    Matrix3d link_inertial_inertia = Matrix3d::Zero();
    double link_inertial_mass = 0.;

    // but only if we actually have inertial data
    if (urdf_child->inertial) {
      link_inertial_mass = urdf_child->inertial->mass;

      link_inertial_position.set (
          urdf_child->inertial->origin.position.x,
          urdf_child->inertial->origin.position.y,
          urdf_child->inertial->origin.position.z
          );
      urdf_child->inertial->origin.rotation.getRPY (link_inertial_rpy[0], link_inertial_rpy[1], link_inertial_rpy[2]);

      link_inertial_inertia(0,0) = urdf_child->inertial->ixx;
      link_inertial_inertia(0,1) = urdf_child->inertial->ixy;
      link_inertial_inertia(0,2) = urdf_child->inertial->ixz;

      link_inertial_inertia(1,0) = urdf_child->inertial->ixy;
      link_inertial_inertia(1,1) = urdf_child->inertial->iyy;
      link_inertial_inertia(1,2) = urdf_child->inertial->iyz;

      link_inertial_inertia(2,0) = urdf_child->inertial->ixz;
      link_inertial_inertia(2,1) = urdf_child->inertial->iyz;
      link_inertial_inertia(2,2) = urdf_child->inertial->izz;

      if (link_inertial_rpy != Vector3d (0., 0., 0.)) {
        cerr << "Error while processing body '" << urdf_child->name << "': rotation of body frames not yet supported. Please rotate the joint frame instead." << endl;
        return false;
      }
    }

    Body rbdl_body = Body (link_inertial_mass, link_inertial_position, link_inertial_inertia);

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
      Body null_body (0., Vector3d::Zero(3), zero_matrix);
      Joint joint_txtytz(JointTypeTranslationXYZ);
      string trans_body_name = urdf_child->name + "_Translate";
      rbdl_model->AddBody (rbdl_parent_id, rbdl_joint_frame, joint_txtytz, null_body, trans_body_name);

      Joint joint_euler_zyx (JointTypeEulerXYZ);
      rbdl_model->AppendBody (SpatialTransform(), joint_euler_zyx, rbdl_body, urdf_child->name);
    } else {
      rbdl_model->AddBody (rbdl_parent_id, rbdl_joint_frame, rbdl_joint, rbdl_body, urdf_child->name);
    }
  }
  cout<< "URDF parsed successfully" <<endl;
  return true;
}

RBDL_DLLAPI bool URDFReadFromFileWithModularity (const char* filename, Model* model, std::vector<std::string> joint_names, bool floating_base, bool verbose) {
	ifstream model_file (filename);
	if (!model_file) {
		cerr << "Error opening file '" << filename << "'." << endl;
		abort();
	}

	// reserve memory for the contents of the file
	string model_xml_string;
	model_file.seekg(0, std::ios::end);
	model_xml_string.reserve(model_file.tellg());
	model_file.seekg(0, std::ios::beg);
	model_xml_string.assign((std::istreambuf_iterator<char>(model_file)), std::istreambuf_iterator<char>());

	model_file.close();

	return URDFReadFromStringWithModularity (model_xml_string.c_str(), model, joint_names, floating_base, verbose);
}

RBDL_DLLAPI bool URDFReadFromStringWithModularity (const char* model_xml_string, Model* model, std::vector<std::string> joint_names, bool floating_base, bool verbose) {
	assert (model);

	ModelPtr urdf_model = urdf::parseURDF (model_xml_string);

	if (!construct_model_with_modularity (model, urdf_model, joint_names, floating_base, verbose)) {
		cerr << "Error constructing model from urdf file." << endl;
		return false;
	}

	model->gravity.set (0., 0., -9.81);

	return true;
}

}

}
