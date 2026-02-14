#ifndef RBDL_URDFREADER_H
#define RBDL_URDFREADER_H

#include <rbdl/rbdl_config.h>
#include <rbdl/rbdl_math.h>

#include <string>
#include <vector>

namespace RigidBodyDynamics {

struct Model;

namespace Addons {
/**
 * This function will load a URDF model from a file.
 *
 * @param filename: path to the URDF file
 * @param model: reference to the (loaded) multibody model
 * @param floating_pase: does the model use a floating base
 * @param verbose: information will be printed to the command window if this
 *                 is set to true
 */
RBDL_ADDON_DLLAPI bool URDFReadFromFile(const char* filename, Model* model, bool floating_base,
                                        bool verbose = false);

/**
 * This function will load a Partial URDF model from a file.
 *
 * @param filename: path to the URDF file
 * @param model: reference to the (loaded) multibody model
 * @param floating_pase: does the model use a floating base
 * @param verbose: information will be printed to the command window if this
 *                 is set to true
 */
RBDL_ADDON_DLLAPI bool PartialURDFReadFromFile(const char* filename, Model* model,
                                               const std::string& root_link,
                                               const std::vector<std::string>& tip_links,
                                               bool floating_base, bool verbose = false);

/**
 * This function will load a URDF model from a c string.
 *
 * @param model_xml_string: URDF data in form of a string
 * @param model: reference to the (loaded) multibody model
 * @param floating_pase: does the model use a floating base
 * @param verbose: information will be printed to the command window if this
 *                 is set to true
 */
RBDL_ADDON_DLLAPI bool URDFReadFromString(const char* model_xml_string, Model* model,
                                          bool floating_base, bool verbose = false);
/**
 * This function will load a Partial URDF model from a c string.
 *
 * @param model_xml_string: URDF data in form of a string
 * @param model: reference to the (loaded) multibody model
 * @param floating_pase: does the model use a floating base
 * @param verbose: information will be printed to the command window if this
 *                 is set to true
 */
RBDL_ADDON_DLLAPI bool PartialURDFReadFromString(const char* model_xml_string, Model* model,
                                                 const std::string& root_link,
                                                 const std::vector<std::string>& tip_links,
                                                 bool floating_base, bool verbose = false);

// // HyRoDyn Specific Parser
RBDL_ADDON_DLLAPI bool URDFReadLoopClosureFunction(const char* filename, MatrixN_t& G,
                                                   VectorN_t& offset, MatrixN_t& Gu,
                                                   std::vector<std::string>& actuated_joint_names,
                                                   std::vector<std::string>& tree_joint_names);

RBDL_ADDON_DLLAPI bool URDFReadLoopClosureFunctionExterior(
    const char* filename, MatrixN_t& G, VectorN_t& offset,
    const std::vector<std::string>& actuated_joint_names,
    const std::vector<std::string>& tree_joint_names);

RBDL_ADDON_DLLAPI bool URDFReadJointLimits(
    const char* filename, const std::vector<std::string>& joint_names_respecting_modularity,
    VectorN_t& q_max, VectorN_t& q_min, VectorN_t& vel_limit, VectorN_t& effort_limit);

RBDL_ADDON_DLLAPI bool URDFReadLoopClosureFunctionTransmission(
    const char* filename, MatrixN_t& G, VectorN_t& offset,
    const std::vector<std::string>& independent_joint_names,
    const std::vector<std::string>& tree_joint_names, bool verbose = false);

RBDL_ADDON_DLLAPI bool URDFReadFromStringWithModularity(const char* model_xml_string, Model* model,
                                                        const std::vector<std::string>& joint_names,
                                                        bool floating_base, bool verbose = false);

RBDL_ADDON_DLLAPI bool URDFReadFromFileWithModularity(const char* filename, Model* model,
                                                      const std::vector<std::string>& joint_names,
                                                      bool floating_base, bool verbose = false);

}  // namespace Addons

}  // namespace RigidBodyDynamics

/* _RBDL_URDFREADER_H */
#endif
