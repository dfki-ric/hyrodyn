#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <string>
#include <vector>

#include "robot_model_hyrodyn.hpp"  // Adjust if your class lives elsewhere

namespace py = pybind11;

PYBIND11_MODULE(hyrodyn_py, m) {
  py::class_<hyrodyn::RobotModel_HyRoDyn>(m, "RobotModel")
      .def(py::init<>())
      .def(py::init([](const std::string& urdf, const std::string& submech) {
        auto obj = std::make_unique<hyrodyn::RobotModel_HyRoDyn>();
        obj->load_robotmodel(urdf, submech);
        return obj;
      }))
      .def("welcome", &hyrodyn::RobotModel_HyRoDyn::welcome)
      .def("calculate_system_state", &hyrodyn::RobotModel_HyRoDyn::calculate_system_state)
      .def("calculate_forward_dynamics", &hyrodyn::RobotModel_HyRoDyn::calculate_forward_dynamics)
      .def("calculate_inverse_dynamics", &hyrodyn::RobotModel_HyRoDyn::calculate_inverse_dynamics)
      .def("calculate_forward_kinematics",
           &hyrodyn::RobotModel_HyRoDyn::calculate_forward_kinematics)
      .def("calculate_forward_kinematics_multiple_bodies",
           &hyrodyn::RobotModel_HyRoDyn::calculate_forward_kinematics_multiple_bodies)
      .def("calculate_inverse_kinematics",
           &hyrodyn::RobotModel_HyRoDyn::calculate_inverse_kinematics)
      .def("calculate_condition_number", &hyrodyn::RobotModel_HyRoDyn::calculate_condition_number)
      .def("calculate_com_properties", &hyrodyn::RobotModel_HyRoDyn::calculate_com_properties)
      .def("calculate_kinetic_energy", &hyrodyn::RobotModel_HyRoDyn::calculate_kinetic_energy)
      .def("calculate_potential_energy", &hyrodyn::RobotModel_HyRoDyn::calculate_potential_energy)
      .def("calculate_total_energy", &hyrodyn::RobotModel_HyRoDyn::calculate_total_energy)

      // Properties (exposed as read/write numpy arrays)
      .def_readwrite("Q", &hyrodyn::RobotModel_HyRoDyn::Q)
      .def_readwrite("QDot", &hyrodyn::RobotModel_HyRoDyn::QDot)
      .def_readwrite("QDDot", &hyrodyn::RobotModel_HyRoDyn::QDDot)
      .def_readwrite("Tau_spanningtree", &hyrodyn::RobotModel_HyRoDyn::Tau_spanningtree)

      .def_readwrite("y", &hyrodyn::RobotModel_HyRoDyn::y)
      .def_readwrite("yd", &hyrodyn::RobotModel_HyRoDyn::yd)
      .def_readwrite("ydd", &hyrodyn::RobotModel_HyRoDyn::ydd)

      .def_readwrite("Tau_independentjointspace",
                     &hyrodyn::RobotModel_HyRoDyn::Tau_independentjointspace)
      .def_readwrite("Tau_independentjointspace_input",
                     &hyrodyn::RobotModel_HyRoDyn::Tau_independentjointspace_input)

      .def_readwrite("u", &hyrodyn::RobotModel_HyRoDyn::u)
      .def_readwrite("ud", &hyrodyn::RobotModel_HyRoDyn::ud)
      .def_readwrite("udd", &hyrodyn::RobotModel_HyRoDyn::udd)
      .def_readwrite("Tau_actuated", &hyrodyn::RobotModel_HyRoDyn::Tau_actuated)

      .def_readwrite("pose", &hyrodyn::RobotModel_HyRoDyn::pose)
      .def_readwrite("pose_input", &hyrodyn::RobotModel_HyRoDyn::pose_input)
      .def_readwrite("poses", &hyrodyn::RobotModel_HyRoDyn::poses)

      .def_readonly("mass", &hyrodyn::RobotModel_HyRoDyn::mass)
      .def_readonly("com", &hyrodyn::RobotModel_HyRoDyn::com)
      .def_readonly("com_vel", &hyrodyn::RobotModel_HyRoDyn::com_vel)
      .def_readonly("com_angularmomentum", &hyrodyn::RobotModel_HyRoDyn::com_angularmomentum)

      // Joint name access
      .def_readonly("jointnames_spanningtree",
                    &hyrodyn::RobotModel_HyRoDyn::jointnames_spanningtree)
      .def_readonly("jointnames_independent", &hyrodyn::RobotModel_HyRoDyn::jointnames_independent)
      .def_readonly("jointnames_active", &hyrodyn::RobotModel_HyRoDyn::jointnames_active)

      // Distribution and DoF
      .def_readonly("submechanism_spanningtree_dof_distribution",
                    &hyrodyn::RobotModel_HyRoDyn::submechanism_spanningtree_dof_distribution)
      .def_readonly("spanningtree_dof", &hyrodyn::RobotModel_HyRoDyn::spanningtree_dof)
      .def_readonly("submechanism_independent_dof_distribution",
                    &hyrodyn::RobotModel_HyRoDyn::submechanism_independent_dof_distribution)
      .def_readonly("independent_dof", &hyrodyn::RobotModel_HyRoDyn::independent_dof)
      .def_readonly("submechanism_active_dof_distribution",
                    &hyrodyn::RobotModel_HyRoDyn::submechanism_active_dof_distribution)
      .def_readonly("active_dof", &hyrodyn::RobotModel_HyRoDyn::active_dof);
}
