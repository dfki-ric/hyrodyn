#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <string>
#include <vector>

#include "HyRoDyn_Limit_Checker.hpp"  // Adjust the include path to match your project

namespace py = pybind11;
using namespace pybind11::literals;

class PyHyRoDynLimitChecker {
 public:
  std::unique_ptr<HyRoDyn_Limit_Checker> hlc;

  PyHyRoDynLimitChecker(const std::string& urdf_filename,
                        const std::string& submechanisms_filename) {
    hlc = std::make_unique<HyRoDyn_Limit_Checker>(urdf_filename, submechanisms_filename);
  }

  std::vector<std::vector<double>> get_report(const std::vector<unsigned int>& raw_report) {
    std::vector<std::vector<double>> result;
    for (auto val : raw_report) {
      result.push_back({static_cast<double>(val)});  // wrap each int in a vector
    }
    return result;
  }

  std::vector<std::vector<double>> input_pos_limit_violation_report() {
    return get_report(hlc->input_pos_limit_violation_report);
  }

  std::vector<std::vector<double>> input_vel_limit_violation_report() {
    return get_report(hlc->input_vel_limit_violation_report);
  }

  std::vector<std::vector<double>> cspace_pos_limit_violation_report() {
    return get_report(hlc->cspace_pos_limit_violation_report);
  }

  std::vector<std::vector<double>> cspace_vel_limit_violation_report() {
    return get_report(hlc->cspace_vel_limit_violation_report);
  }

  std::vector<std::vector<double>> actuator_pos_limit_violation_report() {
    return get_report(hlc->actuator_pos_limit_violation_report);
  }

  std::vector<std::vector<double>> actuator_vel_limit_violation_report() {
    return get_report(hlc->actuator_vel_limit_violation_report);
  }

  std::vector<std::vector<double>> actuator_effort_limit_violation_report() {
    return get_report(hlc->actuator_effort_limit_violation_report);
  }

  bool hierarchical_capability_check(py::array_t<double> y, py::array_t<double> yd,
                                     py::array_t<double> ydd) {
    auto y_buf = y.unchecked<1>();
    auto yd_buf = yd.unchecked<1>();
    auto ydd_buf = ydd.unchecked<1>();

    Eigen::VectorXd y_vec(y_buf.shape(0));
    Eigen::VectorXd yd_vec(yd_buf.shape(0));
    Eigen::VectorXd ydd_vec(ydd_buf.shape(0));

    for (ssize_t i = 0; i < y_buf.shape(0); ++i) y_vec[i] = y_buf(i);
    for (ssize_t i = 0; i < yd_buf.shape(0); ++i) yd_vec[i] = yd_buf(i);
    for (ssize_t i = 0; i < ydd_buf.shape(0); ++i) ydd_vec[i] = ydd_buf(i);

    return hlc->hierarchical_capability_check(y_vec, yd_vec, ydd_vec);
  }
};

PYBIND11_MODULE(limit_checker_py, m) {
  py::class_<PyHyRoDynLimitChecker>(m, "HyRoDynLimitChecker")
      .def(py::init<const std::string&, const std::string&>(), "urdf_filename"_a,
           "submechanisms_filename"_a)

      .def_property_readonly("input_pos_limit_violation_report",
                             &PyHyRoDynLimitChecker::input_pos_limit_violation_report)
      .def_property_readonly("input_vel_limit_violation_report",
                             &PyHyRoDynLimitChecker::input_vel_limit_violation_report)
      .def_property_readonly("cspace_pos_limit_violation_report",
                             &PyHyRoDynLimitChecker::cspace_pos_limit_violation_report)
      .def_property_readonly("cspace_vel_limit_violation_report",
                             &PyHyRoDynLimitChecker::cspace_vel_limit_violation_report)
      .def_property_readonly("actuator_pos_limit_violation_report",
                             &PyHyRoDynLimitChecker::actuator_pos_limit_violation_report)
      .def_property_readonly("actuator_vel_limit_violation_report",
                             &PyHyRoDynLimitChecker::actuator_vel_limit_violation_report)
      .def_property_readonly("actuator_effort_limit_violation_report",
                             &PyHyRoDynLimitChecker::actuator_effort_limit_violation_report)

      .def("hierarchical_capability_check", &PyHyRoDynLimitChecker::hierarchical_capability_check,
           "Check hierarchical capabilities", py::arg("y"), py::arg("yd"), py::arg("ydd"));
}
