#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace py = pybind11;

// ---- load_joint_limits_yaml ----
std::vector<std::tuple<std::string, double, double>> load_joint_limits_yaml(
    const std::string& yaml_filename) {
  std::ifstream file(yaml_filename);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open YAML file: " + yaml_filename);
  }

  YAML::Node config = YAML::Load(file);
  auto names = config["limits"]["names"];
  auto elements = config["limits"]["elements"];

  std::vector<std::tuple<std::string, double, double>> result;
  for (std::size_t i = 0; i < elements.size(); ++i) {
    std::string name = names[i].as<std::string>();
    double min_val = elements[i]["min"]["position"].as<double>();
    double max_val = elements[i]["max"]["position"].as<double>();

    result.emplace_back(name, min_val, max_val);
  }

  return result;
}

// ---- discretize_joint_space ----
py::array_t<float> discretize_joint_space(
    const std::vector<std::tuple<std::string, double, double>>& joint_data, int num_steps,
    py::dtype dtype = py::dtype::of<float>()) {
  std::vector<std::vector<float>> discretized;

  for (const auto& joint : joint_data) {
    const auto& [name, min_val, max_val] = joint;
    std::vector<float> values(num_steps);
    float step = (max_val - min_val) / (num_steps - 1);
    for (int i = 0; i < num_steps; ++i) {
      values[i] = static_cast<float>(min_val + i * step);
    }

    std::cout << "Generated " << values.size() << " points for joint " << name << " (from "
              << min_val << " to " << max_val << ")" << std::endl;

    discretized.push_back(std::move(values));
  }

  // Cartesian product
  std::vector<std::vector<float>> product = {{}};
  for (const auto& dim : discretized) {
    std::vector<std::vector<float>> temp;
    for (const auto& p : product) {
      for (float val : dim) {
        auto copy = p;
        copy.push_back(val);
        temp.push_back(std::move(copy));
      }
    }
    product = std::move(temp);
  }

  std::cout << "Generated " << product.size() << " points from cartesian product" << std::endl;

  // Convert to numpy array
  py::array_t<float> result(
      {static_cast<ssize_t>(product.size()), static_cast<ssize_t>(joint_data.size())});
  auto buf = result.mutable_unchecked<2>();

  for (size_t i = 0; i < product.size(); ++i) {
    for (size_t j = 0; j < product[i].size(); ++j) {
      buf(i, j) = product[i][j];
    }
  }

  return result;
}

PYBIND11_MODULE(utils_py, m) {
  m.def("load_joint_limits_yaml", &load_joint_limits_yaml, "Load joint limits from a YAML file");
  m.def("discretize_joint_space", &discretize_joint_space, py::arg("joint_data"),
        py::arg("num_steps"), py::arg("dtype") = py::dtype::of<float>(),
        "Discretize joint space based on limits and number of steps");
}
