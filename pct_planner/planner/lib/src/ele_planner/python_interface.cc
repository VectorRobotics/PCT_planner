#include "ele_planner/offline_ele_planner.h"
#include "pybind11/eigen.h"
#include "pybind11/pybind11.h"

namespace py = pybind11;

PYBIND11_MODULE(ele_planner, m) {
  auto pyOfflineElePlanner =
      py::class_<OfflineElePlanner>(m, "OfflineElePlanner");
  pyOfflineElePlanner
      .def(py::init<double, bool>(), py::arg("max_heading_rate"),
           py::arg("use_quintic") = false)
      .def("init_map", &OfflineElePlanner::InitMap)
      .def("plan", &OfflineElePlanner::Plan)
      .def("debug", &OfflineElePlanner::Debug)
      .def("set_reference_height", &OfflineElePlanner::SetReferenceHeight)
      .def("set_max_iterations", &OfflineElePlanner::set_max_iterations)
      .def("set_sample_interval", &OfflineElePlanner::set_sample_interval)
      .def("set_interpolate_num", &OfflineElePlanner::set_interpolate_num)
      .def("set_lambda_initial", &OfflineElePlanner::set_lambda_initial)
      .def("set_qc_position", &OfflineElePlanner::set_qc_position)
      .def("set_qc_heading", &OfflineElePlanner::set_qc_heading)
      .def("set_path_debug", &OfflineElePlanner::set_path_debug)
      .def("get_path_finder", &OfflineElePlanner::get_path_finder)
      .def("get_map", &OfflineElePlanner::get_map)
      .def("get_trajectory_optimizer",
           &OfflineElePlanner::get_trajectory_optimizer)
      .def("get_trajectory_optimizer_wnoj",
           &OfflineElePlanner::get_trajectory_optimizer_wnoj)
      .def("get_debug_path", &OfflineElePlanner::GetDebugPath);
}