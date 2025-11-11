#pragma once

#include <memory>

#include "a_star/a_star_search.h"
#include "common/data_types.h"
#include "map_manager/dense_elevation_map.h"
#include "trajectory_optimization/gpmp_optimizer/gpmp_optimizer.h"
#include "trajectory_optimization/gpmp_optimizer/gpmp_optimizer_wnoa.h"

class OfflineElePlanner {
 public:
  OfflineElePlanner(const double max_heading_rate, bool use_quintic)
      : use_quintic_(use_quintic), max_heading_rate_(max_heading_rate) {}
  ~OfflineElePlanner() = default;

  void InitMap(const double a_start_cost_threshold,
               const double safe_cost_margin, const double resolution,
               const int num_layers, const double step_cost_weight, const Eigen::MatrixXd& cost_map,
               const Eigen::MatrixXd& height_map,
               const Eigen::MatrixXd& ceiling, const Eigen::MatrixXd& ele_map,
               const Eigen::MatrixXd& grad_x, const Eigen::MatrixXd& grad_y);

  bool Plan(const Eigen::Vector3i& start, const Eigen::Vector3i& goal,
            const bool optimize = true);

  void SetReferenceHeight(const double height) {
    trajectory_optimizer_wnoj_.SetReferenceHeight(height);
  }

  void Debug() {
    path_finder_.Debug();
    trajectory_optimizer_.SetDebug(true);
    trajectory_optimizer_wnoj_.SetDebug(true);
  }

  Eigen::MatrixXd GetDebugPath() const {
    return path_finder_.GetResultMatrix();
  }

  void set_max_iterations(int max_iterations) {
    trajectory_optimizer_.set_max_iterations(max_iterations);
    trajectory_optimizer_wnoj_.set_max_iterations(max_iterations);
  }

  void set_sample_interval(int sample_interval) {
    trajectory_optimizer_.set_sample_interval(sample_interval);
    trajectory_optimizer_wnoj_.set_sample_interval(sample_interval);
  }

  void set_interpolate_num(int interpolate_num) {
    trajectory_optimizer_.set_interpolate_num(interpolate_num);
    trajectory_optimizer_wnoj_.set_interpolate_num(interpolate_num);
  }

  void set_lambda_initial(double lambda_initial) {
    trajectory_optimizer_wnoj_.set_lambda_initial(lambda_initial);
  }

  void set_qc_position(double qc_position) {
    trajectory_optimizer_wnoj_.set_qc_position(qc_position);
  }

  void set_qc_heading(double qc_heading) {
    trajectory_optimizer_wnoj_.set_qc_heading(qc_heading);
  }

  void set_path_debug(bool debug) {
    path_finder_.Debug(debug);
  }

  const Astar& get_path_finder() const { return path_finder_; }
  const DenseElevationMap& get_map() const { return *map_; }
  const GPMPOptimizerWnoa& get_trajectory_optimizer() const {
    return trajectory_optimizer_;
  }
  const GPMPOptimizer& get_trajectory_optimizer_wnoj() const {
    return trajectory_optimizer_wnoj_;
  }

 private:
  double max_heading_rate_ = 0.5;
  bool use_quintic_ = false;

  std::shared_ptr<DenseElevationMap> map_;
  Astar path_finder_;
  GPMPOptimizerWnoa trajectory_optimizer_;
  GPMPOptimizer trajectory_optimizer_wnoj_;

  std::vector<PathPoint> path_;
  std::vector<Eigen::Vector3d> trajectory_;
};