#include "trajectory_optimization/gpmp_optimizer/gpmp_optimizer.h"

#include <chrono>

#include "gtsam/nonlinear/LevenbergMarquardtOptimizer.h"
#include "gtsam/nonlinear/LevenbergMarquardtParams.h"
#include "gtsam/nonlinear/Symbol.h"
#include "trajectory_optimization/gpmp_optimizer/factors/gp_heading_rate_factor.h"
#include "trajectory_optimization/gpmp_optimizer/factors/gp_interpolate_heading_rate_factor.h"
#include "trajectory_optimization/gpmp_optimizer/factors/gp_interpolate_obstacle_factor.h"
#include "trajectory_optimization/gpmp_optimizer/factors/gp_obstacle_factor.h"
#include "trajectory_optimization/gpmp_optimizer/factors/gp_prior_factor.h"
#include "trajectory_optimization/gpmp_optimizer/interpolator/wnoj_trajectory_interpolator.h"

using gtsam::noiseModel::Diagonal;
using gtsam::noiseModel::Isotropic;
using PriorFactor6 = gtsam::PriorFactor<Vector6>;

bool GPMPOptimizer::GenerateTrajectory(const std::vector<PathPoint>& input_path,
                                       const double T) {
  auto t0 = std::chrono::high_resolution_clock::now();

  // Compute Qc values from parameters
  const double kQc = qc_position_ * qc_position_;
  const double kQcHeading = qc_heading_;

  if (debug_) {
    printf("[GPMP] === Trajectory Optimization Parameters ===\n");
    printf("[GPMP] sample_interval: %d\n", sample_interval_);
    printf("[GPMP] interpolate_num: %d\n", interpolate_num_);
    printf("[GPMP] max_iterations: %d\n", max_iterations_);
    printf("[GPMP] lambda_initial: %.1f\n", lambda_initial_);
    printf("[GPMP] qc_position: %.4f (kQc=%.6f)\n", qc_position_, kQc);
    printf("[GPMP] qc_heading: %.4f\n", kQcHeading);
    printf("[GPMP] safe_cost_margin: %.1f\n", safe_cost_margin_);
    printf("[GPMP] max_heading_rate: %.2f\n", max_heading_rate_);
    printf("[GPMP] ==========================================\n");
  }

  Vector6 s_init, s_target;
  // Position sigma: 0.01 = 1cm tolerance (was 0.001=1mm which was too tight)
  // Velocity sigma: 0.5 for flexibility (was 0.1/1.0)
  // This balances position accuracy with trajectory smoothness
  s_init << 0.01, 0.5, 1, 0.01, 0.5, 1;
  s_target << 0.01, 0.5, 1, 0.01, 0.5, 1;

  auto sigma_initial = Diagonal::Sigmas(s_init);
  auto sigma_goal = Diagonal::Sigmas(s_target);

  std::vector<PathPoint> path;
  SubSamplePath(input_path, path);

  double T_tmp = static_cast<double>(input_path.size() - 1) / 3;
  int N = path.size();
  double dt = T_tmp / (N - 1);
  double tau = dt / (interpolate_num_ + 1);

  if (debug_) {
    printf("[GPMP] Input path size: %zu, Subsampled path size: %d\n",
           input_path.size(), N);
    printf("[GPMP] dt: %.4f, tau: %.4f\n", dt, tau);
  }

  Vector6 x0, xN;
  PathPointToNode(path.front(), x0);
  PathPointToNode(path.back(), xN);

  if (debug_) {
    printf("[GPMP] === Initial/Goal States ===\n");
    printf("[GPMP] x0: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n",
           x0(0), x0(1), x0(2), x0(3), x0(4), x0(5));
    printf("[GPMP] xN: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n",
           xN(0), xN(1), xN(2), xN(3), xN(4), xN(5));
    printf("[GPMP] path.front(): x=%.3f, y=%.3f, heading=%.3f, ref_v=%.3f\n",
           path.front().x, path.front().y, path.front().heading, path.front().ref_v);
    printf("[GPMP] path.back(): x=%.3f, y=%.3f, heading=%.3f, ref_v=%.3f\n",
           path.back().x, path.back().y, path.back().heading, path.back().ref_v);
    printf("[GPMP] ==============================\n");
  }

  auto graph = gtsam::NonlinearFactorGraph();
  int factor_idx = 0;
  std::vector<int> obstacle_factor_idx;

  gtsam::Values init_values;
  opt_init_value_ = Eigen::MatrixXd(6, N + (N - 1) * interpolate_num_);
  opt_init_layer_ = Eigen::VectorXd(N + (N - 1) * interpolate_num_);

  int col_index = 0;
  for (int i = 0; i < N; ++i) {
    Vector6 x1, x2;
    PathPointToNode(path[i], x1);
    init_values.insert<Vector6>(gtsam::Symbol('x', i), x1);
    opt_init_value_.col(col_index) = x1;
    opt_init_layer_(col_index) = path[i].layer;
    if (debug_) {
      printf("path[%d/%d], layer: %d, height: %f\n", i, N - 1, path[i].layer,
             path[i].height);
    }
    col_index++;
    if (i < N - 1) {
      PathPointToNode(path[i + 1], x2);
      for (int j = 0; j < interpolate_num_; ++j) {
        auto inter_x =
            GPInterpolator::Interpolate(x1, x2, kQc, dt, tau * (j + 1));
        double height_hint =
            path[i].height + (path[i + 1].height - path[i].height) * (j + 1) /
                                 (interpolate_num_ + 1);
        opt_init_value_.col(col_index) = inter_x;
        opt_init_layer_(col_index) = map_->UpdateLayerSafe(
            path[i].layer, inter_x(0, 0), inter_x(3, 0), height_hint);
        col_index++;
        if (debug_) {
          printf(
              "path[%d/%d], layer: %d, inter_layer: %f, height: %f, "
              "height2:%f, hint%f, (%f, %f)\n",
              i, opt_init_layer_.size() - 1, path[i].layer,
              opt_init_layer_(col_index), path[i].height, path[i + 1].height,
              height_hint, inter_x(0, 0), inter_x(3, 0));
        }
      }
    }
  }

  if (debug_) {
    std::cout << "opt_init_layer OK" << std::endl;
  }

  graph.add(PriorFactor6(gtsam::Symbol('x', 0), x0, sigma_initial));
  factor_idx++;
  for (int i = 1; i < N; ++i) {
    gtsam::Key last_x = gtsam::Symbol('x', i - 1);
    gtsam::Key this_x = gtsam::Symbol('x', i);
    graph.add(GPPriorFactor(last_x, this_x, dt, kQc));
    factor_idx++;
    for (int j = 0; j < interpolate_num_; ++j) {
      double height_hint =
          path[i - 1].height + (path[i].height - path[i - 1].height) * (j + 1) /
                                   (interpolate_num_ + 1);
      graph.add(GPInterpolateObstacleFactor(
          last_x, this_x, map_, path[i].layer, height_hint, safe_cost_margin_,
          0.1, kQc, dt, (i - 1) * dt, tau * (j + 1)));
      obstacle_factor_idx.emplace_back(factor_idx + 1e7);
      graph.add(GPInterpolateHeadingRateFactor(
          last_x, this_x, max_heading_rate_, kQcHeading, kQc, dt, (i - 1) * dt,
          tau * (j + 1)));
      factor_idx += 2;
    }
    if (i < N - 1) {
      graph.add(GPObstacleFactor(this_x, map_, path[i].layer, path[i].height,
                                 0.1, safe_cost_margin_, true));
      obstacle_factor_idx.emplace_back(factor_idx);
      graph.add(GPHeadingRateFactor(this_x, max_heading_rate_, kQcHeading));
      factor_idx += 2;
    }
  }
  graph.add(PriorFactor6(gtsam::Symbol('x', N - 1), xN, sigma_goal));
  factor_idx++;

  // Analyze initial error before optimization
  if (debug_) {
    double total_error = graph.error(init_values);
    printf("[GPMP] === Initial Error Analysis ===\n");
    printf("[GPMP] Total initial error: %.4f\n", total_error);
    printf("[GPMP] Number of factors: %zu\n", graph.size());

    // Break down error by factor type
    double prior_error = 0.0;
    double gp_prior_error = 0.0;
    double obstacle_error = 0.0;
    double heading_rate_error = 0.0;

    for (size_t i = 0; i < graph.size(); ++i) {
      auto factor = graph.at(i);
      double error = factor->error(init_values);

      // Categorize by factor type (rough heuristic based on keys)
      if (error > 1000.0) {
        printf("[GPMP] Factor %zu: error=%.4f (LARGE!)\n", i, error);
      }

      // Simple classification by number of keys
      if (factor->keys().size() == 1) {
        prior_error += error;
      } else if (factor->keys().size() == 2) {
        gp_prior_error += error;
      }
    }

    printf("[GPMP] Approximate breakdown:\n");
    printf("[GPMP]   Single-key factors (priors/heading): %.4f\n", prior_error);
    printf("[GPMP]   Two-key factors (GP/obstacles): %.4f\n", gp_prior_error);
    printf("[GPMP] =====================================\n");
  }

  gtsam::LevenbergMarquardtParams param;
  param.setlambdaInitial(lambda_initial_);
  param.setMaxIterations(max_iterations_);
  // param.setAbsoluteErrorTol(5e-4);
  // param.setRelativeErrorTol(0.01);
  // param.setErrorTol(1.0);
  if (debug_) {
    param.setVerbosity("ERROR");
  }

  gtsam::LevenbergMarquardtOptimizer opt(graph, init_values, param);
  opt_results_ = Eigen::MatrixXd(N, 6);
  auto solution = opt.optimize();

  opt_layers_ = Eigen::VectorXd::Zero(N + (N - 1) * interpolate_num_);
  opt_layers_(0) = path.front().layer;
  opt_layers_(opt_layers_.size() - 1) = path.back().layer;
  for (int i = 0; i < obstacle_factor_idx.size(); ++i) {
    // printf("%d, %d, %d, %d\n", i + 1, N, obstacle_factor_idx.size(),
    //        obstacle_factor_idx[i]);
    if (obstacle_factor_idx[i] < 1e7) {
      opt_layers_(i + 1) = dynamic_cast<GPObstacleFactor*>(
                               graph.at(obstacle_factor_idx[i]).get())
                               ->GetNodeLayer();
    } else {
      opt_layers_(i + 1) = dynamic_cast<GPInterpolateObstacleFactor*>(
                               graph.at(obstacle_factor_idx[i] - 1e7).get())
                               ->GetNodeLayer();
    }
  }

  for (int i = 0; i < N; ++i) {
    opt_results_.row(i) =
        solution.at<Vector6>(gtsam::Symbol('x', i)).transpose();
  }

  WnojTrajectoryInterpolator traj_interpolator =
      WnojTrajectoryInterpolator(opt_results_, dt, kQc);
  trajectory_ = traj_interpolator.GenerateTrajectory(interpolate_num_);
  printf("shape: %d, %d\n", trajectory_.rows(), trajectory_.cols());

  printf("smooth height\n");
  opt_height_ = Eigen::VectorXd::Zero(N + (N - 1) * interpolate_num_);
  opt_ceiling_ = Eigen::VectorXd::Zero(N + (N - 1) * interpolate_num_);

  for (int i = 0; i < opt_layers_.size(); ++i) {
    opt_height_(i) =
        map_->GetHeight(opt_layers_(i), trajectory_(i, 0), trajectory_(i, 3)) +
        reference_height_;
    opt_ceiling_(i) =
        map_->GetCeiling(opt_layers_(i), trajectory_(i, 0), trajectory_(i, 3));
  }
  opt_height_ = height_smoother_.Smooth(opt_height_, opt_ceiling_, tau, N, dt);

  auto t1 = std::chrono::high_resolution_clock::now();
  double final_cost = opt.error();
  int iterations = opt.iterations();

  printf(
      "[GPMP] Optimization finished: N=%d, iterations=%d/%d, time=%.2fms, cost=%.4f",
      N, iterations, max_iterations_,
      std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / 1000.0,
      final_cost);

  if (iterations >= max_iterations_) {
    printf(" [WARNING: Max iterations reached, may not have converged!]");
  }
  if (final_cost > 50.0) {
    printf(" [WARNING: High cost, poor convergence!]");
  }
  printf("\n");

  if (debug_) {
    printf("[GPMP] Final trajectory points: %ld\n", trajectory_.rows());
  }

  return true;
}

void GPMPOptimizer::PathPointToNode(const PathPoint& path_point, Vector6& x) {
  // Use much smaller default velocity (0.1 instead of 1.0) to avoid
  // constraint violations with tight position priors
  // A* doesn't set ref_v, so it defaults to 0.0
  double v = (path_point.ref_v > 0.01) ? path_point.ref_v : 0.1;
  x(0, 0) = path_point.x;
  x(1, 0) = std::cos(path_point.heading) * v;
  x(2, 0) = 0.0;
  x(3, 0) = path_point.y;
  x(4, 0) = std::sin(path_point.heading) * v;
  x(5, 0) = 0.0;
}

void GPMPOptimizer::SubSamplePath(const std::vector<PathPoint>& path,
                                  std::vector<PathPoint>& sub_sampled_path) {
  assert(path.size() > 1);

  if (!sub_sampled_path.empty()) {
    sub_sampled_path.clear();
  }

  int size = path.size();
  int num_segs = std::ceil((size - 1) / sample_interval_);

  if (num_segs <= 1) {
    sub_sampled_path.emplace_back(path.front());
    sub_sampled_path.emplace_back(path.back());
    return;
  }

  double new_interval = static_cast<double>(size - 1) / num_segs;

  int idx;
  double float_idx = 0.0;

  sub_sampled_path.emplace_back(path.front());

  for (int i = 1; i < num_segs + 1; ++i) {
    float_idx += new_interval;
    idx = static_cast<int>(float_idx);

    if (idx == size - 1) {
      sub_sampled_path.emplace_back(path[idx]);
    } else {
      sub_sampled_path.emplace_back(
          PathPoint::Interpolate(path[idx], path[idx + 1], float_idx - idx));
    }
  }

  printf("num segs: %d, new interval: %f\n", num_segs, new_interval);
}

Eigen::VectorXd GPMPOptimizer::GetHeadingRate() const {
  assert(trajectory_.rows() > 0);

  Eigen::VectorXd heading_rate = Eigen::VectorXd::Zero(trajectory_.rows());
  for (int i = 0; i < trajectory_.rows(); ++i) {
    double dx = trajectory_(i, 1);
    double ddx = trajectory_(i, 2);
    double dy = trajectory_(i, 4);
    double ddy = trajectory_(i, 5);
    heading_rate(i) = (ddy * dx - dy * ddx) / (dx * dx + dy * dy + 1e-6);
  }
  return heading_rate;
}