#include "a_star/a_star_search.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <queue>
#include <unordered_map>
#include <unordered_set>

using std::cout;
using std::endl;

// 9 neighbors in 2d
static std::vector<Eigen::Vector2i> kNeighbors = std::vector<Eigen::Vector2i>{
    Eigen::Vector2i(-1, -1), Eigen::Vector2i(-1, 0), Eigen::Vector2i(-1, 1),
    Eigen::Vector2i(0, -1),  Eigen::Vector2i(0, 1),  Eigen::Vector2i(1, -1),
    Eigen::Vector2i(1, 0),   Eigen::Vector2i(1, 1),
};

void Astar::Init(const double cost_threshold, const int num_layers,
                 const double resolution,  const double step_cost_weight, const Eigen::MatrixXd& cost_map,
                 const Eigen::MatrixXd& height_map,
                 const Eigen::MatrixXd& ele_map) {
  auto t0 = std::chrono::high_resolution_clock::now();
  cost_threshold_ = cost_threshold;
  step_cost_weight_ = step_cost_weight;
  resolution_ = resolution;

  max_x_ = cost_map.cols();
  max_y_ = cost_map.rows() / num_layers;
  max_layers_ = num_layers;
  xy_size_ = max_x_ * max_y_;

  int row_offset = 0;
  grid_map_.resize(max_layers_);
  for (size_t i = 0; i < max_layers_; ++i) {
    row_offset = i * max_y_;
    grid_map_[i].resize(max_y_);
    for (size_t j = 0; j < max_y_; ++j) {
      grid_map_[i][j].resize(max_x_);
      for (size_t k = 0; k < max_x_; ++k) {
        double height = height_map(j + row_offset, k);
        double z = static_cast<int>(height / resolution);
        grid_map_[i][j][k] = Node(Eigen::Vector3i(z, j, k), nullptr);
        grid_map_[i][j][k].cost = cost_map(j + row_offset, k);
        grid_map_[i][j][k].height = height;
        grid_map_[i][j][k].ele = ele_map(j + row_offset, k);
        grid_map_[i][j][k].layer = i;
      }
    }
  }
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::high_resolution_clock::now() - t0);

  search_layers_offset_.clear();
  search_layers_offset_.emplace_back(0);
  for (int i = 0; i < search_layer_depth_; ++i) {
    search_layers_offset_.emplace_back(-(i + 1));
    search_layers_offset_.emplace_back(i + 1);
  }

  printf(
      "Astar initialized, max_x: %d, max_y: %d, max_layers: %d, time elapsed: "
      "%f ms\n",
      max_x_, max_y_, max_layers_, duration.count() / 1000.0);
}

void Astar::Reset() {
  for (size_t i = 0; i < grid_map_.size(); ++i) {
    for (size_t j = 0; j < grid_map_[i].size(); ++j) {
      for (size_t k = 0; k < grid_map_[i][j].size(); ++k) {
        grid_map_[i][j][k].Reset();
      }
    }
  }
}

int Astar::GetHash(const Eigen::Vector3i& idx) const {
  return idx[0] * 10000000 + idx[1] * max_x_ + idx[2];
}

bool Astar::Search(const Eigen::Vector3i& start, const Eigen::Vector3i& goal) {
  auto t0 = std::chrono::high_resolution_clock::now();

  if (!search_result_.empty()) {
    Reset();
    search_result_.clear();
  }

  auto start_node = &grid_map_[start[0]][start[2]][start[1]];
  auto goal_node = &grid_map_[goal[0]][goal[2]][goal[1]];
  start_node->g = 0.0;


  std::priority_queue<Node*, std::vector<Node*>, NodeCompare> open_set;
  std::unordered_map<int, Node*> closed_set;

  open_set.push(start_node);

  printf("[A*] Start searching: start=[%d,%d,%d] (layer=%d), goal=[%d,%d,%d] (layer=%d)\n",
         start[0], start[1], start[2], start[0], goal[0], goal[1], goal[2], goal[0]);
  if (debug_) {
    printf("[A*] Start height=%.2f, goal height=%.2f\n",
           start_node->height, goal_node->height);
    printf("[A*] cost_threshold=%.1f, step_cost_weight=%.2f\n",
           cost_threshold_, step_cost_weight_);
  }

  int iterations = 0;
  int layer_transitions = 0;
  while (!open_set.empty()) {
    Node* current_node = open_set.top();
    open_set.pop();
    iterations++;

    if (current_node->idx == goal_node->idx) {
      while (current_node->parent != nullptr) {
        // search_result_.emplace_back(Eigen::Vector3i(
        //     current_node->layer, current_node->idx[1],
        //     current_node->idx[2]));
        search_result_.emplace_back(current_node);
        current_node = current_node->parent;
      }
      std::reverse(search_result_.begin(), search_result_.end());
      if (debug_) ConvertClosedSetToMatrix(closed_set);
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::high_resolution_clock::now() - t0);
      printf("[A*] Path found: %zu waypoints, %d iterations, %d layer transitions, %.2fms\n",
             search_result_.size(), iterations, layer_transitions, duration.count() / 1000.0);

      if (debug_) {
        // Analyze path characteristics
        int consecutive_layer_changes = 0;
        int max_consecutive_changes = 0;
        int current_streak = 0;
        for (size_t i = 1; i < search_result_.size(); ++i) {
          if (search_result_[i]->layer != search_result_[i-1]->layer) {
            consecutive_layer_changes++;
            current_streak++;
            max_consecutive_changes = std::max(max_consecutive_changes, current_streak);
          } else {
            current_streak = 0;
          }
        }
        printf("[A*] Path layer changes: %d total, max %d consecutive\n",
               consecutive_layer_changes, max_consecutive_changes);
      }

      return true;
    }

    closed_set[GetHash(current_node->idx)] = current_node;

    // Determine which layers to explore
    std::vector<int> candidate_layers;

    // Always explore current layer
    candidate_layers.push_back(current_node->layer);

    // Use DecideLayer for elevation-based transitions
    int primary_layer = DecideLayer(current_node);
    if (primary_layer != current_node->layer) {
      candidate_layers.push_back(primary_layer);
      layer_transitions++;
      if (debug_ && iterations % 100 == 0) {
        printf("[A*] Layer transition at iter %d: %d->%d (height=%.2f, cost=%.1f)\n",
               iterations, current_node->layer, primary_layer,
               current_node->height, current_node->cost);
      }
    }

    // Goal-directed layer exploration: if goal is on different layer, try moving towards it
    int layer_diff = goal_node->layer - current_node->layer;
    if (layer_diff != 0) {
      int target_layer = current_node->layer + (layer_diff > 0 ? 1 : -1);
      if (target_layer >= 0 && target_layer < max_layers_) {
        const Node& check = grid_map_[target_layer][current_node->idx[1]][current_node->idx[2]];
        // Only explore if height difference is reasonable
        if (std::abs(check.height - current_node->height) <= 0.5) {
          // Avoid duplicates
          bool already_added = false;
          for (int l : candidate_layers) {
            if (l == target_layer) {
              already_added = true;
              break;
            }
          }
          if (!already_added) {
            candidate_layers.push_back(target_layer);
          }
        }
      }
    }

    if (iterations == 1) {
      printf("First iteration: current_layer=%d, primary_layer=%d, candidate_layers_size=%zu\n",
             current_node->layer, primary_layer, candidate_layers.size());
    }

    int i, j = 0;
    double tentative_g = 0.0;

    // Expand neighbors on all candidate layers
    for (int layer : candidate_layers) {
      for (const auto& neighbor : kNeighbors) {
        i = current_node->idx[1] + neighbor[0];
        j = current_node->idx[2] + neighbor[1];

        if (i < 0 || i >= max_y_ || j < 0 || j >= max_x_) {
          continue;
        }

        auto neighbor_node = &grid_map_[layer][i][j];

      if (neighbor_node->cost > cost_threshold_) {
        if (std::abs(neighbor_node->ele) < 0.5) {
          continue;
        } else {
          double height_diff = std::abs(neighbor_node->height - current_node->height);
          if (height_diff > 0.4) {
            continue;
          }
        }
      }

      auto diff = neighbor_node->idx - current_node->idx;

      // Calculate traversal cost based on actual distance
      // For horizontal movement, use grid distance (diff[1] and diff[2])
      // For vertical, use actual height difference
      double horizontal_dist = std::sqrt(diff[1] * diff[1] + diff[2] * diff[2]);
      double vertical_dist = std::abs(neighbor_node->height - current_node->height) / resolution_;
      double distance_cost = std::sqrt(horizontal_dist * horizontal_dist + vertical_dist * vertical_dist);

      // Apply cost penalty smoothly - no sharp cutoffs
      double step_cost = step_cost_weight_ * neighbor_node->cost;

      tentative_g = current_node->g + distance_cost + step_cost;

      auto p_neighbor = closed_set.find(GetHash(neighbor_node->idx));
      if (p_neighbor != closed_set.end()) {
        if (tentative_g >= p_neighbor->second->g) {
          continue;
        }
      }

      if (tentative_g < neighbor_node->g) {
        neighbor_node->g = tentative_g;
        neighbor_node->f = tentative_g + GetHeuristic(neighbor_node, goal_node);
        neighbor_node->parent = current_node;
        open_set.push(neighbor_node);
      }
      }  // End neighbor loop
    }  // End layer loop
  }  // End while loop

  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::high_resolution_clock::now() - t0);
  printf("[A*] Path NOT found after %d iterations, %.2fms [FAILURE]\n",
         iterations, duration.count() / 1000.0);
  if (debug_) {
    printf("[A*] Explored %zu nodes before giving up\n", closed_set.size());
    ConvertClosedSetToMatrix(closed_set);
  }
  return false;
}

int Astar::DecideLayer(const Node* cur_node) const {
  int layer = cur_node->layer;
  int i = cur_node->idx[1];
  int j = cur_node->idx[2];
  double cur_height = cur_node->height;
  double cur_ele = cur_node->ele;

  int true_layer = layer;

  // Check current node's elevation indicator first
  if (cur_ele > 0.5) {
    // Current node indicates upward stairs
    int candidate = layer + 1;
    if (candidate < max_layers_) {
      const Node& check = grid_map_[candidate][i][j];
      if (std::abs(check.height - cur_height) <= 0.5) {
        return candidate;
      }
    }
  } else if (cur_ele < -0.5) {
    // Current node indicates downward stairs
    int candidate = layer - 1;
    if (candidate >= 0) {
      const Node& check = grid_map_[candidate][i][j];
      if (std::abs(check.height - cur_height) <= 0.5) {
        return candidate;
      }
    }
  }

  // If no clear indicator at current position, check nearby layers
  bool found_transition = false;

  // First pass: try with tight constraint (0.2m)
  for (const auto offset : search_layers_offset_) {
    int cur_layer = layer + offset;

    if (cur_layer < 0 || cur_layer >= max_layers_) {
      continue;
    }

    const Node& search_node = grid_map_[cur_layer][i][j];

    if (std::abs(search_node.height - cur_height) > 0.2) {
      continue;
    }

    if (search_node.ele > 0.5) {
      true_layer = std::min(cur_layer + 1, max_layers_ - 1);
      found_transition = true;
      break;
    } else if (search_node.ele < -0.5) {
      true_layer = std::max(cur_layer - 1, 0);
      found_transition = true;
      break;
    }
  }

  // Second pass: if no transition found and current layer has high cost, try with relaxed constraint
  if (!found_transition && grid_map_[layer][i][j].cost >= cost_threshold_) {
    for (const auto offset : search_layers_offset_) {
      int cur_layer = layer + offset;

      if (cur_layer < 0 || cur_layer >= max_layers_) {
        continue;
      }

      const Node& search_node = grid_map_[cur_layer][i][j];

      // Relaxed height check: 0.6m tolerance as fallback
      double height_diff = std::abs(search_node.height - cur_height);
      if (height_diff > 0.6) {
        continue;
      }

      // Prefer layers with lower cost
      if (search_node.cost < grid_map_[layer][i][j].cost) {
        int candidate_layer = cur_layer;

        if (search_node.ele > 0.5) {
          candidate_layer = std::min(cur_layer + 1, max_layers_ - 1);
        } else if (search_node.ele < -0.5) {
          candidate_layer = std::max(cur_layer - 1, 0);
        }

        // Safety check: prevent cliff drops - don't jump more than 2 layers at once
        int layer_jump = std::abs(candidate_layer - layer);
        if (layer_jump <= 2) {
          // Additional check: validate the target layer height
          const Node& candidate_check = grid_map_[candidate_layer][i][j];
          if (std::abs(candidate_check.height - cur_height) <= 0.6) {
            true_layer = candidate_layer;
            break;
          }
        }
      }
    }
  }

  return true_layer;
}

double Astar::CalculateStepCost(const Node* node1, const Node* node2) const {}

double Astar::GetHeuristic(const Node* node1, const Node* node2) const {
  double cost = 0.0;

  if (h_type_ == kEuclidean) {
    // l2 distance
    cost = (node1->idx - node2->idx).norm();
  } else if (h_type_ == kDiagonal) {
    // octile distance
    Eigen::Vector3i d = node1->idx - node2->idx;
    int dx = abs(d(0)), dy = abs(d(1)), dz = abs(d(2));
    int dmin = std::min(dx, std::min(dy, dz));
    int dmax = std::max(dx, std::max(dy, dz));
    int dmid = dx + dy + dz - dmin - dmax;
    double h =
        std::sqrt(3) * dmin + std::sqrt(2) * (dmid - dmin) + (dmax - dmid);
    cost = h;
  } else if (h_type_ == kManhattan) {
    cost = (node1->idx - node2->idx).lpNorm<1>();
  } else {
    assert(false && "not implemented");
  }

  // cost += std::abs(node1->idx[0] - node2->idx[0]) * 10;
  return cost;
}

std::vector<PathPoint> Astar::GetPathPoints() const {
  std::vector<PathPoint> path_points;

  auto size = search_result_.size();
  path_points.resize(size);

  if (size == 0) {
    printf("path is empty\n, convert to path points failed\n");
    return path_points;
  }

  for (size_t i = 0; i < size; ++i) {
    // path_points[i].layer = search_result_[i][0];
    // path_points[i].x = search_result_[i][2];
    // path_points[i].y = search_result_[i][1];
    // if (i > 0) {
    //   path_points[i].heading =
    //       std::atan2(search_result_[i][1] - search_result_[i - 1][1],
    //                  search_result_[i][2] - search_result_[i - 1][2]);
    // }
    path_points[i].layer = search_result_[i]->layer;
    path_points[i].x = search_result_[i]->idx(2);
    path_points[i].y = search_result_[i]->idx(1);
    path_points[i].height = search_result_[i]->height;
    if (i > 0) {
      path_points[i].heading =
          std::atan2(search_result_[i]->idx(1) - search_result_[i - 1]->idx(1),
                     search_result_[i]->idx(2) - search_result_[i - 1]->idx(2));
    }
  }

  if (size > 1) {
    path_points[0].heading = path_points[1].heading;
  }

  return path_points;
}

Eigen::MatrixXd Astar::GetResultMatrix() const {
  if (search_result_.empty()) {
    printf("path is empty\n, convert to matrix failed\n");
    return Eigen::MatrixXd();
  }

  Eigen::MatrixXd path_matrix(search_result_.size(), 3);
  for (size_t i = 0; i < search_result_.size(); ++i) {
    path_matrix(i, 0) = search_result_[i]->layer;
    path_matrix(i, 1) = search_result_[i]->idx[1];
    path_matrix(i, 2) = search_result_[i]->idx[2];
  }
  return path_matrix;
}

void Astar::ConvertClosedSetToMatrix(
    const std::unordered_map<int, Node*>& closed_set) {
  visited_set_ = Eigen::MatrixXi(closed_set.size(), 3);
  int count = 0;
  for (auto i = closed_set.begin(); i != closed_set.end(); ++i) {
    visited_set_(count, 0) = i->second->layer;
    visited_set_(count, 1) = i->second->idx[1];
    visited_set_(count, 2) = i->second->idx[2];
    count += 1;
  }
}

std::vector<Eigen::Vector3i> Astar::GetNeighbors(Node* node) const {}

Eigen::MatrixXd Astar::GetCostLayer(int layer) const {
  Eigen::MatrixXd cost_layer(max_y_, max_x_);
  for (int i = 0; i < max_y_; ++i) {
    for (int j = 0; j < max_x_; ++j) {
      cost_layer(i, j) = grid_map_[layer][i][j].cost;
    }
  }
  return cost_layer;
}
Eigen::MatrixXd Astar::GetEleLayer(int layer) const {
  Eigen::MatrixXd ele_layer(max_y_, max_x_);
  for (int i = 0; i < max_y_; ++i) {
    for (int j = 0; j < max_x_; ++j) {
      ele_layer(i, j) = grid_map_[layer][i][j].ele;
    }
  }
  return ele_layer;
}