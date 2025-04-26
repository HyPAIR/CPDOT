#include <iostream>
#include "formation_planner/IdentifyHomotopy.h"
#include "formation_planner/topo_prm.h"
#include "formation_planner/visualization/plot.h"

namespace formation_planner {
// Function to calculate the directed area
bool calculateSignedDistance(const std::vector<Point> pt_set) {
  double infeasibility;
  int num_robot = pt_set.size();
  for (int k = 0; k < num_robot; k++) {
    for (int p = 0; p < num_robot; p++) {
      if (p != k && (p + 1) % num_robot != k) {
        int Np = (p + 1) % num_robot;
        infeasibility = (pt_set[k].x - pt_set[p].x) * (pt_set[p].y - pt_set[Np].y) +
                          (pt_set[k].y - pt_set[p].y) * (pt_set[Np].x - pt_set[p].x);
        if (infeasibility <= 0){
          return false;
        } 
      }
    }
  }
  return true;
}

void generateBinaryCombinations(int n, int depth, std::vector<int>& current, std::vector<std::vector<int>>& result) {
    if (depth == n) {
        result.push_back(current);
        return;
    }

    // for each position, either 0 or 1
    for (int i = 0; i <= 1; ++i) {
        current.push_back(i); // select the current number
        generateBinaryCombinations(n, depth + 1, current, result); // retrive to fill the next position
        current.pop_back(); // back track, try the other number
    }
}

void generateCombinations(int k, int n, std::vector<std::vector<int>>& result) {
    if (k <= 0 || n < 0) return;
    if (n == 0) {
      std::vector<int> zeros(k, 0);
      result.push_back(zeros);
    }
    std::vector<int> current(k, 0);
    bool done = false;

    while (!done) {
        result.push_back(current);

        for (int i = 0; i < k; ++i) {
            if (current[i] < n) {
                ++current[i];
                break;
            } else {
                if (i == k - 1) {
                    done = true;  
                }
                current[i] = 0; 
            }
        }
    }
}

template<typename T>
std::vector<int> sortAndGetIndices(const std::vector<T>& nums) {
    std::vector<std::pair<T, int>> valIndices;

    for (int i = 0; i < nums.size(); ++i) {
        valIndices.emplace_back(nums[i], i);
    }

    std::sort(valIndices.begin(), valIndices.end(), 
              [](const std::pair<T, int>& a, const std::pair<T, int>& b) {
                  return a.first < b.first;
              });
    std::vector<int> indices;
    for (const auto& p : valIndices) {
        indices.push_back(p.second);
    }

    return indices;
}

double calPathLength(vector<Eigen::Vector2d> raw_path) {
  double length = 0.0;
  for (int j = 1; j < raw_path.size(); j++) {
    length += hypot(raw_path[j].x() - raw_path[j - 1].x(), raw_path[j].y() - raw_path[j - 1].y());
  }
  return length;
}
 
std::vector<int> evalPathLength(const vector<vector<Eigen::Vector2d>>& raw_paths, const int num_selected_path) {
    std::vector<double> path_length;
    for (int i = 0; i < raw_paths.size(); i++) {
      path_length.push_back(calPathLength(raw_paths[i]));
    }

    auto index_set = findSmallestIndices(path_length, num_selected_path);
    return index_set;
}

vector<Eigen::Vector2d> discretizePath(const vector<Eigen::Vector2d>& path, int pt_num) {
  vector<double> len_list;
  len_list.push_back(0.0);

  for (int i = 0; i < path.size() - 1; ++i) {
    double inc_l = (path[i + 1] - path[i]).norm();
    len_list.push_back(inc_l + len_list[i]);
  }

  // calc pt_num points along the path
  double len_total = len_list.back();
  double dl = len_total / double(pt_num - 1);
  double cur_l;

  vector<Eigen::Vector2d> dis_path;
  for (int i = 0; i < pt_num; ++i) {
    cur_l = double(i) * dl;

    // find the range cur_l in
    int idx = -1;
    for (int j = 0; j < len_list.size() - 1; ++j) {
      if (cur_l >= len_list[j] - 1e-4 && cur_l <= len_list[j + 1] + 1e-4) {
        idx = j;
        break;
      }
    }

    // find lambda and interpolate
    double lambda = (cur_l - len_list[idx]) / (len_list[idx + 1] - len_list[idx]);
    Eigen::Vector2d inter_pt = (1 - lambda) * path[idx] + lambda * path[idx + 1];
    dis_path.push_back(inter_pt);
  }

  return dis_path;
}

bool lineVisib(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, std::shared_ptr<formation_planner::Environment>& env) {
  // Eigen::Vector2d intersect_pt;
  Eigen::Vector2d q1, q2;
  for (int i = 0; i < env->polygons().size(); ++i) {
    if (lineIntersectsPolygon(p1, p2, env->polygons()[i], q1, q2)) {       
      return false;
    } 
  }
  return true;
}

bool ObstacleIntersect(const std::vector<Point>& poly_robot, const std::vector<Point>& poly_obs) {
// Eigen::Vector2d intersect_pt;
  // for (int i = 0; i < env->polygons().size(); ++i) {
  //   std::vector<Point> poly_obs;
  //     for (int j = 0; j < env->polygons()[i].points().size(); j++) {
  //       poly_obs.push_back({env->polygons()[i].points()[j].x(), env->polygons()[i].points()[j].y()});
  //     }
      if (isIntersecting(poly_robot, poly_obs)) { 
        return true;
      } 
  // }
  return false;
}

std::vector<int> findSmallestIndices(const std::vector<double>& nums, const int num_path) {
    std::vector<std::pair<double, int>> valIndices;

    for (int i = 0; i < nums.size(); ++i) {
        valIndices.emplace_back(nums[i], i);
    }

    std::sort(valIndices.begin(), valIndices.end(), 
              [](const std::pair<double, int>& a, const std::pair<double, int>& b) {
                  return a.first < b.first;
              });

    std::vector<int> indices;
    for (int i = 0; i < std::min(num_path, static_cast<int>(valIndices.size())); ++i) {
        indices.push_back(valIndices[i].second);
    }

    return indices;
}
int direction(Point p, Point q, Point r) {
    int val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (val == 0) return 0;  // colinear
    return (val > 0) ? 1 : 2;  // 1: clockwise, 2: counter-clockwise
}

bool onSegment(Point p, Point q, Point r) {
    if (r.x <= std::max(p.x, q.x) && r.x >= std::min(p.x, q.x) &&
        r.y <= std::max(p.y, q.y) && r.y >= std::min(p.y, q.y))
        return true;
    return false;
}

bool doIntersect(Point p1, Point q1, Point p2, Point q2) {
    int d1 = direction(p1, q1, p2);
    int d2 = direction(p1, q1, q2);
    int d3 = direction(p2, q2, p1);
    int d4 = direction(p2, q2, q1);

    if (d1 != d2 && d3 != d4)
        return true;

    if (d1 == 0 && onSegment(p1, q1, p2)) return true;
    if (d2 == 0 && onSegment(p1, q1, q2)) return true;
    if (d3 == 0 && onSegment(p2, q2, p1)) return true;
    if (d4 == 0 && onSegment(p2, q2, q1)) return true;

    return false;
}

// 检查两个多边形是否相交
bool isIntersecting(const std::vector<Point>& poly1, const std::vector<Point>& poly2) {
    int n = poly1.size();
    int m = poly2.size();
    for (int i = 0; i < n; i++) {
        int j = (i + 1) % n;
        for (int k = 0; k < m; k++) {
            int l = (k + 1) % m;
            if (doIntersect(poly1[i], poly1[j], poly2[k], poly2[l])) {
                return true;
            }
        }
    }
    return false;
}

std::vector<int> findTwoSmallest(const std::vector<double>& vec) {
    // double min1 = std::numeric_limits<double>::max(); 
    // double min2 = std::numeric_limits<double>::max();
    // index1 = -1;
    // index2 = -1;

    // for (int i = 0; i < vec.size(); ++i) {
    //     if (vec[i] < min1) {
    //         min2 = min1;
    //         index2 = index1;
    //         min1 = vec[i];
    //         index1 = i;
    //     } else if (vec[i] < min2 && vec[i] != min1) {
    //         min2 = vec[i];
    //         index2 = i;
    //     }
    // }
    std::vector<int> indices(vec.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(),
              [&vec](int i1, int i2) { return vec[i1] < vec[i2]; });

    return indices;
}

bool JudgeHomotopy(vector<vector<Pathset>> paths_set, vector<int> combineation, std::shared_ptr<formation_planner::Environment>& env) {
  int num_robot = combineation.size();
  for (int i = 0; i < combineation.size(); i++) {
    for (int j = 0; j < paths_set[i][combineation[i]].path.size(); j++) {
      Eigen::Vector2d p1(paths_set[(i + 1) % num_robot][combineation[(i + 1) % num_robot]].path[j].x(), paths_set[(i + 1) % num_robot][combineation[(i + 1) % num_robot]].path[j].y());
      Eigen::Vector2d p2(paths_set[i][combineation[i]].path[j].x(), paths_set[i][combineation[i]].path[j].y());
      if (lineVisib(p1, p2, env)) return false;
    }
  }
  return true;
}

bool BeyondHeightCons(vector<vector<Pathset>> paths_set, vector<int> combineation, std::shared_ptr<formation_planner::Environment>& env) {
  VVCM vvcm;
  for (int i = 0; i < paths_set[0][0].path.size(); i++) {
    std::vector<Point> poly_robot;
    for (int j = 0; j < combineation.size(); j++) {
      poly_robot.push_back({paths_set[j][combineation[j]].path[i].x(), paths_set[j][combineation[j]].path[i].y()});
    }
    for (int j = 0; j < env->polygons().size(); ++j) {
      std::vector<Point> poly_obs;
      for (int k = 0; k < env->polygons()[j].points().size(); k++) {
        poly_obs.push_back({env->polygons()[j].points()[k].x(), env->polygons()[j].points()[k].y()});
      }
      if (ObstacleIntersect(poly_robot, poly_obs)) {       
        if (env->heights()[j] > vvcm.zr)
          return true;
      } 
    }
  }
  return false;
}

bool areRobotsAdjacent(const std::vector<int>& nums) {
    std::unordered_map<int, std::vector<int>> positions;
    int n = nums.size();
    // 记录每个数字出现的位置
    for (int i = 0; i < n; ++i) {
        positions[nums[i]].push_back(i);
    }
    // 检查每个数字是否是连续的，或者形成闭环
    for (const auto& entry : positions) {
        const std::vector<int>& pos = entry.second;
        int size = pos.size();
        if (size == 1) continue;  // 只有一个出现的数字不需要检查
        bool isAdjacent = true;
        for (int i = 1; i < size; ++i) {
            if (pos[i] != pos[i - 1] + 1) {
                isAdjacent = false;
                break;
            }
        }
        // 如果数字不连续，检查是否形成闭环
        if (!isAdjacent) {
            if (pos.front() == 0 && pos.back() == n - 1) {
                for (int i = 1; i < size - 1; ++i) {
                    if (pos[i] != pos[i - 1] + 1) {
                        return false;
                    }
                }
            } else {
                return false;
            }
        }
    }

    return true;
}

bool BeyondInterdisCons(const std::vector<std::vector<formation_planner::Pathset>>&paths_sets, const std::vector<int>& combination) {
  for (int i = 0; i < paths_sets[0][0].path.size(); i++) {
    for (int j = 0; j < combination.size(); j++) {
      double point1_x = paths_sets[j][combination[j]].path[i].x();
      double point1_y = paths_sets[j][combination[j]].path[i].y();
      double point2_x = paths_sets[(j + 1) % combination.size()][combination[j]].path[i].x();
      double point2_y = paths_sets[(j + 1) % combination.size()][combination[j]].path[i].y();
      double inter_dis = hypot(point1_x -point1_x, point1_y - point2_y);
      if (inter_dis > 3) {
        return true;
      }
    }
  }
  return true;
}

std::vector<Eigen::Vector2d> RewiretPath(const std::vector<Eigen::Vector2d>& path, const Eigen::Vector2d& point1, const Eigen::Vector2d& point2,
std::shared_ptr<formation_planner::Environment>& env) {
    int n = path.size();
    int startIndex = 0;
    int endIndex = n - 1;
    int i;
    for (i = 0; i < n; ++i) {
        if (!lineVisib(path[i], point1, env)) {
            startIndex = (i - 1) >= 0 ? (i - 1) : 0;
            break;
        }
    }
    int j;
    for (j = n - 1; j >= 0; --j) {
        if (!lineVisib(path[j], point2, env)) {
            endIndex = (j + 1) <= n - 1 ? (j + 1) : n - 1;
            break;
        }
    }
    if (startIndex >= endIndex) {
      std::vector<Eigen::Vector2d> newPath;
      newPath.push_back(point1);

      for (int k = endIndex; k < n; ++k) {
          newPath.push_back(path[k]);
      }
      newPath.push_back(point2);
      return newPath;
    }

    std::vector<Eigen::Vector2d> newPath;
    newPath.push_back(point1);

    for (int k = startIndex; k <= endIndex; ++k) {
        newPath.push_back(path[k]);
    }

    newPath.push_back(point2);
    return newPath;
}

double CalLengthSet(const std::vector<std::vector<Pathset>>&paths_sets, const std::vector<int>& combination) {
  double path_length = 0.0;
  for (int i = 0; i < combination.size(); i++) {
    path_length += paths_sets[i][combination[i]].path_length;
  }
  return path_length / paths_sets.size();
}
double CalTurningSet(const std::vector<std::vector<Pathset>>&paths_sets, const std::vector<int>& combination){
  int turnCount = 0;
  for (int i = 1; i < paths_sets[0][0].path.size() - 1; i++) {
    for (int j = 0; j < combination.size(); j++) {
      // 计算第一个方向向量并归一化
      int dir1_x = paths_sets[j][combination[j]].path[i].x() - paths_sets[j][combination[j]].path[i - 1].x();
      int dir1_y = paths_sets[j][combination[j]].path[i].y() - paths_sets[j][combination[j]].path[i - 1].y();
      double len1 = std::sqrt(dir1_x * dir1_x + dir1_y * dir1_y);
      double norm_dir1_x = dir1_x / len1;
      double norm_dir1_y = dir1_y / len1;

      // 计算第二个方向向量并归一化
      int dir2_x = paths_sets[j][combination[j]].path[i + 1].x() - paths_sets[j][combination[j]].path[i].x();
      int dir2_y = paths_sets[j][combination[j]].path[i + 1].y() - paths_sets[j][combination[j]].path[i].y();
      double len2 = std::sqrt(dir2_x * dir2_x + dir2_y * dir2_y);
      double norm_dir2_x = dir2_x / len2;
      double norm_dir2_y = dir2_y / len2;

      // 比较两个归一化后的方向向量，如果不相等，则计数加1
      if (norm_dir1_x != norm_dir2_x || norm_dir1_y != norm_dir2_y) {
          ++turnCount;
      }
    }
  }
  return turnCount / combination.size();
}

double CalHomotopySet(const std::vector<int>& combination){
  if (combination.empty()) {
      return 1;  
  }
  int firstElement = combination[0];
  for (size_t i = 1; i < combination.size(); ++i) {
      if (combination[i] != firstElement) {
          return 0;  
      }
  }
  return 1; 
}
double CalSafetySet(const std::vector<std::vector<Pathset>>&paths_sets, const std::vector<int>& combination){
  int topolpgy_vio = 0;
  for (int i = 0; i < paths_sets[0][0].path.size(); i++) {
    vector<Point> point_set;
    for (int j = 0; j < combination.size(); j++) {
      point_set.push_back(Point{paths_sets[j][combination[j]].path[i].x(), paths_sets[j][combination[j]].path[i].y()});
    }
    if (!calculateSignedDistance(point_set)) topolpgy_vio++;
  }
  return topolpgy_vio;
}

void CalCombination(
    const vector<vector<vector<Eigen::Vector2d>>>& raw_paths_set, 
    std::shared_ptr<formation_planner::Environment>& env, 
    vector<vector<Pathset>>& paths_sets,
    std::vector<std::vector<int>>& combinations_new,
    double& filter_sort_time) {
    int min_path_num = 1e4;
    for (int i = 0; i < raw_paths_set.size(); i++) {
      if (raw_paths_set[i].size() < min_path_num) {
        min_path_num = raw_paths_set[i].size();
      }
    }
    int num_selected_path = std::min(8, min_path_num);
    // paths_sets(raw_paths_set.size());
    vector<double> path_length_set;
    std::vector<std::vector<int>> combinations;
    std::vector<int> current;
    vector<double> topolpgy_vio_set;
    vector<double> homotopy_set;
    vector<double> turn_num_set;
    /* Heuristics Filtering*/
    std::vector<std::vector<int>> path_index_set(raw_paths_set.size());
    for (int ind = 0; ind < raw_paths_set.size(); ind++) {
        auto index_set = evalPathLength(raw_paths_set[ind], num_selected_path);
        for (int j = 0; j < num_selected_path; j++) {
          Pathset path{calPathLength(raw_paths_set[ind][index_set[j]]), discretizePath(raw_paths_set[ind][index_set[j]], 100)};
          paths_sets[ind].push_back(path);
        }
    }
    std::cout << "[Homotopy]: total num: " << num_selected_path * paths_sets.size();
    std::cout << std::endl;
    generateCombinations(raw_paths_set.size(), num_selected_path - 1, combinations);
    int num_before_filter = combinations.size();
    ros::Time t1;
    t1 = ros::Time::now();
    for (int i = 0; i < combinations.size(); i++) {
      // height constraints violated
      if (BeyondHeightCons(paths_sets, combinations[i], env)) {
        combinations.erase(combinations.begin() + i);
        continue;
      }
      // homotopy constraints violated
      // if (!areRobotsAdjacent(combinations[i])) {
      //   combinations.erase(combinations.begin() + i);
      //   continue;
      // }
      // inter-distance constraints violated
      if (BeyondInterdisCons(paths_sets, combinations[i])) {
        combinations.erase(combinations.begin() + i);
        continue;       
      }
    } 
    int num_after_filter = combinations.size();
    std::cout << num_before_filter - num_after_filter << " combinations are removed due to some constraints!";
    std::cout << std::endl;
    /* Heuristics Sorting*/
    for (auto combination : combinations) {
      topolpgy_vio_set.push_back(CalSafetySet(paths_sets, combination));
      path_length_set.push_back(CalLengthSet(paths_sets, combination));
      homotopy_set.push_back(CalHomotopySet(combination));
      // turn_num_set.push_back(CalTurningSet(paths_sets, combination));
    }
    vector<vector<double>> cost_array = {topolpgy_vio_set, path_length_set, homotopy_set};
    std::vector<double> cost_set = calculateCost(cost_array);
    filter_sort_time = (ros::Time::now() - t1).toSec();
    std::cout << "Total filter & sort time: " << filter_sort_time << std::endl;
    // order by the cost function
    std::vector<int> combinations_sorted_ind = sortAndGetIndices(cost_set);
    for (int i = 0; i < combinations_sorted_ind.size(); i++) {
      // if (topolpgy_vio_set[i] < 20) {
        combinations_new.push_back(combinations[combinations_sorted_ind[combinations_sorted_ind[i]]]);
        // path_length_set_new.push_back(path_length_set[i]);
      // }
    }
}
void CalCorridors(
  const vector<vector<Pathset>> paths_sets,
  const std::vector<int> combinations_new, 
  std::shared_ptr<formation_planner::Environment> env,
  const std::vector<math::Polygon2d>& obstacles,
  const double& bbox_width,
  std::vector<std::vector<std::vector<std::vector<double>>>>& hyperparam_sets,
  std::vector<std::vector<std::vector<math::Vec2d>>>& poly_vertices_sets
  ) {
    std::vector<std::vector<double>> corridor_sets(paths_sets[0][0].path.size(), std::vector<double>(4, 0));
    std::vector<formation_planner::math::Pose> topo_path;
    std::cout << "combination: ";
    for (int i = 0; i < combinations_new.size(); i++) {
      std::cout << combinations_new[i] << ", ";
    }
    std::cout << std::endl;
    std::vector<Eigen::MatrixXd> hPolys;
    vec_Vec2f obs_pc;
    EllipsoidDecomp2D decomp_util;
    vec_E<Polyhedron2D> decompPolys;
    // keyPts.clear();
    obs_pc.push_back(Vec2f(60, 60));
    obs_pc.push_back(Vec2f(-60, 60));
    obs_pc.push_back(Vec2f(-60, -60));
    obs_pc.push_back(Vec2f(60, -60));

    for (const auto& polygon : obstacles) {
        const auto& points = polygon.points();
        int num_points = points.size();
        for (int j = 0; j < num_points; ++j) {
          Eigen::Vector2d start = {points[j].x(), points[j].y()};
          Eigen::Vector2d end = {points[(j+1)%num_points].x(), points[(j+1)%num_points].y()};
          std::vector<Eigen::Vector2d> interpolatedPoints = env->interpolatePoints(start, end, 20);
          for (int i = 0; i < interpolatedPoints.size(); i++) {
            obs_pc.push_back(Vec2f(interpolatedPoints[i](0), interpolatedPoints[i](1)));
          }
        }
    }
    for (int i = 0; i < combinations_new.size(); i++) {
      std::vector<std::vector<std::vector<double>>> hyperparam_set;
      std::vector<std::vector<math::Vec2d>> poly_vertices_set;
      for (int j = 0; j < paths_sets[i][0].path.size(); j++) {
          decomp_util.set_local_bbox(Eigen::Vector2d(bbox_width, bbox_width));
          // Generate corridor for each waypoint
          int next_idx = j + 1; // Next waypoint
          if (paths_sets[i][combinations_new[i]].path[j].x() == paths_sets[i][combinations_new[i]].path[next_idx].x() && paths_sets[i][combinations_new[i]].path[j].y() == paths_sets[i][combinations_new[i]].path[next_idx].y() && !poly_vertices_set.empty()) {
            poly_vertices_set.push_back(poly_vertices_set.back());
            decompPolys.push_back(decompPolys.back());
            continue;
          }
          vec_Vec2f line;
          Eigen::Vector2d path_line = {paths_sets[i][combinations_new[i]].path[j].x(), paths_sets[i][combinations_new[i]].path[j].y()};
          Eigen::Vector2d path_line_next = {paths_sets[i][combinations_new[i]].path[next_idx].x(), paths_sets[i][combinations_new[i]].path[next_idx].y()};
          line.push_back(path_line);
          if (j == paths_sets[i][0].path.size() -1) {
            line.push_back({paths_sets[i][combinations_new[i]].path[j].x() + 0.1, paths_sets[i][combinations_new[i]].path[j].y() + 0.1});
          }
          else {
            line.push_back(path_line_next);
          }
          // keyPts.emplace_back(path[idx], path[next_idx]);

          decomp_util.set_obs(obs_pc);
          decomp_util.dilate(line);

          Polyhedron2D poly = decomp_util.get_polyhedrons()[0];     
          const auto vertices = cal_vertices(poly);
          std::vector<math::Vec2d> poly_vertices;
          for (size_t i = 0; i < vertices.size(); i++) {
              math::Vec2d vertice(vertices[i](0), vertices[i](1));
              poly_vertices.push_back(vertice);
          }
          decompPolys.push_back(poly);
          poly_vertices_set.push_back(poly_vertices);
      }      
      hPolys.clear();
      Eigen::MatrixXd current_poly;
      for (uint i = 0; i < decompPolys.size(); i++) {
          vec_E<Hyperplane2D> current_hyperplanes = decompPolys[i].hyperplanes();
          current_poly.resize(4, current_hyperplanes.size());
          for (uint j = 0; j < current_hyperplanes.size(); j++) {
              current_poly.col(j) << current_hyperplanes[j].n_, current_hyperplanes[j].p_;
          }
          hPolys.push_back(current_poly);
      }
      for (int i = 0; i < hPolys.size(); i++) {
        std::vector<std::vector<double>> hyperparam;
        std::vector<double> test_co;
        for (int j = 0; j < hPolys[i].cols(); j++) {
          Eigen::Vector2d a = hPolys[i].col(j).head(2); 
          Eigen::Vector2d p = hPolys[i].col(j).tail(2);     
          double b = a.dot(p);   
          hyperparam.push_back({a(0), a(1), b});      
          // test_co.push_back(a(0) * path[i](0) + a(1) * path[i](1) - b); 
        }
        hyperparam_set.push_back(hyperparam);
      }
      hyperparam_sets.push_back(hyperparam_set);
      poly_vertices_sets.push_back(poly_vertices_set);
    }
    std::cout << "\n";
  }
}