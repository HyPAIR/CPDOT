/**
 * file topo_prm.h
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief topological PRM
 * data 2024-4-2
 * 
 * @copyright Copyroght(c) 2024
*/

#ifndef _IDENTIFYHOMOTOPY_H
#define _IDENTIFYHOMOTOPY_H

#include <random>
#include <Eigen/Eigen>
#include <list>
#include <vector>
// #include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <cmath>
#include <utility>
#include <unordered_map>
#include <memory>
#include "formation_planner/environment.h"
#include "formation_planner/math/pose.h"
#include "formation_planner/planner_config.h"

using std::vector;
using std::list;
using std::max;
using std::shared_ptr;
using std::random_device;
using std::default_random_engine;
using std::uniform_real_distribution;

namespace formation_planner {
template<typename T>
class Normalizer {
public:
    std::vector<double> normalize(const std::vector<T>& arr) {
        std::vector<double> normalizedArr(arr.size());

        T maxVal = *std::max_element(arr.begin(), arr.end());
        T minVal = *std::min_element(arr.begin(), arr.end());
        T range = maxVal - minVal;

        if (range == 0) {
            std::fill(normalizedArr.begin(), normalizedArr.end(), 0.0);
        } else {
            for (size_t i = 0; i < arr.size(); ++i) {
                normalizedArr[i] = static_cast<double>(arr[i] - minVal) / static_cast<double>(range);
            }
        }

        return normalizedArr;
    }
};

template<typename T>
std::vector<double> calculateCost(const std::vector<std::vector<T>>& arrays) {
    if (arrays.empty()) return {};
    
    size_t length = arrays[0].size();
    std::vector<double> cost(length, 0.0);

    Normalizer<T> normalizer;  

    for (const auto& arr : arrays) {
        std::vector<double> normalizedArr = normalizer.normalize(arr);

        for (size_t i = 0; i < length; ++i) {
            cost[i] += normalizedArr[i];
        }
    }

    return cost;
}
struct Point {
    double x, y;
    Point(double x_val, double y_val) : x(x_val), y(y_val) {}
};
struct Pathset {
    double path_length;
    vector<Eigen::Vector2d> path;
};
void CalCombination(
    const vector<vector<vector<Eigen::Vector2d>>>& raw_paths_set, 
    std::shared_ptr<formation_planner::Environment>& env, 
    std::vector<vector<Pathset>>& paths_sets, 
    std::vector<std::vector<int>>& combinations_new,
    double& filter_sort_time);
void CalCorridors(
    const vector<vector<Pathset>> paths_sets,
    const std::vector<int> combinations_new, 
    std::shared_ptr<formation_planner::Environment> env,
    const std::vector<math::Polygon2d>& obstacles,
    const double& bbox_width,
    std::vector<std::vector<std::vector<std::vector<double>>>>& hyperparam_sets,
    std::vector<std::vector<std::vector<math::Vec2d>>>& poly_vertices_sets);
std::vector<int> findTwoSmallest(const std::vector<double>& vec);
std::vector<int> findSmallestIndices(const std::vector<double>& nums, const int num_path);
std::vector<int> evalPathLength(const vector<vector<Eigen::Vector2d>>& raw_paths);
double cross(const Eigen::Vector2d& p0, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);
std::vector<std::vector<int>> generateCombinations(int k, int n);
bool onSegment(const Eigen::Vector2d& p, const Eigen::Vector2d& q, const Eigen::Vector2d &r);
bool segmentsIntersect(const Eigen::Vector2d &p1, const Eigen::Vector2d &q1, const Eigen::Vector2d &p2, const Eigen::Vector2d &q2);
bool lineIntersectsPolygon(const Eigen::Vector2d &a, const Eigen::Vector2d &b, const math::Polygon2d &polygon, Eigen::Vector2d &p1, Eigen::Vector2d &p2);
bool lineVisib(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, std::shared_ptr<formation_planner::Environment>& env);
bool ObstacleIntersect(const std::vector<Point>& poly_robot, const std::vector<Point>& poly_obs);
bool isIntersecting(const std::vector<Point>& poly1, const std::vector<Point>& poly2);
bool areRobotsAdjacent(const std::vector<int>& nums);
bool BeyondInterdisCons(const std::vector<std::vector<formation_planner::Pathset>>&paths_sets, const std::vector<int>& combination);
double CalLengthSet(const std::vector<std::vector<formation_planner::Pathset>>&paths_sets, const std::vector<int>& combination);
double CalTurningSet(const std::vector<std::vector<formation_planner::Pathset>>&paths_sets, const std::vector<int>& combination);
double CalHomotopySet(const std::vector<int>& combination);
double CalSafetySet(const std::vector<std::vector<formation_planner::Pathset>>&paths_sets, const std::vector<int>& combination);
std::vector<Eigen::Vector2d> RewiretPath(const std::vector<Eigen::Vector2d>& path, const Eigen::Vector2d& point1, const Eigen::Vector2d& point2,
std::shared_ptr<formation_planner::Environment>& env);
}  // namespace fast_planner

#endif