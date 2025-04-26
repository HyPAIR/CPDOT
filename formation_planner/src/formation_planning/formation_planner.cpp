/**
 * file formation_planner.cpp
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief heterogeneous formation planning
 * data 2023-11-22
 * 
 * @copyright Copyroght(c) 2023
*/
#include <ros/ros.h>
#include <ros/package.h> 
#include "formation_planner/time.h"
#include "formation_planner/formation_planner.h"
#include "formation_planner/lightweight_nlp_problem.h"
#include "formation_planner/math/math_utils.h"
#include "formation_planner/visualization/plot.h"
#include "formation_planner/yaml_all.h"
#include "formation_planner/IdentifyHomotopy.h"
#include "formation_planner/forward_kinematics.h"
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>
using namespace forward_kinematics;
namespace formation_planner {
struct Data {
    double x;
    double y;
    double theta;
};
// void operator>>(const YAML::Node& node, Data& data) {
//     if (node["x"] && node["y"] && node["theta"]) {
//         data.x = node["x"].as<double>();
//         data.y = node["y"].as<double>();
//         data.theta = node["theta"].as<double>();
//     } else {
//         throw std::runtime_error("Invalid node; missing one of 'x', 'y', or 'theta'");
//     }
// }

// 计算对称归一化拉普拉斯矩阵
Eigen::MatrixXd computeNormalizedLaplacian(const Eigen::MatrixXd& adjMatrix, const Eigen::MatrixXd& degreeMatrix) {
    Eigen::MatrixXd D_inv_sqrt = Eigen::MatrixXd::Zero(degreeMatrix.rows(), degreeMatrix.cols());

    // 计算度矩阵的平方根逆，只对非零元素操作
    for (int i = 0; i < degreeMatrix.rows(); ++i) {
        if (degreeMatrix(i, i) > 0) {
            D_inv_sqrt(i, i) = 1.0 / std::sqrt(degreeMatrix(i, i));
        } else {
            D_inv_sqrt(i, i) = 0;  // 对于度为零的情况，我们设置为零
        }
    }
    // 计算拉普拉斯矩阵
    Eigen::MatrixXd L = degreeMatrix - adjMatrix;
    // 计算归一化拉普拉斯矩阵
    Eigen::MatrixXd normalizedLaplacian = D_inv_sqrt * L * D_inv_sqrt;
    return normalizedLaplacian;
}

// 计算平均值
double calculateAverage(const std::vector<double>& data) {
    if (data.empty()) {
        return 0.0;
    }
    double sum = std::accumulate(data.begin(), data.end(), 0.0);
    return sum / data.size();
}

// 计算标准差
double calculateStandardDeviation(const std::vector<double>& data) {
    if (data.size() < 2) {
        return 0.0;
    }
    double mean = calculateAverage(data);

    // 使用 lambda 表达式来计算方差
    double variance = std::accumulate(data.begin(), data.end(), 0.0, 
                                      [mean](double accumulator, double value) {
                                          return accumulator + (value - mean) * (value - mean);
                                      }) / (data.size() - 1);  // 样本标准差

    return std::sqrt(variance);
}

// 计算Frobenius范数
double computeFrobeniusNorm(const Eigen::MatrixXd& matrix) {
    return std::sqrt((matrix.array() * matrix.array()).sum());
}

// 计算相似度度量f
double computeSimilarityMetric(const Eigen::MatrixXd& L_current, const Eigen::MatrixXd& L_des) {
    Eigen::MatrixXd diff = L_current - L_des;
    return computeFrobeniusNorm(diff);
}

void computeFormationSim(const std::vector<FullStates> &result, double& e_max, double& e_avg) {
    VVCM vvcm;
    int N = num_robot;  // 顶点数
    double edge_length = hypot(result[1].states[0].x - result[0].states[0].x, result[1].states[0].y - result[0].states[0].y);  // 相邻顶点之间的期望距离
    // 初始化期望的邻接矩阵
    Eigen::MatrixXd adjMatrix_des = Eigen::MatrixXd::Zero(N, N);
    for (int i = 0; i < N; ++i) {
        int next = (i + 1) % N;  // 下一个顶点
        int prev = (i - 1 + N) % N;  // 上一个顶点

        adjMatrix_des(i, next) = edge_length;
        adjMatrix_des(i, prev) = edge_length;
    }

    // 构建期望的度矩阵
    Eigen::MatrixXd degreeMatrix_des = Eigen::MatrixXd::Zero(N, N);
    for (int i = 0; i < N; ++i) {
        degreeMatrix_des(i, i) = adjMatrix_des.row(i).sum();
    }
    Eigen::MatrixXd L_des = computeNormalizedLaplacian(adjMatrix_des, degreeMatrix_des);
    double f_sum = 0.0;
    double f_max = 0.0;
    for (int traj_ind = 0; traj_ind < result[0].states.size(); traj_ind++) {
      // 初始化现在的邻接矩阵
      Eigen::MatrixXd adjMatrix_current = Eigen::MatrixXd::Zero(N, N);
      for (int i = 0; i < N; ++i) {
          int next = (i + 1) % N;  // 下一个顶点
          int prev = (i - 1 + N) % N;  // 上一个顶点

          adjMatrix_current(i, next) = hypot(result[i].states[traj_ind].x - result[next].states[traj_ind].x, result[i].states[traj_ind].y - result[next].states[traj_ind].y);
          adjMatrix_current(i, prev) = hypot(result[i].states[traj_ind].x - result[prev].states[traj_ind].x, result[i].states[traj_ind].y - result[prev].states[traj_ind].y);
      }

      // 构建现在的度矩阵
      Eigen::MatrixXd degreeMatrix_current = Eigen::MatrixXd::Zero(N, N);
      for (int i = 0; i < N; ++i) {
          degreeMatrix_current(i, i) = adjMatrix_current.row(i).sum();
      }
      // 计算期望的归一化拉普拉斯矩阵 L_des
      // 计算当前的归一化拉普拉斯矩阵 L_current
      Eigen::MatrixXd L_current = computeNormalizedLaplacian(adjMatrix_current, degreeMatrix_current);

      // 计算相似度度量f
      double f = computeSimilarityMetric(L_current, L_des);
      f_sum += f;
      if (f_max < f) {
        f_max = f;
      }
    }
    e_avg = f_sum / result[0].states.size();
    e_max = f_max;
}

void DrawTrajectoryRviz(const FullStates sol_traj,std::shared_ptr<formation_planner::PlannerConfig> config,
      int robot_index, ros::Publisher path_pub) {
  vector<formation_planner::visualization::Color> colors = {
    visualization::Color::Red,
    visualization::Color::Green,
    visualization::Color::Blue,
    visualization::Color::Cyan,
    visualization::Color::Magenta,
    visualization::Color::Grey,
    visualization::Color::Yellow,
    visualization::Color::Black
  };
  const std::string robot_name = "Footprint_" + std::to_string(robot_index);
  for(int i = 0; i < 1e3; i++) {
    visualization::Delete(i, robot_name);
  }
  visualization::Trigger();

  nav_msgs::Path msg;
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();
  for(size_t i = 0; i < sol_traj.states.size(); i++) {
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.pose.position.x = sol_traj.states[i].x;
    pose.pose.position.y = sol_traj.states[i].y;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(sol_traj.states[i].theta);
    msg.poses.push_back(pose);

    auto box = config->vehicle.GenerateBox(sol_traj.states[i].pose());
    auto color = colors[robot_index];
    // color.set_alpha(0.4);
    color.set_alpha(0.2);
    // color.set_alpha(0.4);
    visualization::PlotPolygon(math::Polygon2d(box), 0.1, color, i, robot_name);
  }
  path_pub.publish(msg);
  visualization::Trigger();
}

void generateRegularPolygon(const double r, const int k, 
  std::vector<std::vector<double>>& vertice_set) {
    double cx = 0.0, cy = 0.0;
    std::vector<std::pair<double, double>> vertices;
    double angleIncrement = 2 * M_PI / k;
    for (int i = 0; i < k; ++i) {
        TrajectoryPoint tp_temp;
        double angle = i * angleIncrement;
        double x = cx + r * std::cos(angle);
        double y = cy + r * std::sin(angle);
        tp_temp.x = x;
        tp_temp.y = y;
        tp_temp.theta = 0.0;
        vertice_set.push_back({x, y});
    }
}

FormationPlanner::FormationPlanner(std::shared_ptr<PlannerConfig> config, std::shared_ptr<Environment> env)
  : config_(config), env_(env), coarse_path_planner_(config, env) {
  problem_ = std::make_shared<LightweightProblem>(config_, env_);
}

FullStates FormationPlanner::GenerateGuessFromPath(const std::vector<math::Pose> &path, const TrajectoryPoint &start, const int step_num, bool ratio) {
  ROS_ASSERT_MSG(!path.empty(), "global path empty");

  size_t closest_index = 0;
  double closest_dist = path.front().DistanceTo(start.position());
  for(size_t i = 1; i < path.size(); i++) {
    double dist = path[i].DistanceTo(start.position());
    if(dist < closest_dist) {
      closest_dist = dist;
      closest_index = i;
    }
  }
  std::vector<math::Pose> pruned_path;
  std::copy(std::next(path.begin(), closest_index), path.end(), std::back_inserter(pruned_path));
  return ResamplePath(pruned_path, step_num, ratio);
}

FullStates FormationPlanner::StitchPreviousSolution(const FullStates &solution, const TrajectoryPoint &start) {
  FullStates result;
  if(solution.states.empty()) {
    return result;
  }

  size_t closest_index = 0;
  double closest_dist = hypot(solution.states.front().x - start.x, solution.states.front().y - start.y);
  for(size_t i = 1; i < solution.states.size(); i++) {
    double dist = hypot(solution.states[i].x - start.x, solution.states[i].y - start.y);
    if(dist < closest_dist) {
      closest_dist = dist;
      closest_index = i;
    }
  }

  double dt = solution.tf / solution.states.size();
  result.tf = solution.tf - closest_index * dt;
  std::copy(std::next(solution.states.begin(), closest_index), solution.states.end(), std::back_inserter(result.states));
  return result;
}

bool FormationPlanner::CheckGuessFeasibility(const FullStates &guess) {
  if(guess.states.empty()) {
    return false;
  }

  for(auto &state: guess.states) {
    if (env_->CheckPoseCollision(0.0, state.pose())) {
      return false;
    }
  }
  return true;
}

bool FormationPlanner::Plan(const FullStates &prev_sol, const TrajectoryPoint &start, const TrajectoryPoint &goal, 
      iris::IRISProblem &iris_problem, FullStates &result, double& coarse_time, double& solve_time, bool show_cr, 
      const std::vector<std::vector<std::vector<double>>>& hyperparam_set) {
  int ind = int(goal.y);
  ind = 11;
  // if (show_cr) {
  //   std::string file_name;
  //   if (goal.x == 0.0)
  //     file_name = "/home/weijian/mir_robot_test/src/match_mobile_robotics/data"+std::to_string(ind)+".yaml";
  //   else if (goal.x == 1.0)
  //     file_name = "/home/weijian/mir_robot_test/src/match_mobile_robotics/data"+std::to_string(ind)+"_1.yaml";
  //   else if (goal.x == 2.0)
  //     file_name = "/home/weijian/mir_robot_test/src/match_mobile_robotics/data"+std::to_string(ind)+"_2.yaml";
  //     YAML::Node config = YAML::LoadFile(file_name);
  //     std::vector<Data> data(config.size());
  //   try {
  //       // 读取 YAML 文件
  //       // 遍历文件中的每个数据条目
  //       for (std::size_t i = 0; i < config.size(); ++i) {
  //           YAML::Node sublist = config[i];
  //           if (sublist.IsSequence()) {
  //               for (std::size_t j = 0; j < sublist.size(); ++j) {
  //                   YAML::Node dataNode = sublist[j];
  //                   if (dataNode.IsMap()) {
  //                       dataNode >> data[i];

  //                       // 输出读取的数据
  //                   } else {
  //                       throw std::runtime_error("Invalid data node");
  //                   }
  //               }
  //           } else {
  //               throw std::runtime_error("Invalid sublist node");
  //           }
  //       }
  //   } catch (const YAML::Exception &e) {
  //       std::cerr << "YAML Exception: " << e.what() << std::endl;
  //   } catch (const std::exception &e) {
  //       std::cerr << "Exception: " << e.what() << std::endl;
  //   }
  //   FullStates guess;
  //   guess.states.resize(data.size());
  //   for (int i  = 0; i < data.size(); i++) {
  //     guess.states[i].x = data[i].x;
  //     guess.states[i].y = data[i].y;  
  //     guess.states[i].theta = data[i].theta;  
  //   }
  //   guess.tf = data.size() * 0.1 / (1.5);
  //   result = guess;
  //   return true;
  // }
  FullStates guess = StitchPreviousSolution(prev_sol, start);
  if(!CheckGuessFeasibility(guess)) {
    std::vector<math::Pose> initial_path;
    double st = GetCurrentTimestamp();
    if(!coarse_path_planner_.Plan(start.pose(), goal.pose(), initial_path, hyperparam_set)) {
      ROS_ERROR("re-plan coarse path failed!");
      return false;
    }
    guess = GenerateGuessFromPath(initial_path, start, 0, false);
    coarse_time = GetCurrentTimestamp() - st;
    ROS_INFO("coarse path generation time: %f", coarse_time);
    std::vector<double> xs, ys;
    int ind = 0;
    for(auto &pose: initial_path) {
      // xs.push_back(pose.x()); ys.push_back(pose.y());
      auto box = config_->vehicle.GenerateBox(pose);
      auto color = visualization::Color::Red;
      // color.set_alpha(0.4);
      visualization::PlotPolygon(math::Polygon2d(box), 0.2, color, ind++, "coarse"+std::to_string(ind));
    }
    // visualization::Plot(xs, ys, 0.1, visualization::Color::Yellow, 1, "Coarse Path");
    visualization::Trigger();
  }

  Constraints constraints;
  constraints.start = start;
  constraints.goal = goal;

  // int disc_nvar = config_->vehicle.n_disc * 2;
  int vertices_nvar = config_-> vehicle.vertices * 2;
  constraints.corridor_lb.setConstant(guess.states.size(), vertices_nvar, -inf);
  constraints.corridor_ub.setConstant(guess.states.size(), vertices_nvar, inf);
  iris::IRISOptions options;
  for(size_t i = 0; i < guess.states.size(); i++) {
    // auto disc_pos = config_->vehicle.GetDiscPositions(guess.states[i].x, guess.states[i].y, guess.states[i].theta);

    // for(int j = 0; j < config_->vehicle.n_disc; j++) {
    //   math::AABox2d box;
    //   if (!env_->GenerateCorridorBox(0.0, disc_pos[j*2], disc_pos[j*2+1], config_->vehicle.disc_radius, box)) {
    //     ROS_ERROR("%d th corridor box indexed at %zu generation failed!", j, i);
    //     return false;
    //   }

    //   // iris_problem.setSeedPoint(Eigen::Vector2d(disc_pos[j*2], disc_pos[j*2+1]));
    //   // iris::IRISRegion region = inflate_region(iris_problem, options);
    //   // auto points = region.polyhedron.generatorPoints();
    //   // std::vector<math::Vec2d> points_;
    //   // for (const auto& pts : points) {
    //   //   math::Vec2d pts_temp(pts[0], pts[1]);
    //   //   points_.push_back(pts_temp);
    //   // }

    //   auto color = visualization::Color::Green;
    //   color.set_alpha(0.1);
    //   visualization::PlotPolygon(math::Polygon2d(math::Box2d(box)), 0.05, color, i, "Corridor " + std::to_string(j));
    //   // visualization::PlotPolygon(math::Polygon2d({points_}), 0.05, color, i, "Convex Region" + std::to_string(j));

    //   constraints.corridor_lb(i, j*2) = box.min_x();
    //   constraints.corridor_lb(i, j*2+1) = box.min_y();
    //   constraints.corridor_ub(i, j*2) = box.max_x();
    //   constraints.corridor_ub(i, j*2+1) = box.max_y();
    // }
    // auto disc_pos = config_->vehicle.GetDiscPositions(guess.states[i].x, guess.states[i].y, guess.states[i].theta);
    auto f_centre = config_->vehicle.GetFormationCentre(guess.states[i].x, guess.states[i].y, guess.states[i].theta);

    math::AABox2d box;
    if (!env_->GenerateCorridorBox(0.0, f_centre[0], f_centre[1], 0.3, box)) {
      // if (!env_->GenerateCorridorBox(0.0, disc_pos[j*2], disc_pos[j*2+1], config_->vehicle.disc_radius, box)) {
      ROS_ERROR("corridor box indexed at %zu generation failed!", i);
      return false;
    }

    if (show_cr) {
      iris_problem.setSeedPoint(Eigen::Vector2d(f_centre[0], f_centre[1]));
      // iris_problem.setSeedPoint(Eigen::Vector2d(guess.states[i].x, guess.states[i].y - 1.5));

      iris::IRISRegion region = inflate_region(iris_problem, options);
      auto points = region.polyhedron.generatorPoints();
      std::vector<math::Vec2d> points_;
      for (const auto& pts : points) {
        math::Vec2d pts_temp(pts[0], pts[1]);
        points_.push_back(pts_temp);
      }

      auto color = visualization::Color::Blue;
      color.set_alpha(0.05);
      // visualization::PlotPolygon(math::Polygon2d(math::Box2d(box)), 0.05, color, i, "Corridor " + std::to_string(i));
      visualization::PlotPolygon(math::Polygon2d({points_}), 0.05, color, i, "Convex Region" + std::to_string(i));
    }
    for(int j = 0; j < config_->vehicle.vertices; j++) {
      constraints.corridor_lb(i, j*2) = box.min_x();
      constraints.corridor_lb(i, j*2+1) = box.min_y();
      constraints.corridor_ub(i, j*2) = box.max_x();
      constraints.corridor_ub(i, j*2+1) = box.max_y();
    }
  }
 
  visualization::Trigger();

  double infeasibility;
  int i = 5;
  if(!problem_->Solve(config_->opti_w_penalty0, constraints, guess, result, infeasibility, solve_time)) {
    ROS_ERROR("solver failed!");
    return false;
  }
  std::vector<double> offset, re_phi, offset_car;
  // offset_car.push_back(2.0);
  // offset.push_back(hypot(-18-(-20), -21.0083668325-(-20))); 
  // re_phi.push_back(atan2(21.0083668325-20, 20-18)); 
  offset.push_back(2.0);
  offset.push_back(hypot(-10-(-12.016733665), -10.0-(-8.0)));
  re_phi.push_back(0.0);
  re_phi.push_back(atan2(12.016733665-10, 10-8));
  int count_exp = 0;
  // while (!CheckDiffDriveKinematic(result, offset, re_phi) && !CheckCarKinematic(result, offset_car)) {
  //   config_->opti_t = config_->opti_t * config_->factor_a;
  //   config_->opti_w_diff_drive = config_->opti_w_diff_drive * config_->factor_b;
  //   problem_->Solve(config_->opti_w_penalty0, constraints, guess, result, infeasibility, solve_time);
  //   count_exp++;
  //   if (count_exp > 10) {
  //     return false;
  //   }
  // }
  if(infeasibility > config_->opti_varepsilon_tol) {
    ROS_WARN("infeasibility = %.6f > %.6f, trajectory may not be feasible", infeasibility, config_->opti_varepsilon_tol);
  }

  return true;
}

bool FormationPlanner::Plan_car_like(const FullStates &traj_lead, const double offset, FullStates &traj_follower, double& solve_time_car) {
  double solver_st = GetCurrentTimestamp();
  double dt = traj_lead.tf / traj_lead.states.size();
	traj_follower.states.resize(traj_lead.states.size());
	for (size_t l_index = 0; l_index < traj_lead.states.size(); l_index++) {
		traj_follower.states[l_index].x = traj_lead.states[l_index].x + offset * sin(traj_lead.states[l_index].theta);
		traj_follower.states[l_index].y = traj_lead.states[l_index].y - offset * cos(traj_lead.states[l_index].theta);
		traj_follower.states[l_index].theta = traj_lead.states[l_index].theta;
		traj_follower.states[l_index].v = (1 + offset * tan(traj_lead.states[l_index].phi) / config_->vehicle.wheel_base) * traj_lead.states[l_index].v;
		traj_follower.states[l_index].phi = atan(config_->vehicle.wheel_base * tan(traj_lead.states[l_index].phi) / (config_->vehicle.wheel_base + offset * traj_lead.states[l_index].phi));	
	}
	for (size_t l_index = 1; l_index < traj_lead.states.size() - 1; l_index++) {
    traj_follower.states[l_index].a = (traj_lead.states[l_index].v - traj_lead.states[l_index - 1].v) / dt;
		traj_follower.states[l_index].omega = (traj_lead.states[l_index].phi - traj_lead.states[l_index - 1].phi) / dt;
  }
  traj_follower.states[0].a = traj_lead.states[0].a;
  traj_follower.states[0].omega = traj_lead.states[0].omega;
  traj_follower.states[traj_lead.states.size() - 1].a = traj_lead.states.back().a;
  traj_follower.states[traj_lead.states.size() - 1].omega = traj_lead.states.back().omega;
  traj_follower.tf = traj_lead.tf;
  ROS_WARN("Car-like follower feasible trajectory found!");
  solve_time_car = GetCurrentTimestamp() - solver_st;
	return true;
}

bool FormationPlanner::Plan_diff_drive(const FullStates &guess, const FullStates &prev_sol, const TrajectoryPoint &start, const TrajectoryPoint &goal, 
  FullStates &result, const int robot_index, double& infeasibility, double& solve_time) {
  Constraints constraints;
  constraints.start = start;
  constraints.goal = goal;
  double max_error;
  // double infeasibility;
  double avg_error = 0.0, error_time = 0.0;
  if(!problem_->Solve_diff_drive(config_->opti_w_penalty0, constraints, guess, prev_sol, result, infeasibility, solve_time)) {
    ROS_ERROR("solver failed!");
    return false;
  }
  ROS_WARN("Fesible trajectory is found for %dth diff-drive robot!", robot_index);
  for (int i = 0; i < result.states.size(); i++) {
    error_time = hypot(guess.states[i].x - result.states[i].x, guess.states[i].y - result.states[i].y);
    if (error_time > max_error) {
      max_error = error_time;
    }
    avg_error += error_time;
  }
  avg_error /= result.states.size();
  ROS_WARN("max_error: %.6f, average error: %.6f", max_error, avg_error);
  if(infeasibility > config_->opti_varepsilon_tol) {
    ROS_WARN("infeasibility = %.6f > %.6f, trajectory may not be feasible", infeasibility, config_->opti_varepsilon_tol);
  }
  return true;
}

bool FormationPlanner::Plan_car_like_replan(const FullStates &guess, const FullStates &prev_sol, FullStates &result, const int robot_index, double& infeasible, double& solution_car_like) {
  Constraints constraints;
  double infeasibility;
  double max_error = 0.0, avg_error = 0.0, error_time = 0.0;
  if(!problem_->Solve_replan(config_->opti_w_penalty0, constraints, guess, prev_sol, result, infeasibility, solution_car_like)) {
    ROS_ERROR("solver failed!");
    return false;
  }
  ROS_WARN("Fesible trajectory is found for %dth replanned robot!", robot_index);
  for (int i = 0; i < result.states.size(); i++) {
    error_time = hypot(guess.states[i].x - result.states[i].x, guess.states[i].y - result.states[i].y);
    if (error_time > max_error) {
      max_error = error_time;
    }
    avg_error += error_time;
  }
  avg_error /= result.states.size();
  ROS_WARN("max_error: %.6f, average error: %.6f", max_error, avg_error);
  if(infeasibility > config_->opti_varepsilon_tol) {
    ROS_WARN("robot%d: infeasibility = %.6f > %.6f, trajectory may not be feasible", robot_index, infeasibility, config_->opti_varepsilon_tol);
  }
  if(infeasibility > 0.1) {
    ROS_ERROR("trajectory need to be refined!");
  }
  infeasible = infeasibility;
  return true;
}

bool FormationPlanner::CheckCarKinematic(const FullStates &current_result, const std::vector<double> offset_car) {
  for (int idx = 0; idx < offset_car.size(); idx++) { 
    double offset_car_ = offset_car[idx];
    double current_ref_vel, current_radius;
      // check velocity
      for (int i = 0; i < current_result.states.size(); i++) {
        if (current_result.states[i].phi >= 0) {
          current_radius = config_->vehicle.wheel_base / tan(-current_result.states[i].phi);
          current_ref_vel = current_result.states[i].v * (1 + offset_car_ * (current_result.states[i].phi) / config_->vehicle.wheel_base);
          if (current_ref_vel > config_->vehicle.max_velocity) {
            ROS_WARN("No kinematic-feasible trajetory found for car-like robot!");
            return false;
          }
      }
    }
    ROS_WARN("Kinematic-feasible for car-like robot!");
  }
  // check phi: has sovled to set the -phi_max as -0.218508783
  return true;
}

bool FormationPlanner::CheckDiffDriveKinematic(const FullStates &current_result, const std::vector<double> offset, const std::vector<double> re_phi) {
  for (int idx = 0; idx < offset.size(); idx++) { 
    double offset_ = offset[idx];
    double re_phi_ = re_phi[idx];
    for (int i = 1; i < current_result.states.size(); i++) {
      double dis_ts = hypot((current_result.states[i].x + offset_ * cos(current_result.states[i].theta - re_phi_)) - (current_result.states[i - 1].x + offset_ * cos(current_result.states[i - 1].theta - re_phi_)),
      (current_result.states[i].y + offset_ * sin(current_result.states[i].theta - re_phi_)) - (current_result.states[i - 1].y + offset_ * sin(current_result.states[i - 1].theta - re_phi_)));
      double vel_ts = dis_ts / (current_result.tf / current_result.states.size());
      if ( vel_ts  > config_->vehicle.max_velocity) {
        ROS_WARN("No kinematic-feasible trajetory found for diff-drive robot!");
        return false;
      }
    }
  }
  ROS_WARN("Kinematic-feasible for diff-drive robot!");
  return true;
}

void FormationPlanner::DeriveHeight(const std::vector<formation_planner::FullStates> traj_set, const std::vector<std::vector<double>> vertice_set,
std::vector<double> &height) {
  VVCM vvcm;
  height.clear();
  double min_height = 1e4;
  std::vector<std::vector<double>> pos_3d;
  std::vector<std::vector<double>> pos_2d; 
  std::vector<std::vector<int>> taut_set;
  for (int i = 0; i < traj_set[0].states.size(); i++) {
    std::vector<std::vector<double>> robot_pos_set;
    for (int j = 0; j < traj_set.size(); j++) {
      robot_pos_set.push_back({traj_set[j].states[i].x, traj_set[j].states[i].y});
    }
    ForwardKinematics fk_test(traj_set.size(), vvcm.zr, vertice_set, robot_pos_set);
    fk_test.SolveFk(vertice_set, robot_pos_set, pos_3d, pos_2d, taut_set, vvcm.zr);
    for (int height_ind = 0; height_ind < pos_3d.size(); height_ind++) {
      if (pos_3d[height_ind][2] < min_height) {
        min_height = pos_3d[height_ind][2];
      }
    }
    if (!pos_3d.empty()) height.push_back(min_height);
  }
}


bool FormationPlanner::CheckHeightCons(const std::vector<formation_planner::FullStates> traj_set, const std::vector<double> height_cons, const std::vector<std::vector<double>> vertice_set,
std::vector<double> &height) {
  VVCM vvcm;
  height.clear();
  double min_height = 1e4;
  std::vector<std::vector<double>> pos_3d;
  std::vector<std::vector<double>> pos_2d; 
  std::vector<std::vector<int>> taut_set;
  for (int i = 0; i < height_cons.size(); i++) {
    if (height_cons[i] != -1) {
      std::vector<std::vector<double>> robot_pos_set;
      for (int j = 0; j < traj_set.size(); j++) {
        robot_pos_set.push_back({traj_set[j].states[i].x, traj_set[j].states[i].y});
      }
      ForwardKinematics fk_test(traj_set.size(), vvcm.zr, vertice_set, robot_pos_set);
      fk_test.SolveFk(vertice_set, robot_pos_set, pos_3d, pos_2d, taut_set, vvcm.zr);
      for (int height_ind = 0; height_ind < pos_3d.size(); height_ind++) {
        if (pos_3d[height_ind][2] < min_height) {
          min_height = pos_3d[height_ind][2];
        }
      }
      height.push_back(min_height);
      if (height_cons[i] >= min_height) {
        ROS_ERROR("Height constraints violated at index %d !", i);
        return false;
      }
    }
    else {
      std::vector<std::vector<double>> robot_pos_set;
      for (int j = 0; j < traj_set.size(); j++) {
        robot_pos_set.push_back({traj_set[j].states[i].x, traj_set[j].states[i].y});
      }
      ForwardKinematics fk_test(traj_set.size(), vvcm.zr, vertice_set, robot_pos_set);
      fk_test.SolveFk(vertice_set, robot_pos_set, pos_3d, pos_2d, taut_set, vvcm.zr);
      for (int height_ind = 0; height_ind < pos_3d.size(); height_ind++) {
        if (pos_3d[height_ind][2] < min_height) {
          min_height = pos_3d[height_ind][2];
        }
      }
      height.push_back(min_height); 
    }
  }
  ROS_WARN("All height constraitns are satisfied!");
  return true;
}

bool ObstacleIntersecting(const std::vector<Point>& poly_robot, std::shared_ptr<formation_planner::Environment>& env, int& obs_index) {
  // Eigen::Vector2d intersect_pt;
  Eigen::Vector2d q1, q2;
  vector<int> obs_index_set;
  double max_height = 0.0;
  for (int i = 0; i < env->polygons().size(); ++i) {
    std::vector<Point> poly_obs;
    for (int j = 0; j < env->polygons()[i].points().size(); j++) {
      poly_obs.push_back({env->polygons()[i].points()[j].x(), env->polygons()[i].points()[j].y()});
    }
    if (ObstacleIntersect(poly_robot, poly_obs)) {       
      obs_index_set.push_back(i);
    } 
  }
  if (obs_index_set.empty()) {
    obs_index = -1;
    return false;
  }
  else {
    obs_index = obs_index_set[0];
    for (int i = 1; i < obs_index_set.size(); i++) {
      if (env->heights()[obs_index_set[i]] > max_height) {
        obs_index = i;
        max_height = env->heights()[obs_index_set[i]];
      }
    }
    return true;
  }
}

void FormationPlanner::GenerateHeightCons(const std::vector<FullStates>& guess, std::shared_ptr<Environment> env, std::vector<double>& height_cons) {
  int num_robot = guess.size();
  height_cons.resize(guess[0].states.size());
  std::vector<double> collided_obs(guess[0].states.size());
  int obs_index;
  std::vector<Point> poly_robot;
  for (int i = 0; i < guess[0].states.size(); i++) {
    poly_robot.clear();
    for (int j = 0; j < num_robot; j++) {
      poly_robot.push_back({guess[j].states[i].x, guess[j].states[i].y});
    }
    // for (int k = 0; k < env->heights().size(); k++) {
    if (ObstacleIntersecting(poly_robot, env, obs_index)) {
      height_cons[i] = env->heights()[obs_index];
    }
    else {
      height_cons[i] = -1;
    }
    // }
  }
}

// simple version
void GenerateDesiredRP(const std::vector<double>& height_cons, std::vector<double>& height_cons_set) {
  VVCM vvcm;
  // double li = vvcm.xv2 / sqrt(3);
  for (int i = 0; i < height_cons.size(); i++) {
    if (height_cons[i] == -1) {
      height_cons_set[i] = -1;
    }
    else {
      // double r_oi = sqrt(li * li - pow(vvcm.zr - height_cons[i], 2));
      // double r_ij = r_oi * sqrt(3);
      if (height_cons_set[i] == -1) {
        height_cons_set[i] = vvcm.xv2t;
      }
      else {
        height_cons_set[i] += vvcm.radius_inc;
      }
    }
  }
}


bool FormationPlanner::Plan_fm(
  const std::vector<FullStates> &prev_sol, 
  const std::vector<TrajectoryPoint> &start_set, 
  const std::vector<TrajectoryPoint> &goal_set, 
  iris::IRISProblem &iris_problem, 
  std::vector<FullStates> &result, 
  std::vector<double>& coarse_time_set, 
  std::vector<double>& solve_time_set, 
  bool show_cr, 
  const std::vector<math::Polygon2d>& polys_inflat,
  const std::vector<std::vector<std::vector<std::vector<double>>>>& hyperparam_sets,
  std::vector<ros::Publisher> path_pub_set,
  int& warm_start,
  double& cost,
  int& solve_success,
  double& e_max, double& e_avg,
  double& avg, double& std) {
  std::string package_path = ros::package::getPath("formation_planner");   
  double solve_time = 0.0;
  cost = 0.0;
  std::vector<FullStates> result_opt; 
  int num_robot = start_set.size();
  VVCM vvcm;
  std::vector<std::vector<double>> vertice_set;
  std::vector<FullStates> guess(num_robot);
  guess = prev_sol;
  std::vector<Constraints> constraints(num_robot);
  std::vector<std::vector<std::vector<std::vector<double>>>> corridor_cons(num_robot);
  std::vector<std::vector<math::Pose>> initial_plan_set;
  std::vector<formation_planner::visualization::Color> colors = {
    visualization::Color::Red,
    visualization::Color::Green,
    visualization::Color::Blue,
    visualization::Color::Cyan,
    visualization::Color::Yellow,
    visualization::Color::Magenta,
    visualization::Color::Grey,
    visualization::Color::Yellow
  };
  generateRegularPolygon(vvcm.formation_radius, guess.size(), vertice_set);
  if (guess[0].states.empty()) {
    for (int i = 0; i < prev_sol.size(); i++) {
      guess[i] = StitchPreviousSolution(prev_sol[i], start_set[i]);
      if(!CheckGuessFeasibility(guess[i])) {
        std::vector<math::Pose> initial_path;
        double st = GetCurrentTimestamp();
        // if (i == 0) {
          // for (int i_ = 0; i_ < corridor_sets[i].size(); i_++) {
          //   math::Vec2d center((corridor_sets[i][i_][0] + corridor_sets[i][i_][2]) / 2, (corridor_sets[i][i_][1] + corridor_sets[i][i_][3]) / 2);
          //   math::AABox2d box(center, corridor_sets[i][i_][2] - corridor_sets[i][i_][0], corridor_sets[i][i_][3] - corridor_sets[i][i_][1]);
          //   auto color = colors[i];
          //   color.set_alpha(0.1);
          //   visualization::PlotPolygon(math::Polygon2d(math::Box2d(box)), 0.5, color, i, "Corridor " + std::to_string(i_)); 
          //   visualization::Trigger();
          // }
        // }
        if(!coarse_path_planner_.Plan(start_set[i].pose(), goal_set[i].pose(), initial_path, hyperparam_sets[i])) {
          ROS_ERROR("re-plan coarse path failed!");
          return false;
        }
        initial_plan_set.push_back(initial_path);
        guess[i] = GenerateGuessFromPath(initial_path, start_set[i], 0, false);
        coarse_time_set.push_back(GetCurrentTimestamp() - st);
        // ROS_INFO("coarse path generation time: %f", coarse_time);
        // std::vector<double> xs, ys;
        // int ind = 0;
        // for(auto &pose: initial_path) {
        //   xs.push_back(pose.x()); ys.push_back(pose.y());
        // //   if (i == 2) {
        //   // auto box = config_->vehicle.GenerateBox(pose);
        //   // auto color = visualization::Color::Red;
        //   // color.set_alpha(0.4);
        //   // visualization::PlotPolygon(math::Polygon2d(box), 0.2, color, i, "coarse"+std::to_string(i));
        // //   }
        // }
        // visualization::Plot(xs, ys, 0.5, colors[i], i, "Coarse Path" + std::to_string(i));
        // visualization::Trigger();
      }
    }
    double tf_max = 0.0;
    int tf_max_ind;
    for (int i = 0; i < guess.size(); i++) {
      if (guess[i].tf > tf_max) {
        tf_max = guess[i].tf;
        tf_max_ind = i;
      }
    }
    for (int i = 0; i < guess.size(); i++) {
      if (i != tf_max_ind) {
        guess[i] = GenerateGuessFromPath(initial_plan_set[i], start_set[i], guess[tf_max_ind].states.size(), true);
      }
    }
    for (int i = 0; i < guess.size(); i++) {
      if (i != tf_max_ind) {
        double ratio_ = guess[i].tf / guess[tf_max_ind].tf;
        for (int j = 0; j < guess[i].states.size(); j++) {
          guess[i].states[j].v = guess[i].states[j].v * ratio_;
          guess[i].states[j].a = guess[i].states[j].a * ratio_;
          guess[i].states[j].omega = guess[i].states[j].omega * ratio_;
        }
      }
      guess[i].tf = guess[tf_max_ind].tf;  
    }
  }

  for (int ind = 0; ind < guess.size(); ind++) {
    std::vector<Eigen::Vector2d> ref_path;
    std::vector<std::vector<std::vector<double>>> hPolys;
    std::vector<std::vector<math::Vec2d>> corri_vertices_set;
      std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> keyPts;
    constraints[ind].start = start_set[ind];
    constraints[ind].goal = goal_set[ind];
    for(size_t i = 0; i < guess[ind].states.size(); i++) {
      ref_path.push_back({guess[ind].states[i].x, guess[ind].states[i].y});
    }
    env_->generateSFC(ref_path, polys_inflat, 5.0, hPolys, keyPts, corri_vertices_set);
    corridor_cons[ind] = hPolys;
    // Visualise the corridors
    // for (int j = 0; j < corri_vertices_set.size(); j++) {
    //   auto color_poly = visualization::Color::Magenta;
    //   color_poly.set_alpha(0.1);
    //   visualization::PlotPolygon(math::Polygon2d(corri_vertices_set[j]), 0.1, color_poly, 0, "Corridor" + std::to_string(ind) + std::to_string(j));
    // }
    // visualization::Trigger();  
  }
  double infeasibility;
  std::vector<double> height_cons, height;
  std::vector<double> height_cons_set(guess[0].states.size(), vvcm.xv2t);
  warm_start = 0;
  GenerateHeightCons(guess, env_, height_cons);
  result = guess;
  while (
  // !CheckHeightCons(result, height_cons, vertice_set, height) || 
  warm_start < 15) {
    for (int j = 0; j < result.size(); j++) {
      DrawTrajectoryRviz(result[j], config_, j, path_pub_set[j]);
    }
    DeriveHeight(result, vertice_set, height);
    computeFormationSim(result, e_max, e_avg);
    avg = calculateAverage(height);
    std = calculateStandardDeviation(height);
    ROS_WARN("e max: %.6f, e avg : %.6f", e_max, e_avg);
    ROS_WARN("avg: %.6f, std : %.6f", avg, std);
    if (result[0].tf < 150 && warm_start != 0) {
      for (int j = 0; j < result.size(); j++) {
        writeTrajectoryToYAML(result[j], package_path + "/traj_result/flexible_formation/" 
        + std::to_string(num_robot) + "/traj_" + std::to_string(num_robot) + std::to_string(j) + std::to_string(1000) +".yaml");
      }  
    }
    // generate good initial guess
    if (warm_start < 5) {
      if(!problem_->SolveFm(config_->opti_w_penalty0, constraints, guess, result, infeasibility, solve_time, corridor_cons, height_cons_set)) {
        ROS_ERROR("infeasibility = %.6f > %.6f, trajectory may not be feasible", infeasibility, config_->opti_varepsilon_tol);
        return false;
      }    
      solve_time_set.push_back(solve_time);
      if(infeasibility > 1) {
        ROS_ERROR("infeasibility = %.6f > %.6f, trajectory may not be feasible", infeasibility, config_->opti_varepsilon_tol);
        return false;
      }
      guess = result;
      if (result_opt.empty()) {
        result_opt = result;
      }
      GenerateHeightCons(result, env_, height_cons);
      if (result_opt[0].tf > result[0].tf) {
        result_opt = result;
      }
      // if (result_opt[0].tf < 100 && infeasibility < 0.001) {
      //   for (int i = 0; i < result_opt.size(); i++) {
      //     for(int j = 0; j < result_opt[i].states.size(); j++) {
      //       // a^2 + w^2
      //       cost += pow(result_opt[i].states[j].a, 2) + pow(result_opt[i].states[j].omega, 2);
      //     }
      //   }
      //   cost /= num_robot;
      //   result = result_opt;
      //   solve_success = 1;
      //   warm_start++;
      //   return true;
      // }
      warm_start++;
      continue;
    }
    else if (warm_start == 5) {
      guess = result_opt;
      result = result_opt;
      result_opt.clear();
      return false;
    }
    // generate inter-distance constraints
    if (!CheckHeightCons(result, height_cons, vertice_set, height)){
      GenerateHeightCons(guess, env_, height_cons);
      GenerateDesiredRP(height_cons, height_cons_set);
    }
    else if (result_opt.empty() || result_opt[0].tf > result[0].tf) {
      ROS_WARN("Better solution found!");
      result_opt = result;     
    } 
    if(!problem_->SolveFm(config_->opti_w_penalty0, constraints, guess, result, infeasibility, solve_time, corridor_cons, height_cons_set)) {
      ROS_ERROR("solver failed!");
      return false;
    }
    solve_time_set.push_back(solve_time);
    guess = result;
    int max_radius = 0.0; 
    for (int j = 0; j < height_cons_set.size(); j++) {
      if (max_radius < height_cons_set[j]) {
        max_radius = height_cons_set[j];
      }
    }
    if (max_radius > vvcm.formation_radius) {
      ROS_ERROR("Exceed maximum radius!");
      return false;
    }
    if(infeasibility > 0.5) {
      ROS_ERROR("infeasibility = %.6f > %.6f, trajectory may not be feasible", infeasibility, config_->opti_varepsilon_tol);
      return false;
    }
    // if (warm_start > 5 && result_opt[0].tf < 100 && infeasibility < 0.001) {
    //   for (int i = 0; i < result_opt.size(); i++) {
    //     for(int j = 0; j < result_opt[i].states.size(); j++) {
    //       // a^2 + w^2
    //       cost += pow(result_opt[i].states[j].a, 2) + pow(result_opt[i].states[j].omega, 2);
    //     }
    //   }
    //   cost /= num_robot;
    //   result = result_opt;
    //   warm_start++;
    //   solve_success = 1;
    //   return true;
    // }
    warm_start++;
  }
  // for (int i = 0; i < result_opt.size(); i++) {
  //   for(int j = 0; j < result_opt[i].states.size(); j++) {
  //     // a^2 + w^2
  //     cost += pow(result_opt[i].states[j].a, 2) + pow(result_opt[i].states[j].omega, 2);
  //   }
  // }
  // cost /= num_robot;
  // result = result_opt;
  // solve_success = 1;
  return false;
}

FullStates FormationPlanner::ResamplePath(const std::vector<math::Pose> &path, const int step_num, bool ratio) const {
  std::vector<int> gears(path.size());
  std::vector<double> stations(path.size(), 0);

  for(size_t i = 1; i < path.size(); i++) {
    double tracking_angle = atan2(path[i].y() - path[i-1].y(), path[i].x() - path[i-1].x());
    bool gear = std::abs(math::NormalizeAngle(tracking_angle - path[i].theta())) < M_PI_2;
    gears[i] = gear ? 1 : -1;

    stations[i] = stations[i-1] + path[i].DistanceTo(path[i-1]);
  }

  if(gears.size() > 1) {
    gears[0] = gears[1];
  }

  std::vector<double> time_profile(gears.size());
  size_t last_idx = 0;
  double start_time = 0;
  for(size_t i = 0; i < gears.size(); i++) {
    if(i == gears.size() - 1 || gears[i+1] != gears[i]) {
      std::vector<double> station_segment;
      std::copy_n(stations.begin(), i - last_idx + 1, std::back_inserter(station_segment));

      auto profile = GenerateOptimalTimeProfileSegment(station_segment, start_time);
      std::copy(profile.begin(), profile.end(), std::next(time_profile.begin(), last_idx));
      start_time = profile.back();
      last_idx = i;
    }
  }

  int nfe = ratio ? step_num : std::max(config_->min_nfe, int(time_profile.back() / config_->time_step));
  auto interpolated_ticks = math::LinSpaced(time_profile.front(), time_profile.back(), nfe);

  std::vector<double> prev_x(path.size()), prev_y(path.size()), prev_theta(path.size());
  for(size_t i = 0; i < path.size(); i++) {
    prev_x[i] = path[i].x();
    prev_y[i] = path[i].y();
    prev_theta[i] = path[i].theta();
  }

  FullStates result;
  result.tf = interpolated_ticks.back();
  result.states.resize(nfe);
  auto interp_x = math::Interpolate1d(time_profile, prev_x, interpolated_ticks);
  auto interp_y = math::Interpolate1d(time_profile, prev_y, interpolated_ticks);
  auto interp_theta = math::ToContinuousAngle(math::Interpolate1d(time_profile, prev_theta, interpolated_ticks));
  for(size_t i = 0; i < nfe; i++) {
    result.states[i].x = interp_x[i];
    result.states[i].y = interp_y[i];
    result.states[i].theta = interp_theta[i];
  }

  double dt = interpolated_ticks[1] - interpolated_ticks[0];
  for(size_t i = 0; i < nfe-1; i++) {
    double tracking_angle = atan2(result.states[i+1].y - result.states[i].y, result.states[i+1].x - result.states[i].x);
    bool gear = std::abs(math::NormalizeAngle(tracking_angle - result.states[i].theta)) < M_PI_2;
    double velocity = hypot(result.states[i+1].y - result.states[i].y, result.states[i+1].x - result.states[i].x) / dt;

    result.states[i].v = std::min(config_->vehicle.max_velocity, std::max(-config_->vehicle.max_velocity, gear ? velocity : -velocity));
    result.states[i].phi = std::min(config_->vehicle.phi_max, std::max(-config_->vehicle.phi_min, atan((result.states[i+1].theta - result.states[i].theta) * config_->vehicle.wheel_base / (result.states[i].v * dt))));
  }

  for(size_t i = 0; i < nfe-1; i++) {
    result.states[i].a = std::min(config_->vehicle.max_acceleration, std::max(-config_->vehicle.max_acceleration, (result.states[i+1].v - result.states[i].v) / dt));
    result.states[i].omega = std::min(config_->vehicle.omega_max, std::max(-config_->vehicle.omega_max, (result.states[i+1].phi - result.states[i].phi) / dt));
  }
  return result;
}

std::vector<double> FormationPlanner::GenerateOptimalTimeProfileSegment(const std::vector<double> &stations, double start_time) const {
  double max_accel = config_->vehicle.max_acceleration; double max_decel = -config_->vehicle.max_acceleration;
  double max_velocity = config_->vehicle.max_velocity; double min_velocity = -config_->vehicle.max_velocity;

  int accel_idx = 0, decel_idx = stations.size()-1;
  double vi = 0.0;
  std::vector<double> profile(stations.size());
  for (int i = 0; i < stations.size()-1; i++) {
    double ds = stations[i+1] - stations[i];

    profile[i] = vi;
    vi = sqrt(vi * vi + 2 * max_accel * ds);
    vi = std::min(max_velocity, std::max(min_velocity, vi));

    if(vi >= max_velocity) {
      accel_idx = i+1;
      break;
    }
  }

  vi = 0.0;
  for (int i = stations.size()-1; i > accel_idx; i--) {
    double ds = stations[i] - stations[i-1];
    profile[i] = vi;
    vi = sqrt(vi * vi - 2 * max_decel * ds);
    vi = std::min(max_velocity, std::max(min_velocity, vi));

    if(vi >= max_velocity) {
      decel_idx = i;
      break;
    }
  }

  std::fill(std::next(profile.begin(), accel_idx), std::next(profile.begin(), decel_idx), max_velocity);

  std::vector<double> time_profile(stations.size(), start_time);
  for(size_t i = 1; i < stations.size(); i++) {
    if(profile[i] < 1e-6) {
      time_profile[i] = time_profile[i-1];
      continue;
    }
    time_profile[i] = time_profile[i-1] + (stations[i] - stations[i-1]) / profile[i];
  }
  return time_profile;
}

}