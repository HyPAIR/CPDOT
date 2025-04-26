/**
 * file animation.cpp
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief formation planning algorithm test
 * data 2023-11-22
 * 
 * @copyright Copyroght(c) 2023
*/
#include <ros/ros.h>
#include <ros/package.h> 
#include <memory>
#include "formation_planner/formation_planner.h"
#include "formation_planner/visualization/plot.h"

#include <nav_msgs/Path.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>

#include "iris/iris.h"
#include "formation_planner/topo_prm.h"
#include "formation_planner/IdentifyHomotopy.h"
#include "formation_planner/yaml_all.h"
#include "formation_planner/math/generate_obs.h"
#include "formation_planner/optimizer_interface.h"
#include <Eigen/Core>
#include <math.h>
#include <random>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <omp.h>

using namespace formation_planner;
// struct Point {
//     float x, y;
// };

// 计算两个向量的叉积
float crossProduct(Point A, Point B, Point C) {
    return (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x);
}

// 判断点P是否在三角形ABC内的函数
bool isPointInTriangle(Point A, Point B, Point C, Point P) {
    // 计算三角形的三个边与点P的叉积
    float cross1 = crossProduct(A, B, P);
    float cross2 = crossProduct(B, C, P);
    float cross3 = crossProduct(C, A, P);

    // 如果叉积符号相同（都为正或都为负），则点P在三角形内或边上
    return (cross1 >= 0 && cross2 >= 0 && cross3 >= 0) ||
           (cross1 <= 0 && cross2 <= 0 && cross3 <= 0);
}
visualization_msgs::InteractiveMarker CreateMarker(int i, const math::Polygon2d &polygon, double width, visualization::Color c) {
  visualization_msgs::InteractiveMarker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.name = "Obstacle " + std::to_string(i);
  // marker.pose.position.x = polygon.center().x();
  // marker.pose.position.y = polygon.center().y();
  marker.pose.orientation.w = 1.0;

  visualization_msgs::Marker polygon_marker;
  polygon_marker.header.frame_id = marker.header.frame_id;
  polygon_marker.header.stamp = ros::Time();
  polygon_marker.ns = "Obstacles";
  polygon_marker.id = i;

  polygon_marker.action = visualization_msgs::Marker::ADD;
  polygon_marker.type = visualization_msgs::Marker::LINE_STRIP;
  polygon_marker.pose.orientation.w = 1.0;
  polygon_marker.scale.x = width;
  polygon_marker.color = c.toColorRGBA();

  for (size_t i = 0; i < polygon.num_points(); i++) {
    geometry_msgs::Point pt;
    pt.x = polygon.points().at(i).x();
    pt.y = polygon.points().at(i).y();
    polygon_marker.points.push_back(pt);
  }
  polygon_marker.points.push_back(polygon_marker.points.front());

  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back(polygon_marker);

  marker.controls.push_back(box_control);

  visualization_msgs::InteractiveMarkerControl move_control;
  move_control.name = "move_x";
  move_control.orientation.w = 0.707107f;
  move_control.orientation.x = 0;
  move_control.orientation.y = 0.707107f;
  move_control.orientation.z = 0;
  move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

  marker.controls.push_back(move_control);
  return marker;
}

void DrawTrajectoryRviz(const FullStates sol_traj,std::shared_ptr<formation_planner::PlannerConfig> config,
      int robot_index, ros::Publisher path_pub) {
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
    auto color = robot_index > 0 ? visualization::Color::Green : visualization::Color::Red;
    // color.set_alpha(0.4);
    visualization::PlotPolygon(math::Polygon2d(box), 0.2, color, i, robot_name);
  }
  path_pub.publish(msg);
  visualization::Trigger();
}

// void writeVectorToYAML(const std::vector<double>& data, const std::string& file_path) {
//     try {
//         // 尝试加载已有的数据
//         YAML::Node existing_data;
//         std::ifstream file_in(file_path);
//         if (file_in) {
//             existing_data = YAML::Load(file_in);
//             file_in.close();
//         }

//         // 将新的向量追加到数据中
//         YAML::Node new_data;
//         for (const auto& value : data) {
//             new_data.push_back(value);
//         }
//         existing_data.push_back(new_data);

//         // 以追加模式打开文件，并将新数据写入文件
//         std::ofstream file_out(file_path, std::ofstream::out | std::ofstream::trunc);
//         file_out << existing_data;
//         file_out.close();
//     } catch (const std::exception& e) {
//         std::cerr << "Error: " << e.what() << std::endl;
//     }
// }

void generateRegularPolygon(const double cx, const double cy, const double r, const int k, 
  std::vector<Eigen::Vector2d>& vertice_set, std::vector<TrajectoryPoint>& tp_set) {
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
        tp_set.push_back(tp_temp);
    }
}

double getRandomDouble(double min, double max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(min, max);
    return dis(gen);
}

// 生成在[-pi, pi]范围内的随机角度
double getRandomAngle() {
    return getRandomDouble(0, M_PI / 2);
}

// 生成随机三维点
TrajectoryPoint generateRandomStartPoint() {
    TrajectoryPoint point;
    // point.x = getRandomDouble(-20.0, -8.0);
    // point.y = getRandomDouble(-20.0, -5.0);
    point.x = -30.0;
    point.y = getRandomDouble(-30.0, 30.0);
    point.theta = getRandomAngle();
    // point.theta = 0.0;
    return point;
}

TrajectoryPoint generateRandomGoalPoint() {
    TrajectoryPoint point;
    // point.x = getRandomDouble(8.0, 20.0);
    // point.y = getRandomDouble(0.0, 20.0);
    point.x = getRandomDouble(-30.0, 30.0);
    point.y = 30.0;
    // point.theta = M_PI / 2;
    point.theta = getRandomAngle();
    return point;
}

TrajectoryPoint generateRandomnObstacle() {
  TrajectoryPoint point;
  Point A1(-30, -18);
  Point B1(-30, 30);
  Point C1(18, 30);
  Point A2(-18, -30);
  Point B2(30, -30);
  Point C2(30, 18);
  Point P(0, 0);
  do
  {
    point.x = getRandomDouble(-27, 27);
    point.y = getRandomDouble(-27, 27);
    point.theta = getRandomAngle();
    P.x = point.x;
    P.y = point.y;
  } while (!isPointInTriangle(A1, B1, C1, P) && !isPointInTriangle(A2, B2, C2, P));
  return point;
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

/*读取障碍物*/
void ReadRandomObstacle(int num_obs, std::vector<std::vector<double>>& height_set, std::vector<math::Pose>& obstacle,
std::vector<double>& height, std::vector<math::Polygon2d>& polys, std::vector<math::Polygon2d>& polys_inflat,
std::vector<std::vector<math::Vec2d>>& poly_vertices_set, math::GenerateObstacle generateobs
) {
  height_set.clear();
  obstacle.clear();
  height.clear();
  polys.clear();
  polys_inflat.clear();
  poly_vertices_set.clear();
  std::string package_path = ros::package::getPath("formation_planner");
  std::vector<std::vector<double>> obs_list = readVectorsFromYAML(package_path + "/traj_result/obstacle_" + std::to_string(num_obs) + ".yaml");
  height_set = readVectorsFromYAML(package_path + "/traj_result/obs_height_" + std::to_string(num_obs) + ".yaml");
  for (int i = 0; i < obs_list.size(); i++) {
    obstacle.push_back({obs_list[i][0], obs_list[i][1], obs_list[i][2]});
  }
  std::vector<int> obs_type_set = {1, 1, 3, 3, 6, 6};
  for (int obs_ind = 0; obs_ind < obstacle.size(); obs_ind++) {
    std::vector<math::Vec2d> poly_vertices;
    math::Vec2d centre(obstacle[obs_ind].x(), obstacle[obs_ind].y());
    if (obs_ind < num_obs / 2) {
      poly_vertices = generateobs.rotatePoint(centre, obstacle[obs_ind].theta(), false, obs_type_set[obs_ind]);
    }
    else {
      poly_vertices = generateobs.rotatePoint(centre, obstacle[obs_ind].theta(), true, obs_type_set[obs_ind]);
    }
    poly_vertices_set.push_back(poly_vertices);
    polys.push_back(math::Polygon2d(poly_vertices));
  }
  for (int i = 0; i < polys.size(); i++) {
    polys_inflat.push_back(math::Polygon2d({
      {polys[i].points()[0].x() - 0.8, polys[i].points()[0].y() + 0.8}, 
      {polys[i].points()[1].x() - 0.8, polys[i].points()[1].y() - 0.8}, 
      {polys[i].points()[2].x() + 0.8, polys[i].points()[2].y() - 0.8}, 
      {polys[i].points()[3].x() + 0.8, polys[i].points()[3].y() + 0.8}
      }));
  }
}

/*手动生成障碍物*/
void DefineRandomObstacle(int num_obs, std::vector<std::vector<double>>& height_set, std::vector<math::Pose>& obstacle,
std::vector<double>& height, std::vector<math::Polygon2d>& polys, std::vector<math::Polygon2d>& polys_inflat, std::vector<math::Polygon2d>& polys_inflat_,
std::vector<std::vector<math::Vec2d>>& poly_vertices_set, math::GenerateObstacle generateobs
) {
  height_set.clear();
  obstacle.clear();
  height.clear();
  polys.clear();
  polys_inflat.clear();
  poly_vertices_set.clear();
  // height = {0.4, 0.7, 9.4, 9.4, 9.7, 9.7, 9.7 ,9.7};
  height = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.6, 0.79, 0.5};
  height_set.push_back(height);

  // obstacle.push_back({0, -12, 0});
  // obstacle.push_back({ 0, 12, 0});
  // obstacle.push_back({12, -5, 0});
  // obstacle.push_back({ 12, 5, 0});
  // obstacle.push_back({0, -6, 0});
  // obstacle.push_back({ 0, 6, 0});
  // // obstacle.push_back({13, 5, 0});
  // // obstacle.push_back({13, 1.5, 0});
  // // obstacle.push_back({13, -1.5, 0});
  // // obstacle.push_back({13, -5, 0});
  // obstacle.push_back({-12, 4, 0});
  // obstacle.push_back({-12, 0.5, 0});
  // obstacle.push_back({-12, -3, 0});
  // obstacle.push_back({-12, -6.5, 0});
  // obstacle.push_back({0, 0, 0});
  // obstacle.push_back({14, 1.5, 0});
  // obstacle.push_back({14, -1.5, 0});
  // obstacle.push_back({-1.893, 1.505, 1.57});
  // obstacle.push_back({0.106, 1.864, 1.57});
  // obstacle.push_back({-1.135, -0.202, 1.122});
  // obstacle.push_back({-0.149, -0.061, 2.062});
  // obstacle.push_back({-0.787, 1.067, 0.833});
  // obstacle.push_back({-0.986, 2.354, 2.441});
  obstacle.push_back({-5, 3, 0});
  obstacle.push_back({-5, -3, 0});
  obstacle.push_back({6, 0, 0.23});
  std::vector<int> obs_type_set = {1, 1, 3, 3, 6, 6};
  for (int obs_ind = 0; obs_ind < obstacle.size(); obs_ind++) {
    std::vector<math::Vec2d> poly_vertices;
    math::Vec2d centre(obstacle[obs_ind].x(), obstacle[obs_ind].y());
    if (obs_ind < 2) {
      poly_vertices = generateobs.rotatePoint(centre, obstacle[obs_ind].theta(), false, obs_type_set[obs_ind]);
    }
    else {
      poly_vertices = generateobs.rotatePoint(centre, obstacle[obs_ind].theta(), true, obs_type_set[obs_ind]);
    }
    poly_vertices_set.push_back(poly_vertices);
    polys.push_back(math::Polygon2d(poly_vertices));
  }
  for (int i = 0; i < polys.size(); i++) {
    polys_inflat.push_back(math::Polygon2d({
      {polys[i].points()[0].x() - 0.8, polys[i].points()[0].y() + 0.8}, 
      {polys[i].points()[1].x() - 0.8, polys[i].points()[1].y() - 0.8}, 
      {polys[i].points()[2].x() + 0.8, polys[i].points()[2].y() - 0.8}, 
      {polys[i].points()[3].x() + 0.8, polys[i].points()[3].y() + 0.8}
      }));
  }
  for (int i = 0; i < polys.size(); i++) {
    polys_inflat_.push_back(math::Polygon2d({
      {polys[i].points()[0].x() - 0.5, polys[i].points()[0].y() + 0.5}, 
      {polys[i].points()[1].x() - 0.5, polys[i].points()[1].y() - 0.5}, 
      {polys[i].points()[2].x() + 0.5, polys[i].points()[2].y() - 0.5}, 
      {polys[i].points()[3].x() + 0.5, polys[i].points()[3].y() + 0.5}
      }));
  }
}

/*随机生成障碍物*/
void GenerateRandomObstacle(int num_obs, std::vector<std::vector<double>>& height_set, std::vector<math::Pose>& obstacle,
std::vector<double>& height, std::vector<math::Polygon2d>& polys, std::vector<math::Polygon2d>& polys_inflat,
std::vector<std::vector<math::Vec2d>>& poly_vertices_set, math::GenerateObstacle generateobs
) {
  height_set.clear();
  obstacle.clear();
  height.clear();
  polys.clear();
  polys_inflat.clear();
  poly_vertices_set.clear();
  for (int i = 0; i < num_obs; i++) {
    TrajectoryPoint obs_pt = generateRandomnObstacle();
    obstacle.push_back({obs_pt.x, obs_pt.y, obs_pt.theta});
  }
  std::string package_path = ros::package::getPath("formation_planner");
  clearYAMLFile(package_path + "/traj_result/obstacle_" + std::to_string(num_obs) + ".yaml");
  clearYAMLFile(package_path + "/traj_result/obs_height_" + std::to_string(num_obs) + ".yaml");
  for (int i = 0; i < obstacle.size(); i++) {
    std::vector<double> new_vector = {obstacle[i].x(), obstacle[i].y(), obstacle[i].theta()};
    writeVectorToYAML(new_vector, package_path + "/traj_result/obstacle_" + std::to_string(num_obs) + ".yaml");
  }
  for (int i = 0; i < num_obs; i++)
  {
    height.push_back(getRandomDouble(0.45, 0.8));
  }
  // height = {0.4, 0.7, 9.4, 9.4, 9.7, 9.7, 9.7 ,9.7};
  // height = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.6, 0.79, 0.5};
  height_set.push_back(height);
  writeVectorToYAML(height, package_path + "/traj_result/obs_height_" + std::to_string(num_obs) + ".yaml");
  /*读取障碍物*/
  // std::vector<std::vector<double>> obs_list = readVectorsFromYAML("/home/weijian/CPDOT/src/formation_planner/traj_result/obstacle_" + std::to_string(num_obs) + ".yaml");
  // height_set = readVectorsFromYAML("/home/weijian/CPDOT/src/formation_planner/traj_result/obs_height_" + std::to_string(num_obs) + ".yaml");
  // for (int i = 0; i < obs_list.size(); i++) {
  //   obstacle.push_back({obs_list[i][0], obs_list[i][1], obs_list[i][2]});
  // }
  // obstacle.push_back({-7, 0, 0.3});
  // obstacle.push_back({7, 0, 0.1});
  // obstacle.push_back({-1, 0, 0.2});
  // obstacle.push_back({1, 0, 0.17});
  // obstacle.push_back({-1.197/2, 0, 0});
  // obstacle.push_back({1.197/2, 0, 0});
  // obstacle.push_back({-1.197/2, 0, 0});
  // obstacle.push_back({-1.197/2, 0, 0});
  // obstacle.push_back({-15, 3.5, 0});
  // obstacle.push_back({-15, -3.5, 0});
  // obstacle.push_back({-0.361, 4.626, 0.25});
  // obstacle.push_back({4.930, -3.882, 0.3});
  // obstacle.push_back({9.644, 5.065, 0});

  std::vector<int> obs_type_set = {1, 1, 3, 3, 6, 6};
  for (int obs_ind = 0; obs_ind < obstacle.size(); obs_ind++) {
    std::vector<math::Vec2d> poly_vertices;
    math::Vec2d centre(obstacle[obs_ind].x(), obstacle[obs_ind].y());
    if (obs_ind < num_obs / 2) {
      poly_vertices = generateobs.rotatePoint(centre, obstacle[obs_ind].theta(), false, obs_type_set[obs_ind]);
    }
    else {
      poly_vertices = generateobs.rotatePoint(centre, obstacle[obs_ind].theta(), true, obs_type_set[obs_ind]);
    }
    poly_vertices_set.push_back(poly_vertices);
    polys.push_back(math::Polygon2d(poly_vertices));
  }
  // polys.push_back(math::Polygon2d({{-14.295, 0.702}, {-13.105, -0.545}, {-7.934, -1.063}, {-5.950, 0.279}}));
  // polys.push_back(math::Polygon2d({{-2.332, 0.267}, {-0.697, -1.383}, {2.663, -1.508}, {1.877, 0.074}}));
  // polys.push_back(math::Polygon2d({{7.758, -1.695}, {9.018, -2.388}, {10.568, -2.266}, {13.068, -0.810}}));

  // env->heights() = height;
  // auto interactive_cb = [&](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &msg) {
  //   int idx = msg->marker_name.back() - '1';

  //   auto new_poly = polys[idx];
  //   new_poly.Move({ msg->pose.position.x, msg->pose.position.y });
  //   env->polygons(  // for (int i = 0; i < polys.size(); i++)
  // {
  //   height.push_back(getRandomDouble(0.3, 0.8));
  // }
  // writeVectorToYAML(height, "/home/weijian/CPDOT/src/formation_planner/traj_result/obs_height.yaml");).at(idx) = new_poly;
  // };
  // writeObstacleToYAML(poly_vertices_set, "/home/weijian/CPDOT/src/formation_planner/traj_result/obs.yaml");
  for (int i = 0; i < polys.size(); i++) {
    polys_inflat.push_back(math::Polygon2d({
      {polys[i].points()[0].x() - 0.8, polys[i].points()[0].y() + 0.8}, 
      {polys[i].points()[1].x() - 0.8, polys[i].points()[1].y() - 0.8}, 
      {polys[i].points()[2].x() + 0.8, polys[i].points()[2].y() - 0.8}, 
      {polys[i].points()[3].x() + 0.8, polys[i].points()[3].y() + 0.8}
      }));
  }
  // for (int i = 0; i < polys.size(); i++) {
  //   polys_corridor.push_back(math::Polygon2d({
  //     {polys[i].points()[0].x() - 0.5, polys[i].points()[0].y() + 0.5}, 
  //     {polys[i].points()[1].x() - 0.5, polys[i].points(best_tf
  // }
  }

int main(int argc, char **argv) {
  vector<formation_planner::visualization::Color> colors = {
    visualization::Color::Red,
    visualization::Color::Green,
    visualization::Color::Blue,
    visualization::Color::Cyan,
    visualization::Color::Yellow
  };
  ros::init(argc, argv, "liom_test_node");

  auto config_ = std::make_shared<PlannerConfig>();
  config_->vehicle.InitializeDiscs();
  std::vector<std::vector<double>> height_set;
  auto env = std::make_shared<Environment>(config_);
  auto planner_ = std::make_shared<FormationPlanner>(config_, env);
  std::vector<int> num_obs_set = {10, 20, 30, 40, 50};
  // std::vector<int> num_robot_set = {3, 4, 5};
  ros::NodeHandle nh;
  std::vector<ros::Publisher> path_pub_set;
  interactive_markers::InteractiveMarkerServer server_("/liom_obstacle");
  math::GenerateObstacle generateobs;
  std::vector<math::Pose> obstacle;
  std::vector<math::Polygon2d> polys, polys_orig, polys_inflat, polys_inflat_, polys_corridor;
  std::vector<std::vector<std::vector<std::vector<double>>>> hPolys_sets(num_robot);
  std::vector<std::vector<math::Vec2d>> corri_vertices_set;
  std::vector<double> height;
  std::vector<std::vector<math::Vec2d>> poly_vertices_set;
  std::vector<std::vector<std::vector<std::vector<double>>>> hyperparam_sets;
  std::vector<std::vector<std::vector<math::Vec2d>>> poly_vertices_sets;
  for (int i = 0; i < num_robot; i++) {
    ros::Publisher path_pub_temp = nh.advertise<nav_msgs::Path>("/liom_test_path" + std::to_string(i), 1, false);
    path_pub_set.push_back(path_pub_temp);
  }   
  // double inflated_radius;
  // config_->formation.calculateMinCircle(inflated_radius, 0.0, 0.0);
  // for (int obs_ind = 0; obs_ind < obstacle.size(); obs_ind++) {
  //   polys.push_back(math::Polygon2d(env->InflateObstacle(poly_vertices_set[obs_ind], inflated_radius)));
  // }
  iris::IRISProblem iris_problem(2);
  Eigen::MatrixXd obs = Eigen::MatrixXd::Zero(2, 4);
  for (int i = 0; i < polys.size(); i++) {
    obs << polys[i].points()[0].x(), polys[i].points()[1].x(), polys[i].points()[2].x(), polys[i].points()[3].x(),
            polys[i].points()[0].y(), polys[i].points()[1].y(), polys[i].points()[2].y(), polys[i].points()[3].y();
    iris_problem.addObstacle(obs);
  }
  visualization::Init(nh, "odom", "/liom_test_vis");
  VVCM vvcm;
  TrajectoryPoint start, goal;
  FullStates solution;
  ros::Rate r(10);

  auto start_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, [&](const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose) {
    double x = pose->pose.pose.position.x;
    double y = pose->pose.pose.position.y;

    double min_distance = DBL_MAX;
    int idx = -1;
    for(int i = 0; i < solution.states.size(); i++) {
      double distance = hypot(solution.states[i].x - x, solution.states[i].y - y);
      if(distance < min_distance) {
        min_distance = distance;
        idx = i;
      }
    }

    start = solution.states[idx];
  });
  int count_exp = -1;
  int count_factor = 0;
  bool success = false;
  bool solve_fail = false;
  bool show_cr = false;
  // double start_point_x = -35;
  // double start_point_y = 0;
  // double goal_point_x = 35;
  // double goal_point_y = 0;
  // // double start_point_x = -26.222;
  // // double start_point_y = -8.067;
  // // double goal_point_x = 24.108;
  // // double goal_point_y = 15.387;  
  // Eigen::Vector2d fc_start = {start_point_x, start_point_y};
  // Eigen::Vector2d fc_goal = {goal_point_x, goal_point_y};
  // std::vector<Eigen::Vector2d> start_pts;
  // std::vector<Eigen::Vector2d> goal_pts;
  // std::vector<TrajectoryPoint> start_set;
  // std::vector<TrajectoryPoint> goal_set;
  // generateRegularPolygon(start_point_x, start_point_y, vvcm.formation_radius, num_robot, start_pts, start_set);
  // generateRegularPolygon(goal_point_x, goal_point_y, vvcm.formation_radius, num_robot, goal_pts, goal_set);
  /* ablation study*/
  std::vector<std::vector<double>> pair_array;
  for (int i = -5; i < 6; i++) {
    for (int j = -5; j < 6; j++) {
      pair_array.push_back({double(i), double(j)});
    }
  }
  while(ros::ok()) { 
    // for (int robot_num_ind = 0; robot_num_ind < num_robot_set.size(); robot_num_ind++) {
      // int num_robot_ = num_robot_set[robot_num_ind];
      for (int obs_num_ind = 0; obs_num_ind < num_obs_set.size(); obs_num_ind++) {
        // int num_obs = num_obs_set[obs_num_ind];
        int num_obs = 70;
        for (int case_num = 0; case_num < 100; case_num++) {
          double start_point_x = -15;
          double start_point_y = 0;
          double goal_point_x = 15;
          double goal_point_y = 0;
          // double start_point_x = -26.222;
          // double start_point_y = -8.067;
          // double goal_point_x = 24.108;
          // double goal_point_y = 15.387;  
          Eigen::Vector2d fc_start = {start_point_x, start_point_y};
          Eigen::Vector2d fc_goal = {goal_point_x, goal_point_y};
          std::vector<Eigen::Vector2d> start_pts;
          std::vector<Eigen::Vector2d> goal_pts;
          std::vector<TrajectoryPoint> start_set;
          std::vector<TrajectoryPoint> goal_set;
          generateRegularPolygon(start_point_x, start_point_y, vvcm.formation_radius, num_robot, start_pts, start_set); // vvcm.foramtion_radius
          generateRegularPolygon(goal_point_x, goal_point_y, vvcm.formation_radius, num_robot, goal_pts, goal_set);
          // GenerateRandomObstacle(num_obs, height_set, obstacle, height, polys,polys_inflat, poly_vertices_set, generateobs);
          // ReadRandomObstacle(30, height_set, obstacle, height, polys,polys_inflat, poly_vertices_set, generateobs);
          DefineRandomObstacle(num_obs, height_set, obstacle, height, polys, polys_inflat, polys_inflat_, poly_vertices_set, generateobs);
          formation_planner::TopologyPRM topo_prm(config_, env);
          CoarsePathPlanner coarse_topo_path(config_, env);
          std::vector<FullStates> solution_set(num_robot);
          std::vector<FullStates> solution_set_opt;
          double best_tf = std::numeric_limits<double>::max();
          double topo_search_time = 0.0, filter_sort_time = 0.0;
          vector<double> solve_time_set, coarse_path_time_set;
          double solve_ocp_total = 0.0;
          int solve_success = 0;
          int iteration_num_inner = 0;
          int iteration_num_outter = 0;
          double e_max = 0.0;
          double e_avg = 0.0;
          double avg = 0.0; 
          double std = 0.0;
          env->polygons() = polys_inflat;
          env->heights() = height_set[0]; 
          int path_index = 0;
          std::vector<std::vector<formation_planner::math::Pose>> topo_paths;
          std::vector<formation_planner::FullStates> solution_sets;
          vector<vector<vector<Eigen::Vector2d>>> raw_paths_set;
          topo_prm.init(nh);
          list<GraphNode::Ptr>            graph;
          vector<vector<Eigen::Vector2d>> raw_paths, filtered_paths, select_paths;
          for (int i = 0; i < polys.size(); i++) {
            auto color = visualization::Color::Red;
            // color.set_alpha((0.8 - height_set[0][i]) / 0.8);
            // color.set_alpha(1- height[i]);
            visualization::PlotPolygon(polys[i], 0.05, color, i, "Obstacle"+  std::to_string(i));
          }
          visualization::Trigger();   
          double ratio = 1;
          while (select_paths.empty()) {
            /* Optional: PRM */
            topo_prm.findTopoPaths(fc_start, fc_goal, graph,
                                    raw_paths, filtered_paths, select_paths, topo_search_time, ratio);
            ratio *= 1.5;                   
          }
          /* Rewire the paths */
          for (int i = 0; i < num_robot; i++) {
            for (int j = 0; j < select_paths.size(); j++) {
              if (select_paths[j].empty()) continue;
              auto select_paths_disc = discretizePath(select_paths[j], 100);
              auto select_paths_rewired = RewiretPath(select_paths_disc, start_pts[i], goal_pts[i], env);
              select_paths[j] = select_paths_rewired;
            }
            raw_paths_set.push_back(select_paths);
          }
          // for (int ind = 0; ind < raw_paths_set.size(); ind++) {
          //   for (int i = 0; i < raw_paths_set[0].size(); i++) {
          //     vector<double> x_raw(raw_paths_set[ind][i].size()), y_raw(raw_paths_set[ind][i].size());
          //     for (int j  = 0; j < raw_paths_set[ind][i].size(); j++) {
          //       x_raw[j] = raw_paths_set[ind][i][j].x();
          //       y_raw[j] = raw_paths_set[ind][i][j].y();
          //     }
          //     colors[ind].set_alpha(0.2);
          //     visualization::Plot(x_raw, y_raw, 0.3, colors[ind], 1, "Raw Path" + std::to_string(path_index++));
          //     visualization::Trigger();
          //   }
          // }
          env->polygons() = polys;
          vector<vector<Pathset>> paths_sets(num_robot);
          std::vector<std::vector<int>> combinations_new;
          CalCombination(raw_paths_set, env, paths_sets, combinations_new, filter_sort_time);
          std::vector<std::vector<std::vector<double>>> corridors_sets;
          std::vector<std::vector<std::vector<std::vector<double>>>> hPolys_sets(num_robot);
          std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> keyPts;
          std::vector<std::vector<std::vector<math::Vec2d>>> corri_vertices_sets;
          double cost;
          show_cr = false;
          // for (int i = 0; i < combinations_new.size(); i++) {
          //   for (int ind = 0; ind < combinations_new[i].size(); ind++) {
          //       vector<double> x_raw(paths_sets[ind][combinations_new[i][ind]].path.size()), 
          //                      y_raw(paths_sets[ind][combinations_new[i][ind]].path.size());
          //     for (int j = 0; j < paths_sets[ind][combinations_new[i][ind]].path.size(); j++) {
          //         x_raw[j] = paths_sets[ind][combinations_new[i][ind]].path[j](0);
          //         y_raw[j] = paths_sets[ind][combinations_new[i][ind]].path[j](1);

          //     }
          //     colors[1].set_alpha(0.4);
          //     visualization::iteration_num_outterPlot(x_raw, y_raw, 0.5, colors[ind], 1, "Raw Path" + std::to_string(path_index++));
          //     visualization::Trigger();
          //   }
          // }
          ros::Time t1;
          t1 = ros::Time::now();
          #pragma omp parallel 
          {
            std::vector<FullStates> local_best_solution_set;
            double local_best_tf = std::numeric_limits<double>::max();
            
            #pragma omp for nowait
            // for (int i = 0; i < combinations_new.size(); i++) {
            int iter_outer_max = 5 > combinations_new.size() ? combinations_new.size() : 5;
            for (int i = 0; i < iter_outer_max; i++) {
              corridors_sets.clear();
              keyPts.clear();
              hPolys_sets.clear();
              corri_vertices_sets.clear();
              CalCorridors(paths_sets, combinations_new[i], env, polys_inflat, 3, hyperparam_sets, poly_vertices_sets);
              // for (int i = 0; i < corri_vertices_sets.size(); i++) {
              //     for (int j = 0; j < corri_vertices_sets[i].size(); j++) {
              //     auto color_poly = visualization::Color::Magenta;
              //     color_poly.set_alpha(0.1);
              //     visualization::PlotPolygon(math::Polygon2d(corri_vertices_sets[i][j]), 0.1, color_poly, 0, "Corridor" + std::to_string(i) + std::to_string(j));
              //   }
              // }
              // visualization::Trigger();  
              std::vector<FullStates> local_solution_set = solution_set;
              for (int ind_c = 0; ind_c < num_robot; ind_c++) {
                local_solution_set[ind_c].states.clear();
                local_solution_set[ind_c].tf = 0.1;
              }
              // if (raw_paths.size() >= 3) {
                // if(!planner_->Plan(solution, start_set[2], goal_set[2], iris_problem, solution, coarse_path_time, solve_time_leader, show_cr, corridors_sets[2])) {
                //   ROS_ERROR("re-plan trajectory optimization failed!");
                // }
              if(!planner_->Plan_fm(
                local_solution_set, start_set, goal_set, iris_problem, local_solution_set, coarse_path_time_set, 
                solve_time_set, show_cr, polys_inflat_, hyperparam_sets, path_pub_set, iteration_num_inner, cost, solve_success,
                e_max, e_avg, avg, std)) {
                #pragma omp critical 
                {
                  ROS_ERROR("re-plan trajectory optimization failed!");
                }
                iteration_num_outter++;
                continue;
              }
              else {
                double current_tf = local_solution_set[0].tf;
                if (current_tf < local_best_tf) {
                    local_best_tf = current_tf;
                    local_best_solution_set = local_solution_set;
                }
                iteration_num_outter++;
                break;
              }
            }
            
            #pragma omp critical 
            {
              if (local_best_tf < best_tf) {
                  best_tf = local_best_tf;
                  solution_set_opt = local_best_solution_set;
              }
            }
          }
          // vector<vector<double>> x_dis(20), y_dis(20);
          // if (raw_paths_dis_set.size() == 3) {
          //   for (int i = 0; i < 20; i++) {
          //     for (int j = 0; j < raw_paths_dis_set.size(); j++) {
          //       x_dis[i].push_back(raw_paths_dis_set[j][i].x());
          //       y_dis[i].push_back(raw_paths_dis_set[j][i].y());
          //     }
          //     auto color = visualization::Color::Grey;
          //     color.set_alpha(1);
          //     visualization::PlotPolygon(x_dis[i], y_dis[i], 0.5, color, 1, "Formation_dis" + std::to_string(i));
          //     visualization::Trigger();
          //   }
          // }
          // vector<vector<double>> xs(solution_sets.size()), ys(solution_sets.size());
          // for (int i = 0; i < solution_sets.size(); i++) {
          //   for (int j = 0; j < solution_sets[i].states.size(); j++) {
          //     xs[i].push_back(solution_sets[i].states[j].x);
          //     ys[i].push_back(solution_sets[i].states[j].y);
          //   }
          //   colors[i].set_alpha(1.0);
          //   visualization::Plot(xs[i], ys[i], 0.5, colors[i], 1, "Coarse Path" + std::to_string(i));
          //   visualization::Trigger();
          // }
          // const double PI = 3.14159265358979323846;
          // std::random_device rd;
          // std::mt19937 gen(rd());
          // std::uniform_real_distribution<> dis(0, 2 * PI);
          solve_ocp_total = (ros::Time::now() - t1).toSec();
          double distance = 0;
          double coarse_time = 0.0;
          double solve_time = 0.0;
          std::vector<double> new_vector(15);
          if (solution_set_opt.empty() || solve_success == 0) {
              new_vector[0] = 0.0; 
              new_vector[1] = 0.0;
              new_vector[2] = 0.0;
              new_vector[3] = 0.0; 
              new_vector[4] = 0.0; 
              new_vector[5] = 0; 
              new_vector[6] = 0.0; 
              new_vector[7] = 0.0; 
              new_vector[8] = 0.0; 
              new_vector[9] = 0.0; 
              new_vector[10] = 0.0; 
              new_vector[11] = 0.0; 
              new_vector[12] = 0.0;
              new_vector[13] = 0.0;
              new_vector[14] = 0.0; 
          }
          else {
            for (int i = 1; i < solution_set_opt[0].states.size(); i++) {
              distance += hypot(solution_set_opt[0].states[i].x - solution_set_opt[0].states[i-1].x, solution_set_opt[0].states[i].y - solution_set_opt[0].states[i-1].y);
            }
            for (int i = 0; i < coarse_path_time_set.size(); i++) {
              coarse_time += coarse_path_time_set[i];
            }
            for (int i = 0; i < solve_time_set.size(); i++) {
              solve_time += solve_time_set[i];
            }   
            new_vector[0] = distance * 2.02; 
            if (distance > 500) {
              ROS_ERROR("Adasd");
            }
            new_vector[1] = iteration_num_outter;
            new_vector[2] = iteration_num_inner;
            new_vector[3] = cost; 
            new_vector[4] = solution_set_opt[0].tf; 
            new_vector[5] = solve_success; 
            new_vector[6] = topo_search_time; 
            new_vector[7] = filter_sort_time; 
            new_vector[8] = coarse_time / iteration_num_outter; 
            new_vector[9] = solve_time / iteration_num_inner; 
            new_vector[10] = solve_ocp_total;
            new_vector[11] = e_max; 
            new_vector[12] = e_avg;
            new_vector[13] = avg;
            new_vector[14] = std; 
          }
          // writeVectorToYAML(new_vector, "/home/weijian/CPDOT/src/formation_planner/traj_result/icra_result/ablation_without_" + std::to_string(num_robot) + std::to_string(num_obs) + ".yaml");
          for (int j = 0; j < solution_set_opt.size(); j++) {
            DrawTrajectoryRviz(solution_set_opt[j], config_, j, path_pub_set[j]);
          }
        }
      }
    // }
    // for (int j = 0; j < solution_set_opt.size(); j++) {
    //   writeTrajectoryToYAML(solution_set_opt[j], "/home/weijian/CPDOT/src/formation_planner/traj_result/flexible_formation/" 
    //   + std::to_string(num_robot) + "/traj_" + std::to_string(num_robot) + std::to_string(j) + std::to_string(0) +".yaml");
    // }        
    // }
    // std::vector<std::vector<double>> topo_(3);
    // std::vector<std::vector<double>> relative_(3);
    // for (int i = 0; i < solution_set[0].states.size(); i++) {
    //   topo_[0].push_back((solution_set[0].states[i].x - solution_set[2].states[i].x) * (solution_set[2].states[i].y - solution_set[1].states[i].y) +
    //                     (solution_set[0].states[i].y - solution_set[2].states[i].y) * (solution_set[1].states[i].x - solution_set[2].states[i].x));
    //   topo_[1].push_back((solution_set[1].states[i].x - solution_set[0].states[i].x) * (solution_set[0].states[i].y - solution_set[2].states[i].y) +
    //                     (solution_set[1].states[i].y - solution_set[0].states[i].y) * (solution_set[2].states[i].x - solution_set[0].states[i].x));
    //   topo_[2].push_back((solution_set[2].states[i].x - solution_set[1].states[i].x) * (solution_set[1].states[i].y - solution_set[0].states[i].y) +
    //                     (solution_set[2].states[i].y - solution_set[1].states[i].y) * (solution_set[0].states[i].x - solution_set[1].states[i].x));
    // }
    // for (int i = 0; i < solution_set[0].states.size(); i++) {
    //   relative_[0].push_back(sqrt(pow(solution_set[1].states[i].x - solution_set[0].states[i].x, 2) + pow(solution_set[1].states[i].y - solution_set[0].states[i].y, 2)));
    //   relative_[1].push_back(sqrt(pow(solution_set[2].states[i].x - solution_set[1].states[i].x, 2) + pow(solution_set[2].states[i].y - solution_set[1].states[i].y, 2)));
    //   relative_[2].push_back(sqrt(pow(solution_set[0].states[i].x - solution_set[2].states[i].x, 2) + pow(solution_set[0].states[i].y - solution_set[2].states[i].y, 2)));
    // }
    ros::spinOnce();
    r.sleep();
  }

  ros::spin();
  return 0;
}