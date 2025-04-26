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

using namespace formation_planner;
// struct Point {
//     float x, y;
// };

float crossProduct(Point A, Point B, Point C) {
    return (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x);
}

bool isPointInTriangle(Point A, Point B, Point C, Point P) {
    float cross1 = crossProduct(A, B, P);
    float cross2 = crossProduct(B, C, P);
    float cross3 = crossProduct(C, A, P);

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
//         YAML::Node existing_data;
//         std::ifstream file_in(file_path);
//         if (file_in) {
//             existing_data = YAML::Load(file_in);
//             file_in.close();
//         }

//         YAML::Node new_data;
//         for (const auto& value : data) {
//             new_data.push_back(value);
//         }
//         existing_data.push_back(new_data);

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

double getRandomAngle() {
    return getRandomDouble(0, M_PI / 2);
}

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
std::vector<double>& height, std::vector<math::Polygon2d>& polys, std::vector<math::Polygon2d>& polys_inflat,
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
  // auto planner_ = std::make_shared<FormationPlanner>(config_, env);
  std::vector<int> num_obs_set = {10, 20, 30, 40, 50};
  // std::vector<int> num_robot_set = {3, 4, 5};
  ros::NodeHandle nh;
  std::vector<ros::Publisher> path_pub_set;
  interactive_markers::InteractiveMarkerServer server_("/liom_obstacle");
  math::GenerateObstacle generateobs;
  std::vector<math::Pose> obstacle;
  std::vector<math::Polygon2d> polys, polys_orig, polys_inflat, polys_corridor;
  std::vector<double> height;
  std::vector<std::vector<math::Vec2d>> poly_vertices_set;
  for (int i = 0; i < num_robot; i++) {
    ros::Publisher path_pub_temp = nh.advertise<nav_msgs::Path>("/liom_test_path" + std::to_string(i), 1, false);
    path_pub_set.push_back(path_pub_temp);
  }   
  int num_obs = 70;
  DefineRandomObstacle(num_obs, height_set, obstacle, height, polys,polys_inflat, poly_vertices_set, generateobs);
  iris::IRISProblem iris_problem(2);
  // Eigen::MatrixXd obs = Eigen::MatrixXd::Zero(2, 4);
    Eigen::MatrixXd obs(2,4);
  // for (int i = 0; i < num_obs; i++) {
  //   TrajectoryPoint obs_pt = generateRandomnObstacle();
  //   obstacle.push_back({obs_pt.x, obs_pt.y, obs_pt.theta});
  // }
  polys.push_back(math::Polygon2d({{-30.5, -17}, {-30.5, 17}, {-30, 17}, {-30, -17}}));
  polys.push_back(math::Polygon2d({{-30, 17}, {-30, 17.5}, {30, 17.5}, {30, 17}}));
  polys.push_back(math::Polygon2d({{30, 17}, {30.5, 17}, {30.5, -17}, {30, -17}}));
  polys.push_back(math::Polygon2d({{-30, -17}, {-30, -17.5}, {30, -17.5}, {30, -17}}));
  // polys.push_back(math::Polygon2d({{-9, 1}, {-8, -0.8}, {9.5, -0.9}, {10.5, 0.5}}));
  // polys.push_back(math::Polygon2d({{-23.220, 13.959}, {-16.601, 8.275}, {-3.334, 3}, {-4.206, 13.721}}));
  // polys.push_back(math::Polygon2d({{0.729, 15.238}, {0.544, 7.743}, {25.024, 9.113}, {24.791, 13.586}}));
  // polys.push_back(math::Polygon2d({{-26.055, -4.197}, {-21.176, -14.067}, {4.473, -13.498}, {5.976, -3.003}}));
  // polys.push_back(math::Polygon2d({{9.089, -6.939}, {8.933, -15.629}, {27.658, -13.353}, {27.989, -8.821}}));

  for (int i = 0; i < polys.size(); i++) {
    height.push_back(getRandomDouble(0.1, 0.5));
  }
  env->heights() = height;
  // writeVectorToYAML(height, "/home/weijian/CPDOT/src/formation_planner/traj_result/obs_height.yaml");
  // auto interactive_cb = [&](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &msg) {
  //   int idx = msg->marker_name.back() - '1';

  //   auto new_poly = polys[idx];
  //   new_poly.Move({ msg->pose.position.x, msg->pose.position.y });
  //   env->polygons().at(idx) = new_poly;
  // };
  // writeObstacleToYAML(poly_vertices_set, "/home/weijian/CPDOT/src/formation_planner/traj_result/obs.yaml");
  // for (int i = 0; i < polys.size(); i++) {
  //   polys_inflat.push_back(math::Polygon2d({
  //     {polys[i].points()[0].x() - 0.3, polys[i].points()[0].y() + 0.3}, 
  //     {polys[i].points()[1].x() - 0.3, polys[i].points()[1].y() - 0.3}, 
  //     {polys[i].points()[2].x() + 0.3, polys[i].points()[2].y() - 0.3}, 
  //     {polys[i].points()[3].x() + 0.3, polys[i].points()[3].y() + 0.3}
  //     }));
  // }
  env->polygons() = polys_inflat;
  for (int i = 0; i < polys.size(); i++) {
    obs << polys[i].points()[0].x(), polys[i].points()[1].x(), polys[i].points()[2].x(), polys[i].points()[3].x(),
            polys[i].points()[0].y(), polys[i].points()[1].y(), polys[i].points()[2].y(), polys[i].points()[3].y();
    iris_problem.addObstacle(obs);
  }
  
  // for(int i = 0; i < polys.size(); i++) {
  //   auto marker = CreateMarker(i+1, polys[i], 0.2, visualization::Color::Magenta);
  //   server_.insert(marker, interactive_cb);
  // }

  // server_.applyChanges();
    // Inflate a region inside a 1x1 box
  visualization::Init(nh, "odom", "/liom_test_vis");
  VVCM vvcm;
  TrajectoryPoint start, start1, start2, goal, goal1, goal2;
  FullStates solution, solution_car;
  FullStates solution_car_like1, solution_diff_drive1, solution_diff_drive2;
  double max_error1, max_error2;
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
  bool show_cr = true;
  double start_point_x = -15;
  double start_point_y = 0;
  double goal_point_x = 15;
  double goal_point_y = 0;
  double topo_search_time, filter_sort_time;
  std::vector<Eigen::Vector2d> start_pts;
  std::vector<Eigen::Vector2d> goal_pts;
  std::vector<TrajectoryPoint> start_set;
  std::vector<TrajectoryPoint> goal_set;
  generateRegularPolygon(start_point_x, start_point_y, vvcm.formation_radius, num_robot, start_pts, start_set); // vvcm.foramtion_radius
  generateRegularPolygon(goal_point_x, goal_point_y, vvcm.formation_radius, num_robot, goal_pts, goal_set);
  formation_planner::TopologyPRM topo_prm(config_, env);
  std::vector<FullStates> solution_set(num_robot);
  while(ros::ok()) { 
  int path_index = 0;
  std::vector<std::vector<formation_planner::math::Pose>> topo_paths;
  std::vector<formation_planner::FullStates> solution_sets;
  vector<vector<vector<Eigen::Vector2d>>> raw_paths_set;
    topo_prm.init(nh);
    list<GraphNode::Ptr>            graph;
    vector<vector<Eigen::Vector2d>> raw_paths, filtered_paths, select_paths;
    for (int i = 0; i < polys.size(); i++) {
      auto color = visualization::Color::Black;
      color.set_alpha(1- height[i]);
      visualization::PlotPolygon(polys[i], 0.1, color, i, "Obstacle"+  std::to_string(i));
    }
    visualization::Trigger();   
    // for (int i = 0; i < obstacle.size(); i++) {
    //   std::vector<double> new_vector = {obstacle[i].x(), obstacle[i].y(), obstacle[i].theta()};
    //   writeVectorToYAML(new_vector, "/home/weijian/CPDOT/src/formation_planner/traj_result/obstacle.yaml");
    // }

    for (int i = 0; i < num_robot; i++) {
      topo_prm.findTopoPaths(start_pts[i], goal_pts[i], graph,
                              raw_paths, filtered_paths, select_paths, topo_search_time, 1);
      raw_paths_set.push_back(select_paths);
    }
    for(int j = 0; j < raw_paths_set[0].size(); j++) {
      auto path = discretizePath(raw_paths_set[0][j], 40);
      for (int i = 0; i < path.size(); i++) {
        iris::IRISOptions options;
        iris_problem.setSeedPoint(Eigen::Vector2d(path[i](0), path[i](1)));
        iris::IRISRegion region = inflate_region(iris_problem, options);
        auto points = region.polyhedron.generatorPoints();
        std::vector<math::Vec2d> points_;
        for (const auto& pts : points) {
          math::Vec2d pts_temp(pts[0], pts[1]);
          points_.push_back(pts_temp);
        }
        auto color = colors[j];
        color.set_alpha(0.2);
        // visualization::PlotPolygon(math::Polygon2d(math::Box2d(box)), 0.05, color, i, "Corridor " + std::to_string(i));
        visualization::PlotPolygon(math::Polygon2d({points_}), 0.1, color, i*j, "Safe Corridor" + std::to_string(i*j));
      }
    }
    // for (int ind = 0; ind < raw_paths_set.size(); ind++) {
    //   for (int i = 0; i < 5; i++) {
    //     vector<double> x_raw(raw_paths_set[ind][i].size()), y_raw(raw_paths_set[ind][i].size());
    //     for (int j  = 0; j < raw_paths_set[ind][i].size(); j++) {
    //       x_raw[j] = raw_paths_set[ind][i][j].x();
    //       y_raw[j] = raw_paths_set[ind][i][j].y();
    //     }
    //     colors[ind].set_alpha(0.4);
    //     visualization::Plot(x_raw, y_raw, 0.5, colors[ind], 1, "Raw Path" + std::to_string(path_index++));
    //     visualization::Trigger();
    //   }
    // }
    // env->polygons() = polys;
    // vector<vector<Pathset>> paths_sets(num_robot);
    // std::vector<std::vector<int>> combinations_new;
    // CalCombination(raw_paths_set, env, paths_sets, combinations_new, filter_sort_time);
    // std::vector<std::vector<std::vector<double>>> corridors_sets;
    // double coarse_path_time, solve_time_leader;
    // show_cr = true;
    // for (int i = 0; i < 1; i++) {
    //   for (int ind = 0; ind < combinations_new[i].size(); ind++) {
    //       vector<double> x_raw(paths_sets[ind][combinations_new[i][ind]].path.size()), 
    //                      y_raw(paths_sets[ind][combinations_new[i][ind]].path.size());
    //     for (int j = 0; j < paths_sets[ind][combinations_new[i][ind]].path.size(); j++) {
    //         x_raw[j] = paths_sets[ind][combinations_new[i][ind]].path[j](0);
    //         y_raw[j] = paths_sets[ind][combinations_new[i][ind]].path[j](1);

    //     }
    //   auto color = colors[ind];
    //   // color.set_alpha(0.4);
    //   visualization::Plot(x_raw, y_raw, 0.5, color, 1, "Raw Path" + std::to_string(path_index++));
    //   visualization::Trigger();
    // }
    // }
    // for (int i = 0; i < combinations_new.size() - 1; i++) {
    //   corridors_sets.clear();
    //   CalCorridors(paths_sets, combinations_new[i], env, corridors_sets);
    //   for (int ind_c = 0; ind_c < num_robot; ind_c++) {
    //     solution_set[ind_c].states.clear();
    //     solution_set[ind_c].tf = 0.1;
    //   }
      // if (raw_paths.size() >= 3) {
        // if(!planner_->Plan(solution, start_set[2], goal_set[2], iris_problem, solution, coarse_path_time, solve_time_leader, show_cr, corridors_sets[2])) {
        //   ROS_ERROR("re-plan trajectory optimization failed!");
        // }

    //  for (int j = 0; j < solution_set[0].states.size(); j++) {
    //   for (int j = 0; j < solution_set.size(); j++) {
    //     DrawTrajectoryRviz(solution_set[j], config_, j, path_pub_set[j]);
    //   }     
    //   // }
    //   std::vector<std::vector<double>> topo_(3);
    //   std::vector<std::vector<double>> relative_(3);
    //   for (int i = 0; i < solution_set[0].states.size(); i++) {
    //     topo_[0].push_back((solution_set[0].states[i].x - solution_set[2].states[i].x) * (solution_set[2].states[i].y - solution_set[1].states[i].y) +
    //                       (solution_set[0].states[i].y - solution_set[2].states[i].y) * (solution_set[1].states[i].x - solution_set[2].states[i].x));
    //     topo_[1].push_back((solution_set[1].states[i].x - solution_set[0].states[i].x) * (solution_set[0].states[i].y - solution_set[2].states[i].y) +
    //                       (solution_set[1].states[i].y - solution_set[0].states[i].y) * (solution_set[2].states[i].x - solution_set[0].states[i].x));
    //     topo_[2].push_back((solution_set[2].states[i].x - solution_set[1].states[i].x) * (solution_set[1].states[i].y - solution_set[0].states[i].y) +
    //                       (solution_set[2].states[i].y - solution_set[1].states[i].y) * (solution_set[0].states[i].x - solution_set[1].states[i].x));
    //   }
    //   for (int i = 0; i < solution_set[0].states.size(); i++) {
    //     relative_[0].push_back(sqrt(pow(solution_set[1].states[i].x - solution_set[0].states[i].x, 2) + pow(solution_set[1].states[i].y - solution_set[0].states[i].y, 2)));
    //     relative_[1].push_back(sqrt(pow(solution_set[2].states[i].x - solution_set[1].states[i].x, 2) + pow(solution_set[2].states[i].y - solution_set[1].states[i].y, 2)));
    //     relative_[2].push_back(sqrt(pow(solution_set[0].states[i].x - solution_set[2].states[i].x, 2) + pow(solution_set[0].states[i].y - solution_set[2].states[i].y, 2)));
      // }
    // }
    ros::spinOnce();
    r.sleep();
  }
  // }
  ros::spin();
  return 0;
}