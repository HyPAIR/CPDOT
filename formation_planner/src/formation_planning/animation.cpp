/**
 * file animation.cpp
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief animation
 * data 2023-11-22
 * 
 * @copyright Copyroght(c) 2023
*/
#include <iostream>
#include <ros/package.h>
#include <ros/ros.h>
#include "formation_planner/formation_planner.h"
#include "formation_planner/math/generate_obs.h"
#include "formation_planner/visualization/plot.h"
#include "formation_planner/yaml_all.h"
#include "traj_tracking/matplotlibcpp.h"
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include "formation_planner/forward_kinematics.h"

using namespace formation_planner;
using namespace forward_kinematics;
namespace plt = matplotlibcpp;

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

void DrawTrajectoryRviz(const FullStates sol_traj,std::shared_ptr<formation_planner::PlannerConfig> config,
      int robot_index, ros::Publisher path_pub) {
  std::vector<formation_planner::visualization::Color> colors = {
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
    color.set_alpha(0.2);
    // color.set_alpha(0.4);
    visualization::PlotPolygon(math::Polygon2d(box), 0.1, color, i, robot_name);
  }
  path_pub.publish(msg);
  visualization::Trigger();
}

void DrawTrajectoryRvizReal(const std::vector<double> x, const std::vector<double> y,
const std::vector<double> theta, std::shared_ptr<formation_planner::PlannerConfig> config,
      int robot_index) {
  std::vector<formation_planner::visualization::Color> colors = {
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

  for(size_t i = 0; i < x.size(); i+=100) {
    math::Pose pose(x[i], y[i], theta[i]);
    auto box = config->vehicle.GenerateBox(pose);
    auto color = colors[robot_index];
    color.set_alpha(0.2);
    // color.set_alpha(0.4);
    visualization::PlotPolygon(math::Polygon2d(box), 0.1, color, i, robot_name);
  }
  visualization::Trigger();
}

// void DrawTrajectoryRviz(const FullStates sol_traj, std::shared_ptr<formation_planner::PlannerConfig> config,
//       int robot_index, ros::Publisher path_pub, double inflat) {
//   for (int i = 0; i < sol_traj.states.size(); i++) {
//     auto x0_disc = config->vehicle.GetVertexPositions(sol_traj.states[i].x, sol_traj.states[i].y, sol_traj.states[i].theta, inflat);
//     std::vector<double> x1, y1;
//     x1.push_back(x0_disc[0]);x1.push_back(x0_disc[2]);x1.push_back(x0_disc[4]);x1.push_back(x0_disc[6]);x1.push_back(x0_disc[0]);
//     y1.push_back(x0_disc[1]);y1.push_back(x0_disc[3]);y1.push_back(x0_disc[5]);y1.push_back(x0_disc[7]);y1.push_back(x0_disc[1]);
//     auto color = visualization::Color::White;
//     color.set_alpha(0.1);
//     visualization::Plot(x1, y1, 0.2, color, i, "spatial_envelopes" + std::to_string(robot_index));
//   }
//   const std::string robot_name = "Footprint_" + std::to_string(robot_index);
//   for(int i = 0; i < 1e3; i++) {
//     visualization::Delete(i, robot_name);
//   }
//   visualization::Trigger();

//   nav_msgs::Path msg;
//   msg.header.frame_id = "map";
//   msg.header.stamp = ros::Time::now();
//   for(size_t i = 0; i < sol_traj.states.size(); i++) {
//     geometry_msgs::PoseStamped pose;
//     pose.header = msg.header;
//     pose.pose.position.x = sol_traj.states[i].x;
//     pose.pose.position.y = sol_traj.states[i].y;
//     pose.pose.orientation = tf::createQuaternionMsgFromYaw(sol_traj.states[i].theta);
//     msg.poses.push_back(pose);

//     auto box = config->vehicle.GenerateBox(sol_traj.states[i].pose());
//     auto color = robot_index > 0 ? visualization::Color::Yellow : visualization::Color::White;
//     color.set_alpha(0.4);
//     visualization::PlotPolygon(math::Polygon2d(box), 0.05, color, i, robot_name);
//   }
//   path_pub.publish(msg);
//   visualization::Trigger();
// }

FullStates traj2fullstates(Trajectory_temp traj) {
    FullStates solution;
    solution.states.resize(traj.size());
    solution.tf = traj[traj.size() - 1].t;
    for (int i = 0; i < traj.size(); i++) {
        solution.states[i].x = traj[i].x;
        solution.states[i].y = traj[i].y;
        solution.states[i].theta = traj[i].theta;
        solution.states[i].v = traj[i].v;
        solution.states[i].phi = traj[i].phi;
        solution.states[i].a = traj[i].a;
        solution.states[i].omega = traj[i].omega;
    }
    return solution;
}

void PlotPose(std::shared_ptr<formation_planner::PlannerConfig> config, int robot_index, int time_step, Trajectory_temp traj, std::vector<double> robot_type, const double& xo, const double& yo) {
    std::vector<double> xo_set, yo_set;
    std::vector<double> xo_, yo_;
    if (time_step > traj.size() - 1) {
        time_step = traj.size() - 1;
    }
    const std::string robot_name = "Footprint_" + std::to_string(robot_index);
    // visualization::Delete(i, robot_name);
    // visualization::Trigger();
    double theta = traj[time_step].theta;
    if (theta < -M_PI) {
        while (theta < -M_PI) {
            theta += 2 * M_PI;
        }
    }
    else if (theta > M_PI) {
        while (theta > M_PI) {
            theta -= 2 * M_PI;
        }
    }
    math::Pose robot_pos(traj[time_step].x, traj[time_step].y, traj[time_step].theta);
    xo_set.push_back(robot_pos.x());
    xo_set.push_back(xo);
    xo_set.push_back(robot_pos.x());
    yo_set.push_back(robot_pos.y());
    yo_set.push_back(yo);
    yo_set.push_back(robot_pos.y());
    visualization::Plot(xo_set,yo_set, 0.3, visualization::Color::Green, 1, "cable" + std::to_string(robot_index));
    auto box = config->vehicle.GenerateBox(robot_pos);
    auto color = robot_type[robot_index] > 0 ? visualization::Color::Red : visualization::Color::Cyan;
    // color.set_alpha(0.4);
    xo_.push_back(xo+0.3); xo_.push_back(xo+0.3); xo_.push_back(xo-0.3); xo_.push_back(xo-0.3); xo_.push_back(xo+0.3);
    yo_.push_back(yo+0.3); yo_.push_back(yo-0.3); yo_.push_back(yo-0.3); yo_.push_back(yo+0.3); yo_.push_back(yo+0.3);
    visualization::Plot(xo_, yo_, 0.3, visualization::Color::Magenta, 1, "point_s");
    visualization::Trigger();
    visualization::PlotPolygon(math::Polygon2d(box), 0.1, color, 1, robot_name);
    visualization::Trigger();
    // visualization::Plot(xs, ys, 0.5, visualization::Color::Magenta, i, robot_name_ta);
}

void PlotPoseReal(std::shared_ptr<formation_planner::PlannerConfig> config, int robot_index, int time_step, std::vector<double> x, std::vector<double> y,
std::vector<double> theta, std::vector<double> robot_type, const double& xo, const double& yo) {
    std::vector<double> xo_set, yo_set;
    std::vector<double> xo_, yo_;
    if (time_step > x.size() - 1) {
        time_step = x.size() - 1;
    }
    const std::string robot_name = "Footprint_" + std::to_string(robot_index);
    // visualization::Delete(i, robot_name);
    // visualization::Trigger();
    if (theta[time_step] < -M_PI) {
        while (theta[time_step] < -M_PI) {
            theta[time_step] += 2 * M_PI;
        }
    }
    else if (theta[time_step] > M_PI) {
        while (theta[time_step] > M_PI) {
            theta[time_step] -= 2 * M_PI;
        }
    }
    math::Pose robot_pos(x[time_step], y[time_step], theta[time_step]);
    xo_set.push_back(robot_pos.x());
    xo_set.push_back(xo);
    xo_set.push_back(robot_pos.x());
    yo_set.push_back(robot_pos.y());
    yo_set.push_back(yo);
    yo_set.push_back(robot_pos.y());
    visualization::Plot(xo_set,yo_set, 0.1, visualization::Color::Green, 1, "cable" + std::to_string(robot_index));
    auto box = config->vehicle.GenerateBox(robot_pos);
    auto color = robot_type[robot_index] > 0 ? visualization::Color::Red : visualization::Color::Cyan;
    // color.set_alpha(0.4);
    xo_.push_back(xo+0.3); xo_.push_back(xo+0.3); xo_.push_back(xo-0.3); xo_.push_back(xo-0.3); xo_.push_back(xo+0.3);
    yo_.push_back(yo+0.3); yo_.push_back(yo-0.3); yo_.push_back(yo-0.3); yo_.push_back(yo+0.3); yo_.push_back(yo+0.3);
    visualization::Plot(xo_, yo_, 0.3, visualization::Color::Magenta, 1, "point_s");
    visualization::Trigger();
    visualization::PlotPolygon(math::Polygon2d(box), 0.1, color, 1, robot_name);
    visualization::Trigger();
    // visualization::Plot(xs, ys, 0.5, visualization::Color::Magenta, i, robot_name_ta);
}

int main(int argc, char* argv[]) {
    auto config_ = std::make_shared<PlannerConfig>();
    auto env = std::make_shared<Environment>(config_);
    VVCM vvcm;
    FormationPlanner llp(config_, env);
    int traj_index = 1000;
    std::vector<std::vector<double>> pos_3d;
    std::vector<std::vector<double>> pos_2d; 
    std::vector<std::vector<int>> taut_set;
    std::vector<std::vector<double>> vertice_set;
    std::vector<std::vector<double>> robot_pos_set;
    generateRegularPolygon(1.66, num_robot, vertice_set);
    // generateRegularPolygon(vvcm.formation_radius, num_robot, vertice_set);
    ForwardKinematics fk_test(num_robot, vvcm.zr, vertice_set, robot_pos_set);
	std::vector<Trajectory_temp> traj_set;
    std::vector<FullStates> fs_set;
    std::vector<FullStates> solution_set;
    std::vector<double> height_cons;
    FullStates solution1, solution2, solution3, solution4, solution5, solution6;
    std::string package_path = ros::package::getPath("formation_planner");
    std::string file_front = package_path + "/traj_result/flexible_formation/" + std::to_string(num_robot) + "/";
    std::string file_back = ".yaml";
    int traj_ind = 0;
    for (int i = 0; i < num_robot; i++) {
        auto traj_temp = generate_traj(file_front + "traj_" + std::to_string(num_robot)+ std::to_string(i) + std::to_string(traj_index) + ".yaml");
        traj_set.push_back(traj_temp); 
        auto fs_temp = traj2fullstates(traj_temp);
        fs_set.push_back(fs_temp);
    }
    std::vector<double> robot_type = {1, 1, 1, 1, 1, 1, 1, 1, 1};
    // std::vector<std::vector<int>> priority_set = generate_priority_set(file_front + "priority.yaml");
    ros::init (argc, argv, "swarm_traj_planner_rbp");
    ros::NodeHandle nh( "~" );
    ros::Rate rate(20);
    int j = 0;
    visualization::Init(nh, "odom", "/liom_test_vis");
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("cisualization_marker", 1);
	math::GenerateObstacle generateobs;
  	std::vector<math::Pose> obstacle;
	std::vector<math::Polygon2d> polys_orig, polys;
	std::vector<std::vector<math::Vec2d>> poly_vertices_set;
    bool plot_traj = false;

    std::vector<ros::Publisher> path_pub_set;
    ros::Publisher path_pub_car1 = nh.advertise<nav_msgs::Path>("/liom_test_path_car1", 1, false);
    ros::Publisher path_pub_car2 = nh.advertise<nav_msgs::Path>("/liom_test_path_car2", 1, false);
    ros::Publisher path_pub_car3 = nh.advertise<nav_msgs::Path>("/liom_test_path_car3", 1, false);
    ros::Publisher path_pub_car4 = nh.advertise<nav_msgs::Path>("/liom_test_path_car4", 1, false);
    ros::Publisher path_pub_car5 = nh.advertise<nav_msgs::Path>("/liom_test_path_car5", 1, false);
    path_pub_set.push_back(path_pub_car1); path_pub_set.push_back(path_pub_car2); path_pub_set.push_back(path_pub_car3); path_pub_set.push_back(path_pub_car4); path_pub_set.push_back(path_pub_car5);

    std::vector<std::vector<double>> obs_list = readVectorsFromYAML(package_path + "/traj_result/obstacle_30.yaml");
    auto height_set = readVectorsFromYAML(package_path + "/traj_result/obs_height_30.yaml");
    // std::vector<formation_planner::visualization::Vector> height_set = {{0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.6, 0.79, 0.5}};
    // obstacle.push_back({-15, 3.5, 0});
    // obstacle.push_back({-15, -3.5, 0});
    // obstacle.push_back({-0.361, 4.626, 0.25});
    // obstacle.push_back({4.930, -3.882, 0.3});

    // obstacle.push_back({-5, 0, 0});
    // obstacle.push_back({0, 0, 0});
    // obstacle.push_back({10, 0, 0});

    // obstacle.push_back({0, -6, 0});
    // obstacle.push_back({ 0, 6, 0});
    // obstacle.push_back({12, -8, 0});
    // obstacle.push_back({ 12, 8, 0});
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

    // std::vector<int> obs_type_set = {1, 1, 3, 3, 6, 6};
    // for (int obs_ind = 0; obs_ind < obstacle.size(); obs_ind++) {
    //     std::vector<math::Vec2d> poly_vertices;
    //     math::Vec2d centre(obstacle[obs_ind].x(), obstacle[obs_ind].y());
    //     if (obs_ind < 4) {
    //         poly_vertices = generateobs.rotatePoint(centre, obstacle[obs_ind].theta(), false, obs_type_set[obs_ind]);
    //     }
    //     else {
    //         poly_vertices = generateobs.rotatePoint(centre, obstacle[obs_ind].theta(), true, obs_type_set[obs_ind]);
    //     }
    //     poly_vertices_set.push_back(poly_vertices);
    //     polys.push_back(math::Polygon2d(poly_vertices));
    // }
    for (int i = 0; i < obs_list.size(); i++) {
        obstacle.push_back({obs_list[i][0], obs_list[i][1], obs_list[i][2]});
    }
    for (int obs_ind = 0; obs_ind < obstacle.size(); obs_ind++) {
        math::Vec2d centre(obstacle[obs_ind].x(), obstacle[obs_ind].y());
        std::vector<math::Vec2d> poly_vertices;
        poly_vertices = generateobs.rotatePoint(centre, obstacle[obs_ind].theta(), false, 1);

        poly_vertices_set.push_back(poly_vertices);
        polys.push_back(math::Polygon2d(poly_vertices));
    }
    // std::vector<YAML::Node> config;
    std::vector<std::vector<double>> x(num_robot), x_rot_set(num_robot);
    std::vector<std::vector<double>> y(num_robot), y_rot_set(num_robot);
    std::vector<std::vector<double>> theta(num_robot), theta_rot_set(num_robot);
    // for (int i = 0; i < num_robot; i++) {
    //     config.push_back(YAML::LoadFile("/home/weijian/CPDOT/src/formation_planner/traj_result/flexible_formation/" 
    //             + std::to_string(num_robot) + "/traj_real" + std::to_string(num_robot) + std::to_string(i) +".yaml"));
    //     if (config[i]["x"] && config[i]["y"]) {
    //         for (const auto& value : config[i]["x"]) {
    //             x[i].push_back(value.as<double>());
    //         }
    //         for (const auto& value : config[i]["y"]) {
    //             y[i].push_back(value.as<double>());
    //         }
    //     } 
    // }
    YAML::Node config_1 = YAML::LoadFile("/home/weijian/Downloads/data/1/positions_actual_1.yaml");
    YAML::Node config_2 = YAML::LoadFile("/home/weijian/Downloads/data/1/positions_actual_2.yaml");
    YAML::Node config_3 = YAML::LoadFile("/home/weijian/Downloads/data/1/positions_actual_3.yaml");
    std::vector< YAML::Node> config;
    config = {config_1, config_2, config_3};
    std::vector<std::vector<double>> start_pts = {{-0.664, -0.067, 1.6}, {-1.512, 0.368, 1.6}, {-1.512, -0.628, 1.6}};

    // std::vector<std::vector<double>> x(3), y(3);
    for (int i = 0; i < 3; i++) {
        if (config[i]["x"]) {
            for (const auto& value : config[i]["x"]) {
                x[i].push_back(value.as<double>());
            }
        }

        if (config[i]["y"]) {
            for (const auto& value : config[i]["y"]) {
                y[i].push_back(value.as<double>());
            }
        }

        if (config[i]["theta"]) {
            for (const auto& value : config[i]["theta"]) {
                theta[i].push_back(value.as<double>());
            }
        }
    }
    for (int i = 0; i < 3; i++) {
        double x0 = start_pts[i][0];
        double y0 = start_pts[i][1];
        double theta0 = start_pts[i][2];
        double x_start = x[i][0];
        double y_start = y[i][1];
        double theta_start = theta[i][2];
        double dx = x0 - x_start;
        double dy = y0 - y_start;
        double dtheta = theta0 - theta_start;
        for (int j = 0; j < x[0].size(); j++){

            double x_new = x[i][j] + dx;
            double y_new = y[i][j] + dy;
            double x_rot = (x_new - x0) * cos(dtheta) - (y_new - y0) * sin(dtheta) + x0;
            double y_rot = (x_new - x0) * sin(dtheta) + (y_new - y0) * cos(dtheta) + y0;
            double theta_rot = theta[i][2] + dtheta;
            x_rot_set[i].push_back(x_rot);
            y_rot_set[i].push_back(y_rot);
            theta_rot_set[i].push_back(theta_rot);
        }
    }
    std::vector<double> x_, y_, y__;
    std::vector<double> xo, yo;
    for (int i = 0; i < x[0].size(); i++) {
        robot_pos_set.clear();
        pos_2d.clear();
        pos_3d.clear();
        taut_set.clear();
        double po_x = 0.0, po_y = 0.0, po_z = 0.0;
        for (int j = 0; j < x.size(); j++) {
            robot_pos_set.push_back({x[j][i], y[j][i]});
        }
        fk_test.SolveFk(vertice_set, robot_pos_set, pos_3d, pos_2d, taut_set, 1.045);
        for (int po_count = 0; po_count < pos_3d.size(); po_count++) {
            po_x += pos_3d[po_count][0];
            po_y += pos_3d[po_count][1];
            po_z += pos_3d[po_count][2];
        }
        xo.push_back(po_x / pos_3d.size());
        yo.push_back(po_y / pos_3d.size());
        y_.push_back(po_z / pos_3d.size() + 0.65);
        x_.push_back(0.01 * i);
    }
 
    env->polygons() = polys;
    env->heights() = height_set[0];
    // int index_max = traj_set[0].size();
    int index_max = x[0].size();
    llp.GenerateHeightCons(fs_set, env, height_cons);
    writeVectorToYAML(x_, package_path + "/traj_result/flexible_formation/" + std::to_string(num_robot) + "/time_step_icra1.yaml");
    writeVectorToYAML(y_, package_path + "/traj_result/flexible_formation/" + std::to_string(num_robot) + "/height_obj_icra1.yaml");   
    writeVectorToYAML(height_cons, package_path + "/traj_result/flexible_formation/" + std::to_string(num_robot) + "/height_obs_icra1.yaml"); 
    while (ros::ok()) {
        // visualization::Trigger();
        // if (!plot_traj) {    
        // for (int traj_ind = 0; traj_ind < traj_set.size(); traj_ind++) {
        for (int traj_ind = 0; traj_ind < x.size(); traj_ind++) {
            // auto solution = traj2fullstates(traj_set[traj_ind]);
            DrawTrajectoryRvizReal(x_rot_set[traj_ind], y_rot_set[traj_ind], theta_rot_set[traj_ind], config_, traj_ind);
        }
        visualization::Trigger();
        // for (int i = 0; i < polys.size(); i++) {
        //     auto color = visualization::Color::Black;
        //     color.set_alpha(1- height_set[0][i]);
        //     visualization::PlotPolygon(polys[i], 0.1, color, i, "Obstacle"+  std::to_string(i));
        // }
		for (int i = 0; i < traj_set.size(); i++) {
            PlotPose(config_, i, j, traj_set[i], robot_type, xo[j], yo[j]);
		}
  		visualization::Trigger();
        j+=100;
        if (j == index_max) {
            break;
        }
        ros::spinOnce();
        rate.sleep();	
	}
    plt::clf();
    plt::xlabel("time step");
    plt::ylabel("height of the object/m");
    plt::plot(x_, y_);
    // plt::plot(obs1_x, obs1_y);
    // plt::plot(x_, y__);
    plt::show();
	return 0;
}

// ps aux | grep ros |  awk '{print $2}' | xargs kill -9; ps aux | grep rviz |  awk '{print $2}' | xargs kill -9