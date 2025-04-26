//
// Created by weijian on 17/11/23.
//

#include "formation_planner/environment.h"
#include <costmap_2d/cost_values.h>

namespace formation_planner {

bool Environment::CheckBoxCollision(double time, const math::AABox2d &box) const {
  // TODO: reimplement using R-Tree
  for(auto &polygon: polygons_) {
    if(polygon.HasOverlap(math::Box2d(box))) {
      return true;
    }
  }

  for(auto &point: points_) {
    if(box.IsPointIn(point)) {
       return true;
    }
  }

  return false;
}

bool Environment::CheckPoseCollision(double time, math::Pose pose) const {
  auto discs = config_->vehicle.GetDiscPositions(pose.x(), pose.y(), pose.theta());
  double wh = config_->vehicle.disc_radius * 2;
  for(int i = 0; i < discs.size() / 2; i++) {
    if(CheckBoxCollision(time, math::AABox2d({discs[i * 2], discs[i * 2 + 1]}, wh, wh))) {
      return true;
    }
  }

  return false;
}

bool Environment::CheckVerticeCollision(double time, math::Pose pose) const {
  auto f_centre = config_->vehicle.GetFormationCentre(pose.x(), pose.y(), pose.theta());
  if(CheckBoxCollision(time, math::AABox2d({f_centre[0], f_centre[1]}, 2.0 + 1.2, config_->vehicle.offset + 1.2))) {
    return true;
  }
  return false;
}


bool Environment::CheckHomotopyConstraints(double time, math::Pose pose, std::vector<std::vector<std::vector<double>>> hyperparam_set) const {
  bool polygon_found = false;
  int i = 0;
  while (!polygon_found) {
    bool cons_violated = false;
    for (int k = 0; k < hyperparam_set[i].size(); k++) {
      if (pose.x() * hyperparam_set[i][k][0] + pose.y() * hyperparam_set[i][k][1] - hyperparam_set[i][k][2] > 0) {
        cons_violated = true;
        break;
      }
    }
    if (!cons_violated) {
      return true;
    }
    if (i == hyperparam_set.size() - 1) {
      break;
    }
    i++;
  }
  return false;
}

bool Environment::CheckSpatialEnvelopes(double time, math::Pose pose) const {
  auto f_centre = config_->vehicle.GetFormationCentre(pose.x(), pose.y(), pose.theta());
  if(CheckBoxCollision(time, math::AABox2d({f_centre[0], f_centre[1]}, sqrt(2) * (2.0 + 1.2), sqrt(2) * (config_->vehicle.offset + 1.2)))) {
    return true;
  }
  return false;
}

void Environment::UpdateCostmapObstacles(const costmap_2d::Costmap2D *costmap) {
  points_.clear();
  // 遍历每一个网格
  for(int i = 0; i < costmap->getSizeInCellsX()-1; i++) {
    for(int j = 0; j < costmap->getSizeInCellsY()-1; j++) {
      if(costmap->getCost(i,j) == costmap_2d::LETHAL_OBSTACLE) {
        double obs_x, obs_y;
        costmap->mapToWorld(i,j, obs_x, obs_y);
        points_.emplace_back(obs_x, obs_y);
      }
    }
  }
}

bool Environment::GenerateCorridorBox(double time, double x, double y, double radius, math::AABox2d &result) const {
  double ri = radius;
  // 或许应该是math::AABox2d bound({x-ri, y-ri}, {x+ri, y+ri});
  math::AABox2d bound({-ri, -ri}, {ri, ri});

  if(CheckBoxCollision(time, bound)) {
    // initial condition not satisfied, involute to find feasible box
    int inc = 4;
    double real_x, real_y;

    do {
      int iter = inc / 4;
      uint8_t edge = inc % 4;

      real_x = x;
      real_y = y;
      if(edge == 0) {
        real_x = x - iter * 0.05;
      } else if(edge == 1) {
        real_x = x + iter * 0.05;
      } else if(edge == 2) {
        real_y = y - iter * 0.05;
      } else if(edge == 3) {
        real_y = y + iter * 0.05;
      }

      inc++;
      bound = math::AABox2d({real_x-ri, real_y-ri}, {real_x+ri, real_y+ri});
    } while(CheckBoxCollision(time, bound) && inc < config_->corridor_max_iter);
    if(inc > config_->corridor_max_iter) {
      return false;
    }

    x = real_x;
    y = real_y;
  }

  int inc = 4;
  std::bitset<4> blocked;
  double incremental[4] = {0.0};
  double step = radius * 0.2;

  do {
    int iter = inc / 4;
    uint8_t edge = inc % 4;
    inc++;

    if (blocked[edge]) continue;

    incremental[edge] = iter * step;

    math::AABox2d test({-ri - incremental[0], -ri - incremental[2]},
                 {ri + incremental[1], ri + incremental[3]});

    if (CheckBoxCollision(time, test.Offset({x, y})) || incremental[edge] >= config_->corridor_incremental_limit) {
      incremental[edge] -= step;
      blocked[edge] = true;
    }
  } while (!blocked.all() && inc < config_->corridor_max_iter);
  if (inc > config_->corridor_max_iter) {
    return false;
  }

  result = {{x - incremental[0], y - incremental[2]},
            {x + incremental[1], y + incremental[3]}};
  return true;
}

std::vector<math::Vec2d> Environment::InflateObstacle(const std::vector<math::Vec2d>& vertices, double inflationRadius) {
    std::vector<math::Vec2d> inflatedObstacle;
    std::vector<math::Vec2d> inflatedPolygon;

    // Iterate over each vertex of the polygon
    for (size_t i = 0; i < vertices.size(); i++) {
        // Get the current, previous, and next vertices
        math::Vec2d currentVertex = vertices[i];
        math::Vec2d prevVertex = vertices[(i + vertices.size() - 1) % vertices.size()];
        math::Vec2d nextVertex = vertices[(i + 1) % vertices.size()];

        // Calculate the direction vectors of the adjacent edges
        double prevEdgeX = currentVertex.x() - prevVertex.x();
        double prevEdgeY = currentVertex.y() - prevVertex.y();
        double nextEdgeX = nextVertex.x() - currentVertex.x();
        double nextEdgeY = nextVertex.y() - currentVertex.y();

        // Normalize the direction vectors
        double prevEdgeLength = std::sqrt(prevEdgeX * prevEdgeX + prevEdgeY * prevEdgeY);
        double nextEdgeLength = std::sqrt(nextEdgeX * nextEdgeX + nextEdgeY * nextEdgeY);
        prevEdgeX /= prevEdgeLength;
        prevEdgeY /= prevEdgeLength;
        nextEdgeX /= nextEdgeLength;
        nextEdgeY /= nextEdgeLength;

        // Calculate the perpendicular vectors of the adjacent edges
        double prevPerpendicularX = -prevEdgeY;
        double prevPerpendicularY = prevEdgeX;
        double nextPerpendicularX = -nextEdgeY;
        double nextPerpendicularY = nextEdgeX;

        // Calculate the inflated vertices
        double prevX = currentVertex.x() + inflationRadius * (prevPerpendicularX);
        double prevY = currentVertex.y() + inflationRadius * (prevPerpendicularY);
        double nextX = currentVertex.x() + inflationRadius * (nextPerpendicularX);
        double nextY = currentVertex.y() + inflationRadius * (nextPerpendicularY);
        // Define the coefficients of the equation system
        double a1 = prevX - currentVertex.x();
        double b1 = prevY - currentVertex.y();
        double c1 = prevX * (prevX - currentVertex.x()) + prevY * (prevY - currentVertex.y());
        double a2 = nextX - currentVertex.x();
        double b2 = nextY - currentVertex.y();
        double c2 = nextX * (nextX - currentVertex.x()) + nextY * (nextY - currentVertex.y());
        // Solve the equation system
        EquationSystemResult result = SolveEquationSystem(a1, b1, c1, a2, b2, c2);

        // Add the inflated vertex to the inflated polygon
        inflatedPolygon.push_back({result.x, result.y});
    }

    return inflatedPolygon;
}

Environment::EquationSystemResult Environment::SolveEquationSystem(double a1, double b1, double c1, double a2, double b2, double c2) {
    double determinant = a1 * b2 - a2 * b1;

    Environment::EquationSystemResult result;
    result.hasSolution = false;

    if (determinant != 0) {
        result.hasSolution = true;
        result.x = (c1 * b2 - c2 * b1) / determinant;
        result.y = (a1 * c2 - a2 * c1) / determinant;
    }

    return result;
}

void Environment::generateSFC(const std::vector<Eigen::Vector2d>& path,
                    const std::vector<math::Polygon2d>& obstacles,
                    const double bbox_width,
                    // std::vector<Eigen::MatrixXd>& hPolys,
                    std::vector<std::vector<std::vector<double>>>& hyperparam_set,
                    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>& keyPts,
                    std::vector<std::vector<math::Vec2d>>& poly_vertices_set
                    ) {
      assert(path.size() > 1);
      std::vector<Eigen::MatrixXd> hPolys;
      vec_Vec2f obs_pc;
      EllipsoidDecomp2D decomp_util;
      vec_E<Polyhedron2D> decompPolys;
      int path_len = 0;
      for (int i = 0; i < path.size(); i++) {
        if (path[i](0) == 0 && path[i](1) == 0) {
          path_len = i;
          break;
        }
      }
      path_len = path.size();
      keyPts.clear();
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
            std::vector<Eigen::Vector2d> interpolatedPoints = interpolatePoints(start, end, 20);
            for (int i = 0; i < interpolatedPoints.size(); i++) {
              obs_pc.push_back(Vec2f(interpolatedPoints[i](0), interpolatedPoints[i](1)));
            }
          }
      }

      for (int idx = 0; idx < path_len; ++idx) {
          if(idx == 0 || idx == path_len - 1) {
            decomp_util.set_local_bbox(Eigen::Vector2d(1.0, 1.0));
          }
          decomp_util.set_local_bbox(Eigen::Vector2d(bbox_width, bbox_width));
          // Generate corridor for each waypoint
          int next_idx = idx + 1; // Next waypoint
          if (path[idx](0) == path[next_idx](0) && path[idx](1) == path[next_idx](1) && !poly_vertices_set.empty()) {
            poly_vertices_set.push_back(poly_vertices_set.back());
            decompPolys.push_back(decompPolys.back());
            continue;
          }
          vec_Vec2f line;
          line.push_back(path[idx]);
          if (idx == path_len -1) {
            line.push_back({path[idx](0) + 0.1, path[idx](1) + 0.1});
          }
          else {
            line.push_back(path[next_idx]);
          }
          keyPts.emplace_back(path[idx], path[next_idx]);

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

      // filterCorridor(hPolys);

      // Check and adjust corridors
      // Eigen::MatrixXd curIH;
      // Eigen::Vector2d interior;
      // std::vector<int> inflate(hPolys.size(), 0);
      // for (int i = 0; i < (int)hPolys.size(); i++) {     
      //     if (findInteriorDist(hPolys[i], interior) < 0.1) {
      //         inflate[i] = 1;
      //     } else {
      //         compressPoly(hPolys[i], 0.1);
      //     }
      // }
      // for (int i = 1; i < (int)hPolys.size(); i++) {
      //     curIH.resize(4, hPolys[i - 1].cols() + hPolys[i].cols());
      //     curIH << hPolys[i - 1], hPolys[i];
      //     if (!findInterior(curIH, interior)) {
      //         if (!inflate[i - 1]) {
      //             compressPoly(hPolys[i - 1], -0.1);
      //             inflate[i - 1] = 1;
      //         }
      //     } else {
      //         continue;
      //     }
      //     curIH << hPolys[i - 1], hPolys[i];
      //     if (!findInterior(curIH, interior)) {
      //         if (!inflate[i]) {
      //             compressPoly(hPolys[i], -0.1);
      //             inflate[i] = 1;
      //         }
      //     }
      // }
      for (int i = 0; i < hPolys.size(); i++) {
        std::vector<std::vector<double>> hyperparam;
        std::vector<double> test_co;
        for (int j = 0; j < hPolys[i].cols(); j++) {
          Eigen::Vector2d a = hPolys[i].col(j).head(2); 
          Eigen::Vector2d p = hPolys[i].col(j).tail(2);     
          double b = a.dot(p);   
          hyperparam.push_back({a(0), a(1), b});      
          test_co.push_back(a(0) * path[i](0) + a(1) * path[i](1) - b); 
        }
        hyperparam_set.push_back(hyperparam);
      }
  }

  std::vector<Eigen::Vector2d> Environment::interpolatePoints(const Eigen::Vector2d& start, const Eigen::Vector2d& end, int num_points) {
      std::vector<Eigen::Vector2d> points;
      points.reserve(num_points + 2); 

      Eigen::Vector2d step = (end - start) / (num_points + 1);

      for (int i = 0; i <= num_points + 1; ++i) {
          points.push_back(start + i * step);
      }

      return points;
  }
  
}