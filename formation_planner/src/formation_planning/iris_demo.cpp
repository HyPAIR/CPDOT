#include <iostream>
#include <Eigen/Core>
#include "iris/iris.h"
#include <math.h>
// #include "matplotlibcpp.h"

// namespace plt = matplotlibcpp;
#define M_PI 3.14159265358979323846

// int main(int argc, char** argv) {
//   Eigen::Matrix<double, 2, 100> mat;
//   Eigen::Matrix<double, 2, 100> mat_;
//   Eigen::Matrix<double, 2, 2> c;
//   Eigen::Matrix<double, 2, 100> d;
//   int  j = 0;
//   for (float i = 0.0; i < 2*M_PI; i+=2*M_PI/100) {
//        mat(0,j)=cos(i);
//        mat(1,j)=sin(i);
//        j += 1;
//   }
//   iris::IRISProblem problem(2);
//   problem.setSeedPoint(Eigen::Vector2d(0.0, 0.0));

//   Eigen::MatrixXd obs(2,2);
//   int ind_x = 0, ind_y = 0;
//   std::vector<float> obs_x(4);
//   std::vector<float> obs_y(4);
//   // Inflate a region inside a 1x1 box
//   obs << 0, 3,
//          -1, 0;
//   problem.addObstacle(obs);
//   obs_x.at(ind_x++) = obs(0,0);
//   obs_y.at(ind_y++) = obs(0,1);
//   obs << 6, 0,
//          0, 3;
//   problem.addObstacle(obs);
//   obs_x.at(ind_x++) = obs(0,0);
//   obs_y.at(ind_y++) = obs(0,1);
//   obs << 0, -0.5,
//          6, 0;
//   problem.addObstacle(obs);
//   obs_x.at(ind_x++) = obs(0,0);
//   obs_y.at(ind_y++) = obs(0,1);
//   obs << -1, 0,
//          0, -0.5;
//   problem.addObstacle(obs);
//   obs_x.at(ind_x++) = obs(0,0);
//   obs_y.at(ind_y++) = obs(0,1);
//   obs_x.push_back(obs_x.at(0));
//   obs_y.push_back(obs_y.at(0));
//   iris::IRISOptions options;
//   iris::IRISRegion region = inflate_region(problem, options);
//   c = region.ellipsoid.getC();
//   d = region.ellipsoid.getD().replicate(1,100);
//   mat_ = region.ellipsoid.getC() * mat + d;
//   std::cout << "C: " << region.ellipsoid.getC() << std::endl;
//   std::cout << "d: " << region.ellipsoid.getD() << std::endl;
//   std::vector<float> x_data(mat.cols());
//   std::vector<float> y_data(mat.cols());
//   for(int j = 0; j < mat.cols(); ++j) {
//     x_data.at(j) = mat_(0, j);
//     y_data.at(j) = mat_(1, j);
//   }
//   // plt::plot(obs_x, obs_y);
//   plt::plot(obs_x, obs_y,"r--");
//   plt::scatter(x_data, y_data, 6.);
//   plt::show();
//   return 0;
// }
int main(int argc, char** argv) {
 iris::IRISProblem problem(2);
  problem.setSeedPoint(Eigen::Vector2d(-5.0, 0.0));
  Eigen::MatrixXd obs(2,4);
  int ind_x = 0, ind_y = 0;
  std::vector<float> obs_x(4);
  std::vector<float> obs_y(4);
  // Inflate a region inside a 1x1 box
  obs << -30.5, -30.5, -30, -30,
         -30, 30, 30, -30;
  problem.addObstacle(obs);
    obs << -30, 30, 30, 30,
         30, 30.5, 30.5, 30;
  problem.addObstacle(obs);
    obs << 30, 30.5, 30.5, 30,
         30, 30, -30, -30;
  problem.addObstacle(obs);
    obs << -30, -30, 30, 30,
         -30, -30.5, -30.5, -30;
  problem.addObstacle(obs);
    obs << -5, -4, 5.5, 6.5,
         1, -0.8, -0.9, 0.5;
  problem.addObstacle(obs);
  iris::IRISOptions options;
  iris::IRISRegion region = inflate_region(problem, options);
  auto points = region.polyhedron.generatorPoints();
  auto A = region.polyhedron.getA();
  auto b = region.polyhedron.getB();
  std::cout << "points: " << points[0][0] << std::endl;
  std::cout << "C: " << region.ellipsoid.getC() << std::endl;
  std::cout << "d: " << region.ellipsoid.getD() << std::endl;
  return 0;
}
