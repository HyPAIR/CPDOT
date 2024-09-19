/**
 * file generate_obs.h
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief generate obstacles
 * data 2023-11-22
 * 
 * @copyright Copyroght(c) 2023
*/
#pragma once
#include <iostream>
#include <vector>
#include "vec2d.h"
#include <cmath>

namespace formation_planner {
namespace math {
class GenerateObstacle{
// 计算旋转后的点坐标
 public: 
    std::vector<Vec2d> rotatePoint(const Vec2d& center, double angle, bool inflat, int obs_type) {
        std::vector<std::vector<double>> obs_size = {
            {0.4, 0.275},
            {0.282, 0.132},
            {0.28, 0.1},
            {0.795, 0.279},
            {0.7, 0.28},
            {1.197, 0.278}
        };
        std::vector<Vec2d> rotatedPoints;
        double width, height;
        if (inflat) {
            width = 4;
            height = 1;
            // width = 1;
            // height = 0.5;
            // width = 0.41;
            // height = 0.29;        
        }
        else {
            width = 6;
            height = 3;
            // width = 1.5;
            // height = 0.75;
            // width = 2.4;
            // height = 0.28;    
            // width = obs_size[obs_type][0];
            // height = obs_size[obs_type][1];
        }
        std::vector<Vec2d> relativeVertex;
        relativeVertex.push_back({-width / 2, height / 2 });
        relativeVertex.push_back({ -width / 2, -height / 2 });
        relativeVertex.push_back({width / 2, -height / 2 });
        relativeVertex.push_back({ width / 2, height / 2 });
        double s = sin(angle);
        double c = cos(angle);

        // 将点相对于中心点平移到原点
        for (int i = 0; i < relativeVertex.size(); i++) {
            double translatedX = relativeVertex[i].x();
            double translatedY = relativeVertex[i].y();

            // 进行旋转
            Vec2d rotatedpoint(translatedX * c - translatedY * s + center.x(), translatedX * s + translatedY * c + center.y());
            rotatedPoints.push_back(rotatedpoint);
        }
        return rotatedPoints;
    }
};
}
}