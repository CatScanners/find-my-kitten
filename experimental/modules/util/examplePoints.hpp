#ifndef EXAMPLEPOINTS_HPP
#define EXAMPLEPOINTS_HPP

#include "vector3D.hpp"
#include <vector>

void random(std::vector<vector3D>& points, float radius = 1, int resolution = 200) ;

vector3D random3D() ;

void add(std::vector<vector3D>& points, std::vector<vector3D>& points2) ;

void generateHeart(std::vector<vector3D>& points, float scale = 16, int resolution = 200) ;

void generateCircle(std::vector<vector3D>& points, int resolution = 200) ;

void printStatistics(std::vector<vector3D> error);


#endif