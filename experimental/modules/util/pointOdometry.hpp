#ifndef POINT_ODOMETRY_HPP
#define POINT_ODOMETRY_HPP
#include <vector>
#include "../util/vector3D.hpp" 
#include "../util/drone.hpp"
#include "../util/imagePosition.hpp"


// OPTIMAL means minimum squared distance of each feature 
// to their 3D points in space projected on to the camera.
// Fitness function that is intended to be minimized.
// Can be easily run in parallel on gpu.
// 32 GPU threads each collects it's own local sum.
// finally cpu or gpu can add them together.
// thread n should collect all values on array [n mod 32] (n+32*i)
float fitness(
    const std::vector<vector3D>& positions, const std::vector<vector2D>& features, const DroneState previousState
);


// Given an orientation produces optimal location. 
// Uses gradient decent, very fast.
// Also it is easily optimizable to work with GPU.
DroneState optimalLocation(
    const std::vector<vector3D>& positions, const std::vector<vector2D>& features, const DroneState previousState
);

// Given a location produces optimal rotation. 
// Can be optimized to work with GPU 
// by runing multiple shperes in parallel 
// and adding sphere angularmomentums together. 
// Sphere([0...10]) + Shpere([10...20]) = Shpere([0...20]) 
// Addition not implemented yet.
DroneState optimalRotation(
    const std::vector<vector3D>& positions, const std::vector<vector2D>& features, const DroneState previousState, const bool lockZ = false
);


// This is the current best attempt to combine rotation and location solve from 3D and 2D points.
DroneState locateDrone(
    const std::vector<vector3D>& positions, const std::vector<vector2D>& features, const DroneState previousState, const bool lockZ = false, const bool display = false
);

#endif