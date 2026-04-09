#ifndef DRONE_HPP
#define DRONE_HPP

#include "imagePosition.hpp"
#include "vector3D.hpp"
#include "Quaternion.hpp"
#include "frameData.hpp"
#include <optional>
#include <iostream>
#include <unordered_map>
#include <cmath>
#include <vector>
vector2D solvePointOnPlane(const vector3D& pointOnPlain, const vector3D& wVec,const vector3D& hVec, const vector3D& line) ;


struct DroneState{
    
    vector3D loc;
    Quaternion rot;
    
    vector3D forwardRot() const ;
    vector3D rightRot()   const ;
    vector3D downRot()    const ;
    vector3D rotateRelativeToCamera(const vector3D vec) const;
    void rotateTowards(vector3D towards) ;
    void rotateCamera(float rad) ;
};

constexpr int minimumNumberOfPoints = 8;
struct drone{
    DroneState state;
    std::unordered_map<int,vector3D> data;
    bool lost = true;

    drone(DroneState s) : state(s) {};

    std::vector<vector2D> render(const std::vector<vector3D> &points);
    
    void display(const std::vector<vector3D> &positions, const std::vector<vector2D> &features);

private:
    void initEstimate3DPositions(const std::vector<inputPoint> &features);
    void estimate3DPositions(const std::vector<inputPoint> &features);
    
    std::optional<DroneState> initialize(const std::vector<inputPoint>& trackedPoints, const DroneState& start);
public:
    std::optional<DroneState> prosess_frames(
        const std::vector<inputPoint>& trackedPoints, 
        const DroneState start,
        const bool lockZ = true, 
        const bool display = false
    );
};

drone giveDroneExample(int i,float distance);

drone giveDroneExampleError(drone start, int i,float distance);

drone droneRandomWalk(drone start,float distance);

#endif