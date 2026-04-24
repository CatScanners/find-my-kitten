#ifndef DRONE_HPP
#define DRONE_HPP

#include "vector2D.hpp"
#include "vector3D.hpp"
#include "Quaternion.hpp"
#include "inputPoint.hpp"
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

constexpr int minimumNumberOfPoints = 6;
struct Drone{
    DroneState state;
    std::unordered_map<int,vector3D> data;
    bool lost = true;

    Drone(DroneState s) : state(s) {lost = true;};

    std::vector<vector2D> render(const std::vector<vector3D> &points) const;

    void display(const std::vector<vector3D> &positions, const std::vector<vector2D> &features);

private:
    void initEstimate3DPositions(const std::vector<InputPoint> &features, const bool lockZ);
    void estimate3DPositions(const std::vector<InputPoint> &features, const bool lockZ);

    std::optional<DroneState> initialize(const std::vector<InputPoint>& trackedPoints, const DroneState& start, const bool lockZ);
public:
    std::optional<DroneState> process_frames(
        const std::vector<InputPoint>& trackedPoints,
        const DroneState start,
        const bool assumeCorrectRotationIsGiven = false, // if internal sensors are accurate in rotational orientation this could be helpful.
        const bool lockZ = true,
        const bool display = false
    );
};


#endif
