#include "examplePoints.hpp"
#include "vector3D.hpp"
#include <vector>
#include <random>
#include <iostream>

std::random_device rd;
std::mt19937 gen(rd());
void random(std::vector<vector3D>& points, float radius, int resolution) {
    points.clear();
    points.reserve(resolution);

    std::uniform_real_distribution<float> dist(-1.0f, 1.0f); // range [-1, 1]

    for (int i = 0; i < resolution; ++i) {
        float x = radius*dist(gen);
        float y = radius*dist(gen);
        float z = radius*dist(gen);
        points.push_back({x, y, z});
    }
}

vector3D random3D() {
    std::uniform_real_distribution<float> dist(-1.0f, 1.0f); // range [-1, 1]
    return {dist(gen),dist(gen),dist(gen)};;
}


void addRandomNoise(std::vector<vector3D>& points, float radius) {
    for (int i = 0; i < points.size(); ++i) {
        points[i] += random3D()*radius;
    }
}

void add(std::vector<vector3D>& points, std::vector<vector3D>& points2) {
    for (int i = 0; i < points.size(); ++i) {
        points[i] += points2[i];
    }
}


void generateHeart(std::vector<vector3D>& points, float scale, int resolution) {
    points.clear();
    points.reserve(resolution);

    for (int i = 0; i < resolution; ++i) {
        float t = (float)i / (resolution - 1) * 2.0f * M_PI;

        float x = 16 * std::pow(std::sin(t), 3);
        float y = 13 * std::cos(t)
                - 5 * std::cos(2 * t)
                - 2 * std::cos(3 * t)
                - std::cos(4 * t);

        points.push_back({x*scale/16, y*scale/16, 0});
    }
}



void generateCircle(std::vector<vector3D>& points, int resolution) {
    points.clear();
    points.reserve(resolution);

    for (int i = 0; i < resolution; ++i) {
        float t = (float)i / (resolution - 1) * 2.0f * M_PI;

        float x = std::sin(t);
        float y = std::cos(t);

        points.push_back({x, y, 0});
    }
}
void printStatistics(std::vector<vector3D> error){
    float avg = 0;
    for (int t = 0; t < error.size(); t++){
        avg += error[t].magnitude();
    }
    avg *= 1.0f/error.size();
    std::cout << "avg: " << avg << "\n";

    float dev = 0;
    for (int t = 0; t < error.size(); t++){
        dev += (error[t].magnitude() - avg)*(error[t].magnitude() - avg);
    }
    dev /= error.size();
    std::cout << "dev: " << dev << "\n";
}


drone giveDroneExample(int i,float distance){
    Quaternion startRot = {1,0,0,0};
    vector3D startLoc = {distance*std::sin(i/10.0),0,distance*std::cos(i/10.0)};
    drone flying = drone({startLoc,startRot});
    flying.state.rotateTowards(EMPTY_VECTOR3D);
    return flying;
}


drone giveDroneExampleError(drone start, int i,float distance){
    drone copyError = start;
    vector3D offset = {-distance,distance,distance};
    vector3D offsetRot = start.state.loc.crossProduct((vector3D){-distance,-distance,-distance}).normalize()+start.state.loc;
    copyError.state.loc += offset;
    copyError.state.rotateTowards(offsetRot);//(vector3D){-distance,-distance,-distance}
    copyError.state.rotateTowards({-distance/3,-distance/3,-distance/3});
    return copyError;
}

drone droneRandomWalk(drone start,float distance){
    drone copyError = start;
    vector3D offsetLoc = random3D();
    copyError.state.loc += offsetLoc*distance;
    vector3D offsetRot = random3D();
    offsetRot += copyError.state.loc;
    copyError.state.rotateTowards(offsetRot);
    return copyError;
}
