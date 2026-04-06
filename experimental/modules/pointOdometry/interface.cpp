#include "../util/vector3D.hpp" 
#include "../util/vector3D.hpp" 
#include "../util/Quaternion.hpp"
#include "../util/drone.hpp"
#include "../util/examplePoints.hpp"
#include "../util/imagePosition.hpp"
#include <vector>
#include <random>
#include <iostream>
#include <cmath>
#include <string>
#include <climits> 
#include <unordered_map>
struct Sphere {
    const float mass;

    vector3D angularVelocity;   // ω

    // Moment of inertia for a solid sphere: I = 2/5 m R^2
    float inertia() const {
        return 0.4f * mass;
    }

    // Apply an impulse J at offset r (both in world space)
    void applyImpulse(const vector3D& J, const vector3D& r) {
        // Angular momentum change: ΔL = r × J
        vector3D dL = r.crossProduct(J);

        // Angular velocity change: Δω = ΔL / I
        angularVelocity += dL * (1.0f / inertia());
    }
    Quaternion toQuaternion(float dt = 0.2f){
        const vector3D omega = angularVelocity;
        float wx = omega.x, wy = omega.y, wz = omega.z;
        float mag = std::sqrt(wx*wx + wy*wy + wz*wz);

        if (mag == 0.0f) {
            return {1.0f, 0.0f, 0.0f, 0.0f};
        }

        float theta = mag * dt;          // total angle in radians
        float half = 0.5f * theta;
        float s = std::sin(M_PI*half);
        float c = std::cos(M_PI*half);

        float ux = wx / mag;
        float uy = wy / mag;
        float uz = wz / mag;

        Quaternion out = {c, ux * s, uy * s, uz * s};
        out.normalize();
        return out;
    }
};

int mult = 1;
int total = 0;
DroneState optimalRotation(const std::vector<vector3D>& positions, const std::vector<vector2D>& features, DroneState previousState) {
    if (positions.size() != features.size()) std::cout << "positions and their coresponding positions on camera do not match\n";
    DroneState newState = previousState;
    Quaternion step;
    //for (int n = 0; n < 10; n++){
    constexpr float closeEnough = 0.00001f;
    int n = 0;
    do {
        Sphere rotationSphere = {positions.size(),0,0,0};
        for (int i = 0; i < positions.size(); i++){
            auto& [w,h] = features[i];
            vector3D vec3D = (positions[i]-newState.loc).normalize();
            vector3D vec2D = (newState.forwardRot()+newState.rightRot()*w+newState.downRot()*h).normalize();
            vector3D forceOnSphere = vec3D.projectToNormal(vec2D);
            rotationSphere.applyImpulse(forceOnSphere,vec2D);
        }
        step = rotationSphere.toQuaternion(0.07f);
        newState.rot = step*newState.rot;
        total += 1;
        n += step.w >= 1;
    } while (n != 5);
    return newState;
}



float pointSD(const std::vector<vector3D>& positions){
    vector3D sum = {0,0,0};
    for (int i = 0; i < positions.size(); i++){
        sum += positions[i];
    }
    sum *= 1.0f/positions.size();
    float var = 0;
    for (int i = 0; i < positions.size(); i++){
        var += (positions[i] - sum).dot(positions[i] - sum);
    }
    return std::sqrt(var)/positions.size();
}

float fitness(const std::vector<vector3D>& positions, const std::vector<vector2D>& features, DroneState previousState) {
    if (positions.size() != features.size()) std::cout << "positions and their coresponding positions on camera do not match\n";
    float fitness = 0.0f;
    //vector3D sum = {0,0,0};
    for (int i = 0; i < positions.size(); i++){
        auto& [x,y,z] = positions[i];
        auto& [w,h] = features[i];
        auto& [locx,locy,locz] = previousState.loc; 
        auto [x1,y1,z1] = previousState.forwardRot();
        auto [x2,y2,z2] = previousState.rightRot();
        auto [x3,y3,z3] = previousState.downRot();
        float fw = ((y2*z3-y3*z2)*(x-locx)-(x2*z3-x3*z2)*(y-locy)+(x2*y3-x3*y2)*(z-locz))*w+((y1*z3-y3*z1)*(x-locx)-(x1*z3-x3*z1)*(y-locy)+(x1*y3-x3*y1)*(z-locz)); //= 0
        float fh = ((y2*z3-y3*z2)*(x-locx)-(x2*z3-x3*z2)*(y-locy)+(x2*y3-x3*y2)*(z-locz))*h-((y1*z2-y2*z1)*(x-locx)-(x1*z2-x2*z1)*(y-locy)+(x1*y2-x2*y1)*(z-locz)); //= 0
        fitness += (fw*fw+fh*fh);
    }
    return fitness/positions.size();
}


void axisStep(float &axis, const std::vector<vector3D>& positions, const std::vector<vector2D>& features, DroneState &previousState, int size) {
    previousState = optimalRotation(positions, features, previousState);
    float fit = fitness(positions, features, previousState);
    float quant = pointSD(positions)*0.1/size;
    axis += quant;
    previousState = optimalRotation(positions, features, previousState);
    float fit2 = fitness(positions, features, previousState);
    if (fit2 < fit) return;
    axis -= 2*quant;
    previousState = optimalRotation(positions, features, previousState);
    float fit3 = fitness(positions, features, previousState);
    if (fit3 < fit) return;
    axis += quant;
    if (fit2 < fit3) {
        axis += fit/(fit2+fit)*quant;
    }else{
        axis -= fit/(fit3+fit)*quant;
    };
    
}

#include <chrono>
#include <thread>

DroneState gradientDescentLocateDroneV2(const std::vector<vector3D>& positions, const std::vector<vector2D>& features, DroneState previousState, bool display = false) {
    constexpr int extraIterationsV2 = 100;
    constexpr float momentumDecay =  0.8f;
    if (positions.size() != features.size()) std::cout << "positions and their coresponding positions on camera do not match\n";
    DroneState newState = optimalRotation(positions, features, previousState);
    std::vector<vector3D> locations; 
    float fitp = fitness(positions, features, previousState); 
    float fit  = fitness(positions, features, previousState); 
    vector3D momentum = {0,0,0};
    int count = 1;
    do {
        momentum *= momentumDecay;
        newState.loc += momentum;
        vector3D start = newState.loc;
        //gradientStep(positions, features, newState);
        axisStep(newState.loc.x, positions, features, newState, count);
        axisStep(newState.loc.y, positions, features, newState, count);
        axisStep(newState.loc.z, positions, features, newState, count);
        momentum += newState.loc-start;
        fitp = fit;
        fit = fitness(positions, features, newState);
        count += fitp < fit;
        if (display){
            drone temp = {newState};
            std::vector<vector2D> m = temp.render(positions);
            m.insert( m.end(), features.begin(), features.end() );
            temp.display(m);
            if (count == extraIterationsV2) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }
    } while (count != extraIterationsV2);
    return optimalRotation(positions, features, newState);;
}



void test(){
    std::vector<vector3D> points;
    //generateHeart(points);
    std::vector<vector3D> rand;
    //random(rand);
    float distance = 50;
    generateHeart(points,distance*2/4);
    std::vector<vector3D> pointsGuess = points;
    random(rand, distance/2/5);
    add(pointsGuess,rand);
    
    for (int i = 0; i < 150; i+=5){
        drone flying = giveDroneExample(i,distance);
        std::vector<vector2D> cameraReal = flying.render(points);
        drone copyError = giveDroneExampleError(flying,i,distance);
        copyError.state = gradientDescentLocateDroneV2(pointsGuess,cameraReal,copyError.state,true);
    }
}


int main() {
    test();
}
