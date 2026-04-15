#include "pointOdometry.hpp"
#include "vector3D.hpp" 
#include "Quaternion.hpp"
#include "drone.hpp"
#include "vector2D.hpp"
#include <unordered_map>


// fitness is the squared distance between the feature location on the screen and the point where 3D point would be on the screen.
float fitness(const std::vector<vector3D>& positions, const std::vector<vector2D>& features, const DroneState previousState) {
    if (positions.size() != features.size()) {
        std::cout << "3D points and their 2D points do not match\n";
        return 1.0f/0;
    }
    float fitness = 0.0f;
    //vector3D sum = {0,0,0};
    for (int i = 0; i < positions.size(); i++){
        auto& [x,y,z] = positions[i];
        auto& [w,h] = features[i];
        auto& [locx,locy,locz] = previousState.loc; 
        auto [x1,y1,z1] = previousState.forwardRot();
        auto [x2,y2,z2] = previousState.rightRot();
        auto [x3,y3,z3] = previousState.downRot();
        // these are to be optimized to 0
        float fw = ((y2*z3-y3*z2)*(x-locx)-(x2*z3-x3*z2)*(y-locy)+(x2*y3-x3*y2)*(z-locz))*w+((y1*z3-y3*z1)*(x-locx)-(x1*z3-x3*z1)*(y-locy)+(x1*y3-x3*y1)*(z-locz)); //= 0
        float fh = ((y2*z3-y3*z2)*(x-locx)-(x2*z3-x3*z2)*(y-locy)+(x2*y3-x3*y2)*(z-locz))*h-((y1*z2-y2*z1)*(x-locx)-(x1*z2-x2*z1)*(y-locy)+(x1*y2-x2*y1)*(z-locz)); //= 0
        fitness += (fw*fw+fh*fh);
    }
    return fitness/positions.size();
}


// Given an orientation produces optimal location. 
// Also it is easily optimizable to work with GPU.
DroneState optimalLocation(
    const std::vector<vector3D>& positions, const std::vector<vector2D>& features, const DroneState previousState
) {
    if (positions.size() != features.size()) {
        std::cout << "3D points and their 2D points do not match\n";
        return previousState;
    }
    vector3D gradientLoc = EMPTY_VECTOR3D;
    DroneState newState = previousState;
    
    for (int n = 0; n < 100; n++){
        float fitness = 0.0f;
        for (int i = 0; i < positions.size(); i++){
            auto& [x,y,z] = positions[i];
            auto& [w,h] = features[i];
            auto& [locx,locy,locz] = newState.loc; 
            auto [x1,y1,z1] = newState.forwardRot();
            auto [x2,y2,z2] = newState.rightRot();
            auto [x3,y3,z3] = newState.downRot();
            float fw = ((y2*z3-y3*z2)*(x-locx)-(x2*z3-x3*z2)*(y-locy)+(x2*y3-x3*y2)*(z-locz))*w+((y1*z3-y3*z1)*(x-locx)-(x1*z3-x3*z1)*(y-locy)+(x1*y3-x3*y1)*(z-locz)); //= 0
            float dxfw = -(y2*z3-y3*z2)*w-y1*z3+y3*z1;
            float dyfw =  (x2*z3-x3*z2)*w+x1*z3-x3*z1;
            float dzfw = -(x2*y3-x3*y2)*w-x1*y3+x3*y1;
            vector3D gradientLocw = { 2*fw*dxfw, 2*fw*dyfw, 2*fw*dzfw };
            float fh = ((y2*z3-y3*z2)*(x-locx)-(x2*z3-x3*z2)*(y-locy)+(x2*y3-x3*y2)*(z-locz))*h-((y1*z2-y2*z1)*(x-locx)-(x1*z2-x2*z1)*(y-locy)+(x1*y2-x2*y1)*(z-locz)); //= 0
            float dxfh = -h*(y2*z3-y3*z2)+y1*z2-y2*z1;
            float dyfh =  h*(x2*z3-x3*z2)-x1*z2+x2*z1;
            float dzfh = -h*(x2*y3-x3*y2)+x1*y2-x2*y1;
            vector3D gradientLoch = { 2*fh*dxfh, 2*fh*dyfh, 2*fh*dzfh };
            gradientLoc -= gradientLocw;
            gradientLoc -= gradientLoch;
            fitness += fw*fw+fh*fh;
        }
        newState.loc = newState.loc + gradientLoc/positions.size()*0.5;
    }
    return newState;
}


// Multiple inpulses from multiple vectors applied to a single sphere produces a new spin 
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

DroneState optimalRotation(const std::vector<vector3D>& positions, const std::vector<vector2D>& features, const DroneState previousState, const bool lockZ) {
    if (positions.size() != features.size()) {
        std::cout << "3D points and their 2D points do not match\n";
        return previousState;
    }
    // These can be tinkered with. //
    constexpr float closeEnough = 0.0f;// How close to the identity rotation should it be before incrementing smallChanges counter;
    constexpr int maxLoops = 50; // for majority of loops loopNum does not reach 15 loops
    constexpr int maxAmountOfSmallChanges = 1; // How many times is the Quaternioin identity rotation applied before considering it optimal
    constexpr float dt = 0.07f; // change in time to get the amount of movement from the momentum of the impulse shpere.
    // ----- //

    int smallChanges = 0;

    DroneState newState = previousState;
    Quaternion step;
    for (int loopNum = 0; loopNum < maxLoops && smallChanges < maxAmountOfSmallChanges; loopNum++){
                Sphere rotationSphere = {positions.size(),EMPTY_VECTOR3D};
        for (int i = 0; i < positions.size(); i++){
            auto& [w,h] = features[i];
            vector3D vec3D = (positions[i]-newState.loc).normalize();
            vector3D vec2D = (newState.forwardRot()+newState.rightRot()*w+newState.downRot()*h).normalize();
            vector3D forceOnSphere = vec3D.projectToNormal(vec2D);
            rotationSphere.applyImpulse(forceOnSphere,vec2D);
        }
        step = rotationSphere.toQuaternion(dt);
        if (lockZ) step = step.z_axis_component();
        newState.rot = step*newState.rot;
        smallChanges += step.w >= 1;
    }
    return newState;
}

int mult = 1;
int total = 0;

float pointSD(const std::vector<vector3D>& positions){
    vector3D sum = EMPTY_VECTOR3D;
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

// axisStep is most likely suboptimal.
void axisStep(float &axis, const std::vector<vector3D>& positions, const std::vector<vector2D>& features, DroneState &previousState, float stepSize) {
    previousState = optimalRotation(positions, features, previousState);
    float fit = fitness(positions, features, previousState);
    axis += stepSize;
    previousState = optimalRotation(positions, features, previousState);
    float fit2 = fitness(positions, features, previousState);
    if (fit2 < fit) return;
    axis -= 2*stepSize;
    previousState = optimalRotation(positions, features, previousState);
    float fit3 = fitness(positions, features, previousState);
    if (fit3 < fit) return;
    axis += stepSize;
    if (fit2 < fit3) {
        axis += fit/(fit2+fit)*stepSize;
    }else{
        axis -= fit/(fit3+fit)*stepSize;
    };
    
}
#include <chrono>
#include <thread>

DroneState gradientDescentLocateDroneV2(const std::vector<vector3D>& positions, const std::vector<vector2D>& features, const DroneState previousState, const bool lockZ , const bool display) {
    constexpr int extraIterationsV2 = 10;
    constexpr float momentumDecay =  0.8f;
    if (positions.size() != features.size()) {
        std::cout << "3D points and their 2D points do not match\n";
        return previousState;
    }
    DroneState newState = optimalRotation(positions, features, previousState);
    std::vector<vector3D> locations; 
    float fitp = fitness(positions, features, previousState); 
    float fit  = fitness(positions, features, previousState); 
    vector3D momentum = EMPTY_VECTOR3D;
    vector3D sum      = EMPTY_VECTOR3D;
    int count = 1;
    int n = 0;
    constexpr float d = 0.1;
    float stepSize = pointSD(positions)*d;
    do {
        //momentum *= momentumDecay;
        newState.loc += momentum;
        vector3D start = newState.loc;
        //gradientStep(positions, features, newState);
        axisStep(newState.loc.x, positions, features, newState, stepSize/count);
        axisStep(newState.loc.y, positions, features, newState, stepSize/count);
        if (!42 || !lockZ){
            axisStep(newState.loc.z, positions, features, newState, stepSize/count);
        }
        momentum += newState.loc-start;
        momentum *= momentumDecay;//momentum*(1+momentum.dot(newState.loc-start))/2*momentumDecay;//momentumDecay;
        fitp = fit;
        fit = fitness(positions, features, newState);
        count += fitp <= fit;
        if (count != 1) {
            sum += newState.loc;
            n += 1;
        }
        if (display){
            drone temp = {newState};
            temp.display(positions,features);
            //if (count == extraIterationsV2) {
            //    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            //}
        }
    } while (count != extraIterationsV2);
    newState.loc = sum*(1.0f/n);
    return optimalRotation(positions, features, newState);;
}

// unreliable
DroneState gradientDescentLocateDrone(const std::vector<vector3D>& positions, const std::vector<vector2D>& features, const DroneState previousState, const bool lockZ , const bool display) {
    if (positions.size() != features.size()) std::cout << "positions and their coresponding positions on camera do not match\n";
    DroneState bestState = previousState;
    float Minfitness = 340282346638528859811704183484516925440.0000000000000000f; 
    DroneState newState = previousState;
    constexpr int maxLoops = 1000;
    vector3D momentum = EMPTY_VECTOR3D;
    
    for (int loopNum = 0; loopNum < maxLoops; loopNum++){ //&& smallChanges < maxAmountOfSmallChanges
        vector3D gradientLoc = EMPTY_VECTOR3D;
        Sphere rotationSphere = {positions.size(),EMPTY_VECTOR3D};
        float fitness = 0.0f;
        for (int i = 0; i < positions.size(); i++){
            auto& [w,h] = features[i];
            vector3D vec3D = (positions[i]-newState.loc).normalize();
            vector3D vec2D = (newState.forwardRot()+newState.rightRot()*w+newState.downRot()*h).normalize();
            vector3D forceOnSphere = vec3D.projectToNormal(vec2D);
            float sing = (newState.forwardRot().dot(forceOnSphere) >= 0)*2-1;
            vector3D movement = ((vec3D-vec2D)-forceOnSphere)*sing;
            gradientLoc -= movement;
            rotationSphere.applyImpulse(forceOnSphere,vec2D);

        }
        constexpr float momentumDecay = 0.5;
        momentum += gradientLoc/positions.size();
        momentum *= momentumDecay*(momentum.dot(gradientLoc)>0);

        newState.loc = newState.loc + momentum;
        newState.rot = rotationSphere.toQuaternion(0.07f)*newState.rot;
        if (display){
            drone temp = {newState};
            temp.display(positions,features);
            //if (count == extraIterationsV2) {
            //    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            //}
        }
    }

    if (lockZ){
        // this is horible
        newState.loc.z = previousState.loc.z;
        newState.rot = newState.rot.z_axis_component();
        return newState;
    } else {
        return newState;
    }
}


DroneState locateDrone(
    const std::vector<vector3D>& positions, const std::vector<vector2D>& features, const DroneState previousState, const bool lockZ , const bool display
){
    if (positions.size() != features.size()) {
        std::cout << "3D points and their 2D points do not match\n";
        return previousState;
    }
    return gradientDescentLocateDroneV2(positions, features, previousState, lockZ, display);
    //return gradientDescentLocateDrone(positions, features, previousState, lockZ, display);
}; 
