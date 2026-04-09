
#include "drone.hpp"
#include "pointOdometry.hpp"
vector2D solvePointOnPlane(const vector3D& pointOnPlain, const vector3D& wVec,const vector3D& hVec, const vector3D& line) {
    auto& [x1,y1,z1] = pointOnPlain;
    auto& [x2,y2,z2] = wVec;
    auto& [x3,y3,z3] = hVec;
    auto& [x ,y ,z ] = line;
    // solves w,h,t in: pointOnPlain + wVec*w + hVec*h = line*t
    float w = ((-((y1*z3-y3*z1)*x-(x1*z3-x3*z1)*y+(x1*y3-x3*y1)*z))/((y2*z3-y3*z2)*x-(x2*z3-x3*z2)*y+(x2*y3-x3*y2)*z));
    float h = (((y1*z2-y2*z1)*x-(x1*z2-x2*z1)*y+(x1*y2-x2*y1)*z)/((y2*z3-y3*z2)*x-(x2*z3-x3*z2)*y+(x2*y3-x3*y2)*z));
    //3D to 2D the value of t is lost.
    //float t = ((x1*(y2*z3-y3*z2)-x2*(y1*z3-y3*z1)+x3*(y1*z2-y2*z1))/((y2*z3-y3*z2)*x-(x2*z3-x3*z2)*y+(x2*y3-x3*y2)*z));
    return {w,h};
}

constexpr float imgHFOV = 1.0;
constexpr float imgWFOV = 1.0;
constexpr float imgW = 800;
constexpr float imgH = 800;

vector3D DroneState::forwardRot() const {
    return rot.rotateVector({0,0,-1});
}
vector3D DroneState::rightRot() const {
    return rot.rotateVector({1,0,0});
}
vector3D DroneState::downRot() const {
    return rot.rotateVector({0,-1,0});
}
vector3D DroneState::rotateRelativeToCamera(vector3D vec) const {
    return rot.rotateVectorReverse(vec);
}
void DroneState::rotateTowards(vector3D towards) {
    Quaternion rotate = Quaternion().fromTwoVectors(forwardRot(),towards-loc);
    rot = rot*rotate;
}
void DroneState::rotateCamera(float rad) {
    Quaternion rotate = Quaternion().fromAxisAngle(forwardRot(),rad);
    rot = rot*rotate;
}

std::vector<vector2D> drone::render(const std::vector<vector3D> &points){
    vector3D loc        = state.loc;
    vector3D rot        = state.forwardRot();
    vector3D framedown  = state.downRot(); 
    vector3D frameright = state.rightRot();
    std::vector<vector2D> features;
    for (vector3D p : points){
        vector3D dif = p-loc;
        //if (dif.dot(rot) < 0) {
        //    std::cout << "rendering behind\n";
        //    //exit(1);
        //}
        vector2D reltive = solvePointOnPlane(rot,frameright,framedown,dif);
        features.push_back(reltive);
    }
    return features;
}

#ifdef HAS_OPENCV
#include <opencv2/opencv.hpp>
void drone::display(const std::vector<vector3D> &positions,const std::vector<vector2D> &features){
    std::vector<vector2D> points = render(positions);
    points.insert( points.end(), features.begin(), features.end() );
    int w = imgW;
    int h = imgH;
    std::vector<unsigned char> vec(w*h);
    for (vector2D reltive : points){
        auto& [x ,y ] = reltive;
        if (-imgWFOV < x && x < imgWFOV && -imgHFOV < y && y < imgHFOV ){
            vec[(int)(imgW*((x/imgWFOV+1)/2)) + imgW*(int)(imgH*((y/imgHFOV+1)/2))] = 255;
        }
    }
    // Interpret the raw pointer as unsigned char*
    unsigned char* bytes = (unsigned char*)(vec.data());

    // Create an OpenCV image header around the raw data
    cv::Mat img(h, w, CV_8UC1, bytes);

    cv::imshow("Raw Image", img);
    cv::waitKey(1);
}
#else
void drone::display(const std::vector<vector3D> &positions,const std::vector<vector2D> &features){
    return;
}
#endif



void drone::initEstimate3DPositions(const std::vector<inputPoint> &features){
    vector3D loc        = state.loc;
    vector3D rot        = state.forwardRot();
    vector3D framedown  = state.downRot(); 
    vector3D frameright = state.rightRot();
    for (inputPoint feature : features){
        auto& [id, p] = feature;
        vector3D onplane = rot + frameright*p.x + framedown*p.y;
        // TODO make better point estimator.
        //onplane = onplane.normalize();
        //float t = -loc.z/onplane.z; 
        //data[id] = loc + onplane*t;
        data[id] += (loc + onplane*loc.z);
    }
}

void drone::estimate3DPositions(const std::vector<inputPoint> &features){
    vector3D loc        = state.loc;
    vector3D rot        = state.forwardRot();
    vector3D framedown  = state.downRot(); 
    vector3D frameright = state.rightRot();
    for (inputPoint feature : features){
        auto& [id, p] = feature;
        vector3D onplane = rot + frameright*p.x + framedown*p.y;
        // TODO make better point estimator.
        //float t = -loc.z/onplane.z; 
        //data[id] = loc + onplane*t;
        //onplane = onplane.normalize();
        constexpr float retention = 0.9;
        data[id] *= retention;
        data[id] += (loc +  onplane*loc.z)*(1-retention);
    }
}

std::optional<DroneState> drone::initialize(const std::vector<inputPoint>& trackedPoints, const DroneState& start){
    if (trackedPoints.size() >= minimumNumberOfPoints){
        state = start;
        estimate3DPositions(trackedPoints);
        lost = false;
        return std::nullopt;
    }
}

std::optional<DroneState> drone::prosess_frames(const std::vector<inputPoint>& trackedPoints, const DroneState start,const bool lockZ,const bool display){
    if (lost){
        return initialize(trackedPoints, start);
    }
    std::vector<inputPoint> newTrackedPoint;
    std::vector<vector3D> point3D;
    std::vector<vector2D> point2D;
    for (auto trackedPoint : trackedPoints){
        auto& [id, point] = trackedPoint;
        if (data.find(id) == data.end()){
            newTrackedPoint.push_back(trackedPoint);
        } else {
            point3D.push_back(data[id]);
            point2D.push_back(point);
        }
    }
    if (point3D.size() < minimumNumberOfPoints){
        return std::nullopt;
        lost = true;
    }
    state = locateDrone(point3D,point2D,state,lockZ,display);
    //state = optimalLocation(point3D,point2D,state);
    initEstimate3DPositions(newTrackedPoint); 
    estimate3DPositions(trackedPoints);
    return std::optional<DroneState>{state};
};


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
#include "examplePoints.hpp"

drone droneRandomWalk(drone start,float distance){
    drone copyError = start;
    vector3D offsetLoc = random3D();
    copyError.state.loc += offsetLoc*distance;
    vector3D offsetRot = random3D();
    offsetRot += copyError.state.loc;
    copyError.state.rotateTowards(offsetRot);
    return copyError;
}
