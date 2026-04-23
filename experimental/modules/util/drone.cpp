
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

std::vector<vector2D> Drone::render(const std::vector<vector3D> &points) const{
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

#ifdef MAKE_DEMO_VIDEO
// --- Create VideoWriter once (outside your frame loop) ---
static bool initialized = false;
static cv::VideoWriter writer;
#endif

void Drone::display(const std::vector<vector3D> &positions, const std::vector<vector2D> &features){
    std::vector<vector2D> points = render(positions);
    points.insert( points.end(), features.begin(), features.end() );
    int w = imgW;
    int h = imgH;
    constexpr float convertRatio = 1.0f; // zoom 
    std::vector<unsigned char> vec(w*h);
    for (vector2D reltive : points){
        auto& [xr,yr] = reltive;
        float x = xr;
        float y = yr;
        x *= convertRatio;
        y *= convertRatio;
        x += 1;
        y += 1;
        x *= imgW/2;
        y *= imgH/2;
        x = (int)x;
        y = (int)y;
        if (0 <= x && x < imgW && 0 <= y && y < imgH ){
            vec[x + imgW*y] = 255;
        }
    }
    // Interpret the raw pointer as unsigned char*
    unsigned char* bytes = (unsigned char*)(vec.data());

    // Create an OpenCV image header around the raw data
    cv::Mat img(h, w, CV_8UC1, bytes);
    
    #ifndef MAKE_DEMO_VIDEO // during demo video making display is not needed.
    cv::imshow("Raw Image", img);
    constexpr int millisecondsWaitedPerFrame = 1000/60;
    cv::waitKey(millisecondsWaitedPerFrame);
    #endif

    #ifdef MAKE_DEMO_VIDEO // code is only used if MAKE_DEMO_VIDEO is explicitly enabled.
    if (!initialized) {
        int fps = 60;
        int fourcc = cv::VideoWriter::fourcc('M','J','P','G');

        writer.open("/src/output.avi", fourcc, fps, cv::Size(w, h), false);

        if (!writer.isOpened()) {
            std::cerr << "Failed to open VideoWriter!" << std::endl;
        }

        initialized = true;
    }

    writer.write(img);
    #endif

}
#else
void Drone::display(const std::vector<vector3D> &positions,const std::vector<vector2D> &features){
    return;
}
#endif


void Drone::estimate3DPositions(const std::vector<InputPoint> &features, const bool lockZ){
    vector3D loc        = state.loc;
    vector3D rot        = state.forwardRot();
    vector3D framedown  = state.downRot();
    vector3D frameright = state.rightRot();

    // TODO make better point estimator.
    for (InputPoint feature : features){
        auto& [id, p] = feature;
        vector3D newPosition = rot + frameright*p.x + framedown*p.y;
        const float comeUpWithProperScaleForDistance = std::max(1.0f,loc.z);
        if (lockZ){
            float t = -loc.z/newPosition.z;
            newPosition = loc + newPosition*t;
            //newPosition = loc + newPosition*loc.z;
        } else {
            newPosition = loc + newPosition.normalize()*comeUpWithProperScaleForDistance;
        }
        constexpr float retention = 0.9;
        if (data.find(id) == data.end()){
            data[id] = newPosition; //init
        } else{
            data[id] *= retention; //update
            data[id] += newPosition*(1-retention);
        }
    }
}

// currently handled by the same function but it can be sepparated to have their own logic.
void Drone::initEstimate3DPositions(const std::vector<InputPoint> &features, const bool lockZ){
    estimate3DPositions(features,lockZ);
}

std::optional<DroneState> Drone::initialize(const std::vector<InputPoint>& trackedPoints, const DroneState& start, const bool lockZ){
    if (trackedPoints.size() >= minimumNumberOfPoints){
        state = start;
        initEstimate3DPositions(trackedPoints,lockZ);
        lost = false;
        return std::nullopt;
    }
}

std::optional<DroneState> Drone::process_frames(
    const std::vector<InputPoint>& trackedPoints,
    const DroneState givenEstimate,
    const bool assumeCorrectRotationIsGiven,
    const bool lockZ,
    const bool display
){
    if (lost){
        return initialize(trackedPoints, givenEstimate,lockZ);
    }
    std::vector<InputPoint> newTrackedPoint;
    std::vector<vector3D>   point3D;
    std::vector<vector2D>   point2D;
    for (auto trackedPoint : trackedPoints){
        auto& [id, point] = trackedPoint;
        if (data.find(id) == data.end()){
            newTrackedPoint.push_back(trackedPoint);
        } else {
            point3D.push_back(data[id]);
            point2D.push_back(point);
        }
    }
    initEstimate3DPositions(newTrackedPoint,lockZ);
    if (point3D.size() < minimumNumberOfPoints){
        lost = true;
        data.clear();
        return std::nullopt;
    }
    if (!assumeCorrectRotationIsGiven){
        state = locateDrone(point3D,point2D,state,lockZ,false);
    } else {
        state.rot = givenEstimate.rot;
        state = optimalLocation(point3D,point2D,state,lockZ,false);
    }

    if (display){
        Drone temp = {state};
        temp.display(point3D,point2D);
    }
    estimate3DPositions(trackedPoints,lockZ);
    return std::optional<DroneState>{state};
};

