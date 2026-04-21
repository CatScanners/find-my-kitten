
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

std::vector<vector2D> drone::render(const std::vector<vector3D> &points) const{
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

void drone::display(const std::vector<vector3D> &positions, const std::vector<vector2D> &features){
    std::vector<vector2D> points = render(positions);
    points.insert( points.end(), features.begin(), features.end() );
    int w = imgW;
    int h = imgH;
    constexpr float convertRatio = 1.0f; 
    std::vector<unsigned char> vec(w*h);
    for (vector2D reltive : points){
        auto& [xr,yr] = reltive;
        float x = xr;
        float y = yr;
        x /= convertRatio;
        y /= convertRatio;
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

    cv::imshow("Raw Image", img);
    constexpr int millisecondsWaitedPerFrame = 1000/60;
    cv::waitKey(millisecondsWaitedPerFrame);

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

std::optional<DroneState> drone::process_frames(
    const std::vector<inputPoint>& trackedPoints,
    const DroneState givenEstimate,
    const bool assumeCorrectRotationIsGiven,
    const bool lockZ,
    const bool display
){
    if (lost){
        return initialize(trackedPoints, givenEstimate);
    }
    std::vector<inputPoint> newTrackedPoint;
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
    initEstimate3DPositions(newTrackedPoint);
    if (point3D.size() < minimumNumberOfPoints){
        lost = true;
        return std::nullopt;
    }
    if (!assumeCorrectRotationIsGiven){
        state = locateDrone(point3D,point2D,state,lockZ,display);
    } else {
        state.rot = givenEstimate.rot;
        state = optimalLocation(point3D,point2D,state,lockZ,display);
    }
    estimate3DPositions(trackedPoints);
    return std::optional<DroneState>{state};
};

