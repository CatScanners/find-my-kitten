#include "../util/vector3D.hpp" 
#include "../util/vector3D.hpp" 
#include "../util/Quaternion.hpp"
#include "../util/drone.hpp"
#include "../util/examplePoints.hpp"
#include "../util/imagePosition.hpp"
#include "../util/pointOdometry.hpp"
#include "../util/frameData.hpp"
#include <vector>
#include <iostream>

// Generate own test data and experiment with it.
// Uset the test(); to test preformance and algorithms on mathematically perfect points or point with controlled randomness. 
// Controlled randomness can be both on display points and the 3D points.
constexpr float distance = 50;
void test(){
    std::vector<vector3D> points;
    generateHeart(points,distance*2/4);
    drone flying = giveDroneExample(0,distance);
    std::vector<vector2D> cameraReal = flying.render(points);

    std::vector<inputPoint> cameraFeed;
    for (int i = 0; i < 200; i++){
        cameraFeed.push_back({i,cameraReal[i]});
    }
    std::cout << flying.state.loc << "\n";
    flying.prosess_frames(cameraFeed,flying.state,false,true);

    for (int i = 0; i < 100; i+=1){
        flying = droneRandomWalk(flying,distance/5);
        std::cout << flying.state.loc << "\n";
        flying.prosess_frames(cameraFeed,flying.state,false,true);
        std::cout << flying.state.loc << "\n";
    }
}


#include <vector>
#include <string>
#include <iostream>
#include <fstream>

struct feature{
    int a, b;
    float c, d;
};

std::vector<std::vector<inputPoint>> readFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Could not open file " << filename << " \n";
        exit(1);
    }

    int a, b;
    float c, d;
    std::vector<std::vector<feature>> frames;
    int n = 0;
    std::vector<feature> features;
    while (file >> a >> b >> c >> d) {
        if (a != n){
            frames.push_back(features);
            features.clear();
            n = a;
        }
        features.push_back({a, b, c, d});
    }
    frames.push_back(features);
    std::vector<std::vector<inputPoint>> result;
    for (auto frame : frames){
        std::vector<inputPoint> resultFrame;
        for (auto [a,b,c,d] : frame){
            //resultFrame.push_back({b,{c,d}});
            //3840x2160 
            float fov = 1500; // this should be changed to some more appropriate value
            resultFrame.push_back({b,{(c-3840/2)*1.0f/fov,(d-2160/2)*1.0f/fov}});
        }
        result.push_back(resultFrame);
    }
    return result;
}

#include <iostream>
#include <vector>
#include "../util/timer.h"

int main(int argc, char* argv[]) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <fileName>\n";
        return 1;
    }

    //test();

    const char* fileName = argv[1];
    auto video = readFile(fileName);
    drone flying = giveDroneExample(0,1);
    BenchMark t;
    for (auto cameraFeed : video){
        flying.prosess_frames(cameraFeed,flying.state,false,false);
        std::cout << flying.state.loc << "\n";
        t.cycle_Completed("location");
    }
    t.show_benchmark(); 
    
    return 0;
}
