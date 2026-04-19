#include "../util/pointOdometry.hpp"
#include "../util/Quaternion.hpp"
#include "../util/drone.hpp"
#include "../util/inputPoint.hpp"
#include "../util/vector2D.hpp"
#include "../util/vector3D.hpp"
#include <iostream>
#include <vector>

#include "../util/examplePoints.hpp"
#include "../util/timer.hpp"

constexpr float distance = 50;

struct TrackError{
  std::vector<vector3D> locations;
  std::vector<Quaternion> rotations;
  // There are probably better ways to represent rotation error
  void printErrorlLcations(){
    vector3D mean = EMPTY_VECTOR3D;
    for ( vector3D rot : locations){
      mean += rot;
    }
    mean *= 1.0f/rotations.size();
    vector3D standardDeviation =  EMPTY_VECTOR3D;
    std::cout << "Lcation error mean: " << mean << "\n";
    for ( vector3D rot : locations){
      standardDeviation += (rot-mean).elementProduct(rot-mean);
    }
    standardDeviation *= 1.0f/rotations.size();
    std::cout << "Lcation error standard deviation: " << standardDeviation << "\n";
  }

  // There are probably better ways to represent rotation error
  void printErrorRotations(){
    Quaternion mean = {0,0,0,0};
    for ( Quaternion rot : rotations){
      mean += rot;
    }
    mean *= 1.0f/rotations.size();
    Quaternion standardDeviation =  {0,0,0,0};
    std::cout << "Rotation error mean: " << mean << "\n";
    for ( Quaternion rot : rotations){
      standardDeviation += (rot-mean).elementProduct(rot-mean);
    }
    standardDeviation *= 1.0f/rotations.size();
    std::cout << "Rotation error standard deviation: " << standardDeviation << "\n";
  }
  
  void clear(){
    locations.clear();
    rotations.clear();
  }
};

void testOptimalRotation_helper(const std::vector<vector3D> points, const drone real, float noiseOn3D, float noiceOn2D){
  // make image  
  std::vector<vector3D> pointsCopy = points;
  addRandomNoise(pointsCopy, noiceOn2D);
  std::vector<vector2D> cameraReal = real.render(pointsCopy);

  // make 3D points  
  pointsCopy = points;
  addRandomNoise(pointsCopy, noiseOn3D);
  
  drone inaccurateLocation = real;
  
  // error
  TrackError tracker;
  
  // timer
  BenchMark t; 
  for (int i = 0; i < 1000; i += 1) {
    // add error to rotation
    inaccurateLocation = droneRandomWalk(inaccurateLocation, distance / 5);
    inaccurateLocation.state.loc = real.state.loc; 
    // solve 
    inaccurateLocation.state = optimalRotation(pointsCopy,cameraReal,inaccurateLocation.state);
    // compare 
    tracker.rotations.push_back(real.state.rot-inaccurateLocation.state.rot);
    t.cycle_Completed("rotation");
  }
  t.show_benchmark();
  tracker.printErrorRotations();
  tracker.clear();
}

void testOptimalRotation() {
  // Test optimal rotations error free
  std::vector<vector3D> points;
  // Generates hear shape on xy plane.
  generateHeart(points, distance * 2 / 4);
  drone realLoc = giveDroneExample(0, distance);
  constexpr float amountOfError = distance / 5;
  std::cout << "\n";
  std::cout << "Test optimalRotation with error free data: \n";
  testOptimalRotation_helper(points,realLoc,0,0);
  std::cout << "\n";
  std::cout << "Test optimalRotation with random noise added to the render data: \n";
  testOptimalRotation_helper(points,realLoc,0,amountOfError);
  std::cout << "\n";
  std::cout << "Test optimalRotation with random noise added to the 3D points: \n";
  testOptimalRotation_helper(points,realLoc,amountOfError,0);
  std::cout << "\n";
  std::cout << "Test optimalRotation with random noise added to the 3D points and screne data: \n";
  testOptimalRotation_helper(points,realLoc,amountOfError,amountOfError);
}

    // Generate own test data and experiment with it.
    // Uset the test(); to test preformance and algorithms on mathematically perfect
    // points or point with controlled randomness. Controlled randomness can be both
    // on display points and the 3D points.
void test() {
  std::vector<vector3D> points;
  generateHeart(points, distance * 2 / 4);
  drone flying = giveDroneExample(0, distance);
  std::vector<vector2D> cameraReal = flying.render(points);

  std::vector<inputPoint> cameraFeed;
  for (int i = 0; i < 200; i++) {
    cameraFeed.push_back({i, cameraReal[i]});
  }
  std::cout << flying.state.loc << "\n";
  flying.process_frames(cameraFeed, flying.state, false, false, true);

  for (int i = 0; i < 100; i += 1) {
    flying = droneRandomWalk(flying, distance / 5);
    std::cout << flying.state.loc << "\n";
    flying.process_frames(cameraFeed, flying.state, false, false, true);
    std::cout << flying.state.loc << "\n";
  }
}   


#include <fstream>
#include <iostream>
#include <string>
#include <vector>

struct feature {
    int a, b;
    float c, d;
};

std::vector<std::vector<inputPoint>> readFile(const std::string &filename) {
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
    if (a != n) {
      frames.push_back(features);
      features.clear();
      n = a;
    }
    features.push_back({a, b, c, d});
  }
  frames.push_back(features);
  std::vector<std::vector<inputPoint>> result;
  for (auto frame : frames) {
    std::vector<inputPoint> resultFrame;
    for (auto [a, b, c, d] : frame) {
      // resultFrame.push_back({b,{c,d}});
      // 3840x2160
      float fov = 1500; // this should be changed to some more appropriate value
      resultFrame.push_back(
          {b, {(c - 3840 / 2) * 1.0f / fov, (d - 2160 / 2) * 1.0f / fov}});
    }
    result.push_back(resultFrame);
  }
  return result;
}

#include <iostream>
#include <vector>

int main(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <fileName>\n";
    return 1;
  }
  testOptimalRotation();
  // test();

  //const char *fileName = argv[1];
  //auto video = readFile(fileName);
  //drone flying = giveDroneExample(0, 1);
  //BenchMark t;
  //int n = 0;
  //for (auto cameraFeed : video) {
  //  flying.process_frames(cameraFeed, flying.state, false, true, false);
  //  std::cout << flying.state.loc << "\n";
  //  //std::cout << ++n << "," << flying.state.loc.x << "," << flying.state.loc.y << "\n";
  //  t.cycle_Completed("location");
  //}
  //t.show_benchmark();

  return 0;
}
