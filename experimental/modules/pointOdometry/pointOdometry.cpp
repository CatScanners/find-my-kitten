#include "../util/pointOdometry.hpp"
#include "../util/Quaternion.hpp"
#include "../util/drone.hpp"
#include "../util/inputPoint.hpp"
#include "../util/vector2D.hpp"
#include "../util/vector3D.hpp"
#include <iostream>
#include <vector>
#include <string>

#include "../util/examplePoints.hpp"
#include "../util/timer.hpp"

constexpr float scale = 1;
constexpr bool displayTest = false; // with display on, it is recommended to set fps that suits the usecase in drone::display
constexpr int iterations = displayTest ? 10 : 100;
constexpr float amountOfErrorPoints = scale / 20;
constexpr float amountOfErrorDrone = scale;


struct TrackError{
  std::vector<vector3D> locations;
  std::vector<Quaternion> rotations;
  // There are probably better ways to represent rotation error
  void printErrorLocations(){
    vector3D mean = EMPTY_VECTOR3D;
    for ( vector3D loc : locations){
      mean += loc;
    }
    mean *= 1.0f/locations.size();
    vector3D standardDeviation =  EMPTY_VECTOR3D;
    std::cout << "Location error mean: " << mean << "\n";
    for ( vector3D loc : locations){
      standardDeviation += (loc-mean).elementProduct(loc-mean);
    }
    standardDeviation = standardDeviation.elementSqrt();
    standardDeviation *= 1.0f/locations.size();
    std::cout << "Location error standard deviation: " << standardDeviation << "\n";
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
    standardDeviation = standardDeviation.elementSqrt();
    standardDeviation *= 1.0f/rotations.size();
    std::cout << "Rotation error standard deviation: " << standardDeviation << "\n";
  }
  
  void clear(){
    locations.clear();
    rotations.clear();
  }
};


void testPointOdometry_helper(
  const std::vector<vector3D> points, 
  const drone real, 
  float noiseOn3D, 
  float noiceOn2D,
  bool locationError, 
  bool rotationError,
  int test 
){
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

  for (int i = 0; i < iterations; i += 1) {
    // add error to rotation
    inaccurateLocation = droneRandomWalk(inaccurateLocation, amountOfErrorDrone);
    if (!locationError) inaccurateLocation.state.loc = real.state.loc; 
    if (!rotationError) inaccurateLocation.state.rot = real.state.rot; 
    // solve 
    if (test == 0){
      inaccurateLocation.state = optimalRotation(pointsCopy,cameraReal,inaccurateLocation.state,false,displayTest);
    } else if (test == 1){
      inaccurateLocation.state = optimalLocation(pointsCopy,cameraReal,inaccurateLocation.state,false,displayTest);
    } else if (test == 2){
      inaccurateLocation.state = gradientDescentLocateDrone(pointsCopy,cameraReal,inaccurateLocation.state,false,displayTest);
    } else if (test == 3){
      inaccurateLocation.state = gradientDescentLocateDroneV2(pointsCopy,cameraReal,inaccurateLocation.state,false,displayTest);
    } 
    // compare 
    if (locationError) tracker.locations.push_back(real.state.loc-inaccurateLocation.state.loc);
    if (rotationError) tracker.rotations.push_back(real.state.rot-inaccurateLocation.state.rot);
    t.cycle_Completed("cycle");
  }
  t.show_benchmark();
  if (locationError) tracker.printErrorLocations();
  if (rotationError) tracker.printErrorRotations();
}

void testPointOdometry(std::string name, void (*test_helper)(const std::vector<vector3D>, const drone, float, float)) {
  // Test optimal rotations error free
  std::vector<vector3D> points;
  // Generates hear shape on xy plane.
  generateHeart(points, scale * 2 / 4);
  drone realLoc = giveDroneExample(0, scale);

  std::cout << "\n";
  std::cout << "Test " << name << " with error free data: \n";
  test_helper(points,realLoc,0,0);
  std::cout << "\n";
  std::cout << "Test " << name << " with random noise added to the render data: \n";
  test_helper(points,realLoc,0,amountOfErrorPoints);
  std::cout << "\n";
  std::cout << "Test " << name << " with random noise added to the 3D points: \n";
  test_helper(points,realLoc,amountOfErrorPoints,0);
  std::cout << "\n";
  std::cout << "Test " << name << " with random noise added to the 3D points and screne data: \n";
  test_helper(points,realLoc,amountOfErrorPoints,amountOfErrorPoints);
}


void testOptimalRotation_helper(const std::vector<vector3D> points, const drone real, float noiseOn3D, float noiceOn2D){
  testPointOdometry_helper(points,real,noiseOn3D,noiceOn2D,false,true,0);
}
void testOptimalRotation() {
  return testPointOdometry("optimalRotation",testOptimalRotation_helper);
}

void testOptimalLocation_helper(const std::vector<vector3D> points, const drone real, float noiseOn3D, float noiceOn2D){
  testPointOdometry_helper(points,real,noiseOn3D,noiceOn2D,true,false,1);
}
void testOptimalLocation() {
  return testPointOdometry("optimalLocation",testOptimalLocation_helper);
}

void testGradientDecent_helper(const std::vector<vector3D> points, const drone real, float noiseOn3D, float noiceOn2D){
  testPointOdometry_helper(points,real,noiseOn3D,noiceOn2D,true,true,3);
}
void testGradientDecent() {
  return testPointOdometry("gradientDescentLocateDrone",testGradientDecent_helper);
}

void testGradientDecentV2_helper(const std::vector<vector3D> points, const drone real, float noiseOn3D, float noiceOn2D){
  testPointOdometry_helper(points,real,noiseOn3D,noiceOn2D,true,true,3);
}
void testGradientDecentV2() {
  return testPointOdometry("gradientDescentLocateDroneV2",testGradientDecentV2_helper);
}

// Generate own test data and experiment with it.
// Uset the test(); to test preformance and algorithms on mathematically perfect
// points or point with controlled randomness. Controlled randomness can be both
// on display points and the 3D points.
void test() {
  std::vector<vector3D> points;
  generateHeart(points, scale * 2 / 4);
  drone flying = giveDroneExample(0, scale);
  std::vector<vector2D> cameraReal = flying.render(points);

  std::vector<inputPoint> cameraFeed;
  for (int i = 0; i < 200; i++) {
    cameraFeed.push_back({i, cameraReal[i]});
  }
  std::cout << flying.state.loc << "\n";
  flying.process_frames(cameraFeed, flying.state, false, false, true);

  for (int i = 0; i < 100; i += 1) {
    flying = droneRandomWalk(flying, scale / 5);
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
      resultFrame.push_back(convertToUsableForm(3840, 2160, 170, b, c, d, true));
    }
    result.push_back(resultFrame);
  }
  return result;
}

void testOnRealData(int argc, char *argv[]) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <fileName>\n";
    exit(1);
  }

  const char *fileName = argv[1];
  auto video = readFile(fileName);
  drone flying = giveDroneExample(0, 1);
  BenchMark t;
  int n = 0;
  for (auto cameraFeed : video) {
    flying.process_frames(cameraFeed, flying.state, false, true, true);
    std::cout << flying.state.loc << "\n";
    //std::cout << ++n << "," << flying.state.loc.x << "," << flying.state.loc.y << "\n";
    t.cycle_Completed("location");
  }
  t.show_benchmark();
}   

#include <iostream>
#include <vector>

int main(int argc, char *argv[]) {
  //test();

  //testOptimalRotation();
  //testOptimalLocation();
  //testGradientDecent();
  //testGradientDecentV2();

  testOnRealData(argc, argv);
  return 0;
}
