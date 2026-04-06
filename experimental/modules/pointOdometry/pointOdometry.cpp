#include "../util/vector3D.hpp" 
#include "../util/vector3D.hpp" 
#include "../util/Quaternion.hpp"
#include "../util/drone.hpp"
#include "../util/examplePoints.hpp"
#include "../util/imagePosition.hpp"
#include "../util/pointOdometry.hpp"
#include <vector>
#include <iostream>


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
        copyError.state = locateDrone(pointsGuess,cameraReal,copyError.state,true);
    }
}


int main() {
    test();
}
