

#include "timer.hpp"
#include <iostream>
#include <chrono>
#include <vector>


BenchMark::~BenchMark(){
    show_benchmark();
    std::cout << fps() << "fps" << std::endl;
}
double BenchMark::time_split(std::chrono::time_point<std::chrono::steady_clock> &split){
    auto end = std::chrono::steady_clock::now();
    double time = (double)(end-split).count()/1000000000;
    split = end;
    return time;
}
double BenchMark::seconds(){
    auto end = std::chrono::steady_clock::now();
    return (double)(end-start).count()/1000000000;
}

void BenchMark::step_Completed(const char* info){
    if (step == arr.size()) {
        arr.push_back(0.0);
        descr.push_back(info);
    } 
    arr[step] += time_split(benchMark);
    step += 1;
}
void BenchMark::cycle_Completed(const char* info){
    step_Completed(info);
    totalIters += 1;
    step = 0;
}

void BenchMark::show_benchmark(){
    for (int i = 0; i < arr.size(); i++){
        std::cout << "[" << arr[i]/totalIters << "s]\taverage on step [" << i << "] " << descr[i] << std::endl;
    }
}
double BenchMark::fps(){
    return totalIters/seconds();
}
