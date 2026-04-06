#ifndef TIMER_H
#define TIMER_H

#include <iostream>
#include <chrono>
#include <vector>

using namespace std;

struct BenchMark {
    std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> split = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> benchMark = std::chrono::steady_clock::now();
    int totalIters = 0;
    std::vector<double> arr;
    std::vector<const char*> descr;
    int step = 0;
    ~BenchMark(){
        show_benchmark();
        cout << fps() << "fps" << endl;
    }
    inline double time_split(std::chrono::time_point<std::chrono::steady_clock> &split){
        auto end = std::chrono::steady_clock::now();
        double time = (double)(end-split).count()/1000000000;
        split = end;
        return time;
    }
    inline double seconds(){
        auto end = std::chrono::steady_clock::now();
        return (double)(end-start).count()/1000000000;
    }

    inline void step_Completed(const char* info = ""){
        if (step == arr.size()) {
            arr.push_back(0.0);
            descr.push_back(info);
        } 
        arr[step] += time_split(benchMark);
        step += 1;
    }
    inline void cycle_Completed(const char* info = ""){
        step_Completed(info);
        totalIters += 1;
        step = 0;
    }

    inline void show_benchmark(){
        for (int i = 0; i < arr.size(); i++){
            cout << "[" << arr[i]/totalIters << "s]\taverage on step [" << i << "] " << descr[i] << endl;
        }
    }
    inline double fps(){
        return totalIters/seconds();
    }
};

#endif