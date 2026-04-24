#ifndef TIMER_H
#define TIMER_H

#include <iosfwd>
#include <chrono>
#include <vector>

struct BenchMark {
    std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> split = std::chrono::steady_clock::now();
    std::chrono::time_point<std::chrono::steady_clock> benchMark = std::chrono::steady_clock::now();
    int totalIters = 0;
    std::vector<double> arr;
    std::vector<const char*> descr;
    int step = 0;
    ~BenchMark();
    double time_split(std::chrono::time_point<std::chrono::steady_clock> &split);
    double seconds();
    void step_Completed(const char* info = "");
    void cycle_Completed(const char* info = "");
    void show_benchmark();
    double fps();
};

#endif