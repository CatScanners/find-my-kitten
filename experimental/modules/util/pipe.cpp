#include <iostream>
#include <vector>
#include <thread>
#include <fstream>
#include <unistd.h>
#include <sys/stat.h>
#include <functional>

using namespace std;

using PointIntFunc = std::function<std::tuple<char*, int>()>;

void createPipe(const char* pipeName, PointIntFunc func) {
    std::cout << "Publishing images to " << pipeName << endl;
    
    mkfifo(pipeName, 0666);

    std::ofstream pipe(pipeName, std::ios::binary);
    if (!pipe) {
        std::cerr << "Failed to open pipe\n";
        exit(1);
    }

    while (true){
        auto [pointer, size] = func();
        if (pipe.fail()) { std::cerr << "Broken pipe detected\n"; break; }
        pipe.write(pointer, size);
        pipe.flush();
    }
}

std::ifstream makePipe(const char* pipeName) {
    std::ifstream pipe(pipeName, std::ios::binary);
    if (!pipe) {
        std::cerr << "Failed to open pipe\n";
        exit(1);
    }
    return pipe;
}

std::vector<char> usePipe(std::ifstream &pipe, int size) {
    std::vector<char> buffer(size);
    pipe.read(buffer.data(), buffer.size());
    return buffer;
}