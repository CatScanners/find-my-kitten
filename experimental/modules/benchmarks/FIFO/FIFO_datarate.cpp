#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include "../../util/pipe.cpp"
#include "../../util/timer.h"
#include <iostream>
#include <signal.h>
volatile sig_atomic_t stop = 0;
void handle_sigint(int sig) {
    stop = 1;
}

int main(int argc, char* argv[]) {
    signal(SIGINT, handle_sigint);
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " <width> <height> <pixel_width> <pipeName>\n";
        return 1;
    }

    int W = std::atoi(argv[1]);
    int H = std::atoi(argv[2]);
    int pixel_width = std::atoi(argv[3]);
    const char* pipeName = argv[4];

    std::cout << "Width: " << W << "\n";
    std::cout << "Height: " << H << "\n";
    std::cout << "pixel_width: " << pixel_width << "\n";
    std::cout << "Pipe: " << pipeName << "\n";

    std::ifstream pipe = makePipe(pipeName);
    
    int size = W * H * pixel_width;
    {
        auto buffer = usePipe(pipe,size);
    }
    BenchMark t;
    while (!stop) {
        auto buffer = usePipe(pipe,size);
        t.cycle_Completed();
    }
    std::cout << "In time " << t.seconds() << endl;
    std::cout << "Bytes transfered per second:" << (t.totalIters/t.seconds()*W*H*pixel_width)/1000/1000 << "Mb/s" << endl;

    return 0;
}
