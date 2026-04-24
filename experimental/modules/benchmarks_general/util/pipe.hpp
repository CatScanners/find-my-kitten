#ifndef FIFO_PIPE
#define FIFO_PIPE
#include <iostream>
#include <vector>
#include <thread>
#include <fstream>
#include <unistd.h>
#include <sys/stat.h>
#include <functional>

using PointIntFunc = std::function<std::tuple<char*, int>()>;

void createPipe(const char* pipeName, PointIntFunc func);

std::ifstream makePipe(const char* pipeName) ;
std::vector<char> usePipe(std::ifstream &pipe, int size) ;

#endif 