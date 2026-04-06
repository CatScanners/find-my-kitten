#ifndef IMAGE_H
#define IMAGE_H

using namespace std;


struct IMG {
    char* data;
    const int width;
    const int height;
    const int pix_width;
    int size() const {
        return pix_width*width*height;
    };
};

#endif