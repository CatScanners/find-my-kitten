#ifndef IMAGE_H
#define IMAGE_H

struct IMG {
    char* data;
    const int width;
    const int height;
    const int pix_width;
    int size() const ;
};

#endif