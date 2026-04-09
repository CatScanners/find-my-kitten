#ifndef VECTOR3D_H
#define VECTOR3D_H

#include <iosfwd>

struct vector3D {
    float x = 0;
    float y = 0;
    float z = 0;

    vector3D operator+(const vector3D& other) const ;

    vector3D operator-(const vector3D& other) const ;

    vector3D operator*(const float& other) const ;

    vector3D operator/(const float& other) const ;

    void operator+=(const vector3D& other) ;
    void operator-=(const vector3D& other) ;
    
    void operator*=(const float& other) ;
    
    float dot(const vector3D& other) const ;

    vector3D crossProduct(const vector3D& other) const ;

    float magnitude() const ;

    vector3D normalize() const ;

    vector3D projectToNormal(const vector3D& other) const ;

    float angle(const vector3D& other) const ;

    vector3D rotateTowards(const vector3D& other, float o) const ;
};


std::ostream& operator<<(std::ostream& os, const vector3D& vec) ;

constexpr vector3D EMPTY_VECTOR3D = {0,0,0};

#endif