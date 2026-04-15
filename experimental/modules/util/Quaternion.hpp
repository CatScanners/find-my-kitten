#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include "vector3D.hpp"
#include <iosfwd>

struct Quaternion {
    float w, x, y, z;

    Quaternion() : w(1.0f), x(0.0f), y(0.0f), z(0.0f) {}

    Quaternion(float w_, float x_, float y_, float z_)
        : w(w_), x(x_), y(y_), z(z_) {}

    Quaternion fromAxisAngle(const vector3D vec, float angleRad) ;

    void normalize() ;

    Quaternion operator*(const Quaternion& q) const ;

    float fakeMagnitude() ;

    Quaternion operator-(const Quaternion q) const ;

    Quaternion fromTwoVectors(const vector3D from, const vector3D to) ;

    Quaternion fromTwoVectorsScaledDown(const vector3D from, const vector3D to, const float factor) ;
    Quaternion inverse() const ;
    Quaternion z_axis_component() const ;


    vector3D rotateVector(vector3D v) const ;

    vector3D rotateVectorReverse(vector3D v) const ;
};

std::ostream& operator<<(std::ostream& os, const Quaternion& vec) ;

#endif
