#include <iostream>
#include <cmath>
#include "vector3D.hpp"

vector3D vector3D::operator+(const vector3D& other) const {
    return { x + other.x, y + other.y, z + other.z };
}

vector3D vector3D::operator-(const vector3D& other) const {
    return { x - other.x, y - other.y, z - other.z };
}

vector3D vector3D::operator*(const float& other) const {
    return { x * other, y * other, z * other };
}

vector3D vector3D::operator/(const float& other) const {
    return (*this)*(1.0f/other);
}

void vector3D::operator+=(const vector3D& other) {
    x += other.x; 
    y += other.y; 
    z += other.z;
}
void vector3D::operator-=(const vector3D& other) {
    x -= other.x; 
    y -= other.y; 
    z -= other.z;
}

void vector3D::operator*=(const float& other) {
    x *= other; 
    y *= other; 
    z *= other;
}

vector3D vector3D::elementProduct(const vector3D& other) const {
    return {x * other.x , y * other.y , z * other.z};
}

float vector3D::dot(const vector3D& other) const {
    return x * other.x + y * other.y + z * other.z;
}

vector3D vector3D::crossProduct(const vector3D& other) const {
    return {
        y * other.z - z * other.y,
        z * other.x - x * other.z,
        x * other.y - y * other.x
    };
}


float vector3D::magnitude() const {
    return std::sqrt(x*x + y*y + z*z);
}

vector3D vector3D::normalize() const {
    float mag = magnitude();
    if (mag == 0) return (*this);
    float other = 1.0f/mag;
    return { x * other, y * other, z * other };
}

vector3D vector3D::projectToNormal(const vector3D& other) const {
    vector3D framedown = *this;
    return framedown - other*(framedown.dot(other)/(other.dot(other)));
}

float vector3D::angle(const vector3D& other) const {
    float magA = magnitude();
    float magB = other.magnitude();
    float d = dot(other);

    if (magA == 0.0f || magB == 0.0f) return 0.0f;

    float cosTheta = d / (magA * magB);

    if (cosTheta > 1.0f) cosTheta = 1.0f;
    if (cosTheta < -1.0f) cosTheta = -1.0f;

    return std::acos(cosTheta);
}

vector3D vector3D::rotateTowards(const vector3D& other, float o) const {
    float magA = magnitude();
    float magB = other.magnitude();
    if (magA == 0.0f || magB == 0.0f) return *this;

    // Normalize both
    vector3D a = { x / magA, y / magA, z / magA };
    vector3D b = { other.x / magB, other.y / magB, other.z / magB };

    // Angle between them
    float dotAB = a.dot(b);
    dotAB = std::fmax(-1.0f, std::fmin(1.0f, dotAB));
    float angleAB = std::acos(dotAB);

    // If already aligned or angle is zero
    if (angleAB < 1e-6f) return *this;

    // Clamp rotation angle so we don't overshoot
    float theta = (o > angleAB) ? angleAB : o;

    // Rotation axis
    vector3D axis = a.crossProduct(b);
    float axisMag = axis.magnitude(); 
    if (axisMag < 1e-6f) return *this; // vectors are parallel

    axis = { axis.x / axisMag, axis.y / axisMag, axis.z / axisMag };

    // Rodrigues' rotation formula
    float cosT = std::cos(theta);
    float sinT = std::sin(theta);

    vector3D term1 = { a.x * cosT, a.y * cosT, a.z * cosT };
    vector3D term2 = axis.crossProduct(a);
    term2 = { term2.x * sinT, term2.y * sinT, term2.z * sinT };
    float axisDotA = axis.dot(a);
    vector3D term3 = {
        axis.x * axisDotA * (1 - cosT),
        axis.y * axisDotA * (1 - cosT),
        axis.z * axisDotA * (1 - cosT)
    };

    vector3D result = {
        term1.x + term2.x + term3.x,
        term1.y + term2.y + term3.y,
        term1.z + term2.z + term3.z
    };

    // Restore original magnitude
    return { result.x * magA, result.y * magA, result.z * magA };
}



std::ostream& operator<<(std::ostream& os, const vector3D& vec) {
    os << "{ x: " << vec.x << ", y: "  << vec.y << ", z: " << vec.z << " }";
    return os;
}