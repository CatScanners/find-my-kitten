#include "Quaternion.hpp"
#include "vector3D.hpp"
#include <cmath>
#include <iostream>

Quaternion Quaternion::fromAxisAngle(const vector3D vec, float angleRad)
{
    vector3D axis = vec.normalize();
    float half = angleRad * 0.5f;
    float s = std::sin(half);

    return Quaternion(
        std::cos(half),
        axis.x * s,
        axis.y * s,
        axis.z * s
    );
}

void Quaternion::normalize()
{
    float mag = std::sqrt(w*w + x*x + y*y + z*z);
    if (mag > 0.0f)
    {
        float inv = 1.0f / mag;
        w *= inv; x *= inv; y *= inv; z *= inv;
    }
}

Quaternion Quaternion::operator*(const Quaternion& q) const
{
    return Quaternion(
        w*q.w - x*q.x - y*q.y - z*q.z,
        w*q.x + x*q.w + y*q.z - z*q.y,
        w*q.y - x*q.z + y*q.w + z*q.x,
        w*q.z + x*q.y - y*q.x + z*q.w
    );
}

float Quaternion::fakeMagnitude() {return std::sqrt(w*w + x*x + y*y + z*z);}
Quaternion Quaternion::operator-(const Quaternion q) const
{
    return Quaternion(
        w-q.w , x-q.x , y-q.y , z-q.z
    );
}

Quaternion Quaternion::fromTwoVectors(const vector3D from, const vector3D to)
{
    vector3D f = from.normalize();
    vector3D t = to.normalize();

    float dot = f.dot(t);

    if (dot > 0.999999f)
        return Quaternion(); 

    vector3D axis = f.crossProduct(t); 

    float angle = std::acos(dot);
    return fromAxisAngle(axis, angle);
}

Quaternion Quaternion::fromTwoVectorsScaledDown(const vector3D from, const vector3D to, const float factor)
{
    vector3D f = from.normalize();
    vector3D t = to.normalize();

    float dot = f.dot(t);

    if (dot > 0.999999f)
        return Quaternion();

    vector3D axis = f.crossProduct(t); 

    float angle = std::acos(dot);
    return fromAxisAngle(axis, angle*factor);
}

Quaternion Quaternion::inverse() const {
    return {w, -x, -y, -z};
}


vector3D Quaternion::rotateVector(vector3D v) const
{
    Quaternion p(0, v.x, v.y, v.z);
    Quaternion inv = inverse();

    Quaternion result = (*this) * p * inv;
    vector3D resVec =  {result.x, result.y, result.z};
    return resVec;
}

vector3D Quaternion::rotateVectorReverse(vector3D v) const
{
    Quaternion p(0, v.x, v.y, v.z);
    Quaternion inv(w, -x, -y, -z);

    Quaternion result = inv * p * (*this);
    vector3D resVec =  {result.x, result.y, result.z};
    return resVec;
}

std::ostream& operator<<(std::ostream& os, const Quaternion& vec) {
    os << "{ w: " << vec.w << ", x: " << vec.x << ", y: "  << vec.y << ", z: " << vec.z << " }";
    return os;
}

