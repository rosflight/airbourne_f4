/*
 * vector3.cpp
 *
 *  Created on: Sep 8, 2016
 *      Author: James Jackson
 */

#include "vector3.h"

vector3::vector3()
{
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
}

vector3::vector3(float _x, float _y, float _z)
{
    x = _x;
    y = _y;
    z = _z;
}

float vector3::dot(vector3 v)
{
    return x*v.x + y*v.y + z*v.z;
}

vector3 vector3::cross(vector3 v)
{
    vector3 out(y * v.z - z * v.y,
                z * v.x - x * v.z,
                x * v.y - y * v.x);
    return out;
}

float vector3::operator *(vector3& v)
{
    return x*v.x + y*v.y + z*v.z;
}

vector3 vector3::operator +(vector3 v)
{
    vector3 out(x + v.x, y + v.y, z + v.z);
    return out;
}

vector3& vector3::operator += (vector3 v)
{
    x = x + v.x;
    y = y + v.y;
    z = z + v.z;

    return *this;
}

vector3 vector3::operator -(vector3& v)
{
    vector3 out(x - v.x, y - v.y, z - v.z);
    return out;
}

vector3& vector3::operator -= (vector3 v)
{
    x = x - v.x;
    y = y - v.y;
    z = z - v.z;
    return *this;
}


vector3 vector3::operator/(float& s)
{
    vector3 out(x/s, y/s, z/s);
    return out;
}

vector3& vector3::operator/=(float s)
{
    x /= s;
    y /= s;
    z /= s;
    return *this;
}

vector3 vector3::operator*(float s)
{
    vector3 out(x*s, y*s, z*s);
    return out;
}

vector3& vector3::operator*=(float s)
{
    x *= s;
    y *= s;
    z *= s;
    return *this;
}

vector3 vector3::operator*(Quaternionf q)
{
    // from http://gamedev.stackexchange.com/questions/28395/rotating-vector3-by-a-quaternion
    vector3 u(q.x, q.y, q.z);
    float s = q.w;

    vector3 out = u * (2.0f * u.dot((*this)))
            + (*this) * (s*s - u.dot(u))
            + u.cross((*this)) * (2.0f * s);
    return out;
}

vector3& vector3::operator*=(Quaternionf q)
{
    *this = (*this)*q;
}

Quaternionf vector3::rotation_between_vectors(vector3 v)
{
    float w = 1.0f + this->dot(v);
    vector3 xyz = this->cross(v);
    Quaternionf q(w, xyz.x, xyz.y, xyz.z);
    return q.normalize();

}

Quaternionf vector3::operator>>(vector3 v)
{
    return rotation_between_vectors(v);
}

vector3& vector3::normalize()
{
    float recipNorm = turboInvSqrt(x*x + y*y + z*z);
    x = x*recipNorm;
    y = y*recipNorm;
    z = z*recipNorm;

    return *this;
}

float vector3::norm()
{
    return 1.0/turboInvSqrt(x*x + y*y + z*z);
}

float vector3::max()
{
    float max = x;
    max = (y > max) ? y : max;
    max = (z > max) ? z : max;
    return max;
}

float vector3::min()
{
    float min = x;
    min = (y < min) ? y : min;
    min = (z < min) ? z : min;
    return min;
}

vector3 vector3::abs()
{
    vector3 out((x < 0.0) ? -x : x,
                (y < 0.0) ? -y : y,
                (z < 0.0) ? -z : z);
    return out;
}

float vector3::sqrd_norm()
{
    return x*x + y*y + z*z;
}

vector3& vector3::zero()
{
    x = 0.0;
    y = 0.0;
    z = 0.0;
    return *this;
}

vector3& vector3::ones()
{
    x = 1.0;
    y = 1.0;
    z = 1.0;
    return *this;
}









