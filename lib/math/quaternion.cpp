/*
 * quaternion.cpp
 *
 *  Created on: Sep 8, 2016
 *      Author: James Jackson
 */

#include "quaternion.h"
#include "vector3.h"
#include "turbotrig.h"

Quaternionf::Quaternionf()
{
    w = 1.0;
    x = 0.0;
    y = 0.0;
    z = 0.0;
}


Quaternionf::Quaternionf(float _w, float _x, float _y, float _z)
{
    w = _w;
    x = _x;
    y = _y;
    z = _z;
}

Quaternionf::Quaternionf(vector3 v1, vector3 v2)
{
    w = 1.0f + v1.dot(v2);
    vector3 xyz = v1.cross(v2);
    x = xyz.x;
    y = xyz.y;
    z = xyz.z;
}

vector3 Quaternionf::v()
{
    vector3 out(x, y, z);
    return out;
}

Quaternionf& Quaternionf::normalize()
{
    float recipNorm = turboInvSqrt(w*w + x*x + y*y + z*z);
    w = w*recipNorm;
    x = x*recipNorm;
    y = y*recipNorm;
    z = z*recipNorm;

    return *this;
}

Quaternionf Quaternionf::operator * (Quaternionf q)
{
    Quaternionf out(w *q.w - x *q.x - y *q.y - z*q.z,
                    w *q.x + x *q.w - y *q.z + z*q.y,
                    w *q.y + x *q.z + y *q.w - z*q.x,
                    w *q.z - x *q.y + y *q.x + z *q.w);
    return out;
}

Quaternionf& Quaternionf::operator *= (Quaternionf q)
{
    Quaternionf temp;
    temp.w = w *q.w - x *q.x - y *q.y - z*q.z;
    temp.x = w *q.x + x *q.w - y *q.z + z*q.y;
    temp.y = w *q.y + x *q.z + y *q.w - z*q.x;
    temp.z = w *q.z - x *q.y + y *q.x + z *q.w;

    *this = temp;

    return *this;
}

Quaternionf Quaternionf::inverse()
{
    Quaternionf temp = *this;
    temp.x *= -1.0f;
    temp.y *= -1.0f;
    temp.z *= -1.0f;

    return temp;
}

Quaternionf& Quaternionf::invert()
{
    x *= -1.0f;
    y *= -1.0f;
    z *= -1.0f;

    return *this;
}

Quaternionf& Quaternionf::from_two_vectors(vector3 v1, vector3 v2)
{
    w = 1.0f + v1.dot(v2);
    vector3 xyz = v1.cross(v2);
    x = xyz.x;
    y = xyz.y;
    z = xyz.z;

    return *this;
}

void Quaternionf::to_euler_fast(float *phi, float *theta, float *psi)
{
    *phi = atan2_approx(2.0f * (w*x + y*z), 1.0f - 2.0f * (x*x + y*y));
    *theta = asin_approx(2.0f*(w*y - z*x));
    *psi = atan2_approx(2.0f * (w*z + x*y), 1.0f - 2.0f * (y*y + z*z));
}

//void Quaternionf::to_euler_long(float *phi, float *theta, float *psi)
//{
//    *phi = atan2(2.0f * (w*x + y*z), 1.0f - 2.0f * (x*x + y*y));
//    *theta = asin(2.0f*(w*y - z*x));
//    *psi = atan2(2.0f * (w*z + x*y), 1.0f - 2.0f * (y*y + z*z));
//}
