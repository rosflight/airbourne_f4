/*
 * vector3.h
 *
 *  Created on: Sep 8, 2016
 *      Author: James Jackson
 */


#ifndef INCLUDE_LIB_VECTOR3_H_
#define INCLUDE_LIB_VECTOR3_H_

#include "quaternion.h"
#include "turbotrig.h"

// Forward-Declaration of Quaternion
class Quaternionf;

class vector3
{
public:
    float x;
    float y;
    float z;

    vector3();
    vector3(float x, float y, float z);

    float dot(vector3 v);
    vector3 cross(vector3 v);

    float operator*(vector3& v);

    vector3 operator+(vector3 v);
    vector3& operator+=(vector3 v);
    vector3 operator-(vector3& v);
    vector3& operator-=(vector3 v);

    vector3 operator/(float& s);
    vector3& operator/=(float s);
    vector3 operator*(float s);
    vector3& operator*=(float s);

    vector3 operator*(Quaternionf q);
    vector3& operator*=(Quaternionf q);

    Quaternionf operator >> (vector3 v);
    Quaternionf rotation_between_vectors(vector3 v);

    vector3& normalize();
    float norm();
    float max();
    float min();
    vector3 abs();
    float sqrd_norm();
    vector3& zero();
    vector3& ones();
};


#endif /* INCLUDE_LIB_VECTOR3_H_ */
