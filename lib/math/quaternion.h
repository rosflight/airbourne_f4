/*
 * quaternion.h
 *
 *  Created on: Sep 8, 2016
 *      Author: James Jackson
 */

#ifndef INCLUDE_LIB_QUATERNIONF_H_
#define INCLUDE_LIB_QUATERNIONF_H_

#include "vector3.h"

class vector3;

class Quaternionf
{

public:

    float w;
    float x;
    float y;
    float z;

    Quaternionf();

    Quaternionf(float w, float x, float y, float z);
    Quaternionf(vector3 v1, vector3 v2);

    vector3 v();

    Quaternionf& normalize();

    Quaternionf& from_two_vectors(vector3 v1, vector3 v2);
    Quaternionf operator * (Quaternionf q);
    Quaternionf& operator *= (Quaternionf q);
    Quaternionf inverse();
    Quaternionf& invert();

    void to_euler_fast(float* phi, float* theta, float* psi);
//    void to_euler_long(float* phi, float* theta, float* psi);


};

#endif /* INCLUDE_LIB_QUATERNIONF_H_ */


