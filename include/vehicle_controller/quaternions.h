/*******************************************************************************
 * Copyright (c) 2016, Stefan Kohlbrecher, Johannes Meyer, Paul Manns
 *
 * All rights reserved.
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *   TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *   PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#ifndef NAVIGATION_QUATERNIONS_H
#define NAVIGATION_QUATERNIONS_H

#include <geometry_msgs/Quaternion.h>
#include <string>
#include <math.h>

#include <stdexcept>

#define EULER_CONVENTION "zyx"

using geometry_msgs::Quaternion;

static inline void angles2quaternion(const double angles[], Quaternion &quaternion, const std::string& convention = EULER_CONVENTION, bool flip = false) {
  double cang[3];
  double sang[3];

  if (!flip) {
    cang[0] = cos(angles[0]/2);
    cang[1] = cos(angles[1]/2);
    cang[2] = cos(angles[2]/2);
    sang[0] = sin(angles[0]/2);
    sang[1] = sin(angles[1]/2);
    sang[2] = sin(angles[2]/2);
  } else {
    cang[2] = cos(angles[0]/2);
    cang[1] = cos(angles[1]/2);
    cang[0] = cos(angles[2]/2);
    sang[2] = sin(angles[0]/2);
    sang[1] = sin(angles[1]/2);
    sang[0] = sin(angles[2]/2);
  }

  if (convention == "zyx") {
    quaternion.w = cang[0]*cang[1]*cang[2] + sang[0]*sang[1]*sang[2];
    quaternion.x = cang[0]*cang[1]*sang[2] - sang[0]*sang[1]*cang[2];
    quaternion.y = cang[0]*sang[1]*cang[2] + sang[0]*cang[1]*sang[2];
    quaternion.z = sang[0]*cang[1]*cang[2] - cang[0]*sang[1]*sang[2];
  } else if (convention == "xyz") {
    quaternion.w = cang[0]*cang[1]*cang[2] - sang[0]*sang[1]*sang[2];
    quaternion.x = cang[0]*sang[1]*sang[2] + sang[0]*cang[1]*cang[2];
    quaternion.y = cang[0]*sang[1]*cang[2] - sang[0]*cang[1]*sang[2];
    quaternion.z = cang[0]*cang[1]*sang[2] + sang[0]*sang[1]*cang[2];
  } else {
    throw std::invalid_argument("not supported");
  }
}

static inline void quaternion2angles(const Quaternion& quaternion, double angles[], const std::string& convention = EULER_CONVENTION, bool flip = false) {
  double temp[5];

  if (convention == "zyx") {
    temp[0] =  2*(quaternion.x*quaternion.y + quaternion.w*quaternion.z);
    temp[1] =     quaternion.w*quaternion.w + quaternion.x*quaternion.x - quaternion.y*quaternion.y - quaternion.z*quaternion.z;
    temp[2] = -2*(quaternion.x*quaternion.z - quaternion.w*quaternion.y);
    temp[3] =  2*(quaternion.y*quaternion.z + quaternion.w*quaternion.x);
    temp[4] =     quaternion.w*quaternion.w - quaternion.x*quaternion.x - quaternion.y*quaternion.y + quaternion.z*quaternion.z;
  } else if (convention == "xyz") {
    temp[0] =  -2*(quaternion.y*quaternion.z - quaternion.w*quaternion.x);
    temp[1] =      quaternion.w*quaternion.w - quaternion.x*quaternion.x - quaternion.y*quaternion.y + quaternion.z*quaternion.z;
    temp[2] =   2*(quaternion.x*quaternion.z + quaternion.w*quaternion.y);
    temp[3] =  -2*(quaternion.x*quaternion.y - quaternion.w*quaternion.z);
    temp[4] =      quaternion.w*quaternion.w + quaternion.x*quaternion.x - quaternion.y*quaternion.y - quaternion.z*quaternion.z;
  } else {
    throw std::invalid_argument("not supported");
  }

  angles[0] = atan2(temp[0], temp[1]);
  angles[1] = asin(temp[2]);
  angles[2] = atan2(temp[3], temp[4]);

  if (flip) {
    double temp = angles[2];
    angles[2] = angles[0];
    angles[0] = temp;
  }
}

static inline void euler2quaternion(const double euler[], Quaternion &quaternion) {
  angles2quaternion(euler, quaternion, EULER_CONVENTION, true);
}


static inline void quaternion2euler(const Quaternion& quaternion, double euler[]) {
  quaternion2angles(quaternion, euler, EULER_CONVENTION, true);
}

static inline Quaternion& multiply(Quaternion& q, const Quaternion& q2) {
  Quaternion q1(q);
  q.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
  q.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
  q.y = q1.w*q2.y + q1.y*q2.w - q1.x*q2.z + q1.z*q2.x;
  q.z = q1.w*q2.z + q1.z*q2.w + q1.x*q2.y - q1.y*q2.x;
  return q;
}

static inline Quaternion operator*(const Quaternion& q1, const Quaternion& q2) {
  Quaternion q(q1);
  return multiply(q, q2);
}

static inline Quaternion& conjugate(Quaternion& q) {
  q.x = -q.x;
  q.y = -q.y;
  q.z = -q.z;
  return q;
}

static inline Quaternion conjugated(const Quaternion& q1) {
  Quaternion q(q1);
  return conjugate(q);
}

static inline Quaternion& normalize(Quaternion& q) {
  double norm = sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);
  q.w /= norm;
  q.x /= norm;
  q.y /= norm;
  q.z /= norm;
  return q;
}

static inline Quaternion normalized(const Quaternion& q1) {
  Quaternion q(q1);
  return normalize(q);
}

#endif // NAVIGATION_QUATERNIONS_H
