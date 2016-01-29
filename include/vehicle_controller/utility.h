/*******************************************************************************
 * Copyright (c) 2016, Stefan Kohlbrecher, Paul Manns
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

#ifndef UTILITY_H
#define UTILITY_H

#include <cmath>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <Eigen/Dense>


struct Point
{
  float x;
  float y;
  float orientation;
};

struct Leg
{
  Point p1;
  Point p2;
  float course;

  bool backward;
  float speed;
  float length2;
  float length;
  float percent;
};

typedef std::vector<Leg> Legs;

inline static double constrainAngle_0_2pi(double x)
{
    x = fmod(x, 2.0 * M_PI);
    if (x < 0)
        x += 2.0 * M_PI;
    return x;
}

inline static double constrainAngle_mpi_pi(double x)
{
    x = fmod(x + M_PI, 2.0 * M_PI);
    if (x < 0)
        x += 2.0 * M_PI;
    return x - M_PI;
}

inline static double angularNorm(double diff)
{
    static const double M_2PI = 2.0 * M_PI;
    diff -= floor(diff/M_2PI + .5)*M_2PI;
    return diff;
}

inline static double euclideanDistance(geometry_msgs::Point const & p0, geometry_msgs::Point const & p1)
{
  return std::sqrt(std::pow(p1.x - p0.x, 2) + std::pow(p1.y - p0.y, 2) + pow(p1.z - p0.z, 2));
}

inline static Eigen::Quaterniond geomQuat2EigenQuat(geometry_msgs::Quaternion const & quat)
{
    return Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
}

inline static geometry_msgs::Point geomVec32GeomPoint(geometry_msgs::Vector3 const & v)
{
    geometry_msgs::Point p;
    p.x = v.x;
    p.y = v.y;
    p.z = p.z;
    return p;
}

#endif // UTILITY_H
