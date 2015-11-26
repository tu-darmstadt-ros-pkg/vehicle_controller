#ifndef UTILITY_H
#define UTILITY_H

#include <cmath>
#include <geometry_msgs/Point.h>

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

#endif // UTILITY_H
