/*
    Copyright (c) 2015, Paul Manns
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef ps3d_h
#define ps3d_h

#include <vehicle_controller/ps3d_motion_parameters.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <deque>
#include <vector>
#include <string>

typedef Eigen::Vector3d vec3;
typedef Eigen::Quaterniond quat;

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vector_vec3;
typedef std::deque<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > deque_vec3;
typedef std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > vector_quat;

class Pathsmoother3D
{
private:
    double const SMOOTHED_PATH_DISCRETIZATION;
    double const PATH_SMOOTHNESS;             // Temperature / smoothness parameter. Current value is manually tuned.
                                              // The smaller the smoother. <-> The bigger the smaller the error to the original path.
    int         MINIMUM_WAY_POINTS;           // Minimum number of points
    bool        allow_reverse_paths;          // Flag indicating if reverse paths are allowed
                                              // Switch on for tracked vehicles
    vec3 const  local_robot_direction;
    PS3dMotionParameters * mp;

public:
    Pathsmoother3D(bool allow_reverse_paths, PS3dMotionParameters * mp);

    /**
     * @brief smooth is the core function of the path smoother, it computes a smoothed path from a given path
     *        based on the predefined discretization and and temperature parameter.
     * @param in_path consists of support points defining a piecewise linear path to be smoothed
     * @param in_start_orientation quaternion containing the robot's orientation at the beginning of the path
     * @param in_end_orientation quaternion containing the robot's orientation desired for the end of the path
     * @param out_smooth_positions consists of support points defining a piecewise linear path after the smoothing
     * @param out_smooth_orientations consists of quaternions defining the robot's orientations desired for the respective support points
     * @param forbid_reverse_path allows the caller to forbid to interpret the path to be tracked by driving reversely (affects the orientations)
     */
    void smooth(deque_vec3 const & in_path, quat const & in_start_orientation, quat const & in_end_orientation, vector_vec3 & out_smooth_positions,
                vector_quat & out_smooth_orientations, bool forbid_reverse_path) const;

protected:
    std::vector<double> computeAccumulatedDistances(deque_vec3 const & positions) const;

    vector_vec3 computeSmoothedPositions(const std::vector<double> &distances, deque_vec3 const & positions) const;

    vector_quat computeSmoothedOrientations(vector_vec3 const & smoothed_positions, quat const & start_orientation, quat const & end_orientation, bool reverse) const;

    double gaussianWeight(double t0, double t1) const;

};

#endif
