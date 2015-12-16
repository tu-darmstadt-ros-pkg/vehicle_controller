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

#include <vehicle_controller/ps3d.h>
#include <vehicle_controller/quaternions.h>

#include <cmath>

#include <ros/ros.h>

using std::vector;
using std::exp;
using std::pow;

Pathsmoother3D::Pathsmoother3D(bool allow_reverse_paths, PS3dMotionParameters * mp)
    : SMOOTHED_PATH_DISCRETIZATION(0.05),// Hector best practice values
      PATH_SMOOTHNESS(0.125),            // Hector best practice values
      allow_reverse_paths(allow_reverse_paths),
      local_robot_direction(vec3(1,0,0)), // ROS Cosy has always x axis being perpendicular to the front of the robot
      mp(mp)
{

}

double Pathsmoother3D::gaussianWeight(double t0, double t1) const
{
  return exp(-pow(t0 - t1, 2.0) / (2.0 *  pow(PATH_SMOOTHNESS, 2.0)));
}

vector<double> Pathsmoother3D::computeAccumulatedDistances(deque_vec3 const & positions) const
{
  vector<double> result(positions.size(), 0);
  for(unsigned i = 1; i < positions.size(); i++)
    result[i] = result[i - 1] + (positions[i] - positions[i - 1]).norm();
  return result;
}

void Pathsmoother3D::smooth(deque_vec3 const & in_path,
                            quat const & in_start_orientation,
                            quat const & in_end_orientation,
                            vector_vec3 & out_smooth_positions,
                            vector_quat & out_smooth_orientations,
                            bool forbid_reverse_path) const
{
    // Missing
    // forbid_reverse_path has to switched on by the user if the robot is too far away from the path.

    vector_vec3 smoothed_positions;
    vector_quat smoothed_orientations;

    vector<double> distances = computeAccumulatedDistances(in_path);
    smoothed_positions = computeSmoothedPositions(distances, in_path);

    bool reverse = false;
    if(allow_reverse_paths && !forbid_reverse_path)
    {
        if(in_path.size() >= 2 && smoothed_positions.size() >= in_path.size())
        {
            if(mp->isYSymmetric())
            {
                vec3 start_path_delta = (smoothed_positions[0] - smoothed_positions[1]).normalized();
                double start_projection = start_path_delta.dot(in_start_orientation * local_robot_direction);
                reverse = start_path_delta.dot(in_start_orientation * local_robot_direction) > 0.0;

                if(reverse)
                    ROS_INFO("[Pathsmoother3D] REVERSE! start_projection = %f", start_projection);
            }
            else
            {
                // Assume global COSY = COSY in position[0], current direction of looking = (1,0,0)
                // given: smoothed_positions[0] in WORLD COORDINATES
                // given: rotation at position 0
                // searched: direction the robot is looking in LOCAL COORDINATES

                bool distC = distances.back() < 1.5;

                vec3 start_path_delta = (smoothed_positions[0] - smoothed_positions[1]).normalized();
                double start_path_projection = start_path_delta.dot(in_start_orientation * local_robot_direction);

                vec3 end_path_delta = (smoothed_positions[smoothed_positions.size() - 2] - smoothed_positions.back()).normalized();
                vec3 end_vec = (in_end_orientation * local_robot_direction).normalized();
                double end_path_projection = end_path_delta.dot(end_vec);

                bool startC = start_path_projection > 0;
                bool endC = end_path_projection > 0;

                reverse = distC && startC && endC;

                if(reverse)
                    ROS_WARN("[Pathsmoother3D] REVERSE! dist = %d, start = %d, end = %d", distC, startC, endC);
            }
        }
    }

    smoothed_orientations = computeSmoothedOrientations(smoothed_positions, in_start_orientation, in_end_orientation, reverse);
    out_smooth_positions = smoothed_positions;
    out_smooth_orientations = smoothed_orientations;
}

/**
 * @brief Pathsmoother3D::computeSmoothedPositions computes smoothed positions from the piecewise linear input
 *        path positions with the configured discretization and smoothness.
 *        The function assumes distances.size() == positions.size() otherwise the result
 *        is undefined and segmentation fault crashes may happen
 * @param distances array, distance[i] = distance from position[0] to position[i] along the path defined by
 *        positions
 * @param positions defines a piecewise linear input path
 * @return a smoothed piecewise linear output path
 */
vector_vec3 Pathsmoother3D::computeSmoothedPositions(std::vector<double> const & distances,
                                                     deque_vec3 const & positions) const
{
    // The total distance is in accumulatedDistances(...).back()
    // => sample path along the accumulated (approx. for the time needed for the path).
    // ROS_INFO("total linear distance = %f \n", distances.back());

    vector_vec3 smoothed_positions;
    smoothed_positions.reserve(distances.back() / SMOOTHED_PATH_DISCRETIZATION + 1);

    std::vector<double> samples;
    samples.reserve(distances.back() / SMOOTHED_PATH_DISCRETIZATION);

    vector_vec3 samplesX;
    samplesX.reserve(distances.back() / SMOOTHED_PATH_DISCRETIZATION);

    vector<double>::const_iterator itT = distances.begin();
    deque_vec3::const_iterator itX = positions.begin();

    for(double d = 0; d < distances.back(); d += SMOOTHED_PATH_DISCRETIZATION)
    {
        if(d > *(itT + 1))
        {
            itT++;
            itX++;
        }
        samples.push_back(d);
        samplesX.push_back(*itX + (*(itX + 1) - *itX) * (d - *itT) / (*(itT + 1) - *itT));
    }
    samples.push_back(distances.back());
    samplesX.push_back(positions.back());

    smoothed_positions.clear();
    smoothed_positions.push_back(positions.front());
    for(unsigned i = 1; i < samples.size() - 1; ++i)
    {
        vec3 p = vec3::Zero();
        double weight = 0;
        for(unsigned j = 0; j < samples.size(); ++j)
            weight += gaussianWeight(samples[i], samples[j]);
        for(unsigned j = 0; j < samples.size(); ++j)
        {
            double w_ij = gaussianWeight(samples[i], samples[j]);
            p(0) += (w_ij / weight) * samplesX[j](0);
            p(1) += (w_ij / weight) * samplesX[j](1);
            p(2) += (w_ij / weight) * samplesX[j](2);
        }
        smoothed_positions.push_back(p);
    }
    smoothed_positions.push_back(positions.back());
    return smoothed_positions;
}

vector_quat Pathsmoother3D::computeSmoothedOrientations(vector_vec3 const & smoothed_positions,
                                                        quat const & start_orientation,
                                                        quat const & end_orientation,
                                                        bool reverse) const
{
    vector_quat smoothed_orientations;
    smoothed_orientations.reserve(smoothed_positions.size());
    if(!reverse)
    {
        for(unsigned i = 0; i < smoothed_positions.size(); i++)
        {
            // Requires smoothed positions
            // Computes the related orientations with forward difference approximation based on
            // stepwidth of the smoothed path discretization over "virtual time variable"
            if(0 < i && i < smoothed_positions.size() - 1)
            {
                quat q;
                q.setFromTwoVectors(local_robot_direction, smoothed_positions[i + 1] - smoothed_positions[i]);
                smoothed_orientations.push_back(q);
            }
            else if(i == smoothed_positions.size() - 1)
                smoothed_orientations.push_back(end_orientation);
            else // i == 0
                smoothed_orientations.push_back(start_orientation);
        }
    }
    else
    {
        // Requires smoothed positions
        // Computes the related orientations with negative reverse difference approximation based on
        // stepwidth of the smoothed path discretization over "virtual time variable"
        for(unsigned i = 0; i < smoothed_positions.size(); i++)
        {
            if(0 < i && i < smoothed_positions.size() - 1)
            {
                quat q;
                q.setFromTwoVectors(local_robot_direction, smoothed_positions[i] - smoothed_positions[i + 1]);
                smoothed_orientations.push_back(q);
            }
            else if(i == smoothed_positions.size() - 1)
                smoothed_orientations.push_back(end_orientation);
            else // i == 0
                smoothed_orientations.push_back(start_orientation);
        }
    }
    return smoothed_orientations;
}
