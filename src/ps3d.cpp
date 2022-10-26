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

#include <eigen_conversions/eigen_msg.h>
#include <cmath>
#include <ros/ros.h>

void Pathsmoother3D::setSmoothedPathDiscretization(double value)
{
  SMOOTHED_PATH_DISCRETIZATION = value;
}

void Pathsmoother3D::setPathSmoothness(double value)
{
  PATH_SMOOTHNESS = value;
}

Pathsmoother3D::Pathsmoother3D(bool allow_reverse_paths, bool ignore_goal_orientation)
  : SMOOTHED_PATH_DISCRETIZATION(0.05),// Hector best practice values
  PATH_SMOOTHNESS(0.125),            // Hector best practice values
  allow_reverse_paths(allow_reverse_paths),
  ignore_goal_orientation(ignore_goal_orientation),
  local_robot_direction(Eigen::Vector3d::UnitX()) // X points to the front
{

}

double Pathsmoother3D::gaussianWeight(double t0, double t1) const
{
  return exp(-pow(t0 - t1, 2.0) / (2.0 *  pow(PATH_SMOOTHNESS, 2.0)));
}

std::vector<double> Pathsmoother3D::computeAccumulatedDistances(const std::deque<Eigen::Vector3d>& positions)
{
  std::vector<double> result(positions.size(), 0);
  for(unsigned i = 1; i < positions.size(); i++)
    result[i] = result[i - 1] + (positions[i] - positions[i - 1]).norm();
  return result;
}

void Pathsmoother3D::smooth(const std::deque<Eigen::Vector3d>& in_path, const Eigen::Quaterniond& in_start_orientation, const Eigen::Quaterniond& in_end_orientation, std::vector<Eigen::Vector3d>& out_smooth_positions,
                            std::vector<Eigen::Quaterniond> & out_smooth_orientations, bool reverse) const
{
  // Missing
  // forbid_reverse_path has to switched on by the user if the robot is too far away from the path.

  std::vector<Eigen::Vector3d> smoothed_positions;
  std::vector<Eigen::Quaterniond> smoothed_orientations;

  std::vector<double> distances = computeAccumulatedDistances(in_path);
  smoothed_positions = computeSmoothedPositions(distances, in_path);

  if(allow_reverse_paths)
  {
    if(in_path.size() >= 2 && smoothed_positions.size() >= in_path.size())
    {
      if(reverse)
      {
        Eigen::Vector2d start_path_delta = (smoothed_positions[1] - smoothed_positions[0]).block<2, 1>(0, 0).normalized();
        Eigen::Vector2d robot_direction_world = (in_start_orientation * local_robot_direction).block<2, 1>(0, 0).normalized();
        double start_projection = start_path_delta.dot(robot_direction_world); // cos angle between robot and path
        reverse = start_projection < 0.0; // Angle > 90° -> robot is pointing away from the path

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

        Eigen::Vector3d start_path_delta = (smoothed_positions[0] - smoothed_positions[1]).normalized();
        double start_path_projection = start_path_delta.dot(in_start_orientation * local_robot_direction);

        Eigen::Vector3d end_path_delta = (smoothed_positions[smoothed_positions.size() - 2] - smoothed_positions.back()).normalized();
        Eigen::Vector3d end_vec = (in_end_orientation * local_robot_direction).normalized();
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

nav_msgs::Path Pathsmoother3D::smooth(const nav_msgs::Path& path_in, bool reverse) const
{
  std::deque<Eigen::Vector3d> in_path;
  std::transform(path_in.poses.begin(), path_in.poses.end(), std::back_inserter(in_path),
                 [](geometry_msgs::PoseStamped const & pose_)
                 { return Eigen::Vector3d(pose_.pose.position.x, pose_.pose.position.y, pose_.pose.position.z); });

  Eigen::Quaterniond in_start_orientation;
  //  = geomEigen::Quaterniond2EigenQuat(robot_control_state.pose.orientation);
  tf::quaternionMsgToEigen(path_in.poses.front().pose.orientation, in_start_orientation);
  Eigen::Quaterniond in_end_orientation;
  tf::quaternionMsgToEigen(path_in.poses.back().pose.orientation, in_end_orientation);

  std::vector<Eigen::Vector3d> out_smoothed_positions;
  std::vector<Eigen::Quaterniond> out_smoothed_orientations;

  smooth(in_path, in_start_orientation, in_end_orientation,
         out_smoothed_positions, out_smoothed_orientations, reverse);

  nav_msgs::Path path_out;
  path_out.header = path_in.header;
  for (unsigned int i = 0; i < out_smoothed_positions.size(); i++) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = path_out.header.frame_id;
    tf::pointEigenToMsg(out_smoothed_positions[i], pose.pose.position);
    tf::quaternionEigenToMsg(out_smoothed_orientations[i], pose.pose.orientation);
    path_out.poses.push_back(pose);
  }

  return path_out;
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
std::vector<Eigen::Vector3d> Pathsmoother3D::computeSmoothedPositions(const std::vector<double>& distances, const std::deque<Eigen::Vector3d>& positions) const
{
  // The total distance is in accumulatedDistances(...).back()
  // => sample path along the accumulated (approx. for the time needed for the path).
  // ROS_INFO("total linear distance = %f \n", distances.back());

  std::vector<Eigen::Vector3d> smoothed_positions;
  smoothed_positions.reserve(distances.back() / SMOOTHED_PATH_DISCRETIZATION + 1);

  std::vector<double> samples;
  samples.reserve(distances.back() / SMOOTHED_PATH_DISCRETIZATION);

  std::vector<Eigen::Vector3d> samplesX;
  samplesX.reserve(distances.back() / SMOOTHED_PATH_DISCRETIZATION);

  auto itT = distances.begin();
  auto itX = positions.begin();

  for(double d = 0; d < distances.back(); d += SMOOTHED_PATH_DISCRETIZATION)
  {
    while(d > *(itT + 1))
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
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    double weight = 0;
    int step = std::min(static_cast<int>(i), static_cast<int>(samples.size() - 1) - static_cast<int>(i));
    for(unsigned j = i - step; j < i + step + 1; ++j)
      weight += gaussianWeight(samples[i], samples[j]);
    for(unsigned j = i - step; j < i + step + 1; ++j)
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

std::vector<Eigen::Quaterniond> Pathsmoother3D::computeSmoothedOrientations(const std::vector<Eigen::Vector3d>& smoothed_positions, const Eigen::Quaterniond& start_orientation, const Eigen::Quaterniond& end_orientation, bool reverse) const
{
  std::vector<Eigen::Quaterniond> smoothed_orientations;
  smoothed_orientations.reserve(smoothed_positions.size());
  double reverse_factor = reverse ? -1.0 : 1.0;
  for (unsigned i = 0; i < smoothed_positions.size(); i++)
  {
    // Requires smoothed positions
    // robot_direction points to next position
    if (0 < i && i < smoothed_positions.size() - 1)
    {
      Eigen::Quaterniond q;
      Eigen::Vector3d delta_vector = smoothed_positions[i + 1] - smoothed_positions[i];
      delta_vector.z() = 0; // If this is != 0, it leads to solutions rotated around X (local_robot_direction)
      q.setFromTwoVectors(reverse_factor * local_robot_direction, delta_vector);
      smoothed_orientations.push_back(q);
    }
    else if (i == smoothed_positions.size() - 1) {
      if (!ignore_goal_orientation) {
        smoothed_orientations.push_back(end_orientation);
      }
      else {
        smoothed_orientations.push_back(smoothed_orientations.back()); // Repeat last orientation
      }
    }
    else // i == 0
      smoothed_orientations.push_back(start_orientation);
  }
  return smoothed_orientations;
}