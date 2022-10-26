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

#include <Eigen/Eigen>
#include <deque>
#include <nav_msgs/Path.h>

class Pathsmoother3D
{
private:
  double SMOOTHED_PATH_DISCRETIZATION;
  double PATH_SMOOTHNESS;             // Temperature / smoothness parameter. Current value is manually tuned.
                             // The smaller the smoother. <-> The bigger the smaller the error to the original path.
  bool allow_reverse_paths;  // Flag indicating if reverse paths are allowed
                             // Switch on for tracked vehicles
  bool ignore_goal_orientation;
  Eigen::Vector3d local_robot_direction;

public:
  explicit Pathsmoother3D(bool allow_reverse_paths, bool ignore_goal_orientation);

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
  void smooth(const std::deque<Eigen::Vector3d>& in_path, const Eigen::Quaterniond& in_start_orientation, const Eigen::Quaterniond& in_end_orientation, std::vector<Eigen::Vector3d>& out_smooth_positions,
              std::vector<Eigen::Quaterniond> & out_smooth_orientations, bool reverse) const;

  /**
     * @brief smooth Convenience function to smooth a path given by a msg
     * @param path_in Path to be smoothed
     * @return Smoothed path
   */
  nav_msgs::Path smooth(const nav_msgs::Path& path_in, bool reverse) const;

  void setSmoothedPathDiscretization(double value);
  void setPathSmoothness(double value);

protected:
  static std::vector<double> computeAccumulatedDistances(const std::deque<Eigen::Vector3d>& positions) ;

  std::vector<Eigen::Vector3d> computeSmoothedPositions(const std::vector<double>& distances, const std::deque<Eigen::Vector3d>& positions) const;

  std::vector<Eigen::Quaterniond> computeSmoothedOrientations(const std::vector<Eigen::Vector3d>& smoothed_positions, const Eigen::Quaterniond& start_orientation, const Eigen::Quaterniond& end_orientation, bool reverse) const;

  double gaussianWeight(double t0, double t1) const;
};

#endif
