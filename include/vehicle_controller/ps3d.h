#ifndef ps3d_h
#define ps3d_h

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
    float const SMOOTHED_PATH_DISCRETIZATION;
    float const PATH_SMOOTHNESS;              // Smoothing parameter. Current value is hand tuend
                                              // The smaller the smoother.
                                              // The bigger the smaller the error to the original path.
    int         MINIMUM_WAY_POINTS;           // Minimum number of points
    bool        allow_reverse_paths;          // Flag indicating if reverse paths are allowed
                                              // Switch on for tracked vehicles
    vec3 const  local_robot_direction;


public:
    Pathsmoother3D(bool allow_reverse_paths);

    void smooth(deque_vec3 const & in_path, quat const & in_start_orientation, quat const & in_end_orientation, vector_vec3 & out_smooth_positions,
                vector_quat & out_smooth_orientations, bool forbid_reverse_path);

    bool path2BeSmoothed(deque_vec3 const & transformed_path);

    void computePathMatricesStrings4R(deque_vec3 const & in_original_path, vector_vec3 const & in_smoothed_path,
                                      std::string & out_trans, std::string & out_smooth);

protected:
    std::vector<float> computeAccumulatedDistances(deque_vec3 const & positions);

    vector_vec3 computeSmoothedPositions(std::vector<float> const & distances, deque_vec3 const & positions);

    vector_quat computeSmoothedOrientations(vector_vec3 const & smoothed_positions, quat const & start_orientation, quat const & end_orientation, bool reverse);

    float gaussianWeight(float t0, float t1);

};

#endif
