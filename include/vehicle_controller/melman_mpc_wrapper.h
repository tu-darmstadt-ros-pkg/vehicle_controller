#ifndef MELMAN_MPC_WRAPPER_H
#define MELMAN_MPC_WRAPPER_H

#include <vehicle_controller/utility.h>
#include <vehicle_controller/vehicle_control_interface.h>
#include <vehicle_controller/motion_parameters.h>

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado_gnuplot/gnuplot_window.hpp>

namespace ACADO_NS = ACADO;

class MelmanMpcWrapper
{
private:
    double c = 1.0;
    double d = 0.00;

    ACADO_NS::DMatrix Q;
    ACADO_NS::DVector r;

    MotionParameters const & mp;

//    ACADO_NS::DifferentialState theta, xs, ys, xc, yc;

    ACADO_NS::DifferentialState theta, x, y;
    ACADO_NS::Control u1, u2;
    ACADO_NS::Parameter T;
    ACADO_NS::DifferentialEquation f;

    ACADO_NS::Function * h = 0;
    ACADO_NS::OCP      * ocp = 0;
    ACADO_NS::StaticReferenceTrajectory * ref_traj = 0;
    ACADO_NS::VariablesGrid * ref_traj_grid = 0;

    ACADO_NS::RealTimeAlgorithm * alg = 0;
    ACADO_NS::OptimizationAlgorithm * oalg = 0;
    ACADO_NS::Controller * controller = 0;

protected:
    void setupODE();

public:
    MelmanMpcWrapper(MotionParameters const & mp);
    ~MelmanMpcWrapper();

    void updatePath(Legs const & legs, Point state);
    bool feedbackStep(Point state, geometry_msgs::Vector3 target, double t, geometry_msgs::Twist &twist);
};

#endif // MELMAN_MPC_WRAPPER_H
