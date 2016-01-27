#include <vehicle_controller/melman_mpc_wrapper.h>

#include <algorithm>
#include <chrono>

#include <ros/ros.h>

USING_NAMESPACE_ACADO

MelmanMpcWrapper::MelmanMpcWrapper(const MotionParameters &mp)
    : Q(4,4), r(4), mp(mp)//, f(DifferentialEquation(0.0, T))
{
    setupODE();
}

MelmanMpcWrapper::~MelmanMpcWrapper()
{
    if(ocp)
        delete ocp;
    if(h)
        delete h;
    if(ref_traj)
        delete ref_traj;
    if(ref_traj_grid)
        delete ref_traj_grid;
    if(alg)
        delete alg;
    if(controller)
        delete controller;
}

void MelmanMpcWrapper::setupODE()
{
    f << dot(theta) == u2;
    f << dot(x)    == c * u1 * cos(theta);
    f << dot(y)    == c * u1 * sin(theta);
}

void MelmanMpcWrapper::updatePath(Legs const & legs, Point state)
{
    if(ocp)
    {
        delete ocp;
        ocp = 0;
    }
    if(h)
    {
        delete h;
        h = 0;
    }
    if(ref_traj)
    {
        delete ref_traj;
        ref_traj = 0;
    }
    if(ref_traj_grid)
    {
        delete ref_traj_grid;
        ref_traj_grid = 0;
    }
    if(alg)
    {
        delete alg;
        alg = 0;
    }
    if(controller)
    {
        delete controller;
        controller = 0;
    }
}

bool MelmanMpcWrapper::feedbackStep(Point state,
    geometry_msgs::Vector3 target, double t, geometry_msgs::Twist & twist)
{
    auto acado_eval_time_start = std::chrono::system_clock::now();

    double u[2] = {0, 0};

    double u_lin_M = 0.5;
    double u_ang_M = 0.3;


    double alpha = state.orientation;
    double beta  = atan2(target.y - state.y, target.x - state.x);
    double error_2_path   = constrainAngle_mpi_pi( beta - alpha );

    if(error_2_path > M_PI_2)
        error_2_path = error_2_path - M_PI;
    if(error_2_path < -M_PI_2)
        error_2_path = M_PI + error_2_path;

    double t_upper_limit =
            std::sqrt(std::pow(target.x - state.x, 2.0)
                    + std::pow(target.y - state.y, 2.0)) / u_lin_M
          + std::abs(error_2_path) / u_ang_M;
    t_upper_limit *= 1.2;

    double t_E = t_upper_limit;

    if (ocp)
    {
        delete ocp;
        ocp = 0;
    }
    if (oalg)
    {
        delete oalg;
        oalg = 0;
    }

    ocp = new OCP(0.0, t_E, 10);
    ocp->subjectTo(f);
    // ocp->minimizeMayerTerm(T + u1 * u1 + u2 * u2);

    ocp->minimizeMayerTerm((x - target.x) * (x - target.x) + (y - target.y) * (y - target.y));
    ocp->subjectTo(AT_START, x == state.x);
    ocp->subjectTo(AT_START, y == state.y);
    ocp->subjectTo(AT_START, theta == state.orientation);
    //    ocp->subjectTo(AT_END, x == target.x);
    //    ocp->subjectTo(AT_END, y == target.y);
    ocp->subjectTo(-u_lin_M <= u1 <= u_lin_M);
    ocp->subjectTo(-u_ang_M <= u2 <= u_ang_M);
    ocp->subjectTo(-M_PI <= theta <= M_PI);

    oalg = new OptimizationAlgorithm(*ocp);
    oalg->set( KKT_TOLERANCE, 1e-4 );

//    GnuplotWindow window;
//    window.addSubplot(x, "x");
//    window.addSubplot(y, "y");
//    window.addSubplot(theta, "theta");
//    window.addSubplot(u_lin, "CONTROL u lin");
//    window.addSubplot(u_ang, "CONTROL u ang");
//    oalg->set( INTEGRATOR_TOLERANCE, 1e-6 );
//    (*oalg) << window

    int retc = oalg->solve();

    int retu = -1;
    if(retc == 0)
    {
        VariablesGrid uLog;
        retu = oalg->getControls(uLog);
        if (uLog.getNumPoints() > 0)
        {
            DMatrix const & m = uLog.getMatrix(0);
            if (m.getNumCols() > 0 && m.getNumRows() > 1)
            {
                u[0] = m.coeff(0, 0);
                u[1] = m.coeff(1, 0);
            }
        }
    }

    auto acado_eval_time_end   = std::chrono::system_clock::now();
    double secs = std::chrono::duration<double>(
                acado_eval_time_end - acado_eval_time_start).count();

    std::cout << "TIME = " << secs << " s" << std::endl;
    std::cout << "DIFX = " << std::sqrt(std::pow(target.x - state.x, 2.0)
                                        + std::pow(target.y - state.y, 2.0)) << std::endl;
    std::cout << "RETC = " << retc << std::endl;
    std::cout << "RETU = " << retu << std::endl;
    std::cout << "ULIN = " << u[0] << std::endl;
    std::cout << "UANG = " << u[1] << std::endl;
    std::cout << "TUPL = " << t_upper_limit << std::endl;

    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = u[1];
    twist.linear.x  = u[0];
    twist.linear.y  = 0.0;
    twist.linear.z  = 0.0;
    return retc == 0;
}

