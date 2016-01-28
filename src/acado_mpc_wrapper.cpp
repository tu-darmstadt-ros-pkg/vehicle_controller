#include <vehicle_controller/acado_mpc_wrapper.h>
#include <vehicle_controller/quaternions.h>
#include <vehicle_controller/utility.h>

#include <memory>

#include <ros/ros.h>

using geometry_msgs::Quaternion;
using geometry_msgs::Twist;

using namespace ACADO;

AcadoMpcWrapper::AcadoMpcWrapper()
{
    setupODE();
}

void AcadoMpcWrapper::setupODE()
{
    f << dot(theta) == u2;
    f << dot(x)    == c * u1 * cos(theta);
    f << dot(y)    == c * u1 * sin(theta);
#ifdef INCLUDE_STEERING_POINT_OFFSET
    f << dot(xc)   == c * u1 * cos(theta) + d * sin(theta) * u2;
    f << dot(yc)   == c * u1 * sin(theta) + d * cos(theta) * u2;
#endif
}

Point AcadoMpcWrapper::ctr2Steer(Point in)
{
    Point p;
    p.x = in.x + d * cos(in.orientation);
    p.y = in.y + d * sin(in.orientation);
    p.orientation = in.orientation;
    return p;
}

AcadoMpcWrapper::EXECUTE_RETC
    AcadoMpcWrapper::execute(geometry_msgs::Point const & position,
                             Quaternion const & orientation,
                             geometry_msgs::Point const & target_position,
                             Quaternion const & target_orientation,
                             Twist & twist)
{
    double const u_lin_M = 0.3;
    double const u_des   = 0.2;
    double const u_ang_M = 0.3;

    double target_ypr[3];
    quaternion2angles(target_orientation, target_ypr);
    double ypr[3];
    quaternion2angles(orientation, ypr);

    double alpha = ypr[0];
    double beta  = atan2(target_position.y - position.y,
                         target_position.x - position.x);
    double error_2_path   = constrainAngle_mpi_pi( beta - alpha );

    if(error_2_path > M_PI_2)
        error_2_path = error_2_path - M_PI;
    if(error_2_path < -M_PI_2)
        error_2_path = M_PI + error_2_path;

    double x_diff = std::sqrt(std::pow(target_position.x - position.x, 2.0)
                            + std::pow(target_position.y - position.y, 2.0));
    double t_upper_limit = x_diff / u_des /* + std::abs(error_2_path) / u_ang_M */;

    //t_upper_limit = std::min(1.0, t_upper_limit);
//    std::cout << "ELIN = " << x_diff << std::endl;
//    std::cout << "EANG = " << constrainAngle_mpi_pi(target_ypr[0] - ypr[0])
//                              * 180.0 / M_PI << std::endl;
//    std::cout << "TYAW = " << target_ypr[0] * 180.0 / M_PI << std::endl;
//    std::cout << "TUPL = " << t_upper_limit << std::endl;


    double u[2] = {0, 0};

    std::unique_ptr<OCP> ocp(new OCP(0.0, t_upper_limit, 10));
    ocp->subjectTo(f);

#ifdef INCLUDE_STEERING_POINT_OFFSET
    ocp->minimizeMayerTerm(  (xc - target_position.x) * (xc - target_position.x)
                           + (yc - target_position.y) * (yc - target_position.y)
                     + 1.0 * (theta - target_ypr[0]) * (theta - target_ypr[0])
                             );
    Point ctr;
    ctr.x = position.x;
    ctr.y = position.y;
    ctr.orientation = ypr[0];
    Point steer = ctr2Steer(ctr);

    ocp->subjectTo(AT_START, xc == position.x);
    ocp->subjectTo(AT_START, yc == position.y);
    ocp->subjectTo(AT_START, x  == steer.x);
    ocp->subjectTo(AT_START, y  == steer.y);
    ocp->subjectTo(AT_START, theta == ypr[0]);
    ocp->subjectTo(-u_lin_M <= u1 <= u_lin_M);
    ocp->subjectTo(-u_ang_M <= u2 <= u_ang_M);
    ocp->subjectTo(-M_PI <= theta <= M_PI);

#else
    ocp->minimizeMayerTerm((x - target_position.x) * (x - target_position.x)
                         + (y - target_position.y) * (y - target_position.y));
    ocp->subjectTo(AT_START, x == position.x);
    ocp->subjectTo(AT_START, y == position.y);
    ocp->subjectTo(AT_START, theta == ypr[0]);
    ocp->subjectTo(-u_lin_M <= u1 <= u_lin_M);
    ocp->subjectTo(-u_ang_M <= u2 <= u_ang_M);
    ocp->subjectTo(-M_PI <= theta <= M_PI);
#endif

    OptimizationAlgorithm * oalg = new OptimizationAlgorithm(*ocp);
    oalg->set( KKT_TOLERANCE, 1e-5 );

    LogRecord lgr(LOG_AT_EACH_ITERATION);
    lgr << LOG_CONTROLS;
    (*oalg) << lgr;


    int retc = oalg->solve();

//    oalg->getLogRecord ( lgr) ;
//    lgr.print();

    if (retc)
    {
        ROS_WARN("MPC Control computation failed.");
        std::cout << "  TUPL = " << t_upper_limit << std::endl;
        std::cout << "  XDIF = " << x_diff << std::endl;
        std::cout << "  RETC = " << retc << std::endl;
        return RET_ACADO_OCP_FAIL;
    }
    else
    {
        VariablesGrid uLog;
        oalg->getControls(uLog);
        if (uLog.getNumPoints() > 0)
        {
            DMatrix const & m = uLog.getMatrix(0);
            if (m.getNumCols() > 0 && m.getNumRows() > 1)
            {
                u[0] = m.coeff(0, 0);
                u[1] = m.coeff(1, 0);
            }
            else
                return RET_ACADO_DAT_FAIL;
        }
        else
            return RET_ACADO_DAT_FAIL;
    }
    delete oalg;

    twist.linear.x = u[0];
    twist.angular.z = u[1];
    return RET_SUCCESS;
}
