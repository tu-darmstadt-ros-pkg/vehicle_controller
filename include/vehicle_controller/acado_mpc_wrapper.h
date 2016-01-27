#ifndef ACADO_MPC_WRAPPER_H
#define ACADO_MPC_WRAPPER_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>

class AcadoMpcWrapper
{
private:
    double c = 1.0;
    double d = 0.00;

    ACADO::DifferentialState theta, x, y;
    ACADO::Control u1, u2;
    ACADO::Parameter T;
    ACADO::DifferentialEquation f;

protected:
    void setupODE();

public:
    enum EXECUTE_RETC
    {
        RET_SUCCESS = 0,
        RET_EXCEEDED_COMPUTE_TIME = 1,
        RET_ACADO_OCP_FAIL = 2,
        RET_ACADO_DAT_FAIL = 3,
    };

    AcadoMpcWrapper();

    EXECUTE_RETC execute(geometry_msgs::Point const & position,
                         geometry_msgs::Quaternion const & orientation,
                         geometry_msgs::Point const & target_position,
                         geometry_msgs::Quaternion const & target_orientation,
                         geometry_msgs::Twist & twist);
};

#endif // ACADO_MPC_WRAPPER_H
