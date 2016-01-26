#ifndef ACADO_MPC_WRAPPER_H
#define ACADO_MPC_WRAPPER_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

class AcadoMpcWrapper
{
private:


public:
    enum EXECUTE_RETC
    {
        RET_SUCCESS = 0,
        RET_EXCEEDED_COMPUTE_TIME = 1,
    };

    AcadoMpcWrapper();

    EXECUTE_RETC execute(geometry_msgs::Point const & position,
                         geometry_msgs::Quaternion const & orientation,
                         geometry_msgs::Point const & target_position,
                         geometry_msgs::Quaternion const & target_orientation,
                         geometry_msgs::Twist & twist);
};

#endif // ACADO_MPC_WRAPPER_H
