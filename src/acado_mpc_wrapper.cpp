#include <vehicle_controller/acado_mpc_wrapper.h>

AcadoMpcWrapper::AcadoMpcWrapper()
{

}

AcadoMpcWrapper::EXECUTE_RETC
    AcadoMpcWrapper::execute(const geometry_msgs::Point &position,
                             geometry_msgs::Quaternion const & orientation,
                             geometry_msgs::Point const & target_position,
                             geometry_msgs::Quaternion const & target_orientation,
                             geometry_msgs::Twist & twist)
{

    return AcadoMpcWrapper::RET_SUCCESS;
}
