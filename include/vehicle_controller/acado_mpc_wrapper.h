/*******************************************************************************
 * Copyright (c) 2016, Paul Manns
 *
 * All rights reserved.
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 *   TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 *   PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 *   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 *   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 *   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 *   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 *   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#ifndef ACADO_MPC_WRAPPER_H
#define ACADO_MPC_WRAPPER_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>

#include <vehicle_controller/utility.h>

// #define INCLUDE_STEERING_POINT_OFFSET


class AcadoMpcWrapper
{
private:
    double c = 1.0;
    double d = 0.15;

    ACADO::DifferentialState theta, x, y;
#ifdef INCLUDE_STEERING_POINT_OFFSET
    ACADO::DifferentialState xc, yc;
#endif

    ACADO::Control u1, u2;
    ACADO::Parameter T;
    ACADO::DifferentialEquation f;

protected:
    void setupODE();
    Point ctr2Steer(Point in);

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
