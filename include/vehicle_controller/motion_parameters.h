/*******************************************************************************
 * Copyright (c) 2016, Stefan Kohlbrecher, Paul Manns
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

#ifndef MOTION_PARAMETERS_H
#define MOTION_PARAMETERS_H

#include <string>

#include <vehicle_controller/ps3d_motion_parameters.h>

/*
 * Contains motion parameters for the wheeled and tracked robots controlled by
 * the vehicle_controller.
 */
class MotionParameters : public PS3dMotionParameters
{
  public:
    bool USE_FINAL_TWIST_;
    int FINAL_TWIST_TRIALS_MAX_;

    bool y_symmetry;
    double flipper_low_position;
    double flipper_switch_position;
    double flipper_high_position;
    std::string flipper_name;
    std::string pd_params;

    double carrot_distance;
    double min_speed;
    double commanded_speed;
    double inclination_speed_reduction_factor;
    double inclination_speed_reduction_time_constant;
    double current_inclination;
    double max_controller_speed_;
    double max_unlimited_speed_;
    double max_controller_angular_rate_;
    double max_unlimited_angular_rate_;

    inline virtual bool isYSymmetric()
    {
        return y_symmetry;
    }
};

#endif
