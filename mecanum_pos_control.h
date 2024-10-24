/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <lib/geo/geo.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>

#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_control_mode.h>



using namespace time_literals;

extern "C" __EXPORT int mecanum_pos_control_main(int argc, char *argv[]);


class MecanumPosControl : public ModuleBase<MecanumPosControl>, public ModuleParams
{
public:
	MecanumPosControl(int example_param, bool example_flag);

	virtual ~MecanumPosControl() = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static MecanumPosControl *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	bool _manual_driving = false;
	bool _mission_driving = false;
	void parameters_update(bool force = false);

	int setup_serial(const char *device);

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SYS_AUTOSTART>) _param_sys_autostart,   /**< example parameter */
		(ParamInt<px4::params::SYS_AUTOCONFIG>) _param_sys_autoconfig  /**< another parameter */
	)

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _position_setpoint_triplet_sub{ORB_ID(position_setpoint_triplet)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};

	// uORB::Subscription _vehicle_thrust_setpoint_sub{ORB_ID(vehicle_thrust_setpoint)};

	hrt_abstime _control_position_last_called{0}; 	/**<last call of control_position  */


	int _serial_fd;

	position_setpoint_triplet_s _position_setpoint_triplet{};
	vehicle_global_position_s _vehicle_global_position{};
	vehicle_local_position_s _vehicle_local_position{};


	struct actuator_outputs_s		_actuators;

	// struct vehicle_local_position_s	_lp;
	struct vehicle_thrust_setpoint_s 	_thrust;

};

