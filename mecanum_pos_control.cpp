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

#include "mecanum_pos_control.h"

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

int MecanumPosControl::print_status()
{
	int vehicle_thrust_setpoint_sub = orb_subscribe(ORB_ID(vehicle_thrust_setpoint));
	orb_copy(ORB_ID(vehicle_thrust_setpoint), vehicle_thrust_setpoint_sub, &_thrust);

	PX4_INFO("Running");
	// PX4_INFO("thrust %f, %f, %f", (double)_thrust.xyz[0], (double)_thrust.xyz[1], (double)_thrust.xyz[2]);
	PX4_INFO("actuator: %f,%f,%f,%f", (double)_actuators.output[0], (double)_actuators.output[1],
		 (double)_actuators.output[2], (double)_actuators.output[3]);
	// TODO: print additional runtime information about the state of the module

	return 0;
}

int MecanumPosControl::custom_command(int argc, char *argv[])
{
	/*
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	// additional custom commands can be handled like this:
	if (!strcmp(argv[0], "do-something")) {
		get_instance()->do_something();
		return 0;
	}
	 */

	return print_usage("unknown command");
}


int MecanumPosControl::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("mecanum_pos_control",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      1024,
				      (px4_main_t)&run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

MecanumPosControl *MecanumPosControl::instantiate(int argc, char *argv[])
{
	int example_param = 0;
	bool example_flag = false;
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	// parse CLI arguments
	while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'p':
			example_param = (int)strtol(myoptarg, nullptr, 10);
			break;

		case 'f':
			example_flag = true;
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	MecanumPosControl *instance = new MecanumPosControl(example_param, example_flag);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
	}

	return instance;
}

MecanumPosControl::MecanumPosControl(int example_param, bool example_flag)
	: ModuleParams(nullptr)
{
}

int MecanumPosControl::setup_serial(const char *device)
{
	int serial_fd = ::open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (serial_fd < 0) {
		PX4_ERR("Failed to open serial port.");
		return -1;
	}

	struct termios uart_config;

	tcgetattr(serial_fd, &uart_config);

	cfsetspeed(&uart_config, B9600);

	uart_config.c_cflag |= (CLOCAL | CREAD);

	uart_config.c_cflag &= ~CSIZE;

	uart_config.c_cflag |= CS8;

	uart_config.c_cflag &= ~PARENB;

	uart_config.c_cflag &= ~CSTOPB;

	tcsetattr(serial_fd, TCSANOW, &uart_config);

	return serial_fd;
}


void MecanumPosControl::run()
{
	const int8_t STX = 0x78;
	const int8_t ETX = -0x78;  // 0x??
	const int DATA_SIZE = 10;
	const float factor = 1.0 / 4.0;
	const float omega = 0.0;
	const float a = 10.0;
	const float b = 9.0;
	float d = 0.0;
	float target_x, target_y;
	float vx = 0.0;
	float vy = 0.0;
	int16_t pwm[4] = {0};

	// Example: run the loop synchronized to the sensor_combined topic publication
	// int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	int actuator_outputs_sub = orb_subscribe(ORB_ID(actuator_outputs));
	// int vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));

	// init serial code
	const char *device = "/dev/ttyS1";
	_serial_fd = setup_serial(device);

	if (_serial_fd < 0) {
		PX4_ERR("fail to initialize serial setup");
		return;
	}

	px4_pollfd_struct_t fds[1];
	fds[0].fd = actuator_outputs_sub;
	fds[0].events = POLLIN;

	// initialize parameters
	parameters_update(true);

	while (!should_exit()) {



		if (_vehicle_control_mode_sub.updated()) {
			vehicle_control_mode_s vehicle_control_mode{};

			if (_vehicle_control_mode_sub.copy(&vehicle_control_mode)) {
				_manual_driving = vehicle_control_mode.flag_control_manual_enabled;
				_mission_driving = vehicle_control_mode.flag_control_auto_enabled;
			}
		}

		// wait for up to 1000ms for data
		int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

		if (pret == 0) {
			// Timeout: let the loop run anyway, don't do `continue` here

		} else if (pret < 0) {
			// this is undesirable but not much we can do
			PX4_ERR("poll error %d, %d", pret, errno);
			px4_usleep(50000);
			continue;

		} else if (fds[0].revents & POLLIN) {
			// receive actuator output
			orb_copy(ORB_ID(actuator_outputs), actuator_outputs_sub, &_actuators);

			// TODO: implement a transformation logic
			if (_position_setpoint_triplet_sub.updated()) {
				_position_setpoint_triplet_sub.copy(&_position_setpoint_triplet);
			}

			if (_vehicle_global_position_sub.updated()) {
				_vehicle_global_position_sub.copy(&_vehicle_global_position);
			}

			// MapProjection mapProjection(_vehicle_global_position.lat, _vehicle_global_position.lon);
			// mapProjection.project(_position_setpoint_triplet.current.lat, _position_setpoint_triplet.current.lon, target_x,
			// 		      target_y);
			target_x = 8.0;
			target_y = 4.0;


			if (_vehicle_local_position_sub.updated()) {
				_vehicle_local_position_sub.copy(&_vehicle_local_position);

				d = sqrt(pow((double)target_x - (double)_vehicle_local_position.x,
					     2) + pow((double)target_y - (double)_vehicle_local_position.y, 2));

				vx = ((double)target_x - (double)_vehicle_local_position.x) / (double)d * 8000;
				vy = ((double)target_y - (double)_vehicle_local_position.y) / (double)d * 8000;

				PX4_INFO("x: %f y:%f", (double)_vehicle_local_position.x, (double)_vehicle_local_position.y);
				PX4_INFO("d:%f", (double)d);
			}



			PX4_INFO("xt: %f yt:%f", (double)target_x, (double)target_y);
			PX4_INFO("vx: %f vy:%f", (double)vx, (double)vy);



			// float dt = hrt_absolute_time();

			// const double lat_now_rad = math::radians(_vehicle_global_position.lat);
			// const double lon_now_rad = math::radians(_vehicle_global_position.lon);
			// const double lat_next_rad = math::radians(_position_setpoint_triplet.current.lat);
			// const double lon_next_rad = math::radians(_position_setpoint_triplet.current.lon);

			// const double vx = (lon_next_rad - lon_now_rad) / (double)dt;
			// const double vy = (lat_next_rad - lat_now_rad) / (double)dt;

			if (_manual_driving) {
				// PX4_INFO("manual_mode");
				pwm[0] = (((double)_actuators.output[0] - 800) / 1400.0) * 5000.0 - 2500.0;
				pwm[1] = (((double)_actuators.output[1] - 800) / 1400.0) * 5000.0 - 2500.0;
				pwm[2] = (((double)_actuators.output[2] - 800) / 1400.0) * 5000.0 - 2500.0;
				pwm[3] = (((double)_actuators.output[3] - 800) / 1400.0) * 5000.0 - 2500.0;

			} else if (_mission_driving) {
				// PX4_INFO("mission_mode");
				if ((double)d < 0.5) {
					pwm[0] = 0;
					pwm[1] = 0;
					pwm[2] = 0;
					pwm[3] = 0;

				} else {
					pwm[0] = (double)(factor * (1 * vx - 1 * vy - (a + b) * omega)) ;
					pwm[1] = (double)(factor * (1 * vx + 1 * vy + (a + b) * omega)) ;
					pwm[2] = (double)(factor * (1 * vx - 1 * vy + (a + b) * omega)) ;
					pwm[3] = (double)(factor * (1 * vx + 1 * vy - (a + b) * omega)) ;
				}


			}




			PX4_INFO("pwm0:%d pwm1:%d pwm2:%d pwm3:%d", pwm[0], pwm[1], pwm[2], pwm[3]);

			// send to mobieous module



			int8_t data[DATA_SIZE] = {0};
			data[0] = STX;
			memcpy(data + 1, pwm, sizeof(pwm));
			data[9] = ETX;

			for (int i = 0 ; i < 10 ; i++) {
				int sent_len = ::write(_serial_fd, &data[i], 1);

				if (sent_len < 0) {
					PX4_ERR("Failed to write the complete data %d %d.", _serial_fd, sent_len);
				}

				usleep(5000);

			}

			// PX4_ERR("OK : %d %llu", sizeof(data), hrt_absolute_time());
			// usleep(10000);

		}


		parameters_update();
	}

	orb_unsubscribe(actuator_outputs_sub);
}

void MecanumPosControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s update;
		_parameter_update_sub.copy(&update);

		// update parameters from storage
		updateParams();
	}
}

int MecanumPosControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("module", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
	PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int mecanum_pos_control_main(int argc, char *argv[])
{
	return MecanumPosControl::main(argc, argv);
}
