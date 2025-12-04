/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
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

#include <drivers/device/device.h>
#include <drivers/drv_input_capture.h>
#include <drivers/drv_mixer.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <uORB/topics/esc_status.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/debug_array.h>	

#include "DShotTelemetry.h"

#include <matrix/math.hpp>

using namespace time_literals;

#if !defined(DIRECT_PWM_OUTPUT_CHANNELS)
#  error "board_config.h needs to define DIRECT_PWM_OUTPUT_CHANNELS"
#endif

/** Dshot PWM frequency, Hz */
static constexpr unsigned int DSHOT150  =  150000u;
static constexpr unsigned int DSHOT300  =  300000u;
static constexpr unsigned int DSHOT600  =  600000u;
static constexpr unsigned int DSHOT1200 = 1200000u;

static constexpr int DSHOT_DISARM_VALUE = 0;
static constexpr int DSHOT_MIN_THROTTLE = 1;
static constexpr int DSHOT_MAX_THROTTLE = 1999;

// quadcopter model to hold motor_allocation matrix and inverse.
// Values are predefined.
struct QuadcopterModel {
    
    matrix::Matrix<float, 4, 4> motor_allocation_;
    
    matrix::Matrix<float, 4, 4> motor_allocation_inv_;

	matrix::Matrix<float, 4, 4> inertia_matrix_; 

    QuadcopterModel() = default;

    void set_motor_allocation()
    {
        const float m[4][4] = {
            {  1.000000f,  1.000000f,  1.000000f,  1.000000f }, // thrust
            { -0.707107f,  0.707107f,  0.707107f, -0.707107f }, // roll
            {  0.707107f, -0.707107f,  0.707107f, -0.707107f }, // pitch
            {  1.000000f,  1.000000f, -1.000000f, -1.000000f }  // yaw
        };

        motor_allocation_ = matrix::Matrix<float, 4, 4>(m);
        compute_inverse();
		const float j[4][4] = {
			{1,0,0,0},
			{0,1,0,0},
			{0,0,1,0},
			{0,0,0,1}
		}
		inertia_matrix_ = matrix::Matrix<float, 4,4>(j);
    }

    void compute_inverse()
    {
		motor_allocation_inv_ = matrix::inv(motor_allocation_);
    }
};

class DShot : public cdev::CDev, public ModuleBase<DShot>, public OutputModuleInterface
{
public:
	DShot();
	virtual ~DShot();

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	virtual int init();

	virtual int ioctl(file *filp, int cmd, unsigned long arg);

	void mixerChanged() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void retrieve_and_print_esc_info_thread_safe(const int motor_index);

	/**
	 * Send a dshot command to one or all motors
	 * This is expected to be called from another thread.
	 * @param num_repetitions number of times to repeat, set at least to 1
	 * @param motor_index index or -1 for all
	 * @return 0 on success, <0 error otherwise
	 */
	int send_command_thread_safe(const dshot_command_t command, const int num_repetitions, const int motor_index);

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	bool telemetry_enabled() const { return _telemetry != nullptr; }

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

private:

	/** Disallow copy construction and move assignment. */
	DShot(const DShot &) = delete;
	DShot operator=(const DShot &) = delete;

	enum class DShotConfig {
		Disabled  = 0,
		DShot150  = 150,
		DShot300  = 300,
		DShot600  = 600,
		DShot1200 = 1200,
	};

	struct Command {
		dshot_command_t command{};
		int num_repetitions{0};
		uint8_t motor_mask{0xff};
		bool save{false};

		bool valid() const { return num_repetitions > 0; }
		void clear() { num_repetitions = 0; }
	};

	struct Telemetry {
		DShotTelemetry handler{};
		uORB::PublicationMultiData<esc_status_s> esc_status_pub{ORB_ID(esc_status)};
		int last_motor_index{-1};
	};

	void enable_dshot_outputs(const bool enabled);

	void init_telemetry(const char *device);

	void handle_new_telemetry_data(const int motor_index, const DShotTelemetry::EscData &data);

	int request_esc_info();

	void Run() override;

	void update_params();

	void update_telemetry_num_motors();

	void handle_vehicle_commands();

	MixingOutput _mixing_output {PARAM_PREFIX, DIRECT_PWM_OUTPUT_CHANNELS, *this, MixingOutput::SchedulingPolicy::Auto, false, false};

	Telemetry *_telemetry{nullptr};

	static char _telemetry_device[20];
	static px4::atomic_bool _request_telemetry_init;

	px4::atomic<Command *> _new_command{nullptr};

	px4::atomic<DShotTelemetry::OutputBuffer *> _request_esc_info{nullptr};

	bool _outputs_initialized{false};
	bool _outputs_on{false};
	bool _waiting_for_esc_info{false};

	static constexpr unsigned _num_outputs{DIRECT_PWM_OUTPUT_CHANNELS};
	uint32_t _output_mask{0};

	int _class_instance{-1};

	perf_counter_t	_cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	Command _current_command{};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Publication<vehicle_command_ack_s> _command_ack_pub{ORB_ID(vehicle_command_ack)};

	// changes for direct motor control from the NMPC
	QuadcopterModel quadcopter;
	uORB::Subscription _desired_value_sub{ORB_ID(debug_array)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehilcle_angular_velocity)};
	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};

	// motor thrust 
    matrix::Vector<float, 4> last_motor_thrusts_{0.0f, 0.0f, 0.0f, 0.0f};
    matrix::Vector<float, 4> current_motor_thrusts_{0.0f, 0.0f, 0.0f, 0.0f};

	// motor thrust command computed by allocation inverse (T)
	matrix::Vector<float, 4> T_{0.0f, 0.0f, 0.0f, 0.0f};

    // angular rates and derivative
    matrix::Vector3f last_omega_{0.0f, 0.0f, 0.0f};
    matrix::Vector3f last_omega_dot_{0.0f, 0.0f, 0.0f};
    hrt_abstime last_omega_time_{0};

    static constexpr float ang_acc_filter_alpha_ = 0.1f;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::DSHOT_CONFIG>)   _param_dshot_config,
		(ParamFloat<px4::params::DSHOT_MIN>)    _param_dshot_min,
		(ParamBool<px4::params::DSHOT_3D_ENABLE>) _param_dshot_3d_enable,
		(ParamInt<px4::params::DSHOT_3D_DEAD_H>) _param_dshot_3d_dead_h,
		(ParamInt<px4::params::DSHOT_3D_DEAD_L>) _param_dshot_3d_dead_l,
		(ParamInt<px4::params::MOT_POLE_COUNT>) _param_mot_pole_count
	)
};
