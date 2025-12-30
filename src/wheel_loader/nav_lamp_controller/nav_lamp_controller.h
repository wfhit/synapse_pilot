#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/nav_lamp_command.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

class NavLampController : public ModuleBase<NavLampController>, public ModuleParams
{
public:
	NavLampController();
	~NavLampController() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static int run_trampoline(int argc, char *argv[]);
	void run();
	int print_status() override;

	// Public methods for manual control
	uint8_t get_current_state() const { return _current_state; }
	void manual_change_state(uint8_t target_state);
	void set_light_state(bool enable);

private:
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};
	perf_counter_t _command_perf{perf_alloc(PC_COUNT, MODULE_NAME ": commands")};

	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _lamp_command_sub{ORB_ID(nav_lamp_command)};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::NLAMP_PULSE_MS>) _param_pulse_width,
		(ParamInt<px4::params::NLAMP_INT_MS>) _param_state_interval,
		(ParamInt<px4::params::NLAMP_ST_INVERT>) _param_state_invert,
		(ParamInt<px4::params::NLAMP_LT_INVERT>) _param_light_invert
	)

	uint8_t _current_state{0};
	static constexpr uint8_t MAX_STATES = 5;

	void parameters_update();
	void process_commands();
	void change_state_lamp();
	void handle_state_change(uint8_t target_state);
	int calculate_steps_to_target(uint8_t target_state);
};
