#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/perf/perf_counter.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_limit_switch.h>
#include <uORB/topics/parameter_update.h>

#include <board_config.h>
#include "limit_sensor_config.h"

using namespace time_literals;

class LimitSensor : public ModuleBase<LimitSensor>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	// Maximum number of instances (should match board configs)
	static constexpr int MAX_INSTANCES = 4;
	static constexpr uint8_t MANAGER_INSTANCE = 255; // Special instance for manager

	enum LimitFunction : uint8_t {
		BUCKET_LOAD = 0,
		BUCKET_DUMP = 1,
		BOOM_UP = 2,
		BOOM_DOWN = 3,
		STEERING_LEFT = 4,
		STEERING_RIGHT = 5,
		FUNCTION_DISABLED = 255
	};

	LimitSensor(uint8_t instance);
	~LimitSensor() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

	// Get function from board config
	LimitFunction get_function() const
	{
		if (_board_config != nullptr) {
			return static_cast<LimitFunction>(_board_config->function);
		}

		return FUNCTION_DISABLED;
	}

private:
	void Run() override;
	void updateParams() override;

	// Static storage for multiple instances
	static LimitSensor *_instances[MAX_INSTANCES];
	static px4::atomic<uint8_t> _num_instances;
	static LimitSensor *_manager_instance; // Special manager instance

	// Instance details
	const uint8_t _instance;
	const limit_sensor_config_t *_board_config{nullptr};

	// GPIO state for each switch
	struct SwitchState {
		uint32_t pin{0};
		bool configured{false};
		bool inverted{false};
		bool current_state{false};
		bool last_state{false};
		uint64_t last_change_time{0};
		uint8_t debounce_count{0};
	};

	SwitchState _switch_1;
	SwitchState _switch_2;

	// Combined sensor state
	struct SensorState {
		bool combined_state{false};
		bool last_combined_state{false};
		bool redundancy_fault{false};
		uint32_t activation_count{0};
		uint64_t last_activation_time{0};
	} _sensor_state;

	// Track last published state for change-based publishing
	bool _last_published_state{false};
	uint64_t _last_publish_time{0};
	static constexpr uint64_t HEARTBEAT_PUBLISH_INTERVAL_US = 1000000; // 1s heartbeat

	// Publications
	orb_advert_t _pub_handle{nullptr};

	// Subscriptions
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	// Helper methods
	bool is_instance_enabled() const;
	int get_instance_function() const;
	bool configure_switches();
	bool init_publication();
	bool configure_switch(SwitchState &switch_state, uint32_t pin, bool inverted);
	bool read_switch_state(SwitchState &switch_state);
	bool debounce_switch(SwitchState &switch_state);
	void update_combined_state();
	void publish_state();
	void stop_all_sensor_instances();
	static bool start_instance(int instance);

	// Get board configuration for this instance
	const limit_sensor_config_t *get_board_config(uint8_t instance);
	const char *get_function_name(LimitFunction func);

	// Debouncing parameters (configurable via parameters)
	uint64_t _debounce_time_us{2000}; // Default 2ms, loaded from LS_DEBOUNCE_US
	static constexpr uint8_t DEBOUNCE_COUNTS = 1; // Need 1 consistent read after debounce period

	// Performance counters
	perf_counter_t _cycle_perf;
	perf_counter_t _sample_perf;
	perf_counter_t _fault_perf;

	// Polling rate (configurable via parameters)
	uint32_t _run_interval_us{5000}; // Default 200Hz, loaded from LS_POLL_RATE

	// Parameters (using ModuleParams framework with YAML-generated parameters)
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::LS_POLL_RATE>) _param_poll_rate,
		(ParamInt<px4::params::LS_DEBOUNCE_US>) _param_debounce_us,
		(ParamBool<px4::params::LS_DIAG_ENABLE>) _param_diag_enable,
		(ParamBool<px4::params::LS_0_ENABLE>) _param_ls0_enable,
		(ParamInt<px4::params::LS_0_FUNCTION>) _param_ls0_function,
		(ParamBool<px4::params::LS_1_ENABLE>) _param_ls1_enable,
		(ParamInt<px4::params::LS_1_FUNCTION>) _param_ls1_function,
		(ParamBool<px4::params::LS_2_ENABLE>) _param_ls2_enable,
		(ParamInt<px4::params::LS_2_FUNCTION>) _param_ls2_function,
		(ParamBool<px4::params::LS_3_ENABLE>) _param_ls3_enable,
		(ParamInt<px4::params::LS_3_FUNCTION>) _param_ls3_function
	)
};
