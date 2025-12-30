px4_add_module(
	MODULE modules__articulated_chassis__drivetrain_controller
	MAIN drivetrain_controller
	STACK_MAIN 2048
	SRCS
		drivetrain_controller.cpp
		drivetrain_controller_main.cpp
	DEPENDS
		mathlib
		px4_work_queue
		PID
	MODULE_CONFIG
		module.yaml
)
