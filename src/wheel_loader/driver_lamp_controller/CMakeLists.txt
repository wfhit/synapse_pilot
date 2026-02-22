px4_add_module(
	MODULE drivers__wheel_loader__driver_lamp_controller
	MAIN driver_lamp_controller
	COMPILE_FLAGS
	SRCS
		driver_lamp_controller.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		px4_work_queue
)
