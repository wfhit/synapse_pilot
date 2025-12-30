px4_add_module(
	MODULE drivers__quadrature_encoder
	MAIN quadrature_encoder
	STACK_MAIN 2048
	SRCS
		quadrature_encoder.cpp
	DEPENDS
		px4_work_queue
		px4_platform
		arch_quad_encoder
	MODULE_CONFIG
		module.yaml
)
