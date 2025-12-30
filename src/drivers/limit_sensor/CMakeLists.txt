px4_add_module(
    MODULE drivers__limit_sensor
    MAIN limit_sensor
    COMPILE_FLAGS
    SRCS
        limit_sensor.cpp
    MODULE_CONFIG
        module.yaml
    DEPENDS
        perf
        px4_work_queue
)
