#!/bin/bash
# Script to re-initialize all git submodules for SynapsePilot

echo "Removing empty submodule directories..."

# List of all submodules from .gitmodules
SUBMODULES=(
    "Tools/simulation/jmavsim/jMAVSim"
    "Tools/simulation/gazebo-classic/sitl_gazebo-classic"
    "src/drivers/gps/devices"
    "platforms/nuttx/NuttX/nuttx"
    "platforms/nuttx/NuttX/apps"
    "Tools/simulation/flightgear/flightgear_bridge"
    "Tools/simulation/jsbsim/jsbsim_bridge"
    "src/drivers/cyphal/libcanard"
    "src/drivers/cyphal/public_regulated_data_types"
    "src/drivers/cyphal/legacy_data_types"
    "src/lib/crypto/monocypher"
    "src/lib/events/libevents"
    "src/lib/crypto/libtomcrypt"
    "src/lib/crypto/libtommath"
    "src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client"
    "src/lib/cdrstream/cyclonedds"
    "src/lib/cdrstream/rosidl"
    "src/modules/zenoh/zenoh-pico"
    "src/lib/heatshrink/heatshrink"
    "Tools/simulation/gz"
    "boards/modalai/voxl2/libfc-sensor-api"
    "src/drivers/actuators/vertiq_io/iq-module-communication-cpp"
    "src/drivers/uavcan/libdronecan/dsdl"
    "src/drivers/uavcan/libdronecan/libuavcan/dsdl_compiler/pydronecan"
    "test/fuzztest"
    "src/lib/tensorflow_lite_micro/tflite_micro"
    "src/drivers/ins/microstrain/mip_sdk"
    "src/drivers/ins/sbgecom/sbgECom"
)

for submodule in "${SUBMODULES[@]}"; do
    if [ -d "$submodule" ]; then
        echo "Removing $submodule..."
        rm -rf "$submodule"
    fi
done

echo "Synchronizing submodules from .gitmodules..."
git submodule sync

echo "Initializing and updating all submodules (this may take a while)..."
git submodule update --init --recursive --jobs 4

echo "Done! Verifying submodule status..."
git submodule status | head -10
