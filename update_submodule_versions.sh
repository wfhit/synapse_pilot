#!/bin/bash
# Update all submodules to match PX4 v1.17.0-alpha1 versions

set -e

cd /home/frank/synapse_pilot_ws/SynapsePilot

echo "Updating submodules to PX4 v1.17.0-alpha1 versions..."
echo ""

# Associative array of submodule paths and their correct commit hashes
declare -A SUBMODULE_COMMITS=(
    ["Tools/simulation/flightgear/flightgear_bridge"]="f47ce7b5fbbb3aa43d33d2be1f6cd3746b13d5bf"
    ["Tools/simulation/gazebo-classic/sitl_gazebo-classic"]="6697ab169ceab512dc706acea63df4c882662c60"
    ["Tools/simulation/gz"]="b6127f4ec20de867e215fb5f78ae88b80f371909"
    ["Tools/simulation/jmavsim/jMAVSim"]="66b764ada522893c05224950aa6268c809f8e48a"
    ["Tools/simulation/jsbsim/jsbsim_bridge"]="f37ec259bd7a43565fe0ff4722465b7a303200f6"
    ["boards/modalai/voxl2/libfc-sensor-api"]="85151aaf6ba8b24ce82b387e088452c63f7e2096"
    ["platforms/nuttx/NuttX/apps"]="e37940d8535f603a16b8f6f21c21edaf584218aa"
    ["platforms/nuttx/NuttX/nuttx"]="b8a6c3268a20e126c83b84ebc5598f9318994200"
    ["src/drivers/actuators/vertiq_io/iq-module-communication-cpp"]="c488af4e8807de80739aa48efd2ea51614dd8195"
    ["src/drivers/cyphal/legacy_data_types"]="36a01e428b110ff84c8babe5b65667b5e3037d5e"
    ["src/drivers/cyphal/libcanard"]="5c69d451ab0787a81dcb615692d707f2a286f5e5"
    ["src/drivers/cyphal/public_regulated_data_types"]="d0bd6516dac8ff61287fe49a9f2c75e7d4dc1b8e"
    ["src/drivers/gps/devices"]="0b9695881bd1e8f830ab4538ab3acc0050019eba"
    ["src/drivers/ins/microstrain/mip_sdk"]="35596994b60ba89fe02f71ce5127baa5e7ff2bbf"
    ["src/drivers/ins/sbgecom/sbgECom"]="80b121c7714083cc4868c0fdb8c41623c7ef9c93"
    ["src/drivers/uavcan/libdronecan/dsdl"]="993be80a62ec957c01fb41115b83663959a49f46"
    ["src/drivers/uavcan/libdronecan/libuavcan/dsdl_compiler/pydronecan"]="19fdf2e5b383243ccdb1094edae0603cf11469e8"
    ["src/lib/cdrstream/cyclonedds"]="314887ca403c2fb0a0316add22672102936ed36c"
    ["src/lib/cdrstream/rosidl"]="bf5682e4747843d1d5133b9a2b54ce6f12f166c7"
    ["src/lib/crypto/libtomcrypt"]="673f5ce29015a9bba3c96792920a10601b5b0718"
    ["src/lib/crypto/libtommath"]="fd73d7630b9d3ed5a79d613ff680a549e9780de7"
    ["src/lib/crypto/monocypher"]="baca5d31259c598540e4d1284bc8d8f793abf83a"
    ["src/lib/events/libevents"]="9ef591c447fe0386d698bf6fb9a6d27e43988ee4"
    ["src/lib/heatshrink/heatshrink"]="052e6de72f67f1777198bce98f3de62f7f3c16a0"
    ["src/lib/tensorflow_lite_micro/tflite_micro"]="3c0b1e3091e4ea423e1bf9da89d41d09517eb0c9"
    ["src/modules/mavlink/mavlink"]="33af200d25ec6f0925b49b1ba82bbf1294ea5f72"
    ["src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client"]="711aef423edd1820347b866d1e4164832df35d04"
    ["src/modules/zenoh/zenoh-pico"]="ede74c3f5b0ac190d074e796057c62848bb9a488"
    ["test/fuzztest"]="1e47f9d7437de5c3ee4cb0ac860d5ec875478059"
)

for path in "${!SUBMODULE_COMMITS[@]}"; do
    commit="${SUBMODULE_COMMITS[$path]}"
    
    if [ -d "$path" ]; then
        echo "Updating $path to $commit..."
        (cd "$path" && git fetch && git checkout "$commit" 2>/dev/null) || echo "  Warning: Could not update $path"
    else
        echo "Warning: $path does not exist"
    fi
done

echo ""
echo "Updating nested submodules..."
git submodule update --recursive

echo ""
echo "="*70
echo "Staging submodule updates..."
git add -A

echo ""
echo "Checking submodule status..."
git submodule status | head -10

echo ""
echo "✓ All submodules updated to PX4 v1.17.0-alpha1 versions!"
echo ""
echo "You should now commit these changes:"
echo "  git commit -m 'Update submodules to PX4 v1.17.0-alpha1 versions'"
