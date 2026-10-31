#pragma once
#include <cstdint>
#include <type_traits>

// Private Source vehicle state. No Lua vehicle bridge is exported.
inline constexpr int32_t kVJoltVehicleMaxAxles = 4;
inline constexpr uint32_t kVJoltCarMaxWheels = kVJoltVehicleMaxAxles * 2;
inline constexpr uint32_t kVJoltCarWheelDetached = 1;

struct VJoltCarAxleTuning
{
    float wheelInertia = 4.5f; // effective wheel + axle rotational inertia, kg m^2
    float longitudinalPeak = 1.2f, longitudinalSlide = .9f;
    float lateralPeak = 1.2f, lateralSlide = .85f;
    float longitudinalSlip = .08f, lateralSlip = 6; // peak slip ratio / degrees
    float brakeTorque = 2500, handbrakeTorque = 0; // N m per wheel
};
struct VJoltCarTuning
{
    uint32_t size = sizeof(VJoltCarTuning), axleCount = 0;
    VJoltCarAxleTuning axles[kVJoltVehicleMaxAxles];
};
static_assert(sizeof(VJoltCarAxleTuning) == 36 && sizeof(VJoltCarTuning) == 152);
static_assert(std::is_standard_layout_v<VJoltCarTuning> && std::is_trivially_copyable_v<VJoltCarTuning>);

struct VJoltCarWheelConfig
{
    float radius = 12, width = 6; // Source units, including a deflated tire
    float inertia = 4.5f;         // kg m^2
    float grip = 1;               // multiplies tuned curves and per-axle grip
    uint32_t flags = 0;           // detached wheels have no contact or drive
};
struct VJoltCarWheels
{
    uint32_t size = sizeof(VJoltCarWheels), wheelCount = 0;
    VJoltCarWheelConfig wheels[kVJoltCarMaxWheels];
};
static_assert(sizeof(VJoltCarWheelConfig) == 20 && sizeof(VJoltCarWheels) == 168);
static_assert(std::is_standard_layout_v<VJoltCarWheels> && std::is_trivially_copyable_v<VJoltCarWheels>);
