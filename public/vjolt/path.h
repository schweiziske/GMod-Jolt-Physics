#pragma once

#include <vjolt/constraints.h>

// Separate extension: VJoltConstraints001 and its userdata remain ABI compatible.
// Owner-thread API; Source distance / seconds / kg. Nodes are world positions at
// creation. A null reference fixes the track to the world; otherwise it follows
// that PhysObj, including its rotation. The follower must be a non-static prop.
#define VJOLT_PATH_BRIDGE_VERSION "VJoltPathBridge001"
inline constexpr uint32_t kVJoltPathBridgeVersion = 1;
inline constexpr uint32_t kVJoltPathMaxNodes = 32;
using VJoltPathHandle = uint64_t;

enum class VJoltPathError : uint32_t
{
    None, WrongThread, Busy, InvalidBody, DifferentEnvironment, UnsupportedBody,
    InvalidSettings, InvalidNodes, DegeneratePath, LimitExceeded, Stale
};
enum class VJoltPathRotation : uint32_t
{
    Free, AroundTangent, AroundNormal, AroundBinormal, FollowPath, Fixed
};
enum class VJoltPathMotorMode : uint32_t { Off, Velocity, Position };
enum VJoltPathFlags : uint32_t { VJoltPathClosed = 1, VJoltPathSnapToPath = 2 };

struct VJoltPathSettings
{
    uint32_t size = sizeof(VJoltPathSettings);
    uint32_t flags = VJoltPathSnapToPath;
    VJoltPathRotation rotation = VJoltPathRotation::FollowPath;
    float smoothing = 0.75f; // [0.05, 1], tangent length relative to adjacent chords.
};
struct VJoltPathMotor
{
    uint32_t size = sizeof(VJoltPathMotor);
    VJoltPathMotorMode mode = VJoltPathMotorMode::Off;
    float speed = 128;       // Signed Source units / second.
    float target = 0;        // [0, 1], fraction of total arc length (not node index).
    float maxForce = 100000; // kg * Source units / second^2; zero disables drive.
    float friction = 0;     // Same units, applied when the motor is off.
    float frequency = 2;    // Position spring Hz, [0.1, 20].
    float damping = 1;      // Damping ratio, [0, 5].
};
struct VJoltPathState
{
    uint32_t size = sizeof(VJoltPathState);
    uint32_t enabled = 0;
    uint32_t closed = 0;
    uint32_t nodeCount = 0;
    float progress = 0;
    float distance = 0;
    float length = 0;
    float velocity = 0; // Relative anchor velocity projected onto the track.
    VJoltPathMotor motor;
};
struct VJoltPathSample
{
    VJoltConstraintVector position{};
    VJoltConstraintVector tangent{};
    VJoltConstraintVector normal{};
};
static_assert(sizeof(VJoltPathSettings) == 16);
static_assert(sizeof(VJoltPathMotor) == 32);
static_assert(sizeof(VJoltPathState) == 64);
static_assert(sizeof(VJoltPathSample) == 36);
static_assert(std::is_trivially_copyable_v<VJoltPathSettings>);
static_assert(std::is_trivially_copyable_v<VJoltPathMotor>);

class IVJoltPathBridge
{
public:
    virtual uint32_t GetVersion() const = 0;
    // Open paths require 2..32 distinct adjacent nodes; loops require >=3.
    // Snap moves only the follower COM to the nearest point, preserving rotation,
    // velocity and frozen state. Without snap the existing attachment offset is
    // retained. Creation never unfreezes bodies or disables their collisions.
    virtual VJoltPathHandle Create(IPhysicsObject *follower, IPhysicsObject *reference,
        const VJoltConstraintVector *nodes, uint32_t count, const VJoltPathSettings &settings,
        const VJoltPathMotor &motor, VJoltPathError *error = nullptr) = 0;
    virtual bool IsValid(VJoltPathHandle handle) const = 0;
    virtual bool Destroy(VJoltPathHandle handle) = 0;
    virtual bool SetMotor(VJoltPathHandle handle, const VJoltPathMotor &motor,
        VJoltPathError *error = nullptr) = 0;
    virtual bool GetState(VJoltPathHandle handle, VJoltPathState &state) const = 0;
    // Uniform arc-length samples including both endpoints, 2..513. World space.
    virtual bool Sample(VJoltPathHandle handle, uint32_t count, VJoltPathSample *samples) const = 0;
};
