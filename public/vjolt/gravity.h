#pragma once

#include <cstdint>
#include <type_traits>

class IPhysicsObject;

// Source world coordinates, acceleration in Source units / second squared.
// Owner-thread calls only, outside Simulate. No Jolt object crosses this ABI.
#define VJOLT_GRAVITY_BRIDGE_VERSION "VJoltGravityBridge001"
inline constexpr uint32_t kVJoltGravityBridgeVersion = 1;
struct VJoltGravityVector { float x, y, z; };
using VJoltGravityFieldHandle = uint64_t;
enum : uint32_t { VJoltGravityEnabled = 1, VJoltGravityOverride = 2 };
enum : uint32_t { VJoltGravitySphere = 0, VJoltGravityBox = 1 };
enum : uint32_t { VJoltGravityUniform = 0, VJoltGravityAttract = 1, VJoltGravityRepel = 2 };

struct VJoltGravityObject
{
	uint32_t size = sizeof(VJoltGravityObject);
	uint32_t flags = VJoltGravityEnabled;
	float factor = 1.0f;
	VJoltGravityVector gravity{};
};
struct VJoltGravityField
{
	uint32_t size = sizeof(VJoltGravityField);
	uint32_t shape = VJoltGravitySphere;
	uint32_t mode = VJoltGravityUniform;
	int32_t priority = 0;
	VJoltGravityVector center{};
	VJoltGravityVector halfExtent{256, 256, 256}; // Sphere radius is halfExtent.x.
	VJoltGravityVector gravity{0, 0, -600};
	float strength = 600.0f; // Radial acceleration; zero at the exact center.
};
static_assert(sizeof(VJoltGravityVector) == 12);
static_assert(sizeof(VJoltGravityObject) == 24);
static_assert(sizeof(VJoltGravityField) == 56);
static_assert(std::is_trivially_copyable_v<VJoltGravityField>);

class IVJoltGravityBridge
{
public:
	virtual uint32_t GetVersion() const = 0;
	virtual bool GetWorld(int32_t environment, VJoltGravityVector &out) const = 0;
	virtual bool SetWorld(int32_t environment, const VJoltGravityVector &gravity) = 0;
	virtual bool GetObjectState(IPhysicsObject *, VJoltGravityObject &out) const = 0;
	virtual bool SetObject(IPhysicsObject *, const VJoltGravityObject &) = 0;
	virtual VJoltGravityFieldHandle CreateField(int32_t environment, const VJoltGravityField &) = 0;
	virtual bool UpdateField(VJoltGravityFieldHandle, const VJoltGravityField &) = 0;
	virtual bool GetField(VJoltGravityFieldHandle, VJoltGravityField &out) const = 0;
	virtual bool DestroyField(VJoltGravityFieldHandle) = 0;
	virtual bool Sample(int32_t environment, const VJoltGravityVector &position, VJoltGravityVector &out) const = 0;
	virtual bool GetEffectiveGravity(IPhysicsObject *, VJoltGravityVector &out) const = 0;
	// Clears fields and object overrides/factors; leaves world gravity unchanged.
	virtual bool Reset(int32_t environment) = 0;
	virtual uint64_t GetRevision() const = 0;
	virtual bool GetFieldAt(int32_t environment, uint32_t ordinal, VJoltGravityFieldHandle &handle, VJoltGravityField &out) const = 0;
};
