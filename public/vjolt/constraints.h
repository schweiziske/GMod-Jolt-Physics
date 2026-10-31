#pragma once

#include <cstdint>
#include <type_traits>

class IPhysicsObject;

// World-fixed mechanical assemblies. Each handle owns the coupling and its
// two guides (hinge/hinge or hinge/slider), including Jolt's drift correction.
// Positions/distances use Source units; axes are world-space directions.
// No Jolt objects cross this ABI. Calls run on the environment owner thread.
#define VJOLT_CONSTRAINTS_BRIDGE_VERSION "VJoltConstraints001"
inline constexpr uint32_t kVJoltConstraintsBridgeVersion = 1;
using VJoltConstraintHandle = uint64_t;

struct VJoltConstraintVector
{
	float x, y, z;
};

enum VJoltConstraintKind : uint32_t
{
	VJoltGear = 1,
	VJoltRackAndPinion = 2,
};

enum VJoltConstraintFlags : uint32_t
{
	VJoltAnchor1AtCOM = 1,
	VJoltAnchor2AtCOM = 2,
};

struct VJoltGearSettings
{
	uint32_t size = sizeof( VJoltGearSettings );
	uint32_t flags = VJoltAnchor1AtCOM | VJoltAnchor2AtCOM;
	VJoltConstraintVector anchor1{};
	VJoltConstraintVector axis1{ 0, 0, 1 };
	VJoltConstraintVector anchor2{};
	VJoltConstraintVector axis2{ 0, 0, 1 };
	// angle1 + ratio * angle2 = 0. Positive: opposite rotation; negative: same.
	// For meshing gears ratio = teeth2 / teeth1. Creation pose is angle zero.
	float ratio = 1;
};

struct VJoltRackAndPinionSettings
{
	uint32_t size = sizeof( VJoltRackAndPinionSettings );
	uint32_t flags = VJoltAnchor1AtCOM | VJoltAnchor2AtCOM;
	VJoltConstraintVector anchor1{}; // Pinion hinge.
	VJoltConstraintVector axis1{ 0, 0, 1 };
	VJoltConstraintVector anchor2{}; // Rack's zero position.
	VJoltConstraintVector axis2{ 1, 0, 0 };
	// Signed Source distance travelled by the rack per positive pinion turn.
	float travelPerRevolution = 128;
	float minPosition = -128;
	float maxPosition = 128;
};

struct VJoltConstraintState
{
	uint32_t size = sizeof( VJoltConstraintState );
	uint32_t kind = 0;
	float angle1 = 0;           // Degrees, wrapped to [-180, 180].
	float angle2 = 0;           // Second gear, zero for rack.
	float position = 0;         // Rack Source distance from its creation pose.
	float angularVelocity1 = 0; // Degrees / second, projected on each axis.
	float angularVelocity2 = 0;
	float linearVelocity = 0; // Rack Source distance / second.
	float motorSpeed = 0;     // First gear / pinion, degrees / second.
	float maxTorque = 0;      // kg * Source-unit^2 / second^2; zero = off.
};

static_assert( sizeof( VJoltGearSettings ) == 60 );
static_assert( sizeof( VJoltRackAndPinionSettings ) == 68 );
static_assert( sizeof( VJoltConstraintState ) == 40 );
static_assert( std::is_trivially_copyable_v<VJoltGearSettings> );
static_assert( std::is_trivially_copyable_v<VJoltRackAndPinionSettings> );
static_assert( std::is_trivially_copyable_v<VJoltConstraintState> );

class IVJoltConstraintsBridge
{
public:
	virtual uint32_t GetVersion() const = 0;
	// Both bodies must be distinct, non-static props in the same environment.
	// Frozen props are accepted; creation does not unfreeze or change collision filters.
	// Creation, motor changes and snapshots require being outside Simulate.
	virtual VJoltConstraintHandle CreateGear(
		IPhysicsObject *firstGear, IPhysicsObject *secondGear, const VJoltGearSettings &settings ) = 0;
	virtual VJoltConstraintHandle CreateRackAndPinion(
		IPhysicsObject *pinion, IPhysicsObject *rack, const VJoltRackAndPinionSettings &settings ) = 0;
	virtual bool IsValid( VJoltConstraintHandle handle ) const = 0;
	// Destruction from a physics callback is deferred to a safe solver boundary.
	// Removing/replacing/transferring either body retires the entire assembly.
	virtual bool Destroy( VJoltConstraintHandle handle ) = 0;
	virtual bool SetMotor( VJoltConstraintHandle handle, float degreesPerSecond, float maxTorque ) = 0;
	virtual bool GetState( VJoltConstraintHandle handle, VJoltConstraintState &state ) const = 0;
};
