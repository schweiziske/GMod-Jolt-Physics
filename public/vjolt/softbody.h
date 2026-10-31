#pragma once

#include <cstdint>
#include <type_traits>

// Server-owned soft bodies in an existing VPhysics environment. Calls belong on
// the environment owner thread, outside Simulate, except IsValid and Destroy.
// Positions and velocities use Source world coordinates; masses use kilograms.
// Input arrays are copied. Output arrays belong to the caller. No Jolt objects
// or allocator ownership cross this interface.
#define VJOLT_SOFTBODY_BRIDGE_VERSION "VJoltSoftBody001"
inline constexpr uint32_t kVJoltSoftBodyBridgeVersion = 1;
inline constexpr uint32_t kVJoltSoftBodyMaxVertices = 8192;
inline constexpr uint32_t kVJoltSoftBodyMaxTriangles = 16384;
inline constexpr uint32_t kVJoltSoftBodyMaxBodies = 256;
inline constexpr uint32_t kVJoltSoftBodyMaxTotalVertices = 131072;
using VJoltSoftBodyHandle = uint64_t;

struct VJoltSoftBodyVector
{
	float x, y, z;
};

struct VJoltSoftBodyVertex
{
	VJoltSoftBodyVector position{}; // Local to settings.position/settings.angles.
	float mass = 1.0f;
	uint32_t pinned = 0; // Ignores forces. SetVertexVelocity can move a pinned vertex.
};

struct VJoltSoftBodyTriangle
{
	uint32_t vertices[3]; // Zero-based indices; outward winding for a closed mesh.
};

struct VJoltSoftBodyMesh
{
	uint32_t size = sizeof( VJoltSoftBodyMesh );
	uint32_t vertexCount = 0;
	uint32_t triangleCount = 0;
	const VJoltSoftBodyVertex *vertices = nullptr;
	const VJoltSoftBodyTriangle *triangles = nullptr;
};

enum VJoltSoftBodyFlags : uint32_t
{
	VJoltSoftBodyAllowSleeping = 1,
	VJoltSoftBodyDoubleSided = 2,
	VJoltSoftBodyDihedralBend = 4, // Angular bending for shells; default remains distance bending.
};

struct VJoltSoftBodySettings
{
	uint32_t size = sizeof( VJoltSoftBodySettings );
	uint32_t flags = VJoltSoftBodyAllowSleeping | VJoltSoftBodyDoubleSided;
	uint32_t iterations = 5;
	VJoltSoftBodyVector position{};
	VJoltSoftBodyVector angles{};    // Source pitch, yaw, roll in degrees.
	VJoltSoftBodyVector velocity{};  // Initial world velocity of unpinned vertices.
	float edgeCompliance = 0.00001f; // Inverse spring stiffness, seconds^2 / kg.
	float bendCompliance = 0.001f;   // Distance: s^2/kg. DihedralBend: s^2/(kg*m^2), in Jolt metres.
	float pressure = 0.0f;           // Gas energy nRT, kg * Source-unit^2 / second^2, not pascals.
	float gravityFactor = 1.0f;
	float damping = 0.1f;
	float friction = 0.5f;
	float restitution = 0.0f;
	float vertexRadius = 0.5f;
	float maxVelocity = 4000.0f;
};

enum class VJoltSoftBodyError : uint32_t
{
	None,
	InvalidEnvironment,
	SimulationBusy,
	InvalidSettings,
	InvalidMesh,
	InvalidVertex,
	InvalidTriangle,
	InvalidTopology,
	PressureRequiresClosedMesh,
	LimitExceeded,
	AllocationFailed,
};

enum VJoltSoftBodyStateFlags : uint32_t
{
	VJoltSoftBodyActive = 1,
	VJoltSoftBodyClosed = 2,
};

struct VJoltSoftBodyState
{
	uint32_t size = sizeof( VJoltSoftBodyState );
	uint32_t vertexCount = 0;
	uint32_t triangleCount = 0;
	uint32_t pinnedCount = 0;
	uint32_t flags = 0;
	float mass = 0.0f; // Nominal total mass, including pinned vertices.
	float pressure = 0.0f;
	float gravityFactor = 1.0f;
	float volume = 0.0f; // Signed Source-unit^3 for closed meshes, zero for cloth.
	float simulationTime = 0.0f;
	VJoltSoftBodyVector position{}; // Current native origin, which can shift during simulation.
	VJoltSoftBodyVector boundsMin{};
	VJoltSoftBodyVector boundsMax{};
};

struct VJoltSoftBodyVertexState
{
	VJoltSoftBodyVector position; // World space, in the original input vertex order.
	VJoltSoftBodyVector velocity;
	float mass;
	uint32_t pinned;
};

static_assert( sizeof( VJoltSoftBodyVertex ) == 20 );
static_assert( sizeof( VJoltSoftBodyTriangle ) == 12 );
static_assert( sizeof( VJoltSoftBodySettings ) == 84 );
static_assert( sizeof( VJoltSoftBodyState ) == 76 );
static_assert( sizeof( VJoltSoftBodyVertexState ) == 32 );
static_assert( std::is_trivially_copyable_v<VJoltSoftBodyMesh> );
static_assert( std::is_trivially_copyable_v<VJoltSoftBodySettings> );
static_assert( std::is_trivially_copyable_v<VJoltSoftBodyState> );

class IVJoltSoftBodyBridge
{
public:
	virtual uint32_t GetVersion() const = 0;
	// One connected, consistently wound triangle mesh. Creates edge and bend
	// constraints automatically. Pressure requires a closed, outward mesh.
	virtual VJoltSoftBodyHandle Create( int32_t environment, const VJoltSoftBodyMesh &mesh,
		const VJoltSoftBodySettings &settings, VJoltSoftBodyError *error = nullptr ) = 0;
	virtual bool IsValid( VJoltSoftBodyHandle handle ) const = 0;
	// Invalidates immediately; removal during physics callbacks is deferred.
	// Environment destruction and Lua ownership cleanup also release the body.
	virtual bool Destroy( VJoltSoftBodyHandle handle ) = 0;
	virtual bool GetState( VJoltSoftBodyHandle handle, VJoltSoftBodyState &state ) const = 0;
	virtual bool GetVertices(
		VJoltSoftBodyHandle handle, uint32_t first, uint32_t count, VJoltSoftBodyVertexState *vertices ) const = 0;
	virtual bool GetTriangles(
		VJoltSoftBodyHandle handle, uint32_t first, uint32_t count, VJoltSoftBodyTriangle *triangles ) const = 0;
	virtual bool SetPressure( VJoltSoftBodyHandle handle, float pressure ) = 0;
	virtual bool SetGravityFactor( VJoltSoftBodyHandle handle, float factor ) = 0;
	virtual bool SetVertexPinned( VJoltSoftBodyHandle handle, uint32_t vertex, bool pinned ) = 0;
	virtual bool SetVertexVelocity(
		VJoltSoftBodyHandle handle, uint32_t vertex, const VJoltSoftBodyVector &velocity ) = 0;
	// Applies to unpinned vertices. Total impulse is distributed by vertex mass.
	virtual bool SetVelocity( VJoltSoftBodyHandle handle, const VJoltSoftBodyVector &velocity ) = 0;
	virtual bool AddImpulse( VJoltSoftBodyHandle handle, const VJoltSoftBodyVector &impulse ) = 0;
	virtual bool AddVertexImpulse(
		VJoltSoftBodyHandle handle, uint32_t vertex, const VJoltSoftBodyVector &impulse ) = 0;
	virtual bool Wake( VJoltSoftBodyHandle handle ) = 0;
};
