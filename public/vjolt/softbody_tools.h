#pragma once

#include "softbody.h"

// Optional extension; VJoltSoftBody001 and its vtable remain unchanged. Like
// soft-body snapshots, these operations run on the environment owner thread,
// outside Simulate. They are not immutable scene queries for trace workers.
#define VJOLT_SOFTBODY_TOOLS_VERSION "VJoltSoftBodyTools001"
inline constexpr uint32_t kVJoltSoftBodyToolsVersion = 1;

struct VJoltSoftBodyRay
{
	uint32_t size = sizeof( VJoltSoftBodyRay );
	VJoltSoftBodyVector start{}; // Centre of the query box, in Source world units.
	VJoltSoftBodyVector delta{};
	VJoltSoftBodyVector extents{}; // Zero for a line; otherwise box half extents.
	VJoltSoftBodyVector axes[3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
};

enum VJoltSoftBodyHitFlags : uint32_t
{
	VJoltSoftBodyHitSurface = 1,
	VJoltSoftBodyStartSolid = 2,
	VJoltSoftBodyAllSolid = 4,
};

struct VJoltSoftBodyHit
{
	uint32_t size = sizeof( VJoltSoftBodyHit );
	uint32_t flags = 0;
	float fraction = 1;
	float fractionLeftSolid = 0;
	VJoltSoftBodyVector position{}; // Query centre at impact, not a box contact point.
	VJoltSoftBodyVector normal{};
	float planeDistance = 0;
	uint32_t triangle = UINT32_MAX;
	uint32_t vertex = UINT32_MAX; // Closest vertex on the hit face; original input order.
};

struct VJoltSoftBodyPlasticity
{
	uint32_t size = sizeof( VJoltSoftBodyPlasticity );
	float yield = 0.05f; // Elastic edge strain; for dihedral bends, fraction of PI radians.
	float creep = 0; // Rest-length/angle relaxation rate per second; zero disables plasticity.
	float maxStrain = 0.6f; // Maximum rest-length strain, or fraction of PI radians from the original bend.
};

class IVJoltSoftBodyTools
{
public:
	virtual uint32_t GetVersion() const = 0;
	// true means a completed query, including a miss (flags == 0). false means
	// stale handle, invalid input, or a world that is stepping; never a valid miss.
	virtual bool Trace( VJoltSoftBodyHandle body, const VJoltSoftBodyRay &ray, VJoltSoftBodyHit &hit ) const = 0;
	virtual bool SetPlasticity( VJoltSoftBodyHandle body, const VJoltSoftBodyPlasticity &settings ) = 0;
	// Radial falloff, normalized across unpinned vertices, conserves total impulse.
	virtual bool AddImpulseAt( VJoltSoftBodyHandle body, const VJoltSoftBodyVector &position,
		const VJoltSoftBodyVector &impulse, float radius ) = 0;
};

static_assert( sizeof( VJoltSoftBodyRay ) == 76 );
static_assert( sizeof( VJoltSoftBodyHit ) == 52 );
static_assert( sizeof( VJoltSoftBodyPlasticity ) == 16 );
