#pragma once
#include "softbody.h"

// Optional owner-thread extension. The parent remains alive until the caller
// commits all fragments, allowing an allocation failure to roll back safely.
#define VJOLT_SOFTBODY_FRAGMENT_VERSION "VJoltSoftBodyFragment001"
inline constexpr uint32_t kVJoltSoftBodyFragmentVersion = 1;

class IVJoltSoftBodyFragment
{
public:
	virtual uint32_t GetVersion() const = 0;
	// Source indices are unique, zero-based parent vertices. Triangles must be
	// existing parent faces remapped into this array. Fraction (0,1] divides a
	// shared seam vertex's nominal mass between fragments. Copies current motion,
	// pins, rest lengths and plasticity; open fragments have no gas pressure.
	virtual VJoltSoftBodyHandle CreateFragment( VJoltSoftBodyHandle parent,
		uint32_t vertexCount, const uint32_t *sourceIndices, const float *massFractions,
		uint32_t triangleCount, const VJoltSoftBodyTriangle *triangles, VJoltSoftBodyError *error ) = 0;
};
