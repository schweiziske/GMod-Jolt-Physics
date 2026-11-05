#pragma once
#include "softbody.h"

#define VJOLT_SOFTBODY_IMPACT_VERSION "VJoltSoftBodyImpact001"
inline constexpr uint32_t kVJoltSoftBodyImpactVersion = 1;

struct VJoltSoftBodyImpact
{
	uint32_t size = sizeof( VJoltSoftBodyImpact );
	uint32_t contacts = 0;
	uint32_t vertex = 0; // Strongest contact vertex, zero-based.
	float speed = 0; // Normal velocity change, Source units/s; an impact-strength estimate.
	float energy = 0; // Sum of 0.5 * vertex mass * normal delta-v squared, in joules.
	VJoltSoftBodyVector position{};
};

// Owner-thread reads only. Worker callbacks collect native data without invoking
// Lua. Consume returns the strongest contact update since the previous read.
class IVJoltSoftBodyImpact
{
public:
	virtual uint32_t GetVersion() const = 0;
	virtual bool SetImpactTracking( VJoltSoftBodyHandle body, bool enabled ) = 0;
	virtual bool ConsumeImpact( VJoltSoftBodyHandle body, VJoltSoftBodyImpact &impact ) = 0;
};
static_assert( sizeof( VJoltSoftBodyImpact ) == 32 );
