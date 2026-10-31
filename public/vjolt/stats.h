//=================================================================================================
//
// Stable, read-only statistics ABI shared by vphysics_jolt and the GMod server Lua module.
// This header deliberately contains no Source or Jolt types.
//
//=================================================================================================

#pragma once

#include <cstdint>
#include <type_traits>

#define VJOLT_STATS_BRIDGE_INTERFACE_VERSION "VJoltStats001"

static constexpr int32_t kVJoltStatsBridgeVersion = 1;

struct VJoltStatsSnapshot_t
{
	uint32_t nStructSize;
	uint32_t bEnabled;
	uint32_t nWorkerThreads;
	uint32_t nEnvironmentCount;

	uint64_t nSimulationSteps;
	double flLastStepMs;
	double flAverageStepMs;
	double flMaxStepMs;
	double flLastUpdateMs;
	double flAverageUpdateMs;
	double flMaxUpdateMs;

	uint32_t nBodies;
	uint32_t nActiveBodies;
	uint32_t nSourceConstraints;
	uint32_t nSprings;
	uint32_t nControllers;
	uint32_t nLastPhysicsUpdateErrorMask;

	uint64_t nContactAdded;
	uint64_t nContactPersisted;
	uint64_t nNativePlayerPropContacts;
	uint64_t nSourcePlayerContacts;
	uint64_t nPhysgunPenetrationClamps; // Retired clamp; keep the zero-valued slot for VJoltStats001 ABI.
	uint64_t nPhysgunBlockedChainClamps;
	uint64_t nSevereInterpenetrationContacts;
	uint64_t nPhysicsUpdateErrors;
};

static_assert( sizeof( VJoltStatsSnapshot_t ) == 160, "VJoltStatsSnapshot_t changed size" );
static_assert( std::is_standard_layout< VJoltStatsSnapshot_t >::value,
	"VJoltStatsSnapshot_t must be standard layout" );
static_assert( std::is_trivially_copyable< VJoltStatsSnapshot_t >::value,
	"VJoltStatsSnapshot_t must be trivially copyable" );

inline void VJoltStats_InitSnapshot( VJoltStatsSnapshot_t &out )
{
	out = VJoltStatsSnapshot_t{};
	out.nStructSize = static_cast<uint32_t>( sizeof( VJoltStatsSnapshot_t ) );
}

class IVJoltStatsBridge
{
public:
	virtual int32_t GetVersion() const = 0;
	virtual void SetEnabled( bool enabled ) = 0;
	virtual bool IsEnabled() const = 0;
	virtual void Reset() = 0;
	virtual bool GetSnapshot( VJoltStatsSnapshot_t &snapshot ) const = 0;

protected:
	~IVJoltStatsBridge() = default;
};
