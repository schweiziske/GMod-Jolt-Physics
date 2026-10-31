#pragma once

#include <vjolt/stats.h>

#include <atomic>

enum class VJoltStatsEvent : uint32_t
{
	ContactAdded,
	ContactPersisted,
	NativePlayerPropContact,
	SourcePlayerContact,
	PhysgunPenetrationClamp,
	PhysgunBlockedChainClamp,
	SevereInterpenetrationContact,
};

extern std::atomic<bool> g_VJoltStatsEnabled;

inline bool VJoltStats_IsEnabled()
{
	return g_VJoltStatsEnabled.load( std::memory_order_relaxed );
}

void VJoltStats_RegisterEnvironment( void *pEnvironment );
void VJoltStats_UnregisterEnvironment( void *pEnvironment );
void VJoltStats_RecordEnvironmentStep( void *pEnvironment, double totalStepMs,
	double physicsUpdateMs, uint32_t bodies, uint32_t activeBodies,
	uint32_t sourceConstraints, uint32_t springs, uint32_t controllers );
void VJoltStats_RecordPhysicsUpdateResult( void *pEnvironment, uint32_t errorMask );
void VJoltStats_RecordEnabledEvent( VJoltStatsEvent event, uint64_t count );

inline void VJoltStats_RecordEvent( VJoltStatsEvent event, uint64_t count = 1 )
{
	if ( VJoltStats_IsEnabled() )
		VJoltStats_RecordEnabledEvent( event, count );
}
