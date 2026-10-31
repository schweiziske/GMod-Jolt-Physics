#pragma once

#include <Jolt/Physics/Collision/ContactListener.h>

#include <atomic>
#include <cstdint>

class VJoltPhysicsStatsAccumulator final
{
public:
	struct Snapshot
	{
		std::uint64_t impactCount = 0;
		std::uint64_t systemImpactCount = 0;
		std::uint64_t delayedImpactCount = 0;
		std::uint64_t impactCollisionChecks = 0;
	};

	void Record( const JPH::VPhysicsSolverStats &stats )
	{
		m_ImpactCount.fetch_add( stats.mImpactCount, std::memory_order_relaxed );
		m_SystemImpactCount.fetch_add( stats.mSystemImpactCount, std::memory_order_relaxed );
		m_DelayedImpactCount.fetch_add( stats.mDelayedImpactCount, std::memory_order_relaxed );
		m_ImpactCollisionChecks.fetch_add( stats.mImpactCollisionChecks, std::memory_order_relaxed );
	}

	Snapshot Read() const
	{
		return {
			m_ImpactCount.load( std::memory_order_relaxed ),
			m_SystemImpactCount.load( std::memory_order_relaxed ),
			m_DelayedImpactCount.load( std::memory_order_relaxed ),
			m_ImpactCollisionChecks.load( std::memory_order_relaxed )
		};
	}

	void Clear()
	{
		m_ImpactCount.store( 0, std::memory_order_relaxed );
		m_SystemImpactCount.store( 0, std::memory_order_relaxed );
		m_DelayedImpactCount.store( 0, std::memory_order_relaxed );
		m_ImpactCollisionChecks.store( 0, std::memory_order_relaxed );
	}

private:
	std::atomic< std::uint64_t > m_ImpactCount = 0;
	std::atomic< std::uint64_t > m_SystemImpactCount = 0;
	std::atomic< std::uint64_t > m_DelayedImpactCount = 0;
	std::atomic< std::uint64_t > m_ImpactCollisionChecks = 0;
};
