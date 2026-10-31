#include "cbase.h"
#include "vjolt_interface.h"
#include "bridge/vjolt_stats_internal.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <vector>

// memdbgon must be the last include.
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------
// Opt-in module statistics. Contact callbacks only touch relaxed atomics while capture is enabled;
// environment and timing state is updated on the server thread after a completed physics step.
//-------------------------------------------------------------------------------------------------

std::atomic<bool> g_VJoltStatsEnabled = false;

namespace
{
	struct VJoltEnvironmentStats_t
	{
		void *pEnvironment = nullptr;
		uint32_t nBodies = 0;
		uint32_t nActiveBodies = 0;
		uint32_t nSourceConstraints = 0;
		uint32_t nSprings = 0;
		uint32_t nControllers = 0;
		uint32_t nLastPhysicsUpdateErrorMask = 0;
		uint64_t nPhysicsUpdateErrors = 0;
	};

	class JoltStatsBridge final : public IVJoltStatsBridge
	{
	public:
		int32_t GetVersion() const override { return kVJoltStatsBridgeVersion; }
		void SetEnabled( bool enabled ) override
		{
			g_VJoltStatsEnabled.store( enabled, std::memory_order_release );
		}
		bool IsEnabled() const override
		{
			return g_VJoltStatsEnabled.load( std::memory_order_acquire );
		}

		void Reset() override
		{
			m_nSimulationSteps = 0;
			m_flTotalStepMs = 0.0;
			m_flLastStepMs = 0.0;
			m_flMaxStepMs = 0.0;
			m_flTotalUpdateMs = 0.0;
			m_flLastUpdateMs = 0.0;
			m_flMaxUpdateMs = 0.0;
			for ( VJoltEnvironmentStats_t &environment : m_Environments )
			{
				environment.nLastPhysicsUpdateErrorMask = 0;
				environment.nPhysicsUpdateErrors = 0;
			}
			for ( std::atomic<uint64_t> &counter : m_EventCounters )
				counter.store( 0, std::memory_order_relaxed );
		}

		bool GetSnapshot( VJoltStatsSnapshot_t &snapshot ) const override
		{
			if ( snapshot.nStructSize != sizeof( VJoltStatsSnapshot_t ) )
				return false;

			VJoltStats_InitSnapshot( snapshot );
			snapshot.bEnabled = IsEnabled() ? 1u : 0u;
			if ( JPH::JobSystem *pJobSystem = JoltPhysicsInterface::GetInstance().GetJobSystem() )
				snapshot.nWorkerThreads = static_cast<uint32_t>( pJobSystem->GetMaxConcurrency() );
			snapshot.nEnvironmentCount = static_cast<uint32_t>( m_Environments.size() );
			snapshot.nSimulationSteps = m_nSimulationSteps;
			snapshot.flLastStepMs = m_flLastStepMs;
			snapshot.flAverageStepMs = m_nSimulationSteps > 0
				? m_flTotalStepMs / double( m_nSimulationSteps ) : 0.0;
			snapshot.flMaxStepMs = m_flMaxStepMs;
			snapshot.flLastUpdateMs = m_flLastUpdateMs;
			snapshot.flAverageUpdateMs = m_nSimulationSteps > 0
				? m_flTotalUpdateMs / double( m_nSimulationSteps ) : 0.0;
			snapshot.flMaxUpdateMs = m_flMaxUpdateMs;

			for ( const VJoltEnvironmentStats_t &environment : m_Environments )
			{
				snapshot.nBodies += environment.nBodies;
				snapshot.nActiveBodies += environment.nActiveBodies;
				snapshot.nSourceConstraints += environment.nSourceConstraints;
				snapshot.nSprings += environment.nSprings;
				snapshot.nControllers += environment.nControllers;
				snapshot.nLastPhysicsUpdateErrorMask |= environment.nLastPhysicsUpdateErrorMask;
				snapshot.nPhysicsUpdateErrors += environment.nPhysicsUpdateErrors;
			}

			snapshot.nContactAdded = LoadEvent( VJoltStatsEvent::ContactAdded );
			snapshot.nContactPersisted = LoadEvent( VJoltStatsEvent::ContactPersisted );
			snapshot.nNativePlayerPropContacts = LoadEvent( VJoltStatsEvent::NativePlayerPropContact );
			snapshot.nSourcePlayerContacts = LoadEvent( VJoltStatsEvent::SourcePlayerContact );
			snapshot.nPhysgunPenetrationClamps = LoadEvent( VJoltStatsEvent::PhysgunPenetrationClamp );
			snapshot.nPhysgunBlockedChainClamps = LoadEvent( VJoltStatsEvent::PhysgunBlockedChainClamp );
			snapshot.nSevereInterpenetrationContacts = LoadEvent( VJoltStatsEvent::SevereInterpenetrationContact );
			return true;
		}

		void RegisterEnvironment( void *pEnvironment )
		{
			if ( FindEnvironment( pEnvironment ) == m_Environments.end() )
				m_Environments.push_back( { .pEnvironment = pEnvironment } );
		}

		void UnregisterEnvironment( void *pEnvironment )
		{
			const auto it = FindEnvironment( pEnvironment );
			if ( it != m_Environments.end() )
				m_Environments.erase( it );
		}

		void RecordEnvironmentStep( void *pEnvironment, double totalStepMs,
			double physicsUpdateMs, uint32_t bodies, uint32_t activeBodies,
			uint32_t sourceConstraints, uint32_t springs, uint32_t controllers )
		{
			if ( !IsEnabled() )
				return;

			auto it = FindEnvironment( pEnvironment );
			if ( it == m_Environments.end() )
			{
				RegisterEnvironment( pEnvironment );
				it = FindEnvironment( pEnvironment );
			}

			it->nBodies = bodies;
			it->nActiveBodies = activeBodies;
			it->nSourceConstraints = sourceConstraints;
			it->nSprings = springs;
			it->nControllers = controllers;

			++m_nSimulationSteps;
			m_flLastStepMs = totalStepMs;
			m_flTotalStepMs += totalStepMs;
			m_flMaxStepMs = Max( m_flMaxStepMs, totalStepMs );
			m_flLastUpdateMs = physicsUpdateMs;
			m_flTotalUpdateMs += physicsUpdateMs;
			m_flMaxUpdateMs = Max( m_flMaxUpdateMs, physicsUpdateMs );
		}

		void RecordPhysicsUpdateResult( void *pEnvironment, uint32_t errorMask )
		{
			if ( !IsEnabled() )
				return;

			auto it = FindEnvironment( pEnvironment );
			if ( it == m_Environments.end() )
			{
				RegisterEnvironment( pEnvironment );
				it = FindEnvironment( pEnvironment );
			}

			it->nLastPhysicsUpdateErrorMask = errorMask;
			if ( errorMask != 0 )
				++it->nPhysicsUpdateErrors;
		}

		void RecordEvent( VJoltStatsEvent event, uint64_t count )
		{
			if ( !IsEnabled() )
				return;
			m_EventCounters[ static_cast<size_t>( event ) ].fetch_add(
				count, std::memory_order_relaxed );
		}

	private:
		using EnvironmentIterator = std::vector<VJoltEnvironmentStats_t>::iterator;
		using ConstEnvironmentIterator = std::vector<VJoltEnvironmentStats_t>::const_iterator;

		EnvironmentIterator FindEnvironment( void *pEnvironment )
		{
			return std::find_if( m_Environments.begin(), m_Environments.end(),
				[ pEnvironment ]( const VJoltEnvironmentStats_t &environment )
				{
					return environment.pEnvironment == pEnvironment;
				} );
		}
		ConstEnvironmentIterator FindEnvironment( void *pEnvironment ) const
		{
			return std::find_if( m_Environments.begin(), m_Environments.end(),
				[ pEnvironment ]( const VJoltEnvironmentStats_t &environment )
				{
					return environment.pEnvironment == pEnvironment;
				} );
		}
		uint64_t LoadEvent( VJoltStatsEvent event ) const
		{
			return m_EventCounters[ static_cast<size_t>( event ) ].load(
				std::memory_order_relaxed );
		}

		static constexpr size_t kEventCount = 7;
		std::array<std::atomic<uint64_t>, kEventCount> m_EventCounters = {};
		std::vector<VJoltEnvironmentStats_t> m_Environments;
		uint64_t m_nSimulationSteps = 0;
		double m_flTotalStepMs = 0.0;
		double m_flLastStepMs = 0.0;
		double m_flMaxStepMs = 0.0;
		double m_flTotalUpdateMs = 0.0;
		double m_flLastUpdateMs = 0.0;
		double m_flMaxUpdateMs = 0.0;
	};

	JoltStatsBridge g_JoltStatsBridge;
}

EXPOSE_SINGLE_INTERFACE_GLOBALVAR( JoltStatsBridge, IVJoltStatsBridge,
	VJOLT_STATS_BRIDGE_INTERFACE_VERSION, g_JoltStatsBridge );

void VJoltStats_RegisterEnvironment( void *pEnvironment )
{
	g_JoltStatsBridge.RegisterEnvironment( pEnvironment );
}

void VJoltStats_UnregisterEnvironment( void *pEnvironment )
{
	g_JoltStatsBridge.UnregisterEnvironment( pEnvironment );
}

void VJoltStats_RecordEnvironmentStep( void *pEnvironment, double totalStepMs,
	double physicsUpdateMs, uint32_t bodies, uint32_t activeBodies,
	uint32_t sourceConstraints, uint32_t springs, uint32_t controllers )
{
	g_JoltStatsBridge.RecordEnvironmentStep( pEnvironment, totalStepMs, physicsUpdateMs,
		bodies, activeBodies, sourceConstraints, springs, controllers );
}

void VJoltStats_RecordPhysicsUpdateResult( void *pEnvironment, uint32_t errorMask )
{
	g_JoltStatsBridge.RecordPhysicsUpdateResult( pEnvironment, errorMask );
}

void VJoltStats_RecordEnabledEvent( VJoltStatsEvent event, uint64_t count )
{
	g_JoltStatsBridge.RecordEvent( event, count );
}
