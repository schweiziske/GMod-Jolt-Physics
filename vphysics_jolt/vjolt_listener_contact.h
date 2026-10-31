
#pragma once

#include "vjolt_collide.h"
#include "vjolt_controller_fluid.h"
#include "vjolt_friction_contact_cache.h"
#include "vjolt_physics_stats.h"
#include "vjolt_should_collide_cache.h"
#include "vjolt_surfaceprops.h"
#include "bridge/vjolt_stats_internal.h"
#include "vphysics/stats.h"
#include "ankerl/unordered_dense.h"

#include <Jolt/Physics/Collision/EstimateCollisionResponse.h>
#include <Jolt/Physics/Body/BodyLockMulti.h>

#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <unordered_map>
#include <unordered_set>

// Uses the global multiplier sampled at the start of the current simulation.
float VJoltScaleContactFriction( float friction );

struct JoltPhysicsContactPair
{
	JoltPhysicsContactPair( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2 )
		: pObject1(pObject1), pObject2(pObject2)
	{
	}

	JoltPhysicsObject *pObject1 = nullptr;
	JoltPhysicsObject *pObject2 = nullptr;
};

enum VPhysicsGameFlags : uint32
{
	FVPHYSICS_DMG_SLICE				= 0x0001,
	FVPHYSICS_CONSTRAINT_STATIC		= 0x0002,
	FVPHYSICS_PLAYER_HELD			= 0x0004,
	FVPHYSICS_PART_OF_RAGDOLL		= 0x0008,
	FVPHYSICS_MULTIOBJECT_ENTITY	= 0x0010,
	FVPHYSICS_HEAVY_OBJECT			= 0x0020,
	FVPHYSICS_PENETRATING			= 0x0040,
	FVPHYSICS_NO_PLAYER_PICKUP		= 0x0080,
	FVPHYSICS_WAS_THROWN			= 0x0100,
	FVPHYSICS_DMG_DISSOLVE			= 0x0200,
	FVPHYSICS_NO_IMPACT_DMG			= 0x0400,
	FVPHYSICS_NO_NPC_IMPACT_DMG		= 0x0800,
	FVPHYSICS_PUSH_PLAYER			= 0x1000,
	FVPHYSICS_NO_SELF_COLLISIONS	= 0x8000,
};

inline bool VJoltPlayerSupportContinueImpactTestEnabled()
{
	static const bool enabled = std::getenv( "VJOLT_TEST_PLAYER_SUPPORT_CONTINUE_IMPACT" ) != nullptr;
	return enabled;
}

inline bool VJoltPlayerSupportStartupTraceEnabled()
{
	static const bool enabled = std::getenv( "VJOLT_TRACE_PLAYER_SUPPORT_STARTUP" ) != nullptr;
	return enabled;
}

inline float VJoltHeavyOnLightMaximumSolverMassRatio()
{
	static const float ratio = []
	{
		constexpr float kDefaultRatio = 10.0f;
		const char *value = std::getenv( "VJOLT_HEAVY_ON_LIGHT_MAX_MASS_RATIO" );
		if ( !value || !*value )
			return kDefaultRatio;

		char *end = nullptr;
		const float parsed = std::strtof( value, &end );
		if ( end == value || *end != '\0' || !std::isfinite( parsed ) || parsed < 1.0f )
			return kDefaultRatio;

		return parsed;
	}();
	return ratio;
}

inline bool VJoltDebugForceNativeContactTuning()
{
	static const bool enabled = []
	{
		const char *value = std::getenv( "VJOLT_DEBUG_FORCE_NATIVE_CONTACT_TUNING" );
		return value && *value && std::atoi( value ) != 0;
	}();
	return enabled;
}

class JoltPhysicsContactListener final : public JPH::ContactListener
{
public:
	JoltPhysicsContactListener( JPH::PhysicsSystem &physicsSystem )
		: m_PhysicsSystem( physicsSystem )
	{
	}

	void ReadStats( physics_stats_t *pOutput ) const
	{
		if ( !pOutput )
			return;

		*pOutput = physics_stats_t{};
		const VJoltPhysicsStatsAccumulator::Snapshot solver = m_PhysicsStats.Read();
		const JoltFrictionContactCache::Statistics contacts = m_FrictionContactCache.ReadStatistics();

		// Release IVP compiles out writes to maxRescueSpeed, maxSpeedGain, impactSysNum
		// and both rescue counters through IVP_IF. Leave those fields at their real zero
		// value instead of substituting unrelated Jolt measurements. Jolt also has no
		// equivalent of IVP's mutual-energy destroyer, range manager or temporarily-
		// unmovable counter, so those diagnostic-only fields remain explicitly zero.
		pOutput->impactCounter = CounterToInt( solver.impactCount );
		pOutput->impactSumSys = CounterToInt( solver.systemImpactCount );
		pOutput->impactDelayedCount = CounterToInt( solver.delayedImpactCount );
		pOutput->impactCollisionChecks = CounterToInt( solver.impactCollisionChecks );
		pOutput->collisionPairsTotal = CounterToInt( contacts.currentContacts );
		pOutput->collisionPairsCreated = CounterToInt( contacts.contactsCreated );
		pOutput->collisionPairsDestroyed = CounterToInt( contacts.contactsDestroyed );
		pOutput->frictionEventsProcessed = CounterToInt( contacts.contactsProcessed );
	}

	void ClearStats()
	{
		m_PhysicsStats.Clear();
		m_FrictionContactCache.ClearStatistics();
	}

	bool IsVPhysicsImpactingContact( const JPH::Body &body1, const JPH::Body &body2,
		const JPH::ContactManifold &manifold, float *pContactGap = nullptr,
		float *pNormalVelocity = nullptr ) const
	{
		if ( pContactGap )
			*pContactGap = FLT_MAX;
		if ( pNormalVelocity )
			*pNormalVelocity = 0.0f;
		if ( manifold.mRelativeContactPointsOn1.empty() || manifold.mRelativeContactPointsOn2.empty() )
			return false;

		const JPH::RVec3 point1 = manifold.GetWorldSpaceContactPointOn1( 0 );
		const JPH::RVec3 point2 = manifold.GetWorldSpaceContactPointOn2( 0 );
		const JPH::Vec3 relativeVelocity = body2.GetPointVelocity( point2 ) - body1.GetPointVelocity( point1 );
		const float separationSpeed = relativeVelocity.Dot( manifold.mWorldSpaceNormal );
		const float gap = -manifold.mPenetrationDepth;
		if ( pContactGap )
			*pContactGap = gap;
		if ( pNormalVelocity )
			*pNormalVelocity = separationSpeed;
		const float deltaTime = m_FrictionContactCache.GetBuildingFrameDeltaTime();
		if ( deltaTime <= 0.0f )
			return false;
		constexpr float kIVPCollisionTolerance = 0.25f - 1.0e-4f;
		return gap + separationSpeed * deltaTime < SourceToJolt::Distance( kIVPCollisionTolerance );
	}

	void ApplySurfaceMaterialSemantics( const JPH::Body &body1, const JPH::Body &body2,
		const JPH::ContactManifold &manifold, const JoltPhysicsObject *pObject1,
		const JoltPhysicsObject *pObject2, JPH::ContactSettings &settings ) const
	{
		if ( manifold.mRelativeContactPointsOn1.empty()
			|| manifold.mRelativeContactPointsOn2.empty() )
			return;

		const int material1 = JoltResolveSourceContactMaterial( body1,
			manifold.mSubShapeID1, manifold.GetWorldSpaceContactPointOn1( 0 ),
			manifold.mWorldSpaceNormal, pObject1->GetMaterialIndex() );
		const int material2 = JoltResolveSourceContactMaterial( body2,
			manifold.mSubShapeID2, manifold.GetWorldSpaceContactPointOn2( 0 ),
			-manifold.mWorldSpaceNormal, pObject2->GetMaterialIndex() );

		const surfacedata_t *pSurface1 =
			JoltPhysicsSurfaceProps::GetInstance().GetSurfaceData( material1 );
		const surfacedata_t *pSurface2 =
			JoltPhysicsSurfaceProps::GetInstance().GetSurfaceData( material2 );
		if ( !pSurface1 || !pSurface2 )
			return;

		settings.mCombinedFriction = VJoltScaleContactFriction( Clamp<float>(
			pSurface1->physics.friction * pSurface2->physics.friction, 0.0f, 1.0f ) );
		const float retainedEnergy = Clamp<float>(
			pSurface1->physics.elasticity * pSurface2->physics.elasticity, 0.0f, 1.0f );
		settings.mCombinedRestitution = std::sqrt( retainedEnergy );
	}

	bool IsNativePlayerPropContact( JoltPhysicsObject *pPlayer,
		JoltPhysicsObject *pOther ) const
	{
		if ( !pOther->IsMoveable() || pOther->IsControlledByGame()
			|| pOther->GetMass() > pPlayer->GetPlayerPushMassLimitForContacts() )
			return false;

		constexpr uint32 kExcludedGameFlags = FVPHYSICS_CONSTRAINT_STATIC
			| FVPHYSICS_PLAYER_HELD | FVPHYSICS_PART_OF_RAGDOLL
			| FVPHYSICS_MULTIOBJECT_ENTITY | FVPHYSICS_HEAVY_OBJECT
			| FVPHYSICS_PUSH_PLAYER;
		if ( pOther->GetGameFlags() & kExcludedGameFlags )
			return false;

		if ( pOther->GetCallbackFlags() & CALLBACK_IS_VEHICLE_WHEEL )
			return false;

		// A constrained prop belongs to a larger mechanism. Switching only its player
		// contact to the native solver changes the effective constraint network, so keep
		// those mechanisms on the existing Source-compatible path. This counter is safe
		// to read from Jolt's worker-thread contact callback; enumerating the global
		// constraint collection here can lock recursively and deadlock.
		return !pOther->HasConstraintAttachments();
	}

	static JoltFrictionContactCache::ContactPointSource GetIVPContactPointSource(
		const JPH::Body &body1, const JPH::Body &body2,
		const JoltPhysicsObject *pObject1, const JoltPhysicsObject *pObject2 )
	{
		// IVP_Mindist::init_mindist always sorts a ball ahead of a polygon, and for two balls
		// sorts the larger client_data pointer into synapse 0. Its long-term friction point is
		// calculated on synapse 0's feature and shared by both objects.
		const bool bSphere1 = body1.GetShape()->GetSubType() == JPH::EShapeSubType::Sphere;
		const bool bSphere2 = body2.GetShape()->GetSubType() == JPH::EShapeSubType::Sphere;
		if ( bSphere1 != bSphere2 )
			return bSphere1 ? JoltFrictionContactCache::ContactPointSource::Body1
				: JoltFrictionContactCache::ContactPointSource::Body2;
		if ( bSphere1 )
		{
			return reinterpret_cast<std::uintptr_t>( pObject1 )
				>= reinterpret_cast<std::uintptr_t>( pObject2 )
				? JoltFrictionContactCache::ContactPointSource::Body1
				: JoltFrictionContactCache::ContactPointSource::Body2;
		}

		// Static IVP objects do not initiate hull events. For a polygon/static pair the movable
		// object is therefore pop0/synapse0 and owns the shared friction point.
		const bool bMoveable1 = pObject1->IsMoveable();
		const bool bMoveable2 = pObject2->IsMoveable();
		if ( bMoveable1 != bMoveable2 )
			return bMoveable1 ? JoltFrictionContactCache::ContactPointSource::Body1
				: JoltFrictionContactCache::ContactPointSource::Body2;

		return JoltFrictionContactCache::ContactPointSource::Body1;
	}

	void ApplyPlayerContactSemantics( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2,
		const JPH::ContactManifold &manifold, JPH::ContactSettings &settings,
		bool bNewContact ) const
	{
		const bool bPlayer1 = !!( pObject1->GetCallbackFlags() & CALLBACK_IS_PLAYER_CONTROLLER );
		const bool bPlayer2 = !!( pObject2->GetCallbackFlags() & CALLBACK_IS_PLAYER_CONTROLLER );
		if ( bPlayer1 == bPlayer2 )
			return;

		const JPH::Vec3 playerToOtherNormal = bPlayer1
			? manifold.mWorldSpaceNormal : -manifold.mWorldSpaceNormal;
		const bool bPlayerSupportContact = playerToOtherNormal.GetZ() <= -0.7f;
		const bool bNewTouchingPlayerSupport = bPlayerSupportContact && bNewContact
			&& !settings.mVPhysicsCCDContact
			&& std::fabs( manifold.mPenetrationDepth ) <= SourceToJolt::Distance( 0.0005f );
		settings.mVPhysicsPlayerSupportStartupContact =
			settings.mVPhysicsPlayerSupportStartupContact || bNewTouchingPlayerSupport;
		const bool bDeferPlayerSupport = settings.mVPhysicsDeferPlayerSupportContact
			|| bNewTouchingPlayerSupport;
		settings.mVPhysicsDeferPlayerSupportContact = bDeferPlayerSupport;
		if ( bDeferPlayerSupport )
		{
			// Keep touch callbacks and the manifold cache alive, but let the first exact-touch
			// PSI pass without a physical response as IVP does. The contact becomes ordinary
			// support on the next update.
			settings.mIsSensor = true;
			settings.mVPhysicsImpactingContact = false;
			settings.mVPhysicsDisablePenetrationRecovery = true;
		}
		else if ( settings.mVPhysicsPlayerSupportStartupContact
			&& manifold.mPenetrationDepth > 0.0f
			&& VJoltPlayerSupportContinueImpactTestEnabled() )
		{
			// Diagnostic A/B: IVP keeps revisiting the startup mindist while its delayed
			// impact is still pending, even after the effective velocity is separating.
			settings.mVPhysicsImpactingContact = true;
		}

		JoltPhysicsObject *pOther = bPlayer1 ? pObject2 : pObject1;
		// Ordinary lateral prop pushing uses the native finite-mass response.
		// Head impacts keep Source's event-time impact semantics; established loads
		// are still ordinary support, not another impact on every physics step.
		const bool bPlayerOverheadContact = playerToOtherNormal.GetZ() >= 0.7f;
		const bool bNativePlayerProp = !bPlayerOverheadContact
			&& IsNativePlayerPropContact( bPlayer1 ? pObject1 : pObject2, pOther );
		if ( bPlayerSupportContact )
		{
			// A fresh Source support impact separates the physical collision push
			// (which can generate friction) from the normal-only penetration rescue.
			// Treating both as one native normal lambda spends rescue as friction
			// and can consume the player's entire, correctly scaled speed budget.
			// Established support still uses the ordinary iterative contact solve.
			settings.mVPhysicsUseLegacySolverForImpactOnly = true;
		}
		if ( playerToOtherNormal.GetZ() > -0.7f )
		{
			VJoltStats_RecordEvent( bNativePlayerProp
				? VJoltStatsEvent::NativePlayerPropContact
				: VJoltStatsEvent::SourcePlayerContact );
		}

		// Ordinary, unconstrained lateral props use Jolt's native mass/inertia response.
		// Other side contacts keep Source's player-impact semantics. Floor contacts
		// use the Source startup impact above, followed by native iterative support.
		if ( playerToOtherNormal.GetZ() > -0.7f && !bNativePlayerProp )
		{
			if ( bPlayerOverheadContact )
			{
				// The incoming prop needs IVP's event-time impact, but an established
				// head contact is an ordinary load-bearing contact. Keeping the complete
				// island on the one-pass legacy schedule lets gravity consume a little
				// separation every update until the prop jitters through the player.
				settings.mVPhysicsUseLegacySolverForImpactOnly = true;
			}
			else
			{
				settings.mVPhysicsUseLegacySolver = true;
			}
		}

		if ( bNativePlayerProp )
		{
			settings.mCombinedRestitution = 0.0f;
			return;
		}

		if ( pOther->IsMoveable() )
			return;

		// Source game movement owns world / immovable geometry. The VPhysics player shadow
		// blocks velocity against a wall, but IVP does not beam an already-overlapping player
		// out of it. Keep flat-ground support intact and suppress only the lateral/ceiling
		// depenetration that Jolt would otherwise add on top of the game collision hull.
		const float flRealPenetrationEpsilon = SourceToJolt::Distance( 1.0e-4f );
		if ( playerToOtherNormal.GetZ() > -0.99f
			&& manifold.mPenetrationDepth > flRealPenetrationEpsilon )
		{
			settings.mVPhysicsImpactingContact = false;
			settings.mVPhysicsDisablePenetrationRecovery = true;
			settings.mCombinedRestitution = 0.0f;
		}
	}

	void ApplySevereInterpenetrationSemantics( JoltPhysicsObject *pObject1,
		JoltPhysicsObject *pObject2, const JPH::ContactManifold &manifold,
		float normalVelocity, JPH::ContactSettings &settings ) const
	{
		// Ordinary rigid-body overlap belongs to Jolt's native contact solver and
		// penetration recovery. Keep the IVP anomaly path only for player contacts,
		// where Source game movement and callback semantics require compatibility.
		if ( !( ( pObject1->GetCallbackFlags() | pObject2->GetCallbackFlags() )
			& CALLBACK_IS_PLAYER_CONTROLLER ) )
			return;

		// IVP has an anomaly callback for a failed mindist. The depth / travel test
		// below is an adapter heuristic, not that callback or an IVP depth threshold.
		// This is classified per manifold; it does not deduplicate a compound body pair.
		// Restrict this compatibility path to two
		// movable bodies: player-vs-world overlap intentionally remains owned by
		// Source game movement and must not teleport the player out of a wall.
		constexpr float kSeverePenetrationInches = 1.0f;
		constexpr float kMinimumImpactSpeedInches = 0.5f;
		constexpr float kIVPCollisionToleranceInches = 0.25f - 1.0e-4f;
		const float frameDeltaTime = m_FrictionContactCache.GetBuildingFrameDeltaTime();
		const float closingTravel = frameDeltaTime > 0.0f
			? Max( 0.0f, -normalVelocity ) * frameDeltaTime : 0.0f;
		const bool bPlausibleDiscreteOvershoot =
			normalVelocity < -SourceToJolt::Distance( kMinimumImpactSpeedInches )
			&& manifold.mPenetrationDepth <= closingTravel
				+ SourceToJolt::Distance( kIVPCollisionToleranceInches );
		if ( !pObject1->IsMoveable() || !pObject2->IsMoveable()
			// A newly discovered fast contact can be more than one inch deep simply
			// because its per-step travel stayed below Jolt's CCD threshold. IVP still
			// treats that as an ordinary incoming impact; its anomaly manager is for a
			// failed/interpenetrating mindist, not every discretely overshot contact.
			|| bPlausibleDiscreteOvershoot
			|| manifold.mPenetrationDepth <= SourceToJolt::Distance( kSeverePenetrationInches ) )
			return;

		settings.mVPhysicsSevereInterpenetration = true;
		settings.mVPhysicsUseLegacySolver = true;
		settings.mVPhysicsImpactingContact = true;
		settings.mVPhysicsDisablePenetrationRecovery = true;
		settings.mCombinedRestitution = 0.0f;
		VJoltStats_RecordEvent( VJoltStatsEvent::SevereInterpenetrationContact );
	}

	void ApplyHeavyOnLightContactScaling( const JPH::Body &body1,
		const JPH::Body &body2, const JPH::ContactManifold &manifold,
		JPH::ContactSettings &settings ) const
	{
		if ( !settings.mVPhysicsUseNativeContactTuning
			|| !body1.IsDynamic() || !body2.IsDynamic() )
			return;

		const float invMass1 = body1.GetMotionPropertiesUnchecked()->GetInverseMassUnchecked();
		const float invMass2 = body2.GetMotionPropertiesUnchecked()->GetInverseMassUnchecked();
		if ( invMass1 <= 0.0f || invMass2 <= 0.0f )
			return;

		const bool heavyIsBody1 = invMass1 < invMass2;
		const float heavyInvMass = heavyIsBody1 ? invMass1 : invMass2;
		const float lightInvMass = heavyIsBody1 ? invMass2 : invMass1;
		const float massRatio = lightInvMass / heavyInvMass;
		const float kMaximumSolverMassRatio = VJoltHeavyOnLightMaximumSolverMassRatio();
		if ( massRatio <= kMaximumSolverMassRatio )
			return;

		const JPH::Vec3 gravityDirection = m_PhysicsSystem.GetGravity().NormalizedOr(
			JPH::Vec3( 0.0f, 0.0f, -1.0f ) );
		const JPH::Vec3 heavyToLightNormal = heavyIsBody1
			? manifold.mWorldSpaceNormal : -manifold.mWorldSpaceNormal;
		// Only stabilize a load-bearing stack. Side impacts and a light object resting
		// on a heavy one retain their exact physical mass response.
		if ( heavyToLightNormal.Dot( gravityDirection ) < 0.5f )
			return;

		const float inverseMassScale = massRatio / kMaximumSolverMassRatio;
		if ( heavyIsBody1 )
		{
			settings.mInvMassScale1 = inverseMassScale;
			settings.mInvInertiaScale1 = inverseMassScale;
		}
		else
		{
			settings.mInvMassScale2 = inverseMassScale;
			settings.mInvInertiaScale2 = inverseMassScale;
		}
		settings.mVPhysicsNormalImpulseReportScale = inverseMassScale;
	}

	static uint64 MakePenetrationPairKey( const JPH::BodyID &body1, const JPH::BodyID &body2 )
	{
		const uint32 id1 = body1.GetIndexAndSequenceNumber();
		const uint32 id2 = body2.GetIndexAndSequenceNumber();
		return id1 < id2 ? ( uint64( id1 ) << 32 ) | id2 : ( uint64( id2 ) << 32 ) | id1;
	}

	void ApplyGamePenetrationSemantics( const JPH::Body &body1, const JPH::Body &body2,
		JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2,
		const JPH::ContactManifold &manifold, JPH::ContactSettings &settings )
	{
		// IVP reports an inter-penetrating mindist to IPhysicsCollisionSolver::ShouldSolvePenetration.
		// The game uses it to stop separating a pair stuck for more than 3 seconds and then forces
		// both entities to sleep (game/server/physics.cpp). Without it, props pasted inside each
		// other are pushed apart against their welds forever. Player and shadow-controlled
		// objects keep their existing paths: the game always solves those pairs.
		if ( settings.mIsSensor || pObject1->IsFluid() || pObject2->IsFluid()
			|| pObject1->IsTrigger() || pObject2->IsTrigger()
			|| ( ( pObject1->GetCallbackFlags() | pObject2->GetCallbackFlags() ) & CALLBACK_IS_PLAYER_CONTROLLER )
			|| pObject1->GetShadowController() || pObject2->GetShadowController() )
			return;
		if ( m_pGameSolver.load( std::memory_order_acquire ) == nullptr )
			return;

		const uint64 key = MakePenetrationPairKey( body1.GetID(), body2.GetID() );
		// Written only on the owner thread between Updates.
		const bool bDeclined = m_UnsolvedPenetrationPairs.contains( key );
		// Adapter threshold for a real overlap: one IVP collision tolerance. Ordinary resting
		// contacts are held outside the surface and never reach it. A declined pair stays
		// declined while it overlaps at all; resuming the push just below the query depth
		// would restart the same weld/contact fight and the game's 3 second timer.
		constexpr float kPenetrationQueryInches = 0.25f;
		const float flQueryDepth = bDeclined ? 0.0f : SourceToJolt::Distance( kPenetrationQueryInches );
		if ( manifold.mPenetrationDepth <= flQueryDepth )
			return;
		if ( bDeclined )
		{
			settings.mVPhysicsDisablePenetrationRecovery = true;
			settings.mVPhysicsImpactingContact = false;
		}
		// IVP asks once per pair and PSI, however many manifolds the pair has.
		std::lock_guard lock( m_PenetrationLock );
		if ( m_PenetratingPairKeys.insert( key ).second )
			m_PenetratingPairs.push_back( { body1.GetID(), body2.GetID() } );
	}

	// Owner thread, after Update and its deferred mutations. Decisions apply from the next PSI.
	void DispatchPenetrationQueries( float deltaTime )
	{
		// After the game declines a stuck pair it forces both entities to sleep. IVP then never
		// revisits that mindist until the pair is woken. Jolt wakes a body whose welded or
		// touching neighbours are still active, so keep the declined pair unseparated for as
		// long as it stays penetrating instead of resuming the push on the next PSI.
		std::unordered_set< uint64 > unsolved;
		std::vector< std::pair< JPH::BodyID, JPH::BodyID > > pairs;
		{
			std::lock_guard lock( m_PenetrationLock );
			pairs.swap( m_PenetratingPairs );
			for ( const uint64 key : m_PenetratingPairKeys )
				if ( m_UnsolvedPenetrationPairs.contains( key ) )
					unsolved.insert( key );
			m_PenetratingPairKeys.clear();
		}
		m_UnsolvedPenetrationPairs.swap( unsolved );
		IPhysicsCollisionSolver *pGameSolver = m_pGameSolver.load( std::memory_order_acquire );
		if ( !pGameSolver || pairs.empty() )
			return;
		// Worker discovery order is not deterministic; the game's event list is order-sensitive.
		std::sort( pairs.begin(), pairs.end(), []( const auto &a, const auto &b )
		{
			return MakePenetrationPairKey( a.first, a.second ) < MakePenetrationPairKey( b.first, b.second );
		} );
		const JPH::BodyLockInterfaceNoLock &bodies = m_PhysicsSystem.GetBodyLockInterfaceNoLock();
		for ( const auto &[ id1, id2 ] : pairs )
		{
			const JPH::Body *pBody1 = bodies.TryGetBody( id1 );
			const JPH::Body *pBody2 = bodies.TryGetBody( id2 );
			if ( !pBody1 || !pBody2 )
				continue;
			auto *pObject1 = reinterpret_cast< JoltPhysicsObject * >( pBody1->GetUserData() );
			auto *pObject2 = reinterpret_cast< JoltPhysicsObject * >( pBody2->GetUserData() );
			if ( !pObject1 || !pObject2
				|| ( ( pObject1->GetCallbackFlags() | pObject2->GetCallbackFlags() ) & CALLBACK_MARKED_FOR_DELETE ) )
				continue;
			bool bSolve;
			{
				std::unique_lock lock( m_ShouldCollideLock );
				pGameSolver = m_pGameSolver.load( std::memory_order_acquire );
				if ( !pGameSolver )
					return;
				bSolve = pGameSolver->ShouldSolvePenetration( pObject1, pObject2,
					pObject1->GetGameData(), pObject2->GetGameData(), deltaTime ) != 0;
			}
			if ( !bSolve )
				m_UnsolvedPenetrationPairs.insert( MakePenetrationPairKey( id1, id2 ) );
		}
	}

	// IVP_Core::grow_friction_system: when a core is revived, every mindist already inside the
	// friction range becomes a distance keeper. Only mindists that close in during simulation
	// take the impact path and its rescue speed. Jolt finds those contacts discretely, so a
	// pasted or unfrozen prop resting at zero gap would otherwise get the full rescue push.
	bool IsRevivedContact( const JPH::Body &body1, const JPH::Body &body2,
		JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2,
		const JPH::ContactManifold &manifold ) const
	{
		const uint32 previous = m_nPSIIndex.load( std::memory_order_relaxed ) - 1;
		if ( !( body1.IsDynamic() && pObject1->GetRevivedPSI() == previous )
			&& !( body2.IsDynamic() && pObject2->GetRevivedPSI() == previous ) )
			return false;
		// A body created or woken with real velocity still collides: only resting contacts
		// become distance keepers. IVP_Mindist_Settings::speed_after_keeper_dist is the speed
		// gained falling from the keeper distance (2.3 t) to the collision distance (t).
		constexpr float kIVPCollisionToleranceInches = 0.25f - 1.0e-4f;
		const float flKeeperFallSpeed = std::sqrt( 2.0f * 1.3f
			* SourceToJolt::Distance( kIVPCollisionToleranceInches ) * 9.81f );
		for ( JPH::uint i = 0; i < manifold.mRelativeContactPointsOn1.size(); ++i )
		{
			const JPH::RVec3 point = manifold.GetWorldSpaceContactPointOn1( i );
			const float flClosingSpeed = -( body2.GetPointVelocity( point )
				- body1.GetPointVelocity( point ) ).Dot( manifold.mWorldSpaceNormal );
			if ( flClosingSpeed > flKeeperFallSpeed )
				return false;
		}
		return true;
	}

	JPH::ValidateResult OnContactValidate( const JPH::Body &inBody1, const JPH::Body &inBody2, JPH::Vec3Arg inBaseOffset, const JPH::CollideShapeResult &inCollisionResult ) override
	{
		return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
	}

	void OnContactAdded( const JPH::Body &inBody1, const JPH::Body &inBody2, const JPH::ContactManifold &inManifold, JPH::ContactSettings &ioSettings ) override
	{	
		if ( !inBody1.GetUserData() || !inBody2.GetUserData() )
		{
			return;
		}
		ioSettings.mTrackContactResponse = true;
		VJoltStats_RecordEvent( VJoltStatsEvent::ContactAdded );
		JoltPhysicsObject* pObject1 = reinterpret_cast<JoltPhysicsObject*>( inBody1.GetUserData() );
		JoltPhysicsObject* pObject2 = reinterpret_cast<JoltPhysicsObject*>( inBody2.GetUserData() );
		ioSettings.mVPhysicsUseNativeContactTuning =
			!( ( pObject1->GetCallbackFlags() | pObject2->GetCallbackFlags() ) & CALLBACK_IS_PLAYER_CONTROLLER )
			&& ( VJoltDebugForceNativeContactTuning()
				|| ( !pObject1->HasConstraintAttachments() && !pObject2->HasConstraintAttachments() ) );
		ApplyHeavyOnLightContactScaling( inBody1, inBody2, inManifold, ioSettings );
		ApplySurfaceMaterialSemantics( inBody1, inBody2, inManifold,
			pObject1, pObject2, ioSettings );

		bool bShouldCollide = ShouldCollide( pObject1, pObject2 );
		// If the game says we shouldn't collide, we will treat this as a sensor
		// to satisfy the StartTouch/EndTouch events.
		ioSettings.mIsSensor = !bShouldCollide || ioSettings.mIsSensor;
		float normalVelocity = 0.0f;
		const bool bPlayerContact = ( ( pObject1->GetCallbackFlags() | pObject2->GetCallbackFlags() )
			& CALLBACK_IS_PLAYER_CONTROLLER ) != 0;
		ioSettings.mVPhysicsImpactingContact = bPlayerContact
			&& IsVPhysicsImpactingContact(
				inBody1, inBody2, inManifold, nullptr, &normalVelocity );
		ApplyPlayerContactSemantics( pObject1, pObject2, inManifold, ioSettings, true );
		ApplySevereInterpenetrationSemantics(
			pObject1, pObject2, inManifold, normalVelocity, ioSettings );
		// Constrained Source bodies retain IVP/Havana tangential friction semantics,
		// while Jolt keeps the normal contact and penetration solve. This is generic
		// and does not identify simfphys or any vehicle type.
		ioSettings.mVPhysicsUseLegacyFriction = !bPlayerContact
			&& !ioSettings.mIsSensor
			&& !ioSettings.mVPhysicsUseNativeContactTuning;
		ApplyGamePenetrationSemantics( inBody1, inBody2, pObject1, pObject2, inManifold, ioSettings );
		// Player contacts keep their own startup and impact semantics.
		ioSettings.mVPhysicsRevivedContact = !bPlayerContact && !ioSettings.mIsSensor
			&& IsRevivedContact( inBody1, inBody2, pObject1, pObject2, inManifold );
		if ( VJoltPlayerSupportStartupTraceEnabled()
			&& ioSettings.mVPhysicsPlayerSupportStartupContact )
		{
			std::fprintf( stderr,
				"PLAYER_SUPPORT_CONTACT added b1=%u b2=%u active=%d,%d points=%u depth=%.9g impacting=%d defer=%d sensor=%d\n",
				inBody1.GetID().GetIndexAndSequenceNumber(), inBody2.GetID().GetIndexAndSequenceNumber(),
				int( inBody1.IsActive() ), int( inBody2.IsActive() ),
				unsigned( inManifold.mRelativeContactPointsOn1.size() ), inManifold.mPenetrationDepth,
				int( ioSettings.mVPhysicsImpactingContact ), int( ioSettings.mVPhysicsDeferPlayerSupportContact ),
				int( ioSettings.mIsSensor ) );
		}
		const bool bFrictionSensor = ioSettings.mIsSensor
			|| ioSettings.mVPhysicsPlayerSupportStartupContact
			|| pObject1->IsFluid() || pObject2->IsFluid();
		const JoltFrictionContactCache::ContactPointSource contactPointSource =
			GetIVPContactPointSource( inBody1, inBody2, pObject1, pObject2 );
		m_FrictionContactCache.RecordContact( inBody1.GetID(), inBody2.GetID(),
			inManifold, bFrictionSensor, contactPointSource,
			contactPointSource == JoltFrictionContactCache::ContactPointSource::Body1
				? &inBody1 : &inBody2 );

		if ( !m_pGameListener )
			return;

		if ( pObject1->IsFluid() || pObject2->IsFluid() )
		{
			const uint32 uThreadId = GetThreadId();

			if ( pObject1->IsFluid() && ( pObject2->GetCallbackFlags() & CALLBACK_FLUID_TOUCH ) )
				m_FluidStartTouchEvents.EmplaceBack( uThreadId, pObject1, pObject2 );

			if ( pObject2->IsFluid() && ( pObject1->GetCallbackFlags() & CALLBACK_FLUID_TOUCH ) )
				m_FluidStartTouchEvents.EmplaceBack( uThreadId, pObject2, pObject1 );

			return;
		}

		if ( pObject1->IsTrigger() || pObject2->IsTrigger() )
		{
			const uint32 uThreadId = GetThreadId();

			if ( pObject1->IsTrigger() )
				m_EnterTriggerEvents.EmplaceBack( uThreadId, pObject1, pObject2 );
		
			if ( pObject2->IsTrigger() )
				m_EnterTriggerEvents.EmplaceBack( uThreadId, pObject2, pObject1 );
	
			return;
		}

		// Discovery can be speculative. Collision events belong to the actual
		// normal response below; touch ownership is still detected here.

		if ( ShouldTouchCallback( pObject1, pObject2 ) )
			m_StartTouchEvents.EmplaceBack( GetThreadId(), JoltPhysicsCollisionInfo( pObject1, pObject2, inManifold ) );
	}

	void OnContactPersisted( const JPH::Body &inBody1, const JPH::Body &inBody2, const JPH::ContactManifold &inManifold, JPH::ContactSettings &ioSettings ) override
	{
		if ( !inBody1.GetUserData() || !inBody2.GetUserData() )
		{
			return;
		}
		ioSettings.mTrackContactResponse = true;
		VJoltStats_RecordEvent( VJoltStatsEvent::ContactPersisted );
		JoltPhysicsObject* pObject1 = reinterpret_cast<JoltPhysicsObject*>( inBody1.GetUserData() );
		JoltPhysicsObject* pObject2 = reinterpret_cast<JoltPhysicsObject*>( inBody2.GetUserData() );
		ioSettings.mVPhysicsUseNativeContactTuning =
			!( ( pObject1->GetCallbackFlags() | pObject2->GetCallbackFlags() ) & CALLBACK_IS_PLAYER_CONTROLLER )
			&& ( VJoltDebugForceNativeContactTuning()
				|| ( !pObject1->HasConstraintAttachments() && !pObject2->HasConstraintAttachments() ) );
		ApplyHeavyOnLightContactScaling( inBody1, inBody2, inManifold, ioSettings );
		ApplySurfaceMaterialSemantics( inBody1, inBody2, inManifold,
			pObject1, pObject2, ioSettings );

		bool bShouldCollide = ShouldCollide( pObject1, pObject2 );
		// If the game says we shouldn't collide, we will treat this as a sensor
		// to satisfy the StartTouch/EndTouch events.
		ioSettings.mIsSensor = !bShouldCollide || ioSettings.mIsSensor;
		// Player compatibility contacts retain IVP's inner impact-distance test. Ordinary
		// rigid-body contacts keep Jolt's native contact lifecycle.
		float normalVelocity = 0.0f;
		const bool bPlayerContact = ( ( pObject1->GetCallbackFlags() | pObject2->GetCallbackFlags() )
			& CALLBACK_IS_PLAYER_CONTROLLER ) != 0;
		ioSettings.mVPhysicsImpactingContact = bPlayerContact
			&& IsVPhysicsImpactingContact(
				inBody1, inBody2, inManifold, nullptr, &normalVelocity );
		ApplyPlayerContactSemantics( pObject1, pObject2, inManifold, ioSettings, false );
		ApplySevereInterpenetrationSemantics(
			pObject1, pObject2, inManifold, normalVelocity, ioSettings );
		ioSettings.mVPhysicsUseLegacyFriction = !bPlayerContact
			&& !ioSettings.mIsSensor
			&& !ioSettings.mVPhysicsUseNativeContactTuning;
		ApplyGamePenetrationSemantics( inBody1, inBody2, pObject1, pObject2, inManifold, ioSettings );
		if ( VJoltPlayerSupportStartupTraceEnabled()
			&& ioSettings.mVPhysicsPlayerSupportStartupContact )
		{
			std::fprintf( stderr,
				"PLAYER_SUPPORT_CONTACT persisted b1=%u b2=%u active=%d,%d points=%u depth=%.9g impacting=%d defer=%d sensor=%d penetration=%.9g\n",
				inBody1.GetID().GetIndexAndSequenceNumber(), inBody2.GetID().GetIndexAndSequenceNumber(),
				int( inBody1.IsActive() ), int( inBody2.IsActive() ),
				unsigned( inManifold.mRelativeContactPointsOn1.size() ), inManifold.mPenetrationDepth,
				int( ioSettings.mVPhysicsImpactingContact ), int( ioSettings.mVPhysicsDeferPlayerSupportContact ),
				int( ioSettings.mIsSensor ), ioSettings.mVPhysicsPlayerSupportStartupPenetration );
		}
		const bool bFrictionSensor = ioSettings.mIsSensor
			|| ioSettings.mVPhysicsPlayerSupportStartupContact
			|| pObject1->IsFluid() || pObject2->IsFluid();
		const JoltFrictionContactCache::ContactPointSource contactPointSource =
			GetIVPContactPointSource( inBody1, inBody2, pObject1, pObject2 );
		m_FrictionContactCache.RecordContact( inBody1.GetID(), inBody2.GetID(),
			inManifold, bFrictionSensor, contactPointSource,
			contactPointSource == JoltFrictionContactCache::ContactPointSource::Body1
				? &inBody1 : &inBody2 );

		// Sensors don't generate impulses or friction events.
		if ( bFrictionSensor )
			return;
		if ( inManifold.mPenetrationDepth < 0.0f || inManifold.mRelativeContactPointsOn1.empty() )
			return;
		if ( !m_pGameListener || !ShouldFrictionCallback( pObject1, pObject2 ) )
			return;

		// Reject quiet contacts before doing the iterative response estimate.
		const JPH::Vec3 vWorldContact = inManifold.GetWorldSpaceContactPointOn1( 0 );
		const JPH::Vec3 vRel = inBody1.GetPointVelocity( vWorldContact ) - inBody2.GetPointVelocity( vWorldContact );
		const JPH::Vec3 vTangent = vRel - vRel.Dot( inManifold.mWorldSpaceNormal ) * inManifold.mWorldSpaceNormal;
		const float flTangentSpeed = vTangent.Length();
		constexpr float kMinTangentSpeed = 0.5f;
		if ( flTangentSpeed < kMinTangentSpeed )
			return;

		// Friction callbacks still need a pre-solve energy estimate. Normal pressure is no
		// longer guessed here; OnContactSolved publishes the actual solver lambda.
		JPH::CollisionEstimationResult result;
		JPH::EstimateCollisionResponse( inBody1, inBody2, inManifold, result,
			ioSettings.mCombinedFriction, ioSettings.mCombinedRestitution,
			/*minVelocityForRestitution*/ 1.0f, /*numIterations*/ 4 );

		// Sum friction impulse vectors across the manifold's contact points.
		JPH::Vec3 vFrictionImpulse = JPH::Vec3::sZero();
		for ( const JPH::CollisionEstimationResult::Impulse &imp : result.mImpulses )
			vFrictionImpulse += imp.mFrictionImpulse1 * result.mTangent1
			                  + imp.mFrictionImpulse2 * result.mTangent2;

		const float flFrictionImpulseMag = vFrictionImpulse.Length();
		if ( flFrictionImpulseMag < 1e-4f )
			return;

		// IVP's friction event reports `eliminated_energy / dt / mass` (specific power, W/kg)
		// then converts to HL units with the same in^2/m^2 factor we use here. We match that
		// so the engine's energy thresholds behave as they did under IVP. The friction
		// impulse from EstimateCollisionResponse is per-step; divide by the
		// current physics step duration below rather than assuming a fixed tick rate.
		const float flMass1 = pObject1->IsStatic() ? 0.0f : pObject1->GetMass();
		const float flMass2 = pObject2->IsStatic() ? 0.0f : pObject2->GetMass();
		const float flMass = flMass1 > 0.0f ? flMass1 : flMass2;
		if ( flMass <= 0.0f )
			return;

		const float flStepDt = m_FrictionContactCache.GetBuildingFrameDeltaTime();
		if ( flStepDt <= 0.0f )
			return;
		const float flPowerPerMass = ( flFrictionImpulseMag * flTangentSpeed ) / flStepDt / flMass;
		const float flEnergy = JoltToSource::Energy( flPowerPerMass );

		// Snapshot also queries energy absorbed; keep it on the same published manifold.
		m_FrictionContactCache.AccumulateContactResponse( inBody1.GetID(), inBody2.GetID(),
			inManifold, 0.0f, flEnergy );

		m_FrictionEvents.EmplaceBack( GetThreadId(),
			JoltPhysicsCollisionInfo( pObject1, pObject2, inManifold ),
			flEnergy );
	}

	void OnContactSolved( const JPH::SubShapeIDPair &inSubShapePair, float inNormalImpulse,
		bool inWasImpact ) override
	{
		m_FrictionContactCache.AccumulateSolvedNormalImpulse(
			inSubShapePair, inNormalImpulse, inWasImpact );
	}

	void OnVPhysicsSolverStats( const JPH::VPhysicsSolverStats &inStats ) override
	{
		m_PhysicsStats.Record( inStats );
	}

	void OnContactRemoved( const JPH::SubShapeIDPair &inSubShapePair ) override
	{
		m_FrictionContactCache.RemoveContact( inSubShapePair );

		if ( !m_pGameListener )
			return;

		// This is always called with all bodies locked.
		const JPH::BodyLockInterfaceNoLock &bodyInterface = m_PhysicsSystem.GetBodyLockInterfaceNoLock();

		JPH::Body *pBody1 = bodyInterface.TryGetBody( inSubShapePair.GetBody1ID() );
		JPH::Body *pBody2 = bodyInterface.TryGetBody( inSubShapePair.GetBody2ID() );

		// One of the bodies may have been deleted.
		// TODO(Josh): Handle calling end touch when we delete a body.
		if ( !pBody1 || !pBody2 )
			return;

		JoltPhysicsObject *pObject1 = reinterpret_cast< JoltPhysicsObject * >( pBody1->GetUserData() );
		JoltPhysicsObject *pObject2 = reinterpret_cast< JoltPhysicsObject * >( pBody2->GetUserData() );
		if ( !pObject1 || !pObject2 )
			return;

		if ( pObject1->IsFluid() || pObject2->IsFluid() )
		{
			const uint32 uThreadId = GetThreadId();

			if ( pObject1->IsFluid() && ( pObject2->GetCallbackFlags() & CALLBACK_FLUID_TOUCH ) )
				m_FluidEndTouchEvents.EmplaceBack( uThreadId, pObject1, pObject2 );

			if ( pObject2->IsFluid() && ( pObject1->GetCallbackFlags() & CALLBACK_FLUID_TOUCH ) )
				m_FluidEndTouchEvents.EmplaceBack( uThreadId, pObject2, pObject1 );

			return;
		}

		if ( pObject1->IsTrigger() || pObject2->IsTrigger() )
		{
			const uint32 uThreadId = GetThreadId();

			if ( pObject1->IsTrigger() )
				m_LeaveTriggerEvents.EmplaceBack( uThreadId, pObject1, pObject2 );
		
			if ( pObject2->IsTrigger() )
				m_LeaveTriggerEvents.EmplaceBack( uThreadId, pObject2, pObject1 );
	
			return;
		}

		if ( !ShouldTouchCallback( pObject1, pObject2 ) )
			return;

		const uint32 uThreadId = GetThreadId();

		// Josh:
		// We don't have any collision data here
		// and caching it would be annoying and expensive.
		// 
		// Lucky for us though, the game simply just calls the stuff
		// to retrieve the contact point and normal, then just never uses it
		// so we can return anything we want and it will change *nothing*!
		m_EndTouchEvents.EmplaceBack( uThreadId, JoltPhysicsCollisionInfo( pObject1, pObject2 ) );
	}

	static inline std::uint32_t MakeShouldCollideObjectKey( const JoltPhysicsObject *pObject )
	{
		return pObject->GetBodyID().GetIndexAndSequenceNumber();
	}

	void InvalidateShouldCollideCache( JoltPhysicsObject *pObject )
	{
		if ( pObject )
			m_ShouldCollideCache.Invalidate( MakeShouldCollideObjectKey( pObject ) );
	}

	bool ShouldCollide( JoltPhysicsObject *pObject0, JoltPhysicsObject *pObject1 )
	{
		VJoltAssert( pObject0 != pObject1 );

		if ( !pObject0 || !pObject1 )
			return false;

		if ( ( pObject0->GetCallbackFlags() & CALLBACK_ENABLING_COLLISION ) && ( pObject1->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) )
			return false;

		if ( ( pObject1->GetCallbackFlags() & CALLBACK_ENABLING_COLLISION ) && ( pObject0->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) )
			return false;

		IPhysicsCollisionSolver *pGameSolver = m_pGameSolver.load( std::memory_order_acquire );
		if ( !pGameSolver )
			return true;

		// Josh:
		// Do some work the game does ahead of time to
		// avoid needless locking.
		if ( !PreEmptGameShouldCollide( pObject0, pObject1 ) )
			return false;

		// Cache hits only take a shared cache lock. GMod's solver can call Lua, so misses still enter
		// one serialized callback section.
		const std::uint32_t bodyKey0 = MakeShouldCollideObjectKey( pObject0 );
		const std::uint32_t bodyKey1 = MakeShouldCollideObjectKey( pObject1 );
		VJoltShouldCollideCache::Lookup lookup = m_ShouldCollideCache.Find( bodyKey0, bodyKey1 );
		if ( lookup.found )
			return lookup.shouldCollide;

		// Actually ask the game now, locking both bodies so they cannot have
		// concurrent ShouldCollide calls.
		//JoltPhysicsObjectPairLock lock( pObject0->GetCollisionTestLock(), pObject1->GetCollisionTestLock() );
		std::unique_lock lock( m_ShouldCollideLock );

		// Another worker may have populated the pair while this worker waited for the Lua-safe lock.
		lookup = m_ShouldCollideCache.Find( bodyKey0, bodyKey1 );
		if ( lookup.found )
			return lookup.shouldCollide;

		pGameSolver = m_pGameSolver.load( std::memory_order_acquire );
		if ( !pGameSolver )
			return true;

		const bool bCollide = pGameSolver->ShouldCollide(
			pObject0, pObject1, pObject0->GetGameData(), pObject1->GetGameData() );

		// If RecheckCollisionFilter ran during the game callback, the epoch changed and this result
		// must not be inserted after the invalidation. It is still the answer for this invocation.
		m_ShouldCollideCache.StoreIfUnchanged( lookup, bCollide );
		return bCollide;
	}

	bool PreEmptGameShouldCollide( JoltPhysicsObject *pObject0, JoltPhysicsObject *pObject1 )
	{
		// This function pre-empts the result of the
		// game's ShouldCollide implementation to avoid needless locking.

		// Check if the entities are the same and self-collisions are disabled.
		if ( pObject0->GetGameData() == pObject1->GetGameData() )
		{
			if ( ( pObject0->GetGameFlags() | pObject1->GetGameFlags() ) & FVPHYSICS_NO_SELF_COLLISIONS )
				return false;
		}

		// If both of these are constrained to the world, they shouldn't collide.
		if ( pObject0->GetGameFlags() & pObject1->GetGameFlags() & FVPHYSICS_CONSTRAINT_STATIC )
			return false;

		// We do wheels separately, the IS_VEHICLE_WHEEL is just to have some dummy object to return to the game.
		if ( ( pObject0->GetCallbackFlags() & CALLBACK_IS_VEHICLE_WHEEL ) || ( pObject1->GetCallbackFlags() & CALLBACK_IS_VEHICLE_WHEEL ) )
			return false;

		// Two shadow controlled objects should not collide.
		if ( pObject0->GetShadowController() && pObject1->GetShadowController() )
			return false;

		return true;
	}

	bool ShouldFrictionCallback( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2 )
	{
		if ( !( pObject1->GetCallbackFlags() & CALLBACK_GLOBAL_FRICTION ) )
			return false;

		if ( !( pObject2->GetCallbackFlags() & CALLBACK_GLOBAL_FRICTION ) )
			return false;

		return true;
	}

	bool ShouldTouchCallback( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2 )
	{
		uint32 uFlags = 0;
		uFlags |= pObject1->GetCallbackFlags();
		uFlags |= pObject2->GetCallbackFlags();

		if ( !( uFlags & CALLBACK_GLOBAL_TOUCH ) )
			return false;

		if ( !( uFlags & CALLBACK_GLOBAL_TOUCH_STATIC ) && ( pObject1->IsStatic() || pObject2->IsStatic() ) )
			return false;

		return true;
	}

	IPhysicsCollisionEvent *GetGameListener()
	{
		return m_pGameListener;
	}

	void SetGameListener( IPhysicsCollisionEvent *pListener )
	{
		m_pGameListener = pListener;
	}

	IPhysicsCollisionSolver *GetGameSolver()
	{
		return m_pGameSolver.load( std::memory_order_acquire );
	}

	void SetGameSolver( IPhysicsCollisionSolver *pSolver )
	{
		std::unique_lock< std::mutex > lock( m_ShouldCollideLock );
		m_ShouldCollideCache.Clear();
		m_pGameSolver.store( pSolver, std::memory_order_release );
	}

	void FlushCallbacks()
	{
		if ( !m_pGameListener )
			return;

		// Send StartTouch events
		m_StartTouchEvents.ForEach< true >( [ this ]( JoltPhysicsCollisionData& event )
		{
			m_pGameListener->StartTouch( event.GetPair().pObject1, event.GetPair().pObject2, &event );
		});

		// Send EnterTrigger events
		m_EnterTriggerEvents.ForEach< true >( [ this ]( JoltPhysicsContactPair& event )
		{
			m_pGameListener->ObjectEnterTrigger( event.pObject1, event.pObject2 );
		});

		// Send FluidStartTouch events
		m_FluidStartTouchEvents.ForEach< true >( [ this ]( JoltPhysicsContactPair& event )
		{
			m_pGameListener->FluidStartTouch( event.pObject2, event.pObject1->GetFluidController() );
		});

		// Send Friction events. The game uses these to drive scrape sounds and
		// particle effects on sustained sliding contacts.
		m_FrictionEvents.ForEach< true >( [ this ]( JoltPhysicsFrictionEvent& event )
		{
			JoltPhysicsObject *pObj1 = event.m_Data.GetPair().pObject1;
			JoltPhysicsObject *pObj2 = event.m_Data.GetPair().pObject2;
			const int nMtl1Hit = event.m_Data.GetSurfaceMaterial( 1 );
			const int nMtl2Hit = event.m_Data.GetSurfaceMaterial( 0 );
			m_pGameListener->Friction( pObj1, event.m_flEnergy,
				pObj1->GetMaterialIndex(), nMtl1Hit, &event.m_Data );
			m_pGameListener->Friction( pObj2, event.m_flEnergy,
				pObj2->GetMaterialIndex(), nMtl2Hit, &event.m_Data );
		});

		// Send EndTouch events
		m_EndTouchEvents.ForEach< true >( [ this ]( JoltPhysicsCollisionData& event )
		{
			m_pGameListener->EndTouch( event.GetPair().pObject1, event.GetPair().pObject2, &event );
		});

		// Send LeaveTrigger events
		m_LeaveTriggerEvents.ForEach< true >( [ this ]( JoltPhysicsContactPair& event )
		{
			m_pGameListener->ObjectLeaveTrigger( event.pObject1, event.pObject2 );
		});

		// Send FluidEndTouch events
		m_FluidEndTouchEvents.ForEach< true >( [ this ]( JoltPhysicsContactPair& event )
		{
			m_pGameListener->FluidEndTouch( event.pObject2, event.pObject1->GetFluidController() );
		});

	}

	void PostSimulationFrame()
	{
		if ( m_pGameListener )
			m_pGameListener->PostSimulationFrame();
	}

	const std::atomic< uint32 > &GetPSIClock() const { return m_nPSIIndex; }

	void BeginFrictionContactFrame( float frameDeltaTime )
	{
		m_nPSIIndex.fetch_add( 1, std::memory_order_relaxed );
		m_CollisionEventTime += frameDeltaTime;
		for ( auto it = m_LastImpactTimes.begin(); it != m_LastImpactTimes.end(); )
			if ( m_CollisionEventTime - it->second > 10.0 ) it = m_LastImpactTimes.erase( it );
			else ++it;
		m_FrictionContactCache.BeginFrame( frameDeltaTime );
	}

	bool OnContactResponse( const JPH::Body &inBody1, const JPH::Body &inBody2,
		const JPH::ContactManifold &inManifold, JPH::uint inPoint, const std::function<bool()> &inResponse ) override;
	void SetContactResponseBatches( bool enabled ) { m_BatchContactResponses = enabled; }
	bool WantsContactResponseBatches() const override { return m_BatchContactResponses; }
	bool OnContactResponseBatch( const JPH::Body &body, const std::function<bool()> &batch ) override;
	bool OnContactResponseAtTime( const JPH::Body &inBody1, const JPH::Body &inBody2,
		const JPH::ContactManifold &inManifold, JPH::uint inPoint, JPH::RVec3Arg inCenterOfMass1,
		JPH::RVec3Arg inCenterOfMass2, const std::function<bool()> &inResponse ) override;

	void ResetCollisionClock() { m_LastImpactTimes.clear(); m_CollisionEventTime = 0.0; }

	void PublishFrictionContactFrame()
	{
		m_FrictionContactCache.PublishFrame( [ this ]( const JPH::BodyID &body1ID, const JPH::BodyID &body2ID )
		{
			JPH::BodyID bodyIDs[ 2 ] = { body1ID, body2ID };
			JPH::BodyLockMultiRead lock( m_PhysicsSystem.GetBodyLockInterface(), bodyIDs, 2 );
			const JPH::Body *pBody1 = lock.GetBody( 0 );
			const JPH::Body *pBody2 = lock.GetBody( 1 );
			if ( !pBody1 || !pBody2 || !pBody1->IsInBroadPhase() || !pBody2->IsInBroadPhase()
				|| pBody1->IsSensor() || pBody2->IsSensor() )
			{
				return false;
			}

			// Jolt removes solver manifolds when an island goes to sleep. IVP keeps its
			// friction synapses, and Source expects sleeping props to retain ground/heavy-
			// object contacts. Preserve only the all-sleeping case; if either non-static
			// body is still active, a missing/removed callback means the contact is gone.
			return ( pBody1->IsStatic() || !pBody1->IsActive() )
				&& ( pBody2->IsStatic() || !pBody2->IsActive() );
		} );
	}

	void CopyFrictionContacts( const JPH::BodyID &bodyID,
		std::vector< JoltFrictionContactCache::Contact > &out ) const
	{
		m_FrictionContactCache.CopyPublishedContacts( bodyID, out );
	}

	bool CopyFirstFrictionContact( const JPH::BodyID &bodyID,
		JoltFrictionContactCache::Contact &out ) const
	{
		return m_FrictionContactCache.CopyFirstPublishedContact( bodyID, out );
	}

	float GetFrictionNormalImpulseTotal( const JPH::BodyID &bodyID ) const
	{
		return m_FrictionContactCache.GetPublishedNormalImpulseTotal( bodyID );
	}

	void RemovePublishedFrictionContacts( const JPH::BodyID &body1ID, const JPH::BodyID &body2ID )
	{
		m_FrictionContactCache.RemovePublishedContacts( body1ID, body2ID );
	}

	void InvalidateFrictionContacts( const JPH::BodyID &bodyID )
	{
		m_FrictionContactCache.InvalidateBody( bodyID );
	}

private:
	static int CounterToInt( std::uint64_t value )
	{
		return value > static_cast< std::uint64_t >( INT_MAX )
			? INT_MAX : static_cast< int >( value );
	}

	static uint32 GetThreadId()
	{
		extern thread_local uint32 s_ThreadId;
		static std::atomic< uint32 > s_ThreadCtr = { 0u };
		if ( s_ThreadId == ~0u )
			s_ThreadId = s_ThreadCtr++;
		return s_ThreadId;
	}

	const JPH::PhysicsSystem &m_PhysicsSystem;

	IPhysicsCollisionEvent	*m_pGameListener = nullptr;
	bool m_BatchContactResponses = false;
	std::atomic< IPhysicsCollisionSolver * > m_pGameSolver = nullptr;

	std::mutex m_ShouldCollideLock;
	VJoltShouldCollideCache m_ShouldCollideCache;
	// Incremented by the owner before each Update; read by activation and contact callbacks.
	std::atomic< uint32 > m_nPSIIndex = 0;
	std::mutex m_PenetrationLock;
	std::vector< std::pair< JPH::BodyID, JPH::BodyID > > m_PenetratingPairs;
	std::unordered_set< uint64 > m_PenetratingPairKeys;
	std::unordered_set< uint64 > m_UnsolvedPenetrationPairs;
	std::unordered_map<uint64, double> m_LastImpactTimes;
	double m_CollisionEventTime = 0.0;

	class JoltPhysicsCollisionInfo
	{
	public:
		JoltPhysicsCollisionInfo( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2 )
			: m_CollisionPair{ pObject1, pObject2 }
			, m_SurfaceMaterial0( pObject1 ? pObject1->GetMaterialIndex() : 0 )
			, m_SurfaceMaterial1( pObject2 ? pObject2->GetMaterialIndex() : 0 )
		{
		}

		JoltPhysicsCollisionInfo( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2, const JPH::ContactManifold &inManifold, JPH::uint point = 0,
			const JPH::RVec3 *pCenterOfMass1 = nullptr, const JPH::RVec3 *pCenterOfMass2 = nullptr )
			: m_CollisionPair{ pObject1, pObject2 }
			// Both APIs use object 0 -> object 1. Only swapping endpoints reverses it.
			, m_SurfaceNormal( Vector( inManifold.mWorldSpaceNormal.GetX(), inManifold.mWorldSpaceNormal.GetY(), inManifold.mWorldSpaceNormal.GetZ() ) )
			, m_ContactPoint( JoltToSource::Distance( 0.5f * ( inManifold.GetWorldSpaceContactPointOn1( point ) + inManifold.GetWorldSpaceContactPointOn2( point ) ) ) )
		{
			const JPH::RVec3 com1 = pObject1->GetBody()->GetCenterOfMassPosition();
			const JPH::RVec3 com2 = pObject2->GetBody()->GetCenterOfMassPosition();
			const JPH::Vec3 offset1 = pCenterOfMass1 ? JPH::Vec3( *pCenterOfMass1 - com1 ) : JPH::Vec3::sZero();
			const JPH::Vec3 offset2 = pCenterOfMass2 ? JPH::Vec3( *pCenterOfMass2 - com2 ) : JPH::Vec3::sZero();
			const JPH::RVec3 worldPoint = 0.5f * ( inManifold.GetWorldSpaceContactPointOn1( point ) + inManifold.GetWorldSpaceContactPointOn2( point ) );
			m_ContactArm0 = JPH::Vec3( worldPoint - com1 ) - offset1;
			m_ContactArm1 = JPH::Vec3( worldPoint - com2 ) - offset2;
			m_SurfaceMaterial0 = JoltResolveSourceContactMaterial( *pObject1->GetBody(),
				inManifold.mSubShapeID1, inManifold.GetWorldSpaceContactPointOn1( point ) - offset1,
				inManifold.mWorldSpaceNormal, pObject1->GetMaterialIndex() );
			m_SurfaceMaterial1 = JoltResolveSourceContactMaterial( *pObject2->GetBody(),
				inManifold.mSubShapeID2, inManifold.GetWorldSpaceContactPointOn2( point ) - offset2,
				-inManifold.mWorldSpaceNormal, pObject2->GetMaterialIndex() );

			RefreshContactSpeed();
		}

		void RefreshContactSpeed()
		{
			// Save the velocity that enters this response, including legal Pre writes.
			// Keep this contact speed for Post; object GetVelocity reads the real result.
			// IVP defines this vector as surface 1 relative to surface 0.
			m_ContactSpeed = JoltToSource::Distance(
				m_CollisionPair.pObject2->GetBody()->GetPointVelocityCOM( m_ContactArm1 )
				- m_CollisionPair.pObject1->GetBody()->GetPointVelocityCOM( m_ContactArm0 ) );
			m_CollisionSpeed = fabsf( m_ContactSpeed.Dot( m_SurfaceNormal ) );
		}

		int GetSurfaceMaterial( int index ) const
		{
			return index == 0 ? m_SurfaceMaterial0 : m_SurfaceMaterial1;
		}

		JoltPhysicsContactPair m_CollisionPair;

		Vector m_SurfaceNormal = vec3_origin;
		Vector m_ContactPoint  = vec3_origin;
		Vector m_ContactSpeed  = vec3_origin;
		JPH::Vec3 m_ContactArm0 = JPH::Vec3::sZero();
		JPH::Vec3 m_ContactArm1 = JPH::Vec3::sZero();
		float m_CollisionSpeed = 0.0f;

		int m_SurfaceMaterial0 = 0;
		int m_SurfaceMaterial1 = 0;
	};

	class JoltPhysicsCollisionData final : public IPhysicsCollisionData
	{
	public:
		JoltPhysicsCollisionData( const JoltPhysicsCollisionInfo &info )
			: m_CollisionData{ info }
		{
		}

		void GetSurfaceNormal( Vector &out ) override
		{
			out = m_CollisionData.m_SurfaceNormal;
		}

		void GetContactPoint( Vector &out ) override
		{
			out = m_CollisionData.m_ContactPoint;
		}

		void GetContactSpeed( Vector &out ) override
		{
			out = m_CollisionData.m_ContactSpeed;
		}

		JoltPhysicsContactPair GetPair() const
		{
			return m_CollisionData.m_CollisionPair;
		}

		float RefreshContactSpeed()
		{
			m_CollisionData.RefreshContactSpeed();
			return m_CollisionData.m_CollisionSpeed;
		}

		int GetSurfaceMaterial( int index ) const
		{
			return m_CollisionData.GetSurfaceMaterial( index );
		}
	private:
		JoltPhysicsCollisionInfo m_CollisionData;
	};

	class JoltPhysicsCollisionEvent
	{
	public:
		JoltPhysicsCollisionEvent( const JoltPhysicsCollisionInfo &info )
			: m_Data{ info }
		{
			JoltPhysicsObject *pObject1 = m_Data.GetPair().pObject1;
			JoltPhysicsObject* pObject2 = m_Data.GetPair().pObject2;

			m_Event.pObjects[0]			= pObject1;
			m_Event.pObjects[1]			= pObject2;
			m_Event.surfaceProps[0]		= info.GetSurfaceMaterial( 0 );
			m_Event.surfaceProps[1]		= info.GetSurfaceMaterial( 1 );
			m_Event.isCollision			= IsCollision( pObject1, pObject2 );
			m_Event.isShadowCollision	= IsShadowCollision( pObject1, pObject2 );
			m_Event.deltaCollisionTime	= 100.0f;
			m_Event.collisionSpeed		= info.m_CollisionSpeed;
			m_Event.pInternalData		= &m_Data;
		}

		JoltPhysicsCollisionEvent( const JoltPhysicsCollisionEvent &other )
			: m_Event( other.m_Event )
			, m_Data ( other.m_Data )
		{
			// Re-target the event's internal data pointer to our own structure.
			m_Event.pInternalData = &m_Data;
		}

		JoltPhysicsCollisionEvent( JoltPhysicsCollisionEvent &&other )
			: m_Event( std::move( other.m_Event ) )
			, m_Data ( std::move( other.m_Data ) )
		{
			// Re-target the event's internal data pointer to our own structure.
			m_Event.pInternalData = &m_Data;
		}

		static bool IsCollision( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2 )
		{
			bool bIsCollision = ( pObject1->GetCallbackFlags() & pObject2->GetCallbackFlags() ) & CALLBACK_GLOBAL_COLLISION;

			if ( pObject1->IsStatic() && !( pObject2->GetCallbackFlags() & CALLBACK_GLOBAL_COLLIDE_STATIC ) )
				bIsCollision = false;
			if ( pObject2->IsStatic() && !( pObject1->GetCallbackFlags() & CALLBACK_GLOBAL_COLLIDE_STATIC ) )
				bIsCollision = false;

			return bIsCollision;
		}

		static bool IsShadowCollision( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2 )
		{
			return ( pObject1->GetCallbackFlags() ^ pObject2->GetCallbackFlags() ) & CALLBACK_SHADOW_COLLISION;
		}

		vcollisionevent_t			m_Event = {};
		JoltPhysicsCollisionData	m_Data;
	};

	class JoltPhysicsFrictionEvent
	{
	public:
		JoltPhysicsFrictionEvent( const JoltPhysicsCollisionInfo &info, float flEnergy )
			: m_Data( info )
			, m_flEnergy( flEnergy )
		{
		}

		JoltPhysicsCollisionData	m_Data;
		float						m_flEnergy = 0.0f;
	};

	template < typename Data >
	struct JoltPhysicsEventTracker
	{
	public:
		template < typename... T >
		void EmplaceBack( uint32 uThreadId, T&&... val)
		{
			if ( uThreadId >= kMaxThreads )
			{
				// Thread IDs are process-lifetime IDs, not bounded worker-pool slots. A
				// 64-worker pool plus the calling thread can therefore exceed the 64-bit
				// fast mask. Keep the common path lock-free and route rare overflow IDs
				// through one protected queue instead of shifting or indexing out of range.
				std::lock_guard< std::mutex > lock( m_OverflowMutex );
				m_OverflowEvents.emplace_back( std::forward< T >( val )... );
				return;
			}

			m_Events[ uThreadId ].emplace_back( std::forward< T >( val )... );
			m_Mask.fetch_or( 1ull << uThreadId, std::memory_order_release );
		}

		template < bool bClear, typename FuncType >
		void ForEach( FuncType func )
		{
			const uint64_t activeThreads = bClear
				? m_Mask.exchange( 0ull, std::memory_order_acq_rel )
				: m_Mask.load( std::memory_order_acquire );
			for ( uint64_t threads = activeThreads; threads; threads &= threads - 1 )
			{
				const uint32 lowBits = uint32( threads );
				const uint32 i = lowBits != 0
					? JPH::CountTrailingZeros( lowBits )
					: 32u + JPH::CountTrailingZeros( uint32( threads >> 32 ) );
				for ( auto &event : m_Events[ i ] )
					func( event );

				if constexpr ( bClear )
					m_Events[ i ].clear();
			}

			if constexpr ( bClear )
			{
				std::deque< Data > overflowEvents;
				{
					std::lock_guard< std::mutex > overflowLock( m_OverflowMutex );
					overflowEvents.swap( m_OverflowEvents );
				}
				for ( auto &event : overflowEvents )
					func( event );
			}
			else
			{
				// deque keeps element references stable when another overflow thread appends.
				// Snapshot pointers so callbacks never execute while holding the queue lock.
				std::vector< Data * > overflowEvents;
				{
					std::lock_guard< std::mutex > overflowLock( m_OverflowMutex );
					overflowEvents.reserve( m_OverflowEvents.size() );
					for ( auto &event : m_OverflowEvents )
						overflowEvents.push_back( &event );
				}
				for ( Data *pEvent : overflowEvents )
					func( *pEvent );
			}
		}

	private:
		static constexpr uint32 kMaxThreads = 64;
		static_assert( kMaxThreads == sizeof( uint64_t ) * 8 );
		std::atomic< uint64_t >	m_Mask = { 0ull };
		std::vector< Data >		m_Events[ kMaxThreads ];
		std::mutex				m_OverflowMutex;
		std::deque< Data >		m_OverflowEvents;
	};

	JoltPhysicsEventTracker< JoltPhysicsFrictionEvent >		m_FrictionEvents;

	JoltPhysicsEventTracker< JoltPhysicsCollisionData >		m_StartTouchEvents;
	JoltPhysicsEventTracker< JoltPhysicsCollisionData >		m_EndTouchEvents;

	JoltPhysicsEventTracker< JoltPhysicsContactPair >		m_EnterTriggerEvents;
	JoltPhysicsEventTracker< JoltPhysicsContactPair >		m_LeaveTriggerEvents;

	// For the fluid events:
	//   Object1 = the fluid
	//   Object2 = the object
	JoltPhysicsEventTracker< JoltPhysicsContactPair >		m_FluidStartTouchEvents;
	JoltPhysicsEventTracker< JoltPhysicsContactPair >		m_FluidEndTouchEvents;

	JoltFrictionContactCache m_FrictionContactCache;
	VJoltPhysicsStatsAccumulator m_PhysicsStats;

};
