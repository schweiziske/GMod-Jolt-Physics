#include "cbase.h"

#include "vjolt_environment.h"
#include <chrono>

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

// RaphaelIT7: This is needed for 64x due to else creating - relocation R_X86_64_TPOFF32 against hidden symbol
thread_local uint32 s_ThreadId = ~0u;

bool JoltPhysicsContactListener::OnContactResponseBatch( const JPH::Body &body, const std::function<bool()> &batch )
{
	auto *object = reinterpret_cast<JoltPhysicsObject *>( body.GetUserData() );
	return object ? object->GetJoltEnvironment()->RunContactBatch( batch ) : batch();
}

bool JoltPhysicsContactListener::OnContactResponse( const JPH::Body &body1, const JPH::Body &body2,
	const JPH::ContactManifold &manifold, JPH::uint point, const std::function<bool()> &response )
{
	return OnContactResponseAtTime( body1, body2, manifold, point,
		body1.GetCenterOfMassPosition(), body2.GetCenterOfMassPosition(), response );
}

bool JoltPhysicsContactListener::OnContactResponseAtTime( const JPH::Body &body1, const JPH::Body &body2,
	const JPH::ContactManifold &manifold, JPH::uint point, JPH::RVec3Arg centerOfMass1,
	JPH::RVec3Arg centerOfMass2, const std::function<bool()> &response )
{
	auto *object1 = reinterpret_cast<JoltPhysicsObject *>( body1.GetUserData() );
	auto *object2 = reinterpret_cast<JoltPhysicsObject *>( body2.GetUserData() );
	if ( !m_pGameListener || !object1 || !object2 || object1->IsTrigger() || object2->IsTrigger()
		|| object1->IsFluid() || object2->IsFluid()
		|| !object1->GetJoltEnvironment()->HasSynchronousContactCallbacks()
		|| !( JoltPhysicsCollisionEvent::IsCollision( object1, object2 )
			|| JoltPhysicsCollisionEvent::IsShadowCollision( object1, object2 ) ) )
		return response();

	return object1->GetJoltEnvironment()->RunContactCallback( object1, object2, centerOfMass1, centerOfMass2, [&]()
	{
		IPhysicsCollisionEvent *listener = m_pGameListener;
		if ( !listener || ( ( object1->GetCallbackFlags() | object2->GetCallbackFlags() ) & CALLBACK_MARKED_FOR_DELETE ) )
			return response();
		JoltPhysicsCollisionEvent event( JoltPhysicsCollisionInfo( object1, object2, manifold, point, &centerOfMass1, &centerOfMass2 ) );
		uint32 id1 = body1.GetID().GetIndexAndSequenceNumber(), id2 = body2.GetID().GetIndexAndSequenceNumber();
		if ( id2 < id1 ) std::swap( id1, id2 );
		const uint64 key = ( uint64( id1 ) << 32 ) | id2;
		const auto last = m_LastImpactTimes.find( key );
		event.m_Event.deltaCollisionTime = last == m_LastImpactTimes.end() ? 1.0f
			: float( std::max( 0.0, m_CollisionEventTime - last->second ) );
		m_LastImpactTimes[key] = m_CollisionEventTime;
		const bool profile = object1->GetJoltEnvironment()->IsProfilingContactCallbacks();
		using Clock = std::chrono::steady_clock;
		const auto preBegin = profile ? Clock::now() : Clock::time_point{};
		listener->PreCollision( &event.m_Event );
		object1->GetJoltEnvironment()->RefreshCallbackBodyProperties();
		const auto preEnd = profile ? Clock::now() : Clock::time_point{};
		if ( object1->IsMoveable() ) object1->CommitPendingVelocityChanges();
		if ( object2->IsMoveable() ) object2->CommitPendingVelocityChanges();
		event.m_Event.collisionSpeed = event.m_Data.RefreshContactSpeed();
		const auto responseBegin = profile ? Clock::now() : Clock::time_point{};
		const bool applied = response();
		const auto responseEnd = profile ? Clock::now() : Clock::time_point{};
		// Source owns one game event: keep each Pre/response/Post indivisible.
		listener->PostCollision( &event.m_Event );
		object1->GetJoltEnvironment()->RefreshCallbackBodyProperties();
		if ( profile ) object1->GetJoltEnvironment()->RecordContactCallbackTimings(
			std::chrono::duration<double, std::milli>( ( preEnd - preBegin ) + ( Clock::now() - responseEnd ) ).count(),
			std::chrono::duration<double, std::milli>( responseEnd - responseBegin ).count() );
		if ( object1->IsMoveable() ) object1->CommitPendingVelocityChanges();
		if ( object2->IsMoveable() ) object2->CommitPendingVelocityChanges();
		return applied;
	} );
}
