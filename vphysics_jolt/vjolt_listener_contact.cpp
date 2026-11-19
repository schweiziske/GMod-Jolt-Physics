#include "cbase.h"

#include "vjolt_environment.h"
#include <chrono>

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

// RaphaelIT7: This is needed for 64x due to else creating - relocation R_X86_64_TPOFF32 against hidden symbol
thread_local uint32 s_ThreadId = ~0u;

void JoltPhysicsContactListener::QueueTriggerEvent( TriggerTransition transition, JoltPhysicsObject *trigger, JoltPhysicsObject *other )
{
	std::lock_guard lock( m_TriggerEventMutex );
	m_TriggerEvents.push_back( { transition, trigger, other } );
}

void JoltPhysicsContactListener::RemoveTriggerContacts( JoltPhysicsObject *trigger )
{
	auto *environment = trigger->GetJoltEnvironment();
	auto *system = environment->GetPhysicsSystem();
	const JPH::BodyID triggerID = trigger->GetBodyID();
	JPH::Array<JPH::SubShapeIDPair> contacts;
	// Consume actual entries, including static and speculative contacts. Geometric
	// overlap alone cannot establish which pairs received an EnterTrigger.
	system->ClearContactUserDataForBody( triggerID, Trigger1, Trigger2, contacts );
	for ( const auto &contact : contacts )
	{
		if ( !m_pGameListener ) break;
		const JPH::Body *triggerBody = system->GetBodyLockInterfaceNoLock().TryGetBody( triggerID );
		if ( !triggerBody || triggerBody->GetUserData() != reinterpret_cast<JPH::uint64>( trigger )
			|| ( trigger->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) ) break;
		const JPH::BodyID otherID = contact.GetBody1ID() == triggerID ? contact.GetBody2ID() : contact.GetBody1ID();
		const JPH::Body *body = system->GetBodyLockInterfaceNoLock().TryGetBody( otherID );
		auto *other = body ? reinterpret_cast<JoltPhysicsObject *>( body->GetUserData() ) : nullptr;
		if ( !other || ( other->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) ) continue;
		if ( m_bTriggerEventsPending )
			QueueTriggerEvent( TriggerTransition::Leave, trigger, other );
		else
			m_pGameListener->ObjectLeaveTrigger( trigger, other );
	}
}

bool JoltPhysicsContactListener::QueryGameShouldCollide( JoltPhysicsObject *object0, JoltPhysicsObject *object1 )
{
	auto *environment = object0->GetJoltEnvironment();
	// GMod may skip Lua when this runs off the simulation owner. A mutex alone
	// cannot make that answer valid. Queue before taking the callback mutex so
	// no other solver worker can block the owner's cooperative pause.
	if ( environment->HasOwnerCallbackJobs() && !environment->IsInContactCallback() )
		return environment->RunContactBatch( [&]() { return ShouldCollide( object0, object1 ); } );

	std::unique_lock lock( m_ShouldCollideLock );
	const auto lookup = m_ShouldCollideCache.Find(
		MakeShouldCollideObjectKey( object0 ), MakeShouldCollideObjectKey( object1 ) );
	if ( lookup.found )
		return lookup.shouldCollide;
	IPhysicsCollisionSolver *solver = m_pGameSolver.load( std::memory_order_acquire );
	if ( !solver )
		return true;
	const bool collide = solver->ShouldCollide(
		object0, object1, object0->GetGameData(), object1->GetGameData() );
	// A rule change during the callback must not publish its preceding answer.
	m_ShouldCollideCache.StoreIfUnchanged( lookup, collide );
	return collide;
}

void JoltPhysicsContactListener::OnContactValidationBatch( const std::function<void()> &batch )
{
	if ( m_Environment.HasOwnerCallbackJobs() )
		m_Environment.RunContactBatch( [&]()
		{
			if ( !m_bPrepareGameFilterQueries.exchange( false, std::memory_order_acq_rel ) )
				return false;
			const float distance = m_PhysicsSystem.GetPhysicsSettings().mSpeculativeContactDistance;
			const auto &bodies = m_PhysicsSystem.GetBodyLockInterfaceNoLock();
			for ( const auto &pair : m_InvalidatedFilterPairs )
			{
				const auto *body1 = bodies.TryGetBody( JPH::BodyID( pair.bodyKey0 ) );
				const auto *body2 = bodies.TryGetBody( JPH::BodyID( pair.bodyKey1 ) );
				if ( !body1 || !body2 || !body1->IsInBroadPhase() || !body2->IsInBroadPhase()
					|| !body1->IsRigidBody() || !body2->IsRigidBody() ) continue;
				// Use the same active-body ordering and filters as Jolt's broadphase.
				// In particular, never cache an answer for a NO_COLLIDE former pair.
				if ( body1->GetIndexInActiveBodiesInternal() > body2->GetIndexInActiveBodiesInternal() )
					std::swap( body1, body2 );
				if ( !body1->IsActive() || !JPH::Body::sFindCollidingPairsCanCollide( *body1, *body2 )
					|| !m_PhysicsSystem.GetObjectVsBroadPhaseLayerFilter().ShouldCollide( body1->GetObjectLayer(), body2->GetBroadPhaseLayer() )
					|| !m_PhysicsSystem.GetObjectLayerPairFilter().ShouldCollide( body1->GetObjectLayer(), body2->GetObjectLayer() ) ) continue;
				auto bounds = body1->GetWorldSpaceBounds();
				bounds.ExpandBy( JPH::Vec3::sReplicate( distance ) );
				if ( !bounds.Overlaps( body2->GetWorldSpaceBounds() ) ) continue;
				auto *object1 = reinterpret_cast<JoltPhysicsObject *>( body1->GetUserData() );
				auto *object2 = reinterpret_cast<JoltPhysicsObject *>( body2->GetUserData() );
				if ( !object1 || !object2 || ( ( object1->GetCallbackFlags() | object2->GetCallbackFlags() )
					& CALLBACK_MARKED_FOR_DELETE ) ) continue;
				ShouldCollide( object1, object2 );
			}
			// All jobs are parked in this owner callback. Publish the refreshed
			// answers before they resume parallel collision detection.
			m_ShouldCollideCache.PublishReadSnapshot();
			m_InvalidatedFilterPairs.clear();
			return false;
		} );
	batch();
}

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
