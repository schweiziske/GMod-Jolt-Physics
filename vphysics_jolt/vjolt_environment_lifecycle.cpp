// Object deletion can call game code. Keep queue ownership and environment
// teardown together so those callbacks never invalidate an active traversal.
#include "cbase.h"
#include "vjolt_environment.h"
#include "vjolt_collide.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

namespace
{
template <typename T, typename Delete>
void DrainDeleteQueue( std::vector<T *> &queue, const Delete &destroy )
{
	// Callbacks may append and reallocate this vector. Retain an index, not an
	// iterator/reference, and consume the new entries before clearing the queue.
	for ( size_t i = 0; i < queue.size(); ++i )
	{
		T *entry = queue[i];
		queue[i] = nullptr;
		if ( entry ) destroy( entry );
	}
	queue.clear();
}
}

void JoltPhysicsEnvironment::DestroyObject( IPhysicsObject *pObject )
{
	if ( !pObject )
		return;

	auto *pJoltObject = static_cast<JoltPhysicsObject *>( pObject );
	// A callback may request deletion of a still-live wrapper that teardown has
	// already queued. Coalesce it just like a repeated DestroyConstraint request.
	if ( pJoltObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE )
		return;

	pJoltObject->AddCallbackFlags( CALLBACK_MARKED_FOR_DELETE );
	ForgetRestoredPointer( pJoltObject );
	m_pDeadObjects.push_back( pJoltObject );
	if ( !m_bSimulating && !m_bEnableDeleteQueue && !m_bDeletingDeadObjects )
		DeleteDeadObjects();
}

void JoltPhysicsEnvironment::CleanupDeleteList()
{
	// Simulate still owns snapshots of wrappers until its callback phase ends.
	if ( !m_bSimulating )
		DeleteDeadObjects();
}

void JoltPhysicsEnvironment::EnableDeleteQueue( bool enable )
{
	m_bEnableDeleteQueue = enable;
}

void JoltPhysicsEnvironment::DestroyCollideOnDeadObjectFlush( CPhysCollide *pCollide )
{
	bool defer = m_bDeletingDeadObjects;
	for ( JoltPhysicsObject *pObject : m_pDeadObjects )
	{
		// Consumed entries are cleared before their callbacks run.
		if ( pObject && pObject->GetCollide() == pCollide )
		{
			defer = true;
			break;
		}
	}
	if ( defer )
	{
		if ( !VectorContains( m_pDeadObjectCollides, pCollide ) )
			m_pDeadObjectCollides.push_back( pCollide );
		return;
	}
	JoltPhysicsCollision::GetInstance().DestroyCollide( pCollide );
}

void JoltPhysicsEnvironment::RemoveBodyAndDeleteObject( JoltPhysicsObject *pObject )
{
	ForgetRestoredPointer( pObject );
	JPH::BodyInterface &bodyInterface = m_PhysicsSystem.GetBodyInterfaceNoLock();
	if ( !pObject->GetBodyID().IsInvalid() )
	{
		m_ContactListener.InvalidateFrictionContacts( pObject->GetBodyID() );
		bodyInterface.RemoveBody( pObject->GetBodyID() );
	}
	delete pObject;
}

void JoltPhysicsEnvironment::DeleteDeadObjects()
{
	if ( m_bDeletingDeadObjects ||
		( m_pDeadConstraints.empty() && m_pDeadObjects.empty() && m_pDeadObjectCollides.empty() ) )
		return;

	struct DeleteScope
	{
		bool &active;
		explicit DeleteScope( bool &active ) : active( active ) { active = true; }
		~DeleteScope() { active = false; }
	} scope( m_bDeletingDeadObjects );

	do
	{
		DrainDeleteQueue( m_pDeadConstraints, []( JoltPhysicsConstraint *constraint ) { delete constraint; } );
		DrainDeleteQueue( m_pDeadObjects, [this]( JoltPhysicsObject *object ) { RemoveBodyAndDeleteObject( object ); } );
	}
	while ( !m_pDeadConstraints.empty() || !m_pDeadObjects.empty() );

	// Keep collision data alive until all object callbacks have finished, including
	// callbacks that queue another object using the same collision model.
	DrainDeleteQueue( m_pDeadObjectCollides,
		[]( CPhysCollide *collide ) { JoltPhysicsCollision::GetInstance().DestroyCollide( collide ); } );
}

void JoltPhysicsEnvironment::DestroyEnvironmentObjects()
{
	m_bEnableDeleteQueue = true;
	DeleteDeadObjects();

	JPH::BodyIDVector bodies;
	for ( ;; )
	{
		// GetObjectList also uses m_CachedBodies. A game callback may query it or
		// delete another body, so teardown must own its snapshot and recheck IDs.
		GetSourceBodiesForRead( bodies );
		if ( bodies.empty() )
			break;
		for ( const JPH::BodyID &id : bodies )
		{
			const JPH::Body *body = m_PhysicsSystem.GetBodyLockInterfaceNoLock().TryGetBody( id );
			if ( body )
			{
				auto *object = reinterpret_cast<JoltPhysicsObject *>( body->GetUserData() );
				// No callbacks run while collecting this batch. Queue each live body
				// once, even if game code has already set its public deletion flag.
				object->AddCallbackFlags( CALLBACK_MARKED_FOR_DELETE );
				ForgetRestoredPointer( object );
				m_pDeadObjects.push_back( object );
			}
		}
		DeleteDeadObjects();
	}
}
