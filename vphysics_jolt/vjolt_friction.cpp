
#include "cbase.h"

#include "vjolt_collide.h"
#include "vjolt_friction.h"
#include "vjolt_object.h"
#include "vjolt_environment.h"
#include "vjolt_listener_contact.h"
#include "vjolt_surfaceprops.h"

#include "tier0/memdbgon.h"

JoltPhysicsFrictionSnapshot::JoltPhysicsFrictionSnapshot( JoltPhysicsObject *pObject )
	: m_pSelf( pObject )
{
	if ( !pObject || !pObject->IsCollisionEnabled() )
		return;

	// The snapshot owns the transformed records below, so this immutable published-frame staging
	// copy can retain its allocation between snapshots on the same game thread.
	thread_local std::vector< JoltFrictionContactCache::Contact > cachedContacts;
	pObject->GetJoltEnvironment()->GetContactListener()->CopyFrictionContacts(
		pObject->GetBodyID(), cachedContacts );
	m_contacts.reserve( cachedContacts.size() );

	for ( const JoltFrictionContactCache::Contact &cached : cachedContacts )
	{
		Contact contact;
		contact.otherBodyID = cached.otherBodyID;
		contact.selfSubShapeID = cached.selfSubShapeID;
		contact.otherSubShapeID = cached.otherSubShapeID;
		contact.vNormal = Vector( cached.normal.GetX(), cached.normal.GetY(), cached.normal.GetZ() );
		JPH::RVec3 worldContactPoint = cached.contactPoint;
		if ( !cached.contactPointBodyID.IsInvalid() )
		{
			JPH::BodyLockRead pointBodyLock(
				pObject->GetJoltEnvironment()->GetBodyLockInterfaceForRead(), cached.contactPointBodyID );
			if ( !pointBodyLock.Succeeded() )
				continue;
			worldContactPoint = pointBodyLock.GetBody().GetCenterOfMassTransform()
				* cached.contactPointLocal;
		}
		contact.vContactPoint = JoltToSource::Distance( worldContactPoint );
		contact.worldContactPoint = worldContactPoint;
		contact.flPenetrationDepth = cached.penetrationDepth;
		contact.flNormalImpulse = cached.normalImpulse;
		contact.flFrictionEnergy = cached.frictionEnergy;
		contact.flFrameDeltaTime = cached.frameDeltaTime;

		// Resolve the sequence-numbered BodyID now and discard contacts whose object was
		// destroyed after the manifold frame was published.
		if ( ResolveOtherObject( contact ) )
			m_contacts.push_back( contact );
	}
}

//-------------------------------------------------------------------------------------------------

JoltPhysicsObject *JoltPhysicsFrictionSnapshot::ResolveOtherObject( const Contact &contact ) const
{
	if ( !m_pSelf || contact.otherBodyID.IsInvalid() )
		return nullptr;

	JPH::BodyLockRead lock( m_pSelf->GetJoltEnvironment()->GetBodyLockInterfaceForRead(), contact.otherBodyID );
	if ( !lock.Succeeded() )
		return nullptr;

	JoltPhysicsObject *pOther = reinterpret_cast< JoltPhysicsObject * >( lock.GetBody().GetUserData() );
	return pOther != m_pSelf ? pOther : nullptr;
}

//-------------------------------------------------------------------------------------------------

bool JoltPhysicsFrictionSnapshot::IsValid()
{
	return m_index < m_contacts.size();
}

//-------------------------------------------------------------------------------------------------

IPhysicsObject *JoltPhysicsFrictionSnapshot::GetObject( int index )
{
	if ( !IsValid() )
		return nullptr;
	return ( index == 0 ) ? static_cast< IPhysicsObject * >( m_pSelf )
	                      : static_cast< IPhysicsObject * >( ResolveOtherObject( m_contacts[ m_index ] ) );
}

int JoltPhysicsFrictionSnapshot::GetMaterial( int index )
{
	if ( !IsValid() )
		return 0;

	const Contact &contact = m_contacts[ m_index ];
	JPH::SubShapeID subShapeID;
	const JPH::Vec3 selfNormal(
		contact.vNormal.x, contact.vNormal.y, contact.vNormal.z );
	if ( index == 0 )
	{
		subShapeID.SetValue( contact.selfSubShapeID );
		return JoltResolveSourceContactMaterial( *m_pSelf->GetBody(), subShapeID,
			contact.worldContactPoint, selfNormal, m_pSelf->GetMaterialIndex() );
	}

	JoltPhysicsObject *pOther = ResolveOtherObject( contact );
	if ( !pOther )
		return 0;
	subShapeID.SetValue( contact.otherSubShapeID );
	return JoltResolveSourceContactMaterial( *pOther->GetBody(), subShapeID,
		contact.worldContactPoint, -selfNormal, pOther->GetMaterialIndex() );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFrictionSnapshot::GetContactPoint( Vector &out )
{
	if ( !IsValid() )
	{
		out.Zero();
		return;
	}
	out = m_contacts[ m_index ].vContactPoint;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFrictionSnapshot::GetSurfaceNormal( Vector &out )
{
	if ( !IsValid() )
	{
		out.Zero();
		return;
	}
	out = m_contacts[ m_index ].vNormal;
}

float JoltPhysicsFrictionSnapshot::GetNormalForce()
{
	if ( !IsValid() )
		return 0.0f;

	const float flImpulse_N_s = m_contacts[ m_index ].flNormalImpulse;
	const float flStepDt = m_contacts[ m_index ].flFrameDeltaTime;
	if ( flStepDt <= 0.0f )
		return 0.0f;
	const float flForce_N = flImpulse_N_s / flStepDt;
	return JoltToSource::Distance( flForce_N );
}

float JoltPhysicsFrictionSnapshot::GetPenetrationDepth() const
{
	if ( m_index >= m_contacts.size() )
		return 0.0f;
	return JoltToSource::Distance( m_contacts[ m_index ].flPenetrationDepth );
}

float JoltPhysicsFrictionSnapshot::GetEnergyAbsorbed()
{
	if ( !IsValid() )
		return 0.0f;

	// Already in HL energy units - the contact listener fed JoltToSource::Energy()
	// when it accumulated this value (matches IVP's ConvertEnergyToHL of get_eliminated_energy).
	return m_contacts[ m_index ].flFrictionEnergy;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFrictionSnapshot::RecomputeFriction()
{
	// IVP recomputes the contact basis and advances its tangential spring to the
	// environment's current time. Jolt stores contact points in body-local space and
	// derives the current world basis when the next constraint is built, so there is
	// no stale geometry or per-contact timestamp to refresh here. In particular, this
	// operation must not wake an otherwise sleeping contact.
}

void JoltPhysicsFrictionSnapshot::ClearFrictionForce()
{
	if ( !IsValid() || !m_pSelf )
		return;

	const Contact &contact = m_contacts[ m_index ];
	JPH::SubShapeID selfSubShapeID;
	selfSubShapeID.SetValue( contact.selfSubShapeID );
	JPH::SubShapeID otherSubShapeID;
	otherSubShapeID.SetValue( contact.otherSubShapeID );
	auto *environment = m_pSelf->GetJoltEnvironment();
	const auto clear = [system = environment->GetPhysicsSystem(), selfID = m_pSelf->GetBodyID(),
		selfSubShapeID, otherID = contact.otherBodyID, otherSubShapeID]()
	{
		system->ClearVPhysicsFrictionState( selfID, selfSubShapeID, otherID, otherSubShapeID );
	};
	if ( !environment->DeferObjectMutation( m_pSelf, clear ) ) clear();
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFrictionSnapshot::MarkContactForDelete()
{
	if ( !IsValid() )
		return;

	JoltPhysicsObject *pOther = ResolveOtherObject( m_contacts[ m_index ] );
	if ( !pOther )
		return;

	// Match IVP's deferred-delete pattern: collect the other-body IDs, act on
	// them in DeleteAllMarkedContacts.
	m_DeleteList.push_back( m_contacts[ m_index ].otherBodyID );
}

void JoltPhysicsFrictionSnapshot::DeleteAllMarkedContacts( bool wakeObjects )
{
	if ( m_DeleteList.empty() )
		return;
	if ( m_pSelf && m_pSelf->GetJoltEnvironment()->IsInContactCallback() )
	{
		m_pSelf->GetJoltEnvironment()->DeferObjectMutation( m_pSelf, [self = m_pSelf, ids = m_DeleteList, wakeObjects]()
		{
			JoltPhysicsFrictionSnapshot deferred( self );
			deferred.m_DeleteList = ids;
			deferred.DeleteAllMarkedContacts( wakeObjects );
		} );
		m_DeleteList.clear();
		return;
	}

	// IVP's CFrictionSnapshot::DeleteAllMarkedContacts calls DeleteAllFrictionPairs
	// (= unlink_contact_points_for_object) per other-body. Jolt manages the contact
	// manifold cache internally and exposes invalidation per body rather than per
	// body pair. Invalidate both involved bodies, remove the pair from the published
	// Source snapshot, and let the next update re-detect any contact that still exists.
	JPH::BodyInterface *pBodyInterface = nullptr;
	if ( wakeObjects && m_pSelf )
	{
		pBodyInterface = &m_pSelf->GetJoltEnvironment()->GetPhysicsSystem()->GetBodyInterfaceNoLock();
	}

	for ( const JPH::BodyID &otherBodyID : m_DeleteList )
	{
		Contact contact;
		contact.otherBodyID = otherBodyID;
		JoltPhysicsObject *pOther = ResolveOtherObject( contact );
		if ( !pOther )
			continue;

		m_pSelf->GetJoltEnvironment()->GetContactListener()->RemovePublishedFrictionContacts(
			m_pSelf->GetBodyID(), pOther->GetBodyID() );
		if ( !pBodyInterface )
			pBodyInterface = &m_pSelf->GetJoltEnvironment()->GetPhysicsSystem()->GetBodyInterfaceNoLock();
		pBodyInterface->InvalidateContactCache( m_pSelf->GetBodyID() );
		pBodyInterface->InvalidateContactCache( pOther->GetBodyID() );

		if ( wakeObjects && !pOther->IsStatic() )
			pBodyInterface->ActivateBody( pOther->GetBodyID() );
	}

	if ( wakeObjects && m_pSelf )
		m_pSelf->Wake();

	m_DeleteList.clear();
	m_contacts.clear();
	m_index = 0;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsFrictionSnapshot::NextFrictionData()
{
	if ( m_index < m_contacts.size() )
		m_index++;
}

float JoltPhysicsFrictionSnapshot::GetFrictionCoefficient()
{
	if ( !IsValid() )
		return 0.0f;

	const int matSelf = GetMaterial( 0 );
	const int matOther = GetMaterial( 1 );
	if ( !ResolveOtherObject( m_contacts[ m_index ] ) )
		return 0.0f;

	surfacedata_t *pSelf = JoltPhysicsSurfaceProps::GetInstance().GetSurfaceData( matSelf );
	surfacedata_t *pOther = JoltPhysicsSurfaceProps::GetInstance().GetSurfaceData( matOther );
	if ( !pSelf || !pOther )
		return 0.0f;

	// CIVPMaterialManager reports the effective contact coefficient, not the raw
	// surfaceprop product. Keep the public snapshot consistent with the solver.
	return Clamp<float>( pSelf->physics.friction * pOther->physics.friction, 0.0f, 1.0f )
		* m_pSelf->GetJoltEnvironment()->GetContactFrictionScale();
}
