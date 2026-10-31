//=================================================================================================
//
// A physics object, implemented as a wrapper over JPH::Body
// Every tangible object in the game has one of these
//
//=================================================================================================

#include "cbase.h"
#include "bridge/vjolt_gravity_internal.h"

#include "vjolt_collide.h"
#include "vjolt_surfaceprops.h"
#include "vjolt_friction.h"
#include "vjolt_environment.h"
#include "vjolt_layers.h"
#include "vjolt_listener_contact.h"
#include "vjolt_controller_drag.h"
#include "vjolt_controller_shadow.h"
#include "vjolt_shadow_control.h"
#include "bridge/vjolt_stats_internal.h"

#include "vjolt_object.h"

#include <Jolt/Physics/Collision/Shape/EmptyShape.h>

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

static void SetLinearVelocityWithoutImmediateClamp( JPH::Body &body, JPH::Vec3Arg velocity )
{
	JPH::MotionProperties *motion = body.GetMotionProperties();
	const float oldMaximum = motion->GetMaxLinearVelocity();
	const float requiredMaximum = velocity.Length();
	if ( requiredMaximum > oldMaximum )
		motion->SetMaxLinearVelocity( requiredMaximum );
	body.SetLinearVelocity( velocity );
	if ( requiredMaximum > oldMaximum )
		motion->SetMaxLinearVelocity( oldMaximum );
}

static void SetAngularVelocityWithoutImmediateClamp( JPH::Body &body, JPH::Vec3Arg velocity )
{
	JPH::MotionProperties *motion = body.GetMotionProperties();
	const float oldMaximum = motion->GetMaxAngularVelocity();
	const float requiredMaximum = velocity.Length();
	if ( requiredMaximum > oldMaximum )
		motion->SetMaxAngularVelocity( requiredMaximum );
	body.SetAngularVelocity( velocity );
	if ( requiredMaximum > oldMaximum )
		motion->SetMaxAngularVelocity( oldMaximum );
}

//-------------------------------------------------------------------------------------------------

#if GAME_GMOD
// RaphaelIT7: Gmod needs to keep track of which objects exists
// This is because Lua stores a pointer to the IPhysicsObject
// And it cannot invalidate all pointers since
// first - there can be multiple userdatas that keep a pointer stored
// secondly - an IPhysicsObject can be deleted outside of Lua

// NOTE: Perferably gmod will use the SetLuaReference function
// & use the IGModPhysicsObjectEvent::ObjectDestroyed callback to invalidate references instead
static std::unordered_set< JoltPhysicsObject* > g_pObjects;
inline void RegisterPhysicsObject( JoltPhysicsObject* pObject )
{
	auto it = g_pObjects.find(pObject);
	if (it == g_pObjects.end())
		g_pObjects.insert(pObject);
}

inline void UnregisterPhysicsObject( JoltPhysicsObject* pObject )
{
	auto it = g_pObjects.find(pObject);
	if (it != g_pObjects.end())
		g_pObjects.erase(it);
}

bool IsValidPhyiscsObject( IPhysicsObject* pObject ) // for vjolt_interface.cpp to use
{
	JoltPhysicsObject *pJoltObject = static_cast< JoltPhysicsObject * >( pObject );
	return g_pObjects.find( pJoltObject ) != g_pObjects.end();
}
#endif

JoltPhysicsObject::JoltPhysicsObject( JPH::Body *pBody, JoltPhysicsEnvironment *pEnvironment, bool bStatic, int nMaterialIndex, const objectparams_t *pParams )
	: m_pBody( pBody )
	, m_pEnvironment( pEnvironment )
	, m_pPhysicsSystem( pEnvironment->GetPhysicsSystem() )
	, m_bStatic( bStatic )
	, m_flCachedMass( pParams->mass )
	, m_flCachedInvMass( m_flCachedMass ? 1.0f / m_flCachedMass : 0.0f )
	, m_flCurrentInvMass( m_flCachedInvMass )
	, m_pGameData( pParams->pGameData )
	, m_materialIndex( Max( nMaterialIndex, 0 ) ) // Sometimes we get passed -1.
	, m_flVolume( pParams->volume )
	, m_pName( pParams->pName )
{
	// Josh:
	// Assert that m_pGameData is the first element, some games
	// decide to just read this directly by offsetting by the vtable size
	// instead of calling GetGameData().
	static_assert( offsetof( JoltPhysicsObject, m_pGameData ) == sizeof( void * ) );

	// Set the body's userdata as ourselves
	pBody->SetUserData( reinterpret_cast<uint64>( this ) );
	const JPH::Shape *pShape = pBody->GetShape();
	// Collision-model shapes created by JoltPhysicsCollision own a
	// JoltCollideUserData pointer. A Source sphere object has no CPhysCollide and
	// is created directly by the environment, so this also keeps a fallback
	// polygonal SphereShape from being mistaken for COLLIDE_BALL.
	m_bSphereObject = pShape->GetSubType() == JPH::EShapeSubType::Sphere
		&& pShape->GetUserData() == 0;
	if ( m_bSphereObject )
	{
		m_flSphereRadius = JoltToSource::Distance(
			static_cast< const JPH::SphereShape * >( pShape )->GetRadius() );
	}
	if ( !m_pBody->IsStatic() )
	{
		JPH::MotionProperties* pMotionProperties = m_pBody->GetMotionProperties();
		pMotionProperties->SetLinearDamping( pParams->damping );
		pMotionProperties->SetAngularDamping( pParams->rotdamping );
	}

	if ( !IsStatic() )
	{
		GetVelocity( &m_vLastVelocity, &m_vLastAngularVelocity );
		GetPosition( &m_vLastPosition, &m_qLastOrientation );

		RecomputeDrag();

		if ( pParams->dragCoefficient != 0.0f )
		{
			float flDrag = pParams->dragCoefficient;
			SetDragCoefficient( &flDrag, &flDrag );
			EnableDrag( true );
		}
	}

	UpdateMaterialProperties();

#if GAME_GMOD
	// RaphaelIT7's validity tracking was only ever hooked up in the save/restore constructor
	// below, while the destructor unregisters every object. That left g_pObjects empty for all
	// normally created objects, so IsValidPhyiscsObject answered false for every live PhysObj.
	RegisterPhysicsObject( this );
#endif
}

JoltPhysicsObject::JoltPhysicsObject( JPH::Body *pBody, JoltPhysicsEnvironment *pEnvironment, void *pGameData, JPH::StateRecorder &recorder, bool extended )
	: m_pBody( pBody )
	, m_pEnvironment( pEnvironment )
	, m_pPhysicsSystem( pEnvironment->GetPhysicsSystem() )
	, m_pGameData( pGameData )
{
	RestoreObjectState( recorder, extended );
	const JPH::Shape *pShape = m_pBody->GetShape();
	if ( !extended )
	{
		m_bSphereObject = pShape->GetSubType() == JPH::EShapeSubType::Sphere
			&& pShape->GetUserData() == 0;
		if ( m_bSphereObject )
			m_flSphereRadius = JoltToSource::Distance(
				static_cast< const JPH::SphereShape * >( pShape )->GetRadius() );
	}

#if GAME_GMOD
	if (m_pEnvironment->GetGModObjectEvents())
		m_pEnvironment->GetGModObjectEvents()->ObjectCreated( this );

	RegisterPhysicsObject( this );
#endif
}

JoltPhysicsObject::~JoltPhysicsObject()
{
	// Remove only this object's ShouldCollide pairs before its BodyID can be recycled. The cache
	// key contains Jolt's sequence number, but eagerly dropping the reverse-indexed pairs also keeps
	// long-running servers from retaining results for destroyed props.
	m_pEnvironment->GetContactListener()->InvalidateShouldCollideCache( this );

	if ( m_bPendingVelocityQueued )
	{
		m_pEnvironment->RemovePendingVelocityCommit( this );
		m_bPendingVelocityQueued = false;
	}

#if GAME_GMOD
	UnregisterPhysicsObject( this );

	if (m_pEnvironment->GetGModObjectEvents())
		m_pEnvironment->GetGModObjectEvents()->ObjectDestroyed( this );
#endif

	RemoveShadowController();

	// A constraint callback can delete other listeners as well as itself. Consume
	// the current listener first, then read the updated list after the callback.
	while ( m_destroyedListeners.Count() > 0 )
	{
		const int last = m_destroyedListeners.Count() - 1;
		IJoltObjectDestroyedListener *listener = m_destroyedListeners[ last ];
		m_destroyedListeners.Remove( last );
		listener->OnJoltPhysicsObjectDestroyed( this );
	}
	VJoltGravityObjectDestroyed( this );

	m_pEnvironment->RemoveDirtyStaticBody( GetBodyID() );

	JPH::BodyInterface& bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
	if ( GetBodyID().IsInvalid() ) bodyInterface.DestroyBodyWithoutID( m_pBody );
	else bodyInterface.DestroyBody( GetBodyID() );
	JoltPhysicsCollision::GetInstance().DestroyCollide( m_pOwnedRestoredCollide );
}

bool JoltPhysicsObject::IsStatic() const
{
	// Whether this a static body to VPhysics and not
	// to Jolt (static or motion disabled) itself.
	return m_bStatic;
}

bool JoltPhysicsObject::IsAsleep() const
{
	return !m_pBody->IsActive();
}

bool JoltPhysicsObject::IsTrigger() const
{
	return m_pBody->IsSensor();
}

bool JoltPhysicsObject::IsFluid() const
{
	return m_pFluidController != nullptr;
}

bool JoltPhysicsObject::IsHinged() const
{
	return m_iHingedAxis >= 0;
}

bool JoltPhysicsObject::IsCollisionEnabled() const
{
	return m_bCachedCollisionEnabled;
}

bool JoltPhysicsObject::IsGravityEnabled() const
{
	bool enabled;
	if ( VJoltGravityIsObjectEnabled( this, enabled ) ) return enabled;
	if ( !IsStatic() )
	{
		JPH::MotionProperties* pMotionProperties = m_pBody->GetMotionPropertiesUnchecked();
		return pMotionProperties->GetGravityFactor() != 0.0f;
	}

	return false;
}

bool JoltPhysicsObject::IsDragEnabled() const
{
	return m_bDragEnabled;
}

bool JoltPhysicsObject::IsMotionEnabled() const
{
	return !m_bPinned;
}

bool JoltPhysicsObject::IsMoveable() const
{
	return IsMotionEnabled() && !IsStatic();
}

bool JoltPhysicsObject::IsAttachedToConstraint( bool bExternalOnly ) const
{
	for (JPH::Constraint *constraint : m_pEnvironment->GetConstraintsForRead())
	{
		if ( constraint->GetType() != JPH::EConstraintType::TwoBodyConstraint )
			continue;

		JPH::TwoBodyConstraint *twoBody = static_cast<JPH::TwoBodyConstraint*>( constraint );
		if ( twoBody->GetBody1() == m_pBody )
		{
			// RaphaelIT7: (ToDo) Check if this might need to call TryGetBody to be safe?
			if ( bExternalOnly )
			{
				JoltPhysicsObject *pObject = reinterpret_cast< JoltPhysicsObject * >( twoBody->GetBody2()->GetUserData() );
				// RaphaelIT7: vphysics seems to compare bExternalOnly by comparing the GameData, so lets keep that behavior too
				if ( pObject && pObject->GetGameData() != GetGameData() )
					return true;
				continue;
			}

			return true;
		} else if ( twoBody->GetBody2() == m_pBody )
		{
			if ( bExternalOnly )
			{
				JoltPhysicsObject *pObject = reinterpret_cast< JoltPhysicsObject * >( twoBody->GetBody1()->GetUserData() );
				if ( pObject && pObject->GetGameData() != GetGameData() )
					return true;
				continue;
			}

			return true;
		}
	}

	return false;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::EnableCollisions( bool enable )
{
	if ( m_pEnvironment->IsInContactCallback() && m_pEnvironment->DeferObjectMutation( this, [this, enable]() { EnableCollisions( enable ); } ) ) return;
	// Josh:
	// When collisions are disabled, they disable for EVERYTHING, including against the world.
	m_bCachedCollisionEnabled = enable;
	// UpdateLayer invalidates friction contacts before applying the new collision layer.
	UpdateLayer();
}

void JoltPhysicsObject::EnableGravity( bool enable )
{
	if ( VJoltGravityEnableObject( this, enable ) ) return;
	if ( !IsStatic() )
	{
		JPH::MotionProperties* pMotionProperties = m_pBody->GetMotionPropertiesUnchecked();
		pMotionProperties->SetGravityFactor( enable ? 1.0f : 0.0f );
	}
}

void JoltPhysicsObject::EnableDrag( bool enable )
{
	if ( IsStatic() )
		return;

	if ( m_bDragEnabled == enable )
		return;

	m_bDragEnabled = enable;

	JoltPhysicsDragController *pDragController = m_pEnvironment->GetDragController();
	if ( enable )
		pDragController->RegisterObject( this );
	else
		pDragController->UnregisterObject( this );
}

void JoltPhysicsObject::EnableMotion( bool enable )
{
	if ( m_pEnvironment->IsInContactCallback() && m_pEnvironment->DeferObjectMutation( this, [this, enable]() { EnableMotion( enable ); } ) ) return;
	if ( IsStatic() )
		return;

	const bool bPinned = !enable;

	if ( m_bPinned == bPinned )
		return;

	// IVP_Real_Object::set_pinned clears speed and rot_speed for both pin and
	// unpin, and clears speed_change / rot_speed_change when pinning. Pending
	// public impulses cannot survive either transition here: a pinned object
	// cannot acquire a new batch, and preserving an old one would replay motion
	// when the object becomes dynamic again.
	ClearPendingVelocityChanges();
	if ( bPinned )
	{
		m_pBody->SetLinearVelocity( JPH::Vec3::sZero() );
		m_pBody->SetAngularVelocity( JPH::Vec3::sZero() );
	}

	if ( bPinned )
	{
		// IVP_Real_Object::set_pinned stores the ordinary rotational inertia but
		// zeros the live inverse inertia and inverse mass.
		if ( !m_bHasUnhingedInertia )
			CaptureUnhingedInertia();
		if ( m_bHingedInertiaApplied )
			RestoreUnhingedInertia();
		JPH::MotionProperties *pMotion = m_pBody->GetMotionProperties();
		pMotion->SetInverseInertia( JPH::Vec3::sZero(), JPH::Quat::sIdentity() );
		pMotion->SetInverseMass( 0.0f );
		m_flCurrentInvMass = 0.0f;
		m_bHingedInertiaApplied = false;
	}
	else if ( m_bHasUnhingedInertia )
	{
		RestoreUnhingedInertia();
	}

	m_bPinned = bPinned;
	UpdateLayer();
	if ( enable )
	{
		// UpdateLayer changes a pinned body back to Dynamic. Clear the stored Jolt
		// velocities after that transition as IVP set_pinned(false) does.
		m_pBody->SetLinearVelocity( JPH::Vec3::sZero() );
		m_pBody->SetAngularVelocity( JPH::Vec3::sZero() );
	}
	if ( enable && IsHinged() )
		BecomeHinged( m_iHingedAxis );
	else if ( enable )
		m_bHasUnhingedInertia = false;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::SetGameData( void *pGameData )
{
	if (m_pGameData != pGameData) SetNPCNonSolid(false);
	m_pGameData = pGameData;
}

void *JoltPhysicsObject::GetGameData() const
{
	return m_pGameData;
}

void JoltPhysicsObject::SetGameFlags( unsigned short userFlags )
{
	m_gameFlags = userFlags;
}

unsigned short JoltPhysicsObject::GetGameFlags() const 
{
	return m_gameFlags;
}

void JoltPhysicsObject::SetGameIndex( unsigned short gameIndex )
{
	m_gameIndex = gameIndex;
}

unsigned short JoltPhysicsObject::GetGameIndex() const
{
	return m_gameIndex;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::SetCallbackFlags( unsigned short callbackflags )
{
	m_callbackFlags = callbackflags;
}

unsigned short JoltPhysicsObject::GetCallbackFlags() const
{
	return m_callbackFlags;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::Wake()
{
	if ( m_pEnvironment->IsInContactCallback() && m_pEnvironment->DeferObjectMutation( this, [this]() { Wake(); } ) ) return;
	if ( !m_pBody->IsStatic() )
	{
		JPH::BodyInterface& bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
		bodyInterface.ActivateBody( m_pBody->GetID() );
	}
	else
	{
		// See other comments in UpdateLayer.
		m_pEnvironment->AddDirtyStaticBody( m_pBody->GetID() );
	}
}

void JoltPhysicsObject::Sleep()
{
	if ( m_pEnvironment->IsInContactCallback() && m_pEnvironment->DeferObjectMutation( this, [this]() { Sleep(); } ) ) return;
	if ( !m_pBody->IsStatic() )
	{
		JPH::BodyInterface& bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
		bodyInterface.DeactivateBody( m_pBody->GetID() );
	}
}

void JoltPhysicsObject::RecheckCollisionFilter()
{
	RecheckContactPoints();
}

void JoltPhysicsObject::RecheckContactPoints( bool bSearchForNewContacts /*= false*/ )
{
	if ( m_pEnvironment->IsInContactCallback() && m_pEnvironment->DeferObjectMutation( this, [this, bSearchForNewContacts]() { RecheckContactPoints( bSearchForNewContacts ); } ) ) return;
	GetJoltEnvironment()->GetContactListener()->InvalidateFrictionContacts( GetBodyID() );
	JPH::BodyInterface& bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
	bodyInterface.InvalidateContactCache( GetBodyID() );

	GetJoltEnvironment()->GetContactListener()->InvalidateShouldCollideCache( this );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::SetMass( float mass )
{
	// Updating a frozen body's stored properties needs no broadphase transition.
	// Unpin/repin through EnableMotion would defer and leave a zero mass in the
	// live callback, or change the layout of already prepared contact rows.
	if ( m_bPinned && !IsStatic() )
	{
		ClearPendingVelocityChanges();
		RestoreUnhingedInertia();
	}

	// IVP_Core::set_mass scales the existing rotational inertia by the same
	// mass ratio. It does not ask the collision shape to calculate a new tensor.
	mass = Clamp( mass, 1.0f, VPHYSICS_MAX_MASS );

	if ( !IsStatic() )
	{
		if ( m_bHingedInertiaApplied )
			RestoreUnhingedInertia();

		JPH::MotionProperties* pMotionProperties = m_pBody->GetMotionPropertiesUnchecked();
		pMotionProperties->ScaleToMass( mass );
		m_flCachedMass = mass;
		m_flCachedInvMass = mass ? 1.0f / mass : 0.0f;
		m_flCurrentInvMass = m_flCachedInvMass;
		if ( IsHinged() )
		{
			// IVP_Core::set_mass recalculates inverse inertia from the ordinary
			// rotational inertia. The hinged flag survives, but its physical lock is
			// gone until BecomeHinged is called again.
			CaptureUnhingedInertia();
			m_bHingedInertiaApplied = false;
		}

		CalculateBuoyancy();
		RecomputeDrag();
	}
	else
	{
		m_flCachedMass = mass;
		m_flCachedInvMass = mass ? 1.0f / mass : 0.0f;
		m_flCurrentInvMass = m_flCachedInvMass;
	}

	if ( m_bPinned && !IsStatic() )
	{
		CaptureUnhingedInertia();
		auto *motion = m_pBody->GetMotionPropertiesUnchecked();
		motion->SetInverseMass( 0.0f );
		motion->SetInverseInertia( JPH::Vec3::sZero(), JPH::Quat::sIdentity() );
		m_flCurrentInvMass = 0.0f;
	}
	m_pEnvironment->NotifyCallbackBodyPropertiesChanged( this );
}

float JoltPhysicsObject::GetMass() const
{
	return m_flCachedMass;
}

float JoltPhysicsObject::GetInvMass() const
{
	return m_flCurrentInvMass;
}

Vector JoltPhysicsObject::GetInertia() const
{
	if ( IsStatic() )
		return Vector( 1.0f, 1.0f, 1.0f );

	// Jolt stores principal moments plus a rotation. Eigen decomposition is free
	// to permute those principal axes even when Source supplied a body-axis
	// diagonal tensor, so expose the diagonal of the reconstructed local tensor.
	const JPH::Mat44 localInverseInertia =
		( m_bHingedInertiaApplied || m_bPinned ) && m_bHasUnhingedInertia
		? JPH::Mat44::sRotation( m_qUnhingedInertiaRotation )
			* JPH::Mat44::sScale( m_vUnhingedInverseInertia )
			* JPH::Mat44::sRotation( m_qUnhingedInertiaRotation ).Transposed()
		: m_pBody->GetMotionPropertiesUnchecked()->GetLocalSpaceInverseInertiaUnchecked();
	return Vector(
		localInverseInertia( 0, 0 ) > 0.0f ? 1.0f / localInverseInertia( 0, 0 ) : 0.0f,
		localInverseInertia( 1, 1 ) > 0.0f ? 1.0f / localInverseInertia( 1, 1 ) : 0.0f,
		localInverseInertia( 2, 2 ) > 0.0f ? 1.0f / localInverseInertia( 2, 2 ) : 0.0f );
}

Vector JoltPhysicsObject::GetInvInertia() const
{
	if ( IsStatic() )
		return Vector( 1.0f, 1.0f, 1.0f );
	const JPH::Mat44 localInverseInertia = m_pBody->GetMotionPropertiesUnchecked()->GetLocalSpaceInverseInertiaUnchecked();
	return Abs( Vector(
		localInverseInertia( 0, 0 ),
		localInverseInertia( 1, 1 ),
		localInverseInertia( 2, 2 ) ) );
}

void JoltPhysicsObject::SetInertia( const Vector &inertia )
{
	if ( IsStatic() )
		return;

	Vector ri = Abs( inertia );

	JPH::Vec3 joltInvInertia(
		ri.x > 1e-9f ? 1.0f / ri.x : 0.0f,
		ri.y > 1e-9f ? 1.0f / ri.y : 0.0f,
		ri.z > 1e-9f ? 1.0f / ri.z : 0.0f );

	m_pBody->GetMotionPropertiesUnchecked()->SetInverseInertia( joltInvInertia, JPH::Quat::sIdentity() );
	m_pBody->GetMotionPropertiesUnchecked()->SetInverseMass( m_flCachedInvMass );
	m_flCurrentInvMass = m_flCachedInvMass;
	if ( IsHinged() || m_bPinned )
	{
		// IVP set_rotation_inertia recalculates the ordinary inverse tensor and
		// therefore removes the temporary hinge lock without clearing its flag.
		m_vUnhingedInverseInertia = joltInvInertia;
		m_qUnhingedInertiaRotation = JPH::Quat::sIdentity();
		m_bHasUnhingedInertia = true;
		m_bHingedInertiaApplied = false;
	}
	m_pEnvironment->NotifyCallbackBodyPropertiesChanged( this );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::SetDamping( const float *speed, const float *rot )
{
	if ( IsStatic() )
		return;
	if ( m_pShadowController )
	{
		m_pShadowController->SetDamping( speed, rot );
		return;
	}

	JPH::MotionProperties* pMotionProperties = m_pBody->GetMotionProperties();
	if ( speed )
		pMotionProperties->SetLinearDamping( *speed );
	if ( rot )
		pMotionProperties->SetAngularDamping( *rot );
}

void JoltPhysicsObject::GetDamping( float *speed, float *rot ) const
{
	if ( IsStatic() )
		return;
	if ( m_pShadowController )
	{
		m_pShadowController->GetDamping( speed, rot );
		return;
	}

	JPH::MotionProperties* pMotionProperties = m_pBody->GetMotionProperties();
	if ( speed )
		*speed = pMotionProperties->GetLinearDamping();
	if ( rot )
		*rot = pMotionProperties->GetAngularDamping();
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::SetDragCoefficient( float *pDrag, float *pAngularDrag )
{
	if ( pDrag )
		m_flLinearDragCoefficient = *pDrag;

	if ( pAngularDrag )
		m_flAngularDragCoefficient = *pAngularDrag;

	// Source 1 behaviour...
	EnableDrag( m_flLinearDragCoefficient || m_flAngularDragCoefficient );
}

void JoltPhysicsObject::SetBuoyancyRatio( float ratio )
{
	m_flBuoyancyRatio = ratio;
}

//-------------------------------------------------------------------------------------------------

int JoltPhysicsObject::GetMaterialIndex() const
{
	return m_materialIndex;
}

void JoltPhysicsObject::SetMaterialIndex( int materialIndex )
{
	// Gotta clamp it, because we get -1 sometimes
	materialIndex = Max( 0, materialIndex );

	if ( m_materialIndex != materialIndex )
	{
		m_materialIndex = materialIndex;
		UpdateMaterialProperties();
		if ( m_pShadowController )
			m_pShadowController->ObjectMaterialChanged( materialIndex );
	}
}

//-------------------------------------------------------------------------------------------------

unsigned int JoltPhysicsObject::GetContents() const
{
	return m_contents;
}

void JoltPhysicsObject::SetContents( unsigned int contents )
{
	m_contents = contents;
}

//-------------------------------------------------------------------------------------------------

float JoltPhysicsObject::GetSphereRadius() const
{
	return m_bSphereObject ? m_flSphereRadius : 0.0f;
}

void JoltPhysicsObject::SetSphereRadius( float radius )
{
	if ( m_pEnvironment->IsInContactCallback() && m_pEnvironment->DeferObjectMutation( this, [this, radius]() { SetSphereRadius( radius ); } ) ) return;
	if ( !m_bSphereObject || radius == m_flSphereRadius )
		return;

	JPH::RefConst< JPH::Shape > replacement;
	const float joltRadius = SourceToJolt::Distance( radius );
	if ( std::isfinite( joltRadius ) && joltRadius > 0.0f )
	{
		// Source mutates IVP_Real_Object::extra_radius. Jolt shapes are immutable,
		// so replace only this body's private sphere. Do not update mass properties:
		// IVP leaves mass, rotational inertia and velocity untouched.
		const JPH::PhysicsMaterial *pMaterial = m_pBody->GetShape()->GetSubType()
			== JPH::EShapeSubType::Sphere
			? static_cast< const JPH::SphereShape * >( m_pBody->GetShape() )->GetMaterial()
			: nullptr;
		replacement = new JPH::SphereShape( joltRadius, pMaterial );
	}
	else
	{
		// IVP accepts zero, negative and non-finite extra radii. Jolt cannot
		// represent a signed sphere radius; an EmptyShape preserves the no-volume
		// collision result while m_flSphereRadius retains the exact public value.
		replacement = new JPH::EmptyShape();
	}

	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
	bodyInterface.SetShape( GetBodyID(), replacement.GetPtr(), false,
		JPH::EActivation::DontActivate );
	m_flSphereRadius = radius;
}

float JoltPhysicsObject::GetEnergy() const
{
	// Source reports 1/2 m v^2 + 1/2 w I w. Velocities and inertia are metric
	// inside both IVP and Jolt; convert only the final energy back to Source units.
	const float flKineticEnergy = 0.5f * m_flCachedMass * m_pBody->GetLinearVelocity().LengthSq();

	float flAngularEnergy = 0.0f;
	if ( !IsStatic() )
	{
		const JPH::MotionProperties *pMotion = m_pBody->GetMotionProperties();
		const JPH::Quat principalRotation =
			m_pBody->GetRotation() * pMotion->GetInertiaRotation();
		const JPH::Vec3 principalAngularVelocity =
			principalRotation.Conjugated() * m_pBody->GetAngularVelocity();
		const JPH::Vec3 inverseInertia = pMotion->GetInverseInertiaDiagonal();

		for ( int axis = 0; axis < 3; ++axis )
		{
			// Pinned/locked axes have zero inverse inertia and cannot carry a valid
			// angular velocity. Ignore them rather than manufacturing infinite energy.
			if ( inverseInertia[ axis ] > 0.0f )
				flAngularEnergy += 0.5f
					* Square( principalAngularVelocity[ axis ] ) / inverseInertia[ axis ];
		}
	}

	return JoltToSource::Energy( flKineticEnergy + flAngularEnergy );
}

Vector JoltPhysicsObject::GetMassCenterLocalSpace() const
{
	return JoltToSource::Distance( m_pBody->GetShape()->GetCenterOfMass() );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::SetPosition( const Vector &worldPosition, const QAngle &angles, bool isTeleport )
{
	if ( m_pEnvironment->IsInContactCallback() && m_pEnvironment->DeferObjectMutation( this, [this, worldPosition, angles, isTeleport]() { SetPosition( worldPosition, angles, isTeleport ); } ) ) return;

	JPH::Vec3 joltPosition = SourceToJolt::Distance( worldPosition );
	JPH::Quat joltRotation = SourceToJolt::Angle( angles );

	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();

	GetJoltEnvironment()->GetContactListener()->InvalidateFrictionContacts( GetBodyID() );
	bodyInterface.SetPositionAndRotation( m_pBody->GetID(), joltPosition, joltRotation, JPH::EActivation::DontActivate );

	if ( isTeleport || IsStatic() )
	{
		m_vLastPosition = worldPosition;
		m_qLastOrientation = angles;
	}
}

void JoltPhysicsObject::SetPositionMatrix( const matrix3x4_t &matrix, bool isTeleport )
{
	SetPosition( GetColumn( matrix, MatrixAxis::Origin ), ToQAngle( matrix ), isTeleport );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::GetPosition( Vector *worldPosition, QAngle *angles ) const
{
	if ( worldPosition )
		*worldPosition = JoltToSource::Distance( m_pBody->GetPosition() );

	if ( angles )
		*angles = JoltToSource::Angle( m_pBody->GetRotation() );
}

void JoltPhysicsObject::GetPositionMatrix( matrix3x4_t *positionMatrix ) const
{
	matrix3x4_t matrix;
	SetIdentityMatrix( matrix );
	AngleMatrix( JoltToSource::Angle( m_pBody->GetRotation() ), JoltToSource::Distance( m_pBody->GetPosition() ), matrix );
	*positionMatrix = matrix;
}

void JoltPhysicsObject::SetVelocity( const Vector *velocity, const AngularImpulse *angularVelocity )
{
	if ( m_pEnvironment->IsInContactCallback() )
	{
		const Vector v = velocity ? *velocity : vec3_origin;
		const AngularImpulse w = angularVelocity ? *angularVelocity : vec3_origin;
		if ( m_pEnvironment->DeferObjectMutation( this, [this, v, w, linear = velocity != nullptr, angular = angularVelocity != nullptr]()
			{ SetVelocity( linear ? &v : nullptr, angular ? &w : nullptr ); }, true ) ) return;
	}
	if ( !IsMoveable() )
		return;

	const JPH::BodyLockInterfaceNoLock &bodyLockInterface = m_pPhysicsSystem->GetBodyLockInterfaceNoLock();
	JPH::BodyLockWrite lock( bodyLockInterface, m_pBody->GetID() );
	if ( !lock.Succeeded() )
		return;

	JPH::Body &body = lock.GetBody();
	if ( velocity )
	{
		body.SetLinearVelocity( JPH::Vec3::sZero() );
		m_vPendingLinearVelocity = SourceToJolt::Distance( *velocity );
	}

	if ( angularVelocity )
	{
		body.SetAngularVelocity( JPH::Vec3::sZero() );
		m_vPendingAngularVelocityLocal = SourceToJolt::AngularImpulse( *angularVelocity );
	}

	ClampVelocityChanges( body );
	QueuePendingVelocityCommit();
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::SetVelocityInstantaneous( const Vector *velocity, const AngularImpulse *angularVelocity )
{
	if ( m_pEnvironment->IsInContactCallback() )
	{
		const Vector v = velocity ? *velocity : vec3_origin;
		const AngularImpulse w = angularVelocity ? *angularVelocity : vec3_origin;
		if ( m_pEnvironment->DeferObjectMutation( this, [this, v, w, linear = velocity != nullptr, angular = angularVelocity != nullptr]()
			{ SetVelocityInstantaneous( linear ? &v : nullptr, angular ? &w : nullptr ); }, true ) ) return;
	}
	if ( !IsMoveable() )
		return;

	const JPH::BodyLockInterfaceNoLock &bodyLockInterface = m_pPhysicsSystem->GetBodyLockInterfaceNoLock();
	JPH::BodyLockWrite lock( bodyLockInterface, m_pBody->GetID() );
	if ( !lock.Succeeded() )
		return;

	JPH::Body &body = lock.GetBody();
	if ( velocity )
	{
		SetLinearVelocityWithoutImmediateClamp( body, SourceToJolt::Distance( *velocity ) );
		m_vPendingLinearVelocity = JPH::Vec3::sZero();
	}

	if ( angularVelocity )
	{
		const JPH::Vec3 localAngularVelocity = SourceToJolt::AngularImpulse( *angularVelocity );
		SetAngularVelocityWithoutImmediateClamp( body, body.GetRotation() * localAngularVelocity );
		m_vPendingAngularVelocityLocal = JPH::Vec3::sZero();
	}

	ClampVelocityChanges( body );
	Wake();
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::GetVelocity( Vector *velocity, AngularImpulse *angularVelocity ) const
{
	if ( velocity )
		*velocity = JoltToSource::Distance( m_pBody->GetLinearVelocity() + m_vPendingLinearVelocity );

	if ( angularVelocity )
	{
		const JPH::Vec3 localAngularVelocity = m_pBody->GetRotation().Conjugated() * m_pBody->GetAngularVelocity()
			+ m_vPendingAngularVelocityLocal;
		*angularVelocity = JoltToSource::AngularImpulse( localAngularVelocity );
	}
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::AddVelocity( const Vector *velocity, const AngularImpulse *angularVelocity )
{
	if ( m_pEnvironment->IsInContactCallback() )
	{
		const Vector v = velocity ? *velocity : vec3_origin;
		const AngularImpulse w = angularVelocity ? *angularVelocity : vec3_origin;
		if ( m_pEnvironment->DeferObjectMutation( this, [this, v, w, linear = velocity != nullptr, angular = angularVelocity != nullptr]()
			{ AddVelocity( linear ? &v : nullptr, angular ? &w : nullptr ); }, true ) ) return;
	}
	if ( !IsMoveable() )
		return;

	if ( velocity )
		m_vPendingLinearVelocity += SourceToJolt::Distance( *velocity );
	if ( angularVelocity )
		m_vPendingAngularVelocityLocal += SourceToJolt::AngularImpulse( *angularVelocity );

	ClampVelocityChanges( *m_pBody );
	QueuePendingVelocityCommit();
}

void JoltPhysicsObject::GetVelocityAtPoint( const Vector &worldPosition, Vector *pVelocity ) const
{
	VJoltAssert( pVelocity );

	const JPH::Vec3 linearVelocity = m_pBody->GetLinearVelocity() + m_vPendingLinearVelocity;
	const JPH::Vec3 angularVelocity = m_pBody->GetAngularVelocity()
		+ m_pBody->GetRotation() * m_vPendingAngularVelocityLocal;
	const JPH::Vec3 arm = JPH::Vec3( SourceToJolt::Distance( worldPosition ) - m_pEnvironment->GetObjectResponseCenterOfMass( this ) );
	*pVelocity = JoltToSource::Distance( linearVelocity + angularVelocity.Cross( arm ) );
}

void JoltPhysicsObject::GetImplicitVelocity( Vector *velocity, AngularImpulse *angularVelocity ) const
{
	if ( velocity )
		*velocity = m_vLastVelocity;

	if ( angularVelocity )
		*angularVelocity = m_vLastAngularVelocity;
}

void JoltPhysicsObject::LocalToWorld( Vector *worldPosition, const Vector &localPosition ) const
{
	matrix3x4_t matrix;
	GetPositionMatrix( &matrix );
	// Copy in case src == dest
	VectorTransform( Vector( localPosition ), matrix, *worldPosition );
}

void JoltPhysicsObject::WorldToLocal( Vector *localPosition, const Vector &worldPosition ) const
{
	matrix3x4_t matrix;
	GetPositionMatrix( &matrix );
	// Copy in case src == dest
	VectorITransform( Vector( worldPosition ), matrix, *localPosition );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::LocalToWorldVector( Vector *worldVector, const Vector &localVector ) const
{
	matrix3x4_t matrix;
	GetPositionMatrix( &matrix );
	// Copy in case src == dest
	VectorRotate( Vector( localVector ), matrix, *worldVector );
}

void JoltPhysicsObject::WorldToLocalVector( Vector *localVector, const Vector &worldVector ) const
{
	matrix3x4_t matrix;
	GetPositionMatrix( &matrix );
	// Copy in case src == dest
	VectorIRotate( Vector( worldVector ), matrix, *localVector );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::ApplyForceCenter( const Vector &forceVector )
{
	if ( m_pEnvironment->IsInContactCallback() && m_pEnvironment->DeferObjectMutation( this, [this, forceVector]() { ApplyForceCenter( forceVector ); }, true ) ) return;
	if ( !IsMoveable() )
		return;

	const JPH::Vec3 impulse = SourceToJolt::Distance( forceVector );

	m_vPendingLinearVelocity += impulse * m_pBody->GetMotionProperties()->GetInverseMass();
	ClampVelocityChanges( *m_pBody );
	QueuePendingVelocityCommit();
}

void JoltPhysicsObject::ApplyForceOffset( const Vector &forceVector, const Vector &worldPosition )
{
	if ( m_pEnvironment->IsInContactCallback() && m_pEnvironment->DeferObjectMutation( this, [this, forceVector, worldPosition]() { ApplyForceOffset( forceVector, worldPosition ); }, true ) ) return;
	if ( !IsMoveable() )
		return;

	const JPH::Vec3 impulse = SourceToJolt::Distance( forceVector );
	const JPH::Vec3 point = SourceToJolt::Distance( worldPosition );

	JPH::MotionProperties *motion = m_pBody->GetMotionProperties();
	const JPH::Vec3 arm = JPH::Vec3( point - m_pEnvironment->GetObjectResponseCenterOfMass( this ) );
	const JPH::Vec3 angularVelocityWorld = motion->MultiplyWorldSpaceInverseInertiaByVector(
		m_pBody->GetRotation(), arm.Cross( impulse ) );
	m_vPendingLinearVelocity += impulse * motion->GetInverseMass();
	m_vPendingAngularVelocityLocal += m_pBody->GetRotation().Conjugated() * angularVelocityWorld;
	ClampVelocityChanges( *m_pBody );
	QueuePendingVelocityCommit();
}

void JoltPhysicsObject::ApplyTorqueCenter( const AngularImpulse &torque )
{
	if ( m_pEnvironment->IsInContactCallback() && m_pEnvironment->DeferObjectMutation( this, [this, torque]() { ApplyTorqueCenter( torque ); }, true ) ) return;
	if ( !IsMoveable() )
		return;

	const JPH::Vec3 angularImpulseWorld = SourceToJolt::AngularImpulse( torque );
	const JPH::Vec3 angularVelocityWorld = m_pBody->GetMotionProperties()->MultiplyWorldSpaceInverseInertiaByVector(
		m_pBody->GetRotation(), angularImpulseWorld );
	m_vPendingAngularVelocityLocal += m_pBody->GetRotation().Conjugated() * angularVelocityWorld;
	ClampVelocityChanges( *m_pBody );
	QueuePendingVelocityCommit();
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::QueuePendingVelocityCommit()
{
	Wake();
	if ( m_bPendingVelocityQueued )
		return;

	m_bPendingVelocityQueued = true;
	m_pEnvironment->QueuePendingVelocityCommit( this );
}

void JoltPhysicsObject::ClearPendingVelocityChanges()
{
	m_vPendingLinearVelocity = JPH::Vec3::sZero();
	m_vPendingAngularVelocityLocal = JPH::Vec3::sZero();
	if ( m_bPendingVelocityQueued )
	{
		m_pEnvironment->RemovePendingVelocityCommit( this );
		m_bPendingVelocityQueued = false;
	}
}

void JoltPhysicsObject::ClampVelocityChanges( JPH::Body &body )
{
	// CPhysicsObject::ClampVelocity deliberately leaves shadow-controlled objects alone.
	if ( m_pShadowController )
		return;

	JPH::MotionProperties *motion = body.GetMotionProperties();
	const float maxLinearVelocity = motion->GetMaxLinearVelocity();
	if ( maxLinearVelocity > 0.0f )
	{
		JPH::Vec3 linearVelocity = body.GetLinearVelocity();
		const float linearLength = linearVelocity.Length();
		if ( linearLength > maxLinearVelocity )
		{
			linearVelocity *= maxLinearVelocity / linearLength;
			SetLinearVelocityWithoutImmediateClamp( body, linearVelocity );
		}

		const float pendingLinearLength = m_vPendingLinearVelocity.Length();
		if ( pendingLinearLength > maxLinearVelocity )
			m_vPendingLinearVelocity *= maxLinearVelocity / pendingLinearLength;
	}

	const float maxAngularVelocity = motion->GetMaxAngularVelocity();
	if ( maxAngularVelocity > 0.0f )
	{
		JPH::Vec3 angularVelocity = body.GetAngularVelocity();
		const float angularLength = angularVelocity.Length();
		if ( angularLength > maxAngularVelocity )
		{
			angularVelocity *= maxAngularVelocity / angularLength;
			SetAngularVelocityWithoutImmediateClamp( body, angularVelocity );
		}

		// Preserve IVP_Core::apply_velocity_limit exactly, including its historical use of
		// speed_change (rather than rot_speed_change) to choose the pending angular scale.
		const float pendingLinearLength = m_vPendingLinearVelocity.Length();
		if ( pendingLinearLength > maxAngularVelocity )
			m_vPendingAngularVelocityLocal *= maxAngularVelocity / pendingLinearLength;
	}
}

void JoltPhysicsObject::CommitPendingVelocityChanges()
{
	const JPH::Vec3 pendingLinearVelocity = m_vPendingLinearVelocity;
	const JPH::Vec3 pendingAngularVelocityLocal = m_vPendingAngularVelocityLocal;
	m_vPendingLinearVelocity = JPH::Vec3::sZero();
	m_vPendingAngularVelocityLocal = JPH::Vec3::sZero();
	m_bPendingVelocityQueued = false;

	const JPH::BodyLockInterfaceNoLock &bodyLockInterface = m_pPhysicsSystem->GetBodyLockInterfaceNoLock();
	JPH::BodyLockWrite lock( bodyLockInterface, m_pBody->GetID() );
	if ( !lock.Succeeded() )
		return;

	JPH::Body &body = lock.GetBody();
	SetLinearVelocityWithoutImmediateClamp( body, body.GetLinearVelocity() + pendingLinearVelocity );
	SetAngularVelocityWithoutImmediateClamp( body, body.GetAngularVelocity()
		+ body.GetRotation() * pendingAngularVelocityLocal );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::CalculateForceOffset( const Vector &forceVector, const Vector &worldPosition, Vector *centerForce, AngularImpulse *centerTorque ) const
{
	JPH::Vec3 pos = SourceToJolt::Distance( worldPosition );
	JPH::Vec3 force = SourceToJolt::Distance( forceVector );

	JPH::Vec3 com = pos - m_pEnvironment->GetObjectResponseCenterOfMass( this );
	JPH::Vec3 cross = com.Cross( force );

	if ( centerForce )
		*centerForce = JoltToSource::Distance( force );

	if ( centerTorque )
	{
		Vector worldTorque = JoltToSource::AngularImpulse( cross );
		WorldToLocalVector( centerTorque, worldTorque );
	}
}

void JoltPhysicsObject::CalculateVelocityOffset( const Vector &forceVector, const Vector &worldPosition, Vector *centerVelocity, AngularImpulse *centerAngularVelocity ) const
{
	// Convert force to SI units to multiply by mass for impulse.
	JPH::Vec3 siForce = SourceToJolt::Distance( forceVector );

	if ( centerVelocity )
		*centerVelocity = JoltToSource::Distance( siForce * m_flCachedInvMass );

	if ( centerAngularVelocity )
	{
		JPH::Vec3 siPosition = SourceToJolt::Distance( worldPosition );

		JPH::Vec3 siRelativePosition = siPosition - m_pEnvironment->GetObjectResponseCenterOfMass( this );
		JPH::Vec3 cross = siRelativePosition.Cross( siForce );

		Vector worldAngVel = JoltToSource::AngularImpulse( m_pBody->GetInverseInertia() * cross );
		WorldToLocalVector( centerAngularVelocity, worldAngVel );
	}
}

float JoltPhysicsObject::CalculateLinearDrag( const Vector &unitDirection ) const
{
	return GetDragInDirection( SourceToJolt::Unitless( unitDirection ) );
}

float JoltPhysicsObject::CalculateAngularDrag( const Vector &objectSpaceRotationAxis ) const
{
	// Drag factor is per-radian internally; Source 1 callers expect per-degree.
	return GetAngularDragInDirection( SourceToJolt::Unitless( objectSpaceRotationAxis ) ) * DEG2RAD( 1.0f );
}

//-------------------------------------------------------------------------------------------------

bool JoltPhysicsObject::GetContactPoint( Vector *contactPoint, IPhysicsObject **contactObject ) const
{
	// Source returns the first existing IVP friction synapse; it does not run a fresh query. Reuse
	// the same published contact frame as IPhysicsFrictionSnapshot so sleeping/speculative contacts
	// and the selected point stay consistent across both public interfaces. Copy only the first
	// immutable record: this interface is called frequently and must not allocate a full snapshot.
	if ( !IsCollisionEnabled() )
		return false;

	JoltFrictionContactCache::Contact cached;
	if ( !m_pEnvironment->GetContactListener()->CopyFirstFrictionContact( m_pBody->GetID(), cached ) )
		return false;

	Vector resolvedContactPoint;
	if ( contactPoint )
	{
		JPH::RVec3 worldContactPoint = cached.contactPoint;
		if ( !cached.contactPointBodyID.IsInvalid() )
		{
			JPH::BodyLockRead pointBodyLock(
				m_pEnvironment->GetBodyLockInterfaceForRead(),
				cached.contactPointBodyID );
			if ( !pointBodyLock.Succeeded() )
				return false;
			worldContactPoint = pointBodyLock.GetBody().GetCenterOfMassTransform()
				* cached.contactPointLocal;
		}
		resolvedContactPoint = JoltToSource::Distance( worldContactPoint );
	}

	JoltPhysicsObject *pOther = nullptr;
	if ( contactObject )
	{
		JPH::BodyLockRead otherBodyLock(
			m_pEnvironment->GetBodyLockInterfaceForRead(), cached.otherBodyID );
		if ( !otherBodyLock.Succeeded() )
			return false;
		pOther = reinterpret_cast< JoltPhysicsObject * >(
			otherBodyLock.GetBody().GetUserData() );
		if ( !pOther )
			return false;
	}

	if ( contactPoint )
		*contactPoint = resolvedContactPoint;
	if ( contactObject )
		*contactObject = pOther;
	return true;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::SetShadow( float maxSpeed, float maxAngularSpeed, bool allowPhysicsMovement, bool allowPhysicsRotation )
{
	// Source's SetupVPhysicsHull immediately dereferences GetShadowController(),
	// including when an impact spawns an NPC inside a contact callback. Publish
	// the interface and its body properties synchronously.
	if ( m_pShadowController )
	{
		m_pShadowController->MaxSpeed( maxSpeed, maxAngularSpeed );
	}
	else
	{
#if 1
		m_bShadowTemporarilyDisableGravity = false;
#endif

		m_pShadowController = static_cast<JoltPhysicsShadowController *>( m_pEnvironment->CreateShadowController( this, allowPhysicsMovement, allowPhysicsRotation ) );
		m_pShadowController->MaxSpeed( maxSpeed, maxAngularSpeed );
		RecheckCollisionFilter();
	}
}

void JoltPhysicsObject::UpdateShadow( const Vector &targetPosition, const QAngle &targetAngles, bool tempDisableGravity, float timeOffset )
{
	if ( m_pShadowController )
	{
#if 1
		if ( tempDisableGravity != m_bShadowTemporarilyDisableGravity )
		{
			m_bShadowTemporarilyDisableGravity = tempDisableGravity;
			if ( !m_pShadowController || m_pShadowController->AllowsTranslation() )
				EnableGravity( !m_bShadowTemporarilyDisableGravity );
		}
#endif
		m_pShadowController->Update( targetPosition, targetAngles, timeOffset );
	}
}

//-------------------------------------------------------------------------------------------------

int JoltPhysicsObject::GetShadowPosition( Vector *position, QAngle *angles ) const
{
	const float flTimeStep = m_pEnvironment->GetSimulationTimestep();

	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();

	JPH::Vec3 joltPosition, joltLinearVelocity, joltAngularVelocity;
	JPH::Quat joltRotation;

	bodyInterface.GetPositionAndRotation( m_pBody->GetID(), joltPosition, joltRotation );
	bodyInterface.GetLinearAndAngularVelocity( m_pBody->GetID(), joltLinearVelocity, joltAngularVelocity );

	if ( position )
	{
		*position = JoltToSource::Distance( joltPosition + ( joltLinearVelocity * flTimeStep ) );
	}
	if ( angles )
	{
		// From Jolt's AddRotationStep.

		JPH::Vec3 joltAngularVelocityTimesDT = joltAngularVelocity * flTimeStep;
		float len = joltAngularVelocityTimesDT.Length();

		JPH::Quat newQuat = joltRotation;
		if ( len > 1.0e-6f ) 
			newQuat = ( JPH::Quat::sRotation( joltAngularVelocityTimesDT / len, len ) * joltRotation ).Normalized();
		*angles = JoltToSource::Angle( newQuat );
	}

	return 1;
}

IPhysicsShadowController *JoltPhysicsObject::GetShadowController() const
{
	return m_pShadowController;
}

void JoltPhysicsObject::RemoveShadowController()
{
	if ( m_pShadowController )
	{
		m_pEnvironment->DestroyShadowController(m_pShadowController);
		m_pShadowController = nullptr;
		if ( !( GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) )
			RecheckCollisionFilter();
	}
}

namespace
{
	// Adapter workaround introduced in b40ea10b, not part of Source shadow control.
	// Removing it exposes wall-squeeze failures even though container lift still works.
	// See docs/audit/v1-legacy-penetration-cleanup-2026-09-08.md before changing this path.
	// Jolt's ordinary CCD follows the body that already owns the velocity. A physgun-held
	// body can instead give a sleeping body a large contact velocity at the end of the step;
	// that downstream body then reaches its blocker before the discrete chain has converged.
	// Look ahead through only that push chain and cap the held body's inward component to the
	// remaining clearance. Open-space pushes never reach a terminal blocker and are unchanged.
	constexpr int kPhysgunClearanceMaxBodies = 8;
	constexpr float kPhysgunClearanceMarginInches = 0.05f;

	class PhysgunClearanceBodyFilter final : public JPH::BodyFilter
	{
	public:
		PhysgunClearanceBodyFilter( JoltPhysicsObject *pMovingObject,
			const JPH::BodyID *pVisitedBodies, int nVisitedBodies )
			: m_pMovingObject( pMovingObject )
			, m_pVisitedBodies( pVisitedBodies )
			, m_nVisitedBodies( nVisitedBodies )
		{
		}

		bool ShouldCollide( const JPH::BodyID &bodyID ) const override
		{
			for ( int i = 0; i < m_nVisitedBodies; ++i )
			{
				if ( m_pVisitedBodies[i] == bodyID )
					return false;
			}
			return true;
		}

		bool ShouldCollideLocked( const JPH::Body &body ) const override
		{
			JoltPhysicsObject *pOtherObject = reinterpret_cast<JoltPhysicsObject *>( body.GetUserData() );
			if ( !pOtherObject || pOtherObject == m_pMovingObject
				|| pOtherObject->GetJoltEnvironment() != m_pMovingObject->GetJoltEnvironment()
				|| body.IsSensor() || pOtherObject->IsTrigger()
				|| !pOtherObject->IsCollisionEnabled() )
				return false;

			return m_pMovingObject->GetJoltEnvironment()->GetContactListener()
				->ShouldCollide( m_pMovingObject, pOtherObject );
		}

	private:
		JoltPhysicsObject *m_pMovingObject;
		const JPH::BodyID *m_pVisitedBodies;
		int m_nVisitedBodies;
	};

	class PhysgunForwardShapeCastCollector final : public JPH::CastShapeCollector
	{
	public:
		explicit PhysgunForwardShapeCastCollector( JPH::Vec3Arg displacement )
			: m_vDisplacement( displacement )
		{
		}

		void AddHit( const JPH::ShapeCastResult &hit ) override
		{
			if ( hit.mFraction < 0.0f || hit.mFraction > 1.0f )
				return;

			const float flIntoSurface = hit.mPenetrationAxis.Dot( m_vDisplacement );
			if ( flIntoSurface <= 1.0e-8f )
				return;

			const float flOrdering = hit.mFraction > 0.0f
				? hit.mFraction
				: -hit.mPenetrationDepth;
			if ( m_bHadHit && flOrdering >= m_flOrdering )
				return;

			m_bHadHit = true;
			m_flOrdering = flOrdering;
			m_Hit = hit;
			UpdateEarlyOutFraction( flOrdering );
		}

		bool HadHit() const { return m_bHadHit; }
		const JPH::ShapeCastResult &GetHit() const { return m_Hit; }

	private:
		JPH::Vec3 m_vDisplacement;
		JPH::ShapeCastResult m_Hit;
		float m_flOrdering = FLT_MAX;
		bool m_bHadHit = false;
	};

	bool CastPhysgunClearanceShape( JoltPhysicsObject *pMovingObject,
		JPH::Vec3Arg displacement, const JPH::BodyID *pVisitedBodies,
		int nVisitedBodies, JPH::ShapeCastResult &outHit )
	{
		if ( displacement.LengthSq() <= Square( SourceToJolt::Distance( 0.001f ) ) )
			return false;

		JPH::Body *pBody = pMovingObject->GetBody();
		JPH::PhysicsSystem *pPhysicsSystem = pMovingObject->GetJoltEnvironment()->GetPhysicsSystem();
		JPH::ShapeCastSettings settings;
		settings.mUseShrunkenShapeAndConvexRadius = true;
		settings.mReturnDeepestPoint = true;
		settings.mActiveEdgeMovementDirection = displacement;

		PhysgunForwardShapeCastCollector collector( displacement );
		PhysgunClearanceBodyFilter bodyFilter( pMovingObject, pVisitedBodies, nVisitedBodies );
		const JPH::DefaultBroadPhaseLayerFilter broadPhaseFilter =
			pPhysicsSystem->GetDefaultBroadPhaseLayerFilter( pBody->GetObjectLayer() );
		const JPH::DefaultObjectLayerFilter objectLayerFilter =
			pPhysicsSystem->GetDefaultLayerFilter( pBody->GetObjectLayer() );
		const JPH::RShapeCast shapeCast( pBody->GetShape(), JPH::Vec3::sOne(),
			pBody->GetCenterOfMassTransform(), displacement );
		pPhysicsSystem->GetNarrowPhaseQueryNoLock().CastShape( shapeCast, settings,
			shapeCast.mCenterOfMassStart.GetTranslation(), collector,
			broadPhaseFilter, objectLayerFilter, bodyFilter );

		if ( !collector.HadHit() )
			return false;

		outHit = collector.GetHit();
		return true;
	}

	void ClampPhysgunHeldVelocityAgainstBlockedChain( JoltPhysicsObject *pHeldObject,
		JPH::Vec3 &ioLinearVelocity, float flDeltaTime )
	{
		if ( flDeltaTime <= 0.0f || ioLinearVelocity.IsNaN()
			|| !pHeldObject->IsCollisionEnabled() )
			return;

		const float flHeldSpeed = ioLinearVelocity.Length();
		if ( flHeldSpeed <= SourceToJolt::Distance( 0.01f ) )
			return;

		const JPH::Vec3 vMoveDirection = ioLinearVelocity / flHeldSpeed;
		JPH::BodyID visitedBodies[kPhysgunClearanceMaxBodies];
		int nVisitedBodies = 1;
		visitedBodies[0] = pHeldObject->GetBodyID();

		JoltPhysicsObject *pMovingObject = pHeldObject;
		float flVelocityAmplification = 1.0f;
		bool bFirstCast = true;

		for ( int nDepth = 0; nDepth < kPhysgunClearanceMaxBodies - 1; ++nDepth )
		{
			const JPH::Vec3 vCurrentVelocity = pMovingObject == pHeldObject
				? ioLinearVelocity
				: pMovingObject->GetBody()->GetLinearVelocity();
			const float flCurrentForwardSpeed = Max( 0.0f, vCurrentVelocity.Dot( vMoveDirection ) );
			const float flPredictedForwardSpeed = bFirstCast
				? flHeldSpeed
				: flCurrentForwardSpeed + flHeldSpeed * flVelocityAmplification;
			const JPH::Vec3 vCastDisplacement = vMoveDirection * flPredictedForwardSpeed * flDeltaTime;

			JPH::ShapeCastResult hit;
			if ( !CastPhysgunClearanceShape( pMovingObject, vCastDisplacement,
				visitedBodies, nVisitedBodies, hit ) )
				return;

			JPH::BodyLockRead hitLock( pHeldObject->GetJoltEnvironment()->GetPhysicsSystem()->GetBodyLockInterface(), hit.mBodyID2 );
			if ( !hitLock.SucceededAndIsInBroadPhase() )
				return;

			const JPH::Body &hitBody = hitLock.GetBody();
			JoltPhysicsObject *pHitObject = reinterpret_cast<JoltPhysicsObject *>( hitBody.GetUserData() );
			if ( !pHitObject )
				return;

			const JPH::Vec3 vHitAxis = hit.mPenetrationAxis.NormalizedOr( vMoveDirection );
			const float flAxisAlignment = vMoveDirection.Dot( vHitAxis );
			if ( flAxisAlignment <= 1.0e-5f )
				return;

			const float flHitDistance = Max( 0.0f, hit.mFraction ) * vCastDisplacement.Length();
			const float flNormalClearance = Max( 0.0f,
				flHitDistance * flAxisAlignment - SourceToJolt::Distance( kPhysgunClearanceMarginInches ) );
			const bool bMovableHit = hitBody.IsDynamic() && pHitObject->IsMoveable()
				&& !( pHitObject->GetGameFlags() & FVPHYSICS_CONSTRAINT_STATIC );

			if ( !bMovableHit )
			{
				const float flExistingInwardSpeed = pMovingObject == pHeldObject
					? 0.0f
					: Max( 0.0f, vCurrentVelocity.Dot( vHitAxis ) );
				const float flMaximumHeldInwardSpeed = Max( 0.0f,
					flNormalClearance / flDeltaTime - flExistingInwardSpeed )
					/ Max( flVelocityAmplification, 1.0e-3f );
				const float flHeldInwardSpeed = ioLinearVelocity.Dot( vHitAxis );
				if ( flHeldInwardSpeed > flMaximumHeldInwardSpeed )
				{
					ioLinearVelocity -= vHitAxis * ( flHeldInwardSpeed - flMaximumHeldInwardSpeed );
					VJoltStats_RecordEvent( VJoltStatsEvent::PhysgunBlockedChainClamp );
				}
				return;
			}

			const float flMovingMass = Max( pMovingObject->GetMass(), 0.001f );
			const float flHitMass = Max( pHitObject->GetMass(), 0.001f );
			const float flMaximumTransfer = Min( 2.0f,
				2.0f * flMovingMass / ( flMovingMass + flHitMass ) );
			flVelocityAmplification *= flMaximumTransfer;

			visitedBodies[nVisitedBodies++] = hit.mBodyID2;
			pMovingObject = pHitObject;
			bFirstCast = false;
		}
	}
}

float JoltPhysicsObject::ComputeShadowControl( const hlshadowcontrol_params_t &params, float flSecondsToArrival, float flDeltaTime )
{
	// Direct game-side drive and the native grab motor must not both drive this body.
	VJoltShadowControl::Params joltParams =
	{
		.targetPosition		= SourceToJolt::Distance( params.targetPosition ),
		.targetRotation		= SourceToJolt::Angle( params.targetRotation ),
		.maxAngular			= SourceToJolt::Angle( params.maxAngular ),
		.maxDampAngular		= SourceToJolt::Angle( params.maxDampAngular ),
		.maxSpeed			= SourceToJolt::Distance( params.maxSpeed ),
		.maxDampSpeed		= SourceToJolt::Distance( params.maxDampSpeed ),
		.dampFactor			= params.dampFactor,
		.teleportDistance	= SourceToJolt::Distance( params.teleportDistance ),
	};

	JPH::BodyInterface& bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();

	JPH::Vec3 position;
	JPH::Quat rotation;
	bodyInterface.GetPositionAndRotation( m_pBody->GetID(), position, rotation );
	JPH::Vec3 linearVelocity;
	JPH::Vec3 angularVelocity;
	bodyInterface.GetLinearAndAngularVelocity( m_pBody->GetID(), linearVelocity, angularVelocity );

	JPH::Vec3 scratchPosition = position;
	JPH::Quat scratchRotation = rotation;
	JPH::Vec3 scratchLinearVelocity = linearVelocity;
	JPH::Vec3 scratchAngularVelocity = angularVelocity;
	const VJoltShadowControl::Result shadowResult = VJoltShadowControl::Compute(
		joltParams, scratchPosition, scratchRotation, scratchLinearVelocity,
		scratchAngularVelocity, flSecondsToArrival, flDeltaTime );

	if ( GetGameFlags() & FVPHYSICS_PLAYER_HELD )
	{
		// Movable contacts share the drive impulse through the contact solver.
		// Projecting it against cached penetration normals treats payloads as fixed
		// blockers and can erase the drive before either body receives a response.
		ClampPhysgunHeldVelocityAgainstBlockedChain( this, scratchLinearVelocity, flDeltaTime );
	}

	if ( scratchPosition != position || scratchRotation != rotation )
	{
		GetJoltEnvironment()->GetContactListener()->InvalidateFrictionContacts( GetBodyID() );
		bodyInterface.SetPositionAndRotation( m_pBody->GetID(), scratchPosition, scratchRotation, JPH::EActivation::Activate );
	}

	if ( scratchLinearVelocity != linearVelocity || scratchAngularVelocity != angularVelocity )
		bodyInterface.SetLinearAndAngularVelocity( m_pBody->GetID(), scratchLinearVelocity, scratchAngularVelocity );

	return shadowResult.secondsToArrival;
}

//-------------------------------------------------------------------------------------------------

const CPhysCollide *JoltPhysicsObject::GetCollide() const
{
	const CPhysCollide *pCollide = CPhysCollide::FromShape( m_pBody->GetShape() );
	return pCollide;
}

const char *JoltPhysicsObject::GetName() const
{
	return m_pName;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::BecomeTrigger()
{
	if ( m_pEnvironment->IsInContactCallback() && m_pEnvironment->DeferObjectMutation( this, [this]() { BecomeTrigger(); } ) ) return;
	if ( IsTrigger() )
		return;

	EnableDrag( false );
	EnableGravity( false );

	GetJoltEnvironment()->GetContactListener()->InvalidateFrictionContacts( GetBodyID() );
	m_pBody->SetIsSensor( true );
}

void JoltPhysicsObject::RemoveTrigger()
{
	if ( m_pEnvironment->IsInContactCallback() && m_pEnvironment->DeferObjectMutation( this, [this]() { RemoveTrigger(); } ) ) return;
	if ( !IsTrigger() )
		return;

	// Note: Does not change drag/gravity state back, to mimick Source 1 behaviour.

	// Josh:
	// All this logic below is to trigger ObjectLeaveTrigger
	// when the trigger is deleted.
	IPhysicsCollisionEvent *pEventListener = m_pEnvironment->GetContactListener()->GetGameListener();
	if ( pEventListener && IsTrigger() )
	{
		class SourceTriggerCollector : public JPH::CollideShapeCollector
		{
		public:
			SourceTriggerCollector( JPH::PhysicsSystem *pPhysicsSystem, IPhysicsCollisionEvent *pGameListener, JoltPhysicsObject *pTrigger )
				: m_pPhysicsSystem( pPhysicsSystem )
				, m_pGameListener ( pGameListener )
				, m_pTrigger      ( pTrigger ) {}

			void AddHit( const ResultType &inResult ) override
			{
				const JPH::BodyID inBodyID = inResult.mBodyID2;

				JPH::BodyLockWrite lock( m_pPhysicsSystem->GetBodyLockInterface(), inBodyID );
				JPH::Body &body = lock.GetBody();
				JoltPhysicsObject *pObject = reinterpret_cast<JoltPhysicsObject *>( body.GetUserData() );

				if ( !pObject )
					return;

				m_pGameListener->ObjectLeaveTrigger( m_pTrigger, pObject );
			}

		private:
			JPH::PhysicsSystem     *m_pPhysicsSystem;
			IPhysicsCollisionEvent *m_pGameListener;
			JoltPhysicsObject      *m_pTrigger;
		};

		SourceTriggerCollector collector( m_pPhysicsSystem, pEventListener, this );

		JPH::IgnoreSingleBodyFilter body_filter( GetBodyID() );

		JPH::CollideShapeSettings collideSettings;
		collideSettings.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideWithAll;

		JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
		JPH::Mat44 queryTransform = bodyInterface.GetCenterOfMassTransform( GetBodyID() );

		const JPH::Shape *pShape = GetCollide()->ToShape();

		m_pPhysicsSystem->GetNarrowPhaseQueryNoLock().CollideShape(
			pShape, JPH::Vec3::sReplicate( 1.0f ), queryTransform, collideSettings, JPH::Vec3::sZero(), collector,
			JPH::SpecifiedBroadPhaseLayerFilter( BroadPhaseLayers::MOVING ), JPH::SpecifiedObjectLayerFilter( Layers::MOVING ), body_filter );
	}

	GetJoltEnvironment()->GetContactListener()->InvalidateFrictionContacts( GetBodyID() );
	m_pBody->SetIsSensor( false );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::BecomeHinged( int localAxis )
{
	if ( m_pEnvironment->IsInContactCallback() && m_pEnvironment->DeferObjectMutation( this, [this, localAxis]() { BecomeHinged( localAxis ); } ) ) return;
	if ( IsMoveable() && localAxis >= 0 && localAxis < 3 )
	{
		if ( !m_bHasUnhingedInertia )
			CaptureUnhingedInertia();
		ApplyHingedInertia( localAxis );
	}
	m_iHingedAxis = localAxis;
}

void JoltPhysicsObject::RemoveHinged()
{
	if ( m_pEnvironment->IsInContactCallback() && m_pEnvironment->DeferObjectMutation( this, [this]() { RemoveHinged(); } ) ) return;
	if ( m_bHasUnhingedInertia && !IsStatic() )
		RestoreUnhingedInertia();
	m_iHingedAxis = -1;
	m_bHingedInertiaApplied = false;
	m_bHasUnhingedInertia = false;
}

void JoltPhysicsObject::CaptureUnhingedInertia()
{
	if ( IsStatic() )
		return;

	const JPH::MotionProperties *pMotion = m_pBody->GetMotionPropertiesUnchecked();
	m_vUnhingedInverseInertia = pMotion->GetInverseInertiaDiagonalUnchecked();
	m_qUnhingedInertiaRotation = pMotion->GetInertiaRotation();
	m_bHasUnhingedInertia = true;
}

void JoltPhysicsObject::RestoreUnhingedInertia()
{
	if ( !m_bHasUnhingedInertia || IsStatic() )
		return;

	// Source-frozen props are temporarily Static in Jolt but retain allocated
	// motion properties. Unpin restores these before UpdateLayer makes them Dynamic.
	JPH::MotionProperties *pMotion = m_pBody->GetMotionPropertiesUnchecked();
	pMotion->SetInverseInertia(
		m_vUnhingedInverseInertia, m_qUnhingedInertiaRotation );
	pMotion->SetInverseMass( m_flCachedInvMass );
	m_flCurrentInvMass = m_flCachedInvMass;
	m_bHingedInertiaApplied = false;
}

JPH::Vec3 JoltPhysicsObject::GetUnhingedLocalInverseInertiaDiagonal() const
{
	const JPH::Mat44 rotation = JPH::Mat44::sRotation( m_qUnhingedInertiaRotation );
	return JPH::Vec3(
		m_vUnhingedInverseInertia[ 0 ] * Square( rotation( 0, 0 ) )
			+ m_vUnhingedInverseInertia[ 1 ] * Square( rotation( 0, 1 ) )
			+ m_vUnhingedInverseInertia[ 2 ] * Square( rotation( 0, 2 ) ),
		m_vUnhingedInverseInertia[ 0 ] * Square( rotation( 1, 0 ) )
			+ m_vUnhingedInverseInertia[ 1 ] * Square( rotation( 1, 1 ) )
			+ m_vUnhingedInverseInertia[ 2 ] * Square( rotation( 1, 2 ) ),
		m_vUnhingedInverseInertia[ 0 ] * Square( rotation( 2, 0 ) )
			+ m_vUnhingedInverseInertia[ 1 ] * Square( rotation( 2, 1 ) )
			+ m_vUnhingedInverseInertia[ 2 ] * Square( rotation( 2, 2 ) ) );
}

void JoltPhysicsObject::ApplyHingedInertia( int localAxis )
{
	if ( !m_bHasUnhingedInertia || localAxis < 0 || localAxis >= 3 || IsStatic() )
		return;

	JPH::MotionProperties *pMotion = m_pBody->GetMotionProperties();
	const JPH::Vec3 currentLocalInverse =
		pMotion->GetLocalSpaceInverseInertiaUnchecked().GetDiagonal3();
	JPH::Vec3 hingedInverse = GetUnhingedLocalInverseInertiaDiagonal()
		* Clamp( m_flCachedMass / VPHYSICS_MAX_MASS, 0.0f, 1.0f );
	hingedInverse.SetComponent( localAxis, currentLocalInverse[ localAxis ] );
	pMotion->SetInverseInertia( hingedInverse, JPH::Quat::sIdentity() );
	pMotion->SetInverseMass( 1.0f / VPHYSICS_MAX_MASS );
	m_flCurrentInvMass = 1.0f / VPHYSICS_MAX_MASS;
	m_bHingedInertiaApplied = true;
}

//-------------------------------------------------------------------------------------------------

IPhysicsFrictionSnapshot *JoltPhysicsObject::CreateFrictionSnapshot()
{
	return new JoltPhysicsFrictionSnapshot( this );
}

void JoltPhysicsObject::DestroyFrictionSnapshot( IPhysicsFrictionSnapshot *pSnapshot )
{
	delete pSnapshot;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::OutputDebugInfo() const
{
	Log_Stub( LOG_VJolt );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::SetUseAlternateGravity( bool bSet )
{
	Log_Stub( LOG_VJolt );
}

void JoltPhysicsObject::SetCollisionHints( uint32 collisionHints )
{
	if ( m_pEnvironment->IsInContactCallback() && m_pEnvironment->DeferObjectMutation( this, [this, collisionHints]() { SetCollisionHints( collisionHints ); } ) ) return;
	m_collisionHints = collisionHints;
	UpdateLayer();
}

uint32 JoltPhysicsObject::GetCollisionHints() const
{
	return m_collisionHints;
}

//-------------------------------------------------------------------------------------------------

IPredictedPhysicsObject *JoltPhysicsObject::GetPredictedInterface() const
{
	Log_Stub( LOG_VJolt );
	return nullptr;
}

void JoltPhysicsObject::SyncWith( IPhysicsObject *pOther )
{
	if ( this->IsCollisionEnabled() != pOther->IsCollisionEnabled() )
		EnableCollisions( pOther->IsCollisionEnabled() );

	if ( this->IsGravityEnabled() != pOther->IsGravityEnabled() )
		EnableGravity( pOther->IsGravityEnabled() );

	if ( this->IsDragEnabled() != pOther->IsDragEnabled() )
		EnableDrag(pOther->IsDragEnabled() );

	if ( this->IsMotionEnabled() != pOther->IsMotionEnabled() )
		EnableMotion(pOther->IsMotionEnabled() );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::UpdateEnvironment( JoltPhysicsEnvironment *pEnvironment )
{
	const bool hadPendingVelocity = m_bPendingVelocityQueued;
	if ( hadPendingVelocity )
	{
		m_pEnvironment->RemovePendingVelocityCommit( this );
		m_bPendingVelocityQueued = false;
	}

	m_pEnvironment = pEnvironment;
	m_pPhysicsSystem = pEnvironment->GetPhysicsSystem();

	if ( hadPendingVelocity )
		QueuePendingVelocityCommit();
}

void JoltPhysicsObject::ReplaceBodyForTransfer( JPH::Body *pBody, JoltPhysicsEnvironment *pEnvironment )
{
	Assert( pBody );
	m_pBody = pBody;
	m_pBody->SetUserData( reinterpret_cast< uint64 >( this ) );
	UpdateEnvironment( pEnvironment );
}

void JoltPhysicsObject::AddDestroyedListener( IJoltObjectDestroyedListener *pListener )
{
	m_destroyedListeners.AddToTail( pListener );
}

void JoltPhysicsObject::RemoveDestroyedListener( IJoltObjectDestroyedListener *pListener )
{
	m_destroyedListeners.FindAndRemove( pListener );
}

void JoltPhysicsObject::AddToPosition( JPH::Vec3Arg addPos )
{
	if ( m_pEnvironment->IsInContactCallback() && m_pEnvironment->DeferObjectMutation( this, [this, addPos]() { AddToPosition( addPos ); } ) ) return;
	GetJoltEnvironment()->GetContactListener()->InvalidateFrictionContacts( GetBodyID() );
	const JPH::BodyLockInterfaceNoLock &bodyLockInterface = m_pPhysicsSystem->GetBodyLockInterfaceNoLock();

	JPH::BodyLockWrite lock( bodyLockInterface, m_pBody->GetID() );
	if ( lock.Succeeded() )
	{
		JPH::Body &body = lock.GetBody();

		JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
		bodyInterface.SetPosition( m_pBody->GetID(), body.GetPosition() + addPos, JPH::EActivation::DontActivate );
	}
}

void JoltPhysicsObject::SetPosition( const Vector &worldPosition )
{
	if ( m_pEnvironment->IsInContactCallback() && m_pEnvironment->DeferObjectMutation( this, [this, worldPosition]() { SetPosition( worldPosition ); } ) ) return;
	JPH::Vec3 joltPosition = SourceToJolt::Distance( worldPosition );

	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();

	GetJoltEnvironment()->GetContactListener()->InvalidateFrictionContacts( GetBodyID() );
	bodyInterface.SetPosition( m_pBody->GetID(), joltPosition, JPH::EActivation::DontActivate );
}

void JoltPhysicsObject::AddVelocity( const Vector &worldPosition )
{
	if ( m_pEnvironment->IsInContactCallback() && m_pEnvironment->DeferObjectMutation( this, [this, worldPosition]() { AddVelocity( worldPosition ); }, true ) ) return;
	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
	bodyInterface.AddLinearVelocity( m_pBody->GetID(), SourceToJolt::Distance( worldPosition ) );
}

Vector JoltPhysicsObject::GetVelocity()
{
	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
	return JoltToSource::Distance( bodyInterface.GetLinearVelocity( m_pBody->GetID() ) );
}

void JoltPhysicsObject::CalculateBuoyancy()
{
	if ( m_flVolume != 0.0f )
	{
		float flVolume = SourceToJolt::Volume( Max( m_flVolume, 5.0f ) );
		float flDensity = m_flCachedMass / flVolume;
		m_flBuoyancyRatio = flDensity / m_flMaterialDensity;
	}
	else
	{
		m_flBuoyancyRatio = 1.0f;
	}
}

float JoltPhysicsObject::GetMaterialDensity() const
{
	return m_flMaterialDensity;
}

float JoltPhysicsObject::GetBuoyancyRatio() const
{
	return m_flBuoyancyRatio; 
}

bool JoltPhysicsObject::IsControlledByGame() const
{
	if ( m_pShadowController && !m_pShadowController->IsPhysicallyControlled() )
		return true;

	if ( m_callbackFlags & CALLBACK_IS_PLAYER_CONTROLLER )
		return true;

	return false;
}

void JoltPhysicsObject::SaveObjectState( JPH::StateRecorder &recorder, bool extended )
{
	m_pBody->SaveState( recorder );

	// Josh: Do not write m_pGameData, as this is passed in, in UnserializeObjectFromBuffer.
	//recorder.Write( m_pGameData );
	recorder.Write( m_gameFlags );
	recorder.Write( m_gameIndex );
	recorder.Write( m_callbackFlags );
	recorder.Write( m_bStatic );
	recorder.Write( m_bPinned );
	recorder.Write( m_iHingedAxis );
	bool hasUnhingedInertia = m_bHasUnhingedInertia;
	JPH::Vec3 unhingedInverseInertia = m_vUnhingedInverseInertia;
	JPH::Quat unhingedInertiaRotation = m_qUnhingedInertiaRotation;
	if ( ( IsHinged() || m_bPinned ) && !hasUnhingedInertia && !IsStatic() )
	{
		const JPH::MotionProperties *pMotion = m_pBody->GetMotionProperties();
		unhingedInverseInertia = pMotion->GetInverseInertiaDiagonal();
		unhingedInertiaRotation = pMotion->GetInertiaRotation();
		hasUnhingedInertia = true;
	}
	recorder.Write( hasUnhingedInertia );
	recorder.Write( unhingedInverseInertia );
	recorder.Write( unhingedInertiaRotation );
	recorder.Write( m_materialIndex );
	recorder.Write( m_contents );
	recorder.Write( m_flCachedMass );
	recorder.Write( m_flCachedInvMass );
	recorder.Write( m_flCurrentInvMass );
	recorder.Write( m_bCachedCollisionEnabled );
	recorder.Write( m_flMaterialDensity );
	recorder.Write( m_flBuoyancyRatio );
	recorder.Write( m_flVolume );
	recorder.Write( m_GameMaterial );

	if ( extended )
	{
		recorder.Write( m_collisionHints );
		recorder.Write( m_bSphereObject );
		recorder.Write( m_flSphereRadius );
		recorder.Write( m_bDragEnabled );
		recorder.Write( m_flLinearDragCoefficient );
		recorder.Write( m_flAngularDragCoefficient );
		recorder.Write( m_vDragBasis );
		recorder.Write( m_vAngDragBasis );
		recorder.Write( m_vPendingLinearVelocity );
		recorder.Write( m_vPendingAngularVelocityLocal );
		recorder.Write( m_bShadowTemporarilyDisableGravity );
	}
}

void JoltPhysicsObject::RestoreObjectState( JPH::StateRecorder &recorder, bool extended )
{
	// Restore the body's state.
	m_pBody->RestoreState( recorder );

	// Set the body's userdata as ourselves
	m_pBody->SetUserData( reinterpret_cast<uint64>( this ) );

	// Josh: Do not read m_pGameData, as this is not serialized.
	//recorder.Read( m_pGameData );
	recorder.Read( m_gameFlags );
	recorder.Read( m_gameIndex );
	recorder.Read( m_callbackFlags );
	recorder.Read( m_bStatic );
	recorder.Read( m_bPinned );
	int hingedAxis = -1;
	recorder.Read( hingedAxis );
	recorder.Read( m_bHasUnhingedInertia );
	recorder.Read( m_vUnhingedInverseInertia );
	recorder.Read( m_qUnhingedInertiaRotation );
	recorder.Read( m_materialIndex );
	recorder.Read( m_contents );
	recorder.Read( m_flCachedMass );
	recorder.Read( m_flCachedInvMass );
	recorder.Read( m_flCurrentInvMass );
	recorder.Read( m_bCachedCollisionEnabled );
	recorder.Read( m_flMaterialDensity );
	recorder.Read( m_flBuoyancyRatio );
	recorder.Read( m_flVolume );
	recorder.Read( m_GameMaterial );
	bool dragEnabled = false;
	if ( extended )
	{
		recorder.Read( m_collisionHints );
		recorder.Read( m_bSphereObject );
		recorder.Read( m_flSphereRadius );
		recorder.Read( dragEnabled );
		recorder.Read( m_flLinearDragCoefficient );
		recorder.Read( m_flAngularDragCoefficient );
		recorder.Read( m_vDragBasis );
		recorder.Read( m_vAngDragBasis );
		recorder.Read( m_vPendingLinearVelocity );
		recorder.Read( m_vPendingAngularVelocityLocal );
		recorder.Read( m_bShadowTemporarilyDisableGravity );
	}
	if ( VJoltStateRecorderFailed( recorder ) )
		return;

	// Recompute states.
	UpdateMaterialProperties();
	UpdateLayer();
	if ( m_bPinned && m_flCurrentInvMass == 0.0f && !IsStatic() )
	{
		// Jolt's binary state restore normalizes a zero inverse-inertia diagonal
		// to FLT_MIN. IVP keeps an ordinarily pinned core at exact zero, both for
		// the public getters and for later save/transfer operations.
		JPH::MotionProperties *pMotion = m_pBody->GetMotionPropertiesUnchecked();
		pMotion->SetInverseInertia( JPH::Vec3::sZero(), JPH::Quat::sIdentity() );
		pMotion->SetInverseMass( 0.0f );
	}
	if ( hingedAxis >= 0 )
	{
		m_iHingedAxis = hingedAxis;
		if ( m_bHasUnhingedInertia && !IsStatic() && !m_bPinned )
			RestoreUnhingedInertia();
		if ( IsMoveable() )
			BecomeHinged( hingedAxis );
	}
	if ( !IsStatic() )
	{
		// Keep the saved drag bases: a frozen body's live inverse inertia is zero, so
		// recomputing from it would lose angular drag when the game later unfreezes it.
		EnableDrag( dragEnabled );
		// Rebase interpolation bookkeeping to the restored pose, not the origin.
		GetVelocity( &m_vLastVelocity, &m_vLastAngularVelocity );
		GetPosition( &m_vLastPosition, &m_qLastOrientation );
		if ( m_vPendingLinearVelocity != JPH::Vec3::sZero()
			|| m_vPendingAngularVelocityLocal != JPH::Vec3::sZero() )
			QueuePendingVelocityCommit();
	}
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::PostSimulation( float flTimestep )
{
	Vector vCurrentPos, vCurrentVel;
	AngularImpulse vAngularImpulse;
	QAngle qCurrentOrientation;

	GetPosition( &vCurrentPos, &qCurrentOrientation );
	GetVelocity( &vCurrentVel, &vAngularImpulse );

	m_vLastVelocity = ( vCurrentPos - m_vLastPosition ) / flTimestep;
	m_vLastPosition = vCurrentPos;

	Vector vGlobalAngleVelocity;
	QAngleToAngularImpulse( ( qCurrentOrientation - m_qLastOrientation ) / flTimestep, vGlobalAngleVelocity );

	m_vLastAngularVelocity = JoltToSource::Unitless( m_pBody->GetWorldTransform().Multiply3x3Transposed( SourceToJolt::Unitless( vGlobalAngleVelocity ) ) );

	m_qLastOrientation = qCurrentOrientation;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsObject::UpdateMaterialProperties()
{
	const surfacedata_t *pSurface = JoltPhysicsSurfaceProps::GetInstance().GetSurfaceData( m_materialIndex );

	m_pBody->SetRestitution( pSurface->physics.elasticity );
	m_pBody->SetFriction( pSurface->physics.friction );
	m_flMaterialDensity = pSurface->physics.density;
	m_GameMaterial = pSurface->game.material;
	CalculateBuoyancy();
}

void JoltPhysicsObject::UpdateLayer()
{
	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
	GetJoltEnvironment()->GetContactListener()->InvalidateFrictionContacts( GetBodyID() );

	const bool bCollisionsEnabled = m_bCachedCollisionEnabled;
	const bool bStatic = IsStatic();
	const bool bPinned = m_bPinned;
	const bool bDebris = m_collisionHints & COLLISION_HINT_DEBRIS;
	const bool bStaticSolid = m_collisionHints & COLLISION_HINT_STATICSOLID;

	// Update motion type if not made as a complete solid.
	if ( !bStatic && !IsControlledByGame() )
	{
		bool bStaticMotionType = bStaticSolid || bPinned;

		// If we are transfering to being static, and we were active
		// add us to a list of bodies on the environment so we can be included in
		// GetActiveObjects for the next step.
		// This way the game can correctly update the transforms on the game side
		// when move -> wake -> become pinned happens.
		if ( bStaticMotionType && m_pBody->IsActive() )
			m_pEnvironment->AddDirtyStaticBody( m_pBody->GetID() );
		else if ( !bStaticMotionType )
			m_pEnvironment->RemoveDirtyStaticBody( m_pBody->GetID() );

		bodyInterface.SetMotionType( m_pBody->GetID(), bStaticMotionType ? JPH::EMotionType::Static : JPH::EMotionType::Dynamic, JPH::EActivation::Activate );
		VJoltGravityRefreshObject( this );
	}

	// Update layer
	uint8 layer = Layers::MOVING;

	if ( bDebris )
		layer = Layers::DEBRIS;

	if ( bStatic || bStaticSolid )
		layer = Layers::NON_MOVING_WORLD;
	else if ( bPinned )
		layer = Layers::NON_MOVING_OBJECT;

	if ( !bCollisionsEnabled )
		layer = Layers::NO_COLLIDE;

	bodyInterface.SetObjectLayer( m_pBody->GetID(), layer );
}

// Integral of each differential drag area's torque over the OBB along one face pair.
// l, w, h are HALF widths -- l along the rotation axis, w perpendicular in the face
// plane, h perpendicular to the face. Sum two such integrals (one per face pair) to
// get the basis for one rotation axis.
static float AngDragIntegral( float flInvInertia, float l, float w, float h )
{
	const float w2 = w * w;
	const float l2 = l * l;
	const float h2 = h * h;

	return flInvInertia * ( ( 1.0f / 3.0f ) * w2 * l * l2 + 0.5f * w2 * w2 * l + l * w2 * h2 );
}

void JoltPhysicsObject::RecomputeDrag()
{
	if ( IsStatic() || !GetCollide() )
		return;

	// AABB in Source space; convert delta to Jolt space (m). Area fractions are unitless,
	// in [0,1], populated from compactsurfaceheader_t::dragAxisAreas at .phy load time.
	// CollideGetOrthographicAreas guarantees (1,1,1) when no precomputed value exists.
	Vector vDragMins, vDragMaxs;
	JoltPhysicsCollision::GetInstance().CollideGetAABB( &vDragMins, &vDragMaxs, GetCollide(), vec3_origin, vec3_angle );

	const Vector vAreaFractions = JoltPhysicsCollision::GetInstance().CollideGetOrthographicAreas( GetCollide() );

	JPH::Vec3 vDelta = SourceToJolt::Distance( vDragMaxs - vDragMins ).Abs();

	const float flInvMass = m_flCachedInvMass;

	// Linear basis: each component is the area of the face perpendicular to that axis,
	// scaled by inverse mass. Units: m^2/kg.
	m_vDragBasis = JPH::Vec3(
		vDelta.GetY() * vDelta.GetZ() * vAreaFractions.x,
		vDelta.GetX() * vDelta.GetZ() * vAreaFractions.y,
		vDelta.GetX() * vDelta.GetY() * vAreaFractions.z ) * flInvMass;

	// Angular basis needs HALF widths.
	vDelta *= 0.5f;
	const JPH::Vec3 vInvInertia = m_pBody->GetMotionPropertiesUnchecked()->GetInverseInertiaDiagonalUnchecked();

	m_vAngDragBasis = JPH::Vec3(
		vAreaFractions.z * AngDragIntegral( vInvInertia.GetX(), vDelta.GetX(), vDelta.GetY(), vDelta.GetZ() )
			+ vAreaFractions.y * AngDragIntegral( vInvInertia.GetX(), vDelta.GetX(), vDelta.GetZ(), vDelta.GetY() ),
		vAreaFractions.z * AngDragIntegral( vInvInertia.GetY(), vDelta.GetY(), vDelta.GetX(), vDelta.GetZ() )
			+ vAreaFractions.x * AngDragIntegral( vInvInertia.GetY(), vDelta.GetY(), vDelta.GetZ(), vDelta.GetX() ),
		vAreaFractions.y * AngDragIntegral( vInvInertia.GetZ(), vDelta.GetZ(), vDelta.GetX(), vDelta.GetY() )
			+ vAreaFractions.x * AngDragIntegral( vInvInertia.GetZ(), vDelta.GetZ(), vDelta.GetY(), vDelta.GetX() ) );
}

float JoltPhysicsObject::GetDragInDirection( JPH::Vec3Arg vWorldVelocity ) const
{
	// Transform the world-space velocity into the body's local frame, then dot with
	// the local-space drag basis. Mirrors IVP's m_world_f_core->vimult3.
	const JPH::Vec3 vLocal = m_pBody->GetRotation().Conjugated() * vWorldVelocity;

	// NB: operator-precedence matches IVP/Source -- the linear coefficient applies
	// only to the X term. Preserved verbatim to match upstream drag intensity.
	return m_flLinearDragCoefficient * fabsf( vLocal.GetX() * m_vDragBasis.GetX() )
		+ fabsf( vLocal.GetY() * m_vDragBasis.GetY() )
		+ fabsf( vLocal.GetZ() * m_vDragBasis.GetZ() );
}

float JoltPhysicsObject::GetAngularDragInDirection( JPH::Vec3Arg vLocalAngularVelocity ) const
{
	return m_flAngularDragCoefficient * fabsf( vLocalAngularVelocity.GetX() * m_vAngDragBasis.GetX() )
		+ fabsf( vLocalAngularVelocity.GetY() * m_vAngDragBasis.GetY() )
		+ fabsf( vLocalAngularVelocity.GetZ() * m_vAngDragBasis.GetZ() );
}

#if GAME_GMOD
IPhysicsEnvironment *JoltPhysicsObject::GetEnvironment()
{
	return m_pEnvironment;
}
#endif
