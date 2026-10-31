
#include "cbase.h"

#include "vjolt_layers.h"

#include "vjolt_controller_player.h"
#include "vjolt_friction.h"
#include "vjolt_player_contact_planes.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

static ConVar vjolt_player_character_padding( "vjolt_player_character_padding", "0.02" );

static ConVar vjolt_player_disable_limit( "vjolt_player_disable_limit", "0.1", 0, "The min speed^2 before we just go where physics wants to take us" );

// Source's FVPHYSICS_PENETRATING game flag. Keep the adapter-side name local;
// the public GMod VPhysics headers expose the bit only to game code.
static constexpr uint16 kFVPhysicsPenetrating = 0x0040;
static constexpr uint16 kFVPhysicsConstraintStatic = 0x0002;

//-------------------------------------------------------------------------------------------------

// Component-wise Vector clamp
static Vector ClampVector( const Vector &x, const Vector &min, const Vector &max )
{
	return Vector(
		Clamp( x.x, min.x, max.x ),
		Clamp( x.y, min.y, max.y ),
		Clamp( x.z, min.z, max.z ) );
}

static void ComputePlayerController( Vector &vCurrentSpeed, const Vector &vDelta, const Vector &vMaxSpeed, float flScaleDelta, float flDamping, Vector *pOutImpulse )
{
	if ( vCurrentSpeed.LengthSqr() < 1e-6f )
	{
		vCurrentSpeed = vec3_origin;
	}

	Vector vDampAccel = vCurrentSpeed * -flDamping;
	Vector vDeltaAccel = vDelta * flScaleDelta;

	Vector vAcceleration = ClampVector( vDeltaAccel + vDampAccel, -vMaxSpeed, vMaxSpeed );

	vCurrentSpeed += vAcceleration;
	if ( pOutImpulse )
		*pOutImpulse = vAcceleration;
}

//-------------------------------------------------------------------------------------------------

JoltPhysicsPlayerController::JoltPhysicsPlayerController( JoltPhysicsObject *pObject )
{
	SetObjectInternal( pObject );
}

JoltPhysicsPlayerController::~JoltPhysicsPlayerController()
{
	SetGroundInternal( nullptr );
	SetObjectInternal( nullptr );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsPlayerController::Update( const Vector &position, const Vector &velocity, float secondsToArrival, bool onground, IPhysicsObject *ground )
{
	m_bUpdatedSinceLast = true;

	const bool bHasInputVelocity = velocity.LengthSqr() > vjolt_player_disable_limit.GetFloat();
	if ( !bHasInputVelocity )
		ground = nullptr;
	const float flSecondsToArrival = Max( secondsToArrival, 0.0f );
	// Equal world-space input does not mean equal control state: switching or
	// leaving a moving support changes both the local anchor and relative speed.
	if ( ( velocity - m_vCurrentSpeed ).LengthSqr() < 1e-6f && ( position - m_vTargetPosition ).LengthSqr() < 1e-6f
		&& ground == m_pGround && flSecondsToArrival == m_flSecondsToArrival )
		return;

	m_vTargetPosition = position;
	m_flSecondsToArrival = flSecondsToArrival;

	m_vCurrentSpeed = velocity;
	m_pObject->Wake();
	m_bEnable = true;

	if ( !bHasInputVelocity )
	{
		// No input velocity, just go where physics takes you.
		m_bEnable = false;
		ground = nullptr;
	}
	else
	{
		MaxSpeed( velocity );
	}

	SetGroundInternal( static_cast<JoltPhysicsObject *>( ground ) );
	if ( m_pGround )
		m_pGround->WorldToLocal( &m_vGroundPosition, m_vTargetPosition );
}

void JoltPhysicsPlayerController::SetEventHandler( IPhysicsPlayerControllerEvent *handler )
{
	m_pHandler = handler;
}

bool JoltPhysicsPlayerController::IsInContact()
{
	uint32 nState = GetContactState( 0 );
	return !!( nState & PLAYER_CONTACT_PHYSICS );
}

void JoltPhysicsPlayerController::MaxSpeed( const Vector &velocity )
{
	// IVP evaluates this nonlinear expression in metres; converting only its
	// result would amplify the product of speeds by Source's distance scale.
	const JPH::Vec3 targetVelocity = SourceToJolt::Distance( velocity );
	const float flLength = targetVelocity.Length();
	const JPH::Vec3 direction = targetVelocity.NormalizedOr( JPH::Vec3::sZero() );
	JPH::Vec3 available = targetVelocity;
	const float flDot = direction.Dot( m_pObject->GetBody()->GetLinearVelocity() );
	if ( flDot > 0 )
		available -= direction * ( flDot * flLength );
	m_vMaxSpeed = JoltToSource::Distance( available.Abs() );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsPlayerController::SetObject( IPhysicsObject *pObject )
{
	SetObjectInternal( static_cast<JoltPhysicsObject *>( pObject ) );
}

//-------------------------------------------------------------------------------------------------

int JoltPhysicsPlayerController::GetShadowPosition( Vector *position, QAngle *angles )
{
	return m_pObject->GetShadowPosition( position, angles );
}

void JoltPhysicsPlayerController::StepUp( float height )
{
	if ( height == 0.0f )
		return;

	Vector vPos;
	QAngle qAngles;
	m_pObject->GetPosition( &vPos, &qAngles );
	vPos.z += height;
	// Teleport, do not influence implicit velocity.
	m_pObject->SetPosition( vPos, qAngles, true );
}

void JoltPhysicsPlayerController::Jump()
{
	// This does nothing in VPhysics.
}

void JoltPhysicsPlayerController::GetShadowVelocity( Vector *velocity )
{
	if ( !velocity )
		return;

	m_pObject->GetVelocity( velocity, nullptr );

	*velocity -= GetGroundVelocity();
}

IPhysicsObject *JoltPhysicsPlayerController::GetObject()
{
	return m_pObject;
}

void JoltPhysicsPlayerController::GetLastImpulse( Vector *pOut )
{
	if ( pOut )
		*pOut = m_vLastImpulse;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsPlayerController::SetPushMassLimit( float maxPushMass )
{
	m_flPushableMassLimit = maxPushMass;
	if ( m_pObject )
		m_pObject->SetPlayerPushMassLimitForContacts( maxPushMass );
}

void JoltPhysicsPlayerController::SetPushSpeedLimit( float maxPushSpeed )
{
	m_flPushableSpeedLimit = maxPushSpeed;
}

//-------------------------------------------------------------------------------------------------

float JoltPhysicsPlayerController::GetPushMassLimit()
{
	return m_flPushableMassLimit;
}

float JoltPhysicsPlayerController::GetPushSpeedLimit()
{
	return m_flPushableSpeedLimit;
}

//-------------------------------------------------------------------------------------------------

bool JoltPhysicsPlayerController::WasFrozen()
{
	return m_pObject && m_pObject->IsAsleep();
}

//-------------------------------------------------------------------------------------------------

static void CheckCollision( JoltPhysicsObject *pObject, JPH::CollideShapeCollector &ioCollector, JPH::BodyFilter &ioFilter )
{
	if ( !pObject->IsCollisionEnabled() ) 
	    return;
	
	JPH::PhysicsSystem *pSystem = pObject->GetJoltEnvironment()->GetPhysicsSystem();

	if ( !pObject->IsCollisionEnabled() )
		return;

	// Create query broadphase layer filter
	JPH::DefaultBroadPhaseLayerFilter broadphase_layer_filter = pSystem->GetDefaultBroadPhaseLayerFilter( Layers::MOVING );

	// Create query object layer filter
	JPH::DefaultObjectLayerFilter object_layer_filter = pSystem->GetDefaultLayerFilter( Layers::MOVING );

	// Determine position to test
	JPH::Vec3 position;
	JPH::Quat rotation;
	JPH::BodyInterface &bi = pSystem->GetBodyInterfaceNoLock();
	bi.GetPositionAndRotation( pObject->GetBodyID(), position, rotation );
	JPH::Mat44 query_transform = JPH::Mat44::sRotationTranslation( rotation, position + rotation * pObject->GetBody()->GetShape()->GetCenterOfMass() );

	// Settings for collide shape
	JPH::CollideShapeSettings settings;
	settings.mActiveEdgeMode = JPH::EActiveEdgeMode::CollideOnlyWithActive;
	settings.mActiveEdgeMovementDirection = bi.GetLinearVelocity( pObject->GetBodyID() );
	settings.mBackFaceMode = JPH::EBackFaceMode::IgnoreBackFaces;
	settings.mMaxSeparationDistance = vjolt_player_character_padding.GetFloat();

	pSystem->GetNarrowPhaseQueryNoLock().CollideShape( pObject->GetBody()->GetShape(), JPH::Vec3::sReplicate( 1.0f ), query_transform, settings, JPH::Vec3::sZero(), ioCollector, broadphase_layer_filter, object_layer_filter, ioFilter );
}


template <bool MoveablesOnly>
class SourceHitFilter : public JPH::BodyFilter
{
public:
	SourceHitFilter( JPH::PhysicsSystem* pPhysicsSystem, JoltPhysicsObject* pSelfObject )
		: m_pPhysicsSystem( pPhysicsSystem )
		, m_pSelfObject( pSelfObject )
	{
	}

	bool ShouldCollideLocked( const JPH::Body &inBody ) const override
	{
		JoltPhysicsObject* pObject = reinterpret_cast<JoltPhysicsObject*>( inBody.GetUserData() );
		if ( !inBody.IsRigidBody() )
		{
			return false;
		}
		// Independent native NPC capsules are solid dynamic bodies without an
		// IPhysicsObject. Player contact queries must still see their geometry.
		if ( !pObject ) return !inBody.IsSensor() && ( !MoveablesOnly || inBody.IsDynamic() );

		// Ignore self if specified. This can be nullptr if you don't want this.
		if ( pObject == m_pSelfObject )
			return false;

		if constexpr ( MoveablesOnly )
		{
			if ( pObject->IsTrigger() || !pObject->IsMoveable() )
				return false;
		}

		if ( !pObject->GetJoltEnvironment()->GetContactListener()->ShouldCollide( m_pSelfObject, pObject ) )
			return false;

		return true;
	}

private:
	JPH::PhysicsSystem	*m_pPhysicsSystem;
	JoltPhysicsObject	*m_pSelfObject;
};

uint32 JoltPhysicsPlayerController::GetContactState( uint16 nGameFlags )
{
	if ( !m_pObject->IsCollisionEnabled() )
		return 0;

	// Collector that finds the hit with the normal that is the most 'up'
	class ContactStateCollector : public JPH::CollideShapeCollector
	{
	public:
		ContactStateCollector( JPH::PhysicsSystem *pPhysicsSystem, JoltPhysicsObject *pPlayerObject, uint16 nGameFlags )
			: m_pPhysicsSystem( pPhysicsSystem )
			, m_pPlayerObject( pPlayerObject )
			, m_nGameFlags( nGameFlags )
		{
		}

		void Reset() override
		{
			JPH::CollideShapeCollector::Reset();

			m_nFlagsOut = 0;
		}

		void AddHit( const JPH::CollideShapeResult &inResult ) override
		{
			JPH::BodyLockRead lock( m_pPhysicsSystem->GetBodyLockInterfaceNoLock(), inResult.mBodyID2 );
			const JPH::Body &body = lock.GetBody();

			JoltPhysicsObject *pObject = reinterpret_cast<JoltPhysicsObject *>( body.GetUserData() );
			if ( !pObject ) { m_nFlagsOut |= PLAYER_CONTACT_PHYSICS; return; }

			if ( !pObject->IsControlledByGame() )
				m_nFlagsOut |= PLAYER_CONTACT_PHYSICS;

			if ( pObject->GetGameFlags() & m_nGameFlags )
				m_nFlagsOut |= PLAYER_CONTACT_GAMEOBJECT;
		}

		uint32					m_nFlagsOut = 0;

	private:
		JPH::PhysicsSystem		*m_pPhysicsSystem;
		JoltPhysicsObject		*m_pPlayerObject;
		uint16					m_nGameFlags;
	};

	JPH::PhysicsSystem *pSystem = m_pObject->GetJoltEnvironment()->GetPhysicsSystem();
	ContactStateCollector collector( pSystem, m_pObject, nGameFlags );
	SourceHitFilter<true> filter( pSystem, m_pObject );
	CheckCollision( m_pObject, collector, filter );

	return collector.m_nFlagsOut;
}

//-------------------------------------------------------------------------------------------------

int JoltPhysicsPlayerController::TryTeleportObject()
{
	if ( m_pHandler )
	{
		if ( !m_pHandler->ShouldMoveTo( m_pObject, m_vTargetPosition ) )
			return 0;
	}

	QAngle qCurrentAngles;
	m_pObject->GetPosition( nullptr, &qCurrentAngles );
	m_pObject->SetPosition( m_vTargetPosition, qCurrentAngles, true );
	return 1;
}

void JoltPhysicsPlayerController::OnPreSimulate( float flDeltaTime )
{
	if ( !m_pObject )
		return;

	Vector vOldPosition;
	Vector vOldVelocity;
	m_pObject->GetPosition( &vOldPosition, nullptr );
	m_pObject->GetVelocity( &vOldVelocity, nullptr );

	Vector vGroundVelocity = vec3_origin;
	if ( m_pGround )
	{
		m_pGround->LocalToWorld( &m_vTargetPosition, m_vGroundPosition );
		vGroundVelocity = GetGroundVelocity();
	}

	Vector vDeltaPos = m_vTargetPosition - vOldPosition;

	if ( m_bEnable )
	{
		// Totally bogus! Measure error using last known estimate not current position.
		if ( vDeltaPos.LengthSqr() > JPH::Square( m_flMaxDeltaPosition ) )
		{
			if ( TryTeleportObject() )
				return;
		}
	}

	float flFraction = Min( m_flSecondsToArrival > 0.0f ? flDeltaTime / m_flSecondsToArrival : 1.0f, 1.0f );

	if ( m_bEnable )
	{
		Vector vControllerVelocity = vOldVelocity;

		vControllerVelocity -= vGroundVelocity;
		if ( !m_bUpdatedSinceLast )
		{
			float flLen = m_vLastImpulse.Length();
			Vector vTempMaxSpeed = Vector( flLen, flLen, flLen );
			ComputePlayerController( vControllerVelocity, vDeltaPos, vTempMaxSpeed, flFraction / flDeltaTime, m_flDampFactor, nullptr );
		}
		else
		{
			ComputePlayerController( vControllerVelocity, vDeltaPos, m_vMaxSpeed, flFraction / flDeltaTime, m_flDampFactor, &m_vLastImpulse );
		}
		vControllerVelocity += vGroundVelocity;

		// Match Havana's final player-contact pass. The controller first computes its
		// requested velocity, then friction contacts clamp motion into immovable or
		// over-mass-limit objects. This is deliberately based on the previously
		// published contact frame, just like IVP's live friction-synapse list at PSI start.
		bool bGroundContact = false;
		float flLimitVelocity = m_flPushableSpeedLimit;
		Vector vGravity;
		m_pObject->GetJoltEnvironment()->GetGravity( &vGravity );
		PlayerContactNormalList contactNormals;
		IPhysicsFrictionSnapshot *pSnapshot = m_pObject->CreateFrictionSnapshot();
		if ( pSnapshot )
		{
			while ( pSnapshot->IsValid() )
			{
				Vector normal;
				pSnapshot->GetSurfaceNormal( normal );
				if ( normal.z < -0.7f )
					bGroundContact = true;

				// Preserve Source's flat-ground exception. Walls, ceilings and slopes
				// participate in the push-speed clamp.
				if ( normal.z > -0.99f )
				{
					IPhysicsObject *pOther = pSnapshot->GetObject( 1 );
					if ( pOther )
					{
						if ( !pOther->IsMoveable() || pOther->GetMass() > m_flPushableMassLimit )
							flLimitVelocity = 0.0f;

						const float flPushSpeed = DotProduct( vControllerVelocity, normal );
						float flNormalForce = pSnapshot->GetNormalForce();
						// IVP's get_vert_force() exposes the distance-keeper pressure, not
						// the entire static support impulse needed to cancel gravity. Jolt's
						// solved lambda includes that support impulse, which can make a player
						// on a frozen slope look as if it is pushing into every facet. Remove
						// only the predictable gravity component for immovable/constraint-static
						// support; retain the full pressure for dynamic props and walls.
						const bool bImmovableSupport = !pOther->IsMoveable()
							|| ( pOther->GetGameFlags() & kFVPhysicsConstraintStatic );
						const float flGravityIntoContact = DotProduct( vGravity, normal );
						if ( bImmovableSupport && normal.z < -0.7f
							&& flGravityIntoContact > 0.0f )
						{
							flNormalForce = Max( 0.0f, flNormalForce
								- m_pObject->GetMass() * flGravityIntoContact );
						}
						const float flContactVelocity = flNormalForce * m_pObject->GetInvMass();
						// Avoid selecting an otherwise tangential contact solely because of
						// floating-point residue in its normal. Real pressure still selects it;
						// the unilateral projection below preserves valid separating movement.
						constexpr float kContactVelocityEpsilon = 1.0e-2f;
						if ( flPushSpeed + flContactVelocity
							> flLimitVelocity + kContactVelocityEpsilon )
							contactNormals.AddNormal( normal );
					}
				}
				pSnapshot->NextFrictionData();
			}
			m_pObject->DestroyFrictionSnapshot( pSnapshot );
		}

		const Vector vClampedVelocity = contactNormals.ClampVector( vControllerVelocity, flLimitVelocity );
		const Vector vContactLimit = vClampedVelocity - vControllerVelocity;
		vControllerVelocity = vClampedVelocity;
		m_vLastImpulse += vContactLimit;

		if ( bGroundContact )
		{
			Vector gravity;
			m_pObject->GetJoltEnvironment()->GetGravity( &gravity );
			const float flGravityStep = gravity.Length() * flDeltaTime;
			// IVP k[1] is negative Source Z. If the controller impulse is downward,
			// retain exactly one gravity step of downward pressure and no more.
			if ( m_vLastImpulse.z <= 0.0f )
			{
				const float flDelta = -flGravityStep - m_vLastImpulse.z;
				vControllerVelocity.z += flDelta;
				m_vLastImpulse.z += flDelta;
			}
		}

		// Havana's player controller drives the actual player physics object. Keeping a
		// second JPH::Character body disconnects contacts and BodyIDs from the object that
		// Source queries through IPhysicsObject, so write the solved velocity to that body.
		m_pObject->GetBody()->SetLinearVelocity( SourceToJolt::Distance( vControllerVelocity ) );
		m_bUpdatedSinceLast = false;
	}

}

void JoltPhysicsPlayerController::OnPostSimulate( float flDeltaTime )
{
	if ( !m_pObject )
		return;

	// Keep the solved pose after contacts/CCD. The earlier game hull sweep is
	// not a collision-free destination once props have moved during this step.
	if ( m_bEnable )
	{
		m_flSecondsToArrival = Max( m_flSecondsToArrival - flDeltaTime, 0.0f );
	}
}

void JoltPhysicsPlayerController::OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject )
{
	if ( pObject == m_pGround )
		SetGroundInternal( nullptr );

	if ( pObject == m_pObject )
		SetObjectInternal( nullptr );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsPlayerController::SetObjectInternal( JoltPhysicsObject *pObject )
{
	if ( m_pObject == pObject )
		return;

	// Reset the last object
	if ( m_pObject )
	{
		SetGroundInternal( nullptr );
		JPH::MotionProperties *pMotion = m_pObject->GetBody()->GetMotionProperties();
		if ( pMotion && m_bAngularDampingSaved )
			pMotion->SetAngularDamping( m_flSavedAngularDamping );
		m_bAngularDampingSaved = false;
		m_pObject->RemoveDestroyedListener( this );
		m_pObject->RemoveCallbackFlags( CALLBACK_IS_PLAYER_CONTROLLER );
		m_pObject->SetPlayerPushMassLimitForContacts( VPHYSICS_MAX_MASS );
		m_pObject->UpdateLayer();
	}

	// Set our new object
	m_pObject = pObject;

	// Adjust the new object
	if ( m_pObject )
	{
		JPH::BodyInterface &bodyInterface = m_pObject->GetJoltEnvironment()
			->GetPhysicsSystem()->GetBodyInterfaceNoLock();
		bodyInterface.SetMotionType( m_pObject->GetBodyID(), JPH::EMotionType::Dynamic,
			JPH::EActivation::Activate );
		m_pObject->EnableDrag( false );
		JPH::MotionProperties *pMotion = m_pObject->GetBody()->GetMotionProperties();
		if ( pMotion )
		{
			// CPlayerController::AttachObject stores rot_speed_damp_factor and sets all
			// three IVP axes to 100. Jolt uses the same exponential damping coefficient,
			// but the scalar applies to all axes.
			m_flSavedAngularDamping = pMotion->GetAngularDamping();
			pMotion->SetAngularDamping( 100.0f );
			m_bAngularDampingSaved = true;
		}
		m_pObject->AddDestroyedListener( this );
		m_pObject->AddCallbackFlags( CALLBACK_IS_PLAYER_CONTROLLER );
		m_pObject->SetPlayerPushMassLimitForContacts( m_flPushableMassLimit );
		m_pObject->UpdateLayer();
	}
}

void JoltPhysicsPlayerController::SetGroundInternal( JoltPhysicsObject *pGround )
{
	if ( m_pGround == pGround )
		return;

	if ( m_pGround )
		m_pGround->RemoveDestroyedListener( this );

	m_pGround = pGround;
	if ( m_pGround )
		m_pGround->AddDestroyedListener( this );
}

Vector JoltPhysicsPlayerController::GetGroundVelocity() const
{
	Vector vGroundVelocity = vec3_origin;
	if ( m_pGround )
	{
		Vector vGroundWorldPosition;
		m_pGround->LocalToWorld( &vGroundWorldPosition, m_vGroundPosition );
		m_pGround->GetVelocityAtPoint( vGroundWorldPosition, &vGroundVelocity );
	}
	return vGroundVelocity;
}
