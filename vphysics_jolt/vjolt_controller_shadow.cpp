#include "cbase.h"

#include "vjolt_surfaceprops.h"
#include "vjolt_controller_shadow.h"
#include "vjolt_listener_contact.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

JoltPhysicsShadowController::JoltPhysicsShadowController( JoltPhysicsObject *pObject,
	bool allowTranslation, bool allowRotation )
	: m_pObject( pObject )
	, m_allowTranslation( allowTranslation )
	, m_allowRotation( allowRotation )
{
	m_control.targetPosition = pObject->GetBody()->GetPosition();
	m_control.targetRotation = pObject->GetBody()->GetRotation();
	m_control.dampFactor = 1.0f;
	if ( auto *motion = pObject->GetBody()->GetMotionPropertiesUnchecked() )
		m_controlLinearDamping = motion->GetLinearDamping();

	// The callback gate has parked solver workers. IVP attaches synchronously;
	// update the body now and refresh prepared equations before resuming them.
	AttachObject();
}

void JoltPhysicsShadowController::AttachObject()
{
	JPH::Body *pBody = m_pObject->GetBody();
	// Havana shadows remain simulated bodies. The allow flags change how strongly
	// physics may move them; they do not disable target translation or rotation.
	m_savedMotionType = pBody->GetMotionType();
	// Attaching does not unfreeze a Source object. Keep its Jolt motion type:
	// live contact storage is specialized for that type and cannot change here.
	if ( m_pObject->m_bPinned )
		m_pObject->RestoreUnhingedInertia();

	JPH::MotionProperties *pMotion = pBody->GetMotionPropertiesUnchecked();
	if ( pMotion )
	{
		m_savedMass = m_pObject->GetMass();
		m_savedInvInertiaDiagonal = pMotion->GetInverseInertiaDiagonalUnchecked();
		m_savedInertiaRotation = pMotion->GetInertiaRotation();
		m_savedLinearDamping = pMotion->GetLinearDamping();
		m_savedAngularDamping = pMotion->GetAngularDamping();
		m_savedMaxLinearVelocity = pMotion->GetMaxLinearVelocity();
		m_savedMaxAngularVelocity = pMotion->GetMaxAngularVelocity();
		m_motionPropertiesSaved = true;

		// IVP damps in its gravity controller before the shadow controller. Jolt
		// damps after OnPreSimulate, so reproduce IVP's ordering ourselves.
		pMotion->SetLinearDamping( 0.0f );
		pMotion->SetAngularDamping( 0.0f );
		// CPhysicsObject::ClampVelocity explicitly skips shadow-controlled objects.
		// Keep Jolt's internal gravity pass from silently reintroducing that clamp.
		pMotion->SetMaxLinearVelocity( JPH::cLargeFloat );
		pMotion->SetMaxAngularVelocity( JPH::cLargeFloat );

		if ( !m_allowRotation )
		{
			// Havana uses a 1e14 kg*m^2 tensor: effectively locked to physical
			// impulses, while the controller may still write angular velocity.
			pMotion->SetInverseInertia( JPH::Vec3::sReplicate( 1.0e-14f ),
				JPH::Quat::sIdentity() );
			if ( m_pObject->m_bPinned ) m_pObject->CaptureUnhingedInertia();
		}
	}

	if ( !m_allowTranslation )
	{
		m_pObject->SetMass( VPHYSICS_MAX_MASS );
		m_pObject->EnableGravity( false );
	}
	if ( m_pObject->m_bPinned ) m_pObject->RestoreUnhingedInertia();

	m_savedMaterialIndex = m_pObject->GetMaterialIndex();
	UseShadowMaterial( true );

	m_savedCallbackFlags = m_pObject->GetCallbackFlags();
	uint16 flags = m_savedCallbackFlags | CALLBACK_SHADOW_COLLISION;
	flags &= ~CALLBACK_GLOBAL_FRICTION;
	flags &= ~CALLBACK_GLOBAL_COLLIDE_STATIC;
	m_pObject->SetCallbackFlags( flags );
	m_pObject->EnableDrag( false );
	m_attached = true;
	m_pObject->GetJoltEnvironment()->NotifyCallbackBodyPropertiesChanged( m_pObject );
}

JoltPhysicsShadowController::JoltPhysicsShadowController( JoltPhysicsObject *pObject,
	JPH::StateRecorder &recorder )
{
	// The body's mass, inertia, damping and material were already restored. Attaching through
	// the ordinary constructor would overwrite the saved pre-shadow properties a second time.
	if ( RestoreState( recorder ) )
	{
		m_pObject = pObject;
		m_attached = true;
	}
}

void JoltPhysicsShadowController::SaveState( JPH::StateRecorder &recorder ) const
{
	recorder.Write( m_control.targetPosition );
	recorder.Write( m_control.targetRotation );
	recorder.Write( m_control.lastPosition );
	recorder.Write( m_control.lastImpulse );
	recorder.Write( m_control.maxAngular );
	recorder.Write( m_control.maxDampAngular );
	recorder.Write( m_control.maxSpeed );
	recorder.Write( m_control.maxDampSpeed );
	recorder.Write( m_control.dampFactor );
	recorder.Write( m_control.teleportDistance );
	recorder.Write( m_secondsToArrival );
	recorder.Write( m_maxSpeed );
	recorder.Write( m_maxAngular );
	recorder.Write( m_isPhysicallyControlled );
	recorder.Write( m_allowTranslation );
	recorder.Write( m_allowRotation );
	recorder.Write( m_enabled );
	recorder.Write( m_savedMaterialIndex );
	recorder.Write( m_savedCallbackFlags );
	recorder.Write( m_savedMass );
	recorder.Write( m_savedLinearDamping );
	recorder.Write( m_savedAngularDamping );
	recorder.Write( m_savedMaxLinearVelocity );
	recorder.Write( m_savedMaxAngularVelocity );
	recorder.Write( m_controlLinearDamping );
	recorder.Write( m_controlAngularDamping );
	recorder.Write( m_savedInvInertiaDiagonal );
	recorder.Write( m_savedInertiaRotation );
	recorder.Write( m_savedMotionType );
	recorder.Write( m_motionPropertiesSaved );
}

bool JoltPhysicsShadowController::RestoreState( JPH::StateRecorder &recorder )
{
	recorder.Read( m_control.targetPosition );
	recorder.Read( m_control.targetRotation );
	recorder.Read( m_control.lastPosition );
	recorder.Read( m_control.lastImpulse );
	recorder.Read( m_control.maxAngular );
	recorder.Read( m_control.maxDampAngular );
	recorder.Read( m_control.maxSpeed );
	recorder.Read( m_control.maxDampSpeed );
	recorder.Read( m_control.dampFactor );
	recorder.Read( m_control.teleportDistance );
	recorder.Read( m_secondsToArrival );
	recorder.Read( m_maxSpeed );
	recorder.Read( m_maxAngular );
	recorder.Read( m_isPhysicallyControlled );
	recorder.Read( m_allowTranslation );
	recorder.Read( m_allowRotation );
	recorder.Read( m_enabled );
	recorder.Read( m_savedMaterialIndex );
	recorder.Read( m_savedCallbackFlags );
	recorder.Read( m_savedMass );
	recorder.Read( m_savedLinearDamping );
	recorder.Read( m_savedAngularDamping );
	recorder.Read( m_savedMaxLinearVelocity );
	recorder.Read( m_savedMaxAngularVelocity );
	recorder.Read( m_controlLinearDamping );
	recorder.Read( m_controlAngularDamping );
	recorder.Read( m_savedInvInertiaDiagonal );
	recorder.Read( m_savedInertiaRotation );
	recorder.Read( m_savedMotionType );
	recorder.Read( m_motionPropertiesSaved );
	return !VJoltStateRecorderFailed( recorder ) && m_savedMotionType <= JPH::EMotionType::Dynamic;
}

JoltPhysicsShadowController::~JoltPhysicsShadowController()
{
	if ( !m_pObject || !m_attached )
		return;

	if ( !( m_pObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) )
	{
		m_pObject->SetCallbackFlags( m_savedCallbackFlags );
		m_pObject->EnableDrag( true );
		m_pObject->EnableGravity( true );
		UseShadowMaterial( false );

		JPH::Body *pBody = m_pObject->GetBody();
		JPH::MotionProperties *pMotion = pBody->GetMotionPropertiesUnchecked();
		if ( pMotion && m_motionPropertiesSaved )
		{
			// SetMass scales inertia, so restore the exact saved tensor afterwards.
			m_pObject->SetMass( m_savedMass );
			if ( m_pObject->m_bPinned ) m_pObject->RestoreUnhingedInertia();
			pMotion->SetInverseInertia( m_savedInvInertiaDiagonal, m_savedInertiaRotation );
			if ( m_pObject->m_bPinned ) m_pObject->CaptureUnhingedInertia();
			// Source keeps linear damping edits made while controlled, but restores
			// the angular damping value saved by AttachObject.
			pMotion->SetLinearDamping( m_controlLinearDamping );
			pMotion->SetAngularDamping( m_savedAngularDamping );
			pMotion->SetMaxLinearVelocity( m_savedMaxLinearVelocity );
			pMotion->SetMaxAngularVelocity( m_savedMaxAngularVelocity );
		}

		m_pObject->GetJoltEnvironment()->NotifyCallbackBodyPropertiesChanged( m_pObject );
	}
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsShadowController::Update( const Vector &position, const QAngle &angles,
	float timeOffset )
{
	const JPH::Vec3 targetPosition = SourceToJolt::Distance( position );
	const JPH::Quat targetRotation = SourceToJolt::Angle( angles );
	const bool samePosition = targetPosition.IsClose( m_control.targetPosition, 1.0e-8f );
	const bool sameRotation = targetRotation.IsClose( m_control.targetRotation, 1.0e-8f );

	m_control.targetPosition = targetPosition;
	m_control.targetRotation = targetRotation;
	m_secondsToArrival = Max( timeOffset, 0.0f );
	m_enabled = true;

	if ( !samePosition || !sameRotation )
		m_pObject->Wake();
}

void JoltPhysicsShadowController::MaxSpeed( float maxSpeed, float maxAngularSpeed )
{
	m_maxSpeed = maxSpeed;
	m_maxAngular = maxAngularSpeed;
	// CShadowController::MaxSpeed stores these directly in the IVP-space controller
	// state. This is intentionally different from hlshadowcontrol_params_t, whose
	// standalone ComputeShadowControl path performs explicit unit conversion.
	m_control.maxSpeed = maxSpeed;
	m_control.maxDampSpeed = maxSpeed;
	m_control.maxAngular = maxAngularSpeed;
	m_control.maxDampAngular = maxAngularSpeed;
}

void JoltPhysicsShadowController::StepUp( float height )
{
	if ( height != 0.0f )
		m_pObject->AddToPosition( JPH::Vec3( 0.0f, 0.0f, SourceToJolt::Distance( height ) ) );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsShadowController::SetTeleportDistance( float teleportDistance )
{
	m_control.teleportDistance = SourceToJolt::Distance( teleportDistance );
}

bool JoltPhysicsShadowController::AllowsTranslation()
{
	return m_allowTranslation;
}

bool JoltPhysicsShadowController::AllowsRotation()
{
	return m_allowRotation;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsShadowController::SetPhysicallyControlled( bool isPhysicallyControlled )
{
	m_isPhysicallyControlled = isPhysicallyControlled;
}

bool JoltPhysicsShadowController::IsPhysicallyControlled()
{
	return m_isPhysicallyControlled;
}

void JoltPhysicsShadowController::GetLastImpulse( Vector *pOut )
{
	if ( pOut )
		*pOut = JoltToSource::Distance( m_control.lastImpulse );
}

void JoltPhysicsShadowController::UseShadowMaterial( bool useShadowMaterial )
{
	if ( !m_pObject )
		return;

	const int current = m_pObject->GetMaterialIndex();
	const int target = useShadowMaterial
		? JoltPhysicsSurfaceProps::GetInstance().GetShadowMaterialIndex()
		: m_savedMaterialIndex;
	if ( target != current )
	{
		// Once the interface is published, SetMaterialIndex notifies this
		// controller. An internal material swap must not replace the material
		// that will be restored on detach.
		const int savedMaterialIndex = m_savedMaterialIndex;
		m_pObject->SetMaterialIndex( target );
		m_savedMaterialIndex = savedMaterialIndex;
	}
}

void JoltPhysicsShadowController::ObjectMaterialChanged( int materialIndex )
{
	if ( m_pObject )
		m_savedMaterialIndex = materialIndex;
}

//-------------------------------------------------------------------------------------------------

float JoltPhysicsShadowController::GetTargetPosition( Vector *pPositionOut,
	QAngle *pAnglesOut )
{
	if ( pPositionOut )
		*pPositionOut = JoltToSource::Distance( m_control.targetPosition );
	if ( pAnglesOut )
		*pAnglesOut = JoltToSource::Angle( m_control.targetRotation );
	return m_secondsToArrival;
}

float JoltPhysicsShadowController::GetTeleportDistance()
{
	return JoltToSource::Distance( m_control.teleportDistance );
}

void JoltPhysicsShadowController::GetMaxSpeed( float *pMaxSpeedOut,
	float *pMaxAngularSpeedOut )
{
	if ( pMaxSpeedOut )
		*pMaxSpeedOut = m_maxSpeed;
	if ( pMaxAngularSpeedOut )
		*pMaxAngularSpeedOut = m_maxAngular;
}

void JoltPhysicsShadowController::SetDamping( const float *pLinear, const float *pAngular )
{
	if ( pLinear )
		m_controlLinearDamping = Max( *pLinear, 0.0f );
	if ( pAngular )
		m_controlAngularDamping = Max( *pAngular, 0.0f );
}

void JoltPhysicsShadowController::GetDamping( float *pLinear, float *pAngular ) const
{
	if ( pLinear )
		*pLinear = m_controlLinearDamping;
	if ( pAngular )
		*pAngular = m_controlAngularDamping;
}

void JoltPhysicsShadowController::SetEnvironmentVelocityLimits( float maxLinearVelocity,
	float maxAngularVelocity )
{
	m_savedMaxLinearVelocity = maxLinearVelocity;
	m_savedMaxAngularVelocity = maxAngularVelocity;
	if ( !m_pObject )
		return;

	if ( JPH::MotionProperties *pMotion = m_pObject->GetBody()->GetMotionPropertiesUnchecked() )
	{
		pMotion->SetMaxLinearVelocity( JPH::cLargeFloat );
		pMotion->SetMaxAngularVelocity( JPH::cLargeFloat );
	}
}

//-------------------------------------------------------------------------------------------------

bool JoltPhysicsShadowController::IsOnGround() const
{
	thread_local std::vector< JoltFrictionContactCache::Contact > contacts;
	m_pObject->GetJoltEnvironment()->GetContactListener()->CopyFrictionContacts(
		m_pObject->GetBodyID(), contacts );
	for ( const JoltFrictionContactCache::Contact &contact : contacts )
	{
		if ( contact.normal.GetZ() < -0.7f )
			return true;
	}
	return false;
}

void JoltPhysicsShadowController::OnPreSimulate( float deltaTime )
{
	if ( !m_pObject || !m_attached || deltaTime <= 0.0f )
		return;

	JPH::Body *pBody = m_pObject->GetBody();
	if ( !pBody->IsDynamic() || !m_pObject->IsMotionEnabled() ) return;
	JPH::MotionProperties *pMotion = pBody->GetMotionProperties();
	if ( !pMotion )
		return;

	JPH::BodyInterface &bodyInterface = m_pObject->GetJoltEnvironment()
		->GetPhysicsSystem()->GetBodyInterfaceNoLock();
	if ( !pBody->IsActive() )
	{
		if ( !m_enabled )
		{
			m_control.lastPosition = JPH::Vec3::sZero();
			return;
		}

		// IVP's movement test observes rotational progress even for symmetric or
		// near-symmetric shapes. Jolt can otherwise put a controlled body to sleep
		// while a target error remains, permanently stopping the controller because
		// an identical Update call deliberately does not wake it.
		const bool atTargetPosition =
			( m_control.targetPosition - pBody->GetPosition() ).LengthSq() <= 1.0e-8f;
		const bool atTargetRotation = VJoltShadowControl::QuaternionDeltaAngles(
			m_control.targetRotation, pBody->GetRotation() ).LengthSq() <= 1.0e-8f;
		if ( atTargetPosition && atTargetRotation )
			return;

		bodyInterface.ActivateBody( m_pObject->GetBodyID() );
	}

	JPH::Vec3 linearVelocity = pBody->GetLinearVelocity()
		* VJoltShadowControl::LinearDampingFactor( m_controlLinearDamping, deltaTime );
	JPH::Vec3 angularVelocity = pBody->GetAngularVelocity()
		* VJoltShadowControl::AngularDampingFactor( m_controlAngularDamping, deltaTime );

	// Source gravity runs before its shadow controller. Include gravity while solving,
	// then subtract it before Jolt's normal gravity pass so it is applied exactly once.
	const JPH::Vec3 gravityStep = m_pObject->GetJoltEnvironment()->GetPhysicsSystem()->GetGravity()
		* pMotion->GetGravityFactor() * deltaTime;
	linearVelocity += gravityStep;

	JPH::Vec3 position = pBody->GetPosition();
	JPH::Quat rotation = pBody->GetRotation();
	bool teleported = false;
	if ( m_enabled )
	{
		const VJoltShadowControl::Result result = VJoltShadowControl::Compute(
			m_control, position, rotation, linearVelocity, angularVelocity,
			m_secondsToArrival, deltaTime );
		m_secondsToArrival = result.secondsToArrival;
		teleported = result.teleported;

		VJoltShadowControl::ApplyGroundCorrection( m_allowTranslation, IsOnGround(),
			gravityStep, linearVelocity, m_control.lastImpulse );
	}
	else
	{
		m_control.lastPosition = JPH::Vec3::sZero();
	}

	if ( teleported )
	{
		m_pObject->GetJoltEnvironment()->GetContactListener()->InvalidateFrictionContacts(
			m_pObject->GetBodyID() );
		bodyInterface.InvalidateContactCache( m_pObject->GetBodyID() );
		bodyInterface.SetPositionAndRotation( m_pObject->GetBodyID(), position, rotation,
			JPH::EActivation::Activate );
	}

	bodyInterface.SetLinearAndAngularVelocity( m_pObject->GetBodyID(),
		linearVelocity - gravityStep, angularVelocity );
}
