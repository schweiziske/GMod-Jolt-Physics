//=================================================================================================
//
// Constraints
//
//=================================================================================================

#include "cbase.h"

#include <optional>

#include "vjolt_environment.h"
#include "vjolt_layers.h"
#include "vjolt_object.h"

#include "vjolt_constraints.h"
#include "vjolt_constraint_frames.h"
#include "vjolt_havana_constraints.h"

#include "vjolt_layers.h"

#include "tier0/basetypes.h"
#include "mathlib/mathlib.h"

enum MatrixAxisType_t
{
	X_AXIS = 0,
	Y_AXIS = 1,
	Z_AXIS = 2,
};

static JPH::Mat44 BuildHavanaRagdollFrame(
	const matrix3x4_t &sourceFrame,
	JPH::Vec3Arg shapeCOM );

//-------------------------------------------------------------------------------------------------

static ConVar vjolt_constraint_velocity_substeps( "vjolt_constraint_velocity_substeps", "0" );
static ConVar vjolt_constraint_position_substeps( "vjolt_constraint_position_substeps", "0" );

static ConVar vjolt_ragdoll_min_torque_friction( "vjolt_ragdoll_min_torque_friction", "0.05" );

static constexpr float UNBREAKABLE_BREAK_LIMIT = 1e12f;

static bool UsesHavanaBreakableWrapper( const constraint_breakableparams_t &params )
{
	return ( params.forceLimit != 0.0f && params.forceLimit < UNBREAKABLE_BREAK_LIMIT )
		|| ( params.torqueLimit != 0.0f && params.torqueLimit < UNBREAKABLE_BREAK_LIMIT )
		|| ( params.bodyMassScale[0] != 1.0f && params.bodyMassScale[0] != 0.0f )
		|| ( params.bodyMassScale[1] != 1.0f && params.bodyMassScale[1] != 0.0f );
}

//-------------------------------------------------------------------------------------------------

JoltPhysicsConstraintGroup::JoltPhysicsConstraintGroup( JoltPhysicsEnvironment *pEnvironment, const constraint_groupparams_t &params, bool active )
	: m_pEnvironment( pEnvironment ), m_ErrorParams( params ), m_bActive( active )
{
	// An explicit Source group is shared even while it only contains one constraint. Breakable
	// constraints in that LCS must not be mistaken for a standalone, wrapper-owned local system.
	m_Solver.SetRollbackOnBreak( false );
	// additionalIterations is a construction parameter in Havana. SetErrorParams only changes the
	// error reporting fields, and negative values are deliberately preserved.
	m_Solver.SetAdditionalIterations( params.additionalIterations );
}

JoltPhysicsConstraintGroup::~JoltPhysicsConstraintGroup()
{
	// The Source interface permits the group wrapper to be destroyed separately from its member
	// constraints. Break both directions of the relationship before m_Solver goes away so no
	// constraint retains a dangling LegacyConstraintGroup pointer.
	const std::vector< JoltPhysicsConstraint * > constraints = m_pConstraints;
	for ( JoltPhysicsConstraint *pConstraint : constraints )
		pConstraint->OnConstraintGroupDestroyed( this );
	m_pConstraints.clear();
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraintGroup::Activate()
{
	// A callback-created member is not in the live solver yet. Queue activation
	// behind its registration rather than dropping the request as an empty group.
	if ( m_pEnvironment->DeferPhysicsMutation( [this]() { Activate(); } ) )
		return;
	// hk_Local_Constraint_System::activate() on an empty group is a no-op and does not remember a
	// pending activation request. Once active, however, later additions inherit the active state.
	if ( m_bActive || m_Solver.GetConstraintCount() == 0 )
		return;

	m_bActive = true;
	for ( JoltPhysicsConstraint *pConstraint : m_pConstraints )
		pConstraint->SetEnabledFromConstraintGroup( this, true );
}

bool JoltPhysicsConstraintGroup::IsInErrorState()
{
	// Havana's error accumulation is compiled out, so m_errorCount remains zero. Preserve the one
	// observable boundary case from has_error(): non-positive minErrorTicks reports an error.
	return m_ErrorParams.minErrorTicks <= 0;
}

void JoltPhysicsConstraintGroup::ClearErrorState()
{
}

void JoltPhysicsConstraintGroup::GetErrorParams( constraint_groupparams_t *pParams )
{
	if ( pParams )
		*pParams = m_ErrorParams;
}

void JoltPhysicsConstraintGroup::SetErrorParams( const constraint_groupparams_t &params )
{
	// Source only updates these two fields after construction. In particular, callers that obtain
	// the params, edit minErrorTicks and write them back must not reconfigure solver iterations.
	m_ErrorParams.minErrorTicks = params.minErrorTicks;
	m_ErrorParams.errorTolerance = params.errorTolerance;
}

void JoltPhysicsConstraintGroup::SolvePenetration( IPhysicsObject *pObj0, IPhysicsObject *pObj1 )
{
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraintGroup::AddConstraint( JoltPhysicsConstraint *pConstraint )
{
	m_pConstraints.push_back( pConstraint );
}

void JoltPhysicsConstraintGroup::RemoveConstraint( JoltPhysicsConstraint *pConstraint )
{
	Erase( m_pConstraints, pConstraint );
}

void JoltPhysicsConstraintGroup::AddSolverConstraint( JPH::Constraint *pConstraint )
{
	m_Solver.AddConstraint( pConstraint );
	pConstraint->SetEnabled( m_bActive );
}

void JoltPhysicsConstraintGroup::RemoveSolverConstraint( JPH::Constraint *pConstraint )
{
	m_Solver.RemoveConstraint( pConstraint );
}

//-------------------------------------------------------------------------------------------------

JoltPhysicsConstraint::JoltPhysicsConstraint( JoltPhysicsEnvironment *pPhysicsEnvironment, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, constraintType_t Type, JPH::Constraint* pConstraint, void *pGameData, JoltPhysicsConstraintGroup *pGroup, bool systemDestroyed )
	: m_pPhysicsEnvironment( pPhysicsEnvironment )
	, m_pPhysicsSystem( pPhysicsEnvironment->GetPhysicsSystem() )
	, m_pObjReference( static_cast<JoltPhysicsObject*>( pReferenceObject ) )
	, m_pObjAttached( static_cast<JoltPhysicsObject*>( pAttachedObject ) )
	, m_ConstraintType( Type )
	, m_pConstraint( pConstraint )
	, m_pGameData( pGameData )
	, m_pGroup( pGroup )
	, m_bConstraintSystemDestroyed( systemDestroyed )
{
	m_PrivateSolver.SetRollbackOnBreak( true );
	m_pObjReference->AddConstraintAttachment();
	m_pObjAttached->AddConstraintAttachment();
	m_pObjReference->AddDestroyedListener( this );
	m_pObjAttached->AddDestroyedListener( this );
	m_pPhysicsEnvironment->RegisterConstraint( this );
	// Restore constructs the Jolt constraint before this wrapper. Recreate Source's
	// standalone one-constraint local system for that path as well.
	m_bRegisteredWithPhysicsSystem = pConstraint != nullptr;
	if ( m_pGroup )
		m_pGroup->AddConstraint( this );
	RegisterSolverConstraint();
}

JoltPhysicsConstraint::~JoltPhysicsConstraint()
{
	UnregisterSolverConstraint();

	if ( m_pGroup )
	{
		m_pGroup->RemoveConstraint( this );
		m_pGroup = nullptr;
	}

	m_pPhysicsEnvironment->UnregisterConstraint( this );

	DestroyConstraint();
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::Activate()
{
	if ( m_pPhysicsEnvironment->IsInContactCallback() && m_pPhysicsEnvironment->DeferPhysicsMutation( [this]() { Activate(); } ) ) return;
	// Grouped Havana constraints do not own an LCS, so their individual activation methods are
	// no-ops. The same remains true after their shared LCS has been destroyed.
	if ( m_pGroup || m_bConstraintSystemDestroyed )
		return;

	if ( m_pConstraint )
		m_pConstraint->SetEnabled( true );
}

void JoltPhysicsConstraint::Deactivate()
{
	if ( m_pPhysicsEnvironment->IsInContactCallback() && m_pPhysicsEnvironment->DeferPhysicsMutation( [this]() { Deactivate(); } ) ) return;
	if ( m_pGroup || m_bConstraintSystemDestroyed )
		return;

	if ( m_pConstraint )
		m_pConstraint->SetEnabled( false );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::SetGameData( void *gameData )
{
	m_pGameData = gameData;
}

void *JoltPhysicsConstraint::GetGameData() const
{
	return m_pGameData;
}

//-------------------------------------------------------------------------------------------------

IPhysicsObject *JoltPhysicsConstraint::GetReferenceObject() const
{
	return m_pObjReference;
}

IPhysicsObject *JoltPhysicsConstraint::GetAttachedObject() const
{
	return m_pObjAttached;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::SetLinearMotor( float speed, float maxLinearImpulse )
{
	if ( m_pPhysicsEnvironment->IsInContactCallback() && m_pPhysicsEnvironment->DeferPhysicsMutation( [this, speed, maxLinearImpulse]() { SetLinearMotor( speed, maxLinearImpulse ); } ) ) return;
	if ( !m_pConstraint )
		return;

	speed = SourceToJolt::Distance( speed );
	maxLinearImpulse = fabsf( SourceToJolt::Distance( maxLinearImpulse ) );

	switch ( m_ConstraintType )
	{
		case CONSTRAINT_SLIDING:
		{
			JPH::SliderConstraint *pConstraint = static_cast<JPH::SliderConstraint *>( m_pConstraint.GetPtr() );
			pConstraint->SetLegacyMotor( speed, maxLinearImpulse );

			break;
		}
	}
}

void JoltPhysicsConstraint::SetAngularMotor( float rotSpeed, float maxAngularImpulse )
{
	if ( m_pPhysicsEnvironment->IsInContactCallback() && m_pPhysicsEnvironment->DeferPhysicsMutation( [this, rotSpeed, maxAngularImpulse]() { SetAngularMotor( rotSpeed, maxAngularImpulse ); } ) ) return;
	if ( !m_pConstraint )
		return;

	if ( m_pConstraint->GetSubType() == JPH::EConstraintSubType::User1 )
	{
		// Source only updates ragdoll friction through this API when rotSpeed is zero.
		if ( m_ConstraintType == CONSTRAINT_RAGDOLL && rotSpeed == 0.0f )
			static_cast< JPH::HavanaConstraint * >( m_pConstraint.GetPtr() )->SetRagdollFriction(
				fabsf( DEG2RAD( maxAngularImpulse ) ) );
		return;
	}

	// Match Source's actual Havana adapter: both values go through ConvertAngleToIVP (DEG2RAD),
	// including the motor/friction limit. This API's legacy parameter name and header units are
	// misleading, but changing it to SI torque would not be VPhysics-compatible.
	rotSpeed = DEG2RAD( rotSpeed );
	maxAngularImpulse = fabsf( DEG2RAD( maxAngularImpulse ) );

	switch ( m_ConstraintType )
	{
		case CONSTRAINT_RAGDOLL:
		{
			break;
		}

		case CONSTRAINT_HINGE:
		{
			JPH::HingeConstraint *pConstraint = static_cast<JPH::HingeConstraint *>( m_pConstraint.GetPtr() );
			pConstraint->SetLegacyMotor( rotSpeed, maxAngularImpulse );

			break;
		}
	}
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::UpdateRagdollTransforms( const matrix3x4_t &constraintToReference, const matrix3x4_t &constraintToAttached )
{
	if ( m_pPhysicsEnvironment->IsInContactCallback() && m_pPhysicsEnvironment->DeferPhysicsMutation( [this, constraintToReference, constraintToAttached]() { UpdateRagdollTransforms( constraintToReference, constraintToAttached ); } ) ) return;
	if ( m_ConstraintType != CONSTRAINT_RAGDOLL || !m_pConstraint
		|| m_pConstraint->GetSubType() != JPH::EConstraintSubType::User1 )
		return;

	static_cast< JPH::HavanaConstraint * >( m_pConstraint.GetPtr() )->UpdateRagdollTransforms(
		BuildHavanaRagdollFrame(
			constraintToReference,
			m_pObjReference->GetBody()->GetShape()->GetCenterOfMass() ),
		BuildHavanaRagdollFrame(
			constraintToAttached,
			m_pObjAttached->GetBody()->GetShape()->GetCenterOfMass() ) );
}

static matrix3x4_t JoltConstraintFrameToSourceObjectLocal( const JPH::Mat44 &frameCOM, JPH::Vec3Arg shapeCOM )
{
	return matrix3x4_t
	{
		JoltToSource::Unitless( frameCOM.GetAxisX() ),
		JoltToSource::Unitless( frameCOM.GetAxisY() ),
		JoltToSource::Unitless( frameCOM.GetAxisZ() ),
		JoltToSource::Distance( frameCOM.GetTranslation() + shapeCOM ),
	};
}

static JPH::Mat44 BuildConstraintFrame(
	JPH::Vec3Arg pointCOM,
	JPH::Vec3Arg axisX,
	JPH::Vec3Arg axisY )
{
	return JPH::Mat44(
		JPH::Vec4( axisX, 0.0f ),
		JPH::Vec4( axisY, 0.0f ),
		JPH::Vec4( axisX.Cross( axisY ), 0.0f ),
		JPH::Vec4( pointCOM, 1.0f ) );
}

static JPH::Mat44 BuildHavanaRagdollFrame(
	const matrix3x4_t &sourceFrame,
	JPH::Vec3Arg shapeCOM )
{
	JPH::Mat44 frame = SourceToJolt::Matrix( sourceFrame );
	frame.SetTranslation( frame.GetTranslation() - shapeCOM );
	return JPH::HavanaFromSourceTransform( frame );
}

bool JoltPhysicsConstraint::GetConstraintTransform( matrix3x4_t *pConstraintToReference, matrix3x4_t *pConstraintToAttached ) const
{
	if ( !m_pConstraint || !m_pObjReference || !m_pObjAttached )
		return false;

	// Source only exposes meaningful frames for constraints that actually retain a frame. Do not
	// return the bodies' world matrices for unsupported types: that looks valid to callers but is
	// semantically wrong and makes frame diagnostics impossible.
	if ( m_pConstraint->GetSubType() == JPH::EConstraintSubType::User1 )
	{
		// Ragdolls report their runtime frames. A weld's frames are the attached object's origin
		// in reference space and in its own space. Length constraints keep no frame.
		const auto *pHavana = static_cast< const JPH::HavanaConstraint * >( m_pConstraint.GetPtr() );
		if ( pHavana->GetHavanaKind() == JPH::EHavanaConstraintKind::Length )
			return false;
		if ( pConstraintToReference )
			*pConstraintToReference = JoltConstraintFrameToSourceObjectLocal(
				pHavana->GetConstraintToBody1Matrix(),
				m_pObjReference->GetBody()->GetShape()->GetCenterOfMass() );
		if ( pConstraintToAttached )
			*pConstraintToAttached = JoltConstraintFrameToSourceObjectLocal(
				pHavana->GetConstraintToBody2Matrix(),
				m_pObjAttached->GetBody()->GetShape()->GetCenterOfMass() );
		return true;
	}

	if ( m_pConstraint->GetSubType() == JPH::EConstraintSubType::Point )
	{
		const auto *pPoint = static_cast< const JPH::PointConstraint * >( m_pConstraint.GetPtr() );
		if ( pConstraintToReference )
			*pConstraintToReference = JoltConstraintFrameToSourceObjectLocal( pPoint->GetConstraintToBody1Matrix(), m_pObjReference->GetBody()->GetShape()->GetCenterOfMass() );
		if ( pConstraintToAttached )
			*pConstraintToAttached = JoltConstraintFrameToSourceObjectLocal( pPoint->GetConstraintToBody2Matrix(), m_pObjAttached->GetBody()->GetShape()->GetCenterOfMass() );
		return true;
	}

	return false;
}

bool JoltPhysicsConstraint::GetConstraintParams( constraint_breakableparams_t *pParams ) const
{
	if ( !pParams )
		return false;

	pParams->forceLimit = m_SourceForceLimit;
	pParams->torqueLimit = m_SourceTorqueLimit;
	pParams->bodyMassScale[0] = m_BodyMassScale[0];
	pParams->bodyMassScale[1] = m_BodyMassScale[1];
	pParams->strength = m_BreakStrength;
	pParams->isActive = m_pConstraint ? m_pConstraint->GetEnabled() : false;
	return true;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::SetBreakableParams( const constraint_breakableparams_t &params )
{
	m_SourceForceLimit = params.forceLimit;
	m_SourceTorqueLimit = params.torqueLimit;
	m_BreakStrength = params.strength;
	m_BodyMassScale[0] = params.bodyMassScale[0];
	m_BodyMassScale[1] = params.bodyMassScale[1];

	const bool bBreakLinear = params.forceLimit > 0.0f && params.forceLimit < UNBREAKABLE_BREAK_LIMIT;
	const bool bBreakAngular = params.torqueLimit > 0.0f && params.torqueLimit < UNBREAKABLE_BREAK_LIMIT;

	m_LinearBreakImpulse = bBreakLinear ? SourceToJolt::Distance( params.forceLimit ) : 0.0f;
	m_AngularBreakImpulse = bBreakAngular ? DEG2RAD( params.torqueLimit ) : 0.0f;
	if ( bBreakLinear || bBreakAngular )
		m_pPhysicsEnvironment->RegisterBreakableConstraint( this );
	else
		m_pPhysicsEnvironment->UnregisterBreakableConstraint( this );
	if ( m_pConstraint )
		m_pConstraint->SetLegacyBreakLimits( m_LinearBreakImpulse, m_AngularBreakImpulse );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::OutputDebugInfo()
{

}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject )
{
	DestroyConstraint();

	// Normal VPhysics calls ConstraintBroken when an object being killed destroys the constraint.
	m_pPhysicsEnvironment->NotifyConstraintDisabled( this );
}

//-------------------------------------------------------------------------------------------------
// Ragdoll
//-------------------------------------------------------------------------------------------------

static std::optional<JoltMatrixAxes> DOFBitToAxis( uint32 uDOFMask )
{
	if ( uDOFMask & 0b001 )
		return MatrixAxis::X;
	else if ( uDOFMask & 0b010 )
		return MatrixAxis::Y;
	else if ( uDOFMask & 0b100 )
		return MatrixAxis::Z;
	else
		return std::nullopt;
}

struct RagdollLimits_t
{
	struct Limit_t
	{
		float Min;
		float Max;

		float GetRange() const
		{
			return Max - Min;
		}
	};

	RagdollLimits_t( const constraint_ragdollparams_t &ragdoll )
	{
		for ( int i = 0; i < 3; i++ )
		{
			if ( ragdoll.useClockwiseRotations )
			{
				lAxisLimitsRad[i].Min = DEG2RAD( -ragdoll.axes[i].maxRotation );
				lAxisLimitsRad[i].Max = DEG2RAD( -ragdoll.axes[i].minRotation );
			}
			else
			{
				lAxisLimitsRad[i].Min = DEG2RAD( ragdoll.axes[i].minRotation );
				lAxisLimitsRad[i].Max = DEG2RAD( ragdoll.axes[i].maxRotation );
			}
		}
	}

	uint32 GetDegreesOfFreedomMask() const
	{
		uint32 uDOFMask = 0;

		for ( int i = 0; i < 3; i++ )
		{
			if ( lAxisLimitsRad[i].GetRange() > DEG2RAD( 5.0f ) )
				uDOFMask |= 1u << i;
		}

		return uDOFMask;
	}

	Limit_t lAxisLimitsRad[3]{};
};

void JoltPhysicsConstraint::InitialiseRagdoll( IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll )
{
	// Source performs this conversion before creating its Havana ragdoll constraint. It is not an
	// optional Jolt optimization: the two object-local perpendicular axes define the hinge angle
	// and limit reference exactly, so preserve that path first.
	if ( InitialiseHingeFromRagdoll( pGroup, ragdoll ) )
		return;

	SetGroup( pGroup );
	m_ConstraintType = CONSTRAINT_RAGDOLL;
	SetBreakableParams( ragdoll.constraint );

	JPH::Body *pRefBody = m_pObjReference->GetBody();
	JPH::Body *pAttBody = m_pObjAttached->GetBody();
	JPH::HavanaConstraintSettings settings;
	settings.mKind = JPH::EHavanaConstraintKind::Ragdoll;
	settings.mFrame1 = BuildHavanaRagdollFrame(
		ragdoll.constraintToReference, pRefBody->GetShape()->GetCenterOfMass() );
	settings.mFrame2 = BuildHavanaRagdollFrame(
		ragdoll.constraintToAttached, pAttBody->GetShape()->GetCenterOfMass() );
	settings.mTau = 1.0f;
	settings.mStrength = 1.0f;
	settings.mRagdollUseClockwiseRotations = ragdoll.useClockwiseRotations;
	settings.mRagdollConstrainTranslation = !ragdoll.onlyAngularLimits;
	const float referenceMass = m_pObjReference->GetMass();
	for ( int axis = 0; axis < 3; ++axis )
	{
		settings.mRagdollLimitMin.SetComponent( axis, DEG2RAD( ragdoll.axes[axis].minRotation ) );
		settings.mRagdollLimitMax.SetComponent( axis, DEG2RAD( ragdoll.axes[axis].maxRotation ) );
		settings.mRagdollMotorVelocity.SetComponent( axis, DEG2RAD( ragdoll.axes[axis].angularVelocity ) );
		settings.mRagdollMotorForce.SetComponent( axis, ragdoll.axes[axis].torque * referenceMass );
	}

	m_pConstraint = settings.Create( *pRefBody, *pAttBody );
	m_pConstraint->SetLegacyBreakLimits( m_LinearBreakImpulse, m_AngularBreakImpulse );
	m_pConstraint->SetEnabled( !m_pGroup && ragdoll.constraint.isActive );
	RegisterWithPhysicsSystem();
}

//-------------------------------------------------------------------------------------------------
// Hinge
//-------------------------------------------------------------------------------------------------

bool JoltPhysicsConstraint::InitialiseHingeFromRagdoll( IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll )
{
	int nDegreesOfFreedom = 0;
	int dofIndex = 0;
	for ( int i = 0; i < 3; ++i )
	{
		if ( ragdoll.axes[i].minRotation != ragdoll.axes[i].maxRotation )
		{
			dofIndex = i;
			++nDegreesOfFreedom;
		}
	}

	if ( nDegreesOfFreedom != 1 )
		return false;

	constraint_hingeparams_t hinge;
	hinge.Defaults();
	hinge.constraint = ragdoll.constraint;
	// This is forced in Source because many ragdoll descriptions leave strength unset and the
	// Havana ragdoll solver ignores it.
	hinge.constraint.strength = 1.0f;

	matrix3x4_t referenceToWorld;
	m_pObjReference->GetPositionMatrix( &referenceToWorld );
	matrix3x4_t constraintToWorld;
	ConcatTransforms( referenceToWorld, ragdoll.constraintToReference, constraintToWorld );
	hinge.worldPosition = GetColumn( constraintToWorld, MatrixAxis::Origin );
	hinge.worldAxisDirection = GetColumn( constraintToWorld, JoltMatrixAxes( dofIndex ) );
	hinge.hingeAxis = ragdoll.axes[dofIndex];

	Vector referencePerpAxisObject;
	referencePerpAxisObject.Init();
	referencePerpAxisObject[( dofIndex + 1 ) % 3] = 1.0f;
	Vector perpAxisConstraint;
	VectorIRotate( referencePerpAxisObject, ragdoll.constraintToReference, perpAxisConstraint );
	Vector attachedPerpAxisObject;
	VectorRotate( perpAxisConstraint, ragdoll.constraintToAttached, attachedPerpAxisObject );

	// Source's ragdoll friction is multiplied by reference mass before entering the hinge adapter.
	// InitHinge then applies DEG2RAD, so RAD2DEG here intentionally cancels that conversion.
	hinge.hingeAxis.torque = RAD2DEG( hinge.hingeAxis.torque * m_pObjReference->GetMass() );

	// Ragdoll's historical clockwise flag changes its public limit convention. Source flips the
	// axis limits here; InitialiseHingeInternal performs the separate Havana -> Jolt sign flip.
	if ( !ragdoll.useClockwiseRotations )
	{
		const float minRotation = hinge.hingeAxis.minRotation;
		hinge.hingeAxis.minRotation = -hinge.hingeAxis.maxRotation;
		hinge.hingeAxis.maxRotation = -minRotation;
	}

	InitialiseHingeInternal(
		pGroup,
		hinge,
		SourceToJolt::Unitless( referencePerpAxisObject ),
		SourceToJolt::Unitless( attachedPerpAxisObject ) );
	return true;
}

void JoltPhysicsConstraint::InitialiseHinge( IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge )
{
	InitialiseHingeInternal( pGroup, hinge, JPH::Vec3::sZero(), JPH::Vec3::sZero() );
}

void JoltPhysicsConstraint::InitialiseHingeInternal(
	IPhysicsConstraintGroup *pGroup,
	const constraint_hingeparams_t &hinge,
	JPH::Vec3Arg referencePerpAxisObject,
	JPH::Vec3Arg attachedPerpAxisObject )
{
	SetGroup( pGroup );
	m_ConstraintType = CONSTRAINT_HINGE;
	SetBreakableParams( hinge.constraint );

	// Get our bodies
	JPH::Body *refBody = m_pObjReference->GetBody();
	JPH::Body *attBody = m_pObjAttached->GetBody();

	JPH::HingeConstraintSettings settings;
	settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;

	const JPH::Mat44 referenceObjectToWorld = JPH::Mat44::sRotationTranslation(
		refBody->GetRotation(), JPH::Vec3( refBody->GetPosition() ) );
	const JPH::Mat44 attachedObjectToWorld = JPH::Mat44::sRotationTranslation(
		attBody->GetRotation(), JPH::Vec3( attBody->GetPosition() ) );
	const JPH::Vec3 sourceAxis = SourceToJolt::Unitless( hinge.worldAxisDirection );
	if ( sourceAxis.LengthSq() <= Square( FLT_EPSILON ) )
		Log_Warning( LOG_VJolt, "Hinge constraint has a zero world axis; using world X.\n" );

	const JoltHingeConstraintFrames frames = BuildJoltHingeConstraintFrames(
		referenceObjectToWorld,
		attachedObjectToWorld,
		SourceToJolt::Distance( hinge.worldPosition ),
		sourceAxis,
		referencePerpAxisObject,
		attachedPerpAxisObject,
		refBody->GetShape()->GetCenterOfMass(),
		attBody->GetShape()->GetCenterOfMass() );
	settings.mPoint1 = frames.m_PointReferenceCOM;
	settings.mHingeAxis1 = frames.m_HingeAxisReferenceCOM;
	settings.mNormalAxis1 = frames.m_NormalAxisReferenceCOM;
	settings.mPoint2 = frames.m_PointAttachedCOM;
	settings.mHingeAxis2 = frames.m_HingeAxisAttachedCOM;
	settings.mNormalAxis2 = frames.m_NormalAxisAttachedCOM;

	if ( hinge.hingeAxis.minRotation != hinge.hingeAxis.maxRotation )
	{
		settings.mLimitsMin = DEG2RAD( -hinge.hingeAxis.maxRotation );
		settings.mLimitsMax = DEG2RAD( -hinge.hingeAxis.minRotation );
	}

	// Source's hinge adapter explicitly uses DEG2RAD for this legacy motor/friction limit.
	const float maxFrictionTorque = fabsf( DEG2RAD( hinge.hingeAxis.torque ) );

	m_pConstraint = settings.Create( *refBody, *attBody );
	m_pConstraint->SetLegacyVelocitySolverSettings( hinge.constraint.strength, 1.0f );
	m_pConstraint->SetLegacyBreakLimits( m_LinearBreakImpulse, m_AngularBreakImpulse );
	m_pConstraint->SetEnabled( !pGroup && hinge.constraint.isActive );

	static_cast< JPH::HingeConstraint * >( m_pConstraint.GetPtr() )->SetLegacyMotor(
		DEG2RAD( hinge.hingeAxis.angularVelocity ), maxFrictionTorque );

	RegisterWithPhysicsSystem();
}

//-------------------------------------------------------------------------------------------------
// Sliding
//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::InitialiseSliding( IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding )
{
	SetGroup( pGroup );
	m_ConstraintType = CONSTRAINT_SLIDING;
	SetBreakableParams( sliding.constraint );

	// Get our bodies
	JPH::Body *refBody = m_pObjReference->GetBody();
	JPH::Body *attBody = m_pObjAttached->GetBody();

	JPH::SliderConstraintSettings settings;
	settings.mNumVelocityStepsOverride = vjolt_constraint_velocity_substeps.GetInt();
	settings.mNumPositionStepsOverride = vjolt_constraint_position_substeps.GetInt();
	settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	settings.mAutoDetectPoint = false;

	const JPH::Vec3 sourceAxis = SourceToJolt::Unitless( sliding.slideAxisRef );
	if ( sourceAxis.LengthSq() <= Square( FLT_EPSILON ) )
		Log_Warning( LOG_VJolt, "Sliding constraint has a zero reference axis; using local X.\n" );

	const JoltSliderConstraintFrames frames = BuildJoltSliderConstraintFrames(
		SourceToJolt::Matrix( sliding.attachedRefXform ),
		sourceAxis,
		refBody->GetShape()->GetCenterOfMass(),
		attBody->GetShape()->GetCenterOfMass() );
	settings.mPoint1 = frames.m_PointReferenceCOM;
	settings.mSliderAxis1 = frames.m_SliderAxisReferenceCOM;
	settings.mNormalAxis1 = frames.m_NormalAxisReferenceCOM;
	settings.mPoint2 = frames.m_PointAttachedCOM;
	settings.mSliderAxis2 = frames.m_SliderAxisAttachedCOM;
	settings.mNormalAxis2 = frames.m_NormalAxisAttachedCOM;

	if ( sliding.limitMin != sliding.limitMax )
	{
		settings.mLimitsMin = SourceToJolt::Distance( sliding.limitMin );
		settings.mLimitsMax = SourceToJolt::Distance( sliding.limitMax );
	}

	const float maxForce = fabsf( SourceToJolt::Distance( sliding.friction ) );

	m_pConstraint = settings.Create( *refBody, *attBody );
	m_pConstraint->SetLegacyVelocitySolverSettings( sliding.constraint.strength, 1.0f );
	m_pConstraint->SetLegacyBreakLimits( m_LinearBreakImpulse, m_AngularBreakImpulse );
	m_pConstraint->SetEnabled( !pGroup && sliding.constraint.isActive );

	// A normal prismatic is constructed directly in the local constraint system, which scales both
	// desired velocity and max force by epsilon (0.2). Source's breakable path is a real exception:
	// it constructs the underlying prismatic outside the system and wraps it afterwards, so the
	// motor keeps epsilon 1. Runtime SetLinearMotor also keeps epsilon 1.
	const float motorEpsilon = UsesHavanaBreakableWrapper( sliding.constraint ) ? 1.0f : 0.2f;
	static_cast<JPH::SliderConstraint *>( m_pConstraint.GetPtr() )->SetLegacyMotor(
		SourceToJolt::Distance( sliding.velocity ), maxForce, motorEpsilon );

	RegisterWithPhysicsSystem();
}

//-------------------------------------------------------------------------------------------------
// Ballsocket
//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::InitialiseBallsocket( IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket )
{
	SetGroup( pGroup );
	m_ConstraintType = CONSTRAINT_BALLSOCKET;
	SetBreakableParams( ballsocket.constraint );

	// Get our bodies
	JPH::Body *refBody = m_pObjReference->GetBody();
	JPH::Body *attBody = m_pObjAttached->GetBody();

	JPH::PointConstraintSettings settings;
	settings.mNumVelocityStepsOverride = vjolt_constraint_velocity_substeps.GetInt();
	settings.mNumPositionStepsOverride = vjolt_constraint_position_substeps.GetInt();
	settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	settings.mPoint1 = SourceToJolt::Distance( ballsocket.constraintPosition[0] ) - refBody->GetShape()->GetCenterOfMass();
	settings.mPoint2 = SourceToJolt::Distance( ballsocket.constraintPosition[1] ) - attBody->GetShape()->GetCenterOfMass();
	// Havana ball-socket is the exception to the other basic constraints: Source writes
	// constraint.strength to m_strength (the velocity term), while m_tau stays at 1.

	m_pConstraint = settings.Create( *refBody, *attBody );
	m_pConstraint->SetLegacyVelocitySolverSettings( 1.0f, ballsocket.constraint.strength );
	m_pConstraint->SetLegacyBreakLimits( m_LinearBreakImpulse, m_AngularBreakImpulse );
	m_pConstraint->SetEnabled( !pGroup && ballsocket.constraint.isActive );

	RegisterWithPhysicsSystem();
}

//-------------------------------------------------------------------------------------------------
// Fixed
//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::InitialiseFixed( IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed )
{
	SetGroup( pGroup );
	m_ConstraintType = CONSTRAINT_FIXED;
	SetBreakableParams( fixed.constraint );

	// Get our bodies
	JPH::Body *refBody = m_pObjReference->GetBody();
	JPH::Body *attBody = m_pObjAttached->GetBody();

	const JoltFixedConstraintFrames frames = BuildJoltFixedConstraintFrames(
		SourceToJolt::Matrix( fixed.attachedRefXform ),
		refBody->GetShape()->GetCenterOfMass(),
		attBody->GetShape()->GetCenterOfMass() );
	JPH::HavanaConstraintSettings settings;
	settings.mKind = JPH::EHavanaConstraintKind::Fixed;
	settings.mFrame1 = JPH::HavanaFromSourceTransform( BuildConstraintFrame(
		frames.m_PointReferenceCOM, frames.m_AxisXReferenceCOM, frames.m_AxisYReferenceCOM ) );
	settings.mFrame2 = JPH::HavanaFromSourceTransform( BuildConstraintFrame(
		frames.m_PointAttachedCOM, frames.m_AxisXAttachedCOM, frames.m_AxisYAttachedCOM ) );
	settings.mTau = fixed.constraint.strength;
	settings.mStrength = 1.0f;

	m_pConstraint = settings.Create( *refBody, *attBody );
	m_pConstraint->SetLegacyBreakLimits( m_LinearBreakImpulse, m_AngularBreakImpulse );
	m_pConstraint->SetEnabled( !pGroup && fixed.constraint.isActive );

	RegisterWithPhysicsSystem();
}

//-------------------------------------------------------------------------------------------------
// Length
//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::InitialiseLength( IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length )
{
	SetGroup( pGroup );
	m_ConstraintType = CONSTRAINT_LENGTH;
	SetBreakableParams( length.constraint );

	// Get our bodies
	JPH::Body *refBody = m_pObjReference->GetBody();
	JPH::Body *attBody = m_pObjAttached->GetBody();

	JPH::HavanaConstraintSettings settings;
	settings.mKind = JPH::EHavanaConstraintKind::Length;
	settings.mFrame1 = JPH::Mat44::sTranslation( JPH::HavanaFromSourceVector(
		SourceToJolt::Distance( length.objectPosition[0] ) - refBody->GetShape()->GetCenterOfMass() ) );
	settings.mFrame2 = JPH::Mat44::sTranslation( JPH::HavanaFromSourceVector(
		SourceToJolt::Distance( length.objectPosition[1] ) - attBody->GetShape()->GetCenterOfMass() ) );
	settings.mMinLength = SourceToJolt::Distance( length.minLength );
	settings.mMaxLength = SourceToJolt::Distance( length.totalLength );
	settings.mTau = length.constraint.strength;
	settings.mStrength = 0.5f;

	m_pConstraint = settings.Create( *refBody, *attBody );
	m_pConstraint->SetLegacyBreakLimits( m_LinearBreakImpulse, m_AngularBreakImpulse );
	m_pConstraint->SetEnabled( !pGroup && length.constraint.isActive );

	RegisterWithPhysicsSystem();
}

//-------------------------------------------------------------------------------------------------
// Pulley
//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::InitialisePulley( IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley )
{
	SetGroup( pGroup );
	m_ConstraintType = CONSTRAINT_PULLEY;
	SetBreakableParams( pulley.constraint );

	// Get our bodies
	JPH::Body* refBody = m_pObjReference->GetBody();
	JPH::Body* attBody = m_pObjAttached->GetBody();

	JPH::PulleyConstraintSettings settings;
	settings.mNumVelocityStepsOverride = vjolt_constraint_velocity_substeps.GetInt();
	settings.mNumPositionStepsOverride = vjolt_constraint_position_substeps.GetInt();
	settings.mSpace = JPH::EConstraintSpace::LocalToBodyCOM;
	settings.mBodyPoint1 = SourceToJolt::Distance( pulley.objectPosition[0] ) - refBody->GetShape()->GetCenterOfMass();
	settings.mBodyPoint2 = SourceToJolt::Distance( pulley.objectPosition[1] ) - attBody->GetShape()->GetCenterOfMass();

	settings.mFixedPoint1 = SourceToJolt::Distance( pulley.pulleyPosition[0] );
	settings.mFixedPoint2 = SourceToJolt::Distance( pulley.pulleyPosition[1] );

	settings.mRatio = pulley.gearRatio;

	settings.mMaxLength = SourceToJolt::Distance( pulley.totalLength ); // PiMoN: from my testing, it is the same value as Jolt would calculate automatically

	m_pConstraint = settings.Create( *refBody, *attBody );
	m_pConstraint->SetLegacyBreakLimits( m_LinearBreakImpulse, m_AngularBreakImpulse );
	m_pConstraint->SetEnabled( !pGroup && pulley.constraint.isActive );

	RegisterWithPhysicsSystem();
}

//-------------------------------------------------------------------------------------------------

bool JoltPhysicsConstraint::CheckBroken()
{
	if ( !m_pConstraint )
		return false;

	if ( m_LinearBreakImpulse <= 0.0f && m_AngularBreakImpulse <= 0.0f )
		return false;

	if ( !m_pConstraint->ConsumeLegacyBroken() )
		return false;

	// A broken Source constraint cannot break a second time. Remove it before the game callback,
	// which is allowed to destroy this constraint or related constraints re-entrantly.
	m_pPhysicsEnvironment->UnregisterBreakableConstraint( this );

	m_pConstraint->SetEnabled( false );
	m_pPhysicsEnvironment->NotifyConstraintDisabled( this );
	return true;
}

bool JoltPhysicsConstraint::MarkForDelete()
{
	if ( m_bMarkedForDelete )
		return false;
	m_bMarkedForDelete = true;
	return true;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::SaveConstraintSettings( JPH::StateRecorder &recorder )
{
	recorder.Write( m_ConstraintType );
	auto settings = m_pConstraint->GetConstraintSettings();
	settings->SaveBinaryState( recorder );
	m_pConstraint->SaveState( recorder );
	recorder.Write( m_pConstraint->GetLegacyVelocityBiasFactor() );
	recorder.Write( m_pConstraint->GetLegacyVelocityRelaxation() );
	recorder.Write( m_SourceForceLimit );
	recorder.Write( m_SourceTorqueLimit );
	recorder.Write( m_BreakStrength );
	recorder.Write( m_BodyMassScale[0] );
	recorder.Write( m_BodyMassScale[1] );
}

void JoltPhysicsConstraint::RestoreConstraintSettings( JPH::StateRecorder &recorder )
{
	float legacyVelocityBiasFactor = 0.0f;
	float legacyVelocityRelaxation = 1.0f;
	recorder.Read( legacyVelocityBiasFactor );
	recorder.Read( legacyVelocityRelaxation );
	m_pConstraint->SetLegacyVelocitySolverSettings( legacyVelocityBiasFactor, legacyVelocityRelaxation );

	constraint_breakableparams_t breakable;
	breakable.Defaults();
	recorder.Read( breakable.forceLimit );
	recorder.Read( breakable.torqueLimit );
	recorder.Read( breakable.strength );
	recorder.Read( breakable.bodyMassScale[0] );
	recorder.Read( breakable.bodyMassScale[1] );
	SetBreakableParams( breakable );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsConstraint::SetGroup( IPhysicsConstraintGroup *pGroup )
{
	UnregisterSolverConstraint();

	if ( m_pGroup )
	{
		m_pGroup->RemoveConstraint( this );
	}
	m_pGroup = static_cast< JoltPhysicsConstraintGroup * >( pGroup );
	if ( m_pGroup )
	{
		m_pGroup->AddConstraint( this );
		RegisterSolverConstraint();
	}
}

bool JoltPhysicsConstraint::UsesPrivateLegacySolver() const
{
	switch ( m_ConstraintType )
	{
		case CONSTRAINT_HINGE:
		case CONSTRAINT_FIXED:
		case CONSTRAINT_BALLSOCKET:
		case CONSTRAINT_SLIDING:
		case CONSTRAINT_LENGTH:
		case CONSTRAINT_RAGDOLL:
		case CONSTRAINT_PULLEY:
			return true;
		default:
			return false;
	}
}

void JoltPhysicsConstraint::RegisterSolverConstraint()
{
	if ( !m_pConstraint || m_bMarkedForDelete || m_bConstraintSystemDestroyed )
		return;
	// Group insertion can resize Havana's row storage, so it belongs to the same
	// safe boundary as PhysicsSystem registration, never a live response stack.
	if ( m_pPhysicsEnvironment->DeferPhysicsMutation( [this]() { RegisterSolverConstraint(); } ) )
		return;

	if ( m_pGroup )
		m_pGroup->AddSolverConstraint( m_pConstraint.GetPtr() );
	else if ( UsesPrivateLegacySolver() )
		m_PrivateSolver.AddConstraint( m_pConstraint.GetPtr() );
}

void JoltPhysicsConstraint::RegisterWithPhysicsSystem()
{
	if ( m_bRegisteredWithPhysicsSystem || m_bMarkedForDelete || !m_pConstraint
		|| ( m_pObjReference->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE )
		|| ( m_pObjAttached->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) )
		return;
	if ( m_pPhysicsEnvironment->DeferPhysicsMutation( [this]() { RegisterWithPhysicsSystem(); } ) )
		return;
	RegisterSolverConstraint();
	m_pPhysicsSystem->AddConstraint( m_pConstraint );
	m_bRegisteredWithPhysicsSystem = true;
}

void JoltPhysicsConstraint::UnregisterSolverConstraint()
{
	if ( !m_pConstraint || !m_pConstraint->GetLegacyConstraintGroup() )
		return;

	if ( m_pGroup )
		m_pGroup->RemoveSolverConstraint( m_pConstraint.GetPtr() );
	else
		m_PrivateSolver.RemoveConstraint( m_pConstraint.GetPtr() );
}

void JoltPhysicsConstraint::OnConstraintGroupDestroyed( JoltPhysicsConstraintGroup *pGroup )
{
	if ( m_pGroup != pGroup )
		return;

	UnregisterSolverConstraint();
	m_pGroup = nullptr;
	m_bConstraintSystemDestroyed = true;
	if ( m_pConstraint )
		m_pConstraint->SetEnabled( false );
}

void JoltPhysicsConstraint::SetEnabledFromConstraintGroup( JoltPhysicsConstraintGroup *pGroup, bool bEnabled )
{
	if ( m_pPhysicsEnvironment->IsInContactCallback() && m_pPhysicsEnvironment->DeferPhysicsMutation( [this, pGroup, bEnabled]() { SetEnabledFromConstraintGroup( pGroup, bEnabled ); } ) ) return;
	if ( m_pGroup == pGroup && !m_bConstraintSystemDestroyed && m_pConstraint )
		m_pConstraint->SetEnabled( bEnabled );
}

void JoltPhysicsConstraint::DestroyConstraint()
{
	m_pPhysicsEnvironment->UnregisterBreakableConstraint( this );

	// This must happen before m_pConstraint is cleared. Otherwise object destruction leaves the
	// group's LegacyConstraintGroup holding a pointer to a released Jolt constraint.
	UnregisterSolverConstraint();

	if ( m_pObjAttached )
	{
		m_pObjAttached->RemoveConstraintAttachment();
		m_pObjAttached->RemoveDestroyedListener( this );
		m_pObjAttached = nullptr;
	}
	if ( m_pObjReference )
	{
		m_pObjReference->RemoveConstraintAttachment();
		m_pObjReference->RemoveDestroyedListener( this );
		m_pObjReference = nullptr;
	}

	if ( m_pConstraint )
	{
		if ( m_bRegisteredWithPhysicsSystem )
			m_pPhysicsSystem->RemoveConstraint( m_pConstraint );
		m_bRegisteredWithPhysicsSystem = false;
		m_pConstraint = nullptr;
	}
}
