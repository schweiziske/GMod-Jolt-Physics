//=================================================================================================
//
// Source/Havana compatibility constraints backed by Jolt bodies.
//
//=================================================================================================

#if defined( VJOLT_HAVANA_STANDALONE )
#include <Jolt/Jolt.h>
#else
#include "cbase.h"
#endif

#include "vjolt_havana_constraints.h"

#include <Jolt/ObjectStream/TypeDeclarations.h>
#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Constraints/ConstraintPart/RotationEulerConstraintPart.h>
#include <Jolt/Physics/StateRecorder.h>
#ifdef JPH_DEBUG_RENDERER
#include <Jolt/Renderer/DebugRenderer.h>
#endif

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <cstdio>
#include <cstdlib>

namespace JPH
{

static bool IsHavanaRagdollSolveTraceEnabled()
{
	// Diagnostic environment variables are fixed before process startup. Cache this once so the
	// ragdoll hot path does not enter the CRT environment lookup for every axis and every scan.
	static const bool enabled = []()
	{
#if defined( _MSC_VER )
		char *value = nullptr;
		size_t length = 0;
		const bool found = _dupenv_s( &value, &length,
			"VJOLT_TRACE_HAVANA_RAGDOLL_SOLVES" ) == 0 && value != nullptr;
		std::free( value );
		return found;
#else
		return std::getenv( "VJOLT_TRACE_HAVANA_RAGDOLL_SOLVES" ) != nullptr;
#endif
	}();
	return enabled;
}

JPH_IMPLEMENT_SERIALIZABLE_VIRTUAL( HavanaConstraintSettings )
{
	JPH_ADD_BASE_CLASS( HavanaConstraintSettings, TwoBodyConstraintSettings )
}

static Mat44 HavanaBasisFromSource()
{
	// Source (x, y, z) -> IVP/Havana (x, -z, y).
	return Mat44(
		Vec4( 1, 0, 0, 0 ),
		Vec4( 0, 0, 1, 0 ),
		Vec4( 0, -1, 0, 0 ),
		Vec4( 0, 0, 0, 1 ) );
}

Vec3 HavanaFromSourceVector( Vec3Arg source )
{
	return Vec3( source.GetX(), -source.GetZ(), source.GetY() );
}

Vec3 SourceFromHavanaVector( Vec3Arg havana )
{
	return Vec3( havana.GetX(), havana.GetZ(), -havana.GetY() );
}

Mat44 HavanaFromSourceTransform( Mat44Arg source )
{
	const Mat44 basis = HavanaBasisFromSource();
	return basis * source * basis.InversedRotationTranslation();
}

Mat44 SourceFromHavanaTransform( Mat44Arg havana )
{
	const Mat44 basis = HavanaBasisFromSource();
	return basis.InversedRotationTranslation() * havana * basis;
}

TwoBodyConstraint *HavanaConstraintSettings::Create( Body &body1, Body &body2 ) const
{
	return new HavanaConstraint( body1, body2, *this );
}

void HavanaConstraintSettings::SaveBinaryState( StreamOut &stream ) const
{
	ConstraintSettings::SaveBinaryState( stream );
	stream.Write( mKind );
	stream.Write( mFrame1 );
	stream.Write( mFrame2 );
	stream.Write( mTau );
	stream.Write( mStrength );
	stream.Write( mMinLength );
	stream.Write( mMaxLength );
	stream.Write( mRagdollLimitMin );
	stream.Write( mRagdollLimitMax );
	stream.Write( mRagdollMotorVelocity );
	stream.Write( mRagdollMotorForce );
	stream.Write( mRagdollUseClockwiseRotations );
	stream.Write( mRagdollConstrainTranslation );
}

void HavanaConstraintSettings::RestoreBinaryState( StreamIn &stream )
{
	ConstraintSettings::RestoreBinaryState( stream );
	stream.Read( mKind );
	stream.Read( mFrame1 );
	stream.Read( mFrame2 );
	stream.Read( mTau );
	stream.Read( mStrength );
	stream.Read( mMinLength );
	stream.Read( mMaxLength );
	stream.Read( mRagdollLimitMin );
	stream.Read( mRagdollLimitMax );
	stream.Read( mRagdollMotorVelocity );
	stream.Read( mRagdollMotorForce );
	stream.Read( mRagdollUseClockwiseRotations );
	stream.Read( mRagdollConstrainTranslation );
}

static Mat44 BodyInverseInertiaSource( const Body &body )
{
	return body.IsDynamic()
		? body.GetMotionProperties()->GetInverseInertiaForRotation( Mat44::sRotation( body.GetRotation() ) )
		: Mat44::sZero();
}

static Mat44 BodyInverseInertiaHavana( const Body &body )
{
	return HavanaFromSourceTransform( BodyInverseInertiaSource( body ) );
}

static double PreciseDot( Vec3Arg a, Vec3Arg b )
{
	return double( a.GetX() ) * b.GetX() + double( a.GetY() ) * b.GetY()
		+ double( a.GetZ() ) * b.GetZ();
}

static void BuildPreciseInverseInertiaHavana( const Body &body, double (&outInertia)[3][3] )
{
	Vec3 axes[3] = { Vec3::sZero(), Vec3::sZero(), Vec3::sZero() };
	Vec3 diagonal = Vec3::sZero();
	if ( body.IsDynamic() )
	{
		const MotionProperties *motion = body.GetMotionProperties();
		// Follow Jolt's MultiplyWorldSpaceInverseInertiaByVector: R D R^T,
		// with forbidden world angular axes masked on both sides. Form the
		// tensor in double instead of promoting a rounded float world tensor.
		const Mat44 rotation = Mat44::sRotation( body.GetRotation() * motion->GetInertiaRotation() );
		diagonal = motion->GetInverseInertiaDiagonal();
		for ( uint axis = 0; axis < 3; ++axis )
			axes[axis] = HavanaFromSourceVector( motion->LockAngular( Vec3( rotation.GetColumn4(axis) ) ) );
	}
	for ( uint row = 0; row < 3; ++row )
		for ( uint column = 0; column <= row; ++column )
		{
			double value = 0.0;
			for ( uint axis = 0; axis < 3; ++axis )
				value += double(axes[axis][row]) * diagonal[axis] * axes[axis][column];
			outInertia[row][column] = outInertia[column][row] = value;
		}
}

bool HavanaConstraint::FixedJacobianSystem::Setup( const Body &body1, const Body &body2,
	float inverseMass1, float inverseMass2,
	Mat44Arg inverseInertia1, Mat44Arg inverseInertia2 )
{
	const Vec3 diagonal1 = body1.IsDynamic() ? body1.GetMotionProperties()->GetInverseInertiaDiagonal() : Vec3::sZero();
	const Vec3 diagonal2 = body2.IsDynamic() ? body2.GetMotionProperties()->GetInverseInertiaDiagonal() : Vec3::sZero();
	const double maximum1 = diagonal1.ReduceMax(), maximum2 = diagonal2.ReduceMax();
	const double minimum = double( diagonal1.ReduceMin() ) + diagonal2.ReduceMin();
	double leverage = 0.0;
	for ( uint row = 0; row < 3; ++row )
		leverage += maximum1 * PreciseDot( mRows[row].mAngular1, mRows[row].mAngular1 )
			+ maximum2 * PreciseDot( mRows[row].mAngular2, mRows[row].mAngular2 );
	// The three cross-product rows sum to twice the squared lever length. Select
	// extra precision before rotational terms can bury translational mass in float.
	// This changes arithmetic precision, never mass, damping, iteration count or limits.
	constexpr double cMaximumFloatCondition = 4096.0;
	mPrecise = !( leverage < 2.0 * cMaximumFloatCondition * ( double( inverseMass1 ) + inverseMass2 )
		&& maximum1 + maximum2 < cMaximumFloatCondition * minimum );
	if ( !mPrecise )
	{
		if ( JacobianSystem< 6 >::Setup( body1, body2, inverseMass1, inverseMass2, inverseInertia1, inverseInertia2 ) )
			return true;
		mPrecise = true;
	}
	mValid = false;
	BuildPreciseInverseInertiaHavana( body1, mInverseInertia1 );
	BuildPreciseInverseInertiaHavana( body2, mInverseInertia2 );
	double angular1[6][3], angular2[6][3];
	for ( uint column = 0; column < 6; ++column )
		for ( uint axis = 0; axis < 3; ++axis )
		{
			angular1[column][axis] = angular2[column][axis] = 0.0;
			for ( uint k = 0; k < 3; ++k )
			{
				angular1[column][axis] += mInverseInertia1[axis][k] * mRows[column].mAngular1[k];
				angular2[column][axis] += mInverseInertia2[axis][k] * mRows[column].mAngular2[k];
			}
		}

	// J M^-1 J^T is symmetric positive definite for an unconstrained dynamic body.
	// LDL^T needs only its lower triangle and avoids a full Gauss-Jordan inverse.
	// Both this Gram sum and the velocity update use the symmetric double tensor.
	// Mirroring one half of a Gram matrix built from an asymmetric float tensor
	// changes the equations; a captured distant weld amplified error 5.44x per step.
	// Double starts BEFORE the Gram sum: promoting an already rounded float matrix
	// cannot recover a small translational term lost beside a long lever arm.
	for ( uint row = 0; row < 6; ++row )
		for ( uint column = 0; column <= row; ++column )
		{
			const JacobianRow &r = mRows[row], &c = mRows[column];
			double value = double( inverseMass1 ) * PreciseDot( r.mLinear1, c.mLinear1 )
				+ double( inverseMass2 ) * PreciseDot( r.mLinear2, c.mLinear2 );
			for ( uint axis = 0; axis < 3; ++axis )
				value += double( r.mAngular1[axis] ) * angular1[column][axis]
					+ double( r.mAngular2[axis] ) * angular2[column][axis];
			for ( uint k = 0; k < column; ++k )
				value -= mLDLT[row][k] * mLDLT[column][k] * mLDLT[k][k];
			if ( row == column )
			{
				if ( !std::isfinite( value ) || value < 1.0e-16 )
					return false;
				mLDLT[row][row] = value;
				mInverseDiagonal[row] = 1.0 / value;
			}
			else
				mLDLT[row][column] = value * mInverseDiagonal[column];
		}
	mValid = true;
	return true;
}

bool HavanaConstraint::FixedJacobianSystem::SolveAndApply( Body &body1, Body &body2,
	const Vector< 6 > &correction, float biasFactor, float dampingFactor,
	const BodySolverData &bodies ) const
{
	if ( !mValid )
		return false;
	if ( !mPrecise )
	{
		const Vector< 6 > lambda = mEffectiveMass
			* ( biasFactor * correction - dampingFactor * GetVelocity( body1, body2 ) );
		return ApplyImpulse( body1, body2, lambda, bodies.mInverseMass1, bodies.mInverseMass2,
			bodies.mInverseInertiaSource1, bodies.mInverseInertiaSource2 );
	}
	const Vec3 linear1 = HavanaFromSourceVector( body1.GetLinearVelocity() );
	const Vec3 angular1 = HavanaFromSourceVector( body1.GetAngularVelocity() );
	const Vec3 linear2 = HavanaFromSourceVector( body2.GetLinearVelocity() );
	const Vec3 angular2 = HavanaFromSourceVector( body2.GetAngularVelocity() );
	double lambda[6];
	for ( uint row = 0; row < 6; ++row )
	{
		const JacobianRow &j = mRows[row];
		const double velocity = PreciseDot( j.mLinear1, linear1 ) + PreciseDot( j.mAngular1, angular1 )
			+ PreciseDot( j.mLinear2, linear2 ) + PreciseDot( j.mAngular2, angular2 );
		lambda[row] = double( biasFactor ) * correction[row] - double( dampingFactor ) * velocity;
		for ( uint column = 0; column < row; ++column )
			lambda[row] -= mLDLT[row][column] * lambda[column];
	}
	for ( uint row = 0; row < 6; ++row )
		lambda[row] *= mInverseDiagonal[row];
	bool nonzero = false;
	for ( int row = 5; row >= 0; --row )
	{
		for ( uint column = uint( row ) + 1; column < 6; ++column )
			lambda[row] -= mLDLT[column][row] * lambda[column];
		nonzero |= lambda[row] != 0.0;
	}
	if ( !nonzero )
		return false;

	// Translation and rotation rows may carry large, opposing torques. Do not
	// round each lambda/torque to float before cancelling them at the body.
	double impulses[4][3] = {};
	for ( uint row = 0; row < 6; ++row )
		for ( uint axis = 0; axis < 3; ++axis )
		{
			impulses[0][axis] += lambda[row] * mRows[row].mLinear1[axis];
			impulses[1][axis] += lambda[row] * mRows[row].mAngular1[axis];
			impulses[2][axis] += lambda[row] * mRows[row].mLinear2[axis];
			impulses[3][axis] += lambda[row] * mRows[row].mAngular2[axis];
		}
	auto apply = []( Body &body, const double *linear, const double *angular,
		float inverseMass, const double (&inverseInertia)[3][3] )
	{
		if ( !body.IsDynamic() )
			return;
		double deltaAngular[3] = {};
		for ( uint axis = 0; axis < 3; ++axis )
			for ( uint k = 0; k < 3; ++k )
				deltaAngular[axis] += inverseInertia[axis][k] * angular[k];
		MotionProperties *motion = body.GetMotionProperties();
		motion->AddLinearVelocityStep( Vec3( float( inverseMass * linear[0] ),
			float( inverseMass * linear[2] ), float( -inverseMass * linear[1] ) ) );
		motion->AddAngularVelocityStep( Vec3( float( deltaAngular[0] ),
			float( deltaAngular[2] ), float( -deltaAngular[1] ) ) );
	};
	apply( body1, impulses[0], impulses[1], bodies.mInverseMass1, mInverseInertia1 );
	apply( body2, impulses[2], impulses[3], bodies.mInverseMass2, mInverseInertia2 );
	return true;
}

template <uint N>
bool HavanaConstraint::JacobianSystem< N >::Setup( const Body &body1, const Body &body2,
	float inverseMass1, float inverseMass2,
	Mat44Arg inverseInertia1, Mat44Arg inverseInertia2 )
{
	// Each inertia/column product is shared by every matrix row. Retain the
	// original dot/add order so this removes redundant setup work, not solver precision.
	Vec3 inverseInertiaAngular1[N], inverseInertiaAngular2[N];
	for ( uint column = 0; column < N; ++column )
	{
		inverseInertiaAngular1[column] = inverseInertia1.Multiply3x3( mRows[column].mAngular1 );
		inverseInertiaAngular2[column] = inverseInertia2.Multiply3x3( mRows[column].mAngular2 );
	}
	Matrix< N, N > inverseEffectiveMass;
	for ( uint row = 0; row < N; ++row )
	{
		for ( uint column = 0; column < N; ++column )
		{
			const JacobianRow &r = mRows[row];
			const JacobianRow &c = mRows[column];
			inverseEffectiveMass( row, column ) =
				inverseMass1 * r.mLinear1.Dot( c.mLinear1 )
				+ r.mAngular1.Dot( inverseInertiaAngular1[column] )
				+ inverseMass2 * r.mLinear2.Dot( c.mLinear2 )
				+ r.mAngular2.Dot( inverseInertiaAngular2[column] );
		}
	}

	mValid = mEffectiveMass.SetInversed( inverseEffectiveMass );
	return mValid;
}

template <uint N>
Vector< N > HavanaConstraint::JacobianSystem< N >::GetVelocity( const Body &body1, const Body &body2 ) const
{
	const Vec3 linear1 = HavanaFromSourceVector( body1.GetLinearVelocity() );
	const Vec3 angular1 = HavanaFromSourceVector( body1.GetAngularVelocity() );
	const Vec3 linear2 = HavanaFromSourceVector( body2.GetLinearVelocity() );
	const Vec3 angular2 = HavanaFromSourceVector( body2.GetAngularVelocity() );

	Vector< N > velocity;
	for ( uint row = 0; row < N; ++row )
	{
		const JacobianRow &j = mRows[row];
		velocity[row] = j.mLinear1.Dot( linear1 ) + j.mAngular1.Dot( angular1 )
			+ j.mLinear2.Dot( linear2 ) + j.mAngular2.Dot( angular2 );
	}
	return velocity;
}

template <uint N>
bool HavanaConstraint::JacobianSystem< N >::ApplyImpulse( Body &body1, Body &body2,
	const Vector< N > &lambda, float inverseMass1, float inverseMass2,
	Mat44Arg inverseInertiaSource1, Mat44Arg inverseInertiaSource2 ) const
{
	if ( !mValid || lambda.IsZero() )
		return false;

	Vec3 linearImpulse1 = Vec3::sZero();
	Vec3 angularImpulse1 = Vec3::sZero();
	Vec3 linearImpulse2 = Vec3::sZero();
	Vec3 angularImpulse2 = Vec3::sZero();
	for ( uint row = 0; row < N; ++row )
	{
		linearImpulse1 += lambda[row] * mRows[row].mLinear1;
		angularImpulse1 += lambda[row] * mRows[row].mAngular1;
		linearImpulse2 += lambda[row] * mRows[row].mLinear2;
		angularImpulse2 += lambda[row] * mRows[row].mAngular2;
	}

	if ( body1.IsDynamic() )
	{
		MotionProperties *motion = body1.GetMotionProperties();
		motion->AddLinearVelocityStep( inverseMass1 * SourceFromHavanaVector( linearImpulse1 ) );
		motion->AddAngularVelocityStep(
			inverseInertiaSource1.Multiply3x3( SourceFromHavanaVector( angularImpulse1 ) ) );
	}
	if ( body2.IsDynamic() )
	{
		MotionProperties *motion = body2.GetMotionProperties();
		motion->AddLinearVelocityStep( inverseMass2 * SourceFromHavanaVector( linearImpulse2 ) );
		motion->AddAngularVelocityStep(
			inverseInertiaSource2.Multiply3x3( SourceFromHavanaVector( angularImpulse2 ) ) );
	}
	return true;
}

static Vec3 MatrixAxis( Mat44Arg matrix, int axis )
{
	switch ( axis )
	{
		case 0: return matrix.GetAxisX();
		case 1: return matrix.GetAxisY();
		default: return matrix.GetAxisZ();
	}
}

static void SetMatrixAxis( Mat44 &matrix, int axis, Vec3Arg value )
{
	matrix.SetColumn4( axis, Vec4( value, 0.0f ) );
}

static void RotateFrameLocally( Mat44 &frame, int axis, float angle )
{
	if ( angle == 0.0f )
		return;
	const Vec3 localAxis = axis == 0 ? Vec3::sAxisX() : axis == 1 ? Vec3::sAxisY() : Vec3::sAxisZ();
	frame = frame * Mat44::sRotation( Quat::sRotation( localAxis, angle ) );
}

HavanaConstraint::HavanaConstraint( Body &body1, Body &body2, const HavanaConstraintSettings &settings )
	: TwoBodyConstraint( body1, body2, settings )
	, mSettings( settings )
{
	if ( mSettings.mKind == EHavanaConstraintKind::Ragdoll )
		BuildRagdollRuntime();
}

void HavanaConstraint::NotifyShapeChanged( const BodyID &bodyID, Vec3Arg deltaCOM )
{
	const Vec3 deltaHavana = HavanaFromSourceVector( deltaCOM );
	Mat44 *frame = mBody1->GetID() == bodyID ? &mSettings.mFrame1 : &mSettings.mFrame2;
	frame->SetTranslation( frame->GetTranslation() - deltaHavana );

	if ( mSettings.mKind == EHavanaConstraintKind::Ragdoll )
	{
		Mat44 *original = mBody1->GetID() == bodyID ? &mOriginalRagdollFrame1 : &mOriginalRagdollFrame2;
		Mat44 *runtime = mBody1->GetID() == bodyID ? &mRagdollFrame1 : &mRagdollFrame2;
		original->SetTranslation( original->GetTranslation() - deltaHavana );
		runtime->SetTranslation( runtime->GetTranslation() - deltaHavana );
	}
}

void HavanaConstraint::SetupVelocityConstraint( float )
{
	// Fixed may prepare geometry into its group's storage through Jolt's setup jobs.
	// Velocity-dependent setup stays interleaved with the controller's impulse solves,
	// so predictive rope limits observe the preceding compatibility velocity phases.
}

void HavanaConstraint::ResetWarmStart()
{
}

void HavanaConstraint::WarmStartVelocityConstraint( float )
{
	// Havana local constraint systems do not carry Jolt-style accumulated lambdas between frames.
}

bool HavanaConstraint::SolveVelocityConstraint( float deltaTime )
{
	// Every adapter-created Havana constraint belongs to either an explicit Source local
	// constraint system or its one-member private system. Those systems provide the transient
	// scratch storage through SetupAndSolveLegacyVelocityConstraint / StepLegacyVelocityConstraint.
	// Reaching Jolt's ordinary per-constraint path would lose that storage and is a wiring error.
	JPH_ASSERT( GetLegacyConstraintGroup() == nullptr );
	(void)deltaTime;
	return false;
}

size_t HavanaConstraint::GetLegacySolverStorageSize() const
{
	switch ( mSettings.mKind )
	{
		case EHavanaConstraintKind::Fixed: return sizeof( FixedSolverStorage );
		case EHavanaConstraintKind::Length: return sizeof( LengthSolverStorage );
		case EHavanaConstraintKind::Ragdoll: return sizeof( RagdollSolverStorage );
	}
	return 0;
}

size_t HavanaConstraint::GetLegacySolverStorageAlignment() const
{
	switch ( mSettings.mKind )
	{
		case EHavanaConstraintKind::Fixed: return alignof( FixedSolverStorage );
		case EHavanaConstraintKind::Length: return alignof( LengthSolverStorage );
		case EHavanaConstraintKind::Ragdoll: return alignof( RagdollSolverStorage );
	}
	return 1;
}

void HavanaConstraint::InitializeLegacySolverStorage( void *storage ) const
{
	switch ( mSettings.mKind )
	{
		case EHavanaConstraintKind::Fixed: new ( storage ) FixedSolverStorage(); break;
		case EHavanaConstraintKind::Length: new ( storage ) LengthSolverStorage(); break;
		case EHavanaConstraintKind::Ragdoll: new ( storage ) RagdollSolverStorage(); break;
	}
}

void HavanaConstraint::DestroyLegacySolverStorage( void *storage ) const
{
	switch ( mSettings.mKind )
	{
		case EHavanaConstraintKind::Fixed: static_cast< FixedSolverStorage * >( storage )->~FixedSolverStorage(); break;
		case EHavanaConstraintKind::Length: static_cast< LengthSolverStorage * >( storage )->~LengthSolverStorage(); break;
		case EHavanaConstraintKind::Ragdoll: static_cast< RagdollSolverStorage * >( storage )->~RagdollSolverStorage(); break;
	}
}

bool HavanaConstraint::CanPrepareLegacyVelocityConstraint() const
{
	// Fixed geometry is independent of velocity. Length/Ragdoll predictive activation is not.
	return mSettings.mKind == EHavanaConstraintKind::Fixed;
}

void HavanaConstraint::PrepareLegacyVelocityConstraint( void *storage, float deltaTime )
{
	JPH_ASSERT( CanPrepareLegacyVelocityConstraint() );
	auto &work = *static_cast< FixedSolverStorage * >( storage );
	PrepareBodySolverData( work.mBodies );
	SetupFixed( deltaTime, work );
	// A failed preparation is also complete. Keep mJacobian.mValid false instead of
	// repeating the same matrix calculation when the controller consumes this entry.
	work.mPrepared = true;
}

bool HavanaConstraint::SetupAndSolveLegacyVelocityConstraint( void *storage, float deltaTime,
	float tauFactor, float dampingFactor )
{
	bool setup = false;
	bool applied = false;
	switch ( mSettings.mKind )
	{
		case EHavanaConstraintKind::Fixed:
		{
			auto &work = *static_cast< FixedSolverStorage * >( storage );
			if ( work.mPrepared && work.mBodies.mRevision == mBodyPropertiesRevision )
				setup = work.mJacobian.mValid;
			else
			{
				PrepareBodySolverData( work.mBodies );
				setup = SetupFixed( deltaTime, work );
			}
			work.mPrepared = false;
			if ( setup ) applied = SolveFixed( work, tauFactor, dampingFactor, true );
			break;
		}
		case EHavanaConstraintKind::Length:
		{
			auto &work = *static_cast< LengthSolverStorage * >( storage );
			PrepareBodySolverData( work.mBodies );
			setup = SetupLength( deltaTime, work );
			if ( setup ) applied = SolveLength( work, tauFactor, dampingFactor );
			break;
		}
		case EHavanaConstraintKind::Ragdoll:
		{
			auto &work = *static_cast< RagdollSolverStorage * >( storage );
			PrepareBodySolverData( work.mBodies );
			setup = SetupRagdoll( deltaTime, work );
			if ( setup ) applied = SolveRagdoll( deltaTime, work,
				tauFactor, dampingFactor, true );
			break;
		}
	}
	return applied;
}

bool HavanaConstraint::StepLegacyVelocityConstraint( void *storage, float deltaTime,
	float tauFactor, float dampingFactor )
{
	const BodySolverData *bodies = mSettings.mKind == EHavanaConstraintKind::Fixed
		? &static_cast<FixedSolverStorage *>( storage )->mBodies
		: mSettings.mKind == EHavanaConstraintKind::Length
			? &static_cast<LengthSolverStorage *>( storage )->mBodies
			: &static_cast<RagdollSolverStorage *>( storage )->mBodies;
	if ( bodies->mRevision != mBodyPropertiesRevision )
		return SetupAndSolveLegacyVelocityConstraint( storage, deltaTime, tauFactor, dampingFactor );
	bool applied = false;
	switch ( mSettings.mKind )
	{
		case EHavanaConstraintKind::Fixed:
			applied = SolveFixed( *static_cast< FixedSolverStorage * >( storage ),
				tauFactor, dampingFactor, false );
			break;
		case EHavanaConstraintKind::Length:
			applied = SolveLength( *static_cast< LengthSolverStorage * >( storage ),
				tauFactor, dampingFactor );
			break;
		case EHavanaConstraintKind::Ragdoll:
			applied = SolveRagdoll( deltaTime, *static_cast< RagdollSolverStorage * >( storage ),
				tauFactor, dampingFactor, false );
			break;
	}
	return applied;
}

bool HavanaConstraint::SolvePositionConstraint( float, float )
{
	// Havana constraints are velocity controllers. IVP does not run a later split-position
	// correction for these limits, so feeding them into Jolt's position pass changes their
	// predictive semantics and over-constrains loaded simfphys wheels.
	return false;
}

bool HavanaConstraint::SolvePositionConstraintInLegacyGroup( float deltaTime, float baumgarte )
{
	(void)deltaTime;
	(void)baumgarte;
	return false;
}

bool HavanaConstraint::SolvePostContactVelocityConstraintInLegacyGroup( float deltaTime )
{
	if ( !RequiresPostContactLegacySolve() )
		return false;

	// A mixed Source LCS has already run at the ordinary controller stage. Jolt contact friction
	// can add angular velocity in later outer iterations, so rebuild the real Havana ragdoll work
	// data and perform one ordinary setup solve against that final velocity. Do not replace this
	// with a three-axis lock: the cone/plane degeneracies are observable wheel semantics.
	RagdollSolverStorage work;
	PrepareBodySolverData( work.mBodies );
	if ( !SetupRagdoll( deltaTime, work ) )
		return false;
	return SolveRagdoll( deltaTime, work, 1.0f, 1.0f, true );
}

bool HavanaConstraint::RequiresPostContactLegacySolve() const
{
	if ( mSettings.mKind != EHavanaConstraintKind::Ragdoll )
		return false;

	// Tiny non-zero limits are predictive in Havana: dynamic friction has already generated the
	// wheel's rolling angular velocity when the constraint controller runs. Jolt spreads contact
	// friction over outer iterations, so these constraints must observe the completed contact pass.
	constexpr float kMicroAngle = DegreesToRadians( 0.5f );
	for ( int axis = 0; axis < 3; ++axis )
	{
		const float minimum = mSettings.mRagdollLimitMin[axis];
		const float maximum = mSettings.mRagdollLimitMax[axis];
		if ( minimum >= maximum || minimum <= -kMicroAngle || maximum >= kMicroAngle )
			return false;
	}
	return true;
}

Mat44 HavanaConstraint::GetConstraintToBody1Matrix() const
{
	return SourceFromHavanaTransform( mSettings.mKind == EHavanaConstraintKind::Ragdoll ? mRagdollFrame1 : mSettings.mFrame1 );
}

Mat44 HavanaConstraint::GetConstraintToBody2Matrix() const
{
	return SourceFromHavanaTransform( mSettings.mKind == EHavanaConstraintKind::Ragdoll ? mRagdollFrame2 : mSettings.mFrame2 );
}

Ref<ConstraintSettings> HavanaConstraint::GetConstraintSettings() const
{
	HavanaConstraintSettings *settings = new HavanaConstraintSettings( mSettings );
	ToConstraintSettings( *settings );
	return settings;
}

void HavanaConstraint::SaveState( StateRecorder &stream ) const
{
	TwoBodyConstraint::SaveState( stream );
	for ( const RagdollLimit &limit : mRagdollLimits )
		stream.Write( limit.mReferencePosition );
}

void HavanaConstraint::RestoreState( StateRecorder &stream )
{
	TwoBodyConstraint::RestoreState( stream );
	for ( RagdollLimit &limit : mRagdollLimits )
		stream.Read( limit.mReferencePosition );
}

#ifdef JPH_DEBUG_RENDERER
void HavanaConstraint::DrawConstraint( DebugRenderer *renderer ) const
{
	const RVec3 point1 = mBody1->GetCenterOfMassTransform() * GetConstraintToBody1Matrix().GetTranslation();
	const RVec3 point2 = mBody2->GetCenterOfMassTransform() * GetConstraintToBody2Matrix().GetTranslation();
	renderer->DrawLine( point1, point2, Color::sOrange );
}
#endif

Mat44 HavanaConstraint::GetBodyTransformHavana( const Body &body ) const
{
	return HavanaFromSourceTransform( Mat44::sRotationTranslation(
		body.GetRotation(), Vec3( body.GetCenterOfMassPosition() ) ) );
}

Mat44 HavanaConstraint::GetBodyInverseInertiaHavana( const Body &body ) const
{
	return BodyInverseInertiaHavana( body );
}

Quat HavanaConstraint::PredictBodyRotation( const Body &body, float deltaTime ) const
{
	const Vec3 angularStep = body.GetAngularVelocity() * deltaTime;
	const float length = angularStep.Length();
	return length > 1.0e-6f
		? ( Quat::sRotation( angularStep / length, length ) * body.GetRotation() ).Normalized()
		: body.GetRotation();
}

void HavanaConstraint::PrepareBodySolverData( BodySolverData &storage ) const
{
	storage.mRevision = mBodyPropertiesRevision;
	storage.mInverseMass1 = mBody1->IsDynamic()
		? mBody1->GetMotionProperties()->GetInverseMass() : 0.0f;
	storage.mInverseMass2 = mBody2->IsDynamic()
		? mBody2->GetMotionProperties()->GetInverseMass() : 0.0f;
	storage.mInverseInertiaSource1 = BodyInverseInertiaSource( *mBody1 );
	storage.mInverseInertiaSource2 = BodyInverseInertiaSource( *mBody2 );
	storage.mInverseInertiaHavana1 = HavanaFromSourceTransform( storage.mInverseInertiaSource1 );
	storage.mInverseInertiaHavana2 = HavanaFromSourceTransform( storage.mInverseInertiaSource2 );
}

bool HavanaConstraint::SetupFixed( float deltaTime, FixedSolverStorage &storage )
{
	if ( deltaTime <= 0.0f )
		return false;

	const Mat44 transform1 = GetBodyTransformHavana( *mBody1 );
	const Mat44 transform2 = GetBodyTransformHavana( *mBody2 );
	// For Fixed, frame 1 is the requested attached-object transform in reference-COM space and
	// frame 2 is the attached-object origin in attached-COM space. This reproduces Havana's
	// t0 * m_transform_os_ks versus t1 comparison exactly. In particular, the Jacobian anchor is
	// the target attached-object origin; using any other coincident weld frame changes the lever
	// arms as soon as the bodies drift apart.
	const Mat44 targetAttachedObject = transform1 * mSettings.mFrame1;
	const Mat44 actualAttachedObject = transform2 * mSettings.mFrame2;
	const Vec3 anchor = targetAttachedObject.GetTranslation();
	const Vec3 relative1 = anchor - transform1.GetTranslation();
	const Vec3 relative2 = anchor - transform2.GetTranslation();

	const Vec3 worldAxes[3] = { Vec3::sAxisX(), Vec3::sAxisY(), Vec3::sAxisZ() };
	for ( int axis = 0; axis < 3; ++axis )
	{
		JacobianRow &row = storage.mJacobian.mRows[axis];
		row.mLinear1 = worldAxes[axis];
		row.mAngular1 = relative1.Cross( row.mLinear1 );
		row.mLinear2 = -worldAxes[axis];
		row.mAngular2 = relative2.Cross( row.mLinear2 );
	}
	for ( int axis = 0; axis < 3; ++axis )
	{
		JacobianRow &row = storage.mJacobian.mRows[3 + axis];
		const Vec3 jointAxis = MatrixAxis( targetAttachedObject, axis );
		row.mAngular1 = jointAxis;
		row.mAngular2 = -jointAxis;
	}

	if ( !storage.mJacobian.Setup( *mBody1, *mBody2, storage.mBodies.mInverseMass1, storage.mBodies.mInverseMass2,
		storage.mBodies.mInverseInertiaHavana1, storage.mBodies.mInverseInertiaHavana2 ) )
		return false;

	const Vec3 positionError = actualAttachedObject.GetTranslation() - targetAttachedObject.GetTranslation();
	storage.mCorrection[0] = positionError.GetX() / deltaTime;
	storage.mCorrection[1] = positionError.GetY() / deltaTime;
	storage.mCorrection[2] = positionError.GetZ() / deltaTime;
	storage.mCorrection[3] = actualAttachedObject.GetAxisY().Dot( targetAttachedObject.GetAxisZ() ) / deltaTime;
	storage.mCorrection[4] = -actualAttachedObject.GetAxisX().Dot( targetAttachedObject.GetAxisZ() ) / deltaTime;
	storage.mCorrection[5] = actualAttachedObject.GetAxisX().Dot( targetAttachedObject.GetAxisY() ) / deltaTime;
	return true;
}

bool HavanaConstraint::SolveFixed( FixedSolverStorage &storage, float tauFactor,
	float dampingFactor, bool firstSolve )
{
	if ( !storage.mJacobian.mValid )
		return false;
	const float biasFactor = ( firstSolve ? mSettings.mTau : 1.0f ) * tauFactor;
	return storage.mJacobian.SolveAndApply( *mBody1, *mBody2, storage.mCorrection,
		biasFactor, mSettings.mStrength * dampingFactor, storage.mBodies );
}

static float HavanaIntervalDistance( float current, float minimum, float maximum )
{
	if ( minimum > current )
		return current - minimum;
	if ( current <= maximum )
		return 0.0f;
	return current - maximum;
}

bool HavanaConstraint::SetupLength( float deltaTime, LengthSolverStorage &storage )
{
	if ( deltaTime <= 0.0f )
		return false;

	const Mat44 transform1 = GetBodyTransformHavana( *mBody1 );
	const Mat44 transform2 = GetBodyTransformHavana( *mBody2 );
	const Vec3 point1 = Vec3( transform1 * mSettings.mFrame1.GetTranslation() );
	const Vec3 point2 = Vec3( transform2 * mSettings.mFrame2.GetTranslation() );
	Vec3 direction = point2 - point1;
	const float currentLength = direction.Length();
	if ( currentLength <= FLT_EPSILON )
	{
		storage.mSkipSolve = true;
		return false;
	}
	direction /= currentLength;

	const float currentDistance = HavanaIntervalDistance( currentLength, mSettings.mMinLength, mSettings.mMaxLength );
	storage.mSkipSolve = false;
	if ( mSettings.mMinLength != mSettings.mMaxLength && currentDistance == 0.0f )
	{
		const Vec3 nextPosition1 = Vec3( mBody1->GetCenterOfMassPosition() ) + mBody1->GetLinearVelocity() * deltaTime;
		const Vec3 nextPosition2 = Vec3( mBody2->GetCenterOfMassPosition() ) + mBody2->GetLinearVelocity() * deltaTime;
		const Mat44 nextTransform1 = HavanaFromSourceTransform( Mat44::sRotationTranslation(
			PredictBodyRotation( *mBody1, deltaTime ), nextPosition1 ) );
		const Mat44 nextTransform2 = HavanaFromSourceTransform( Mat44::sRotationTranslation(
			PredictBodyRotation( *mBody2, deltaTime ), nextPosition2 ) );
		const Vec3 nextPoint1 = Vec3( nextTransform1 * mSettings.mFrame1.GetTranslation() );
		const Vec3 nextPoint2 = Vec3( nextTransform2 * mSettings.mFrame2.GetTranslation() );
		const float nextDistance = HavanaIntervalDistance(
			( nextPoint2 - nextPoint1 ).Length(), mSettings.mMinLength, mSettings.mMaxLength );
		storage.mSkipSolve = nextDistance == 0.0f;
		if ( storage.mSkipSolve )
			return true;
	}

	JacobianRow &row = storage.mJacobian.mRows[0];
	row.mLinear1 = direction;
	row.mAngular1 = ( point1 - transform1.GetTranslation() ).Cross( direction );
	row.mLinear2 = -direction;
	// Havana intentionally evaluates both body Jacobians at point1 for stiff-spring constraints.
	row.mAngular2 = ( point1 - transform2.GetTranslation() ).Cross( -direction );
	if ( !storage.mJacobian.Setup( *mBody1, *mBody2,
		storage.mBodies.mInverseMass1, storage.mBodies.mInverseMass2,
		storage.mBodies.mInverseInertiaHavana1, storage.mBodies.mInverseInertiaHavana2 ) )
		return false;
	storage.mCorrection[0] = currentDistance / deltaTime;
	return true;
}

bool HavanaConstraint::SolveLength( LengthSolverStorage &storage, float tauFactor,
	float dampingFactor )
{
	if ( storage.mSkipSolve || !storage.mJacobian.mValid )
		return false;
	const Vector< 1 > lambda = storage.mJacobian.mEffectiveMass
		* ( tauFactor * mSettings.mTau * storage.mCorrection
			- dampingFactor * mSettings.mStrength * storage.mJacobian.GetVelocity( *mBody1, *mBody2 ) );
	return storage.mJacobian.ApplyImpulse( *mBody1, *mBody2, lambda,
		storage.mBodies.mInverseMass1, storage.mBodies.mInverseMass2,
		storage.mBodies.mInverseInertiaSource1, storage.mBodies.mInverseInertiaSource2 );
}

void HavanaConstraint::BuildRagdollRuntime()
{
	mOriginalRagdollFrame1 = mSettings.mFrame1;
	mOriginalRagdollFrame2 = mSettings.mFrame2;
	mRagdollFrame1 = mSettings.mFrame1;
	mRagdollFrame2 = mSettings.mFrame2;
	mRagdollAxisMap = { 0, 1, 2 };
	mRagdollConeCenter = 0.0f;

	struct RawLimit
	{
		float mMin = 0.0f;
		float mMax = 0.0f;
	};
	std::array< RawLimit, 3 > inputLimits;
	static constexpr int sourceToHavanaAxis[3] = { 0, 2, 1 };
	for ( int sourceAxis = 0; sourceAxis < 3; ++sourceAxis )
	{
		const int havanaAxis = sourceToHavanaAxis[sourceAxis];
		if ( sourceAxis == 2 )
		{
			inputLimits[havanaAxis].mMin = -mSettings.mRagdollLimitMax[sourceAxis];
			inputLimits[havanaAxis].mMax = -mSettings.mRagdollLimitMin[sourceAxis];
		}
		else
		{
			inputLimits[havanaAxis].mMin = mSettings.mRagdollLimitMin[sourceAxis];
			inputLimits[havanaAxis].mMax = mSettings.mRagdollLimitMax[sourceAxis];
		}
	}
	if ( mSettings.mRagdollUseClockwiseRotations )
	{
		for ( RawLimit &limit : inputLimits )
		{
			const float minimum = limit.mMin;
			limit.mMin = -limit.mMax;
			limit.mMax = -minimum;
		}
	}

	float limitDifference[3];
	float limitMiddle[3];
	int degreesOfFreedom = 0;
	for ( int axis = 0; axis < 3; ++axis )
	{
		limitDifference[axis] = inputLimits[axis].mMax - inputLimits[axis].mMin;
		limitMiddle[axis] = 0.5f * ( inputLimits[axis].mMax + inputLimits[axis].mMin );
		if ( limitDifference[axis] > FLT_EPSILON )
			++degreesOfFreedom;
		mRagdollLimits[axis] = {};
	}

	if ( degreesOfFreedom == 0 )
	{
		mRagdollLimits[1].mLimitMin = -0.1f;
		mRagdollLimits[1].mLimitMax = 0.1f;
	}
	else if ( degreesOfFreedom == 1 )
	{
		int freeAxis = 0;
		for ( int axis = 0; axis < 3; ++axis )
			if ( limitDifference[axis] > FLT_EPSILON )
				freeAxis = axis;
		Mat44 rotated1 = mRagdollFrame1;
		RotateFrameLocally( rotated1, freeAxis, limitMiddle[freeAxis] );
		const int nextAxis = freeAxis == 2 ? 0 : freeAxis + 1;
		const int previousAxis = freeAxis == 0 ? 2 : freeAxis - 1;
		SetMatrixAxis( mRagdollFrame1, 0, MatrixAxis( rotated1, nextAxis ) );
		SetMatrixAxis( mRagdollFrame2, 0, MatrixAxis( mOriginalRagdollFrame2, nextAxis ) );
		SetMatrixAxis( mRagdollFrame1, 1, MatrixAxis( rotated1, previousAxis ) );
		SetMatrixAxis( mRagdollFrame2, 1, MatrixAxis( mOriginalRagdollFrame2, previousAxis ) );
		SetMatrixAxis( mRagdollFrame1, 2, MatrixAxis( rotated1, freeAxis ) );
		SetMatrixAxis( mRagdollFrame2, 2, MatrixAxis( mOriginalRagdollFrame2, freeAxis ) );
		if ( limitDifference[freeAxis] > JPH_PI && limitDifference[freeAxis] < 2.0f * JPH_PI )
			limitDifference[freeAxis] = JPH_PI;
		mRagdollLimits[1].mLimitMin = -0.5f * limitDifference[freeAxis];
		mRagdollLimits[1].mLimitMax = 0.5f * limitDifference[freeAxis];
		mRagdollAxisMap = { uint8( nextAxis ), uint8( freeAxis ), uint8( previousAxis ) };
	}
	else
	{
		const Mat44 bodyTransform1 = GetBodyTransformHavana( *mBody1 );
		const Mat44 bodyTransform2 = GetBodyTransformHavana( *mBody2 );
		const Mat44 worldFrame1 = bodyTransform1 * mRagdollFrame1;
		const Mat44 worldFrame2 = bodyTransform2 * mRagdollFrame2;
		const Vec3 relativePosition1 = worldFrame1.GetTranslation() - bodyTransform1.GetTranslation();
		const Vec3 relativePosition2 = worldFrame2.GetTranslation() - bodyTransform2.GetTranslation();
		const Mat44 inverseInertia1 = GetBodyInverseInertiaHavana( *mBody1 );
		const float inverseMass1 = mBody1->IsDynamic() ? mBody1->GetMotionProperties()->GetInverseMass() : 0.0f;
		const float inverseMass2 = mBody2->IsDynamic() ? mBody2->GetMotionProperties()->GetInverseMass() : 0.0f;

		float greatestInverseVirtualMass = -1.0f;
		int twistAxis = 0;
		for ( int axis = 0; axis < 3; ++axis )
		{
			const Vec3 axisReference = MatrixAxis( worldFrame1, axis );
			const Vec3 axisAttached = MatrixAxis( worldFrame2, axis );
			// Preserve the Havana builder's body-B query bug: both angular terms use body A.
			float inverseVirtualMass = 2.0f * axisReference.Dot( inverseInertia1.Multiply3x3( axisReference ) );
			inverseVirtualMass += inverseMass1 * relativePosition1.Cross( axisReference ).LengthSq();
			inverseVirtualMass += inverseMass2 * relativePosition2.Cross( axisAttached ).LengthSq();
			if ( inverseVirtualMass > greatestInverseVirtualMass )
			{
				greatestInverseVirtualMass = inverseVirtualMass;
				twistAxis = axis;
			}
		}

		int lowerAxis = twistAxis == 2 ? 0 : twistAxis + 1;
		int upperAxis = twistAxis == 0 ? 2 : twistAxis - 1;
		if ( limitDifference[lowerAxis] > limitDifference[upperAxis] )
			std::swap( lowerAxis, upperAxis );
		Mat44 rotated1 = mRagdollFrame1;
		RotateFrameLocally( rotated1, upperAxis, limitMiddle[upperAxis] );
		SetMatrixAxis( mRagdollFrame1, 0, MatrixAxis( rotated1, twistAxis ) );
		SetMatrixAxis( mRagdollFrame2, 0, MatrixAxis( mOriginalRagdollFrame2, twistAxis ) );
		SetMatrixAxis( mRagdollFrame1, 1, MatrixAxis( rotated1, lowerAxis ) );
		SetMatrixAxis( mRagdollFrame2, 1, MatrixAxis( mOriginalRagdollFrame2, lowerAxis ) );
		SetMatrixAxis( mRagdollFrame1, 2, MatrixAxis( rotated1, upperAxis ) );
		SetMatrixAxis( mRagdollFrame2, 2, MatrixAxis( mOriginalRagdollFrame2, upperAxis ) );
		mRagdollLimits[0].mLimitMin = inputLimits[twistAxis].mMin;
		mRagdollLimits[0].mLimitMax = inputLimits[twistAxis].mMax;
		mRagdollLimits[1].mLimitMin = -0.5f * limitDifference[upperAxis];
		mRagdollLimits[1].mLimitMax = 0.5f * limitDifference[upperAxis];
		mRagdollLimits[2].mLimitMin = inputLimits[lowerAxis].mMin;
		mRagdollLimits[2].mLimitMax = inputLimits[lowerAxis].mMax;
		mRagdollAxisMap = { uint8( twistAxis ), uint8( upperAxis ), uint8( lowerAxis ) };
	}

	int reverseAxisMap[3];
	for ( int axis = 0; axis < 3; ++axis )
		reverseAxisMap[mRagdollAxisMap[axis]] = axis;
	for ( int sourceAxis = 0; sourceAxis < 3; ++sourceAxis )
	{
		const int havanaAxis = sourceToHavanaAxis[sourceAxis];
		RagdollLimit &limit = mRagdollLimits[reverseAxisMap[havanaAxis]];
		limit.mDesiredVelocity = 0.2f * mSettings.mRagdollMotorVelocity[sourceAxis];
		limit.mJointFriction = 0.2f * mSettings.mRagdollMotorForce[sourceAxis];
		limit.mFrictionEnabled = limit.mJointFriction != 0.0f;
		// GMod treats a full revolution (including exactly +/-180 degrees) as
		// unlimited. Keeping this row enabled predicts a stop at the angle-wrap
		// seam and stalls LVS wheels despite a continuously active drive motor.
		if ( limit.mLimitMax - limit.mLimitMin >= 2.0f * JPH_PI )
			limit.mLimitEnabled = false;
	}

	mRagdollConeCenter = 0.5f * ( mRagdollLimits[1].mLimitMax + mRagdollLimits[1].mLimitMin );
	const float coneDifference = mRagdollLimits[1].mLimitMax - mRagdollLimits[1].mLimitMin;
	if ( std::fabs( mRagdollConeCenter ) > 0.001f )
		RotateFrameLocally( mRagdollFrame1, 2, mRagdollConeCenter );
	mRagdollLimits[1].mLimitMin = std::cos( 0.5f * coneDifference );
	mRagdollLimits[1].mLimitMax = 100.0f;
	const float planeMinimum = mRagdollLimits[2].mLimitMin;
	const float planeMaximum = mRagdollLimits[2].mLimitMax;
	mRagdollLimits[2].mLimitMin = -std::sin( planeMaximum );
	mRagdollLimits[2].mLimitMax = -std::sin( planeMinimum );

}

bool HavanaConstraint::SetupRagdoll( float deltaTime, RagdollSolverStorage &storage )
{
	if ( deltaTime <= 0.0f )
		return false;

	const Mat44 bodyTransform1 = GetBodyTransformHavana( *mBody1 );
	const Mat44 bodyTransform2 = GetBodyTransformHavana( *mBody2 );
	const Mat44 frame1World = bodyTransform1 * mRagdollFrame1;
	const Mat44 frame2World = bodyTransform2 * mRagdollFrame2;
	storage.mTwistAxisReference = frame1World.GetAxisX();
	storage.mPlaneAxisReference = frame1World.GetAxisZ();
	storage.mTwistAxisAttached = frame2World.GetAxisX();
	storage.mTwistAxis = storage.mTwistAxisReference + storage.mTwistAxisAttached;
	const float twistLength = storage.mTwistAxis.Length();
	if ( twistLength > FLT_EPSILON )
	{
		storage.mTwistAxis /= twistLength;
		storage.mTwistFactor = 0.5f * twistLength;
	}
	else
	{
		storage.mTwistAxis = storage.mTwistAxisReference.NormalizedOr( Vec3::sAxisX() );
		storage.mTwistFactor = 0.0f;
	}

	Vec3 basisY = storage.mTwistAxis.Cross( storage.mPlaneAxisReference );
	basisY = basisY.NormalizedOr( storage.mTwistAxis.GetNormalizedPerpendicular() );
	const Vec3 basisZ = basisY.Cross( storage.mTwistAxis );
	const Vec3 attachedPerpendicular = frame2World.GetAxisZ();
	const Vec3 attachedInJoint(
		attachedPerpendicular.Dot( storage.mTwistAxis ),
		attachedPerpendicular.Dot( basisY ),
		attachedPerpendicular.Dot( basisZ ) );
	storage.mJointAngles.SetX( -std::atan2( attachedInJoint.GetY(), attachedInJoint.GetZ() ) );
	storage.mJointAngles.SetY( storage.mTwistAxisReference.Dot( storage.mTwistAxisAttached ) );
	storage.mJointAngles.SetZ( storage.mPlaneAxisReference.Dot( storage.mTwistAxisAttached ) );
	if ( IsHavanaRagdollSolveTraceEnabled() )
	{
		auto printVector = []( const char *name, Vec3Arg value )
		{
			std::fprintf( stderr, " %s=(%.9g,%.9g,%.9g)", name,
				value.GetX(), value.GetY(), value.GetZ() );
		};
		std::fprintf( stderr, "HAVANA_RAGDOLL_SETUP body1=%u body2=%u",
			mBody1->GetID().GetIndexAndSequenceNumber(),
			mBody2->GetID().GetIndexAndSequenceNumber() );
		printVector( "frame1_x", frame1World.GetAxisX() );
		printVector( "frame1_y", frame1World.GetAxisY() );
		printVector( "frame1_z", frame1World.GetAxisZ() );
		printVector( "frame2_x", frame2World.GetAxisX() );
		printVector( "frame2_y", frame2World.GetAxisY() );
		printVector( "frame2_z", frame2World.GetAxisZ() );
		printVector( "twist", storage.mTwistAxis );
		printVector( "basis_y", basisY );
		printVector( "basis_z", basisZ );
		printVector( "attached_joint", attachedInJoint );
		std::fprintf( stderr,
			" twist_factor=%.9g joint=(%.9g,%.9g,%.9g) limits0=(%.9g,%.9g) limits1=(%.9g,%.9g) limits2=(%.9g,%.9g)\n",
			storage.mTwistFactor, storage.mJointAngles.GetX(), storage.mJointAngles.GetY(),
			storage.mJointAngles.GetZ(), mRagdollLimits[0].mLimitMin, mRagdollLimits[0].mLimitMax,
			mRagdollLimits[1].mLimitMin, mRagdollLimits[1].mLimitMax,
			mRagdollLimits[2].mLimitMin, mRagdollLimits[2].mLimitMax );
	}

	if ( mSettings.mRagdollConstrainTranslation )
	{
		const Vec3 point1 = frame1World.GetTranslation();
		const Vec3 point2 = frame2World.GetTranslation();
		const Vec3 relative1 = point1 - bodyTransform1.GetTranslation();
		const Vec3 relative2 = point2 - bodyTransform2.GetTranslation();
		const Vec3 axes[3] = { Vec3::sAxisX(), Vec3::sAxisY(), Vec3::sAxisZ() };
		for ( int axis = 0; axis < 3; ++axis )
		{
			JacobianRow &row = storage.mLinearJacobian.mRows[axis];
			row.mLinear1 = axes[axis];
			row.mAngular1 = relative1.Cross( row.mLinear1 );
			row.mLinear2 = -axes[axis];
			row.mAngular2 = relative2.Cross( row.mLinear2 );
		}
		if ( !storage.mLinearJacobian.Setup( *mBody1, *mBody2,
			storage.mBodies.mInverseMass1, storage.mBodies.mInverseMass2,
			storage.mBodies.mInverseInertiaHavana1, storage.mBodies.mInverseInertiaHavana2 ) )
			return false;
		const Vec3 error = point2 - point1;
		storage.mLinearCorrection[0] = error.GetX() / deltaTime;
		storage.mLinearCorrection[1] = error.GetY() / deltaTime;
		storage.mLinearCorrection[2] = error.GetZ() / deltaTime;
	}
	return true;
}

float HavanaConstraint::GetAngularInverseMassHavana( Vec3Arg axisHavana,
	const BodySolverData &storage ) const
{
	float inverseMass = 0.0f;
	if ( mBody1->IsDynamic() )
		inverseMass += axisHavana.Dot( storage.mInverseInertiaHavana1.Multiply3x3( axisHavana ) );
	if ( mBody2->IsDynamic() )
		inverseMass += axisHavana.Dot( storage.mInverseInertiaHavana2.Multiply3x3( axisHavana ) );
	return inverseMass;
}

bool HavanaConstraint::ApplyAngularImpulseHavana( Vec3Arg axisHavana, float impulse,
	const BodySolverData &storage )
{
	if ( impulse == 0.0f )
		return false;
	const Vec3 sourceImpulse = SourceFromHavanaVector( impulse * axisHavana );
	if ( mBody1->IsDynamic() )
		mBody1->GetMotionProperties()->AddAngularVelocityStep(
			storage.mInverseInertiaSource1.Multiply3x3( sourceImpulse ) );
	if ( mBody2->IsDynamic() )
		mBody2->GetMotionProperties()->AddAngularVelocityStep(
			storage.mInverseInertiaSource2.Multiply3x3( -sourceImpulse ) );
	return true;
}

bool HavanaConstraint::SolveRagdollAngularLimit( int limitIndex, Vec3Arg axisHavana, float angle,
	RagdollLimit &limit,
	float tauFactor, float dampingFactor, float deltaTime, bool firstSolve,
	RagdollSolverStorage &storage )
{
	const float inverseVirtualMass = GetAngularInverseMassHavana( axisHavana, storage.mBodies );
	if ( inverseVirtualMass <= FLT_EPSILON )
		return false;
	const float virtualMass = 1.0f / inverseVirtualMass;
	const Vec3 angular1 = HavanaFromSourceVector( mBody1->GetAngularVelocity() );
	const Vec3 angular2 = HavanaFromSourceVector( mBody2->GetAngularVelocity() );
	const float relativeVelocity = axisHavana.Dot( angular1 - angular2 );
	float impulse = 0.0f;
	float nextAngle = angle;
	int limitBranch = 0;

	if ( limit.mFrictionEnabled )
	{
		float angleError = limit.mReferencePosition - angle;
		limit.mReferencePosition += limit.mDesiredVelocity * deltaTime;
		if ( angleError > JPH_PI )
			angleError -= 2.0f * JPH_PI;
		else if ( angleError < -JPH_PI )
			angleError += 2.0f * JPH_PI;
		impulse = virtualMass * ( angleError * 0.8f * tauFactor / deltaTime - dampingFactor * relativeVelocity );
		if ( std::fabs( impulse ) / deltaTime > limit.mJointFriction )
		{
			const float factor = limit.mJointFriction * deltaTime / std::fabs( impulse );
			impulse *= factor;
			limit.mReferencePosition -= ( 1.0f - factor ) * angleError;
		}
	}

	if ( limit.mLimitEnabled )
	{
		nextAngle = angle + ( relativeVelocity + impulse * inverseVirtualMass ) * deltaTime;
		if ( nextAngle > limit.mLimitMax )
		{
			limitBranch = 1;
			impulse -= virtualMass * ( nextAngle - limit.mLimitMax ) * limit.mLimitTau * tauFactor / deltaTime;
		}
		else if ( nextAngle < limit.mLimitMin )
		{
			limitBranch = -1;
			impulse -= virtualMass * ( nextAngle - limit.mLimitMin ) * limit.mLimitTau * tauFactor / deltaTime;
		}
	}
	if ( IsHavanaRagdollSolveTraceEnabled() )
		std::fprintf( stderr,
			"HAVANA_RAGDOLL_LIMIT solve=%u kind=angular index=%d first=%d angle=%.9g min=%.9g max=%.9g rel_v=%.9g next=%.9g branch=%d inv_mass=%.9g impulse=%.9g axis=(%.9g,%.9g,%.9g)\n",
			storage.mSolveOrdinal, limitIndex, firstSolve ? 1 : 0, angle, limit.mLimitMin,
			limit.mLimitMax, relativeVelocity, nextAngle, limitBranch, inverseVirtualMass, impulse,
			axisHavana.GetX(), axisHavana.GetY(), axisHavana.GetZ() );
	return ApplyAngularImpulseHavana( axisHavana, impulse, storage.mBodies );
}

bool HavanaConstraint::SolveRagdollPlaneLimit( int limitIndex, Vec3Arg axisHavana,
	Vec3Arg planeNormalHavana, float position,
	RagdollLimit &limit, float tauFactor, float dampingFactor, float deltaTime,
	bool firstSolve, RagdollSolverStorage &storage )
{
	Vec3 impulseAxis = axisHavana.Cross( planeNormalHavana );
	const float lengthSquared = impulseAxis.LengthSq();
	if ( lengthSquared < 0.01f )
	{
		// GMod's shipped Havana binary rejects a nearly parallel angular-plane row at the ordinary
		// float epsilon boundary. The public source snapshot names a much smaller HK_REAL_EPS here,
		// but direct DLL A/B shows the real cutoff: 0.014 degrees (cross^2=5.97e-8) is skipped while
		// 0.020 degrees (cross^2=1.218e-7) is solved. Using 1e-16 makes simfphys' micro-angle wheel
		// cone continuously generate a predictive impulse that the original engine never creates.
		if ( lengthSquared <= FLT_EPSILON )
			return false;
		impulseAxis = 0.1f * impulseAxis.Normalized();
	}
	const float inverseVirtualMass = GetAngularInverseMassHavana( impulseAxis, storage.mBodies );
	if ( inverseVirtualMass <= FLT_EPSILON )
		return false;
	const float virtualMass = 1.0f / inverseVirtualMass;
	const Vec3 angular1 = HavanaFromSourceVector( mBody1->GetAngularVelocity() );
	const Vec3 angular2 = HavanaFromSourceVector( mBody2->GetAngularVelocity() );
	const float relativeVelocity = impulseAxis.Dot( angular1 - angular2 );
	float impulse = 0.0f;
	float nextPosition = position;
	int limitBranch = 0;

	if ( limit.mFrictionEnabled )
	{
		const float error = position - limit.mReferencePosition;
		limit.mReferencePosition += limit.mDesiredVelocity * deltaTime;
		impulse = virtualMass * ( error * 0.8f * tauFactor / deltaTime - dampingFactor * relativeVelocity );
		if ( std::fabs( impulse ) / deltaTime > limit.mJointFriction )
		{
			const float factor = limit.mJointFriction * deltaTime / std::fabs( impulse );
			impulse *= factor;
			limit.mReferencePosition += ( 1.0f - factor ) * error;
		}
	}

	if ( limit.mLimitEnabled )
	{
		nextPosition = position - ( relativeVelocity + impulse * inverseVirtualMass ) * deltaTime;
		if ( nextPosition > limit.mLimitMax )
		{
			limitBranch = 1;
			impulse += virtualMass * ( nextPosition - limit.mLimitMax ) * limit.mLimitTau * tauFactor / deltaTime;
		}
		else if ( nextPosition < limit.mLimitMin )
		{
			limitBranch = -1;
			impulse += virtualMass * ( nextPosition - limit.mLimitMin ) * limit.mLimitTau * tauFactor / deltaTime;
		}
	}
	if ( IsHavanaRagdollSolveTraceEnabled() )
		std::fprintf( stderr,
			"HAVANA_RAGDOLL_LIMIT solve=%u kind=plane index=%d first=%d position=%.9g min=%.9g max=%.9g rel_v=%.9g next=%.9g branch=%d cross_len2=%.9g inv_mass=%.9g impulse=%.9g axis=(%.9g,%.9g,%.9g) normal=(%.9g,%.9g,%.9g)\n",
			storage.mSolveOrdinal, limitIndex, firstSolve ? 1 : 0, position, limit.mLimitMin,
			limit.mLimitMax, relativeVelocity, nextPosition, limitBranch, lengthSquared,
			inverseVirtualMass, impulse, impulseAxis.GetX(), impulseAxis.GetY(), impulseAxis.GetZ(),
			planeNormalHavana.GetX(), planeNormalHavana.GetY(), planeNormalHavana.GetZ() );
	return ApplyAngularImpulseHavana( impulseAxis, impulse, storage.mBodies );
}

bool HavanaConstraint::SolveRagdoll( float deltaTime, RagdollSolverStorage &storage,
	float tauFactor, float dampingFactor, bool firstSolve )
{
	++storage.mSolveOrdinal;
	if ( IsHavanaRagdollSolveTraceEnabled() )
	{
		const Vec3 angular1 = HavanaFromSourceVector( mBody1->GetAngularVelocity() );
		const Vec3 angular2 = HavanaFromSourceVector( mBody2->GetAngularVelocity() );
		std::fprintf( stderr,
			"HAVANA_RAGDOLL_SOLVE solve=%u body1=%u body2=%u first=%d tau=%.9g damping=%.9g axis_map=(%u,%u,%u) joint=(%.9g,%.9g,%.9g) w1=(%.9g,%.9g,%.9g) w2=(%.9g,%.9g,%.9g)\n",
			storage.mSolveOrdinal, mBody1->GetID().GetIndexAndSequenceNumber(),
			mBody2->GetID().GetIndexAndSequenceNumber(), firstSolve ? 1 : 0,
			tauFactor, dampingFactor, uint( mRagdollAxisMap[0] ),
			uint( mRagdollAxisMap[1] ), uint( mRagdollAxisMap[2] ),
			storage.mJointAngles.GetX(), storage.mJointAngles.GetY(), storage.mJointAngles.GetZ(),
			angular1.GetX(), angular1.GetY(), angular1.GetZ(),
			angular2.GetX(), angular2.GetY(), angular2.GetZ() );
	}
	bool applied = SolveRagdollAngularLimit(
		0,
		storage.mTwistAxis,
		storage.mJointAngles.GetX(),
		mRagdollLimits[0],
		tauFactor * storage.mTwistFactor,
		dampingFactor * storage.mTwistFactor,
		deltaTime,
		firstSolve,
		storage );
	for ( int limitIndex = 2; limitIndex >= 1; --limitIndex )
	{
		const Vec3 plane = limitIndex == 1 ? storage.mTwistAxisReference : storage.mPlaneAxisReference;
		applied |= SolveRagdollPlaneLimit(
			limitIndex,
			storage.mTwistAxisAttached,
			plane,
			storage.mJointAngles[limitIndex],
			mRagdollLimits[limitIndex],
			tauFactor,
			dampingFactor,
			deltaTime,
			firstSolve,
			storage );
	}

	if ( mSettings.mRagdollConstrainTranslation && storage.mLinearJacobian.mValid )
	{
		const float damping = firstSolve ? 0.0f : dampingFactor * mSettings.mStrength;
		const Vector< 3 > lambda = storage.mLinearJacobian.mEffectiveMass
			* ( tauFactor * mSettings.mTau * storage.mLinearCorrection
				- damping * storage.mLinearJacobian.GetVelocity( *mBody1, *mBody2 ) );
		applied |= storage.mLinearJacobian.ApplyImpulse( *mBody1, *mBody2, lambda,
			storage.mBodies.mInverseMass1, storage.mBodies.mInverseMass2,
			storage.mBodies.mInverseInertiaSource1, storage.mBodies.mInverseInertiaSource2 );
	}
	return applied;
}

void HavanaConstraint::SetRagdollFriction( float maxAngularImpulse )
{
	if ( mSettings.mKind != EHavanaConstraintKind::Ragdoll )
		return;
	for ( RagdollLimit &limit : mRagdollLimits )
	{
		limit.mFrictionEnabled = maxAngularImpulse != 0.0f;
		limit.mJointFriction = maxAngularImpulse;
		limit.mDesiredVelocity = 0.0f;
	}
}

void HavanaConstraint::UpdateRagdollTransforms( Mat44Arg frame1Havana, Mat44Arg frame2Havana )
{
	if ( mSettings.mKind != EHavanaConstraintKind::Ragdoll )
		return;
	mOriginalRagdollFrame1 = frame1Havana;
	mOriginalRagdollFrame2 = frame2Havana;
	SetMatrixAxis( mRagdollFrame1, 0, MatrixAxis( frame1Havana, mRagdollAxisMap[0] ) );
	SetMatrixAxis( mRagdollFrame2, 0, MatrixAxis( frame2Havana, mRagdollAxisMap[0] ) );
	SetMatrixAxis( mRagdollFrame1, 1, MatrixAxis( frame1Havana, mRagdollAxisMap[2] ) );
	SetMatrixAxis( mRagdollFrame2, 1, MatrixAxis( frame2Havana, mRagdollAxisMap[2] ) );
	SetMatrixAxis( mRagdollFrame1, 2, MatrixAxis( frame1Havana, mRagdollAxisMap[1] ) );
	SetMatrixAxis( mRagdollFrame2, 2, MatrixAxis( frame2Havana, mRagdollAxisMap[1] ) );
	if ( std::fabs( mRagdollConeCenter ) > 0.001f )
		RotateFrameLocally( mRagdollFrame1, 2, mRagdollConeCenter );
}

template struct HavanaConstraint::JacobianSystem< 1 >;
template struct HavanaConstraint::JacobianSystem< 3 >;
template struct HavanaConstraint::JacobianSystem< 6 >;

} // namespace JPH
