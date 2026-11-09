//=================================================================================================
//
// Source/Havana compatibility constraints backed by Jolt bodies.
//
// These constraints deliberately do not translate Source constraint parameters to Jolt's native
// Fixed / Distance / SixDOF semantics. They retain the Source local-constraint-system equations and
// use Jolt only for body state, island ownership and applying the resulting impulses.
//
//=================================================================================================

#pragma once

#include <Jolt/Math/Matrix.h>
#include <Jolt/Math/Mat44.h>
#include <Jolt/Physics/Constraints/TwoBodyConstraint.h>

#include <array>

namespace JPH
{

enum class EHavanaConstraintKind : uint8
{
	Fixed,
	Length,
	Ragdoll,
	Hinge,
	BallSocket,
	Sliding,
	Pulley,
};

/// Source coordinates and Havana/IVP coordinates use different Y/Z conventions. Jolt is kept in
/// Source coordinates by Volt, so the compatibility solver performs an explicit basis change and
/// runs the original equations in Havana coordinates.
Vec3 HavanaFromSourceVector( Vec3Arg source );
Vec3 SourceFromHavanaVector( Vec3Arg havana );
Mat44 HavanaFromSourceTransform( Mat44Arg source );
Mat44 SourceFromHavanaTransform( Mat44Arg havana );

class HavanaConstraint;

class HavanaConstraintSettings final : public TwoBodyConstraintSettings
{
	JPH_DECLARE_SERIALIZABLE_VIRTUAL( JPH_EXPORT, HavanaConstraintSettings )

public:
	TwoBodyConstraint *Create( Body &body1, Body &body2 ) const override;
	void SaveBinaryState( StreamOut &stream ) const override;

	EHavanaConstraintKind mKind = EHavanaConstraintKind::Fixed;

	/// Constraint-space-to-body-COM transforms, already expressed in Havana coordinates.
	Mat44 mFrame1 = Mat44::sIdentity();
	Mat44 mFrame2 = Mat44::sIdentity();

	/// Fixed and length correction parameters.
	float mTau = 1.0f;
	float mStrength = 1.0f;
	float mMinLength = 0.0f;
	float mMaxLength = 0.0f;

	/// Ragdoll public Source-axis parameters. The constructor reproduces Source's axis remap and
	/// hk_Ragdoll_Constraint_BP_Builder step before storing the runtime limits.
	Vec3 mRagdollLimitMin = Vec3::sZero();
	Vec3 mRagdollLimitMax = Vec3::sZero();
	Vec3 mRagdollMotorVelocity = Vec3::sZero();
	Vec3 mRagdollMotorForce = Vec3::sZero();
	bool mRagdollUseClockwiseRotations = false;
	bool mRagdollConstrainTranslation = true;

	/// Hinge frames store the hinge axis in X and its reference normal in Y.
	/// Unlike ragdoll frames, their columns are vectors in Havana body-COM space.
	bool mHingeLimitEnabled = false;
	float mHingeLimitMin = 0.0f;
	float mHingeLimitMax = 0.0f;
	float mHingeMotorVelocity = 0.0f;
	float mHingeMotorForce = 0.0f;

	/// Prismatic target frame, reference-object origin and local sliding direction.
	Vec3 mSlidingReferenceOrigin = Vec3::sZero();
	Vec3 mSlidingAxis = Vec3::sAxisX();
	bool mSlidingLimitEnabled = false;
	float mSlidingLimitMin = 0.0f;
	float mSlidingLimitMax = 0.0f;
	float mSlidingMotorVelocity = 0.0f;
	float mSlidingMotorForce = 0.0f;
	float mSlidingMotorEpsilon = 0.2f;

	/// Pulley fixed points are expressed in Havana world coordinates.
	Vec3 mPulleyFixedPoint1 = Vec3::sZero();
	Vec3 mPulleyFixedPoint2 = Vec3::sZero();
	float mPulleyRatio = 1.0f;
	bool mPulleyIsRigid = false;

protected:
	void RestoreBinaryState( StreamIn &stream ) override;
};

/// One exact Source/Havana two-body constraint. Group ordering is still owned by
/// LegacyConstraintGroup, but the mathematical solve is independent of Jolt's native constraints.
class HavanaConstraint final : public TwoBodyConstraint
{
public:
	JPH_OVERRIDE_NEW_DELETE

	HavanaConstraint( Body &body1, Body &body2, const HavanaConstraintSettings &settings );

	EConstraintSubType GetSubType() const override { return EConstraintSubType::User1; }
	void NotifyShapeChanged( const BodyID &bodyID, Vec3Arg deltaCOM ) override;
	void SetupVelocityConstraint( float deltaTime ) override;
	void OnCallbackBodyPropertiesChanged( float ) override { ++mBodyPropertiesRevision; }
	void ResetWarmStart() override;
	void WarmStartVelocityConstraint( float warmStartImpulseRatio ) override;
	bool SolveVelocityConstraint( float deltaTime ) override;
	bool SolvePositionConstraint( float deltaTime, float baumgarte ) override;
	bool SolvePositionConstraintInLegacyGroup( float deltaTime, float baumgarte ) override;
	bool SolvePostContactVelocityConstraintInLegacyGroup( float deltaTime ) override;
	bool RequiresPostContactLegacySolve() const override;
	size_t GetLegacySolverStorageSize() const override;
	size_t GetLegacySolverStorageAlignment() const override;
	void InitializeLegacySolverStorage( void *storage ) const override;
	void DestroyLegacySolverStorage( void *storage ) const override;
	bool CanPrepareLegacyVelocityConstraint() const override;
	void PrepareLegacyVelocityConstraint( void *storage, float deltaTime ) override;
	bool SetupAndSolveLegacyVelocityConstraint( void *storage, float deltaTime,
		float tauFactor, float dampingFactor ) override;
	bool StepLegacyVelocityConstraint( void *storage, float deltaTime,
		float tauFactor, float dampingFactor ) override;
	Mat44 GetConstraintToBody1Matrix() const override;
	Mat44 GetConstraintToBody2Matrix() const override;
	Ref<ConstraintSettings> GetConstraintSettings() const override;
	void SaveState( StateRecorder &stream ) const override;
	void RestoreState( StateRecorder &stream ) override;

#ifdef JPH_DEBUG_RENDERER
	void DrawConstraint( DebugRenderer *renderer ) const override;
#endif

	EHavanaConstraintKind GetHavanaKind() const { return mSettings.mKind; }
	void SetRagdollFriction( float maxAngularImpulse );
	void SetHingeMotor( float angularVelocity, float maxAngularForce );
	void SetSlidingMotor( float velocity, float maxForce, float epsilon = 1.0f );
	void UpdateRagdollTransforms( Mat44Arg frame1Havana, Mat44Arg frame2Havana );
	Mat44 GetOriginalRagdollFrame1() const { return mOriginalRagdollFrame1; }
	Mat44 GetOriginalRagdollFrame2() const { return mOriginalRagdollFrame2; }
	Mat44 GetRuntimeRagdollFrame1() const { return mRagdollFrame1; }
	Mat44 GetRuntimeRagdollFrame2() const { return mRagdollFrame2; }

private:
	struct JacobianRow
	{
		Vec3 mLinear1 { Vec3::sZero() };
		Vec3 mAngular1 { Vec3::sZero() };
		Vec3 mLinear2 { Vec3::sZero() };
		Vec3 mAngular2 { Vec3::sZero() };
	};

	template <uint N>
	struct JacobianSystem
	{
		std::array< JacobianRow, N > mRows;
		Matrix< N, N > mEffectiveMass;
		bool mValid = false;

		bool Setup( const Body &body1, const Body &body2,
			float inverseMass1, float inverseMass2,
			Mat44Arg inverseInertia1, Mat44Arg inverseInertia2 );
		Vector< N > GetVelocity( const Body &body1, const Body &body2 ) const;
		bool ApplyImpulse( Body &body1, Body &body2, const Vector< N > &lambda,
			float inverseMass1, float inverseMass2,
			Mat44Arg inverseInertiaSource1, Mat44Arg inverseInertiaSource2 ) const;
	};

	struct BodySolverData
	{
		uint64 mRevision = 0;
		float mInverseMass1 = 0.0f;
		float mInverseMass2 = 0.0f;
		Mat44 mInverseInertiaSource1 { Mat44::sZero() };
		Mat44 mInverseInertiaSource2 { Mat44::sZero() };
		Mat44 mInverseInertiaHavana1 { Mat44::sZero() };
		Mat44 mInverseInertiaHavana2 { Mat44::sZero() };
	};

	// Coupled point/angular rows can carry opposing impulses. Keep their mass
	// factorization and impulse cancellation in double when requested. Fixed6
	// retains its adaptive precision; Hinge5 uses this path for every solve.
	template <uint N>
	struct PreciseJacobianSystem : JacobianSystem< N >
	{
		using JacobianSystem< N >::mRows;
		using JacobianSystem< N >::mValid;
		using JacobianSystem< N >::mEffectiveMass;
		using JacobianSystem< N >::GetVelocity;
		using JacobianSystem< N >::ApplyImpulse;
		double mLDLT[N][N];
		double mInverseDiagonal[N];
		// Share one symmetric inertia operator between factorization and application.
		double mInverseInertia1[3][3];
		double mInverseInertia2[3][3];
		bool mPrecise = false;

		bool Setup( const Body &body1, const Body &body2, float inverseMass1, float inverseMass2,
			Mat44Arg inverseInertia1, Mat44Arg inverseInertia2, bool forcePrecision = false );
		bool SolveAndApply( Body &body1, Body &body2, const Vector< N > &correction,
			float biasFactor, float dampingFactor, const BodySolverData &bodies ) const;
	};

	struct FixedSolverStorage
	{
		FixedSolverStorage() { mCorrection.SetZero(); }
		BodySolverData mBodies;
		PreciseJacobianSystem< 6 > mJacobian;
		Vector< 6 > mCorrection;
		bool mPrepared = false;
	};

	struct LengthSolverStorage
	{
		LengthSolverStorage() { mCorrection.SetZero(); }
		BodySolverData mBodies;
		JacobianSystem< 1 > mJacobian;
		Vector< 1 > mCorrection;
		bool mSkipSolve = false;
	};

	struct RagdollSolverStorage
	{
		RagdollSolverStorage() { mLinearCorrection.SetZero(); }
		BodySolverData mBodies;
		Vec3 mTwistAxis { Vec3::sAxisX() };
		Vec3 mTwistAxisReference { Vec3::sAxisX() };
		Vec3 mTwistAxisAttached { Vec3::sAxisX() };
		Vec3 mPlaneAxisReference { Vec3::sAxisZ() };
		Vec3 mJointAngles { Vec3::sZero() };
		float mTwistFactor = 1.0f;
		uint32 mSolveOrdinal = 0;
		JacobianSystem< 3 > mLinearJacobian;
		Vector< 3 > mLinearCorrection;
	};

	struct HingeSolverStorage
	{
		HingeSolverStorage() { mCorrection.SetZero(); }
		BodySolverData mBodies;
		PreciseJacobianSystem< 5 > mJacobian;
		Vector< 5 > mCorrection;
		Vec3 mHingeAxis { Vec3::sAxisX() };
		float mAngle = 0.0f;
	};

	struct BallSocketSolverStorage
	{
		BallSocketSolverStorage() { mCorrection.SetZero(); }
		BodySolverData mBodies;
		PreciseJacobianSystem< 3 > mJacobian;
		Vector< 3 > mCorrection;
	};

	struct SlidingSolverStorage
	{
		SlidingSolverStorage() { mCorrection.SetZero(); }
		BodySolverData mBodies;
		PreciseJacobianSystem< 5 > mJacobian;
		Vector< 5 > mCorrection;
		JacobianSystem< 1 > mLimitJacobian;
		float mPosition = 0.0f;
	};

	struct RagdollLimit
	{
		bool mLimitEnabled = true;
		bool mFrictionEnabled = false;
		float mLimitMin = 0.0f;
		float mLimitMax = 0.0f;
		float mLimitTau = 1.0f;
		float mJointFriction = 0.0f;
		float mDesiredVelocity = 0.0f;
		float mReferencePosition = 0.0f;
	};

	void BuildRagdollRuntime();
	void BuildSlidingRuntime();
	bool SetupBallSocket( float deltaTime, BallSocketSolverStorage &storage );
	bool SolveBallSocket( BallSocketSolverStorage &storage, float tauFactor, float dampingFactor );
	bool SetupSliding( float deltaTime, SlidingSolverStorage &storage );
	bool SolveSliding( float deltaTime, SlidingSolverStorage &storage, float tauFactor, float dampingFactor );
	bool SolveSlidingLinearLimit( float deltaTime, const SlidingSolverStorage &storage,
		float tauFactor, float dampingFactor );
	bool SetupPulley( float deltaTime, LengthSolverStorage &storage );
	bool SetupHinge( float deltaTime, HingeSolverStorage &storage );
	bool SolveHinge( float deltaTime, HingeSolverStorage &storage,
		float tauFactor, float dampingFactor, bool firstSolve );
	bool SolveHingeAngularLimit( float deltaTime, const HingeSolverStorage &storage,
		float tauFactor, float dampingFactor );
	void PrepareBodySolverData( BodySolverData &storage ) const;
	bool SetupFixed( float deltaTime, FixedSolverStorage &storage );
	bool SolveFixed( FixedSolverStorage &storage, float tauFactor, float dampingFactor,
		bool firstSolve );
	bool SetupLength( float deltaTime, LengthSolverStorage &storage );
	bool SolveLength( LengthSolverStorage &storage, float tauFactor, float dampingFactor );
	bool SetupRagdoll( float deltaTime, RagdollSolverStorage &storage );
	bool SolveRagdoll( float deltaTime, RagdollSolverStorage &storage,
		float tauFactor, float dampingFactor, bool firstSolve );
	bool SolveRagdollAngularLimit( int limitIndex, Vec3Arg axisHavana, float angle, RagdollLimit &limit,
		float tauFactor, float dampingFactor, float deltaTime, bool firstSolve,
		RagdollSolverStorage &storage );
	bool SolveRagdollPlaneLimit( int limitIndex, Vec3Arg axisHavana, Vec3Arg planeNormalHavana, float position,
		RagdollLimit &limit, float tauFactor, float dampingFactor, float deltaTime,
		bool firstSolve, RagdollSolverStorage &storage );
	bool ApplyAngularImpulseHavana( Vec3Arg axisHavana, float impulse, const BodySolverData &storage );
	float GetAngularInverseMassHavana( Vec3Arg axisHavana, const BodySolverData &storage ) const;
	Mat44 GetBodyTransformHavana( const Body &body ) const;
	Mat44 GetBodyInverseInertiaHavana( const Body &body ) const;
	Quat PredictBodyRotation( const Body &body, float deltaTime ) const;

	HavanaConstraintSettings mSettings;

	Mat44 mOriginalRagdollFrame1 { Mat44::sIdentity() };
	Mat44 mOriginalRagdollFrame2 { Mat44::sIdentity() };
	Mat44 mRagdollFrame1 { Mat44::sIdentity() };
	Mat44 mRagdollFrame2 { Mat44::sIdentity() };
	std::array< RagdollLimit, 3 > mRagdollLimits;
	RagdollLimit mHingeLimit;
	RagdollLimit mSlidingLimit;
	float mSlidingPositionInterpolation = 0.0f;
	std::array< uint8, 3 > mRagdollAxisMap { 0, 1, 2 };
	float mRagdollConeCenter = 0.0f;
	uint64 mBodyPropertiesRevision = 0;
};

} // namespace JPH
