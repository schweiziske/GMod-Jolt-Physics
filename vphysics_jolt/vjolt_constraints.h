//=================================================================================================
//
// Constraints
//
//=================================================================================================

#pragma once

#include "vjolt_internal_listeners.h"

#include <Jolt/Physics/Constraints/DistanceConstraint.h>

enum constraintType_t
{
	CONSTRAINT_UNKNOWN = 0,
	CONSTRAINT_RAGDOLL,
	CONSTRAINT_HINGE,
	CONSTRAINT_FIXED,
	CONSTRAINT_BALLSOCKET,
	CONSTRAINT_SLIDING,
	CONSTRAINT_PULLEY,
	CONSTRAINT_LENGTH,
};

class JoltPhysicsConstraint;
class JoltPhysicsEnvironment;

class JoltPhysicsConstraintGroup final : public IPhysicsConstraintGroup
{
public:
	JoltPhysicsConstraintGroup( JoltPhysicsEnvironment *pEnvironment, const constraint_groupparams_t &params, bool active = false );
	~JoltPhysicsConstraintGroup() override;

	void Activate() override;
	bool IsInErrorState() override;
	void ClearErrorState() override;
	void GetErrorParams( constraint_groupparams_t *pParams ) override;
	void SetErrorParams( const constraint_groupparams_t &params ) override;
	void SolvePenetration( IPhysicsObject *pObj0, IPhysicsObject *pObj1 ) override;

	void AddConstraint( JoltPhysicsConstraint *pConstraint );
	void RemoveConstraint( JoltPhysicsConstraint *pConstraint );
	void AddSolverConstraint( JPH::Constraint *pConstraint );
	void RemoveSolverConstraint( JPH::Constraint *pConstraint );
	bool IsActive() const { return m_bActive; }

private:
	JoltPhysicsEnvironment				*m_pEnvironment;
	std::vector< JoltPhysicsConstraint * >	m_pConstraints;
	constraint_groupparams_t				m_ErrorParams = {};
	JPH::LegacyConstraintGroup			m_Solver;
	bool								m_bActive = false;
};

class JoltPhysicsConstraint final : public IPhysicsConstraint, public IJoltObjectDestroyedListener
{
friend class JoltPhysicsConstraintGroup;

public:
	JoltPhysicsConstraint( JoltPhysicsEnvironment *pPhysicsEnvironment, IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, constraintType_t Type = CONSTRAINT_UNKNOWN, JPH::Constraint* pConstraint = nullptr, void *pGameData = nullptr, JoltPhysicsConstraintGroup *pGroup = nullptr, bool systemDestroyed = false );
	~JoltPhysicsConstraint() override;

	void			Activate() override;
	void			Deactivate() override;

	void			SetGameData( void *gameData ) override;
	void *			GetGameData() const override;

	IPhysicsObject *GetReferenceObject() const override;
	IPhysicsObject *GetAttachedObject() const override;

	void			SetLinearMotor( float speed, float maxLinearImpulse ) override;
	void			SetAngularMotor( float rotSpeed, float maxAngularImpulse ) override;

	void			UpdateRagdollTransforms( const matrix3x4_t &constraintToReference, const matrix3x4_t &constraintToAttached ) override;
	bool			GetConstraintTransform( matrix3x4_t *pConstraintToReference, matrix3x4_t *pConstraintToAttached ) const override;
	bool			GetConstraintParams( constraint_breakableparams_t *pParams ) const override;

	void			OutputDebugInfo() override;

	// IJoltObjectDestroyedListener
	void OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject ) override;

public:
	bool InitialiseHingeFromRagdoll( IPhysicsConstraintGroup* pGroup, const constraint_ragdollparams_t& ragdoll );
	void InitialiseRagdoll( IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll );
	void InitialiseHinge( IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge );
	void InitialiseSliding( IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding );
	void InitialiseBallsocket( IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket );
	void InitialiseFixed( IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed );
	void InitialiseLength( IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length );
	void InitialisePulley( IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley );

	void SaveConstraintSettings( JPH::StateRecorder &recorder );
	void RestoreConstraintSettings( JPH::StateRecorder &recorder );
	JoltPhysicsConstraintGroup *GetGroup() const { return m_pGroup; }
	bool WasConstraintSystemDestroyed() const { return m_bConstraintSystemDestroyed; }
	bool CanSave() const { return m_pConstraint && m_pObjReference && m_pObjAttached && !m_bMarkedForDelete; }

	bool CheckBroken();
	bool MarkForDelete();
	bool IsMarkedForDelete() const { return m_bMarkedForDelete; }

private:

	void SetGroup( IPhysicsConstraintGroup *pGroup );
	bool UsesPrivateLegacySolver() const;
	void RegisterSolverConstraint();
	void RegisterWithPhysicsSystem();
	void UnregisterSolverConstraint();
	void OnConstraintGroupDestroyed( JoltPhysicsConstraintGroup *pGroup );
	void SetEnabledFromConstraintGroup( JoltPhysicsConstraintGroup *pGroup, bool bEnabled );
	void InitialiseHingeInternal(
		IPhysicsConstraintGroup *pGroup,
		const constraint_hingeparams_t &hinge,
		JPH::Vec3Arg referencePerpAxisObject,
		JPH::Vec3Arg attachedPerpAxisObject );

	void DestroyConstraint();

	void SetBreakableParams( const constraint_breakableparams_t &params );

	JoltPhysicsObject			*m_pObjReference = nullptr;
	JoltPhysicsObject			*m_pObjAttached = nullptr;
	JPH::Ref< JPH::Constraint > m_pConstraint;
	constraintType_t			m_ConstraintType = CONSTRAINT_UNKNOWN;

	JoltPhysicsConstraintGroup	*m_pGroup = nullptr;
	// Havana clears a grouped constraint's LCS pointer when the group is destroyed. It does not
	// turn that constraint into a standalone one, and later per-constraint activation stays a no-op.
	bool						m_bConstraintSystemDestroyed = false;
	// Source creates a one-constraint hk_Local_Constraint_System even when the caller
	// does not supply an IPhysicsConstraintGroup. Keep an equivalent owned solver for
	// constraint types whose Havana velocity-bias path has been implemented.
	JPH::LegacyConstraintGroup	m_PrivateSolver;

	void						*m_pGameData = nullptr;
	JoltPhysicsEnvironment		*m_pPhysicsEnvironment = nullptr;
	JPH::PhysicsSystem			*m_pPhysicsSystem = nullptr;

	float						m_LinearBreakImpulse = 0.0f;
	float						m_AngularBreakImpulse = 0.0f;
	bool						m_bMarkedForDelete = false;
	bool						m_bRegisteredWithPhysicsSystem = false;
	float						m_SourceForceLimit = 0.0f;
	float						m_SourceTorqueLimit = 0.0f;
	float						m_BreakStrength = 1.0f;
	float						m_BodyMassScale[2] = { 1.0f, 1.0f };
};
