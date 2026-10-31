
#pragma once

#include "vjolt_object.h"
#include "vjolt_environment.h"
#include "vjolt_shadow_control.h"

class JoltPhysicsShadowController final : public IPhysicsShadowController, public IJoltPhysicsController
{
public:
	JoltPhysicsShadowController( JoltPhysicsObject *pObject, bool allowTranslation, bool allowRotation );
	JoltPhysicsShadowController( JoltPhysicsObject *pObject, JPH::StateRecorder &recorder );
	~JoltPhysicsShadowController() override;

	void Update( const Vector &position, const QAngle &angles, float timeOffset ) override;
	void MaxSpeed( float maxSpeed, float maxAngularSpeed ) override;
	void StepUp( float height ) override;

	void SetTeleportDistance( float teleportDistance ) override;
	bool AllowsTranslation() override;
	bool AllowsRotation() override;

	void SetPhysicallyControlled( bool isPhysicallyControlled ) override;
	bool IsPhysicallyControlled() override;
	void GetLastImpulse( Vector *pOut ) override;
	void UseShadowMaterial( bool bUseShadowMaterial ) override;
	void ObjectMaterialChanged( int materialIndex ) override;

	float GetTargetPosition( Vector *pPositionOut, QAngle *pAnglesOut ) override;

	float GetTeleportDistance() override;
	void GetMaxSpeed( float *pMaxSpeedOut, float *pMaxAngularSpeedOut ) override;

	void SetDamping( const float *pLinear, const float *pAngular );
	void GetDamping( float *pLinear, float *pAngular ) const;
	void SetEnvironmentVelocityLimits( float maxLinearVelocity, float maxAngularVelocity );
	JoltPhysicsObject *GetObject() const { return m_pObject; }
	void SaveState( JPH::StateRecorder &recorder ) const;
	bool RestoreState( JPH::StateRecorder &recorder );

	// IJoltPhysicsController
	void OnPreSimulate( float flDeltaTime ) override;

private:
	void AttachObject();
	bool IsOnGround() const;

	JoltPhysicsObject *m_pObject = nullptr;

	VJoltShadowControl::Params m_control;
	float m_secondsToArrival = 0;

	float m_maxSpeed = 0.0f;
	float m_maxAngular = 0.0f;
	bool m_isPhysicallyControlled = false;
	bool m_allowTranslation = false;
	bool m_allowRotation = false;

	bool m_enabled = false;

	uint16 m_savedMaterialIndex = 0;
	uint16 m_savedCallbackFlags = 0;
	float m_savedMass = 0.0f;
	float m_savedLinearDamping = 0.0f;
	float m_savedAngularDamping = 0.0f;
	float m_savedMaxLinearVelocity = 0.0f;
	float m_savedMaxAngularVelocity = 0.0f;
	float m_controlLinearDamping = 0.0f;
	float m_controlAngularDamping = 100.0f;
	JPH::Vec3 m_savedInvInertiaDiagonal = JPH::Vec3::sZero();
	JPH::Quat m_savedInertiaRotation = JPH::Quat::sIdentity();
	JPH::EMotionType m_savedMotionType = JPH::EMotionType::Dynamic;
	bool m_motionPropertiesSaved = false;
	bool m_attached = false;
};
