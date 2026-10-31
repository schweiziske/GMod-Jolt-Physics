
#pragma once

#include "vjolt_object.h"
#include "vjolt_environment.h"

class JoltPhysicsPlayerController : public IPhysicsPlayerController, public IJoltObjectDestroyedListener, public IJoltPhysicsController
{
public:
	JoltPhysicsPlayerController( JoltPhysicsObject *pObject );
	~JoltPhysicsPlayerController() override;

	void Update( const Vector &position, const Vector &velocity, float secondsToArrival, bool onground, IPhysicsObject *ground ) override;
	void SetEventHandler( IPhysicsPlayerControllerEvent *handler ) override;
	bool IsInContact( void ) override;
	void MaxSpeed( const Vector &maxVelocity ) override;

	void SetObject( IPhysicsObject *pObject ) override;

	int GetShadowPosition( Vector *position, QAngle *angles ) override;
	void StepUp( float height ) override;
	void Jump() override;
	void GetShadowVelocity( Vector *velocity ) override;
	IPhysicsObject *GetObject() override;
	void GetLastImpulse( Vector *pOut ) override;

	void SetPushMassLimit( float maxPushMass ) override;
	void SetPushSpeedLimit( float maxPushSpeed ) override;

	float GetPushMassLimit() override;
	float GetPushSpeedLimit() override;

	bool WasFrozen() override;

#if defined( GAME_GMOD_64X )
	uint32 GetContactState( uint16 nGameFlags ); // Not a virtual in Gmod.
#else
	uint32 GetContactState( uint16 nGameFlags ) override_portal2;
#endif

	// IJoltObjectDestroyedListener
	void OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject ) override;
	// IJoltPhysicsController
	int GetSimulationPriority() const override { return VJoltControllerPriority::Player; }
	void OnPreSimulate( float flDeltaTime ) override;
	void OnPostSimulate( float flDeltaTime ) override;

	int TryTeleportObject();

private:
	void SetObjectInternal( JoltPhysicsObject *pObject );
	void SetGroundInternal( JoltPhysicsObject *pGround );
	Vector GetGroundVelocity() const;

private:
	
	JoltPhysicsObject *m_pObject = nullptr;
	JoltPhysicsObject *m_pGround = nullptr;

	Vector m_vGroundPosition = vec3_origin;

	IPhysicsPlayerControllerEvent *m_pHandler = nullptr;
	float m_flMaxDeltaPosition = 24.0f;
	float m_flDampFactor = 1.0f;
	float m_flSecondsToArrival = 0.0f;
	float m_flPushableSpeedLimit = 1e4f;
	float m_flPushableMassLimit = VPHYSICS_MAX_MASS;
	Vector m_vTargetPosition = vec3_origin;
	Vector m_vMaxSpeed = vec3_origin;
	Vector m_vCurrentSpeed = vec3_origin;
	Vector m_vLastImpulse = vec3_origin;
	float m_flSavedAngularDamping = 0.0f;

	bool m_bEnable = false;
	bool m_bUpdatedSinceLast = false;
	bool m_bAngularDampingSaved = false;


};
