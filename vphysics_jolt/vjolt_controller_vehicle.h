
#pragma once

#include "vjolt_object.h" // IJoltObjectDestroyedListener
#include "vjolt_environment.h" // IJoltPhysicsController
#include "vjolt_rigid_body_filter.h"
#include "vjolt_vehicle_settings.h"

struct JoltPhysicsWheel
{
	JoltPhysicsObject* pObject = nullptr;
	JPH::Ref< JPH::WheelSettingsWV > pSettings;
	JPH::LinearCurve baseLongitudinalFriction;
	JPH::LinearCurve baseLateralFriction;
	float frictionScale = 1.0f;
	float carGripScale = 1.0f;
	uint32_t carFlags = 0;
	bool InWater = false;
	float Depth = 0.0f;
};

struct JoltPhysicsInternalVehicleState
{
	bool  EngineDisabled = false;
	float BoostDelay = 0.0f;
	float BoosterRemainingTime = 0.0f;
	float LargestWheelRadius = 0.0f;
};

// Optional Jolt-native settings used only by the Lua vehicle bridge. Source vehicles keep using
// vehicleparams_t unchanged; in particular, their springAdditionalLength semantics do not change.
struct JoltVehicleBridgeAxleConfig_t
{
	float flSuspensionMinLength = 0.0f;
	float flSuspensionMaxLength = 0.0f;
	float flSuspensionPreloadLength = 0.0f;
};

struct JoltVehicleBridgeConfig_t
{
	bool bEnabled = false;
	bool bUseCylinderCollisionTester = false;
	JoltVehicleBridgeAxleConfig_t axles[ VEHICLE_MAX_AXLE_COUNT ];
};

// A single wheel's post-step state, in Source units/degrees. Filled by
// JoltPhysicsVehicleController::GetWheelSnapshot for consumers that need more than the dummy
// wheel object exposes (the Lua vehicle bridge). Deliberately free of Jolt types.
struct JoltVehicleWheelSnapshot_t
{
	Vector	vPosition;					// world space
	QAngle	angles;						// world space
	Vector	vContactPoint;				// world space, only meaningful when bHasContact
	float	flSteerAngle		= 0.0f;	// degrees, positive is to the left
	float	flRotationAngle		= 0.0f;	// degrees, spin about the axle
	float	flSuspensionLength	= 0.0f;	// distance from the hard point
	float	flSuspensionMinLength = 0.0f;
	float	flSuspensionMaxLength = 0.0f;
	float	flSuspensionPreloadLength = 0.0f;
	bool	bHasContact			= false;
	bool	bHitHardPoint		= false;
	int		nContactSurfaceProps = 0;

	// Raw Jolt units (radians, metres, newton seconds). Deliberately not converted: these exist
	// to be compared against Jolt's own debug output.
	float	flAngularVelocity		= 0.0f;
	float	flLongitudinalSlip		= 0.0f;
	float	flLateralSlipDegrees	= 0.0f;
	float	flLongitudinalFriction	= 0.0f;
	float	flLateralFriction		= 0.0f;
	float	flSuspensionLambda		= 0.0f;
	float	flLongitudinalLambda	= 0.0f;
	float	flLateralLambda			= 0.0f;
	Vector	vContactLongitudinal;	// world space, unit length while in contact
};

static constexpr int JOLT_AIRBOAT_MAX_PONTOONS = 4;

struct JoltAirboatPontoon
{
	Vector	hp_cs;
	Vector	raycast_start_cs;
	Vector	raycast_dir_cs;
	float	raycast_length		= 0.0f;
	float	spring_length		= 0.0f;
	float	spring_constant		= 0.0f;
	float	spring_damp_relax	= 0.0f;
	float	spring_damp_compress = 0.0f;
	float	friction_of_wheel	= 1.0f;
	float	wheel_radius		= 0.0f;
	float	raycast_dist		= 0.0f;
	bool	wheel_is_fixed		= true;
};

struct JoltAirboatImpact
{
	bool	bImpact			= false;
	bool	bImpactWater	= false;
	bool	bInWater		= false;
	Vector	vecImpactPointWS;
	Vector	vecImpactNormalWS;
	Vector	raycast_dir_ws;
	Vector	surface_speed_wheel_ws;
	Vector	projected_surface_speed_wheel_ws;
	float	flDepth			= 0.0f;
	float	flFriction		= 1.0f;
	float	flDampening		= 0.0f;
	float	friction_value	= 1.0f;
	float	inv_normal_dot_dir = 1.0f;
	int		nSurfaceProps	= 0;
};

struct JoltAirboatState
{
	float	m_flThrust			= 0.0f;
	float	m_SteeringAngle		= 0.0f;
	bool	m_bAnalogSteering	= false;
	bool	m_bSteeringReversed	= false;
	float	m_flPrevSteeringAngle = 0.0f;
	float	m_flSteerTime		= 0.0f;
	bool	m_bAirborne			= false;
	float	m_flAirTime			= 0.0f;
	bool	m_bWeakJump			= false;
	float	m_flPitchErrorPrev	= 0.0f;
	float	m_flRollErrorPrev	= 0.0f;
	float	m_flSpeed			= 0.0f;
	Vector	m_vecLocalVelocity = Vector( 0, 0, 0 );
};

class JoltPhysicsVehicleController final : public IPhysicsVehicleController, public IJoltObjectDestroyedListener, public IJoltPhysicsController
{
public:
	static constexpr int MaxWheels = VEHICLE_MAX_WHEEL_COUNT;

	JoltPhysicsVehicleController( JoltPhysicsEnvironment *pEnvironment, JPH::PhysicsSystem *pPhysicsSystem, JoltPhysicsObject *pVehicleBodyObject, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace, const JoltVehicleBridgeConfig_t *pBridgeConfig = nullptr, const std::vector< JoltPhysicsObject * > *pRestoredWheels = nullptr );
	~JoltPhysicsVehicleController() override;

	void				Update( float dt, vehicle_controlparams_t &controls ) override;
	const vehicle_operatingparams_t & GetOperatingParams() override;
	const vehicleparams_t & GetVehicleParams() override;
	vehicleparams_t &	GetVehicleParamsForChange() override;
	float				UpdateBooster( float dt ) override;
	int					GetWheelCount( void ) override;
	IPhysicsObject *	GetWheel( int index ) override;
	bool				GetWheelContactPoint( int index, Vector *pContactPoint, int *pSurfaceProps ) override;
	void				SetSpringLength( int wheelIndex, float length ) override;
	void				SetWheelFriction( int wheelIndex, float friction ) override;

	void				OnVehicleEnter( void ) override;
	void				OnVehicleExit( void ) override;

	void				SetEngineDisabled( bool bDisable ) override;
	bool				IsEngineDisabled( void ) override;

	void				GetCarSystemDebugData( vehicle_debugcarsystem_t &debugCarSystem ) override;
	void				VehicleDataReload() override;

public:
	// IJoltObjectDestroyedListener
	void OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject ) override;

	float GetSpeed();

	// Narrow accessors used by the Lua vehicle bridge. They exist so the bridge never has to
	// reach into JPH::VehicleConstraint or the dummy wheel objects itself.

	// False once the chassis object has been destroyed underneath us (see DetachObject). The
	// controller stays alive and inert in that state, it is never reattached to another body.
	bool HasChassis() const { return m_pCarBodyObject != nullptr; }
	JoltPhysicsObject *GetBodyObject() const { return m_pCarBodyObject; }
	unsigned int GetVehicleType() const { return m_VehicleType; }
	const JoltVehicleBridgeConfig_t &GetBridgeConfig() const { return m_BridgeVehicleConfig; }
	const JPH::Constraint *GetNativeConstraint() const { return m_VehicleConstraint; }
	bool HasSaveableTopology() const;
	void SaveState( JPH::StateRecorder &recorder );
	bool RestoreState( JPH::StateRecorder &recorder );
	void ReleaseRestoredWheels();
	bool OwnsObjectForTransfer( const JoltPhysicsObject *pObject ) const
	{
		if ( m_pCarBodyObject == pObject )
			return true;
		for ( const JoltPhysicsWheel &wheel : m_Wheels )
		{
			if ( wheel.pObject == pObject )
				return true;
		}
		return false;
	}

	// Overwrites the driver input that the next OnPreSimulate will hand to Jolt. Unlike
	// Update() this does no boost bookkeeping, so it is safe to call at an arbitrary rate.
	void SetDriverInput( float flThrottle, float flSteering, float flBrake, bool bHandbrake );
	bool ApplyCarTuning( const VJoltCarTuning &tuning );
	bool SetAxleGrip( uint32_t axle, float scale );
	bool ApplyCarWheels( const VJoltCarWheels &config );
	bool GetCarWheels( VJoltCarWheels &config ) const;

	// index is 0 based. Returns false for an airboat, an out of range index, or a controller
	// whose constraint has already been torn down.
	bool GetWheelSnapshot( int index, JoltVehicleWheelSnapshot_t &out );
	void ConfigureBridgeDrivetrain( float flMinRPM, float flPowerBandStartRPM, float flPowerBandEndRPM,
		float flReverseGearRatio, float flDifferentialRatio );

	// IJoltPhysicsController
	int GetSimulationPriority() const override { return VJoltControllerPriority::ConstraintsMax; }
	void OnPreSimulate( float flDeltaTime ) override;
	void OnPostSimulate( float flDeltaTime ) override;

private:

	bool IsAirboat() const { return m_VehicleType == VEHICLE_TYPE_AIRBOAT_RAYCAST; }

	void HandleBoostKey();
	void HandleBoostDecay();

	JoltPhysicsWheel &CreateWheelObject( const vehicle_axleparams_t &axle,
		const Vector &wheelPositionWorld, float wheelRadius, float wheelWidth, JoltPhysicsObject *pRestoredWheel = nullptr );
	void CreateWheel( JPH::VehicleConstraintSettings &vehicleSettings, matrix3x4_t &bodyMatrix, int axleIdx, int wheelIdx, JoltPhysicsObject *pRestoredWheel );
	void CreateWheels( JPH::VehicleConstraintSettings& vehicleSettings, const std::vector< JoltPhysicsObject * > *pRestoredWheels );

	JPH::WheeledVehicleControllerSettings *CreateVehicleController();
	JPH::WheeledVehicleController *GetWheeledVehicleController();
	void ApplyVehicleControllerSettings( const JPH::WheeledVehicleControllerSettings &settings, bool applyDifferentials );

	matrix3x4_t GetBodyMatrix() const;

	void DetachObject();

	// Airboat
	void InitAirboat( const vehicleparams_t &params, const std::vector< JoltPhysicsObject * > *pRestoredWheels );
	void AirboatUpdate( float dt, vehicle_controlparams_t &controls );
	void AirboatOnPreSimulate( float flDeltaTime );
	void AirboatPreRaycasts( JoltAirboatImpact *pImpacts, const matrix3x4_t &matWorldFromCore );
	void AirboatDoRaycasts( JoltAirboatImpact *pImpacts );
	bool AirboatPostRaycasts( JoltAirboatImpact *pImpacts, const matrix3x4_t &matWorldFromCore );
	void AirboatUpdateAirborne( JoltAirboatImpact *pImpacts, float flDeltaTime );
	int  AirboatCountSurfaceContacts( JoltAirboatImpact *pImpacts );
	float AirboatComputeFrontPontoonWaveNoise( int nIndex, float flSpeedRatio, float flCurrentTime );
	void AirboatDoPontoons( JoltAirboatImpact *pImpacts, float flDeltaTime );
	void AirboatDoPontoonGround( JoltAirboatPontoon *pPontoon, JoltAirboatImpact *pImpact, float flDeltaTime );
	void AirboatDoPontoonWater( JoltAirboatPontoon *pPontoon, JoltAirboatImpact *pImpact, float flDeltaTime );
	void AirboatDoDrag( JoltAirboatImpact *pImpacts, float flDeltaTime, const matrix3x4_t &matWorldFromCore );
	void AirboatDoTurbine( float flDeltaTime, const matrix3x4_t &matWorldFromCore );
	void AirboatDoSteering( float flDeltaTime );
	void AirboatDoKeepUprightPitch( JoltAirboatImpact *pImpacts, float flDeltaTime, const matrix3x4_t &matWorldFromCore );
	void AirboatDoKeepUprightRoll( JoltAirboatImpact *pImpacts, float flDeltaTime, const matrix3x4_t &matWorldFromCore );

	void ApplyImpulseAtPointMetric( const Vector &vImpulseMetric, const Vector &vWorldPosMetric );
	void ApplyImpulseCenterMetric( const Vector &vImpulseMetric );
	void ApplyAngularImpulseMetric( const Vector &vAngularImpulseMetric );

	JoltPhysicsEnvironment					*m_pEnvironment = nullptr;
	JPH::PhysicsSystem						*m_pPhysicsSystem = nullptr;
	JoltPhysicsObject						*m_pCarBodyObject = nullptr;
	IPhysicsGameTrace						*m_pGameTrace = nullptr;
	vehicleparams_t							m_VehicleParams = {};
	JoltVehicleBridgeConfig_t				m_BridgeVehicleConfig = {};
	float									m_flBridgeEngineMinRPM = 0.0f;
	float									m_flBridgePowerBandStartRPM = 0.0f;
	float									m_flBridgePowerBandEndRPM = 0.0f;
	float									m_flBridgeReverseGearRatio = 0.0f;
	float									m_flBridgeDifferentialRatio = 0.0f;
	JPH::Vec3								m_vVehicleForward = JPH::Vec3( 0, 1, 0 );
	unsigned int							m_VehicleType = 0u;
	int										m_nCreatedAxleCount = 0;
	int										m_nCreatedWheelsPerAxle = 0;

	vehicle_operatingparams_t				m_OperatingParams = {};
	vehicle_controlparams_t					m_ControlParams = {};

	std::vector< JoltPhysicsWheel >			m_Wheels;

	float									m_TotalWheelMass = 0.0f;
	float									m_flTorqueScale = 1.0f;
	JoltPhysicsInternalVehicleState			m_InternalState;

	JPH::Ref< JPH::VehicleConstraint >		m_VehicleConstraint;
	JPH::Ref< JPH::VehicleCollisionTester >	m_Tester;
	JoltRigidBodyFilter						m_CollisionFilter;

	// Airboat
	int										m_nAirboatPontoons	= 0;
	JoltAirboatPontoon						m_AirboatPontoons[ JOLT_AIRBOAT_MAX_PONTOONS ];
	JoltAirboatImpact						m_AirboatImpacts[ JOLT_AIRBOAT_MAX_PONTOONS ];
	JoltAirboatState						m_AirboatState;
};
