
#include "cbase.h"
#include "bridge/vjolt_gravity_internal.h"

#include "vjolt_layers.h"

#include "vjolt_collide.h"
#include "vjolt_controller_vehicle.h"
#include "vjolt_vehicle_constraint.h"
#include "vjolt_surfaceprops.h"

#include "cmodel.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//------------------------------------------------------------------------------------------------

static ConVar vjolt_vehicle_throttle_opposition_limit( "vjolt_vehicle_throttle_opposition_limit", "5", FCVAR_NONE,
	"Below what speed should we be attempting to drive/climb with handbrake on to avoid falling down." );

static ConVar vjolt_vehicle_disable_handbrakes( "vjolt_vehicle_disable_handbrakes", "0", FCVAR_NONE );

static ConVar vjolt_vehicle_disable_autobrake( "vjolt_vehicle_disable_autobrake", "0", FCVAR_NONE );
static ConVar vjolt_vehicle_disable_brake( "vjolt_vehicle_disable_brake", "0", FCVAR_NONE );

static ConVar vjolt_vehicle_throttle_override( "vjolt_vehicle_throttle_override", "-1.0", FCVAR_NONE );

//------------------------------------------------------------------------------------------------

static const JPH::Vec3 VehicleUpVector		= JPH::Vec3( 0, 0, 1 );
static const JPH::Vec3 DefaultVehicleForwardVector = JPH::Vec3( 0, 1, 0 );

static JPH::Vec3 DetermineVehicleForward( const vehicleparams_t &params )
{
	if ( params.axleCount >= 2 )
	{
		Vector forward = params.axles[ 0 ].offset - params.axles[ params.axleCount - 1 ].offset;
		forward.z = 0.0f;
		if ( forward.NormalizeInPlace() > 1e-4f )
			return SourceToJolt::Unitless( forward );
	}

	// Preserve Volt's original convention for malformed/single-axle data.
	return DefaultVehicleForwardVector;
}

static int GetDifferentialsPerAxle( int wheelsPerAxle )
{
	return wheelsPerAxle > 0 ? ( wheelsPerAxle + 1 ) / 2 : 0;
}

static int GetVehicleDifferentialCount( const vehicleparams_t &params )
{
	return params.axleCount * GetDifferentialsPerAxle( params.wheelsPerAxle );
}

static float GetVehicleEngineTopologyScale( int wheelsPerAxle )
{
	// Source applies the same torque to every wheel and its normal two-wheel axle
	// uses a hard-coded 0.5 multiplier. Scaling the engine by wheels/2 while
	// keeping the differential weights normalized reproduces that per-wheel
	// torque without violating Jolt's requirement that all differential ratios
	// sum to one.
	return wheelsPerAxle > 0 ? 0.5f * float( wheelsPerAxle ) : 0.0f;
}

static void ConfigureVehicleDifferential( JPH::VehicleDifferentialSettings &differential,
	const vehicleparams_t &params, float torqueScale, float torqueMultiplier,
	int axleIndex, int differentialInAxle )
{
	const int wheelsPerAxle = params.wheelsPerAxle;
	const int firstWheelInAxle = 2 * differentialInAxle;
	const int wheelsInDifferential = Min( 2, wheelsPerAxle - firstWheelInAxle );
	const float axleTorqueFactor = params.axles[ axleIndex ].torqueFactor;

	if ( wheelsInDifferential <= 0 || wheelsPerAxle <= 0 || axleTorqueFactor <= 0.0f )
	{
		differential.mLeftWheel = -1;
		differential.mRightWheel = -1;
		differential.mEngineTorqueRatio = 0.0f;
		return;
	}

	const int firstWheel = axleIndex * wheelsPerAxle + firstWheelInAxle;
	differential.mLeftWheel = firstWheel;
	differential.mRightWheel = wheelsInDifferential == 2 ? firstWheel + 1 : -1;
	differential.mEngineTorqueRatio = torqueMultiplier * torqueScale
		* axleTorqueFactor * ( float( wheelsInDifferential ) / float( wheelsPerAxle ) );
}

static void SetRuntimeWheelFrictionScale( JoltPhysicsWheel &wheel, float friction )
{
	friction = Max( friction, 0.0f );
	wheel.frictionScale = friction;
	friction *= wheel.carGripScale;

	// IVP stores SetWheelFriction as an absolute per-wheel scalar and multiplies
	// it into the contact friction. Rebuild from the construction-time curves so
	// repeated calls (the jetski does this every frame) never compound the scale
	// or replace the already validated Volt tyre profile.
	JPH::WheelSettingsWV &settings = *wheel.pSettings;
	settings.mLongitudinalFriction = wheel.baseLongitudinalFriction;
	settings.mLateralFriction = wheel.baseLateralFriction;
	for ( JPH::LinearCurve::Point &point : settings.mLongitudinalFriction.mPoints )
		point.mY *= friction;
	for ( JPH::LinearCurve::Point &point : settings.mLateralFriction.mPoints )
		point.mY *= friction;
}

constexpr float MPH_TO_METERSPERSECOND = 0.44707f;
inline float MphToGameVel( float mph ) { return mph * MPH_TO_METERSPERSECOND * JoltToSource::Factor; }

static inline constexpr float AIRBOAT_STEERING_RATE_MIN			= 0.00045f;
static inline constexpr float AIRBOAT_STEERING_RATE_MAX			= 5.0f * AIRBOAT_STEERING_RATE_MIN;
static inline constexpr float AIRBOAT_STEERING_INTERVAL			= 0.5f;

static inline constexpr float AIRBOAT_ROT_DRAG					= 0.00004f;
static inline constexpr float AIRBOAT_ROT_DAMPING				= 0.001f;

static inline constexpr float AIRBOAT_THRUST_MAX				= 11.0f;
static inline constexpr float AIRBOAT_THRUST_MAX_REVERSE		= 7.5f;

static inline constexpr float AIRBOAT_WATER_DRAG_LEFT_RIGHT		= 0.6f;
static inline constexpr float AIRBOAT_WATER_DRAG_FORWARD_BACK	= 0.005f;
static inline constexpr float AIRBOAT_WATER_DRAG_UP_DOWN		= 0.0025f;

static inline constexpr float AIRBOAT_GROUND_DRAG_LEFT_RIGHT	= 2.0f;
static inline constexpr float AIRBOAT_GROUND_DRAG_FORWARD_BACK	= 1.0f;
static inline constexpr float AIRBOAT_GROUND_DRAG_UP_DOWN		= 0.8f;

static inline constexpr float AIRBOAT_DRY_FRICTION_SCALE		= 0.6f;

static inline constexpr float AIRBOAT_RAYCAST_DIST				= 0.35f;
static inline constexpr float AIRBOAT_RAYCAST_DIST_WATER_LOW	= 0.1f;
static inline constexpr float AIRBOAT_RAYCAST_DIST_WATER_HIGH	= 0.35f;

static inline constexpr float AIRBOAT_WATER_NOISE_MIN			= 0.01f;
static inline constexpr float AIRBOAT_WATER_NOISE_MAX			= 0.03f;
static inline constexpr float AIRBOAT_WATER_FREQ_MIN			= 1.5f;
static inline constexpr float AIRBOAT_WATER_FREQ_MAX			= 1.5f;
static inline constexpr float AIRBOAT_WATER_PHASE_MIN			= 0.0f;
static inline constexpr float AIRBOAT_WATER_PHASE_MAX			= 1.5f;

static inline constexpr float AIRBOAT_GRAVITY					= 9.81f;

static inline constexpr float AIRBOAT_BUOYANCY_SCALAR			= 1.6f;
static inline constexpr float AIRBOAT_PONTOON_AREA_2D			= 2.8f;
static inline constexpr float AIRBOAT_PONTOON_HEIGHT			= 0.41f;

static const char *VehicleTypeToName( unsigned int VehicleType )
{
	switch ( VehicleType )
	{
		case VEHICLE_TYPE_CAR_WHEELS:		return "Car Wheels";
		case VEHICLE_TYPE_CAR_RAYCAST:		return "Car Raycast";
		case VEHICLE_TYPE_JETSKI_RAYCAST:	return "Jetski Raycast";
		case VEHICLE_TYPE_AIRBOAT_RAYCAST:	return "Airboat Raycast";
		default:							return "Unknown";
	}
}

JPH::Ref< JPH::VehicleCollisionTester > CreateVehicleCollisionTester( unsigned int VehicleType, float LargestWheelRadius )
{
	switch ( VehicleType )
	{
	default:
		Log_Warning( LOG_VJolt, "Don't know how to make vehicle type: %s (%u).\n", VehicleTypeToName( VehicleType ), VehicleType );
		[[ fallthrough ]];
	case VEHICLE_TYPE_CAR_WHEELS:
		return new JPH::VehicleCollisionTesterCastSphere( Layers::MOVING, LargestWheelRadius, VehicleUpVector );
	}
}

// Bridge vehicles cast every wheel each PSI. The full cast clears the old
// contact BodyID before excluding detached wheels.
class JoltCarCollisionTester final : public JPH::VehicleCollisionTesterCastCylinder
{
public:
	explicit JoltCarCollisionTester( const std::vector<JoltPhysicsWheel> &wheels )
		: JPH::VehicleCollisionTesterCastCylinder( Layers::MOVING ), m_Wheels( wheels ) {}
	bool Collide( JPH::PhysicsSystem &system, const JPH::VehicleConstraint &vehicle, JPH::uint index,
		JPH::RVec3Arg origin, JPH::Vec3Arg direction, const JPH::BodyID &bodyID, JPH::Body *&body,
		JPH::SubShapeID &shape, JPH::RVec3 &point, JPH::Vec3 &normal, float &length ) const override
	{
		if ( index >= m_Wheels.size() || ( m_Wheels[index].carFlags & kVJoltCarWheelDetached ) )
		{
			body = nullptr;
			return false;
		}
		return VehicleCollisionTesterCastCylinder::Collide( system, vehicle, index, origin, direction,
			bodyID, body, shape, point, normal, length );
	}
private:
	const std::vector<JoltPhysicsWheel> &m_Wheels;
};

//------------------------------------------------------------------------------------------------

JoltPhysicsVehicleController::JoltPhysicsVehicleController( JoltPhysicsEnvironment* pEnvironment, JPH::PhysicsSystem* pPhysicsSystem, JoltPhysicsObject* pVehicleBodyObject, const vehicleparams_t& params, unsigned int nVehicleType, IPhysicsGameTrace* pGameTrace, const JoltVehicleBridgeConfig_t *pBridgeConfig, const std::vector< JoltPhysicsObject * > *pRestoredWheels )
	: m_pEnvironment( pEnvironment )
	, m_pPhysicsSystem( pPhysicsSystem )
	, m_pCarBodyObject( pVehicleBodyObject )
	, m_pGameTrace( pGameTrace )
	, m_VehicleParams( params )
	, m_VehicleType( nVehicleType )
	, m_nCreatedAxleCount( params.axleCount )
	, m_nCreatedWheelsPerAxle( params.wheelsPerAxle )
{
	if ( pBridgeConfig )
		m_BridgeVehicleConfig = *pBridgeConfig;

	m_pCarBodyObject->AddConstraintAttachment();
	m_pCarBodyObject->AddDestroyedListener( this );

	// Saved parameters are already in runtime units. VehicleDataReload would convert the three
	// engine speed limits from mph a second time on every load.
	if ( !pRestoredWheels )
		VehicleDataReload();
	else
	{
		float totalTorque = 0.0f;
		for ( int axle = 0; axle < m_VehicleParams.axleCount; ++axle )
			totalTorque += m_VehicleParams.axles[ axle ].torqueFactor;
		m_flTorqueScale = totalTorque > 0.0f ? 1.0f / totalTorque : 1.0f;
	}

	if ( IsAirboat() )
	{
		InitAirboat( params, pRestoredWheels );
		return;
	}

	m_vVehicleForward = DetermineVehicleForward( m_VehicleParams );

	JPH::VehicleConstraintSettings vehicle;
	vehicle.mUp					= VehicleUpVector;
	vehicle.mForward			= m_vVehicleForward;
	vehicle.mDrawConstraintSize = 0.1f;
	CreateWheels( vehicle, pRestoredWheels );
	vehicle.mController			= CreateVehicleController();

	if ( m_BridgeVehicleConfig.bEnabled && m_BridgeVehicleConfig.bUseCylinderCollisionTester )
		m_Tester = new JoltCarCollisionTester( m_Wheels );
	else
		m_Tester = CreateVehicleCollisionTester( nVehicleType, m_InternalState.LargestWheelRadius );

	m_CollisionFilter.IgnoreBody( m_pCarBodyObject->GetBodyID() );
	m_Tester->SetBodyFilter( &m_CollisionFilter );

	m_VehicleConstraint = new JoltPinnableVehicleConstraint( *m_pCarBodyObject->GetBody(), vehicle );
	m_pPhysicsSystem->AddConstraint( m_VehicleConstraint );
	m_pPhysicsSystem->AddStepListener( m_VehicleConstraint );
	VJoltGravityManageVehicle( m_pCarBodyObject, true );
}

JoltPhysicsVehicleController::~JoltPhysicsVehicleController()
{
	DetachObject();

	for ( auto &wheel : m_Wheels )
	{
		if ( wheel.pObject )
		{
			wheel.pObject->RemoveDestroyedListener( this );
			m_pEnvironment->DestroyObject( wheel.pObject );
		}
	}
	m_Wheels.clear();
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::Update( float dt, vehicle_controlparams_t &controls )
{
	m_ControlParams = controls;

	UpdateBooster( dt );

	HandleBoostKey();

	if ( IsAirboat() )
		AirboatUpdate( dt, controls );
}

const vehicle_operatingparams_t &JoltPhysicsVehicleController::GetOperatingParams()
{
	return m_OperatingParams;
}

const vehicleparams_t &JoltPhysicsVehicleController::GetVehicleParams()
{
	return m_VehicleParams;
}

vehicleparams_t &JoltPhysicsVehicleController::GetVehicleParamsForChange()
{
	return m_VehicleParams;
}

float JoltPhysicsVehicleController::UpdateBooster( float dt )
{
	m_InternalState.BoostDelay				= Max( m_InternalState.BoostDelay - dt, 0.0f );
	m_InternalState.BoosterRemainingTime	= Max( m_InternalState.BoosterRemainingTime - dt, 0.0f );

	return m_InternalState.BoostDelay;
}

int JoltPhysicsVehicleController::GetWheelCount()
{
	if ( IsAirboat() )
		return m_nAirboatPontoons;
	return int( m_Wheels.size() );
}

IPhysicsObject *JoltPhysicsVehicleController::GetWheel( int index )
{
	if ( index < 0 || index >= int( m_Wheels.size() ) )
		return nullptr;

	return m_Wheels[ index ].pObject;
}

bool JoltPhysicsVehicleController::GetWheelContactPoint( int index, Vector *pContactPoint, int *pSurfaceProps )
{
	if ( IsAirboat() )
	{
		if ( index < 0 || index >= m_nAirboatPontoons )
		{
			if ( pContactPoint ) *pContactPoint = vec3_origin;
			if ( pSurfaceProps ) *pSurfaceProps = 0;
			return false;
		}

		const JoltAirboatImpact &impact = m_AirboatImpacts[ index ];
		if ( impact.bImpact )
		{
			if ( pContactPoint ) *pContactPoint = impact.vecImpactPointWS;
			if ( pSurfaceProps ) *pSurfaceProps = impact.nSurfaceProps;
			return true;
		}

		if ( pContactPoint ) *pContactPoint = vec3_origin;
		if ( pSurfaceProps ) *pSurfaceProps = 0;
		return false;
	}

	if ( index >= 0 && index < int( m_Wheels.size() ) && m_VehicleConstraint->GetWheels()[ index ]->HasContact() )
	{
		JPH::Wheel *pWheel = m_VehicleConstraint->GetWheels()[ index ];
		if ( pContactPoint )
			*pContactPoint = JoltToSource::Distance( pWheel->GetContactPosition() );

		if ( pSurfaceProps )
		{
			JPH::BodyLockRead lock( m_pEnvironment->GetBodyLockInterfaceForRead(),
				pWheel->GetContactBodyID() );
			const JPH::Body *pContactBody = lock.Succeeded() ? &lock.GetBody() : nullptr;
			JoltPhysicsObject *pContactObject = pContactBody
				? reinterpret_cast< JoltPhysicsObject * >( pContactBody->GetUserData() )
				: nullptr;
			*pSurfaceProps = pContactObject
				? JoltResolveSourceContactMaterial( *pContactBody,
					pWheel->GetContactSubShapeID(), pWheel->GetContactPosition(),
					pWheel->GetContactNormal(), pContactObject->GetMaterialIndex() )
				: 0;
		}

		return true;
	}
	else
	{
		if ( pContactPoint )
			*pContactPoint = vec3_origin;

		if ( pSurfaceProps )
			*pSurfaceProps = 0;

		return false;
	}
}

void JoltPhysicsVehicleController::SetSpringLength( int wheelIndex, float length )
{
	if ( IsAirboat() )
	{
		if ( wheelIndex >= 0 && wheelIndex < m_nAirboatPontoons )
		{
			// Source's CPhysics_Airboat stores spring_len separately from the
			// water/ground ray length. Its ground force continues to use the
			// dynamically chosen raycast_length, so this must not rewrite it.
			m_AirboatPontoons[ wheelIndex ].spring_length = length;
		}
		return;
	}

	if ( wheelIndex < 0 || wheelIndex >= int( m_Wheels.size() )
		|| !m_Wheels[ wheelIndex ].pSettings )
		return;

	// Source passes this value directly to IVP in metres (the jetski uses
	// 0.10-0.35), so do not apply SourceToJolt::Distance here.
	JPH::WheelSettingsWV *pSettings = m_Wheels[ wheelIndex ].pSettings;
	pSettings->mSuspensionMaxLength = Max( pSettings->mSuspensionMinLength, length );
}

void JoltPhysicsVehicleController::SetWheelFriction( int wheelIndex, float friction )
{
	if ( IsAirboat() )
	{
		if ( wheelIndex >= 0 && wheelIndex < m_nAirboatPontoons )
			m_AirboatPontoons[ wheelIndex ].friction_of_wheel = friction;
		return;
	}

	if ( wheelIndex < 0 || wheelIndex >= int( m_Wheels.size() )
		|| !m_Wheels[ wheelIndex ].pSettings )
		return;

	SetRuntimeWheelFrictionScale( m_Wheels[ wheelIndex ], friction );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::OnVehicleEnter()
{
	// Undo any damping we may have set to slow the boat when
	// we got out.
	if ( m_VehicleType == VEHICLE_TYPE_AIRBOAT_RAYCAST )
	{
		float flDampSpeed = 0.0f;
		float flDampRotSpeed = 0.0f;
		m_pCarBodyObject->SetDamping( &flDampSpeed, &flDampRotSpeed );
	}
}

void JoltPhysicsVehicleController::OnVehicleExit()
{
	// If we are an airboat, set a bunch of damping to slow us down.
	if ( m_VehicleType == VEHICLE_TYPE_AIRBOAT_RAYCAST )
	{
		float flDampSpeed = 1.0f;
		float flDampRotSpeed = 1.0f;
		m_pCarBodyObject->SetDamping( &flDampSpeed, &flDampRotSpeed );
	}

	SetEngineDisabled( false );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::SetEngineDisabled( bool bDisable )
{
	m_InternalState.EngineDisabled = bDisable;
}

bool JoltPhysicsVehicleController::IsEngineDisabled()
{
	return m_InternalState.EngineDisabled;
}

void JoltPhysicsVehicleController::GetCarSystemDebugData( vehicle_debugcarsystem_t &debugCarSystem )
{
	V_memset( &debugCarSystem, 0, sizeof( debugCarSystem ) );
	if ( !m_pCarBodyObject )
		return;

	const matrix3x4_t bodyMatrix = GetBodyMatrix();
	if ( IsAirboat() )
	{
		for ( int wheelIndex = 0; wheelIndex < Min( m_nAirboatPontoons,
			VEHICLE_DEBUGRENDERDATA_MAX_WHEELS ); ++wheelIndex )
		{
			const JoltAirboatPontoon &pontoon = m_AirboatPontoons[ wheelIndex ];
			Vector rayStart;
			VectorTransform( pontoon.raycast_start_cs, bodyMatrix, rayStart );
			Vector rayDirection;
			VectorRotate( pontoon.raycast_dir_cs, bodyMatrix, rayDirection );
			debugCarSystem.vecWheelRaycasts[ wheelIndex ][ 0 ] = rayStart;
			debugCarSystem.vecWheelRaycasts[ wheelIndex ][ 1 ] = rayStart
				+ rayDirection * ( pontoon.raycast_length * JoltToSource::Factor );
			debugCarSystem.vecWheelRaycastImpacts[ wheelIndex ] =
				m_AirboatImpacts[ wheelIndex ].bImpact
					? m_AirboatImpacts[ wheelIndex ].vecImpactPointWS
					: debugCarSystem.vecWheelRaycasts[ wheelIndex ][ 1 ];
		}
		return;
	}

	if ( !m_VehicleConstraint )
		return;

	const JPH::RMat44 bodyTransform = m_pCarBodyObject->GetBody()->GetWorldTransform();
	const int wheelCount = Min( GetWheelCount(), VEHICLE_DEBUGRENDERDATA_MAX_WHEELS );
	for ( int wheelIndex = 0; wheelIndex < wheelCount; ++wheelIndex )
	{
		JPH::Wheel *pWheel = m_VehicleConstraint->GetWheels()[ wheelIndex ];
		const JPH::WheelSettings *pSettings = pWheel->GetSettings();
		const JPH::RVec3 rayStart = bodyTransform * pSettings->mPosition;
		const JPH::Vec3 rayDirection = bodyTransform.Multiply3x3(
			pSettings->mSuspensionDirection );
		const JPH::RVec3 rayEnd = rayStart + rayDirection
			* ( pSettings->mSuspensionMaxLength + pSettings->mRadius );

		debugCarSystem.vecWheelRaycasts[ wheelIndex ][ 0 ] = JoltToSource::Distance( rayStart );
		debugCarSystem.vecWheelRaycasts[ wheelIndex ][ 1 ] = JoltToSource::Distance( rayEnd );
		debugCarSystem.vecWheelRaycastImpacts[ wheelIndex ] = pWheel->HasContact()
			? JoltToSource::Distance( pWheel->GetContactPosition() )
			: debugCarSystem.vecWheelRaycasts[ wheelIndex ][ 1 ];
	}
}

void JoltPhysicsVehicleController::VehicleDataReload()
{
	m_flTorqueScale = 1.0f;
	float totalTorqueDistribution = 0.0f;
	for ( int axle = 0; axle < m_VehicleParams.axleCount; ++axle )
		totalTorqueDistribution += m_VehicleParams.axles[ axle ].torqueFactor;
	if ( totalTorqueDistribution > 0.0f )
		m_flTorqueScale /= totalTorqueDistribution;

	m_VehicleParams.engine.maxSpeed		= MphToGameVel( m_VehicleParams.engine.maxSpeed );
	m_VehicleParams.engine.maxRevSpeed	= MphToGameVel( m_VehicleParams.engine.maxRevSpeed );
	m_VehicleParams.engine.boostMaxSpeed = MphToGameVel( m_VehicleParams.engine.boostMaxSpeed );

	if ( IsAirboat() || !m_pCarBodyObject || !m_VehicleConstraint )
		return;

	JPH::WheeledVehicleController *pController = GetWheeledVehicleController();
	JPH::Ref< JPH::WheeledVehicleControllerSettings > controllerSettings =
		CreateVehicleController();
	const int expectedWheelCount = m_VehicleParams.axleCount * m_VehicleParams.wheelsPerAxle;
	const int expectedDifferentialCount = GetVehicleDifferentialCount( m_VehicleParams );
	const bool topologyMatches = m_VehicleParams.axleCount == m_nCreatedAxleCount
		&& m_VehicleParams.wheelsPerAxle == m_nCreatedWheelsPerAxle
		&& expectedWheelCount == int( m_Wheels.size() )
		&& expectedDifferentialCount == int( pController->GetDifferentials().size() );
	ApplyVehicleControllerSettings( *controllerSettings, topologyMatches );

	if ( !topologyMatches )
	{
		Log_Warning( LOG_VJolt,
			"VehicleDataReload cannot change a live vehicle's topology "
			"(%d axles x %d wheels -> %d axles x %d wheels).\n",
			m_nCreatedAxleCount, m_nCreatedWheelsPerAxle,
			m_VehicleParams.axleCount, m_VehicleParams.wheelsPerAxle );
	}
}

//-------------------------------------------------------------------------------------------------

float JoltPhysicsVehicleController::GetSpeed()
{
	const JPH::Vec3 worldForward = m_pCarBodyObject->GetBody()->GetRotation() * m_vVehicleForward;
	const Vector orientation = JoltToSource::Unitless( worldForward );
	return orientation.Dot( m_pCarBodyObject->GetVelocity() );
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::SetDriverInput( float flThrottle, float flSteering, float flBrake, bool bHandbrake )
{
	// Only the fields OnPreSimulate actually consumes. Everything else in m_ControlParams
	// (boost, the brake pedal flags, analog steering) stays at whatever the game set it to,
	// which for a bridge driven vehicle is its zero initialised state.
	m_ControlParams.throttle	= flThrottle;
	m_ControlParams.steering	= flSteering;
	m_ControlParams.brake		= flBrake;
	m_ControlParams.handbrake	= bHandbrake;
}

bool JoltPhysicsVehicleController::ApplyCarTuning( const VJoltCarTuning &tuning )
{
	if ( !m_BridgeVehicleConfig.bEnabled || IsAirboat() || !m_pCarBodyObject
		|| tuning.size != sizeof( tuning ) || tuning.axleCount != uint32_t( m_nCreatedAxleCount )
		|| m_nCreatedWheelsPerAxle != 2 || tuning.axleCount > kVJoltVehicleMaxAxles )
		return false;
	const auto range = []( float value, float lo, float hi ) {
		return std::isfinite( value ) && value >= lo && value <= hi;
	};
	for ( uint32_t axle = 0; axle < tuning.axleCount; ++axle )
	{
		const auto &a = tuning.axles[axle];
		if ( !range( a.wheelInertia, .05f, 500 )
			|| !range( a.longitudinalPeak, .05f, 5 ) || !range( a.longitudinalSlide, .01f, a.longitudinalPeak )
			|| !range( a.lateralPeak, .05f, 5 ) || !range( a.lateralSlide, .01f, a.lateralPeak )
			|| !range( a.longitudinalSlip, .01f, .9f ) || !range( a.lateralSlip, .5f, 29 )
			|| !range( a.brakeTorque, 0, 100000 ) || !range( a.handbrakeTorque, 0, 100000 ) )
			return false;
	}
	for ( uint32_t axle = 0; axle < tuning.axleCount; ++axle )
	{
		const auto &a = tuning.axles[axle];
		for ( uint32_t i = 2 * axle; i < 2 * axle + 2; ++i )
		{
			auto &wheel = m_Wheels[i];
			auto &settings = *wheel.pSettings;
			settings.mInertia = a.wheelInertia;
			settings.mMaxBrakeTorque = a.brakeTorque;
			settings.mMaxHandBrakeTorque = a.handbrakeTorque;
			settings.mLongitudinalFriction.Clear();
			// A rolling tire must still be allowed to transmit static traction. A
			// zero budget at zero slip causes an unnecessarily abrupt launch cycle.
			settings.mLongitudinalFriction.AddPoint( 0, a.longitudinalPeak );
			settings.mLongitudinalFriction.AddPoint( a.longitudinalSlip, a.longitudinalPeak );
			settings.mLongitudinalFriction.AddPoint( 1, a.longitudinalSlide );
			settings.mLateralFriction.Clear();
			settings.mLateralFriction.AddPoint( 0, 0 );
			settings.mLateralFriction.AddPoint( a.lateralSlip, a.lateralPeak );
			settings.mLateralFriction.AddPoint( 30, a.lateralSlide );
			wheel.baseLongitudinalFriction = settings.mLongitudinalFriction;
			wheel.baseLateralFriction = settings.mLateralFriction;
			SetRuntimeWheelFrictionScale( wheel, wheel.frictionScale );
		}
	}
	return true;
}

bool JoltPhysicsVehicleController::SetAxleGrip( uint32_t axle, float scale )
{
	if ( !m_BridgeVehicleConfig.bEnabled || IsAirboat() || !m_pCarBodyObject
		|| m_nCreatedWheelsPerAxle != 2 || axle >= uint32_t( m_nCreatedAxleCount )
		|| !std::isfinite( scale ) || scale < .05f || scale > 2 )
		return false;
	for ( uint32_t i = 2 * axle; i < 2 * axle + 2; ++i )
		SetRuntimeWheelFrictionScale( m_Wheels[i], scale );
	return true;
}

bool JoltPhysicsVehicleController::GetCarWheels( VJoltCarWheels &config ) const
{
	if ( !m_BridgeVehicleConfig.bEnabled || IsAirboat() || !m_pCarBodyObject
		|| config.size != sizeof( config ) || m_nCreatedWheelsPerAxle != 2
		|| m_Wheels.size() > kVJoltCarMaxWheels )
		return false;
	config.wheelCount = uint32_t( m_Wheels.size() );
	for ( uint32_t i = 0; i < config.wheelCount; ++i )
	{
		const auto &wheel = m_Wheels[i];
		auto &out = config.wheels[i];
		out.radius = JoltToSource::Distance( wheel.pSettings->mRadius );
		out.width = JoltToSource::Distance( wheel.pSettings->mWidth );
		out.inertia = wheel.pSettings->mInertia;
		out.grip = wheel.carGripScale;
		out.flags = wheel.carFlags;
	}
	return true;
}

bool JoltPhysicsVehicleController::ApplyCarWheels( const VJoltCarWheels &config )
{
	if ( !m_BridgeVehicleConfig.bEnabled || IsAirboat() || !m_pCarBodyObject
		|| config.size != sizeof( config ) || config.wheelCount != m_Wheels.size()
		|| m_nCreatedWheelsPerAxle != 2 || config.wheelCount > kVJoltCarMaxWheels )
		return false;
	const auto range = []( float v, float lo, float hi ) { return std::isfinite( v ) && v >= lo && v <= hi; };
	bool changed = false;
	for ( uint32_t i = 0; i < config.wheelCount; ++i )
	{
		const auto &w = config.wheels[i];
		if ( !range( w.radius, 2, 64 ) || !range( w.width, 1, 64 ) || !range( w.inertia, .05f, 500 )
			|| !range( w.grip, .01f, 2 ) || ( w.flags & ~kVJoltCarWheelDetached ) ) return false;
		const auto &old = m_Wheels[i];
		changed |= fabsf( old.pSettings->mRadius - SourceToJolt::Distance( w.radius ) ) > 1e-6f
			|| fabsf( old.pSettings->mWidth - SourceToJolt::Distance( w.width ) ) > 1e-6f
			|| old.pSettings->mInertia != w.inertia || old.carGripScale != w.grip || old.carFlags != w.flags;
	}
	if ( !changed ) return true;
	for ( uint32_t i = 0; i < config.wheelCount; ++i )
	{
		const auto &in = config.wheels[i];
		auto &wheel = m_Wheels[i];
		wheel.pSettings->mRadius = SourceToJolt::Distance( in.radius );
		wheel.pSettings->mWidth = SourceToJolt::Distance( in.width );
		wheel.pSettings->mInertia = in.inertia;
		wheel.carGripScale = in.grip;
		wheel.carFlags = in.flags;
		SetRuntimeWheelFrictionScale( wheel, wheel.frictionScale );
		if ( in.flags & kVJoltCarWheelDetached ) m_VehicleConstraint->GetWheels()[i]->SetAngularVelocity( 0 );
	}
	// Prediction cannot remove a cached contact BodyID. Keep full casts for
	// mutable wheels so detachment and geometry changes replace every contact.
	m_VehicleConstraint->SetNumStepsBetweenCollisionTestActive( 1 );
	m_VehicleConstraint->SetNumStepsBetweenCollisionTestInactive( 1 );
	m_VehicleConstraint->ResetWarmStart();
	m_pPhysicsSystem->GetBodyInterfaceNoLock().ActivateBody( m_pCarBodyObject->GetBodyID() );
	return true;
}

void JoltPhysicsVehicleController::ConfigureBridgeDrivetrain( float flMinRPM, float flPowerBandStartRPM,
	float flPowerBandEndRPM, float flReverseGearRatio, float flDifferentialRatio )
{
	m_flBridgeEngineMinRPM = flMinRPM;
	m_flBridgePowerBandStartRPM = flPowerBandStartRPM;
	m_flBridgePowerBandEndRPM = flPowerBandEndRPM;
	m_flBridgeReverseGearRatio = flReverseGearRatio;
	m_flBridgeDifferentialRatio = flDifferentialRatio;

	if ( IsAirboat() || m_VehicleConstraint == nullptr )
		return;

	JPH::WheeledVehicleController *pController = GetWheeledVehicleController();
	JPH::VehicleEngine &engine = pController->GetEngine();
	engine.mMinRPM = flMinRPM;
	engine.mNormalizedTorque.Clear();
	engine.mNormalizedTorque.AddPoint( 0.0f, 0.0f );
	engine.mNormalizedTorque.AddPoint( flPowerBandStartRPM / engine.mMaxRPM, 1.0f );
	engine.mNormalizedTorque.AddPoint( flPowerBandEndRPM / engine.mMaxRPM, 1.0f );
	engine.mNormalizedTorque.AddPoint( 1.0f, 0.0f );
	engine.SetCurrentRPM( flMinRPM );

	JPH::VehicleTransmission &transmission = pController->GetTransmission();
	transmission.mReverseGearRatios.clear();
	transmission.mReverseGearRatios.push_back( -flReverseGearRatio );
	pController->SetDifferentialLimitedSlipRatio( FLT_MAX );

	const int differentialsPerAxle = GetDifferentialsPerAxle(
		m_VehicleParams.wheelsPerAxle );
	int differentialIndex = 0;
	for ( int axleIndex = 0; axleIndex < m_VehicleParams.axleCount; ++axleIndex )
	{
		for ( int differentialInAxle = 0;
			differentialInAxle < differentialsPerAxle; ++differentialInAxle )
		{
			if ( differentialIndex >= int( pController->GetDifferentials().size() ) )
				return;
			JPH::VehicleDifferentialSettings &differential =
				pController->GetDifferentials()[ differentialIndex++ ];
			differential.mDifferentialRatio = flDifferentialRatio;
			ConfigureVehicleDifferential( differential, m_VehicleParams,
				m_flTorqueScale, 1.0f, axleIndex, differentialInAxle );
		}
	}
}

bool JoltPhysicsVehicleController::GetWheelSnapshot( int index, JoltVehicleWheelSnapshot_t &out )
{
	out = JoltVehicleWheelSnapshot_t{};

	if ( IsAirboat() || m_VehicleConstraint == nullptr || m_pCarBodyObject == nullptr )
		return false;

	if ( index < 0 || index >= int( m_Wheels.size() ) )
		return false;

	const JPH::Wheel *pWheel = m_VehicleConstraint->GetWheels()[ index ];

	// The wheel model is a cylinder whose axle is its local Y axis. Jolt expects
	// the wheel-model right/up basis here, not the vehicle forward/up basis.
	// Bridge snapshots render that model directly. The Source wheel objects in
	// OnPostSimulate use a different basis for the stock vehicle's roll pose.
	const JPH::Mat44 wheelTransform = m_VehicleConstraint->GetWheelWorldTransform( index, JPH::Vec3::sAxisY(), JPH::Vec3::sAxisX() );

	out.vPosition			= JoltToSource::Distance( wheelTransform.GetTranslation() );
	out.angles				= JoltToSource::Angle( wheelTransform.GetQuaternion() );
	out.flSteerAngle		= JoltToSource::Angle( pWheel->GetSteerAngle() );
	out.flRotationAngle		= JoltToSource::Angle( pWheel->GetRotationAngle() );
	out.flSuspensionLength	= JoltToSource::Distance( pWheel->GetSuspensionLength() );
	const JPH::WheelSettings *pWheelSettings = pWheel->GetSettings();
	out.flSuspensionMinLength = JoltToSource::Distance( pWheelSettings->mSuspensionMinLength );
	out.flSuspensionMaxLength = JoltToSource::Distance( pWheelSettings->mSuspensionMaxLength );
	out.flSuspensionPreloadLength = JoltToSource::Distance( pWheelSettings->mSuspensionPreloadLength );
	out.bHitHardPoint		= pWheel->HasHitHardPoint();

	// GetWheelContactPoint already guards JPH::Wheel::GetContactPosition's HasContact assert.
	out.bHasContact = GetWheelContactPoint( index, &out.vContactPoint, &out.nContactSurfaceProps );

	// Diagnostics, in Jolt's own units. Everything below is what the tyre model actually did on
	// the last step, as opposed to where the wheel ended up.
	const JPH::WheelWV *pWheelWV = static_cast< const JPH::WheelWV * >( pWheel );

	out.flAngularVelocity		= pWheel->GetAngularVelocity();
	out.flLongitudinalSlip		= pWheelWV->mLongitudinalSlip;
	out.flLateralSlipDegrees	= RAD2DEG( pWheelWV->mLateralSlip );
	out.flLongitudinalFriction	= pWheelWV->mCombinedLongitudinalFriction;
	out.flLateralFriction		= pWheelWV->mCombinedLateralFriction;
	out.flSuspensionLambda		= pWheel->GetSuspensionLambda();
	out.flLongitudinalLambda	= pWheel->GetLongitudinalLambda();
	out.flLateralLambda			= pWheel->GetLateralLambda();

	if ( out.bHasContact )
	{
		// A direction, not a distance, so it must not go through JoltToSource::Distance.
		const JPH::Vec3 vLongitudinal = pWheel->GetContactLongitudinal();
		out.vContactLongitudinal = Vector( vLongitudinal.GetX(), vLongitudinal.GetY(), vLongitudinal.GetZ() );
	}

	return true;
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::HandleBoostKey()
{
	// Handle triggering boosting if the key is pressed and we aren't currently boosting or in cooldown.
	if ( m_ControlParams.boost && !m_InternalState.BoostDelay && !m_InternalState.BoosterRemainingTime )
	{
		m_InternalState.BoosterRemainingTime	= m_VehicleParams.engine.boostDuration;
		m_InternalState.BoostDelay				= m_VehicleParams.engine.boostDuration + m_VehicleParams.engine.boostDelay;
	}
}

void JoltPhysicsVehicleController::HandleBoostDecay()
{
	// Decay the boost time if we are currently boosting or have a delay.
	if ( m_VehicleParams.engine.boostDuration || m_VehicleParams.engine.boostDelay )
	{
		m_OperatingParams.boostTimeLeft = m_InternalState.BoostDelay
			? 100.0f - ( 100.0f * ( m_InternalState.BoostDelay / ( m_VehicleParams.engine.boostDuration + m_VehicleParams.engine.boostDelay ) ) )
			: 100.0f;
	}
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::OnPreSimulate( float flDeltaTime )
{
	// DetachObject nulls the chassis the moment its physics object dies, but the controller
	// stays in the environment's list until whoever created it gets round to destroying it.
	// Everything below dereferences the chassis or reads the constraint's body, so there is a
	// window of at least one step where this would be a use after free. Source never noticed
	// because the game tears the controller down in the same frame as the vehicle entity; a
	// Lua owned vehicle whose chassis prop is removed (undo, cleanup, map change) does not.
	if ( !m_pCarBodyObject )
		return;

	if ( IsAirboat() )
	{
		AirboatOnPreSimulate( flDeltaTime );
		return;
	}

	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
	// With any user input, assure that the car is active
	if ( m_ControlParams.steering != 0.0f || m_ControlParams.throttle != 0.0f || m_ControlParams.brake != 0.0f || m_ControlParams.handbrake )
		bodyInterface.ActivateBody( m_pCarBodyObject->GetBodyID() );

	bool bHandbrake = m_ControlParams.handbrake && !vjolt_vehicle_disable_handbrakes.GetBool();

	// Don't throttle when holding handbrake (like Source)
	float flThrottle = bHandbrake ? 0.0f : m_ControlParams.throttle;

	if ( vjolt_vehicle_throttle_override.GetFloat() > 0.0f )
		flThrottle = vjolt_vehicle_throttle_override.GetFloat();

	// Apply a little brake without throttle to stop the vehicle from coasting (like Source).
	const bool bCoasting = flThrottle == 0.0f && m_ControlParams.brake == 0.0f && !bHandbrake;
	float flBrake = bCoasting && !vjolt_vehicle_disable_autobrake.GetBool() ? 0.1f : m_ControlParams.brake;

	if ( vjolt_vehicle_disable_brake.GetBool() )
		flBrake = 0.0f;

	const float ThrottleOpositionSpeed = vjolt_vehicle_throttle_opposition_limit.GetFloat();

	// Enable the handbrake when going at low speeds to avoid slipping when going up hill.
	if ( ( flThrottle < 0.0f && m_OperatingParams.speed > ThrottleOpositionSpeed ) ||
		( flThrottle > 0.0f && m_OperatingParams.speed < -ThrottleOpositionSpeed ) )
		bHandbrake = !vjolt_vehicle_disable_handbrakes.GetBool();

	// Are we boosting?
	float flTotalTorqueMultiplier = 1.0f;
	if ( m_InternalState.BoosterRemainingTime != 0.0f )
	{
		GetWheeledVehicleController()->GetEngine().SetCurrentRPM(m_VehicleParams.engine.maxRPM);
		// Slam the throttle to 1, neeeowm!
		m_ControlParams.throttle = 1.0f;
		flThrottle = 1.0f;

		const float flSpeedFactor = RemapValClamped( fabsf( m_OperatingParams.speed ), 0, m_VehicleParams.engine.maxSpeed, 0.1f, 1.0f );
		const float flTurnFactor = 1.0f - ( fabsf( m_ControlParams.steering ) * 0.95f );
		// Josh: * 2 as the original torque stuff in Source was based around 0.5 being the max, and 1.0 being boost.
		const float flDampedBoost = 2.0f * m_VehicleParams.engine.boostForce * flSpeedFactor * flTurnFactor;

		if ( flDampedBoost > flTotalTorqueMultiplier )
			flTotalTorqueMultiplier = flDampedBoost;
	}

	// Update the torque factors as we may be boosting and be > 1. A live topology
	// change is unsupported by Source; leave the last valid differential map in
	// place instead of indexing the existing Jolt wheels with the new dimensions.
	JPH::WheeledVehicleController::Differentials &differentials =
		GetWheeledVehicleController()->GetDifferentials();
	if ( m_VehicleParams.axleCount == m_nCreatedAxleCount
		&& m_VehicleParams.wheelsPerAxle == m_nCreatedWheelsPerAxle )
	{
		const int differentialsPerAxle = GetDifferentialsPerAxle(
			m_VehicleParams.wheelsPerAxle );
		int differentialIndex = 0;
		for ( int axleIndex = 0; axleIndex < m_VehicleParams.axleCount; ++axleIndex )
		{
			for ( int differentialInAxle = 0;
				differentialInAxle < differentialsPerAxle; ++differentialInAxle )
			{
				if ( differentialIndex >= int( differentials.size() ) )
					break;
				auto &differential = differentials[ differentialIndex++ ];
				ConfigureVehicleDifferential( differential,
					m_VehicleParams, m_flTorqueScale, flTotalTorqueMultiplier,
					axleIndex, differentialInAxle );
				if ( m_BridgeVehicleConfig.bEnabled )
				{
					if ( differential.mLeftWheel >= 0 && ( m_Wheels[differential.mLeftWheel].carFlags & kVJoltCarWheelDetached ) )
						differential.mLeftWheel = -1;
					if ( differential.mRightWheel >= 0 && ( m_Wheels[differential.mRightWheel].carFlags & kVJoltCarWheelDetached ) )
						differential.mRightWheel = -1;
					if ( differential.mLeftWheel < 0 && differential.mRightWheel < 0 ) differential.mEngineTorqueRatio = 0;
				}
			}
		}
	}

	// A vehicle applies its complete effective gravity in its step listener. Do not also
	// apply the generic field correction to this chassis, and do not let OverrideGravity's
	// native factor=0 destroy the user's saved enable switch or multiplier.
	const auto gravity = VJoltGravityForObject( m_pCarBodyObject );
	m_VehicleConstraint->OverrideGravity( SourceToJolt::Distance( Vector( gravity.x, gravity.y, gravity.z ) ) );

	// Pass the input on to the constraint
	GetWheeledVehicleController()->SetDriverInput( flThrottle, m_ControlParams.steering, flBrake, bHandbrake ? 1.0f : 0.0f );

	// Set the collision tester
	m_VehicleConstraint->SetVehicleCollisionTester( m_Tester );
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::OnPostSimulate( float flDeltaTime )
{
	// See OnPreSimulate: no chassis means the constraint's body is gone too.
	if ( !m_pCarBodyObject )
		return;

	if ( IsAirboat() )
	{
		m_OperatingParams.speed = GetSpeed();
		m_OperatingParams.steeringAngle = RAD2DEG( m_AirboatState.m_SteeringAngle );
		m_OperatingParams.wheelsInContact = AirboatCountSurfaceContacts( m_AirboatImpacts );
		m_OperatingParams.wheelsNotInContact = m_nAirboatPontoons - m_OperatingParams.wheelsInContact;
		m_OperatingParams.boostDelay = m_InternalState.BoostDelay;
		HandleBoostDecay();
		return;
	}

	float flSteeringAngle = 0.0f;
	m_OperatingParams.wheelsInContact = 0;
	m_OperatingParams.wheelsNotInContact = 0;
	for ( int w = 0; w < GetWheelCount(); w++ )
	{
		if ( !m_Wheels[ w ].pObject )
			continue;
		// CFourWheelVehiclePhysics feeds -GetPosition().angles.z directly into
		// the stock wheel spin pose. Its proxy wheel therefore has an X axle
		// and Z up, as in IVP. The bridge's Y-axis render cylinder would put
		// spin into pitch and leave roll stuck between 180-degree flips.
		JPH::Mat44 wheelTransform = m_VehicleConstraint->GetWheelWorldTransform( w, JPH::Vec3::sAxisX(), JPH::Vec3::sAxisZ() );

		// Find our greatest steering angle.
		float flWheelSteeringAngle = JoltToSource::Angle( m_VehicleConstraint->GetWheels()[w]->GetSteerAngle() );
		if ( fabsf( flWheelSteeringAngle ) > fabsf( flSteeringAngle ) )
			flSteeringAngle = flWheelSteeringAngle;

		Vector newPos = JoltToSource::Distance( wheelTransform.GetTranslation() );
		QAngle newQuat = JoltToSource::Angle( wheelTransform.GetQuaternion() );
		m_Wheels[ w ].pObject->EnableCollisions( false );
		// Set dummy wheel object pos/angles so the game code can update pose positions for wheels.
		m_Wheels[ w ].pObject->SetPosition( newPos, newQuat, true );
		// Wake it up so that the game bothers to do pose positions.
		m_Wheels[ w ].pObject->Wake();

		if ( m_VehicleConstraint->GetWheels()[w]->HasContact() )
			m_OperatingParams.wheelsInContact++;
		else
			m_OperatingParams.wheelsNotInContact++;
	}

	m_OperatingParams.gear			= GetWheeledVehicleController()->GetTransmission().GetCurrentGear();
	m_OperatingParams.engineRPM		= GetWheeledVehicleController()->GetEngine().GetCurrentRPM();
	m_OperatingParams.speed			= GetSpeed();
	m_OperatingParams.steeringAngle = -flSteeringAngle;
	m_OperatingParams.boostDelay	= m_InternalState.BoostDelay;
	HandleBoostDecay();
}

//------------------------------------------------------------------------------------------------

JoltPhysicsWheel &JoltPhysicsVehicleController::CreateWheelObject(
	const vehicle_axleparams_t &axle, const Vector &wheelPositionWorld,
	float wheelRadius, float wheelWidth, JoltPhysicsObject *pRestoredWheel )
{
	// Josh: Area of a cylinder = π.h.r^2
	// Using radius in terms of Source units as we pass this to CreateSphereObject.
	const float wheelVolume = M_PI * wheelWidth * Cube( wheelRadius );

	if ( pRestoredWheel )
		m_Wheels.push_back( JoltPhysicsWheel{ .pObject = pRestoredWheel } );
	else
	{
		objectparams_t wheelParams =
		{
			.mass		= axle.wheels.mass,
			.inertia	= axle.wheels.inertia,
			.damping	= axle.wheels.damping,
			.rotdamping	= axle.wheels.rotdamping,
			.pName		= "VehicleWheel",
			.pGameData	= m_pCarBodyObject->GetGameData(),
			.volume		= wheelVolume,
		};
		IPhysicsObject *pWheelObject = m_pEnvironment->CreateSphereObject(
			wheelRadius, axle.wheels.materialIndex,
			// QAngle's default constructor leaves its components uninitialized.
			// The wheel can be simulated before OnPostSimulate writes its pose.
			wheelPositionWorld, QAngle( 0.0f, 0.0f, 0.0f ),
			&wheelParams, false );

		JoltPhysicsObject *pJoltWheelObject = static_cast< JoltPhysicsObject * >( pWheelObject );

		pJoltWheelObject->SetGameFlags( m_pCarBodyObject->GetGameFlags() );
		pJoltWheelObject->SetCallbackFlags( CALLBACK_IS_VEHICLE_WHEEL );
		// Josh: The wheel is a fake object, so disable collisions on it.
		pJoltWheelObject->EnableCollisions( false );

		m_Wheels.push_back( JoltPhysicsWheel{ .pObject = pJoltWheelObject } );
	}

	m_Wheels.back().pObject->AddDestroyedListener( this );
	return m_Wheels.back();
}

void JoltPhysicsVehicleController::CreateWheel( JPH::VehicleConstraintSettings &vehicleSettings, matrix3x4_t& bodyMatrix, int axleIdx, int wheelIdx, JoltPhysicsObject *pRestoredWheel )
{
	const vehicle_axleparams_t &axle = m_VehicleParams.axles[ axleIdx ];
	const int wheelIndex = axleIdx * m_VehicleParams.wheelsPerAxle + wheelIdx;
	const Vector wheelPositionLocal = axle.offset +
		( ( wheelIndex & 1 ) ? axle.wheelOffset : -axle.wheelOffset );

	Vector wheelPositionWorld;
	VectorTransform( wheelPositionLocal, bodyMatrix, wheelPositionWorld );

	// Josh: Good enough heuristic.
	const float wheelRadius = axle.wheels.radius;
	const float wheelWidth = wheelRadius / 2.0f;
	JoltPhysicsWheel &wheel = CreateWheelObject(
		axle, wheelPositionWorld, wheelRadius, wheelWidth, pRestoredWheel );

	const float steeringAngle = DEG2RAD( Max( m_VehicleParams.steering.degreesSlow, m_VehicleParams.steering.degreesFast ) );
	const float additionalLength = SourceToJolt::Distance( axle.wheels.springAdditionalLength );

	Vector gravity;
	m_pEnvironment->GetGravity( &gravity );

	JPH::WheelSettingsWV *wheelSettings = new JPH::WheelSettingsWV;
	wheel.pSettings = wheelSettings;
	wheelSettings->mPosition			= SourceToJolt::Distance( wheelPositionLocal );
	wheelSettings->mSuspensionDirection = JPH::Vec3( 0, 0, -1 );
	wheelSettings->mSteeringAxis		= JPH::Vec3( 0, 0, 1 );
	wheelSettings->mWheelUp				= JPH::Vec3( 0, 0, 1 );
	wheelSettings->mWheelForward		= m_vVehicleForward;
	wheelSettings->mAngularDamping		= axle.wheels.rotdamping;
	// TODO(Josh): What about more than 4 wheels?
	wheelSettings->mMaxSteerAngle		= axleIdx == 0 ? steeringAngle : 0.0f;
	wheelSettings->mRadius				= SourceToJolt::Distance( axle.wheels.radius );
	wheelSettings->mWidth				= SourceToJolt::Distance( wheelWidth );
	wheelSettings->mInertia				= 0.5f * axle.wheels.mass * ( wheelSettings->mRadius * wheelSettings->mRadius );
	if ( m_BridgeVehicleConfig.bEnabled )
	{
		const JoltVehicleBridgeAxleConfig_t &bridgeAxle = m_BridgeVehicleConfig.axles[ axleIdx ];
		wheelSettings->mSuspensionMinLength = SourceToJolt::Distance( bridgeAxle.flSuspensionMinLength );
		wheelSettings->mSuspensionMaxLength = SourceToJolt::Distance( bridgeAxle.flSuspensionMaxLength );
		wheelSettings->mSuspensionPreloadLength = SourceToJolt::Distance( bridgeAxle.flSuspensionPreloadLength );
	}
	else
	{
		wheelSettings->mSuspensionMinLength = 0;
		wheelSettings->mSuspensionMaxLength = additionalLength;
	}
	wheelSettings->mSuspensionSpring.mMode = JPH::ESpringMode::StiffnessAndDamping;
	// Source has these divided by the mass of the vehicle for some reason.
	// Convert these to a stiffness of k, in N/m...
	wheelSettings->mSuspensionSpring.mStiffness = axle.suspension.springConstant * m_pCarBodyObject->GetMass();
	wheelSettings->mSuspensionSpring.mDamping = axle.suspension.springDamping * m_pCarBodyObject->GetMass();
	if ( axle.wheels.frictionScale )
	{
		wheelSettings->mLateralFriction.AddPoint( 1.0f, axle.wheels.frictionScale );
		wheelSettings->mLongitudinalFriction.AddPoint( 1.0f, axle.wheels.frictionScale );

		// JPH::LinearCurve::AddPoint appends without sorting, and GetValue binary searches, so
		// the points have to stay ascending in X. Jolt's default lateral curve is already
		// populated out to 20 degrees of slip angle, so a point at 1.0 lands in the middle of
		// it and leaves the array unsorted. That made GetValue return the first point's Y
		// (zero) for every small slip angle, i.e. no lateral grip at all.
		wheelSettings->mLateralFriction.Sort();
		wheelSettings->mLongitudinalFriction.Sort();
	}
	wheel.baseLongitudinalFriction = wheelSettings->mLongitudinalFriction;
	wheel.baseLateralFriction = wheelSettings->mLateralFriction;

	// TODO: We may want to update this every pre-simulation to account for changing gravity.
	wheelSettings->mMaxBrakeTorque =
		0.5f *
		SourceToJolt::Distance( gravity.Length() ) *
		( m_pCarBodyObject->GetMass() + m_TotalWheelMass ) *
		axle.brakeFactor *
		SourceToJolt::Distance( axle.wheels.radius );
	wheelSettings->mMaxHandBrakeTorque = axleIdx == 0 ? 0.0f : wheelSettings->mMaxBrakeTorque * 2.0f;

	vehicleSettings.mWheels.push_back( wheelSettings );
	m_InternalState.LargestWheelRadius = Max( m_InternalState.LargestWheelRadius, wheelSettings->mRadius );
}

void JoltPhysicsVehicleController::CreateWheels( JPH::VehicleConstraintSettings &vehicleSettings, const std::vector< JoltPhysicsObject * > *pRestoredWheels )
{
	matrix3x4_t carBodyMtx = GetBodyMatrix();

	m_Wheels.reserve( m_VehicleParams.axleCount * m_VehicleParams.wheelsPerAxle );
	if ( m_VehicleParams.wheelsPerAxle == 2 )
		vehicleSettings.mAntiRollBars.reserve( m_VehicleParams.axleCount );

	m_TotalWheelMass = 0.0f;
	for ( int axle = 0; axle < m_VehicleParams.axleCount; axle++ )
		m_TotalWheelMass += m_VehicleParams.axles[ axle ].wheels.mass * m_VehicleParams.wheelsPerAxle;

	for ( int axle = 0; axle < m_VehicleParams.axleCount; axle++ )
	{
		for ( int wheel = 0; wheel < m_VehicleParams.wheelsPerAxle; wheel++ )
		{
			const int index = axle * m_VehicleParams.wheelsPerAxle + wheel;
			CreateWheel( vehicleSettings, carBodyMtx, axle, wheel,
				pRestoredWheels ? ( *pRestoredWheels )[ index ] : nullptr );
		}

		// Source's raycast vehicle only enables its stabilizer when there are
		// exactly two wheels per axle. Other topologies explicitly receive zero
		// stabilizer force.
		if ( m_VehicleParams.wheelsPerAxle == 2 )
		{
			JPH::VehicleAntiRollBar rollbar;
			rollbar.mLeftWheel	= ( axle * m_VehicleParams.wheelsPerAxle );
			rollbar.mRightWheel	= ( axle * m_VehicleParams.wheelsPerAxle ) + 1;
			rollbar.mStiffness	= m_VehicleParams.axles[ axle ].suspension.stabilizerConstant;
			vehicleSettings.mAntiRollBars.push_back( rollbar );
		}
	}
}

JPH::WheeledVehicleControllerSettings *JoltPhysicsVehicleController::CreateVehicleController()
{
	static constexpr float HorsePowerToWatts = 745.7f;

	JPH::WheeledVehicleControllerSettings *pController = new JPH::WheeledVehicleControllerSettings;
	// Josh:
	// T = ( 745.7 * P ) / ( 2 * PI * ( RPM / 60 ) )
	pController->mEngine.mMaxTorque = GetVehicleEngineTopologyScale(
		m_VehicleParams.wheelsPerAxle )
		* ( HorsePowerToWatts * m_VehicleParams.engine.horsepower )
		/ ( 2.0f * M_PI * ( m_VehicleParams.engine.maxRPM / 60.0f ) );
	// Josh: Fudge
	pController->mEngine.mMinRPM = Max( m_VehicleParams.engine.shiftDownRPM - 300, 0.0f );
	pController->mEngine.mMaxRPM = m_VehicleParams.engine.maxRPM;
	pController->mEngine.mAngularDamping = 0.0f;

	// Some vehicle scripts define just a single gear and have Autotransmission set to 0, in jolt the gear 0 is
	// neutral so it never moves, also some data like MaxSpeed doesn't really align, this has to do.
	pController->mTransmission.mMode = JPH::ETransmissionMode::Auto;
	pController->mTransmission.mGearRatios.clear();
	for ( int i = 0; i < m_VehicleParams.engine.gearCount; i++ )
		pController->mTransmission.mGearRatios.push_back( m_VehicleParams.engine.gearRatio[ i ] );

	pController->mTransmission.mReverseGearRatios.clear();
	pController->mTransmission.mReverseGearRatios.push_back( -m_VehicleParams.engine.gearRatio[0] );

	pController->mTransmission.mShiftUpRPM = m_VehicleParams.engine.shiftUpRPM;
	pController->mTransmission.mShiftDownRPM = m_VehicleParams.engine.shiftDownRPM;

	const int differentialsPerAxle = GetDifferentialsPerAxle(
		m_VehicleParams.wheelsPerAxle );
	pController->mDifferentials.reserve( GetVehicleDifferentialCount( m_VehicleParams ) );
	for ( int axleIndex = 0; axleIndex < m_VehicleParams.axleCount; ++axleIndex )
	{
		for ( int differentialInAxle = 0;
			differentialInAxle < differentialsPerAxle; ++differentialInAxle )
		{
			JPH::VehicleDifferentialSettings differential;
			ConfigureVehicleDifferential( differential, m_VehicleParams,
				m_flTorqueScale, 1.0f, axleIndex, differentialInAxle );
			pController->mDifferentials.push_back( differential );
		}
	}

	return pController;
}

JPH::WheeledVehicleController *JoltPhysicsVehicleController::GetWheeledVehicleController()
{
	return static_cast<JPH::WheeledVehicleController *>( m_VehicleConstraint->GetController() );
}

void JoltPhysicsVehicleController::ApplyVehicleControllerSettings(
	const JPH::WheeledVehicleControllerSettings &settings, bool applyDifferentials )
{
	JPH::WheeledVehicleController *pController = GetWheeledVehicleController();

	JPH::VehicleEngine &engine = pController->GetEngine();
	const float currentRPM = engine.GetCurrentRPM();
	static_cast< JPH::VehicleEngineSettings & >( engine ) = settings.mEngine;
	engine.SetCurrentRPM( currentRPM );

	JPH::VehicleTransmission &transmission = pController->GetTransmission();
	int currentGear = transmission.GetCurrentGear();
	const float clutchFriction = transmission.GetClutchFriction();
	static_cast< JPH::VehicleTransmissionSettings & >( transmission ) = settings.mTransmission;
	currentGear = clamp( currentGear,
		-int( transmission.mReverseGearRatios.size() ),
		int( transmission.mGearRatios.size() ) );
	transmission.Set( currentGear, clutchFriction );

	if ( applyDifferentials )
		pController->GetDifferentials() = settings.mDifferentials;
}

//------------------------------------------------------------------------------------------------

matrix3x4_t JoltPhysicsVehicleController::GetBodyMatrix() const
{
	matrix3x4_t value;
	m_pCarBodyObject->GetPositionMatrix( &value );
	return value;
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject )
{
	if ( m_pCarBodyObject == pObject )
		DetachObject();
	for ( auto &wheel : m_Wheels )
		if ( wheel.pObject == pObject )
			wheel.pObject = nullptr;
}

void JoltPhysicsVehicleController::ReleaseRestoredWheels()
{
	// A failed controller restore must not delete independently restored physics objects.
	for ( auto &wheel : m_Wheels )
		if ( wheel.pObject )
			wheel.pObject->RemoveDestroyedListener( this );
	m_Wheels.clear();
}

bool JoltPhysicsVehicleController::HasSaveableTopology() const
{
	if ( !m_pCarBodyObject || m_VehicleParams.axleCount != m_nCreatedAxleCount
		|| m_VehicleParams.wheelsPerAxle != m_nCreatedWheelsPerAxle )
		return false;
	for ( const auto &wheel : m_Wheels )
		if ( !wheel.pObject )
			return false;
	return !m_Wheels.empty();
}

static bool RestoreVehicleFrictionCurve( JPH::StreamIn &stream, JPH::LinearCurve &curve )
{
	// LinearCurve::RestoreBinaryState allocates from the embedded count before reading its
	// points. Tyre profiles are small; reject corrupt lengths before making that allocation.
	JPH::uint32 count = 0;
	stream.Read( count );
	if ( stream.IsFailed() || count > 1024 )
		return false;
	curve.Clear();
	curve.Reserve( count );
	for ( JPH::uint32 i = 0; i < count; ++i )
	{
		JPH::LinearCurve::Point point;
		stream.Read( point );
		if ( stream.IsFailed() || !std::isfinite( point.mX ) || !std::isfinite( point.mY )
			|| ( i > 0 && point.mX < curve.GetMaxX() ) )
			return false;
		curve.AddPoint( point.mX, point.mY );
	}
	return true;
}

static bool RestoreVehicleWheelSettings( JPH::StreamIn &stream, JPH::WheelSettingsWV &settings )
{
	// Same field order as WheelSettingsWV::SaveBinaryState, with bounded curve reads.
	settings.JPH::WheelSettings::RestoreBinaryState( stream );
	stream.Read( settings.mInertia );
	stream.Read( settings.mAngularDamping );
	stream.Read( settings.mMaxSteerAngle );
	if ( !RestoreVehicleFrictionCurve( stream, settings.mLongitudinalFriction )
		|| !RestoreVehicleFrictionCurve( stream, settings.mLateralFriction ) )
		return false;
	stream.Read( settings.mMaxBrakeTorque );
	stream.Read( settings.mMaxHandBrakeTorque );
	return !stream.IsFailed();
}

void JoltPhysicsVehicleController::SaveState( JPH::StateRecorder &recorder )
{
	recorder.Write( m_OperatingParams );
	recorder.Write( m_ControlParams );
	recorder.Write( m_InternalState );
	recorder.Write( m_TotalWheelMass );
	recorder.Write( m_flTorqueScale );
	recorder.Write( m_flBridgeEngineMinRPM );
	recorder.Write( m_flBridgePowerBandStartRPM );
	recorder.Write( m_flBridgePowerBandEndRPM );
	recorder.Write( m_flBridgeReverseGearRatio );
	recorder.Write( m_flBridgeDifferentialRatio );
	if ( IsAirboat() )
	{
		recorder.WriteBytes( &m_AirboatState, sizeof( m_AirboatState ) );
		for ( int i = 0; i < m_nAirboatPontoons; ++i )
			recorder.WriteBytes( &m_AirboatPontoons[ i ], sizeof( m_AirboatPontoons[ i ] ) );
		return;
	}

	GetWheeledVehicleController()->GetEngine().SaveState( recorder );
	GetWheeledVehicleController()->GetTransmission().SaveState( recorder );
	for ( size_t i = 0; i < m_Wheels.size(); ++i )
	{
		const JoltPhysicsWheel &wheel = m_Wheels[ i ];
		wheel.pSettings->SaveBinaryState( recorder );
		wheel.baseLongitudinalFriction.SaveBinaryState( recorder );
		wheel.baseLateralFriction.SaveBinaryState( recorder );
		recorder.Write( wheel.InWater );
		recorder.Write( wheel.Depth );
		const JPH::Wheel *pWheel = m_VehicleConstraint->GetWheels()[ i ];
		recorder.Write( pWheel->GetAngularVelocity() );
		recorder.Write( pWheel->GetRotationAngle() );
		recorder.Write( pWheel->GetSteerAngle() );
	}
	if ( !m_BridgeVehicleConfig.bEnabled ) return;
	// Controller state is the final item in its bounded Source save payload.
	// Append a versioned trailer so pre-wheel-damage saves remain readable.
	recorder.Write( uint32_t( 0x57484c31 ) ); // WHL1
	recorder.Write( uint32_t( m_Wheels.size() ) );
	for ( const auto &wheel : m_Wheels )
	{
		recorder.Write( wheel.frictionScale );
		recorder.Write( wheel.carGripScale );
		recorder.Write( wheel.carFlags );
	}
}

static bool IsVehicleStateAtEnd( const JPH::StateRecorder &recorder )
{
	// Jolt's IsEOF only becomes true after a failed read. An optional trailer
	// must instead inspect the boundary of the Source save payload.
	if ( const auto *source = dynamic_cast<const VJoltStateRecorder *>( &recorder ) )
		return source->GetBytesRemaining() == 0;
	return false;
}

bool JoltPhysicsVehicleController::RestoreState( JPH::StateRecorder &recorder )
{
	recorder.Read( m_OperatingParams );
	recorder.Read( m_ControlParams );
	recorder.Read( m_InternalState );
	recorder.Read( m_TotalWheelMass );
	recorder.Read( m_flTorqueScale );
	recorder.Read( m_flBridgeEngineMinRPM );
	recorder.Read( m_flBridgePowerBandStartRPM );
	recorder.Read( m_flBridgePowerBandEndRPM );
	recorder.Read( m_flBridgeReverseGearRatio );
	recorder.Read( m_flBridgeDifferentialRatio );
	if ( IsAirboat() )
	{
		recorder.ReadBytes( &m_AirboatState, sizeof( m_AirboatState ) );
		for ( int i = 0; i < m_nAirboatPontoons; ++i )
			recorder.ReadBytes( &m_AirboatPontoons[ i ], sizeof( m_AirboatPontoons[ i ] ) );
		return !VJoltStateRecorderFailed( recorder );
	}
	if ( VJoltStateRecorderFailed( recorder ) )
		return false;
	if ( m_flBridgeEngineMinRPM > 0.0f )
		ConfigureBridgeDrivetrain( m_flBridgeEngineMinRPM, m_flBridgePowerBandStartRPM,
			m_flBridgePowerBandEndRPM, m_flBridgeReverseGearRatio, m_flBridgeDifferentialRatio );

	// Contact BodyIDs and warm-start impulses belong to the old PhysicsSystem. Rebuild contacts
	// on the next step; only drivetrain and wheel motion are transferable to the new environment.
	GetWheeledVehicleController()->GetEngine().RestoreState( recorder );
	GetWheeledVehicleController()->GetTransmission().RestoreState( recorder );
	const auto &transmission = GetWheeledVehicleController()->GetTransmission();
	if ( VJoltStateRecorderFailed( recorder )
		|| transmission.GetCurrentGear() < -int( transmission.mReverseGearRatios.size() )
		|| transmission.GetCurrentGear() > int( transmission.mGearRatios.size() ) )
		return false;
	for ( size_t i = 0; i < m_Wheels.size(); ++i )
	{
		JoltPhysicsWheel &wheel = m_Wheels[ i ];
		if ( !RestoreVehicleWheelSettings( recorder, *wheel.pSettings )
			|| !RestoreVehicleFrictionCurve( recorder, wheel.baseLongitudinalFriction )
			|| !RestoreVehicleFrictionCurve( recorder, wheel.baseLateralFriction ) )
			return false;
		recorder.Read( wheel.InWater );
		recorder.Read( wheel.Depth );
		float angularVelocity = 0.0f, angle = 0.0f, steer = 0.0f;
		recorder.Read( angularVelocity );
		recorder.Read( angle );
		recorder.Read( steer );
		if ( VJoltStateRecorderFailed( recorder ) )
			return false;
		JPH::Wheel *pWheel = m_VehicleConstraint->GetWheels()[ i ];
		pWheel->SetAngularVelocity( angularVelocity );
		pWheel->SetRotationAngle( angle );
		pWheel->SetSteerAngle( steer );
	}
	if ( m_BridgeVehicleConfig.bEnabled && !IsVehicleStateAtEnd( recorder ) )
	{
		uint32_t magic = 0, count = 0;
		recorder.Read( magic );
		recorder.Read( count );
		if ( magic != 0x57484c31 || count != m_Wheels.size() ) return false;
		for ( auto &wheel : m_Wheels )
		{
			recorder.Read( wheel.frictionScale );
			recorder.Read( wheel.carGripScale );
			recorder.Read( wheel.carFlags );
			if ( !std::isfinite( wheel.frictionScale ) || wheel.frictionScale < 0
				|| !std::isfinite( wheel.carGripScale ) || wheel.carGripScale < .01f || wheel.carGripScale > 2
				|| ( wheel.carFlags & ~kVJoltCarWheelDetached ) ) return false;
		}
	}
	else
	{
		for ( auto &wheel : m_Wheels )
		{
			wheel.carFlags = 0;
			wheel.carGripScale = wheel.frictionScale = 1;
			const auto &base = wheel.baseLongitudinalFriction.mPoints;
			const auto &current = wheel.pSettings->mLongitudinalFriction.mPoints;
			for ( size_t i = 0; i < Min( base.size(), current.size() ); ++i )
				if ( base[i].mY > 0 ) { wheel.frictionScale = current[i].mY / base[i].mY; break; }
		}
	}
	return !VJoltStateRecorderFailed( recorder );
}

void JoltPhysicsVehicleController::DetachObject()
{
	if ( m_pCarBodyObject )
	{
		m_pCarBodyObject->RemoveConstraintAttachment();
		m_pCarBodyObject->RemoveDestroyedListener( this );

		// Remove the listeners and constraint now, we can never
		// attach to another body.
		if ( m_VehicleConstraint != nullptr )
		{
			m_pPhysicsSystem->RemoveConstraint( m_VehicleConstraint );
			m_pPhysicsSystem->RemoveStepListener( m_VehicleConstraint );
			VJoltGravityManageVehicle( m_pCarBodyObject, false );
		}

		m_pCarBodyObject = nullptr;
	}
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::ApplyImpulseAtPointMetric( const Vector &vImpulseMetric, const Vector &vWorldPosMetric )
{
	if ( !m_pCarBodyObject || !m_pCarBodyObject->GetBody() )
		return;

	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
	bodyInterface.AddImpulse(
		m_pCarBodyObject->GetBodyID(),
		JPH::Vec3( vImpulseMetric.x, vImpulseMetric.y, vImpulseMetric.z ),
		JPH::Vec3( vWorldPosMetric.x, vWorldPosMetric.y, vWorldPosMetric.z ) );
}

void JoltPhysicsVehicleController::ApplyImpulseCenterMetric( const Vector &vImpulseMetric )
{
	if ( !m_pCarBodyObject || !m_pCarBodyObject->GetBody() )
		return;

	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
	bodyInterface.AddImpulse(
		m_pCarBodyObject->GetBodyID(),
		JPH::Vec3( vImpulseMetric.x, vImpulseMetric.y, vImpulseMetric.z ) );
}

void JoltPhysicsVehicleController::ApplyAngularImpulseMetric( const Vector &vAngularImpulseMetric )
{
	if ( !m_pCarBodyObject || !m_pCarBodyObject->GetBody() )
		return;

	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
	bodyInterface.AddAngularImpulse(
		m_pCarBodyObject->GetBodyID(),
		JPH::Vec3( vAngularImpulseMetric.x, vAngularImpulseMetric.y, vAngularImpulseMetric.z ) );
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::InitAirboat( const vehicleparams_t &params, const std::vector< JoltPhysicsObject * > *pRestoredWheels )
{
	const float bodyMass = m_pCarBodyObject->GetMass();

	const int configuredPontoonCount = params.axleCount * params.wheelsPerAxle;
	m_nAirboatPontoons = Max( 0,
		Min( configuredPontoonCount, JOLT_AIRBOAT_MAX_PONTOONS ) );

	matrix3x4_t bodyMatrix = GetBodyMatrix();
	m_Wheels.reserve( m_nAirboatPontoons );

	int idx = 0;
	for ( int axleIdx = 0; axleIdx < params.axleCount
		&& idx < m_nAirboatPontoons; ++axleIdx )
	{
		const vehicle_axleparams_t &axle = params.axles[ axleIdx ];

		for ( int wheelIdx = 0; wheelIdx < params.wheelsPerAxle
			&& idx < m_nAirboatPontoons; ++wheelIdx, ++idx )
		{
			const Vector wheelOffsetSign = ( idx & 1 ) ? axle.wheelOffset : -axle.wheelOffset;
			const Vector traceOffsetSign = ( idx & 1 ) ? axle.raytraceOffset : -axle.raytraceOffset;

			const Vector wheelPositionLocal = axle.offset + wheelOffsetSign;
			const Vector tracePositionLocal = axle.raytraceCenterOffset + traceOffsetSign;
			Vector wheelPositionWorld;
			VectorTransform( wheelPositionLocal, bodyMatrix, wheelPositionWorld );
			const float wheelRadius = axle.wheels.radius;
			CreateWheelObject( axle, wheelPositionWorld,
				wheelRadius, wheelRadius / 2.0f, pRestoredWheels ? ( *pRestoredWheels )[ idx ] : nullptr );

			JoltAirboatPontoon &pontoon = m_AirboatPontoons[ idx ];
			pontoon.hp_cs				= wheelPositionLocal;
			pontoon.raycast_start_cs	= tracePositionLocal;
			pontoon.raycast_dir_cs		= Vector( 0.0f, 0.0f, -1.0f );
			pontoon.raycast_length		= AIRBOAT_RAYCAST_DIST;
			pontoon.spring_length		= SourceToJolt::Distance( axle.wheels.springAdditionalLength );
			pontoon.spring_constant		= axle.suspension.springConstant * bodyMass;
			pontoon.spring_damp_relax	= axle.suspension.springDamping * bodyMass;
			pontoon.spring_damp_compress = axle.suspension.springDampingCompression * bodyMass;
			pontoon.friction_of_wheel	= 1.0f;
			pontoon.wheel_radius		= SourceToJolt::Distance( axle.wheels.radius );
			pontoon.raycast_dist		= AIRBOAT_RAYCAST_DIST;
			pontoon.wheel_is_fixed		= true;
		}
	}

	m_pCarBodyObject->EnableGravity( false );
	m_pCarBodyObject->SetCallbackFlags( m_pCarBodyObject->GetCallbackFlags() & ~CALLBACK_DO_FLUID_SIMULATION );

}

void JoltPhysicsVehicleController::AirboatUpdate( float dt, vehicle_controlparams_t &controls )
{
	JoltAirboatState &state = m_AirboatState;

	JPH::BodyInterface &bodyInterface = m_pPhysicsSystem->GetBodyInterfaceNoLock();
	if ( controls.steering != 0.0f || controls.throttle != 0.0f || fabsf( state.m_flThrust ) > 0.01f )
		bodyInterface.ActivateBody( m_pCarBodyObject->GetBodyID() );

	float flThrottle = controls.throttle;
	const float flAbsSpeed = fabsf( m_OperatingParams.speed );
	const float flMaxSpeed = Max( m_VehicleParams.engine.maxSpeed, 1.0f );
	if ( flThrottle > 0.0f && flAbsSpeed > flMaxSpeed )
	{
		const float flFrac = flAbsSpeed / flMaxSpeed;
		if ( flFrac > m_VehicleParams.engine.autobrakeSpeedGain )
			flThrottle = 0.0f;
		flThrottle *= 0.1f;
	}
	if ( flThrottle < 0.0f && flAbsSpeed > m_VehicleParams.engine.maxRevSpeed )
		flThrottle *= 0.1f;

	if ( fabsf( flThrottle ) < 0.01f )
		state.m_flThrust = 0.0f;
	else if ( flThrottle > 0.0f )
		state.m_flThrust = AIRBOAT_THRUST_MAX * flThrottle;
	else
		state.m_flThrust = AIRBOAT_THRUST_MAX_REVERSE * flThrottle;

	state.m_bAnalogSteering	= controls.bAnalogSteering;
	state.m_SteeringAngle	= DEG2RAD( controls.steering * Max( m_VehicleParams.steering.degreesSlow, m_VehicleParams.steering.degreesFast ) );
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::AirboatOnPreSimulate( float flDeltaTime )
{
	if ( flDeltaTime <= 0.0f )
		return;

	{
		const float bodyMass = m_pCarBodyObject->GetMass();
		Vector vGravityImpulse( 0.0f, 0.0f, -AIRBOAT_GRAVITY * bodyMass * flDeltaTime );
		ApplyImpulseCenterMetric( vGravityImpulse );
	}

	matrix3x4_t matWorldFromCore = GetBodyMatrix();

	JPH::Vec3 jLinearMetric = m_pCarBodyObject->GetBody()->GetLinearVelocity();
	Vector vLinearMetric( jLinearMetric.GetX(), jLinearMetric.GetY(), jLinearMetric.GetZ() );
	m_AirboatState.m_flSpeed = vLinearMetric.Length();
	m_pCarBodyObject->WorldToLocalVector( &m_AirboatState.m_vecLocalVelocity, vLinearMetric );

	JoltAirboatImpact *pImpacts = m_AirboatImpacts;
	for ( int i = 0; i < m_nAirboatPontoons; ++i )
		pImpacts[i] = JoltAirboatImpact{};

	AirboatPreRaycasts( pImpacts, matWorldFromCore );
	AirboatDoRaycasts( pImpacts );
	if ( !AirboatPostRaycasts( pImpacts, matWorldFromCore ) )
		return;

	AirboatUpdateAirborne( pImpacts, flDeltaTime );

	AirboatDoPontoons( pImpacts, flDeltaTime );
	AirboatDoDrag( pImpacts, flDeltaTime, matWorldFromCore );
	AirboatDoTurbine( flDeltaTime, matWorldFromCore );
	AirboatDoSteering( flDeltaTime );
	AirboatDoKeepUprightPitch( pImpacts, flDeltaTime, matWorldFromCore );
	AirboatDoKeepUprightRoll( pImpacts, flDeltaTime, matWorldFromCore );
}

//------------------------------------------------------------------------------------------------

float JoltPhysicsVehicleController::AirboatComputeFrontPontoonWaveNoise( int nIndex, float flSpeedRatio, float flCurrentTime )
{
	const float flNoiseScale = RemapValClamped( 1.0f - flSpeedRatio, 0.0f, 1.0f, AIRBOAT_WATER_NOISE_MIN, AIRBOAT_WATER_NOISE_MAX );

	float flPhaseShift = 0.0f;
	if ( flSpeedRatio < 0.3f )
		flPhaseShift = float( nIndex ) * AIRBOAT_WATER_PHASE_MAX;

	const float flFrequency = RemapValClamped( flSpeedRatio, 0.0f, 1.0f, AIRBOAT_WATER_FREQ_MIN, AIRBOAT_WATER_FREQ_MAX );
	return flNoiseScale * sinf( flFrequency * ( flCurrentTime + flPhaseShift ) );
}

void JoltPhysicsVehicleController::AirboatPreRaycasts( JoltAirboatImpact *pImpacts, const matrix3x4_t &matWorldFromCore )
{
	(void)matWorldFromCore;
	for ( int i = 0; i < m_nAirboatPontoons; ++i )
	{
		JoltAirboatPontoon &pontoon = m_AirboatPontoons[ i ];
		pontoon.raycast_length = AIRBOAT_RAYCAST_DIST;
	}
}

void JoltPhysicsVehicleController::AirboatDoRaycasts( JoltAirboatImpact *pImpacts )
{
	if ( !m_pGameTrace )
		return;

	IPhysicsObject *pPhysAirboat = static_cast< IPhysicsObject * >( m_pCarBodyObject );

	const float flForwardSpeedRatioRaw = clamp( m_AirboatState.m_vecLocalVelocity.y / 10.0f, 0.0f, 1.0f );
	const float flSpeedRatio           = clamp( m_AirboatState.m_flSpeed / 15.0f, 0.0f, 1.0f );
	const float flForwardSpeedRatio    = m_AirboatState.m_flThrust ? flForwardSpeedRatioRaw : flForwardSpeedRatioRaw * 0.5f;
	const float flCurrentTime          = float( Plat_FloatTime() );

	matrix3x4_t matWorldFromCore = GetBodyMatrix();

	struct PontoonRayInfo
	{
		Vector start;
		Vector dirWorldSrc;
		float lengthMetric;
	};

	PontoonRayInfo rayInfo[ JOLT_AIRBOAT_MAX_PONTOONS ];

	int nFrontPontoonsInWater = 0;
	for ( int i = 0; i < m_nAirboatPontoons; ++i )
	{
		JoltAirboatPontoon &pontoon = m_AirboatPontoons[ i ];

		Vector startWorldSrc;
		VectorTransform( pontoon.raycast_start_cs, matWorldFromCore, startWorldSrc );

		Vector dirWorldSrc;
		VectorRotate( pontoon.raycast_dir_cs, matWorldFromCore, dirWorldSrc );

		pImpacts[i].raycast_dir_ws = dirWorldSrc;
		pImpacts[i].bInWater = m_pGameTrace->VehiclePointInWater( startWorldSrc );
		if ( pImpacts[i].bInWater )
			dirWorldSrc.Negate();

		float lengthMetric = pontoon.raycast_length;
		Vector endWorldSrc = startWorldSrc + dirWorldSrc * ( lengthMetric * JoltToSource::Factor );

		if ( m_pGameTrace->VehiclePointInWater( endWorldSrc ) )
		{
			lengthMetric = AIRBOAT_RAYCAST_DIST_WATER_LOW;
			if ( i < 2 )
			{
				++nFrontPontoonsInWater;
				lengthMetric += AirboatComputeFrontPontoonWaveNoise( i, flSpeedRatio, flCurrentTime );
			}
			endWorldSrc = startWorldSrc + dirWorldSrc * ( lengthMetric * JoltToSource::Factor );
		}

		rayInfo[i].start = startWorldSrc;
		rayInfo[i].dirWorldSrc = dirWorldSrc;
		rayInfo[i].lengthMetric = lengthMetric;
	}

	if ( nFrontPontoonsInWater == 2 )
	{
		for ( int i = 0; i < 2; ++i )
		{
			float lengthMetric = RemapValClamped( flForwardSpeedRatio, 0.0f, 1.0f, AIRBOAT_RAYCAST_DIST_WATER_LOW, AIRBOAT_RAYCAST_DIST_WATER_HIGH );
			lengthMetric += AirboatComputeFrontPontoonWaveNoise( i, flSpeedRatio, flCurrentTime );
			rayInfo[i].lengthMetric = lengthMetric;
		}
	}

	trace_t trace;
	for ( int i = 0; i < m_nAirboatPontoons; ++i )
	{
		JoltAirboatPontoon &pontoon = m_AirboatPontoons[ i ];
		JoltAirboatImpact &impact = pImpacts[ i ];

		pontoon.raycast_length = rayInfo[i].lengthMetric;

		Vector endWorldSrc = rayInfo[i].start + rayInfo[i].dirWorldSrc * ( rayInfo[i].lengthMetric * JoltToSource::Factor );

		Ray_t ray;
		ray.Init( rayInfo[i].start, endWorldSrc );

		if ( impact.bInWater )
		{
			m_pGameTrace->VehicleTraceRay( ray, pPhysAirboat->GetGameData(), &trace );

			Ray_t waterRay;
			Vector vecUp = rayInfo[i].start;
			vecUp.z += 1000.0f;
			waterRay.Init( rayInfo[i].start, vecUp );

			trace_t waterTrace;
			m_pGameTrace->VehicleTraceRayWithWater( waterRay, pPhysAirboat->GetGameData(), &waterTrace );
			impact.flDepth = 1000.0f * waterTrace.fractionleftsolid;
		}
		else
		{
			m_pGameTrace->VehicleTraceRayWithWater( ray, pPhysAirboat->GetGameData(), &trace );
		}

		impact.bImpact = false;
		impact.bImpactWater = false;

		if ( trace.fraction != 1.0f )
		{
			impact.bImpact = true;
			impact.flDepth = 0.0f;
			if ( trace.contents & MASK_WATER )
				impact.bImpactWater = true;

			impact.vecImpactPointWS = trace.endpos;
			impact.vecImpactNormalWS = trace.plane.normal;

			impact.nSurfaceProps = trace.surface.surfaceProps;

			const surfacedata_t *pSurface = JoltPhysicsSurfaceProps::GetInstance().GetSurfaceData( trace.surface.surfaceProps );
			if ( pSurface )
			{
				impact.flDampening = pSurface->physics.dampening;
				impact.flFriction = pSurface->physics.friction;
			}
		}
	}
}

bool JoltPhysicsVehicleController::AirboatPostRaycasts( JoltAirboatImpact *pImpacts, const matrix3x4_t &matWorldFromCore )
{
	bool bReturn = true;

	matrix3x4_t bodyMatrix = matWorldFromCore;

	for ( int i = 0; i < m_nAirboatPontoons; ++i )
	{
		JoltAirboatPontoon &pontoon = m_AirboatPontoons[ i ];
		JoltAirboatImpact &impact = pImpacts[ i ];

		if ( impact.bInWater )
			impact.raycast_dir_ws.Negate();

		Vector startWorldSrc;
		VectorTransform( pontoon.raycast_start_cs, bodyMatrix, startWorldSrc );

		if ( impact.bImpact )
		{
			Vector deltaSrc = impact.vecImpactPointWS - startWorldSrc;
			pontoon.raycast_dist = deltaSrc.Length() * SourceToJolt::Factor;

			float dotND = fabsf( impact.raycast_dir_ws.Dot( impact.vecImpactNormalWS ) );
			impact.inv_normal_dot_dir = 1.1f / ( dotND + 0.1f );
			impact.friction_value = impact.flFriction * pontoon.friction_of_wheel;
		}
		else
		{
			pontoon.raycast_dist = pontoon.raycast_length;
			impact.inv_normal_dot_dir = 1.0f;
			impact.vecImpactNormalWS = -impact.raycast_dir_ws;
			impact.friction_value = 1.0f;
			impact.vecImpactPointWS = startWorldSrc + impact.raycast_dir_ws * ( pontoon.raycast_dist * JoltToSource::Factor );
		}

		Vector surfaceSpeedSrc;
		m_pCarBodyObject->GetVelocityAtPoint( impact.vecImpactPointWS, &surfaceSpeedSrc );
		impact.surface_speed_wheel_ws = surfaceSpeedSrc * SourceToJolt::Factor;

		Vector n = impact.vecImpactNormalWS;
		float ns = n.Dot( impact.surface_speed_wheel_ws );
		impact.projected_surface_speed_wheel_ws = impact.surface_speed_wheel_ws - n * ns;
	}

	return bReturn;
}

//------------------------------------------------------------------------------------------------

int JoltPhysicsVehicleController::AirboatCountSurfaceContacts( JoltAirboatImpact *pImpacts )
{
	int nContacts = 0;
	for ( int i = 0; i < m_nAirboatPontoons; ++i )
	{
		if ( pImpacts[i].bImpact )
			++nContacts;
	}
	return nContacts;
}

void JoltPhysicsVehicleController::AirboatUpdateAirborne( JoltAirboatImpact *pImpacts, float flDeltaTime )
{
	JoltAirboatState &state = m_AirboatState;
	int nCount = AirboatCountSurfaceContacts( pImpacts );
	if ( !nCount )
	{
		if ( !state.m_bAirborne )
		{
			state.m_bAirborne = true;
			state.m_flAirTime = 0;
			if ( state.m_flSpeed < 11.0f )
				state.m_bWeakJump = true;
		}
		else
		{
			state.m_flAirTime += flDeltaTime;
		}
	}
	else
	{
		state.m_bAirborne = false;
		state.m_bWeakJump = false;
	}
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::AirboatDoPontoons( JoltAirboatImpact *pImpacts, float flDeltaTime )
{
	for ( int i = 0; i < m_nAirboatPontoons; ++i )
	{
		JoltAirboatPontoon &pontoon = m_AirboatPontoons[ i ];
		JoltAirboatImpact &impact = pImpacts[ i ];

		if ( impact.bImpact )
			AirboatDoPontoonGround( &pontoon, &impact, flDeltaTime );
		else if ( impact.bInWater )
			AirboatDoPontoonWater( &pontoon, &impact, flDeltaTime );
	}
}

void JoltPhysicsVehicleController::AirboatDoPontoonGround( JoltAirboatPontoon *pPontoon, JoltAirboatImpact *pImpact, float flDeltaTime )
{
	const float flDiff = pPontoon->raycast_dist - pPontoon->raycast_length;
	if ( flDiff >= 0.0f )
		return;

	float flForce = -flDiff * pPontoon->spring_constant;
	const float flInvNormalDotDir = clamp( pImpact->inv_normal_dot_dir, 0.0f, 3.0f );
	flForce *= flInvNormalDotDir;

	Vector vecSpeedDelta = pImpact->projected_surface_speed_wheel_ws - pImpact->surface_speed_wheel_ws;
	const float flSpeed = vecSpeedDelta.Dot( pImpact->raycast_dir_ws );
	if ( flSpeed > 0.0f )
		flForce -= pPontoon->spring_damp_relax * flSpeed;
	else
		flForce -= pPontoon->spring_damp_compress * flSpeed;

	if ( flForce < 0.0f )
		flForce = 0.0f;

	const float flImpulse = flForce * flDeltaTime;
	Vector vImpulseWS = pImpact->vecImpactNormalWS * flImpulse;

	ApplyImpulseAtPointMetric( vImpulseWS, pImpact->vecImpactPointWS * SourceToJolt::Factor );
}

void JoltPhysicsVehicleController::AirboatDoPontoonWater( JoltAirboatPontoon *pPontoon, JoltAirboatImpact *pImpact, float flDeltaTime )
{
	(void)pPontoon;

	const float flDepthMetric = clamp( pImpact->flDepth * SourceToJolt::Factor, 0.0f, AIRBOAT_PONTOON_HEIGHT );

	const float flSubmergedVolume = AIRBOAT_PONTOON_AREA_2D * flDepthMetric;
	const float bodyMass = m_pCarBodyObject->GetMass();
	const float flForce = AIRBOAT_BUOYANCY_SCALAR * 0.25f * bodyMass * flSubmergedVolume * 1000.0f;
	const float flImpulse = flForce * flDeltaTime;

	Vector vImpulseWS( 0.0f, 0.0f, flImpulse );
	ApplyImpulseAtPointMetric( vImpulseWS, pImpact->vecImpactPointWS * SourceToJolt::Factor );
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::AirboatDoDrag( JoltAirboatImpact *pImpacts, float flDeltaTime, const matrix3x4_t &matWorldFromCore )
{
	const float flSpeed = m_AirboatState.m_flSpeed;

	int nPointsInWater = 0;
	int nPointsOnGround = 0;
	float flGroundFriction = 0.0f;
	for ( int i = 0; i < m_nAirboatPontoons; ++i )
	{
		const JoltAirboatImpact &impact = pImpacts[i];
		if ( !impact.bImpact )
			continue;

		if ( impact.bImpactWater )
		{
			++nPointsInWater;
		}
		else
		{
			++nPointsOnGround;
			flGroundFriction += impact.flFriction;
		}
	}

	const float bodyMass = m_pCarBodyObject->GetMass();

	if ( nPointsInWater )
	{
		Vector negDirLS = -m_AirboatState.m_vecLocalVelocity;

		Vector vDragLS(
			AIRBOAT_WATER_DRAG_LEFT_RIGHT  * negDirLS.x,
			AIRBOAT_WATER_DRAG_FORWARD_BACK * negDirLS.y,
			AIRBOAT_WATER_DRAG_UP_DOWN     * negDirLS.z );

		vDragLS *= flSpeed * bodyMass * flDeltaTime;

		Vector vDragWS;
		VectorRotate( vDragLS, matWorldFromCore, vDragWS );
		ApplyImpulseCenterMetric( vDragWS );
	}

	if ( nPointsOnGround && flSpeed > 0.0f )
	{
		flGroundFriction /= float( nPointsOnGround );

		float flFrictionDrag = bodyMass * AIRBOAT_GRAVITY * AIRBOAT_DRY_FRICTION_SCALE * flGroundFriction;
		flFrictionDrag /= flSpeed;

		Vector negDirLS = -m_AirboatState.m_vecLocalVelocity;
		Vector vDragLS(
			AIRBOAT_GROUND_DRAG_LEFT_RIGHT  * negDirLS.x,
			AIRBOAT_GROUND_DRAG_FORWARD_BACK * negDirLS.y,
			AIRBOAT_GROUND_DRAG_UP_DOWN     * negDirLS.z );

		vDragLS *= flFrictionDrag * flDeltaTime;

		Vector vDragWS;
		VectorRotate( vDragLS, matWorldFromCore, vDragWS );
		ApplyImpulseCenterMetric( vDragWS );
	}
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::AirboatDoTurbine( float flDeltaTime, const matrix3x4_t &matWorldFromCore )
{
	float flThrust = m_AirboatState.m_flThrust;
	if ( m_AirboatState.m_bWeakJump || ( m_AirboatState.m_bAirborne && flThrust < 0.0f ) )
		flThrust *= 0.5f;

	Vector vForwardLocal( 0.0f, 1.0f, 0.0f );
	Vector vForwardWS;
	VectorRotate( vForwardLocal, matWorldFromCore, vForwardWS );

	if ( vForwardWS.z > 0.5f && flThrust > 0.0f )
	{
		float flFactor = 1.0f - vForwardWS.z;
		flThrust *= flFactor;
	}
	else if ( vForwardWS.z < -0.5f && flThrust < 0.0f )
	{
		float flFactor = 1.0f + vForwardWS.z;
		flThrust *= flFactor;
	}

	const float bodyMass = m_pCarBodyObject->GetMass();
	Vector vImpulse = vForwardWS * ( flThrust * bodyMass * flDeltaTime );
	ApplyImpulseCenterMetric( vImpulse );
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::AirboatDoSteering( float flDeltaTime )
{
	JoltAirboatState &state = m_AirboatState;

	if ( state.m_SteeringAngle == 0.0f || state.m_flThrust != 0.0f )
	{
		if ( !state.m_bAnalogSteering )
		{
			if ( state.m_flThrust < 0.0f )
				state.m_bSteeringReversed = true;
			else if ( state.m_flThrust > 0.0f || state.m_vecLocalVelocity.y > 0.0f )
				state.m_bSteeringReversed = false;
		}
		else
		{
			if ( state.m_flThrust < -2.0f )
				state.m_bSteeringReversed = true;
			else if ( state.m_flThrust > 2.0f || state.m_vecLocalVelocity.y > 0.0f )
				state.m_bSteeringReversed = false;
		}
	}

	const float bodyMass = m_pCarBodyObject->GetMass();
	const float invDt = ( flDeltaTime > 0.0f ) ? ( 1.0f / flDeltaTime ) : 0.0f;

	float flForceSteering = 0.0f;
	if ( fabsf( state.m_SteeringAngle ) > 0.01f )
	{
		float flSteeringSign = state.m_SteeringAngle < 0.0f ? -1.0f : 1.0f;
		if ( state.m_bSteeringReversed )
			flSteeringSign *= -1.0f;

		float flPrevSign = state.m_flPrevSteeringAngle < 0.0f ? -1.0f : 1.0f;
		if ( fabsf( state.m_flPrevSteeringAngle ) < 0.01f || flSteeringSign != flPrevSign )
			state.m_flSteerTime = 0.0f;

		float flSteerScale = 0.0f;
		if ( !state.m_bAnalogSteering )
			flSteerScale = RemapValClamped( state.m_flSteerTime, 0.0f, AIRBOAT_STEERING_INTERVAL, AIRBOAT_STEERING_RATE_MIN, AIRBOAT_STEERING_RATE_MAX );
		else
			flSteerScale = RemapValClamped( fabsf( state.m_SteeringAngle ), 0.0f, AIRBOAT_STEERING_INTERVAL, AIRBOAT_STEERING_RATE_MIN, AIRBOAT_STEERING_RATE_MAX );

		flForceSteering = flSteerScale * bodyMass * invDt;
		flForceSteering *= -flSteeringSign;

		state.m_flSteerTime += flDeltaTime;
	}

	state.m_flPrevSteeringAngle = state.m_SteeringAngle * ( state.m_bSteeringReversed ? -1.0f : 1.0f );

	JPH::Vec3 jAngWorld = m_pCarBodyObject->GetBody()->GetAngularVelocity();
	Vector vAngWorld( jAngWorld.GetX(), jAngWorld.GetY(), jAngWorld.GetZ() );
	Vector vAngLocal;
	m_pCarBodyObject->WorldToLocalVector( &vAngLocal, vAngWorld );
	const float yawRate = -vAngLocal.z;
	const float yawSign = yawRate < 0.0f ? -1.0f : 1.0f;

	const float flRotDrag = AIRBOAT_ROT_DRAG * yawRate * yawRate * bodyMass * invDt * yawSign;
	const float flRotDamp = AIRBOAT_ROT_DAMPING * fabsf( yawRate ) * bodyMass * invDt * yawSign;

	const float flForceRotational = flForceSteering + flRotDrag + flRotDamp;

	Vector vAngImpLocal( 0.0f, 0.0f, flForceRotational );
	Vector vAngImpWorld;
	matrix3x4_t bodyMatrix = GetBodyMatrix();
	VectorRotate( vAngImpLocal, bodyMatrix, vAngImpWorld );
	ApplyAngularImpulseMetric( vAngImpWorld );
}

//------------------------------------------------------------------------------------------------

void JoltPhysicsVehicleController::AirboatDoKeepUprightPitch( JoltAirboatImpact *pImpacts, float flDeltaTime, const matrix3x4_t &matWorldFromCore )
{
	JoltAirboatState &state = m_AirboatState;
	if ( state.m_bWeakJump )
		return;

	if ( flDeltaTime <= 0.0f )
		return;

	const float kCos10 = cosf( DEG2RAD( 10.0f ) );
	const float kSin10 = sinf( DEG2RAD( 10.0f ) );
	Vector vUpCS( 0.0f, kSin10, kCos10 );

	Vector vGoalAxisWS( 0.0f, 0.0f, 1.0f );

	Vector vGoalAxisCS;
	VectorIRotate( vGoalAxisWS, matWorldFromCore, vGoalAxisCS );

	vGoalAxisCS.x = vUpCS.x;
	VectorNormalize( vGoalAxisCS );

	Vector vRotAxisCS = vUpCS.Cross( vGoalAxisCS );
	const float cosine = vUpCS.Dot( vGoalAxisCS );
	float sine = VectorNormalize( vRotAxisCS );
	const float angle = atan2f( sine, cosine );

	if ( AirboatCountSurfaceContacts( pImpacts ) > 0 )
	{
		state.m_flPitchErrorPrev = angle;
		return;
	}

	const float bodyMass = m_pCarBodyObject->GetMass();
	const float invDt = 1.0f / flDeltaTime;
	Vector vAngImpCS = vRotAxisCS * ( bodyMass * ( 0.1f * angle + 0.04f * invDt * ( angle - state.m_flPitchErrorPrev ) ) );
	state.m_flPitchErrorPrev = angle;

	float len = VectorNormalize( vAngImpCS );
	const float maxLen = DEG2RAD( 1.5f ) * bodyMass;
	if ( len > maxLen )
		len = maxLen;
	vAngImpCS *= len;

	Vector vAngImpWS;
	VectorRotate( vAngImpCS, matWorldFromCore, vAngImpWS );
	ApplyAngularImpulseMetric( vAngImpWS );
}

void JoltPhysicsVehicleController::AirboatDoKeepUprightRoll( JoltAirboatImpact *pImpacts, float flDeltaTime, const matrix3x4_t &matWorldFromCore )
{
	JoltAirboatState &state = m_AirboatState;
	if ( flDeltaTime <= 0.0f )
		return;

	const float kCos10 = cosf( DEG2RAD( 10.0f ) );
	const float kSin10 = sinf( DEG2RAD( 10.0f ) );
	Vector vUpCS( 0.0f, kSin10, kCos10 );

	Vector vGoalAxisWS( 0.0f, 0.0f, 1.0f );

	Vector vGoalAxisCS;
	VectorIRotate( vGoalAxisWS, matWorldFromCore, vGoalAxisCS );

	vGoalAxisCS.z = vUpCS.z;
	VectorNormalize( vGoalAxisCS );

	Vector vRotAxisCS = vUpCS.Cross( vGoalAxisCS );
	const float cosine = vUpCS.Dot( vGoalAxisCS );
	float sine = VectorNormalize( vRotAxisCS );
	const float angle = atan2f( sine, cosine );

	if ( AirboatCountSurfaceContacts( pImpacts ) > 0 )
	{
		state.m_flRollErrorPrev = angle;
		return;
	}

	if ( fabsf( angle ) < DEG2RAD( 10.0f ) )
	{
		state.m_flRollErrorPrev = angle;
		return;
	}

	const float bodyMass = m_pCarBodyObject->GetMass();
	const float invDt = 1.0f / flDeltaTime;
	Vector vAngImpCS = vRotAxisCS * ( bodyMass * ( 0.2f * angle + 0.3f * invDt * ( angle - state.m_flRollErrorPrev ) ) );
	state.m_flRollErrorPrev = angle;

	float len = VectorNormalize( vAngImpCS );
	const float maxLen = DEG2RAD( 2.0f ) * bodyMass;
	if ( len > maxLen )
		len = maxLen;
	vAngImpCS *= len;

	Vector vAngImpWS;
	VectorRotate( vAngImpCS, matWorldFromCore, vAngImpWS );
	ApplyAngularImpulseMetric( vAngImpWS );
}
