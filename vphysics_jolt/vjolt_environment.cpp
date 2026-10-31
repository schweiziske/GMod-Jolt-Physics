//=================================================================================================
//
// Interface to a physics scene
// Physics environments are implemented as discrete JPH::PhysicsSystems.
// Portal and Portal 2 have two of these.
//
// Notes:
//  Josh: We always use BodyInterfaceNoLock and deal with unlocked bodies now.
//  Jolt starts to get very angry and asserts at us for doing that but we are never simulating
//  or having it do things while the bodies are technically unlocked -- but it doesn't matter.
//  We need to do this so we can assume the lock during the callbacks for collisions and stuff.
//
//=================================================================================================

#include "cbase.h"
#include "bridge/vjolt_gravity_internal.h"
#include "bridge/vjolt_constraints_internal.h"
#include "bridge/vjolt_path_internal.h"
#include "bridge/vjolt_softbody_internal.h"
#include "coordsize.h"

#include "vjolt_collide.h"
#include "vjolt_constraints.h"
#include "vjolt_controller_drag.h"
#include "vjolt_controller_fluid.h"
#include "vjolt_controller_motion.h"
#include "vjolt_controller_player.h"
#include "vjolt_controller_shadow.h"
#include "vjolt_controller_vehicle.h"
#include "vjolt_layers.h"
#include "vjolt_object.h"
#include "bridge/vjolt_stats_internal.h"

#include "vjolt_environment.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <iterator>
#include <optional>
#include <vjolt/cpu_policy.h>
#include <Jolt/Physics/Collision/Shape/EmptyShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

// This is the max amount of rigid bodies that you can add to the physics system. If you try to add more you'll get an error.
static constexpr uint kMaxBodies = 65536;

// This determines how many mutexes to allocate to protect rigid bodies from concurrent access. Set it to 0 for the default settings.
static constexpr uint kNumBodyMutexes = 0;

// This is the max amount of body pairs that can be queued at any time (the broad phase will detect overlapping
// body pairs based on their bounding boxes and will insert them into a queue for the narrowphase). If you make this buffer
// too small the queue will fill up and the broad phase jobs will start to do narrow phase work. This is slightly less efficient.
static constexpr uint kMaxBodyPairs = kMaxBodies;

// This is the maximum size of the contact constraint buffer. If more contacts (collisions between bodies) are detected than this
// number then these contacts will be ignored and bodies will start interpenetrating / fall through the world.
static constexpr uint kMaxContactConstraints = kMaxBodies;

static ConVar vjolt_linearcast( "vjolt_linearcast", "0", FCVAR_NONE, "Enable native Jolt LinearCast CCD for newly created environments, including native NPC bodies. Disabled by default; restart the map after changing." );
static ConVar vjolt_enhanced_inactive_edge_detection( "vjolt_enhanced_inactive_edge_detection", "1", FCVAR_NONE, "Whether bodies will be created with enhanced inactive edge detection (only takes effect after map restart)." );
static ConVar vjolt_initial_simulation( "vjolt_initial_simulation", "0", FCVAR_NONE, "Whether to pre-settle physics objects on map load." );
static ConVar vjolt_friction_scale( "vjolt_friction_scale", "1", FCVAR_NONE,
	"Contact friction multiplier: 0 disables contact friction, 1 uses normal materials (range 0-10). Applies next simulation; wakes sleeping bodies. Does not change sv_friction, drag, damping or joint friction.",
	true, 0.0f, true, 10.0f );

// Sample Source cvars on the simulation owner thread, never from contact jobs.
// Environments share the backend's job system/allocator and run sequentially.
static std::atomic<float> s_ContactFrictionScale{ 1.0f };

float VJoltScaleContactFriction( float friction )
{
	return friction * s_ContactFrictionScale.load( std::memory_order_relaxed );
}

static ConVar vjolt_collision_callbacks( "vjolt_collision_callbacks", "1", FCVAR_NONE,
	"Synchronous collision events: 0 disabled, 1 batch dispatch, 2 per-impact dispatch (comparison).",
	true, 0.0f, true, 2.0f );

// A Source/IVP PSI includes controller callbacks, pending velocity commits, wake propagation,
// contact solving and the resulting contact snapshot. Keep this cvar as a compatibility override,
// but interpret it as a minimum number of complete PSI passes, never as Jolt collision substeps.
static ConVar vjolt_substeps( "vjolt_substeps", "1", FCVAR_NONE,
	"Minimum number of complete VPhysics PSI steps per Simulate call.", true, 1.0f, true, 8.0f );

static constexpr int kMaximumPhysicsPSIStepsPerCall = 8;

static int ConsumePhysicsPSISteps( double &ioAccumulator, double stepTime, int nMinimumSteps )
{
	// Keep the accumulator in double precision so a long-running server does not slowly lose
	// fractional PSI time. The epsilon only absorbs float representation noise at tick boundaries.
	constexpr double kAccumulatorEpsilon = 1.0e-9;
	const int nDueSteps = clamp( static_cast< int >(
		( ioAccumulator + kAccumulatorEpsilon ) / stepTime ),
		0, kMaximumPhysicsPSIStepsPerCall );
	// A caller may update more often than its configured PSI. Only advance after enough time
	// has elapsed; the normal game explicitly sets this interval to its server tick interval.
	const int nSteps = nDueSteps > 0 ? Max( nMinimumSteps, nDueSteps ) : 0;

	// Explicit extra steps are an administrator precision override. Consume only elapsed external
	// time from the accumulator; forced steps intentionally run ahead of wall-clock time and are
	// not turned into negative time debt.
	ioAccumulator -= static_cast< double >( nDueSteps ) * stepTime;
	if ( ioAccumulator < 0.0 )
		ioAccumulator = 0.0;
	return nSteps;
}

static ConVar vjolt_velocity_steps( "vjolt_velocity_steps", "10", FCVAR_NONE, "Number of velocity steps to perform.", true, 0.0f, true, 64.0f );
static ConVar vjolt_position_steps( "vjolt_position_steps", "2", FCVAR_NONE, "Number of velocity steps to perform.", true, 0.0f, true, 64.0f );
static ConVar vjolt_mass_ratio_iterations( "vjolt_mass_ratio_iterations", "0", FCVAR_NONE,
	"Enable extra island solver iteration floors for high original mass ratios. Disabled by default; independent of contact mass scaling. Applies next simulation." );
static ConVar vjolt_mass_ratio_threshold( "vjolt_mass_ratio_threshold", "10", FCVAR_NONE,
	"Original island mass ratio that enables optional extra iterations. Requires vjolt_mass_ratio_iterations. Applies next simulation.", true, 1.0f, true, 1000000.0f );
static ConVar vjolt_mass_ratio_velocity_steps( "vjolt_mass_ratio_velocity_steps", "30", FCVAR_NONE,
	"Minimum velocity iterations when the optional mass-ratio policy triggers. Applies next simulation.", true, 0.0f, true, 64.0f );
static ConVar vjolt_mass_ratio_position_steps( "vjolt_mass_ratio_position_steps", "4", FCVAR_NONE,
	"Minimum position iterations when the optional mass-ratio policy triggers. Applies next simulation.", true, 0.0f, true, 64.0f );
static ConVar vjolt_deterministic( "vjolt_deterministic", "0", FCVAR_NONE, "Whether the simulation is deterministic or not." );

static ConVar vjolt_baumgarte_factor( "vjolt_baumgarte_factor", "0.2", FCVAR_NONE, "Baumgarte stabilization factor (how much of the position error to 'fix' in 1 update). Changing this may help with constraint stability. Requires a map restart to change.", true, 0.0f, true, 1.0f );

class JoltObjectLayerPairFilter final : public JPH::ObjectLayerPairFilter
{
public:
	// Function that determines if two object layers can collide
	bool ShouldCollide( JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2 ) const override
	{
		switch ( inObject1 )
		{
		// NO_COLLIDE collides with nothing.
		case Layers::NO_COLLIDE:
			return	false;
		// NON_MOVING collides with moving objects and debris.
		case Layers::NON_MOVING_WORLD:
		case Layers::NON_MOVING_OBJECT:
			return	inObject2 == Layers::MOVING ||
					inObject2 == Layers::NATIVE_NPC ||
					inObject2 == Layers::DEBRIS;
		// MOVING collides with moving and non-moving objects.
		case Layers::MOVING:
			return	inObject2 == Layers::MOVING ||
					inObject2 == Layers::NATIVE_NPC ||
					inObject2 == Layers::NON_MOVING_WORLD ||
					inObject2 == Layers::NON_MOVING_OBJECT;
		case Layers::NATIVE_NPC:
			return inObject2 == Layers::MOVING || inObject2 == Layers::NON_MOVING_WORLD ||
			       inObject2 == Layers::NON_MOVING_OBJECT;
	
		// DEBRIS only collides with non-moving objects.
		case Layers::DEBRIS:
			return	inObject2 == Layers::NON_MOVING_WORLD || inObject2 == Layers::NON_MOVING_OBJECT;
		default:
			VJoltAssert( false );
			return false;
	}
};

private:
};

// BroadPhaseLayerInterface implementation
// This defines a mapping between object and broadphase layers.
class JoltBroadPhaseLayerInterface final : public JPH::BroadPhaseLayerInterface
{
public:
	JoltBroadPhaseLayerInterface()
	{
		// Create a mapping table from object to broad phase layer
		mObjectToBroadPhase[Layers::NON_MOVING_WORLD] = BroadPhaseLayers::NON_MOVING_WORLD;
		mObjectToBroadPhase[Layers::NON_MOVING_OBJECT] = BroadPhaseLayers::NON_MOVING_OBJECT;
		mObjectToBroadPhase[Layers::MOVING] = BroadPhaseLayers::MOVING;
		mObjectToBroadPhase[Layers::NO_COLLIDE] = BroadPhaseLayers::NO_COLLIDE;
		mObjectToBroadPhase[Layers::DEBRIS] = BroadPhaseLayers::DEBRIS;
		mObjectToBroadPhase[Layers::NATIVE_NPC] = BroadPhaseLayers::NATIVE_NPC;
	}

	uint GetNumBroadPhaseLayers() const override
	{
		return Layers::NUM_LAYERS;
	}

	JPH::BroadPhaseLayer GetBroadPhaseLayer( JPH::ObjectLayer inLayer ) const override
	{
		VJoltAssert( inLayer < Layers::NUM_LAYERS );
		return mObjectToBroadPhase[inLayer];
	}

#if defined( JPH_EXTERNAL_PROFILE ) || defined( JPH_PROFILE_ENABLED )
	const char *GetBroadPhaseLayerName( JPH::BroadPhaseLayer inLayer ) const override
	{
		switch ( (JPH::BroadPhaseLayer::Type)inLayer )
		{
		case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING_WORLD:	return "NON_MOVING_WORLD";
		case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::NON_MOVING_OBJECT:	return "NON_MOVING_OBJECT";
		case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::NO_COLLIDE:			return "NO_COLLIDE";
		case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::DEBRIS:				return "DEBRIS";
		case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::MOVING:				return "MOVING";
		case (JPH::BroadPhaseLayer::Type)BroadPhaseLayers::NATIVE_NPC: return "NATIVE_NPC";
		default:																VJoltAssert( false ); return "INVALID";
		}
	}
#endif

private:
	JPH::BroadPhaseLayer mObjectToBroadPhase[Layers::NUM_LAYERS];
};

class JoltObjectVsBroadPhaseLayerFilter final : public JPH::ObjectVsBroadPhaseLayerFilter
{
public:
	// Function that determines if two broadphase layers can collide
	bool ShouldCollide( JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2 ) const override
	{
		switch (inLayer1)
		{
		// NO_COLLIDE collides with nothing.
		case Layers::NO_COLLIDE:
			return false;

		// NON_MOVING collides with moving objects and debris.
		case Layers::NON_MOVING_WORLD:
		case Layers::NON_MOVING_OBJECT:
			return inLayer2 == BroadPhaseLayers::MOVING ||
				   inLayer2 == BroadPhaseLayers::NATIVE_NPC ||
				   inLayer2 == BroadPhaseLayers::DEBRIS;

		// MOVING collides with moving and non-moving objects.
		case Layers::MOVING:
			return inLayer2 == BroadPhaseLayers::MOVING ||
				   inLayer2 == BroadPhaseLayers::NATIVE_NPC ||
				   inLayer2 == BroadPhaseLayers::NON_MOVING_WORLD ||
				   inLayer2 == BroadPhaseLayers::NON_MOVING_OBJECT;
		case Layers::NATIVE_NPC:
			return inLayer2 == BroadPhaseLayers::MOVING || inLayer2 == BroadPhaseLayers::NON_MOVING_WORLD ||
			       inLayer2 == BroadPhaseLayers::NON_MOVING_OBJECT;
	
		// DEBRIS only collides with non-moving objects.
		case Layers::DEBRIS:
			return inLayer2 == BroadPhaseLayers::NON_MOVING_WORLD || inLayer2 == BroadPhaseLayers::NON_MOVING_OBJECT;

		default:
			VJoltAssert( false );
			return false;
		}
	}
private:
};

//-------------------------------------------------------------------------------------------------

JoltBroadPhaseLayerInterface JoltPhysicsEnvironment::s_BroadPhaseLayerInterface;
JoltObjectVsBroadPhaseLayerFilter JoltPhysicsEnvironment::s_BroadPhaseFilter;
JoltObjectLayerPairFilter JoltPhysicsEnvironment::s_LayerPairFilter;

//-------------------------------------------------------------------------------------------------

JoltPhysicsActivationListener::JoltPhysicsActivationListener( JPH::PhysicsSystem &physicsSystem )
	: m_PhysicsSystem( physicsSystem )
{
}

void JoltPhysicsActivationListener::OnBodyActivated( const JPH::BodyID &inBodyID, JPH::uint64 inBodyUserData )
{
	if ( !inBodyUserData ) return;
	// Activation inside an Update records that PSI; activation between Updates records the
	// last completed one. Either way the body's first full PSI is the next one.
	if ( m_pPSIClock )
		reinterpret_cast< JoltPhysicsObject * >( inBodyUserData )->MarkRevived(
			m_pPSIClock->load( std::memory_order_relaxed ) );
	m_Callbacks.Push( { inBodyID, inBodyUserData, true } );
}

void JoltPhysicsActivationListener::OnBodyDeactivated( const JPH::BodyID &inBodyID, JPH::uint64 inBodyUserData )
{
	if ( !inBodyUserData ) return;
	m_Callbacks.Push( { inBodyID, inBodyUserData, false } );
}

void JoltPhysicsActivationListener::SetGameListener( IPhysicsObjectEvent *pListener )
{
	m_Callbacks.SetListener( pListener );
}

IPhysicsObjectEvent *JoltPhysicsActivationListener::GetGameListener() const
{
	return m_Callbacks.GetListener();
}

void JoltPhysicsActivationListener::FlushCallbacks()
{
	auto batch = m_Callbacks.TakeBatch();
	if ( !batch.pListener )
		return;

	const JPH::BodyLockInterfaceNoLock &bodyInterface = m_PhysicsSystem.GetBodyLockInterfaceNoLock();
	for ( const ActivationEvent &event : batch.events )
	{
		// A game callback may replace its own handler. Stop immediately so the remaining
		// events cannot be delivered through a stale pointer after that replacement.
		if ( !m_Callbacks.IsCurrent( batch ) )
			break;

		JPH::Body *pBody = bodyInterface.TryGetBody( event.bodyID );
		if ( !pBody || pBody->GetUserData() != event.bodyUserData )
			continue;

		JoltPhysicsObject *pObject = reinterpret_cast< JoltPhysicsObject * >( event.bodyUserData );
		if ( !pObject || ( pObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) )
			continue;

		if ( event.activated )
			batch.pListener->ObjectWake( pObject );
		else
			batch.pListener->ObjectSleep( pObject );
	}
}

void JoltPhysicsActivationListener::DiscardCallbacks()
{
	m_Callbacks.Discard();
}

void JoltPhysicsActivationListener::DiscardBodyCallbacks( const JPH::BodyID &body )
{
	m_Callbacks.DiscardIf( [&body]( const ActivationEvent &event ) { return event.bodyID == body; } );
}

//-------------------------------------------------------------------------------------------------

class JoltPhysicsSpring;

struct JoltSpringImpulse
{
	JPH::Body *m_pBody;
	JPH::RVec3 m_vWorldPoint;
	JPH::Vec3 m_vImpulse;
};

class JoltPhysicsSpringDamper final : public JPH::PhysicsStepListener
{
public:
	void Register( JoltPhysicsSpring *pSpring )
	{
		if ( !VectorContains( m_pSprings, pSpring ) )
			m_pSprings.push_back( pSpring );
	}

	void Unregister( JoltPhysicsSpring *pSpring )
	{
		Erase( m_pSprings, pSpring );
	}

	void OnStep( const JPH::PhysicsStepListenerContext &inContext ) override;
	uint32 GetSpringCount() const { return static_cast< uint32 >( m_pSprings.size() ); }
	void DetachObjectForTransfer( JoltPhysicsObject *pObject );

private:
	std::vector< JoltPhysicsSpring * > m_pSprings;
	std::vector< JoltSpringImpulse > m_PendingImpulses;
};

JoltPhysicsEnvironment::JoltPhysicsEnvironment()
	: m_ActivationListener( m_PhysicsSystem )
	, m_ContactListener( m_PhysicsSystem )
{
	m_PerformanceParams.Defaults();

	m_PhysicsSystem.Init(
		kMaxBodies, kNumBodyMutexes, kMaxBodyPairs, kMaxContactConstraints,
		s_BroadPhaseLayerInterface, s_BroadPhaseFilter, s_LayerPairFilter);

	// Drag, springs and vehicle listeners may access the same chassis or wheel-contact
	// body. Keep their per-substep callbacks ordered; the physics solver remains parallel.
	JPH::PhysicsSettings settings = m_PhysicsSystem.GetPhysicsSettings();
	settings.mStepListenerBatchesPerJob = INT_MAX;
	m_PhysicsSystem.SetPhysicsSettings( settings );

	// A body activation listener gets notified when bodies activate and go to sleep
	// Note that this is called from a job so whatever you do here needs to be thread safe.
	// Registering one is entirely optional.
	m_ActivationListener.SetPSIClock( &m_ContactListener.GetPSIClock() );
	m_PhysicsSystem.SetBodyActivationListener( &m_ActivationListener );

	// A contact listener gets notified when bodies (are about to) collide, and when they separate again.
	// Note that this is called from a job so whatever you do here needs to be thread safe.
	// Registering one is entirely optional.
	m_PhysicsSystem.SetContactListener( &m_ContactListener );

	// Match IVP's CIVPMaterialManager::get_friction_factor: product clamped to [0, 1].
	// Jolt's geometric mean gives systematically higher friction than IVP for the same
	// surfaceprop pair, which makes thrown objects stick instead of slide.
	m_PhysicsSystem.SetCombineFriction( []( const JPH::Body &inBody1, const JPH::SubShapeID &inSubShapeID1, const JPH::Body &inBody2, const JPH::SubShapeID &inSubShapeID2 ) -> float
	{
		return VJoltScaleContactFriction( Clamp<float>(
			inBody1.GetFriction() * inBody2.GetFriction(), 0.0f, 1.0f ) );
	} );

	// IVP multiplies the two surface values, but its impact solver treats that product as
	// retained energy and gives back sqrt(product) of the incoming normal speed. Jolt expects
	// a coefficient of restitution (the speed ratio) directly, so convert the semantics here.
	// Clamp first because raw surfaceprop values can be far above 1 (e.g. Metal_bouncy = 1000).
	m_PhysicsSystem.SetCombineRestitution( []( const JPH::Body &inBody1, const JPH::SubShapeID& inSubShapeID1, const JPH::Body &inBody2, const JPH::SubShapeID& inSubShapeID2 ) -> float
	{
		const float retainedEnergy = Clamp<float>(
			inBody1.GetRestitution() * inBody2.GetRestitution(), 0.0f, 1.0f );
		return std::sqrt( retainedEnergy );
	} );

	// Set our linear cast member
	m_bUseLinearCast = vjolt_linearcast.GetBool();
	m_bUseEnhancedEdgeDetection = vjolt_enhanced_inactive_edge_detection.GetBool();

	m_pDragController = new JoltPhysicsDragController();
	m_PhysicsSystem.AddStepListener( m_pDragController );

	m_pSpringDamper = new JoltPhysicsSpringDamper();
	m_PhysicsSystem.AddStepListener( m_pSpringDamper );
}

JoltPhysicsEnvironment::~JoltPhysicsEnvironment()
{
	VJoltSoftBodyEnvironmentDestroyed( this );
	VJoltConstraintsEnvironmentDestroyed( this );
	VJoltPathEnvironmentDestroyed( this );
	// Removing bodies can generate deactivation notifications. Stop routing them before object
	// teardown so no queued callback can outlive this environment or its game listener.
	m_PhysicsSystem.SetBodyActivationListener( nullptr );
	m_ActivationListener.SetGameListener( nullptr );
	m_ActivationListener.DiscardCallbacks();

	DestroyEnvironmentObjects();

	m_PhysicsSystem.RemoveStepListener( m_pDragController );
	delete m_pDragController;
	m_pDragController = nullptr;

	m_PhysicsSystem.RemoveStepListener( m_pSpringDamper );
	delete m_pSpringDamper;
	m_pSpringDamper = nullptr;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::SetDebugOverlay( CreateInterfaceFn debugOverlayFactory )
{
	m_pDebugOverlay = nullptr;
	if ( debugOverlayFactory )
	{
		m_pDebugOverlay = (IVJoltDebugOverlay *)debugOverlayFactory( VJOLT_DEBUG_OVERLAY_VERSION, nullptr );

		JoltPhysicsInterface::GetInstance().SetDebugOverlay( m_pDebugOverlay );
	}
}

IVPhysicsDebugOverlay *JoltPhysicsEnvironment::GetDebugOverlay()
{
	// Slart: For some reason this is part of the vphysics interface, nothing ever uses it
	// outside of vphysics and we shouldn't either, we want to be able to pick between the
	// full debugoverlay or the vphysics one based on a compile-time parameter
	return nullptr;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::SetGravity( const Vector &gravityVector )
{
	if ( IsInContactCallback() && DeferPhysicsMutation( [this, gravityVector]() { SetGravity( gravityVector ); } ) ) return;
	JPH::Vec3 gravity = SourceToJolt::Distance( gravityVector );
	if ( gravity == m_PhysicsSystem.GetGravity() ) return;
	m_PhysicsSystem.SetGravity( gravity );
	VJoltGravityWorldChanged( this );
}

void JoltPhysicsEnvironment::GetGravity( Vector *pGravityVector ) const
{
	VJoltAssert( pGravityVector );
	*pGravityVector = JoltToSource::Distance( m_PhysicsSystem.GetGravity() );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::SetAirDensity( float density )
{
	m_pDragController->SetAirDensity( density );
}

float JoltPhysicsEnvironment::GetAirDensity() const
{
	return m_pDragController->GetAirDensity();
}

//-------------------------------------------------------------------------------------------------

static objectparams_t NormalizeObjectParams( objectparams_t* pParams )
{
	objectparams_t params = *pParams;
	params.mass = Clamp( pParams->mass, VPHYSICS_MIN_MASS, VPHYSICS_MAX_MASS );

	return params;
}

static float NormalizeInertiaScale( float inertia )
{
	if ( inertia <= 0.0f )
		return 1.0f;
	return Min( inertia, 1.0e14f );
}

static JPH::Vec3 ApplyRotInertiaLimit( JPH::Vec3Arg inertia, float rotInertiaLimit )
{
	if ( rotInertiaLimit == 0.0f )
		return inertia;

	const float minInertia = inertia.Length() * rotInertiaLimit;
	return JPH::Vec3::sMax( inertia, JPH::Vec3::sReplicate( minInertia ) );
}

static JPH::MassProperties BuildSourceMassProperties(
	const JPH::Shape *pShape,
	const JoltCollideUserData *pCollideData,
	const objectparams_t &params )
{
	JPH::Vec3 inertiaPerMass;
	if ( pCollideData && pCollideData->hasSourceMassProperties )
	{
		inertiaPerMass = pCollideData->sourceRotationInertiaPerMass;
	}
	else
	{
		// Procedural collision shapes do not carry an IVP compact surface. Use
		// Jolt's COM-space tensor as the fallback, but retain Source's diagonal
		// body-axis representation and never include a COM override translation.
		JPH::MassProperties shapeMassProperties = pShape->GetMassProperties();
		shapeMassProperties.ScaleToMass( 1.0f );
		inertiaPerMass = JPH::Vec3(
			shapeMassProperties.mInertia( 0, 0 ),
			shapeMassProperties.mInertia( 1, 1 ),
			shapeMassProperties.mInertia( 2, 2 ) );
	}

	JPH::Vec3 inertia = inertiaPerMass * ( params.mass * NormalizeInertiaScale( params.inertia ) );
	inertia = ApplyRotInertiaLimit( inertia, params.rotInertiaLimit );

	JPH::MassProperties result;
	result.mMass = params.mass;
	result.mInertia = JPH::Mat44::sScale( inertia );
	return result;
}

//-------------------------------------------------------------------------------------------------

JPH::Body *JoltPhysicsEnvironment::CreateBodyForObject( const JPH::BodyCreationSettings &settings )
{
	auto &bodies = m_PhysicsSystem.GetBodyInterfaceNoLock();
	if ( IsInContactCallback() )
	{
		// Allocation is independent of the locked body list. Return a usable Source
		// object now, but publish its ID/broadphase entry only at the safe boundary.
		JPH::Body *body = bodies.CreateBodyWithoutID( settings );
		if ( !body ) return nullptr;
		DeferPhysicsMutation( [this, body]()
		{
			auto *object = reinterpret_cast<JoltPhysicsObject *>( body->GetUserData() );
			if ( object->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) return;
			auto &bodyInterface = m_PhysicsSystem.GetBodyInterfaceNoLock();
			if ( bodyInterface.AssignBodyID( body ) )
				bodyInterface.AddBody( body->GetID(), JPH::EActivation::DontActivate );
			else
				DestroyObject( object );
		} );
		return body;
	}
	JPH::Body *body = bodies.CreateBody( settings );
	if ( body ) bodies.AddBody( body->GetID(), JPH::EActivation::DontActivate );
	return body;
}

IPhysicsObject *JoltPhysicsEnvironment::CreatePolyObject( const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams )
{
	objectparams_t params = NormalizeObjectParams( pParams );

	const JPH::Shape *pBaseShape = pCollisionModel->ToShape();
	const JoltCollideUserData *pCollideData = pCollisionModel->GetUserData();
	JPH::Vec3 massCenter = pCollideData && pCollideData->hasSourceMassProperties
		? pCollideData->sourceMassCenter
		: pBaseShape->GetCenterOfMass();
	if ( params.massCenterOverride )
		massCenter = SourceToJolt::Distance( *params.massCenterOverride );
	JPH::RefConst<JPH::Shape> shape = CreateCOMOverrideShape( pBaseShape, massCenter );

	JPH::BodyCreationSettings settings( shape.GetPtr(), SourceToJolt::Distance( position ), SourceToJolt::Angle( angles ), JPH::EMotionType::Dynamic, Layers::MOVING );
	settings.mMassPropertiesOverride = BuildSourceMassProperties( pBaseShape, pCollideData, params );
	settings.mOverrideMassProperties = JPH::EOverrideMassProperties::MassAndInertiaProvided;
	settings.mMaxLinearVelocity = SourceToJolt::Distance( MaxVelocity() );
	settings.mMaxAngularVelocity = DEG2RAD( MaxAngularVelocity() );

	if ( m_bUseLinearCast )
		settings.mMotionQuality = JPH::EMotionQuality::LinearCast;

	if ( m_bUseEnhancedEdgeDetection )
		settings.mEnhancedInternalEdgeRemoval = true;

	JPH::Body *pBody = CreateBodyForObject( settings );
	return pBody ? new JoltPhysicsObject( pBody, this, false, materialIndex, &params ) : nullptr;
}

IPhysicsObject *JoltPhysicsEnvironment::CreatePolyObjectStatic( const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams )
{
	objectparams_t params = NormalizeObjectParams( pParams );

	JPH::BodyCreationSettings settings( pCollisionModel->ToShape(), SourceToJolt::Distance( position ), SourceToJolt::Angle( angles ), JPH::EMotionType::Static, Layers::NON_MOVING_WORLD );

	if ( m_bUseLinearCast )
		settings.mMotionQuality = JPH::EMotionQuality::LinearCast;

	if ( m_bUseEnhancedEdgeDetection )
		settings.mEnhancedInternalEdgeRemoval = true;

	JPH::Body *pBody = CreateBodyForObject( settings );
	return pBody ? new JoltPhysicsObject( pBody, this, true, materialIndex, &params ) : nullptr;
}

IPhysicsObject *JoltPhysicsEnvironment::CreateSphereObject( float radius, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams, bool isStatic )
{
	objectparams_t params = NormalizeObjectParams( pParams );

	const float joltRadius = SourceToJolt::Distance( radius );
	JPH::RefConst<JPH::Shape> shape = new JPH::SphereShape( joltRadius );

	JPH::EMotionType motionType = isStatic ? JPH::EMotionType::Static : JPH::EMotionType::Dynamic;
	JPH::ObjectLayer objectLayer = isStatic ? Layers::NON_MOVING_WORLD : Layers::MOVING;

	JPH::BodyCreationSettings settings( shape.GetPtr(), SourceToJolt::Distance( position ), SourceToJolt::Angle( angles ), motionType, objectLayer );

	if ( !isStatic )
	{
		if ( m_bUseLinearCast )
			settings.mMotionQuality = JPH::EMotionQuality::LinearCast;

		const float sphereInertia = 0.4f * params.mass * joltRadius * joltRadius * NormalizeInertiaScale( params.inertia );
		const JPH::Vec3 inertia = ApplyRotInertiaLimit( JPH::Vec3::sReplicate( sphereInertia ), params.rotInertiaLimit );
		settings.mMassPropertiesOverride.mMass = params.mass;
		settings.mMassPropertiesOverride.mInertia = JPH::Mat44::sScale( inertia );
		settings.mOverrideMassProperties = JPH::EOverrideMassProperties::MassAndInertiaProvided;
		settings.mMaxLinearVelocity = SourceToJolt::Distance( MaxVelocity() );
		settings.mMaxAngularVelocity = DEG2RAD( MaxAngularVelocity() );
	}

	JPH::Body *pBody = CreateBodyForObject( settings );
	return pBody ? new JoltPhysicsObject( pBody, this, isStatic, materialIndex, &params ) : nullptr;
}

//-------------------------------------------------------------------------------------------------

IPhysicsFluidController *JoltPhysicsEnvironment::CreateFluidController( IPhysicsObject *pFluidObject, fluidparams_t *pParams )
{
	JoltPhysicsObject *pJoltObject = static_cast< JoltPhysicsObject * >( pFluidObject );
	JoltPhysicsFluidController *pFluidController = new JoltPhysicsFluidController( &m_PhysicsSystem, pJoltObject, pParams );
	RegisterPhysicsController( pFluidController );
	return pFluidController;
}

void JoltPhysicsEnvironment::DestroyFluidController( IPhysicsFluidController *pFluidController )
{
	JoltPhysicsFluidController *pInternalFluidController = static_cast<JoltPhysicsFluidController *>( pFluidController );
	UnregisterPhysicsController( pInternalFluidController );
	delete pInternalFluidController;
}

//-------------------------------------------------------------------------------------------------

class JoltPhysicsSpring final : public IPhysicsSpring, public IJoltObjectDestroyedListener
{
public:
	JoltPhysicsSpring( JPH::PhysicsSystem *pPhysicsSystem, JoltPhysicsSpringDamper *pDamper, JoltPhysicsObject *pObjectStart, JoltPhysicsObject *pObjectEnd, springparams_t *pParams );
	~JoltPhysicsSpring() override;

	void GetEndpoints( Vector *worldPositionStart, Vector *worldPositionEnd ) override;
	void SetSpringConstant( float flSpringConstant ) override;
	void SetSpringDamping( float flSpringDamping ) override;
	void SetSpringLength( float flSpringLength ) override;

	IPhysicsObject *GetStartObject() override;
	IPhysicsObject *GetEndObject() override;
	void GetSaveParams( springparams_t &params ) const;

	void CalculateImpulse( float flDeltaTime, std::vector< JoltSpringImpulse > &pendingImpulses );
	void OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject );

private:
	void DetachObjects();

	JPH::PhysicsSystem *m_pPhysicsSystem = nullptr;
	JoltPhysicsSpringDamper *m_pDamper = nullptr;

	JoltPhysicsObject *m_pObjectStart = nullptr;
	JoltPhysicsObject *m_pObjectEnd = nullptr;

	JPH::DistanceConstraint *m_pConstraint = nullptr;
	bool m_OnlyStretch = false;
	float m_flSpringConstant = 0.0f;
	float m_flNaturalLength = 0.0f;
	float m_flSpringDamping = 0.0f;
	float m_flRelativeDamping = 0.0f;
};

//-------------------------------------------------------------------------------------------------

JoltPhysicsSpring::JoltPhysicsSpring( JPH::PhysicsSystem *pPhysicsSystem, JoltPhysicsSpringDamper *pDamper, JoltPhysicsObject *pObjectStart, JoltPhysicsObject *pObjectEnd, springparams_t *pParams )
	: m_pPhysicsSystem( pPhysicsSystem )
	, m_pDamper( pDamper )
	, m_pObjectStart( pObjectStart )
	, m_pObjectEnd( pObjectEnd )
	, m_OnlyStretch( pParams->onlyStretch )
	, m_flSpringConstant( pParams->constant )
	, m_flNaturalLength( SourceToJolt::Distance( pParams->naturalLength ) )
	, m_flSpringDamping( pParams->damping )
	, m_flRelativeDamping( pParams->relativeDamping )
{
	JPH::Body *refBody = m_pObjectStart->GetBody();
	JPH::Body *attBody = m_pObjectEnd->GetBody();

	JPH::DistanceConstraintSettings settings;
	settings.mSpace = pParams->useLocalPositions ? JPH::EConstraintSpace::LocalToBodyCOM : JPH::EConstraintSpace::WorldSpace;
	settings.mPoint1 = SourceToJolt::Distance( pParams->startPosition );
	settings.mPoint2 = SourceToJolt::Distance( pParams->endPosition );
	if ( pParams->useLocalPositions )
	{
		// Source local spring endpoints are relative to the physics-object origin;
		// Jolt's LocalToBodyCOM constraint space is relative to each body's COM.
		settings.mPoint1 -= refBody->GetShape()->GetCenterOfMass();
		settings.mPoint2 -= attBody->GetShape()->GetCenterOfMass();
	}
	// The real IVP spring is an actuator, not a soft distance constraint. Keep a no-op distance
	// constraint solely to preserve body-island and activation linkage; OnStep applies the exact
	// IVP actuator impulse after all springs have sampled the same step-start state.
	settings.mMinDistance = 0.0f;
	settings.mMaxDistance = FLT_MAX;

	m_pConstraint = static_cast< JPH::DistanceConstraint * >( settings.Create( *refBody, *attBody ) );
	m_pConstraint->SetEnabled( true );

	m_pPhysicsSystem->AddConstraint( m_pConstraint );

	m_pObjectStart->AddConstraintAttachment();
	m_pObjectEnd->AddConstraintAttachment();
	m_pObjectStart->AddDestroyedListener( this );
	m_pObjectEnd->AddDestroyedListener( this );

	m_pDamper->Register( this );
	m_pObjectStart->Wake();
	m_pObjectEnd->Wake();
}

JoltPhysicsSpring::~JoltPhysicsSpring()
{
	if ( m_pDamper )
		m_pDamper->Unregister( this );

	DetachObjects();

	if ( m_pConstraint )
		m_pPhysicsSystem->RemoveConstraint( m_pConstraint );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsSpring::GetEndpoints( Vector *worldPositionStart, Vector *worldPositionEnd )
{
	if ( !m_pConstraint || !m_pObjectStart || !m_pObjectEnd )
		return;

	if ( worldPositionStart )
	{
		const JPH::RVec3 worldStart = m_pObjectStart->GetBody()->GetCenterOfMassTransform()
			* m_pConstraint->GetConstraintToBody1Matrix().GetTranslation();
		*worldPositionStart = JoltToSource::Distance( JPH::Vec3( worldStart ) );
	}

	if ( worldPositionEnd )
	{
		const JPH::RVec3 worldEnd = m_pObjectEnd->GetBody()->GetCenterOfMassTransform()
			* m_pConstraint->GetConstraintToBody2Matrix().GetTranslation();
		*worldPositionEnd = JoltToSource::Distance( JPH::Vec3( worldEnd ) );
	}
}

void JoltPhysicsSpring::DetachObjects()
{
	if ( m_pObjectStart )
	{
		m_pObjectStart->RemoveConstraintAttachment();
		m_pObjectStart->RemoveDestroyedListener( this );
		m_pObjectStart = nullptr;
	}

	if ( m_pObjectEnd )
	{
		m_pObjectEnd->RemoveConstraintAttachment();
		m_pObjectEnd->RemoveDestroyedListener( this );
		m_pObjectEnd = nullptr;
	}
}

void JoltPhysicsSpring::SetSpringConstant( float flSpringConstant )
{
	if ( !m_pConstraint )
		return;

	m_pObjectStart->Wake();
	m_pObjectEnd->Wake();

	m_flSpringConstant = flSpringConstant;
}

void JoltPhysicsSpring::SetSpringDamping( float flSpringDamping )
{
	if ( !m_pConstraint )
		return;

	m_pObjectStart->Wake();
	m_pObjectEnd->Wake();

	m_flSpringDamping = flSpringDamping;
}

void JoltPhysicsSpring::SetSpringLength( float flSpringLength )
{
	if ( !m_pConstraint )
		return;

	const float flLength = SourceToJolt::Distance( flSpringLength );
	if ( fabsf( flLength - m_flNaturalLength ) < SourceToJolt::Distance( 1.0e-3f ) )
		return;
	m_flNaturalLength = flLength;
	m_pObjectStart->Wake();
	m_pObjectEnd->Wake();
}

//-------------------------------------------------------------------------------------------------

IPhysicsObject *JoltPhysicsSpring::GetStartObject()
{
	return m_pObjectStart;
}

IPhysicsObject *JoltPhysicsSpring::GetEndObject()
{
	return m_pObjectEnd;
}

void JoltPhysicsSpring::GetSaveParams( springparams_t &params ) const
{
	params = {};
	params.startPosition = JoltToSource::Distance(
		m_pConstraint->GetConstraintToBody1Matrix().GetTranslation()
		+ m_pObjectStart->GetBody()->GetShape()->GetCenterOfMass() );
	params.endPosition = JoltToSource::Distance(
		m_pConstraint->GetConstraintToBody2Matrix().GetTranslation()
		+ m_pObjectEnd->GetBody()->GetShape()->GetCenterOfMass() );
	params.naturalLength = JoltToSource::Distance( m_flNaturalLength );
	params.constant = m_flSpringConstant;
	params.damping = m_flSpringDamping;
	params.relativeDamping = m_flRelativeDamping;
	params.onlyStretch = m_OnlyStretch;
	params.useLocalPositions = true;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsSpring::CalculateImpulse( float flDeltaTime, std::vector< JoltSpringImpulse > &pendingImpulses )
{
	if ( flDeltaTime <= 0.0f || !m_pConstraint || !m_pObjectStart || !m_pObjectEnd )
		return;

	JPH::Body *pBodyStart = m_pObjectStart->GetBody();
	JPH::Body *pBodyEnd = m_pObjectEnd->GetBody();
	if ( !pBodyStart || !pBodyEnd )
		return;

	const bool bStartDynamic = pBodyStart->IsDynamic() && pBodyStart->IsActive();
	const bool bEndDynamic = pBodyEnd->IsDynamic() && pBodyEnd->IsActive();
	if ( !bStartDynamic && !bEndDynamic )
		return;

	const JPH::RVec3 vWorldStart = pBodyStart->GetCenterOfMassTransform() * m_pConstraint->GetConstraintToBody1Matrix().GetTranslation();
	const JPH::RVec3 vWorldEnd = pBodyEnd->GetCenterOfMassTransform() * m_pConstraint->GetConstraintToBody2Matrix().GetTranslation();
	const JPH::Vec3 vArmStart = JPH::Vec3( vWorldStart - pBodyStart->GetCenterOfMassPosition() );
	const JPH::Vec3 vArmEnd = JPH::Vec3( vWorldEnd - pBodyEnd->GetCenterOfMassPosition() );

	JPH::Vec3 vDirection = JPH::Vec3( vWorldStart - vWorldEnd );
	const float flLength = vDirection.Length();
	if ( flLength < FLT_EPSILON || ( m_OnlyStretch && flLength <= m_flNaturalLength ) )
		return;
	vDirection /= flLength;

	const JPH::Vec3 vRelativeVelocity =
		pBodyEnd->GetPointVelocityCOM( vArmEnd ) - pBodyStart->GetPointVelocityCOM( vArmStart );
	const float flAxialSpeed = vDirection.Dot( vRelativeVelocity );
	const float flForce = ( flLength - m_flNaturalLength ) * m_flSpringConstant
		- flAxialSpeed * m_flSpringDamping;
	const JPH::Vec3 vImpulse = vDirection * ( flForce * flDeltaTime )
		- vRelativeVelocity * ( m_flRelativeDamping * flDeltaTime );

	if ( bEndDynamic )
		pendingImpulses.push_back( { pBodyEnd, vWorldEnd, vImpulse } );
	if ( bStartDynamic )
		pendingImpulses.push_back( { pBodyStart, vWorldStart, -vImpulse } );
}

void JoltPhysicsSpringDamper::OnStep( const JPH::PhysicsStepListenerContext &inContext )
{
	m_PendingImpulses.clear();
	m_PendingImpulses.reserve( m_pSprings.size() * 2 );

	// IVP springs accumulate asynchronous velocity changes so every spring sees
	// the same step-start body velocities. Apply only after all have calculated.
	for ( JoltPhysicsSpring *pSpring : m_pSprings )
		pSpring->CalculateImpulse( inContext.mDeltaTime, m_PendingImpulses );

	for ( const JoltSpringImpulse &pendingImpulse : m_PendingImpulses )
		pendingImpulse.m_pBody->AddImpulse( pendingImpulse.m_vImpulse, pendingImpulse.m_vWorldPoint );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsSpring::OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject )
{
	DetachObjects();

	// Raphael: As soon as one of the physics objects / bodies get destroyed, 
	//          we need to remove the constraint or else it will crash on the next simulation.
	if ( m_pConstraint )
	{
		m_pPhysicsSystem->RemoveConstraint( m_pConstraint );
		m_pConstraint = nullptr;
	}
}

void JoltPhysicsSpringDamper::DetachObjectForTransfer( JoltPhysicsObject *pObject )
{
	const std::vector< JoltPhysicsSpring * > springs = m_pSprings;
	for ( JoltPhysicsSpring *pSpring : springs )
	{
		if ( pSpring && ( pSpring->GetStartObject() == pObject || pSpring->GetEndObject() == pObject ) )
			pSpring->OnJoltPhysicsObjectDestroyed( pObject );
	}
}

//-------------------------------------------------------------------------------------------------

IPhysicsSpring *JoltPhysicsEnvironment::CreateSpring( IPhysicsObject *pObjectStart, IPhysicsObject *pObjectEnd, springparams_t *pParams )
{
	JoltPhysicsObject *pJoltObjectStart = static_cast< JoltPhysicsObject *>( pObjectStart );
	JoltPhysicsObject *pJoltObjectEnd = static_cast< JoltPhysicsObject *>( pObjectEnd );

	return new JoltPhysicsSpring( &m_PhysicsSystem, m_pSpringDamper, pJoltObjectStart, pJoltObjectEnd, pParams );
}

void JoltPhysicsEnvironment::DestroySpring( IPhysicsSpring *pSpring )
{
	ForgetRestoredPointer( pSpring );
	JoltPhysicsSpring *pJoltSpring = static_cast< JoltPhysicsSpring * >( pSpring );
	delete pJoltSpring;
}

//-------------------------------------------------------------------------------------------------

IPhysicsConstraint *JoltPhysicsEnvironment::CreateRagdollConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll )
{
	JoltPhysicsConstraint *pConstraint = new JoltPhysicsConstraint( this, pReferenceObject, pAttachedObject );
	pConstraint->InitialiseRagdoll( pGroup, ragdoll );
	return pConstraint;
}

IPhysicsConstraint *JoltPhysicsEnvironment::CreateHingeConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge )
{
	JoltPhysicsConstraint *pConstraint = new JoltPhysicsConstraint( this, pReferenceObject, pAttachedObject );
	pConstraint->InitialiseHinge( pGroup, hinge );
	return pConstraint;
}

IPhysicsConstraint *JoltPhysicsEnvironment::CreateFixedConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed )
{
	JoltPhysicsConstraint *pConstraint = new JoltPhysicsConstraint( this, pReferenceObject, pAttachedObject );
	pConstraint->InitialiseFixed( pGroup, fixed );
	return pConstraint;
}

IPhysicsConstraint *JoltPhysicsEnvironment::CreateSlidingConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding )
{
	JoltPhysicsConstraint *pConstraint = new JoltPhysicsConstraint( this, pReferenceObject, pAttachedObject );
	pConstraint->InitialiseSliding( pGroup, sliding );
	return pConstraint;
}

IPhysicsConstraint *JoltPhysicsEnvironment::CreateBallsocketConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket )
{
	JoltPhysicsConstraint *pConstraint = new JoltPhysicsConstraint( this, pReferenceObject, pAttachedObject );
	pConstraint->InitialiseBallsocket( pGroup, ballsocket );
	return pConstraint;
}

IPhysicsConstraint *JoltPhysicsEnvironment::CreatePulleyConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley )
{
	JoltPhysicsConstraint *pConstraint = new JoltPhysicsConstraint( this, pReferenceObject, pAttachedObject );
	pConstraint->InitialisePulley( pGroup, pulley );
	return pConstraint;
}

IPhysicsConstraint *JoltPhysicsEnvironment::CreateLengthConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length )
{
	JoltPhysicsConstraint *pConstraint = new JoltPhysicsConstraint( this, pReferenceObject, pAttachedObject );
	pConstraint->InitialiseLength( pGroup, length );
	return pConstraint;
}

void JoltPhysicsEnvironment::DestroyConstraint( IPhysicsConstraint *pConstraint )
{
	if ( !pConstraint )
		return;

	JoltPhysicsConstraint *pJoltConstraint = static_cast< JoltPhysicsConstraint * >( pConstraint );
	// ConstraintBroken callbacks and entity teardown can converge on the same wrapper during one
	// simulation. Objects already have an equivalent delete marker; constraints need one too so the
	// deferred queue never owns the same pointer twice.
	if ( !pJoltConstraint->MarkForDelete() )
		return;
	ForgetRestoredPointer( pJoltConstraint );

	if ( m_bWakeObjectsOnConstraintDeletion )
	{
		IPhysicsObject *pObjectRef = pJoltConstraint->GetReferenceObject();
		if ( pObjectRef )
			pObjectRef->Wake();

		IPhysicsObject *pObjectAtt = pJoltConstraint->GetAttachedObject();
		if ( pObjectAtt )
			pObjectAtt->Wake();
	}

	// Suprisingly, the quick delete thing only affects whether we wake
	// constraints or not.
	// It does not affect whether it goes in the delete queue.
	if ( m_bSimulating )
	{
		pJoltConstraint->Deactivate();
		m_pDeadConstraints.push_back( pJoltConstraint );
	}
	else
	{
		delete pJoltConstraint;
	}
}

//-------------------------------------------------------------------------------------------------

IPhysicsConstraintGroup *JoltPhysicsEnvironment::CreateConstraintGroup( const constraint_groupparams_t &groupParams )
{
	return new JoltPhysicsConstraintGroup( this, groupParams );
}

void JoltPhysicsEnvironment::DestroyConstraintGroup( IPhysicsConstraintGroup *pGroup )
{
	if ( IsInContactCallback() && DeferPhysicsMutation( [this, pGroup]() { DestroyConstraintGroup( pGroup ); } ) ) return;
	ForgetRestoredPointer( pGroup );
	delete static_cast<JoltPhysicsConstraintGroup *>( pGroup );
}

//-------------------------------------------------------------------------------------------------

IPhysicsShadowController *JoltPhysicsEnvironment::CreateShadowController( IPhysicsObject *pObject, bool allowTranslation, bool allowRotation )
{
	JoltPhysicsShadowController *pController = new JoltPhysicsShadowController( static_cast<JoltPhysicsObject *>( pObject ), allowTranslation, allowRotation );
	RegisterPhysicsController( pController );
	return pController;
}

void JoltPhysicsEnvironment::DestroyShadowController( IPhysicsShadowController *pShadowController )
{
	JoltPhysicsShadowController *pController = static_cast< JoltPhysicsShadowController * >( pShadowController );
	ForgetRestoredPointer( pController );
	if ( pController->GetObject() && pController->GetObject()->GetShadowController() == pController )
		pController->GetObject()->SetRestoredShadowController( nullptr );
	UnregisterPhysicsController( pController );
	delete pController;
}

//-------------------------------------------------------------------------------------------------

IPhysicsPlayerController *JoltPhysicsEnvironment::CreatePlayerController( IPhysicsObject *pObject )
{
	JoltPhysicsPlayerController *pController = new JoltPhysicsPlayerController( static_cast<JoltPhysicsObject *>( pObject ) );
	RegisterPhysicsController( pController );
	return pController;
}

void JoltPhysicsEnvironment::DestroyPlayerController( IPhysicsPlayerController *pPlayerController )
{
	JoltPhysicsPlayerController *pController = static_cast< JoltPhysicsPlayerController * >( pPlayerController );
	UnregisterPhysicsController( pController );
	delete pController;
}

//-------------------------------------------------------------------------------------------------

IPhysicsMotionController *JoltPhysicsEnvironment::CreateMotionController( IMotionEvent *pHandler )
{
	JoltPhysicsMotionController *pController = new JoltPhysicsMotionController( this, pHandler );
	RegisterPhysicsController( pController );
	return pController;
}

void JoltPhysicsEnvironment::DestroyMotionController( IPhysicsMotionController *pController )
{
	ForgetRestoredPointer( pController );
	JoltPhysicsMotionController *pJoltController = static_cast< JoltPhysicsMotionController * >( pController );
	UnregisterPhysicsController( pJoltController );
	if ( pJoltController ) pJoltController->Destroy();
}

//-------------------------------------------------------------------------------------------------

bool JoltPhysicsEnvironment::IsVehicleControlledObject( const JoltPhysicsObject *pObject ) const
{
	if ( !pObject ) return false;
	for ( IJoltPhysicsController *pController : m_pPhysicsControllers )
		if ( auto *pVehicle = dynamic_cast<JoltPhysicsVehicleController *>( pController ) )
			if ( pVehicle->OwnsObjectForTransfer( pObject ) ) return true;
	return false;
}

IPhysicsVehicleController *JoltPhysicsEnvironment::CreateVehicleController( IPhysicsObject *pVehicleBodyObject, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace )
{
	JoltPhysicsObject *pJoltCarBodyObject = static_cast< JoltPhysicsObject * >( pVehicleBodyObject );
	if ( !pJoltCarBodyObject || pJoltCarBodyObject->GetJoltEnvironment() != this || IsVehicleControlledObject( pJoltCarBodyObject ) ) return nullptr;

	JoltPhysicsVehicleController *pController = new JoltPhysicsVehicleController( this, &m_PhysicsSystem, pJoltCarBodyObject, params, nVehicleType, pGameTrace );
	RegisterPhysicsController( pController );
	return pController;
}

IPhysicsVehicleController *JoltPhysicsEnvironment::CreateBridgeVehicleController( IPhysicsObject *pVehicleBodyObject, const vehicleparams_t &params, const JoltVehicleBridgeConfig_t &bridgeConfig )
{
	JoltPhysicsObject *pJoltCarBodyObject = static_cast< JoltPhysicsObject * >( pVehicleBodyObject );
	if ( !pJoltCarBodyObject || pJoltCarBodyObject->GetJoltEnvironment() != this || IsVehicleControlledObject( pJoltCarBodyObject ) ) return nullptr;

	JoltPhysicsVehicleController *pController = new JoltPhysicsVehicleController(
		this, &m_PhysicsSystem, pJoltCarBodyObject, params, VEHICLE_TYPE_CAR_WHEELS, nullptr, &bridgeConfig );
	RegisterPhysicsController( pController );
	return pController;
}

void JoltPhysicsEnvironment::DestroyVehicleController( IPhysicsVehicleController *pVehicleController )
{
	// A vehicle owns a native constraint and step listener. Their removal must
	// wait until Update releases its constraints, like other topology mutations.
	if ( DeferPhysicsMutation( [this, pVehicleController]() { DestroyVehicleController( pVehicleController ); } ) ) return;
	JoltPhysicsVehicleController *pJoltController = static_cast<JoltPhysicsVehicleController *>( pVehicleController );
	ForgetRestoredPointer( pJoltController );
	UnregisterPhysicsController( pJoltController );
	delete pJoltController;
}

void JoltPhysicsEnvironment::RegisterPhysicsController( IJoltPhysicsController *pController )
{
	if ( !pController || VectorContains( m_pPhysicsControllers, pController ) )
		return;

	m_pPhysicsControllers.push_back( pController );
	m_bPhysicsControllerOrderDirty = true;
	if ( !m_bSimulating )
		RebuildPhysicsControllerOrder();
}

void JoltPhysicsEnvironment::UnregisterPhysicsController( IJoltPhysicsController *pController )
{
	if ( !pController )
		return;

	Erase( m_pPhysicsControllers, pController );
	// Do not mutate the ordered vector while OnPreSimulate is traversing it. Membership is checked
	// before each callback, and the stale pointer is removed by the next rebuild without dereference.
	m_bPhysicsControllerOrderDirty = true;
	if ( !m_bSimulating )
		RebuildPhysicsControllerOrder();
}

void JoltPhysicsEnvironment::NotifyPhysicsControllerAttached( IJoltPhysicsController *pController )
{
	// IVP_Simulation_Unit::add_controller_of_core performs this sort immediately. In particular,
	// changing a priority after attachment does not re-sort the existing simulation unit. Moving a
	// newly attached controller to the tail also preserves IVP's reverse attachment order for equal
	// priorities rather than accidentally using controller creation order.
	const auto it = std::find( m_pPhysicsControllers.begin(), m_pPhysicsControllers.end(), pController );
	if ( it != m_pPhysicsControllers.end() && std::next( it ) != m_pPhysicsControllers.end() )
	{
		m_pPhysicsControllers.erase( it );
		m_pPhysicsControllers.push_back( pController );
	}

	m_bPhysicsControllerOrderDirty = true;
	if ( !m_bSimulating )
		RebuildPhysicsControllerOrder();
}

void JoltPhysicsEnvironment::RebuildPhysicsControllerOrder()
{
	if ( !m_bPhysicsControllerOrderDirty )
		return;

	m_pOrderedPhysicsControllers = m_pPhysicsControllers;
	std::stable_sort( m_pOrderedPhysicsControllers.begin(), m_pOrderedPhysicsControllers.end(),
		[]( const IJoltPhysicsController *pLeft, const IJoltPhysicsController *pRight )
		{
			return pLeft->GetSimulationPriority() < pRight->GetSimulationPriority();
		} );
	m_bPhysicsControllerOrderDirty = false;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::SetCollisionSolver( IPhysicsCollisionSolver *pSolver )
{
	m_ContactListener.SetGameSolver( pSolver );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::Simulate( float deltaTime )
{
	VJoltCPU::SourceOwner().Bind();
	// Handle pausing the game...
	if ( deltaTime == 0.0f )
		return;

	const bool bCaptureStats = VJoltStats_IsEnabled();
	const auto statsStepBegin = bCaptureStats
		? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
	double flPhysicsUpdateMs = 0.0;

	{
		JPH::PhysicsSettings settings = m_PhysicsSystem.GetPhysicsSettings();
		settings.mBaumgarte = vjolt_baumgarte_factor.GetFloat(); // 0.2 def
		settings.mNumVelocitySteps = vjolt_velocity_steps.GetInt();
		settings.mNumPositionSteps = vjolt_position_steps.GetInt();
		settings.mUseHighMassRatioSolverSteps = vjolt_mass_ratio_iterations.GetBool();
		const float massRatioThreshold = vjolt_mass_ratio_threshold.GetFloat();
		settings.mHighMassRatioThreshold = std::isfinite( massRatioThreshold ) ? massRatioThreshold : 10.0f;
		settings.mHighMassRatioVelocitySteps = vjolt_mass_ratio_velocity_steps.GetInt();
		settings.mHighMassRatioPositionSteps = vjolt_mass_ratio_position_steps.GetInt();
		settings.mDeterministicSimulation = vjolt_deterministic.GetBool();
		// IVP keeps friction contacts around a 2 * collision-tolerance gap. Its velocity equation uses
		// fixed convergence rates: 20 / second while descending toward that gap, 1 / second while
		// gently separating from inside it. keeper_dist = 2.3 * tolerance is only the detection/lifetime
		// range, not a second hard solver target.
		constexpr float flIVPCollisionTolerance = 0.25f - 1.0e-4f;
		settings.mContactSeparationTarget = SourceToJolt::Distance( 2.0f * flIVPCollisionTolerance );
		settings.mContactVelocityBiasFactorOutsideTarget = 20.0f;
		settings.mContactVelocityBiasFactorInsideTarget = 1.0f;
		// Jolt's 1 m/s default suppresses restitution for ordinary Source-scale impacts.
		// IVP's impact solver uses 2 * collision tolerance (0.5 in/s here) as its minimum
		// collision speed, so use the same threshold before applying the material response.
		settings.mMinVelocityForRestitution =
			SourceToJolt::Distance( 2.0f * flIVPCollisionTolerance );
		// IVP's impact solver enforces a minimum separating speed and adds a penetration-dependent
		// rescue term. Keep the 1.2 safety factor in Jolt's velocity constraint rather than trying to
		// reproduce it with split-position correction.
		settings.mContactRescueVelocityFactor = 1.2f;
		settings.mContactRestitutionOnlyOnNewContact = true;
		// IVP retains a managed friction mindist out to friction_dist + 2.5 * tolerance:
		// 2 * tolerance + 2.5 * tolerance = 4.5 * tolerance. Pending manifolds inside this
		// broad range use the inner collision target and do not apply an early support impulse.
		settings.mSpeculativeContactDistance =
			SourceToJolt::Distance( 4.5f * flIVPCollisionTolerance );
		settings.mPenetrationSlop = 0.0f;
		// Match Jolt's intended motion-quality policy: only bodies that move at least 75% of their
		// inner radius in one step need a linear cast. Forcing this to zero cast every moving body,
		// which dominated dense vehicle scenes without improving their trajectories.
		settings.mLinearCastThreshold = 0.75f;
		settings.mLinearCastMaxPenetration = 0.25f;
		m_PhysicsSystem.SetPhysicsSettings( settings );
	}

	// Grab our shared assets from the interface
	JPH::TempAllocator *tempAllocator = JoltPhysicsInterface::GetInstance().GetTempAllocator();
	JPH::JobSystem *jobSystem = JoltPhysicsInterface::GetInstance().GetJobSystem();
	// The offline comparison selects modes before loading the DLL. In game the
	// cvar is sampled once per Simulate, never from a worker or midway through a batch.
	const char *syncOverride = std::getenv( "VJOLT_SYNC_COLLISIONS" );
	const int syncMode = syncOverride ? ( !std::strcmp( syncOverride, "batch" ) ? 1
		: !std::strcmp( syncOverride, "legacy" ) ? 2 : 0 ) : vjolt_collision_callbacks.GetInt();
	const bool synchronousContacts = syncMode != 0 && m_ContactListener.GetGameListener() != nullptr;
	std::optional<VJoltOwnerCallbackJobs> callbackJobs;
	if ( synchronousContacts ) callbackJobs.emplace( *static_cast<JPH::JobSystemWithBarrier *>( jobSystem ) );

	// Clear any dead objects before running the simulation.
	DeleteDeadObjects();

	const float requestedFrictionScale = vjolt_friction_scale.GetFloat();
	const float frictionScale = std::isfinite( requestedFrictionScale )
		? Clamp<float>( requestedFrictionScale, 0.0f, 10.0f ) : 1.0f;
	s_ContactFrictionScale.store( frictionScale, std::memory_order_relaxed );
	if ( frictionScale != m_flContactFrictionScale )
	{
		m_flContactFrictionScale = frictionScale;
		// Sleeping contacts are not solved again until their bodies wake. Keep the
		// normal/manifold cache: Jolt recalculates friction for persisted contacts,
		// and zero friction deactivates the tangential constraints before warm start.
		JPH::BodyIDVector bodies;
		m_PhysicsSystem.GetBodies( bodies );
		JPH::BodyInterface &bodyInterface = m_PhysicsSystem.GetBodyInterfaceNoLock();
		int sleepingCount = 0;
		for ( const JPH::BodyID &id : bodies )
		{
			const JPH::Body *body = m_PhysicsSystem.GetBodyLockInterfaceNoLock().TryGetBody( id );
			if ( body && body->IsDynamic() && body->IsInBroadPhase() && !body->IsActive() )
				bodies[sleepingCount++] = id;
		}
		bodyInterface.ActivateBodies( bodies.data(), sleepingCount );
	}

	// Funnily enough, VPhysics calls this BEFORE
	// doing the simulation...
	m_ContactListener.PostSimulationFrame();

	m_bSimulating = true;

	const bool bRunInitialSettling = !m_bOptimizedBroadPhase && vjolt_initial_simulation.GetBool();
	const float flPhysicsPSITime = m_flStepTime;
	if ( !bRunInitialSettling )
	{
		m_flPhysicsAccumulator += static_cast< double >( deltaTime );
		// Do not allow a long main-thread stall to create an unbounded catch-up loop. The excess
		// elapsed time is deliberately dropped at this boundary; the next call starts with at most
		// the configured maximum number of fixed PSI steps owed.
		const double flMaximumAccumulator =
			static_cast< double >( flPhysicsPSITime ) * kMaximumPhysicsPSIStepsPerCall;
		if ( m_flPhysicsAccumulator > flMaximumAccumulator )
			m_flPhysicsAccumulator = flMaximumAccumulator;
	}

	const int nPhysicsPSISteps = bRunInitialSettling
		? 0
		: ConsumePhysicsPSISteps( m_flPhysicsAccumulator, flPhysicsPSITime, vjolt_substeps.GetInt() );

	// One complete Source/IVP PSI. This boundary must be repeated for every fixed step: Jolt's
	// collision substeps do not call Volt motion controllers, so forwarding a larger substep count
	// to one Update would leave simfphys/native vehicles at the outer server frequency.
	auto preparePhysicsPSI = [ & ]( float flPSITime )
	{
		// Game-side pushes queued before Simulate are consumed at the beginning of the first PSI.
		CommitPendingVelocityChanges();

		// IVP stores controllers in ascending priority order and executes that list backwards.
		// Rebuild before every PSI because a callback may attach or detach a controller in the
		// preceding pass. Stable ordering is significant for equal-priority controllers.
		RebuildPhysicsControllerOrder();
		for ( auto it = m_pOrderedPhysicsControllers.rbegin();
			it != m_pOrderedPhysicsControllers.rend(); ++it )
		{
			// A clean ordered list contains only live controllers. Recheck the dirty flag after
			// every callback: removal can leave stale pointers in this pass's ordered snapshot.
			if ( !m_bPhysicsControllerOrderDirty || VectorContains( m_pPhysicsControllers, *it ) )
				( *it )->OnPreSimulate( flPSITime );
		}

		// Motion-controller adapters write through public IPhysicsObject velocity/force methods.
		// Publish their batch before Jolt step listeners and the contact solver run.
		CommitPendingVelocityChanges();
		VJoltSoftBodyBeforeSimulation( this );
		VJoltConstraintsBeforeSimulation( this );
		VJoltPathBeforeSimulation( this );
		VJoltGravityBeforeSimulation( this );
		m_ConstraintWakePropagator.Propagate( m_PhysicsSystem );
	};

	auto updatePhysics = [ & ]( float flUpdateTime, int nSubSteps )
	{
		// Keep the previous completed manifold frame visible while Source-side controllers run.
		// Contact callbacks fill a separate frame during Update; only publish it after all Jolt
		// worker jobs have completed.
		m_ContactListener.BeginFrictionContactFrame( flUpdateTime );
		const auto updateBegin = bCaptureStats
			? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
		m_pContactCallbackJobs = callbackJobs ? &*callbackJobs : nullptr;
		m_ContactListener.SetContactResponseBatches( synchronousContacts && syncMode == 1
			&& m_ContactListener.GetGameListener() != nullptr );
		const JPH::EPhysicsUpdateError updateErrors =
			m_PhysicsSystem.Update( flUpdateTime, nSubSteps, tempAllocator,
				m_pContactCallbackJobs ? m_pContactCallbackJobs : jobSystem );
		m_pContactCallbackJobs = nullptr;
		m_ContactListener.SetContactResponseBatches( false );
		VJoltSoftBodyAfterSimulation( this, flUpdateTime );
		// Solver rows and island arrays are no longer live. Apply geometry/mass/
		// topology changes requested by callbacks before publishing the new frame.
		auto mutations = std::move( m_DeferredPhysicsMutations );
		m_DeferredPhysicsMutations.clear();
		for ( const auto &mutation : mutations ) mutation();
		// IVP consults the game about stuck pairs inside its PSI. Ask after all workers have
		// finished, before the next PSI's contact callbacks consume the decisions.
		m_ContactListener.DispatchPenetrationQueries( flUpdateTime );
		if ( bCaptureStats )
		{
			flPhysicsUpdateMs += std::chrono::duration<double, std::milli>(
				std::chrono::steady_clock::now() - updateBegin ).count();
		}
		const uint32 nUpdateErrorMask = static_cast<uint32>( updateErrors );
		VJoltStats_RecordPhysicsUpdateResult( this, nUpdateErrorMask );

		const uint64 nPreviousConsecutiveErrors = m_PhysicsUpdateErrors.GetConsecutiveErrors();
		const bool bReportUpdateError = m_PhysicsUpdateErrors.Record( nUpdateErrorMask );
		if ( nUpdateErrorMask == 0 && nPreviousConsecutiveErrors > 0 )
		{
			Log_Msg( LOG_VJolt,
				"PhysicsSystem::Update recovered after %llu consecutive contact-capacity errors (env=%p).\n",
				static_cast<unsigned long long>( nPreviousConsecutiveErrors ), this );
		}
		else if ( bReportUpdateError )
		{
			const uint32 nManifoldCacheFull = static_cast<uint32>( JPH::EPhysicsUpdateError::ManifoldCacheFull );
			const uint32 nBodyPairCacheFull = static_cast<uint32>( JPH::EPhysicsUpdateError::BodyPairCacheFull );
			const uint32 nContactConstraintsFull = static_cast<uint32>( JPH::EPhysicsUpdateError::ContactConstraintsFull );
			Log_Warning( LOG_VJolt,
				"PhysicsSystem::Update dropped contacts: env=%p mask=0x%X manifold_cache=%d body_pair_cache=%d contact_constraints=%d consecutive=%llu total=%llu bodies=%u active=%u limits(body_pairs=%u contacts=%u). The partial frame is retained; Update is non-transactional and is not replayed.\n",
				this, nUpdateErrorMask,
				( nUpdateErrorMask & nManifoldCacheFull ) != 0,
				( nUpdateErrorMask & nBodyPairCacheFull ) != 0,
				( nUpdateErrorMask & nContactConstraintsFull ) != 0,
				static_cast<unsigned long long>( m_PhysicsUpdateErrors.GetConsecutiveErrors() ),
				static_cast<unsigned long long>( m_PhysicsUpdateErrors.GetTotalErrors() ),
				m_PhysicsSystem.GetNumBodies(),
				m_PhysicsSystem.GetNumActiveBodies( JPH::EBodyType::RigidBody ),
				kMaxBodyPairs, kMaxContactConstraints );
		}
		m_ContactListener.PublishFrictionContactFrame();
	};

	bool bRanInitialSettling = false;
	// If we haven't already, optimize the broadphase, currently this can only happen once per-environment
	if ( !m_bOptimizedBroadPhase )
	{
		m_PhysicsSystem.OptimizeBroadPhase();
		m_bOptimizedBroadPhase = true;

		if ( vjolt_initial_simulation.GetBool() )
		{
			// Preserve the historical initial-settling path. It is a startup operation, not elapsed
			// game time, so retain its existing four internal Jolt steps per iteration and do not
			// reinterpret those iterations as normal fixed PSI/controller passes.
			preparePhysicsPSI( deltaTime );
			static constexpr float InitialIterationTimescale = 1.0f / 60.0f;
			static constexpr int MaxInitialIterations = 1024;
			static constexpr int InitialSubSteps = 4;

			int nIterCount = 0;
			while ( m_PhysicsSystem.GetNumActiveBodies( JPH::EBodyType::RigidBody ) && nIterCount < MaxInitialIterations )
			{
				updatePhysics( InitialIterationTimescale, InitialSubSteps );
				nIterCount++;
			}
			bRanInitialSettling = true;
		}
	}

	if ( !bRanInitialSettling )
	{
		// Every normal external call consumes an integer number of fixed 1/64 PSI passes. At 32
		// tick this is two complete passes; at 64 tick it is one. Contact/controller state is
		// published between passes, while game callbacks remain deferred until below.
		for ( int nPSI = 0; nPSI < nPhysicsPSISteps; ++nPSI )
		{
			preparePhysicsPSI( flPhysicsPSITime );
			updatePhysics( flPhysicsPSITime, 1 );
		}
	}

	// IVP exposes one clock step per external Simulate call. Jolt substeps and the optional
	// first-frame settling loop are solver details and must not advance the public clock again.
	m_SimulationClock.Advance( deltaTime );

	m_ContactListener.FlushCallbacks();
	m_ActivationListener.FlushCallbacks();

	// Source installs its listener only on hk_Breakable_Constraint wrappers. Match that sparse
	// path: ordinary welds/ropes never need an adapter-side break check. Deletion during the
	// callback is deferred and marked, so the snapshot remains safe without an O(N) membership
	// search for every entry.
	const std::vector< JoltPhysicsConstraint * > constraints = m_pBreakableConstraints;
	for ( JoltPhysicsConstraint *pConstraint : constraints )
	{
		if ( pConstraint && !pConstraint->IsMarkedForDelete() )
			pConstraint->CheckBroken();
	}

	const JPH::BodyID *pActiveBodies = m_PhysicsSystem.GetActiveBodiesUnsafe( JPH::EBodyType::RigidBody );
	uint32_t uActiveBodies = m_PhysicsSystem.GetNumActiveBodies( JPH::EBodyType::RigidBody );
	for ( uint32_t i = 0; i < uActiveBodies; i++ )
	{
		JPH::Body* pBody = m_PhysicsSystem.GetBodyLockInterfaceNoLock().TryGetBody( pActiveBodies[i] );
		if ( auto *object = reinterpret_cast< JoltPhysicsObject* >( pBody->GetUserData() ) )
			object->PostSimulation( deltaTime );
	}

	// Post-simulation is a Jolt adapter phase with no IVP priority equivalent. Preserve the existing
	// registration-order behavior.
	for ( IJoltPhysicsController *pController : m_pPhysicsControllers )
		pController->OnPostSimulate( deltaTime );
	m_bSimulating = false;
	JoltPhysicsInterface::GetInstance().ReportTempAllocatorPressure();

	// If the delete queue is disabled, we only added to it during the simulation
	// ie. callbacks etc. So flush that now.
	if ( !m_bEnableDeleteQueue )
		DeleteDeadObjects();

	if ( bCaptureStats )
	{
		const double flTotalStepMs = std::chrono::duration<double, std::milli>(
			std::chrono::steady_clock::now() - statsStepBegin ).count();
		VJoltStats_RecordEnvironmentStep( this, flTotalStepMs, flPhysicsUpdateMs,
			m_PhysicsSystem.GetNumBodies(),
			m_PhysicsSystem.GetNumActiveBodies( JPH::EBodyType::RigidBody ),
			static_cast<uint32_t>( m_pConstraints.size() ),
			m_pSpringDamper->GetSpringCount(),
			static_cast<uint32_t>( m_pPhysicsControllers.size() ) );
	}
}

bool JoltPhysicsEnvironment::HasMotionControllerForObject( JoltPhysicsObject *object ) const
{
	for ( auto *controller : m_pPhysicsControllers )
		if ( auto *motion = dynamic_cast<JoltPhysicsMotionController *>( controller ) )
			if ( motion->ContainsObject( object ) ) return true;
	return false;
}

bool JoltPhysicsEnvironment::IsInSimulation() const
{
	return m_bSimulating;
}

bool JoltPhysicsEnvironment::IsInContactCallback() const
{
	return m_pContactCallbackJobs && m_pContactCallbackJobs->IsInCallback();
}

bool JoltPhysicsEnvironment::RunContactCallback( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2,
	JPH::RVec3Arg centerOfMass1, JPH::RVec3Arg centerOfMass2,
	const std::function<bool()> &callback )
{
	return RunContactBatch( [&]()
	{
		m_pCollisionEndpoint1 = pObject1;
		m_pCollisionEndpoint2 = pObject2;
		m_vCollisionCenterOfMass1 = centerOfMass1;
		m_vCollisionCenterOfMass2 = centerOfMass2;
		const bool result = callback();
		m_pCollisionEndpoint1 = m_pCollisionEndpoint2 = nullptr;
		return result;
	} );
}

bool JoltPhysicsEnvironment::RunContactBatch( const std::function<bool()> &batch )
{
	VJoltAssert( m_pContactCallbackJobs );
	return m_pContactCallbackJobs->Call( [&]()
	{
#ifdef JPH_ENABLE_ASSERTS
		// The owner may itself be paused inside a solver job (notably with zero
		// workers). All other jobs have yielded, but BodyAccess::Grant forbids
		// nesting and always restores unrestricted access. Preserve this job's
		// actual rights across the callback instead of losing its read-only scope.
		struct CallbackBodyAccess
		{
			JPH::BodyAccess::EAccess velocity = JPH::BodyAccess::sVelocityAccess();
			JPH::BodyAccess::EAccess position = JPH::BodyAccess::sPositionAccess();
			CallbackBodyAccess()
			{
				JPH::BodyAccess::sVelocityAccess() = JPH::BodyAccess::EAccess::ReadWrite;
				JPH::BodyAccess::sPositionAccess() = JPH::BodyAccess::EAccess::ReadWrite;
			}
			~CallbackBodyAccess()
			{
				JPH::BodyAccess::sVelocityAccess() = velocity;
				JPH::BodyAccess::sPositionAccess() = position;
			}
		} access;
#endif
		return batch();
	} );
}

JPH::RVec3 JoltPhysicsEnvironment::GetObjectResponseCenterOfMass( const JoltPhysicsObject *pObject ) const
{
	// CCD responds at TOI before storing the final body translations. Point
	// velocities and offset impulses in game callbacks must use that same lever,
	// without changing the body's transform or substituting its velocity.
	if ( IsInContactCallback() )
	{
		if ( pObject == m_pCollisionEndpoint1 ) return m_vCollisionCenterOfMass1;
		if ( pObject == m_pCollisionEndpoint2 ) return m_vCollisionCenterOfMass2;
	}
	return pObject->GetBody()->GetCenterOfMassPosition();
}

bool JoltPhysicsEnvironment::DeferPhysicsMutation( const std::function<void()> &mutation )
{
	if ( !IsInContactCallback() ) return false;
	m_DeferredPhysicsMutations.push_back( mutation );
	return true;
}

void JoltPhysicsEnvironment::NotifyCallbackBodyPropertiesChanged( JoltPhysicsObject *object )
{
	// New callback-created bodies have no solver rows yet. Their body properties
	// are already authoritative even though publishing the ID waits for Update.
	if ( IsInContactCallback() && !object->GetBodyID().IsInvalid()
		&& !VectorContains( m_CallbackChangedBodies, object ) )
		m_CallbackChangedBodies.push_back( object );
}

void JoltPhysicsEnvironment::RefreshCallbackBodyProperties()
{
	for ( auto *object : m_CallbackChangedBodies )
		m_PhysicsSystem.RefreshContactCallbackBodyProperties( *object->GetBody(), m_flStepTime );
	m_CallbackChangedBodies.clear();
}

bool JoltPhysicsEnvironment::DeferObjectMutation( JoltPhysicsObject *pObject,
	const std::function<void()> &mutation, bool velocityOnly )
{
	if ( !IsInContactCallback()
		|| ( velocityOnly && ( pObject == m_pCollisionEndpoint1 || pObject == m_pCollisionEndpoint2 ) ) )
		return false;
	m_DeferredPhysicsMutations.push_back( [pObject, mutation]()
	{
		if ( !( pObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) ) mutation();
	} );
	return true;
}

const JPH::BodyLockInterface &JoltPhysicsEnvironment::GetBodyLockInterfaceForRead() const
{
	return IsInContactCallback()
		? static_cast<const JPH::BodyLockInterface &>( m_PhysicsSystem.GetBodyLockInterfaceNoLock() )
		: m_PhysicsSystem.GetBodyLockInterface();
}

void JoltPhysicsEnvironment::GetSourceBodiesForRead( JPH::BodyIDVector &bodies ) const
{
	if ( IsInContactCallback() ) m_PhysicsSystem.GetBodiesNoLock( bodies );
	else m_PhysicsSystem.GetBodies( bodies );
	// Bridge soft bodies share the simulation, but have no IPhysicsObject wrapper.
	const auto &bodyInterface = m_PhysicsSystem.GetBodyLockInterfaceNoLock();
	bodies.erase( std::remove_if( bodies.begin(), bodies.end(), [&bodyInterface]( const JPH::BodyID &id )
	{
		const auto *body = bodyInterface.TryGetBody( id );
		return !body || !body->IsRigidBody() || body->GetUserData() == 0;
	} ), bodies.end() );
}

JPH::Constraints JoltPhysicsEnvironment::GetConstraintsForRead() const
{
	return IsInContactCallback() ? m_PhysicsSystem.GetConstraintsNoLock() : m_PhysicsSystem.GetConstraints();
}

//-------------------------------------------------------------------------------------------------

float JoltPhysicsEnvironment::GetSimulationTimestep() const
{
	return m_flStepTime;
}

void JoltPhysicsEnvironment::SetSimulationTimestep( float timestep )
{
	// Source sets this to interval_per_tick. Ignoring it created repeated poses
	// on 66.7 Hz servers even though the engine reported a 15 ms simulation step.
	// Snapshot the interval for an entire Simulate call; callback changes must not
	// alter controller/solver timing midway through that call.
	if ( m_bSimulating || !std::isfinite( timestep ) || timestep < 1.0e-5f || timestep > 1.0f )
		return;
	m_flStepTime = timestep;
}

//-------------------------------------------------------------------------------------------------

float JoltPhysicsEnvironment::GetSimulationTime() const
{
	return m_SimulationClock.GetTime();
}

void JoltPhysicsEnvironment::ResetSimulationClock()
{
	m_SimulationClock.Reset();
	m_ContactListener.ResetCollisionClock();
	m_flPhysicsAccumulator = 0.0;
}

float JoltPhysicsEnvironment::GetNextFrameTime() const
{
	return m_SimulationClock.GetNextFrameTime( m_flStepTime );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::SetCollisionEventHandler( IPhysicsCollisionEvent *pCollisionEvents )
{
	m_ContactListener.SetGameListener( pCollisionEvents );
}

void JoltPhysicsEnvironment::SetObjectEventHandler( IPhysicsObjectEvent *pObjectEvents )
{
	m_ActivationListener.SetGameListener( pObjectEvents );
}

void JoltPhysicsEnvironment::SetConstraintEventHandler( IPhysicsConstraintEvent *pConstraintEvents )
{
	m_pConstraintListener = pConstraintEvents;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::SetQuickDelete( bool bQuick )
{
	// Josh:
	// What a weird function, all this does is determine whether
	// to wake objects when the constraint gets deleted or not.
	m_bWakeObjectsOnConstraintDeletion = !bQuick;
}

//-------------------------------------------------------------------------------------------------

// GetActiveObjectCount and GetActiveObjects are always called in pairs
// and outside of the simulation.
//
// If there is a weird case where this is different (GMod lua on collision callbacks maybe?).
// Uncomment the locking code below.

int JoltPhysicsEnvironment::GetActiveObjectCount() const
{
	if ( !m_bActiveObjectCountFirst )
	{
		m_PhysicsSystem.GetActiveBodies( JPH::EBodyType::RigidBody, m_CachedActiveBodies );
		const auto &bodyInterface = m_PhysicsSystem.GetBodyLockInterfaceNoLock();
		m_CachedActiveBodies.erase( std::remove_if( m_CachedActiveBodies.begin(), m_CachedActiveBodies.end(),
			[&bodyInterface]( const JPH::BodyID &id ) {
				const auto *body = bodyInterface.TryGetBody( id );
				return !body || !body->GetUserData();
			} ), m_CachedActiveBodies.end() );
		// Append any dirty static bodies we need the game side transforms
		// to be updated for.
		m_CachedActiveBodies.insert( m_CachedActiveBodies.end(), m_DirtyStaticBodies.begin(), m_DirtyStaticBodies.end() );
	}
	else
	{
		// If this is the first call, then some objects may have become
		// asleep from the initial simulation have their visuals not match where they are.
		GetSourceBodiesForRead( m_CachedActiveBodies );
		m_bActiveObjectCountFirst = false;
	}

	m_DirtyStaticBodies.clear();

	return int( m_CachedActiveBodies.size() );
}

void JoltPhysicsEnvironment::GetActiveObjects( IPhysicsObject **pOutputObjectList ) const
{
	const int nCount = int ( m_CachedActiveBodies.size() );
	for ( int i = 0; i < nCount; i++ )
	{
		JPH::Body *pBody = m_PhysicsSystem.GetBodyLockInterfaceNoLock().TryGetBody( m_CachedActiveBodies[ i ] );
		pOutputObjectList[ i ] = reinterpret_cast<IPhysicsObject *>( pBody->GetUserData() );
	}
}

const IPhysicsObject **JoltPhysicsEnvironment::GetObjectList( int *pOutputObjectCount ) const
{
	GetSourceBodiesForRead( m_CachedBodies );

	const int nCount = int ( m_CachedBodies.size() );

	if ( pOutputObjectCount )
		*pOutputObjectCount = nCount;

	m_CachedObjects.resize( nCount );
	for ( int i = 0; i < nCount; i++ )
	{
		JPH::Body *pBody = m_PhysicsSystem.GetBodyLockInterfaceNoLock().TryGetBody( m_CachedBodies[ i ] );
		m_CachedObjects[ i ] = reinterpret_cast< IPhysicsObject * >( pBody->GetUserData() );
	}

	return m_CachedObjects.data();
}

bool JoltPhysicsEnvironment::TransferObject( IPhysicsObject *pObject, IPhysicsEnvironment *pDestinationEnvironment )
{
	if ( !pObject || !pDestinationEnvironment )
		return false;

	JoltPhysicsObject *pJoltObject = static_cast< JoltPhysicsObject * >( pObject );
	JoltPhysicsEnvironment *pJoltEnv = static_cast< JoltPhysicsEnvironment * >( pDestinationEnvironment );
	if ( pJoltEnv == this )
		return true;

	if ( pJoltObject->GetJoltEnvironment() != this || m_bSimulating || pJoltEnv->m_bSimulating ||
		( pJoltObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) )
	{
		return false;
	}

	// Source destroys and recreates the native IVP object in the destination environment while
	// preserving the IPhysicsObject wrapper. A Jolt BodyID belongs to one BodyManager only, so a
	// cross-system transfer must create a new body instead of adding the source ID to another system.
	// Fluid volumes and vehicles own system-local query/constraint state that Source does not move.
	if ( pJoltObject->IsFluid() )
		return false;

	std::vector< JoltPhysicsPlayerController * > playerControllers;
	std::vector< JoltPhysicsPlayerController * > playerControllersToMove;
	std::vector< JoltPhysicsMotionController * > motionControllers;
	std::vector< JoltPhysicsFluidController * > fluidControllers;
	std::vector< IJoltPhysicsController * > controllersToMove;

	// Rebuild this snapshot after game callbacks, which can destroy controllers
	// or change which object a player/shadow controller owns.
	auto collectControllers = [&]()
	{
		playerControllers.clear();
		playerControllersToMove.clear();
		motionControllers.clear();
		fluidControllers.clear();
		controllersToMove.clear();
		if ( pJoltObject->IsFluid() )
			return false;
		if ( IPhysicsShadowController *pShadow = pJoltObject->GetShadowController() )
			controllersToMove.push_back( static_cast< JoltPhysicsShadowController * >( pShadow ) );

		for ( IJoltPhysicsController *pController : m_pPhysicsControllers )
		{
			if ( JoltPhysicsVehicleController *pVehicle = dynamic_cast< JoltPhysicsVehicleController * >( pController ) )
			{
				if ( pVehicle->OwnsObjectForTransfer( pJoltObject ) )
					return false;
			}
			else if ( JoltPhysicsPlayerController *pPlayer = dynamic_cast< JoltPhysicsPlayerController * >( pController ) )
			{
				playerControllers.push_back( pPlayer );
				if ( pPlayer->GetObject() == pJoltObject )
				{
					playerControllersToMove.push_back( pPlayer );
					if ( !VectorContains( controllersToMove, static_cast< IJoltPhysicsController * >( pPlayer ) ) )
						controllersToMove.push_back( pPlayer );
				}
			}
			else if ( JoltPhysicsMotionController *pMotion = dynamic_cast< JoltPhysicsMotionController * >( pController ) )
			{
				motionControllers.push_back( pMotion );
			}
			else if ( JoltPhysicsFluidController *pFluid = dynamic_cast< JoltPhysicsFluidController * >( pController ) )
			{
				fluidControllers.push_back( pFluid );
			}
		}
		return true;
	};
	if ( !collectControllers() )
		return false;

	JPH::Body *pOldBody = pJoltObject->GetBody();
	if ( !pOldBody )
		return false;
	const JPH::BodyID oldBodyID = pOldBody->GetID();

	JPH::BodyCreationSettings bodyCreationSettings = pOldBody->GetBodyCreationSettings();
	bodyCreationSettings.mMaxLinearVelocity = SourceToJolt::Distance( pJoltEnv->MaxVelocity() );
	bodyCreationSettings.mMaxAngularVelocity = DEG2RAD( pJoltEnv->MaxAngularVelocity() );

	JPH::BodyInterface &destinationBodyInterface = pJoltEnv->m_PhysicsSystem.GetBodyInterfaceNoLock();
	JPH::Body *pNewBody = destinationBodyInterface.CreateBody( bodyCreationSettings );
	if ( !pNewBody )
		return false;

	VJoltConstraintsReleaseObject( pJoltObject );
	VJoltPathReleaseObject( pJoltObject );
	// ConstraintBroken is a game callback and may destroy constraints re-entrantly. Restart the
	// lookup after every notification instead of retaining raw pointers across that callback.
	for ( ;; )
	{
		JoltPhysicsConstraint *pFoundConstraint = nullptr;
		for ( JoltPhysicsConstraint *pConstraint : m_pConstraints )
		{
			if ( pConstraint && ( pConstraint->GetReferenceObject() == pJoltObject ||
				pConstraint->GetAttachedObject() == pJoltObject ) )
			{
				pFoundConstraint = pConstraint;
				break;
			}
		}

		if ( !pFoundConstraint )
			break;
		pFoundConstraint->OnJoltPhysicsObjectDestroyed( pJoltObject );

		// The callback can delete or transfer the wrapper. Resolve the original
		// sequenced BodyID before touching the wrapper again; pointer identity alone
		// cannot distinguish a new object allocated at the same address.
		const JPH::Body *pSourceBody = m_PhysicsSystem.GetBodyLockInterfaceNoLock().TryGetBody( oldBodyID );
		if ( !pSourceBody || pSourceBody->GetUserData() != reinterpret_cast<uint64>( pJoltObject ) ||
			( pJoltObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) )
		{
			destinationBodyInterface.DestroyBody( pNewBody->GetID() );
			return false;
		}
	}
	if ( !collectControllers() )
	{
		destinationBodyInterface.DestroyBody( pNewBody->GetID() );
		return false;
	}

	// No game callbacks remain. Commit using current controller ownership only.
	for ( JoltPhysicsPlayerController *pPlayer : playerControllers )
	{
		if ( VectorContains( playerControllersToMove, pPlayer ) )
			pPlayer->SetObject( nullptr );
		else
			pPlayer->OnJoltPhysicsObjectDestroyed( pJoltObject );
	}
	for ( JoltPhysicsMotionController *pMotion : motionControllers )
		pMotion->DetachObject( pJoltObject );
	for ( JoltPhysicsFluidController *pFluid : fluidControllers )
		pFluid->RemoveObjectForTransfer( pJoltObject );
	m_pSpringDamper->DetachObjectForTransfer( pJoltObject );

	const bool bDragEnabled = pJoltObject->IsDragEnabled();
	if ( bDragEnabled )
		m_pDragController->UnregisterObject( pJoltObject );

	m_ContactListener.InvalidateFrictionContacts( oldBodyID );
	m_ContactListener.InvalidateShouldCollideCache( pJoltObject );
	RemoveDirtyStaticBody( oldBodyID );

	JPH::BodyInterface &sourceBodyInterface = m_PhysicsSystem.GetBodyInterfaceNoLock();
	sourceBodyInterface.RemoveBody( oldBodyID );
	sourceBodyInterface.DestroyBody( oldBodyID );

	pJoltObject->ReplaceBodyForTransfer( pNewBody, pJoltEnv );
	destinationBodyInterface.AddBody( pNewBody->GetID(),
		pNewBody->IsStatic() ? JPH::EActivation::DontActivate : JPH::EActivation::Activate );
	if ( IPhysicsShadowController *pShadow = pJoltObject->GetShadowController() )
	{
		static_cast< JoltPhysicsShadowController * >( pShadow )->SetEnvironmentVelocityLimits(
			bodyCreationSettings.mMaxLinearVelocity, bodyCreationSettings.mMaxAngularVelocity );
	}

	if ( bDragEnabled )
		pJoltEnv->m_pDragController->RegisterObject( pJoltObject );

	for ( IJoltPhysicsController *pController : controllersToMove )
	{
		UnregisterPhysicsController( pController );
		pJoltEnv->RegisterPhysicsController( pController );
	}

	for ( JoltPhysicsPlayerController *pPlayer : playerControllersToMove )
		pPlayer->SetObject( pJoltObject );

	m_CachedBodies.clear();
	m_CachedObjects.clear();
	m_CachedActiveBodies.clear();
	pJoltEnv->m_CachedBodies.clear();
	pJoltEnv->m_CachedObjects.clear();
	pJoltEnv->m_CachedActiveBodies.clear();

	return true;
}

//-------------------------------------------------------------------------------------------------

class JoltStateRecorderSave final : public JPH::StateRecorder
{
public:
	JoltStateRecorderSave( ISave *pSave )
		: m_pSave( pSave )
	{
	}

	JoltStateRecorderSave( IRestore *pRestore )
		: m_pRestore( pRestore )
	{
	}

	void WriteBytes( const void* inData, size_t inNumBytes ) override
	{
		m_pSave->WriteData( reinterpret_cast< const char * >( inData ), int( inNumBytes ) );
	}

	void ReadBytes( void* outData, size_t inNumBytes ) override
	{
		if ( inNumBytes > size_t( INT_MAX )
			|| m_pRestore->ReadData( reinterpret_cast< char * >( outData ), int( inNumBytes ), 0 ) != int( inNumBytes ) )
		{
			memset( outData, 0, inNumBytes );
			m_bFailed = true;
		}
	}

	bool IsEOF()	const override	{ return false; }
	bool IsFailed()	const override	{ return m_bFailed; }

private:
	bool m_bFailed = false;
	union
	{
		ISave		*m_pSave;
		IRestore	*m_pRestore;
	};
};

#if defined( GAME_GMOD_64X )
static bool WritePhysicsSaveRestoreId( ISave *pSave, VJoltSaveRestoreIdTable::Id id )
{
	if ( !pSave || id == VJoltSaveRestoreIdTable::INVALID_ID )
		return false;

	const int serializedId = static_cast< int >( id );
	pSave->WriteInt( &serializedId, 1 );
	return true;
}

static bool ReadPhysicsSaveRestoreId( IRestore *pRestore, VJoltSaveRestoreIdTable::Id &id )
{
	id = VJoltSaveRestoreIdTable::INVALID_ID;
	if ( !pRestore )
		return false;

	int serializedId = -1;
	if ( pRestore->ReadInt( &serializedId, 1, 0 ) <= 0 || serializedId < 0 )
		return false;

	id = static_cast< VJoltSaveRestoreIdTable::Id >( serializedId );
	return true;
}
#endif

// This header belongs to the Jolt payload, after the unchanged engine reference ID. Keeping
// a length and version lets us reject incomplete/newer records before constructing live objects.
static constexpr int kPhysicsSaveMagic = 0x4a535631;
static constexpr int kPhysicsSaveVersion = 1;
static constexpr int kMaxPhysicsSaveBytes = 64 * 1024 * 1024;

bool JoltPhysicsEnvironment::WritePhysicsReference( JPH::StateRecorder &recorder, const void *pObject )
{
	if ( !pObject )
		return false;
#if defined( GAME_GMOD_64X )
	const auto id = m_GModSaveRestoreIds.GetOrCreateSaveId( pObject );
	if ( id == VJoltSaveRestoreIdTable::INVALID_ID )
		return false;
	recorder.Write( id );
#else
	recorder.Write( reinterpret_cast< uintp >( pObject ) );
#endif
	return !VJoltStateRecorderFailed( recorder );
}

template <typename T>
bool JoltPhysicsEnvironment::ReadPhysicsReference( JPH::StateRecorder &recorder,
	T *&pObject, PhysInterfaceId_t type )
{
	pObject = nullptr;
#if defined( GAME_GMOD_64X )
	VJoltSaveRestoreIdTable::Id id = VJoltSaveRestoreIdTable::INVALID_ID;
	recorder.Read( id );
	return !VJoltStateRecorderFailed( recorder ) && m_GModSaveRestoreIds.LookupRestoreId( id, pObject, type ) && pObject;
#else
	uintp oldPointer = 0;
	recorder.Read( oldPointer );
	if ( !VJoltStateRecorderFailed( recorder ) )
		pObject = LookupPhysicsSaveRestorePointer< T >( oldPointer );
	return pObject != nullptr;
#endif
}

bool JoltPhysicsEnvironment::Save( const physsaveparams_t &params )
{
	if ( !params.pSave || !params.pObject || params.type <= PIID_UNKNOWN || params.type >= PIID_NUM_TYPES )
		return false;

#if defined( GAME_GMOD_64X )
	const VJoltSaveRestoreIdTable::Id objectId = m_GModSaveRestoreIds.GetOrCreateSaveId( params.pObject );
	if ( objectId == VJoltSaveRestoreIdTable::INVALID_ID )
	{
		Log_Warning( LOG_VJolt, "Physics Save called without a matching PreSave, or the ID table is full.\n" );
		return false;
	}
#endif

	VJoltStateRecorder recorder;
	if ( !SavePhysicsState( params, recorder ) || VJoltStateRecorderFailed( recorder )
		|| recorder.TellPut() <= 0 || recorder.TellPut() > kMaxPhysicsSaveBytes )
		return false;

#if defined( GAME_GMOD_64X )
	if ( !WritePhysicsSaveRestoreId( params.pSave, objectId ) )
		return false;
#else
	const uintp oldPointer = reinterpret_cast< uintp >( params.pObject );
	params.pSave->WriteData( reinterpret_cast< const char * >( &oldPointer ), sizeof( oldPointer ) );
#endif
	const int header[] = { kPhysicsSaveMagic, kPhysicsSaveVersion, recorder.TellPut() };
	params.pSave->WriteInt( header, 3 );
	params.pSave->WriteData( static_cast< const char * >( recorder.Base() ), recorder.TellPut() );
	return true;
}

bool JoltPhysicsEnvironment::SavePhysicsState( const physsaveparams_t &params, JPH::StateRecorder &recorder )
{
	switch ( params.type )
	{
		default:
		case PIID_UNKNOWN:
			Log_Warning( LOG_VJolt, "Saving PIID_UNKNOWN is unsupported right now.\n" );
			return false;
		case PIID_IPHYSICSOBJECT:
		{
			JoltPhysicsObject *pObject = reinterpret_cast< JoltPhysicsObject * >( params.pObject );
			if ( !pObject )
				return false;

			if ( pObject->GetJoltEnvironment() != this
				|| ( pObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) )
				return false;
			recorder.Write( pObject->IsSphereObject() );
			if ( pObject->IsSphereObject() )
				recorder.Write( pObject->GetSphereRadius() );
			recorder.Write( pObject->GetBody()->IsActive() );
			recorder.Write( pObject->GetBody()->GetShape()->GetCenterOfMass() );
			JPH::BodyCreationSettings bodyCreationSettings = pObject->GetBody()->GetBodyCreationSettings();
			bodyCreationSettings.SaveBinaryState( recorder );
			pObject->SaveObjectState( recorder, true );
			auto *pShadow = static_cast< JoltPhysicsShadowController * >( pObject->GetShadowController() );
			recorder.Write( pShadow != nullptr );
			if ( pShadow )
				pShadow->SaveState( recorder );
			return true;
		}
		case PIID_IPHYSICSFLUIDCONTROLLER:
			// This just returns false in regular VPhysics.
			return false;
		case PIID_IPHYSICSSPRING:
		{
			auto *pSpring = static_cast< JoltPhysicsSpring * >( params.pObject );
			if ( !WritePhysicsReference( recorder, pSpring->GetStartObject() )
				|| !WritePhysicsReference( recorder, pSpring->GetEndObject() ) )
				return false;
			springparams_t springParams;
			pSpring->GetSaveParams( springParams );
			recorder.WriteBytes( &springParams, sizeof( springParams ) );
			return true;
		}
		case PIID_IPHYSICSCONSTRAINTGROUP:
		{
			auto *pGroup = static_cast< JoltPhysicsConstraintGroup * >( params.pObject );
			constraint_groupparams_t groupParams;
			pGroup->GetErrorParams( &groupParams );
			recorder.Write( groupParams );
			recorder.Write( pGroup->IsActive() );
			return true;
		}
		case PIID_IPHYSICSCONSTRAINT:
		{
			JoltPhysicsConstraint *pConstraint = reinterpret_cast< JoltPhysicsConstraint * >( params.pObject );
			if ( !pConstraint || !pConstraint->CanSave() )
				return false;

			JoltPhysicsObject *pRefObject = reinterpret_cast< JoltPhysicsObject * >( pConstraint->GetReferenceObject() );
			JoltPhysicsObject *pAttObject = reinterpret_cast< JoltPhysicsObject * >( pConstraint->GetAttachedObject() );

			if ( !WritePhysicsReference( recorder, pRefObject )
				|| !WritePhysicsReference( recorder, pAttObject ) )
				return false;
			recorder.Write( pConstraint->GetGroup() != nullptr );
			if ( pConstraint->GetGroup() && !WritePhysicsReference( recorder, pConstraint->GetGroup() ) )
				return false;
			recorder.Write( pConstraint->WasConstraintSystemDestroyed() );
			pConstraint->SaveConstraintSettings( recorder );
			return true;
		}
		case PIID_IPHYSICSSHADOWCONTROLLER:
		{
			auto *pController = static_cast< JoltPhysicsShadowController * >( params.pObject );
			if ( !WritePhysicsReference( recorder, pController->GetObject() ) )
				return false;
			pController->SaveState( recorder );
			return true;
		}
		case PIID_IPHYSICSPLAYERCONTROLLER:
			// This just returns false in regular VPhysics.
			return false;
		case PIID_IPHYSICSMOTIONCONTROLLER:
		{
			auto *pController = static_cast< JoltPhysicsMotionController * >( params.pObject );
			recorder.Write( pController->GetPriority() );
			const int count = pController->CountObjects();
			if ( count < 0 || count > int( kMaxBodies ) )
				return false;
			recorder.Write( count );
			std::vector< IPhysicsObject * > objects( count );
			pController->GetObjects( objects.data() );
			for ( IPhysicsObject *pObject : objects )
				if ( !WritePhysicsReference( recorder, pObject ) )
					return false;
			// IMotionEvent is supplied again by the game's OnRestore / SetEventHandler.
			return true;
		}
		case PIID_IPHYSICSVEHICLECONTROLLER:
		{
			auto *pController = static_cast< JoltPhysicsVehicleController * >( params.pObject );
			if ( !pController->HasSaveableTopology()
				|| !WritePhysicsReference( recorder, pController->GetBodyObject() ) )
				return false;
			recorder.Write( pController->GetVehicleType() );
			const vehicleparams_t &vehicleParams = pController->GetVehicleParams();
			recorder.WriteBytes( &vehicleParams, sizeof( vehicleParams ) );
			recorder.Write( pController->GetBridgeConfig() );
			const int count = pController->GetWheelCount();
			recorder.Write( count );
			for ( int i = 0; i < count; ++i )
				if ( !WritePhysicsReference( recorder, pController->GetWheel( i ) ) )
					return false;
			pController->SaveState( recorder );
			return true;
		}
		case PIID_IPHYSICSGAMETRACE:
			Log_Warning( LOG_VJolt, "Saving PIID_IPHYSICSGAMETRACE is unsupported right now.\n" );
			return false;
	}
}

void JoltPhysicsEnvironment::PreRestore( const physprerestoreparams_t &params )
{
	m_RestoredBodyActivation.clear();
#if defined( GAME_GMOD_64X )
	// ID 0 is the persistent world-object reference in GMod's 64-bit save format.
	m_GModSaveRestoreIds.BeginRestore( params.pWorldObject, PIID_IPHYSICSOBJECT );
#else
	m_SaveRestorePointerMap.clear();
	for ( int i = 0; i < params.recreatedObjectCount; i++ )
		AddPhysicsSaveRestorePointer(
			reinterpret_cast< uintp >( params.recreatedObjectList[ i ].pOldObject ),
			params.recreatedObjectList[ i ].pNewObject );
#endif
}

bool JoltPhysicsEnvironment::Restore( const physrestoreparams_t &params )
{
	if ( !params.pRestore || !params.ppObject || params.type <= PIID_UNKNOWN || params.type >= PIID_NUM_TYPES )
		return false;

	*params.ppObject = nullptr;

#if defined( GAME_GMOD_64X )
	VJoltSaveRestoreIdTable::Id objectId;
	if ( !ReadPhysicsSaveRestoreId( params.pRestore, objectId ) )
	{
		Log_Warning( LOG_VJolt, "Failed to read a GMod physics save/restore object ID.\n" );
		return false;
	}

	if ( !m_GModSaveRestoreIds.IsRestoring() )
	{
		Log_Warning( LOG_VJolt, "Physics Restore called without a matching PreRestore.\n" );
		return false;
	}
	void *pExisting = nullptr;
	if ( m_GModSaveRestoreIds.LookupRestoreId( objectId, pExisting ) )
	{
		Log_Warning( LOG_VJolt, "Duplicate physics restore ID %u.\n", objectId );
		return false;
	}
#else
	uintp originalPtr = 0;
	if ( params.pRestore->ReadData( reinterpret_cast< char * >( &originalPtr ), sizeof( originalPtr ), 0 ) != sizeof( originalPtr ) )
		return false;
#endif

	const int payloadPosition = params.pRestore->GetReadPos();
	int magic = 0;
	if ( params.pRestore->ReadInt( &magic, 1, 0 ) <= 0 )
		return false;
	bool restored = false;
	if ( magic == kPhysicsSaveMagic )
	{
		int header[ 2 ] = {};
		if ( params.pRestore->ReadInt( header, 2, 0 ) <= 0
			|| header[ 0 ] != kPhysicsSaveVersion || header[ 1 ] <= 0 || header[ 1 ] > kMaxPhysicsSaveBytes )
		{
			Log_Warning( LOG_VJolt, "Invalid or unsupported physics save payload header.\n" );
			return false;
		}
		std::vector< unsigned char > bytes( header[ 1 ] );
		if ( params.pRestore->ReadData( reinterpret_cast< char * >( bytes.data() ), header[ 1 ], 0 ) != header[ 1 ] )
		{
			Log_Warning( LOG_VJolt, "Truncated physics save payload.\n" );
			return false;
		}
		VJoltStateRecorder recorder( bytes.data(), header[ 1 ], CUtlBuffer::READ_ONLY );
		restored = RestorePhysicsState( params, recorder, true );
	}
	else
	{
		// Previous Jolt saves had no payload header and only implemented these two types.
		if ( params.type != PIID_IPHYSICSOBJECT && params.type != PIID_IPHYSICSCONSTRAINT )
			return false;
		params.pRestore->SetReadPos( payloadPosition );
		JoltStateRecorderSave recorder( params.pRestore );
		restored = RestorePhysicsState( params, recorder, false );
	}
	if ( !restored )
	{
		Log_Warning( LOG_VJolt, "Failed to restore physics interface type %d.\n", int( params.type ) );
		return false;
	}
#if defined( GAME_GMOD_64X )
	// References are registered only after a complete payload has been restored successfully.
	const bool registered = m_GModSaveRestoreIds.RegisterRestoreId( objectId, *params.ppObject, params.type );
	VJoltAssert( registered );
	return registered;
#else
	AddPhysicsSaveRestorePointer( originalPtr, *params.ppObject );
	return true;
#endif
}

bool JoltPhysicsEnvironment::RestorePhysicsState( const physrestoreparams_t &params,
	JPH::StateRecorder &recorder, bool extended )
{
	JPH::BodyInterface &bodyInterface = m_PhysicsSystem.GetBodyInterfaceNoLock();
	auto restoreShadow = [&]( JoltPhysicsObject *pObject ) -> JoltPhysicsShadowController *
	{
		auto *pController = static_cast< JoltPhysicsShadowController * >( pObject->GetShadowController() );
		if ( pController )
		{
			// A shadow may be embedded in the object and also have its own game-side save field.
			VJoltStateRecorder previous;
			pController->SaveState( previous );
			if ( !pController->RestoreState( recorder ) )
			{
				previous.SeekGet( CUtlBuffer::SEEK_HEAD, 0 );
				pController->RestoreState( previous );
				return nullptr;
			}
			return pController;
		}
		pController = new JoltPhysicsShadowController( pObject, recorder );
		if ( !pController->GetObject() )
		{
			delete pController;
			return nullptr;
		}
		pObject->SetRestoredShadowController( pController );
		RegisterPhysicsController( pController );
		return pController;
	};

	switch ( params.type )
	{
		default:
		case PIID_UNKNOWN:
			Log_Warning( LOG_VJolt, "Restoring PIID_UNKNOWN is unsupported right now.\n" );
			return false;
		case PIID_IPHYSICSOBJECT:
		{
			bool sphere = false, active = false;
			float radius = 0.0f;
			JPH::Vec3 massCenter = JPH::Vec3::sZero();
			if ( extended )
			{
				recorder.Read( sphere );
				if ( sphere )
					recorder.Read( radius );
				recorder.Read( active );
				recorder.Read( massCenter );
			}
			JPH::BodyCreationSettings bodyCreationSettings;
			bodyCreationSettings.RestoreBinaryState( recorder );
			if ( VJoltStateRecorderFailed( recorder ) || bodyCreationSettings.mMotionType > JPH::EMotionType::Dynamic
				|| massCenter.IsNaN() || !std::isfinite( massCenter.LengthSq() ) )
				return false;
			JPH::RefConst<JPH::Shape> shape;
			if ( sphere )
			{
				const float joltRadius = SourceToJolt::Distance( radius );
				if ( std::isfinite( joltRadius ) && joltRadius > 0.0f )
					shape = new JPH::SphereShape( joltRadius );
				else
					shape = new JPH::EmptyShape();
			}
			else if ( params.pCollisionModel )
				shape = params.pCollisionModel->ToShape();
			else
				return false;
			// A per-object massCenterOverride is held by a shape decorator, not by the shared
			// collision model supplied by the game or BodyCreationSettings' binary fields.
			if ( extended )
				shape = CreateCOMOverrideShape( shape.GetPtr(), massCenter );
			bodyCreationSettings.SetShape( shape.GetPtr() );
			JPH::Body *pBody = bodyInterface.CreateBody( bodyCreationSettings );
			if ( !pBody )
				return false;

			bodyInterface.AddBody( pBody->GetID(), JPH::EActivation::DontActivate );
			JoltPhysicsObject *pJoltObject = new JoltPhysicsObject( pBody, this, params.pGameData, recorder, extended );
			bool hasShadow = false;
			if ( extended )
				recorder.Read( hasShadow );
			if ( VJoltStateRecorderFailed( recorder ) || ( hasShadow && !restoreShadow( pJoltObject ) ) )
			{
				RemoveBodyAndDeleteObject( pJoltObject );
				return false;
			}
			if ( extended )
				m_RestoredBodyActivation.emplace_back( pBody->GetID(), active );
			*params.ppObject = reinterpret_cast< void * >( pJoltObject );
			return true;
		}
		case PIID_IPHYSICSFLUIDCONTROLLER:
			// Given saving this just returns false, this should never happen.
			Log_Warning( LOG_VJolt, "Restoring PIID_IPHYSICSFLUIDCONTROLLER is unsupported right now.\n" );
			return false;
		case PIID_IPHYSICSSPRING:
		{
			JoltPhysicsObject *pStart = nullptr, *pEnd = nullptr;
			if ( !ReadPhysicsReference( recorder, pStart, PIID_IPHYSICSOBJECT )
				|| !ReadPhysicsReference( recorder, pEnd, PIID_IPHYSICSOBJECT ) || pStart == pEnd )
				return false;
			springparams_t springParams = {};
			recorder.ReadBytes( &springParams, sizeof( springParams ) );
			if ( VJoltStateRecorderFailed( recorder ) || !springParams.startPosition.IsValid()
				|| !springParams.endPosition.IsValid() || !std::isfinite( springParams.naturalLength )
				|| !std::isfinite( springParams.constant ) || !std::isfinite( springParams.damping )
				|| !std::isfinite( springParams.relativeDamping ) )
				return false;
			*params.ppObject = CreateSpring( pStart, pEnd, &springParams );
			return *params.ppObject != nullptr;
		}
		case PIID_IPHYSICSCONSTRAINTGROUP:
		{
			constraint_groupparams_t groupParams = {};
			bool active = false;
			recorder.Read( groupParams );
			recorder.Read( active );
			if ( VJoltStateRecorderFailed( recorder ) || !std::isfinite( groupParams.errorTolerance ) )
				return false;
			// Restore the latched active state even if the group is currently empty. Ordinary
			// Activate() intentionally ignores empty groups; subsequent restored members inherit it.
			*params.ppObject = new JoltPhysicsConstraintGroup( this, groupParams, active );
			return true;
		}
		case PIID_IPHYSICSCONSTRAINT:
		{
			JoltPhysicsObject *pRefObject = nullptr, *pAttObject = nullptr;
			if ( !ReadPhysicsReference( recorder, pRefObject, PIID_IPHYSICSOBJECT )
				|| !ReadPhysicsReference( recorder, pAttObject, PIID_IPHYSICSOBJECT ) || pRefObject == pAttObject )
				return false;
			JoltPhysicsConstraintGroup *pGroup = nullptr;
			bool systemDestroyed = false;
			if ( extended )
			{
				bool hasGroup = false;
				recorder.Read( hasGroup );
				if ( hasGroup && !ReadPhysicsReference( recorder, pGroup, PIID_IPHYSICSCONSTRAINTGROUP ) )
					return false;
				recorder.Read( systemDestroyed );
			}

			constraintType_t type = CONSTRAINT_UNKNOWN;
			recorder.Read( type );
			if ( VJoltStateRecorderFailed( recorder ) || type <= CONSTRAINT_UNKNOWN || type > CONSTRAINT_LENGTH
				|| ( systemDestroyed && pGroup ) )
				return false;
			JPH::ConstraintSettings::ConstraintResult result = JPH::ConstraintSettings::sRestoreFromBinaryState( recorder );

			if ( result.HasError() )
			{
				Log_Warning( LOG_VJolt, "Error restoring constraint: %s.\n", result.GetError().c_str() );
				return false;
			}

			const auto *pSettings = dynamic_cast< const JPH::TwoBodyConstraintSettings * >( result.Get().GetPtr() );
			if ( !pSettings || VJoltStateRecorderFailed( recorder ) )
				return false;
			JPH::Constraint *pConstraint = bodyInterface.CreateConstraint( pSettings, pRefObject->GetBodyID(), pAttObject->GetBodyID() );
			if ( !pConstraint )
				return false;

			pConstraint->RestoreState( recorder );
			m_PhysicsSystem.AddConstraint( pConstraint );
			const bool enabled = pConstraint->GetEnabled();
			JoltPhysicsConstraint* pJoltConstraint = new JoltPhysicsConstraint( this, pRefObject, pAttObject,
				type, pConstraint, params.pGameData, pGroup, systemDestroyed );
			pJoltConstraint->RestoreConstraintSettings( recorder );
			pConstraint->SetEnabled( enabled && !systemDestroyed );
			if ( VJoltStateRecorderFailed( recorder ) )
			{
				delete pJoltConstraint;
				return false;
			}
			*params.ppObject = reinterpret_cast<void*>( pJoltConstraint );
			return true;
		}
		case PIID_IPHYSICSSHADOWCONTROLLER:
		{
			JoltPhysicsObject *pObject = nullptr;
			if ( !ReadPhysicsReference( recorder, pObject, PIID_IPHYSICSOBJECT ) )
				return false;
			*params.ppObject = restoreShadow( pObject );
			return *params.ppObject != nullptr;
		}
		case PIID_IPHYSICSPLAYERCONTROLLER:
			// Given saving this just returns false, this should never happen.
			Log_Warning( LOG_VJolt, "Restoring PIID_IPHYSICSPLAYERCONTROLLER is unsupported right now.\n" );
			return false;
		case PIID_IPHYSICSMOTIONCONTROLLER:
		{
			IPhysicsMotionController::priority_t priority = IPhysicsMotionController::MEDIUM_PRIORITY;
			int count = 0;
			recorder.Read( priority );
			recorder.Read( count );
			if ( VJoltStateRecorderFailed( recorder ) || count < 0 || count > int( kMaxBodies )
				|| priority < IPhysicsMotionController::LOW_PRIORITY || priority > IPhysicsMotionController::HIGH_PRIORITY )
				return false;
			std::vector< JoltPhysicsObject * > objects( count );
			for ( JoltPhysicsObject *&pObject : objects )
				if ( !ReadPhysicsReference( recorder, pObject, PIID_IPHYSICSOBJECT ) || pObject->IsStatic() )
					return false;
			auto *pController = static_cast< JoltPhysicsMotionController * >( CreateMotionController( nullptr ) );
			pController->SetPriority( priority );
			for ( JoltPhysicsObject *pObject : objects )
				pController->AttachObject( pObject, false );
			*params.ppObject = pController;
			return true;
		}
		case PIID_IPHYSICSVEHICLECONTROLLER:
		{
			JoltPhysicsObject *pBody = nullptr;
			if ( !ReadPhysicsReference( recorder, pBody, PIID_IPHYSICSOBJECT )
				|| pBody->IsStatic() || IsVehicleControlledObject( pBody ) )
				return false;
			unsigned int vehicleType = 0;
			vehicleparams_t vehicleParams = {};
			JoltVehicleBridgeConfig_t bridgeConfig;
			int count = 0;
			recorder.Read( vehicleType );
			recorder.ReadBytes( &vehicleParams, sizeof( vehicleParams ) );
			recorder.Read( bridgeConfig );
			recorder.Read( count );
			const bool airboat = vehicleType == VEHICLE_TYPE_AIRBOAT_RAYCAST;
			if ( VJoltStateRecorderFailed( recorder )
				|| ( !airboat && vehicleType != VEHICLE_TYPE_CAR_WHEELS
					&& vehicleType != VEHICLE_TYPE_CAR_RAYCAST && vehicleType != VEHICLE_TYPE_JETSKI_RAYCAST )
				|| vehicleParams.axleCount <= 0 || vehicleParams.axleCount > VEHICLE_MAX_AXLE_COUNT
				|| vehicleParams.wheelsPerAxle <= 0 || vehicleParams.wheelsPerAxle > VEHICLE_MAX_WHEEL_COUNT
				|| count <= 0 || count > ( airboat ? JOLT_AIRBOAT_MAX_PONTOONS : VEHICLE_MAX_WHEEL_COUNT )
				|| count != vehicleParams.axleCount * vehicleParams.wheelsPerAxle
				|| vehicleParams.engine.gearCount < ( airboat ? 0 : 1 )
				|| vehicleParams.engine.gearCount > VEHICLE_MAX_GEAR_COUNT )
				return false;
			for ( int axle = 0; axle < vehicleParams.axleCount; ++axle )
			{
				const auto &wheel = vehicleParams.axles[ axle ].wheels;
				if ( !std::isfinite( wheel.mass ) || wheel.mass <= 0.0f
					|| !std::isfinite( wheel.radius ) || wheel.radius <= 0.0f )
					return false;
			}
			std::vector< JoltPhysicsObject * > wheels;
			wheels.reserve( count );
			for ( int i = 0; i < count; ++i )
			{
				JoltPhysicsObject *pWheel = nullptr;
				if ( !ReadPhysicsReference( recorder, pWheel, PIID_IPHYSICSOBJECT )
					|| pWheel == pBody || pWheel->IsStatic() || IsVehicleControlledObject( pWheel )
					|| std::find( wheels.begin(), wheels.end(), pWheel ) != wheels.end() )
					return false;
				wheels.push_back( pWheel );
			}
			auto *pController = new JoltPhysicsVehicleController( this, &m_PhysicsSystem, pBody,
				vehicleParams, vehicleType, params.pGameTrace, &bridgeConfig, &wheels );
			if ( !pController->RestoreState( recorder ) )
			{
				pController->ReleaseRestoredWheels();
				delete pController;
				return false;
			}
			RegisterPhysicsController( pController );
			*params.ppObject = pController;
			return true;
		}
		case PIID_IPHYSICSGAMETRACE:
			Log_Warning( LOG_VJolt, "Restoring PIID_IPHYSICSGAMETRACE is unsupported right now.\n" );
			return false;
	}
}

void JoltPhysicsEnvironment::PostRestore()
{
	// Springs and controllers may wake their endpoints while attaching. Apply the saved state
	// after all types have been restored, and use BodyIDs so intervening deletion is harmless.
	JPH::BodyInterface &bodyInterface = m_PhysicsSystem.GetBodyInterfaceNoLock();
	for ( const auto &[bodyId, active] : m_RestoredBodyActivation )
	{
		JPH::Body *pBody = m_PhysicsSystem.GetBodyLockInterfaceNoLock().TryGetBody( bodyId );
		if ( !pBody || pBody->IsStatic() )
			continue;
		if ( active )
			bodyInterface.ActivateBody( bodyId );
		else
			bodyInterface.DeactivateBody( bodyId );
	}
	m_RestoredBodyActivation.clear();
#if defined( GAME_GMOD_64X )
	m_GModSaveRestoreIds.EndRestore();
#else
	m_SaveRestorePointerMap.clear();
#endif
}

void JoltPhysicsEnvironment::ForgetRestoredPointer( const void *pObject )
{
#if defined( GAME_GMOD_64X )
	m_GModSaveRestoreIds.ForgetRestorePointer( pObject );
#else
	for ( auto iter = m_SaveRestorePointerMap.begin(); iter != m_SaveRestorePointerMap.end(); )
	{
		if ( iter->second == pObject )
			iter = m_SaveRestorePointerMap.erase( iter );
		else
			++iter;
	}
#endif
}

//-------------------------------------------------------------------------------------------------

bool JoltPhysicsEnvironment::IsCollisionModelUsed( CPhysCollide *pCollide ) const
{
	// Josh: This is only used in debug code.
	return false;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::TraceRay( const Ray_t &ray, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace )
{
	// Josh: This does nothing in regular vphysics.
}

void JoltPhysicsEnvironment::SweepCollideable( const CPhysCollide *pCollide, const Vector &vecAbsStart, const Vector &vecAbsEnd,
	const QAngle &vecAngles, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace )
{
	// Josh: This does nothing in regular vphysics.
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::GetPerformanceSettings( physics_performanceparams_t *pOutput ) const
{
	if ( pOutput )
		*pOutput = m_PerformanceParams;
}

void JoltPhysicsEnvironment::SetPerformanceSettings( const physics_performanceparams_t *pSettings )
{
	if ( !pSettings )
		return;

	m_PerformanceParams = *pSettings;

	// Normalize these values to match VPhysics behaviour.
	m_PerformanceParams.minFrictionMass = Clamp( m_PerformanceParams.minFrictionMass, 1.0f, VPHYSICS_MAX_MASS );
	m_PerformanceParams.maxFrictionMass = Clamp( m_PerformanceParams.maxFrictionMass, 1.0f, VPHYSICS_MAX_MASS );

	GetSourceBodiesForRead( m_CachedBodies );

	for ( auto& id : m_CachedBodies )
	{
		JPH::Body* pBody = m_PhysicsSystem.GetBodyLockInterfaceNoLock().TryGetBody( id );

		if ( pBody )
		{
			JPH::MotionProperties* pMotionProperties = pBody->GetMotionProperties();

			if ( pMotionProperties )
			{
				const float maxLinearVelocity = SourceToJolt::Distance( pSettings->maxVelocity );
				const float maxAngularVelocity = DEG2RAD( pSettings->maxAngularVelocity );
				JoltPhysicsObject *pObject = reinterpret_cast< JoltPhysicsObject * >(
					pBody->GetUserData() );
				if ( pObject && pObject->GetShadowController() )
				{
					static_cast< JoltPhysicsShadowController * >(
						pObject->GetShadowController() )->SetEnvironmentVelocityLimits(
							maxLinearVelocity, maxAngularVelocity );
				}
				else
				{
					pMotionProperties->SetMaxLinearVelocity( maxLinearVelocity );
					pMotionProperties->SetMaxAngularVelocity( maxAngularVelocity );
				}
			}
		}
	}
}

inline int JoltPhysicsEnvironment::MaxCollisionsPerObjectPerTimestep() const
{
	return m_PerformanceParams.maxCollisionsPerObjectPerTimestep;
}

inline int JoltPhysicsEnvironment::MaxCollisionChecksPerTimestep() const
{
	return m_PerformanceParams.maxCollisionChecksPerTimestep;
}

inline float JoltPhysicsEnvironment::MaxVelocity() const
{
	return m_PerformanceParams.maxVelocity;
}

inline float JoltPhysicsEnvironment::MaxAngularVelocity() const
{
	return m_PerformanceParams.maxAngularVelocity;
}

inline float JoltPhysicsEnvironment::LookAheadTimeObjectsVsWorld() const
{
	return m_PerformanceParams.lookAheadTimeObjectsVsWorld;
}

inline float JoltPhysicsEnvironment::LookAheadTimeObjectsVsObject() const
{
	return m_PerformanceParams.lookAheadTimeObjectsVsObject;
}

inline float JoltPhysicsEnvironment::MinFrictionMass() const
{
	return m_PerformanceParams.minFrictionMass;
}

inline float JoltPhysicsEnvironment::MaxFrictionMass() const
{
	return m_PerformanceParams.maxFrictionMass;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::ReadStats( physics_stats_t *pOutput )
{
	m_ContactListener.ReadStats( pOutput );
}

void JoltPhysicsEnvironment::ClearStats()
{
	m_ContactListener.ClearStats();
}

//-------------------------------------------------------------------------------------------------

// StateRecorder implementation that gathers the size required to save a body's state
class VJoltStateSizeRecorder final : public JPH::StateRecorder
{
public:
	// StreamIn
	void ReadBytes( void *outData, size_t inNumBytes ) override {}
	bool IsEOF() const override { return false; }

	// StreamOut
	void WriteBytes( const void *inData, size_t inNumBytes )
	{
		saveSize += inNumBytes;
	}

	// Both
	bool IsFailed() const override { return false; }

public:
	size_t saveSize = 0;
};

namespace
{
	constexpr uint32 kObjectBufferMagic = 0x424F4A56; // VJOB
	constexpr uint32 kObjectBufferVersion = 1;
	constexpr unsigned int kObjectBufferHeaderSize = 4 * sizeof( uint32 );

	// Bare Source spheres have no collide metadata. Serialize them through a
	// private identity wrapper, without adding metadata to the live sphere.
	class ObjectBufferShape
	{
	public:
		explicit ObjectBufferShape( const JPH::Shape *source )
		{
			if ( source->GetUserData() )
				m_Shape = const_cast<JPH::Shape *>( source );
			else
			{
				m_Shape = new JPH::OffsetCenterOfMassShape( source, JPH::Vec3::sZero() );
				m_Shape->SetUserData( reinterpret_cast<uint64>( &m_Data ) );
			}
		}
		CPhysCollide *GetCollide() const { return CPhysCollide::FromShape( m_Shape.GetPtr() ); }
	private:
		JoltCollideUserData m_Data;
		JPH::Ref<JPH::Shape> m_Shape;
	};

	void WriteObjectBufferState( JoltPhysicsObject *object, JPH::StateRecorder &recorder )
	{
		object->GetBody()->GetBodyCreationSettings().SaveBinaryState( recorder );
		object->SaveObjectState( recorder, true );
	}
}

unsigned int JoltPhysicsEnvironment::GetObjectSerializeSize( IPhysicsObject *pObject ) const
{
	if ( !pObject )
		return 0;
	auto *object = static_cast<JoltPhysicsObject *>( pObject );
	ObjectBufferShape shape( object->GetBody()->GetShape() );
	const int shapeSize = JoltPhysicsCollision::GetInstance().CollideSize( shape.GetCollide() );
	VJoltStateSizeRecorder recorder;
	WriteObjectBufferState( object, recorder );
	const size_t size = kObjectBufferHeaderSize + size_t( Max( 0, shapeSize ) ) + recorder.saveSize;
	return shapeSize > 0 && size <= size_t( INT_MAX ) ? unsigned( size ) : 0;
}

void JoltPhysicsEnvironment::SerializeObjectToBuffer( IPhysicsObject *pObject, unsigned char *pBuffer, unsigned int bufferSize )
{
	if ( !pObject || !pBuffer || bufferSize > unsigned( INT_MAX ) )
		return;
	auto *object = static_cast<JoltPhysicsObject *>( pObject );
	ObjectBufferShape shape( object->GetBody()->GetShape() );
	auto &collision = JoltPhysicsCollision::GetInstance();
	const int shapeSize = collision.CollideSize( shape.GetCollide() );
	VJoltStateSizeRecorder sizeRecorder;
	WriteObjectBufferState( object, sizeRecorder );
	if ( shapeSize <= 0 || kObjectBufferHeaderSize + size_t( shapeSize ) + sizeRecorder.saveSize > bufferSize )
	{
		// Invalidate an undersized destination rather than leave an older valid record.
		if ( bufferSize >= sizeof( uint32 ) )
			memset( pBuffer, 0, sizeof( uint32 ) );
		return;
	}
	VJoltStateRecorder recorder( pBuffer, int( bufferSize ) );
	recorder.Write( kObjectBufferMagic );
	recorder.Write( kObjectBufferVersion );
	recorder.Write( uint32( shapeSize ) );
	recorder.Write( uint32( sizeRecorder.saveSize ) );
	std::vector<char> shapeBytes( size_t( shapeSize ), '\0' );
	if ( collision.CollideWrite( shapeBytes.data(), shape.GetCollide() ) != shapeSize )
	{
		memset( pBuffer, 0, sizeof( uint32 ) );
		return;
	}
	recorder.WriteBytes( shapeBytes.data(), shapeBytes.size() );
	WriteObjectBufferState( object, recorder );
}

IPhysicsObject *JoltPhysicsEnvironment::UnserializeObjectFromBuffer( void *pGameData, unsigned char *pBuffer, unsigned int bufferSize, bool enableCollisions )
{
	if ( !pBuffer || bufferSize < kObjectBufferHeaderSize || bufferSize > unsigned( INT_MAX ) )
		return nullptr;
	VJoltStateRecorder header( pBuffer, int( bufferSize ), CUtlBuffer::READ_ONLY );
	uint32 magic = 0, version = 0, shapeSize = 0, stateSize = 0;
	header.Read( magic ); header.Read( version ); header.Read( shapeSize ); header.Read( stateSize );
	if ( magic != kObjectBufferMagic || version != kObjectBufferVersion || shapeSize == 0 || stateSize == 0
		|| uint64( kObjectBufferHeaderSize ) + shapeSize + stateSize > bufferSize )
		return nullptr;
	auto &collision = JoltPhysicsCollision::GetInstance();
	CPhysCollide *ownedCollide = collision.UnserializeCollide(
		reinterpret_cast<char *>( pBuffer + kObjectBufferHeaderSize ), int( shapeSize ), 0 );
	if ( !ownedCollide )
		return nullptr;
	VJoltStateRecorder recorder( pBuffer + kObjectBufferHeaderSize + shapeSize, int( stateSize ), CUtlBuffer::READ_ONLY );
	JPH::BodyCreationSettings bodyCreationSettings;
	bodyCreationSettings.RestoreBinaryState( recorder );
	if ( recorder.IsFailed() )
	{
		collision.DestroyCollide( ownedCollide );
		return nullptr;
	}
	bodyCreationSettings.SetShape( ownedCollide->ToShape() );
	JPH::BodyInterface &bodyInterface = m_PhysicsSystem.GetBodyInterfaceNoLock();
	JPH::Body *pBody = bodyInterface.CreateBody( bodyCreationSettings );
	if ( !pBody )
	{
		collision.DestroyCollide( ownedCollide );
		return nullptr;
	}
	bodyInterface.AddBody( pBody->GetID(), JPH::EActivation::Activate );
	JoltPhysicsObject *pJoltObject = new JoltPhysicsObject( pBody, this, pGameData, recorder, true );
	pJoltObject->OwnRestoredCollide( ownedCollide );
	if ( recorder.IsFailed() || recorder.GetBytesRemaining() != 0 )
	{
		RemoveBodyAndDeleteObject( pJoltObject );
		return nullptr;
	}
	pJoltObject->EnableCollisions( enableCollisions );
	return pJoltObject;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::EnableConstraintNotify( bool bEnable )
{
	m_EnableConstraintNotify = bEnable;
}

void JoltPhysicsEnvironment::DebugCheckContacts()
{
	Log_Stub( LOG_VJolt );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::SetAlternateGravity( const Vector &gravityVector )
{
	Log_Stub( LOG_VJolt );
}

void JoltPhysicsEnvironment::GetAlternateGravity( Vector *pGravityVector ) const
{
	Log_Stub( LOG_VJolt );
}

//-------------------------------------------------------------------------------------------------

float JoltPhysicsEnvironment::GetDeltaFrameTime( int maxTicks ) const
{
	Log_Stub( LOG_VJolt );
	// TODO(Josh): wtf to do here?
	return m_flStepTime * maxTicks;
}

void JoltPhysicsEnvironment::ForceObjectsToSleep( IPhysicsObject **pList, int listCount )
{
	for ( int i = 0; i < listCount; i++ )
		pList[ i ]->Sleep();
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::SetPredicted( bool bPredicted )
{
	Log_Stub( LOG_VJolt );
}

bool JoltPhysicsEnvironment::IsPredicted()
{
	Log_Stub( LOG_VJolt );
	return false;
}

void JoltPhysicsEnvironment::SetPredictionCommandNum( int iCommandNum )
{
	Log_Stub( LOG_VJolt );
}

int JoltPhysicsEnvironment::GetPredictionCommandNum()
{
	Log_Stub( LOG_VJolt );
	return 0;
}

void JoltPhysicsEnvironment::DoneReferencingPreviousCommands( int iCommandNum )
{
	Log_Stub( LOG_VJolt );
}

void JoltPhysicsEnvironment::RestorePredictedSimulation()
{
	Log_Stub( LOG_VJolt );
}

//-------------------------------------------------------------------------------------------------

#if GAME_GMOD
void JoltPhysicsEnvironment::SetGModObjectEventHandler( IGModPhysicsObjectEvent *pGModObjectEvent )
{
	m_pGModObjectEvents = pGModObjectEvent;
}
#endif

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::NotifyConstraintDisabled( JoltPhysicsConstraint* pConstraint )
{
	if ( m_pConstraintListener && m_EnableConstraintNotify )
		m_pConstraintListener->ConstraintBroken( pConstraint );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::RegisterConstraint( JoltPhysicsConstraint *pConstraint )
{
	m_pConstraints.push_back( pConstraint );
}

void JoltPhysicsEnvironment::UnregisterConstraint( JoltPhysicsConstraint *pConstraint )
{
	UnregisterBreakableConstraint( pConstraint );
	Erase( m_pConstraints, pConstraint );
}

void JoltPhysicsEnvironment::RegisterBreakableConstraint( JoltPhysicsConstraint *pConstraint )
{
	if ( pConstraint && !VectorContains( m_pBreakableConstraints, pConstraint ) )
		m_pBreakableConstraints.push_back( pConstraint );
}

void JoltPhysicsEnvironment::UnregisterBreakableConstraint( JoltPhysicsConstraint *pConstraint )
{
	Erase( m_pBreakableConstraints, pConstraint );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsEnvironment::AddDirtyStaticBody( const JPH::BodyID &id )
{
	m_DirtyStaticBodies.push_back( id );
}

void JoltPhysicsEnvironment::RemoveDirtyStaticBody( const JPH::BodyID &id )
{
	Erase( m_DirtyStaticBodies, id );
}

void JoltPhysicsEnvironment::QueuePendingVelocityCommit( JoltPhysicsObject *pObject )
{
	m_pPendingVelocityObjects.push_back( pObject );
}

void JoltPhysicsEnvironment::RemovePendingVelocityCommit( JoltPhysicsObject *pObject )
{
	Erase( m_pPendingVelocityObjects, pObject );
}

void JoltPhysicsEnvironment::CommitPendingVelocityChanges()
{
	// Clear the environment queue before calling into objects. This makes a new public force
	// submitted while committing belong to the next batch instead of being lost by clear().
	m_pPendingVelocityWorkObjects.clear();
	m_pPendingVelocityWorkObjects.swap( m_pPendingVelocityObjects );
	for ( JoltPhysicsObject *pObject : m_pPendingVelocityWorkObjects )
		pObject->CommitPendingVelocityChanges();
	m_pPendingVelocityWorkObjects.clear();

	// Usually no object queues another public force while the current batch is committed. Give the
	// larger retained allocation back to the producer queue in that common case so the very next
	// frame can reuse it. If a callback did queue work, leave both buffers separate: that work must
	// remain pending until the next explicit commit boundary.
	if ( m_pPendingVelocityObjects.empty()
		&& m_pPendingVelocityObjects.capacity() < m_pPendingVelocityWorkObjects.capacity() )
	{
		m_pPendingVelocityObjects.swap( m_pPendingVelocityWorkObjects );
	}
}

//-------------------------------------------------------------------------------------------------

template < typename T >
void JoltPhysicsEnvironment::AddPhysicsSaveRestorePointer( uintp oldPtr, T* newPtr )
{
	VJoltAssert( oldPtr != 0 );
	VJoltAssert( newPtr != 0 );

	m_SaveRestorePointerMap[ oldPtr ] = reinterpret_cast< void * >( newPtr );
}

template < typename T >
T *JoltPhysicsEnvironment::LookupPhysicsSaveRestorePointer( uintp oldPtr )
{
	if ( !oldPtr )
		return nullptr;

	auto iter = m_SaveRestorePointerMap.find( oldPtr );
	if ( iter == m_SaveRestorePointerMap.end() )
		return nullptr;

	return reinterpret_cast< T * >( iter->second );
}

//-------------------------------------------------------------------------------------------------

#if defined( GAME_GMOD_64X )
// NOTE: physprerestoreparams_t was named to physpresaverestoreparams_t though we kept the original for now.
void JoltPhysicsEnvironment::PreSave(const physprerestoreparams_t& params)
{
	// GMod's original 64-bit VPhysics reserves ID 0 for the world object.
	m_GModSaveRestoreIds.BeginSave( params.pWorldObject );
}

void JoltPhysicsEnvironment::PostSave()
{
	m_GModSaveRestoreIds.EndSave();
}
#endif
