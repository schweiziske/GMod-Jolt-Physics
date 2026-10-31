//=================================================================================================
//
// Interface to a physics scene
//
//=================================================================================================

#pragma once

#include "vjolt_interface.h"
#include "vjolt_object.h"
#include "vjolt_constraints.h"
#include "vjolt_constraint_wake.h"
#include "vjolt_environment_events.h"
#include "vjolt_listener_contact.h"
#include "vjolt_saverestore_ids.h"
#include "vjolt_update_error.h"
#include "vjolt_owner_callback_jobs.h"

#include <Jolt/Physics/Body/BodyActivationListener.h>

class JoltBroadPhaseLayerInterface;
class JoltObjectVsBroadPhaseLayerFilter;
class JoltObjectLayerPairFilter;
class JoltPhysicsDragController;
class JoltPhysicsSpringDamper;
struct JoltVehicleBridgeConfig_t;

class JoltPhysicsActivationListener final : public JPH::BodyActivationListener
{
public:
	explicit JoltPhysicsActivationListener( JPH::PhysicsSystem &physicsSystem );

	void OnBodyActivated( const JPH::BodyID &inBodyID, JPH::uint64 inBodyUserData ) override;
	void OnBodyDeactivated( const JPH::BodyID &inBodyID, JPH::uint64 inBodyUserData ) override;

	void SetGameListener( IPhysicsObjectEvent *pListener );
	IPhysicsObjectEvent *GetGameListener() const;
	void SetPSIClock( const std::atomic< uint32 > *pClock ) { m_pPSIClock = pClock; }
	void FlushCallbacks();
	void DiscardCallbacks();
	void DiscardBodyCallbacks( const JPH::BodyID &body );

private:
	struct ActivationEvent
	{
		JPH::BodyID bodyID;
		JPH::uint64 bodyUserData = 0;
		bool activated = false;
	};

	JPH::PhysicsSystem &m_PhysicsSystem;
	const std::atomic< uint32 > *m_pPSIClock = nullptr;
	VJoltDeferredCallbackQueue< ActivationEvent, IPhysicsObjectEvent > m_Callbacks;
};

inline bool VJoltStateRecorderFailed( const JPH::StateRecorder &recorder )
{
	return static_cast< const JPH::StreamIn & >( recorder ).IsFailed()
		|| static_cast< const JPH::StreamOut & >( recorder ).IsFailed();
}

// StateRecorder implementation that saves to a fixed buffer
class VJoltStateRecorder final : public JPH::StateRecorder, public CUtlBuffer
{
public:
	using CUtlBuffer::CUtlBuffer;

	// StreamIn
	void ReadBytes( void* outData, size_t inNumBytes ) override
	{
		if ( inNumBytes > size_t( INT_MAX ) || inNumBytes > size_t( GetBytesRemaining() ) )
		{
			memset( outData, 0, inNumBytes );
			m_bFailed = true;
			m_bReadPastEnd = true;
			return;
		}
		Get( outData, static_cast<int>( inNumBytes ) );
	}

	bool IsEOF() const override { return m_bReadPastEnd; }

	// StreamOut
	void WriteBytes( const void* inData, size_t inNumBytes )
	{
		if ( inNumBytes > size_t( INT_MAX - TellPut() ) )
		{
			m_bFailed = true;
			return;
		}
		Put( inData, static_cast<int>( inNumBytes ) );
	}

	// Both
	bool IsFailed() const override { return m_bFailed || !IsValid(); }

private:
	bool m_bFailed = false;
	bool m_bReadPastEnd = false;
};

class JoltPhysicsEnvironment final : public IPhysicsEnvironment
{
public:
	JoltPhysicsEnvironment();
	~JoltPhysicsEnvironment() override;

	void SetDebugOverlay( CreateInterfaceFn debugOverlayFactory ) override;
	IVPhysicsDebugOverlay* GetDebugOverlay( void ) override;

	void SetGravity( const Vector& gravityVector ) override;
	void GetGravity( Vector* pGravityVector ) const override;

	void SetAirDensity( float density ) override;
	float GetAirDensity() const override;

	IPhysicsObject* CreatePolyObject( const CPhysCollide* pCollisionModel, int materialIndex, const Vector& position, const QAngle& angles, objectparams_t* pParams ) override;
	IPhysicsObject* CreatePolyObjectStatic( const CPhysCollide* pCollisionModel, int materialIndex, const Vector& position, const QAngle& angles, objectparams_t* pParams ) override;
	IPhysicsObject* CreateSphereObject( float radius, int materialIndex, const Vector& position, const QAngle& angles, objectparams_t* pParams, bool isStatic ) override;
	void DestroyObject( IPhysicsObject* ) override;

	IPhysicsFluidController* CreateFluidController( IPhysicsObject* pFluidObject, fluidparams_t* pParams ) override;
	void DestroyFluidController( IPhysicsFluidController* ) override;

	IPhysicsSpring* CreateSpring( IPhysicsObject* pObjectStart, IPhysicsObject* pObjectEnd, springparams_t* pParams ) override;
	void DestroySpring( IPhysicsSpring* ) override;

	IPhysicsConstraint* CreateRagdollConstraint( IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup, const constraint_ragdollparams_t& ragdoll ) override;
	IPhysicsConstraint* CreateHingeConstraint( IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup, const constraint_hingeparams_t& hinge ) override;
	IPhysicsConstraint* CreateFixedConstraint( IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup, const constraint_fixedparams_t& fixed ) override;
	IPhysicsConstraint* CreateSlidingConstraint( IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup, const constraint_slidingparams_t& sliding ) override;
	IPhysicsConstraint* CreateBallsocketConstraint( IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup, const constraint_ballsocketparams_t& ballsocket ) override;
	IPhysicsConstraint* CreatePulleyConstraint( IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup, const constraint_pulleyparams_t& pulley ) override;
	IPhysicsConstraint* CreateLengthConstraint( IPhysicsObject* pReferenceObject, IPhysicsObject* pAttachedObject, IPhysicsConstraintGroup* pGroup, const constraint_lengthparams_t& length ) override;

	void DestroyConstraint( IPhysicsConstraint* ) override;

	IPhysicsConstraintGroup* CreateConstraintGroup( const constraint_groupparams_t& groupParams ) override;
	void DestroyConstraintGroup( IPhysicsConstraintGroup* pGroup ) override;

	IPhysicsShadowController* CreateShadowController( IPhysicsObject* pObject, bool allowTranslation, bool allowRotation ) override;
	void DestroyShadowController( IPhysicsShadowController* ) override;

	IPhysicsPlayerController* CreatePlayerController( IPhysicsObject* pObject ) override;
	void DestroyPlayerController( IPhysicsPlayerController* ) override;

	IPhysicsMotionController* CreateMotionController( IMotionEvent* pHandler ) override;
	void DestroyMotionController( IPhysicsMotionController* pController ) override;

	IPhysicsVehicleController* CreateVehicleController( IPhysicsObject* pVehicleBodyObject, const vehicleparams_t& params, unsigned int nVehicleType, IPhysicsGameTrace* pGameTrace ) override;
	IPhysicsVehicleController* CreateBridgeVehicleController( IPhysicsObject* pVehicleBodyObject, const vehicleparams_t& params, const JoltVehicleBridgeConfig_t& bridgeConfig );
	bool IsVehicleControlledObject( const JoltPhysicsObject *pObject ) const;
	void DestroyVehicleController( IPhysicsVehicleController* ) override;

	void SetCollisionSolver( IPhysicsCollisionSolver* pSolver ) override;

	void Simulate( float deltaTime ) override;
	bool IsInSimulation() const override;

	float GetSimulationTimestep() const override;
	void SetSimulationTimestep( float timestep ) override;

	float GetSimulationTime() const override;
	void ResetSimulationClock() override;
	float GetNextFrameTime() const override;

	void SetCollisionEventHandler( IPhysicsCollisionEvent* pCollisionEvents ) override;
	void SetObjectEventHandler( IPhysicsObjectEvent* pObjectEvents ) override;
	virtual void SetConstraintEventHandler( IPhysicsConstraintEvent* pConstraintEvents ) override;

	void SetQuickDelete( bool bQuick ) override;

	int GetActiveObjectCount() const override;
	void GetActiveObjects( IPhysicsObject** pOutputObjectList ) const override;
	const IPhysicsObject** GetObjectList( int* pOutputObjectCount ) const override;
	bool TransferObject( IPhysicsObject* pObject, IPhysicsEnvironment* pDestinationEnvironment ) override;

	void CleanupDeleteList() override;
	void EnableDeleteQueue( bool enable ) override;

#if defined( GAME_GMOD_64X )
	void PreSave( const physprerestoreparams_t &params ) override;
#endif
	bool Save( const physsaveparams_t& params ) override;
#if defined( GAME_GMOD_64X )
	void PostSave() override;
#endif
	void PreRestore( const physprerestoreparams_t& params ) override;
	bool Restore( const physrestoreparams_t& params ) override;
	void PostRestore() override;

	bool IsCollisionModelUsed( CPhysCollide* pCollide ) const override;

	void TraceRay( const Ray_t& ray, unsigned int fMask, IPhysicsTraceFilter* pTraceFilter, trace_t* pTrace ) override;
	void SweepCollideable( const CPhysCollide* pCollide, const Vector& vecAbsStart, const Vector& vecAbsEnd,
		const QAngle& vecAngles, unsigned int fMask, IPhysicsTraceFilter* pTraceFilter, trace_t* pTrace ) override;

	void GetPerformanceSettings( physics_performanceparams_t* pOutput ) const override;
	void SetPerformanceSettings( const physics_performanceparams_t* pSettings ) override;

	// physics params related
	inline float MaxVelocity() const;
	inline float MaxAngularVelocity() const;
	// most likely will go unused
	inline int MaxCollisionsPerObjectPerTimestep() const;
	inline int MaxCollisionChecksPerTimestep() const;
	inline float LookAheadTimeObjectsVsWorld() const;
	inline float LookAheadTimeObjectsVsObject() const;
	inline float MinFrictionMass() const;
	inline float MaxFrictionMass() const;

	void ReadStats( physics_stats_t* pOutput ) override;
	void ClearStats() override;

	unsigned int GetObjectSerializeSize( IPhysicsObject* pObject ) const override;
	void SerializeObjectToBuffer( IPhysicsObject* pObject, unsigned char* pBuffer, unsigned int bufferSize ) override;
	IPhysicsObject* UnserializeObjectFromBuffer( void* pGameData, unsigned char* pBuffer, unsigned int bufferSize, bool enableCollisions ) override;

	void EnableConstraintNotify( bool bEnable ) override;
	void DebugCheckContacts() override;

	void SetAlternateGravity( const Vector& gravityVector ) override_asw;
	void GetAlternateGravity( Vector* pGravityVector ) const override_asw;

	float GetDeltaFrameTime( int maxTicks ) const override_asw;
	void ForceObjectsToSleep( IPhysicsObject** pList, int listCount ) override_asw;

	void SetPredicted( bool bPredicted ) override_portal2;
	bool IsPredicted() override_portal2;
	void SetPredictionCommandNum( int iCommandNum ) override_portal2;
	int GetPredictionCommandNum() override_portal2;
	void DoneReferencingPreviousCommands( int iCommandNum ) override_portal2;
	void RestorePredictedSimulation() override_portal2;

	void DestroyCollideOnDeadObjectFlush( CPhysCollide* ) override_portal2;

#if GAME_GMOD
	void SetGModObjectEventHandler( IGModPhysicsObjectEvent *pGModObjectEvent ) override;
#endif

public:
	JPH::PhysicsSystem* GetPhysicsSystem() { return &m_PhysicsSystem; }
	bool UsesLinearCast() const { return m_bUseLinearCast; }
	bool HasMotionControllerForObject( JoltPhysicsObject *object ) const;

	JoltPhysicsDragController* GetDragController() { return m_pDragController; }

	JoltPhysicsContactListener* GetContactListener() { return &m_ContactListener; }
	float GetContactFrictionScale() const { return m_flContactFrictionScale; }

	IPhysicsConstraintEvent* GetConstraintEvents() { return m_pConstraintListener; }

	IPhysicsObjectEvent* GetObjectEvents() { return m_ActivationListener.GetGameListener(); }

#if GAME_GMOD
	IGModPhysicsObjectEvent* GetGModObjectEvents() { return m_pGModObjectEvents; }
#endif

	void NotifyConstraintDisabled( JoltPhysicsConstraint* pConstraint );

	void RegisterConstraint( JoltPhysicsConstraint *pConstraint );
	void UnregisterConstraint( JoltPhysicsConstraint *pConstraint );
	void RegisterBreakableConstraint( JoltPhysicsConstraint *pConstraint );
	void UnregisterBreakableConstraint( JoltPhysicsConstraint *pConstraint );

	void AddDirtyStaticBody( const JPH::BodyID &id );
	void RemoveDirtyStaticBody( const JPH::BodyID &id );
	void QueuePendingVelocityCommit( JoltPhysicsObject *pObject );
	void RemovePendingVelocityCommit( JoltPhysicsObject *pObject );
	// IVP re-sorts a simulation unit when a controller is attached to a core.
	// SetPriority alone deliberately does not trigger a re-sort.
	void NotifyPhysicsControllerAttached( IJoltPhysicsController *pController );

	bool RunContactCallback( JoltPhysicsObject *pObject1, JoltPhysicsObject *pObject2,
		JPH::RVec3Arg centerOfMass1, JPH::RVec3Arg centerOfMass2,
		const std::function<bool()> &callback );
	bool IsInContactCallback() const;
	bool RunContactBatch( const std::function<bool()> &batch );
	bool HasSynchronousContactCallbacks() const { return m_pContactCallbackJobs != nullptr; }
	bool IsProfilingContactCallbacks() const { return m_pContactCallbackJobs && m_pContactCallbackJobs->IsProfiling(); }
	void RecordContactCallbackTimings( double gameMs, double responseMs ) { m_pContactCallbackJobs->RecordEventTimings( gameMs, responseMs ); }
	JPH::RVec3 GetObjectResponseCenterOfMass( const JoltPhysicsObject *pObject ) const;
	bool DeferPhysicsMutation( const std::function<void()> &mutation );
	void NotifyCallbackBodyPropertiesChanged( JoltPhysicsObject *object );
	void RefreshCallbackBodyProperties();
	bool DeferObjectMutation( JoltPhysicsObject *pObject, const std::function<void()> &mutation, bool velocityOnly = false );
	const JPH::BodyLockInterface &GetBodyLockInterfaceForRead() const;
	void GetSourceBodiesForRead( JPH::BodyIDVector &bodies ) const;
	JPH::Constraints GetConstraintsForRead() const;

private:
	std::vector<JoltPhysicsObject *> m_CallbackChangedBodies;

	void RemoveBodyAndDeleteObject( JoltPhysicsObject* pObject );
	JPH::Body *CreateBodyForObject( const JPH::BodyCreationSettings &settings );
	void DeleteDeadObjects();
	void DestroyEnvironmentObjects();
	void CommitPendingVelocityChanges();
	void RegisterPhysicsController( IJoltPhysicsController *pController );
	void UnregisterPhysicsController( IJoltPhysicsController *pController );
	void RebuildPhysicsControllerOrder();

	template <typename T>
	void AddPhysicsSaveRestorePointer( uintp oldPtr, T* newPtr );

	template <typename T>
	T* LookupPhysicsSaveRestorePointer( uintp oldPtr );

	bool WritePhysicsReference( JPH::StateRecorder &recorder, const void *pObject );
	template <typename T>
	bool ReadPhysicsReference( JPH::StateRecorder &recorder, T *&pObject, PhysInterfaceId_t type );
	bool SavePhysicsState( const physsaveparams_t &params, JPH::StateRecorder &recorder );
	bool RestorePhysicsState( const physrestoreparams_t &params, JPH::StateRecorder &recorder, bool extended );
	void ForgetRestoredPointer( const void *pObject );

	bool m_bSimulating = false;
	VJoltOwnerCallbackJobs *m_pContactCallbackJobs = nullptr;
	JoltPhysicsObject *m_pCollisionEndpoint1 = nullptr;
	JoltPhysicsObject *m_pCollisionEndpoint2 = nullptr;
	JPH::RVec3 m_vCollisionCenterOfMass1 = JPH::RVec3::sZero();
	JPH::RVec3 m_vCollisionCenterOfMass2 = JPH::RVec3::sZero();
	std::vector<std::function<void()>> m_DeferredPhysicsMutations;
	bool m_bEnableDeleteQueue = false;
	bool m_bDeletingDeadObjects = false;
	bool m_bWakeObjectsOnConstraintDeletion = false;
	bool m_bOptimizedBroadPhase = false;
	bool m_bUseLinearCast = false;
	bool m_bUseEnhancedEdgeDetection = true;
	float m_flStepTime = 1.0f / 64.0f;
	float m_flContactFrictionScale = 1.0f;
	VJoltSimulationClock m_SimulationClock;
	// Accumulates external time until the configured Source/IVP PSI is due.
	// This is deliberately per environment: multiple maps/environments must not share time debt.
	double m_flPhysicsAccumulator = 0.0;

	// Owned. Also stored in m_pPhysicsControllers so OnPreSimulate is called.
	JoltPhysicsDragController* m_pDragController = nullptr;

	// Applies springparams_t::relativeDamping to all live springs.
	JoltPhysicsSpringDamper* m_pSpringDamper = nullptr;

	static JoltBroadPhaseLayerInterface s_BroadPhaseLayerInterface;
	static JoltObjectVsBroadPhaseLayerFilter s_BroadPhaseFilter;
	static JoltObjectLayerPairFilter s_LayerPairFilter;

	// For GetObjectList
	mutable JPH::BodyIDVector m_CachedBodies;
	mutable std::vector< const IPhysicsObject * > m_CachedObjects;

	// For GetActiveObjectCount and GetActiveObjects
	mutable JPH::BodyIDVector m_CachedActiveBodies;

	JPH::PhysicsSystem m_PhysicsSystem;
	VJoltConstraintWakePropagator m_ConstraintWakePropagator;
	VJoltPhysicsUpdateErrorTracker m_PhysicsUpdateErrors;

	// A vector of objects that were awake, and changed their
	// motion type from Dynamic -> Static, so that they can be
	// retrieved in GetActiveObjects, and have their visuals updated.
	// If we don't do this, objects that get moved, woken, and their
	// movement type changed to static will not get their transforms
	// updated on the game side.
	mutable JPH::BodyIDVector m_DirtyStaticBodies;

	std::vector< JoltPhysicsObject * > m_pDeadObjects;
	std::vector< JoltPhysicsObject * > m_pPendingVelocityObjects;
	std::vector< JoltPhysicsObject * > m_pPendingVelocityWorkObjects;
	std::vector< JoltPhysicsConstraint * > m_pDeadConstraints;
	std::vector< CPhysCollide * > m_pDeadObjectCollides;

	// All live constraints are retained for object transfer and public bookkeeping.
	std::vector< JoltPhysicsConstraint * > m_pConstraints;
	// Source wraps only constraints with real force/torque break limits in its breakable path.
	// Keep the same sparse set so ordinary welds, ropes and vehicle constraints do not participate
	// in a per-frame break scan.
	std::vector< JoltPhysicsConstraint * > m_pBreakableConstraints;

	std::vector< IJoltPhysicsController * > m_pPhysicsControllers;
	std::vector< IJoltPhysicsController * > m_pOrderedPhysicsControllers;
	bool m_bPhysicsControllerOrderDirty = true;

	std::unordered_map< uintp, void * > m_SaveRestorePointerMap;
	std::vector< std::pair< JPH::BodyID, bool > > m_RestoredBodyActivation;

#if defined( GAME_GMOD_64X )
	VJoltSaveRestoreIdTable m_GModSaveRestoreIds;
#endif

	// The physics system that simulates the world
	// The debug overlay to render with (if it was ever passed to us)
	IVJoltDebugOverlay *m_pDebugOverlay = nullptr;

	JoltPhysicsActivationListener m_ActivationListener;
	JoltPhysicsContactListener m_ContactListener;
	IPhysicsConstraintEvent *m_pConstraintListener = nullptr;

#if GAME_GMOD
	IGModPhysicsObjectEvent *m_pGModObjectEvents = nullptr;
#endif

	bool m_EnableConstraintNotify = false;

	mutable bool m_bActiveObjectCountFirst = true;

	physics_performanceparams_t m_PerformanceParams;
};
