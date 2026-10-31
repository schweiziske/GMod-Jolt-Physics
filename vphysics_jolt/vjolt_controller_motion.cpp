
#include "cbase.h"

#include "vjolt_controller_motion.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------

JoltPhysicsMotionController::JoltPhysicsMotionController(
	JoltPhysicsEnvironment *pEnvironment, IMotionEvent *pHandler )
	: m_pEnvironment( pEnvironment )
	, m_pMotionEvent( pHandler )
{
}

JoltPhysicsMotionController::~JoltPhysicsMotionController()
{
	for ( JoltPhysicsObject *pObject : m_pObjects )
		pObject->RemoveDestroyedListener( this );
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsMotionController::SetEventHandler( IMotionEvent *pHandler )
{
	m_pMotionEvent = pHandler;
}

void JoltPhysicsMotionController::AttachObject( IPhysicsObject *pObject, bool bCheckIfAlreadyAttached )
{
	if ( !pObject || pObject->IsStatic() || m_bDeleteRequested
		|| ( pObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) )
		return;

	JoltPhysicsObject *pPhysicsObject = static_cast< JoltPhysicsObject * >( pObject );
	if ( bCheckIfAlreadyAttached && VectorContains( m_pObjects, pPhysicsObject ) )
		return;

	const bool bFirstAttachment = m_pObjects.empty();
	// A newly attached game controller takes ownership; never keep our grab motor too.
	pPhysicsObject->AddDestroyedListener( this );
	m_pObjects.push_back( pPhysicsObject );
	m_bObjectListDirty = true;
	if ( bFirstAttachment && m_pEnvironment )
		m_pEnvironment->NotifyPhysicsControllerAttached( this );
}

void JoltPhysicsMotionController::DetachObject( IPhysicsObject *pObject )
{
	if ( !pObject )
		return;

	JoltPhysicsObject *pPhysicsObject = static_cast< JoltPhysicsObject * >( pObject );
	Erase( m_pObjects, pPhysicsObject );
	pPhysicsObject->RemoveDestroyedListener( this );
	RemoveFromSimulation( pPhysicsObject );
}

//-------------------------------------------------------------------------------------------------

int JoltPhysicsMotionController::CountObjects( void )
{
	return int( m_pObjects.size() );
}

void JoltPhysicsMotionController::GetObjects( IPhysicsObject **pObjectList )
{
	for ( size_t i = 0; i < m_pObjects.size(); i++ )
		pObjectList[ i ] = m_pObjects[ i ];
}

void JoltPhysicsMotionController::ClearObjects( void )
{
	for ( JoltPhysicsObject *pObject : m_pObjects )
		pObject->RemoveDestroyedListener( this );
	m_pObjects.clear();
	m_bObjectListDirty = true;
	if ( m_bSimulating )
		std::fill( m_pSimulationObjects.begin(), m_pSimulationObjects.end(), nullptr );
}

void JoltPhysicsMotionController::WakeObjects( void )
{
	for ( JoltPhysicsObject *pObject : m_pObjects )
		pObject->Wake();
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsMotionController::SetPriority( priority_t priority )
{
	int simulationPriority = VJoltControllerPriority::Motion;
	switch ( priority )
	{
		case LOW_PRIORITY:
			simulationPriority = VJoltControllerPriority::ConstraintsMin;
			break;
		default:
		case MEDIUM_PRIORITY:
			simulationPriority = VJoltControllerPriority::Motion;
			break;
		case HIGH_PRIORITY:
			simulationPriority = VJoltControllerPriority::HighMotion;
			break;
	}

	m_nSimulationPriority = simulationPriority;
}

//-------------------------------------------------------------------------------------------------

void JoltPhysicsMotionController::OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject )
{
	JoltPhysicsObject *pPhysicsObject = static_cast< JoltPhysicsObject * >( pObject );
	Erase( m_pObjects, pPhysicsObject );
	RemoveFromSimulation( pPhysicsObject );
}

void JoltPhysicsMotionController::RemoveFromSimulation( JoltPhysicsObject *pObject )
{
	m_bObjectListDirty = true;
	// Tombstone removed entries without invalidating the active iterators. Even if
	// a callback reattaches this object, that new attachment waits until the next PSI.
	if ( m_bSimulating )
		std::replace( m_pSimulationObjects.begin(), m_pSimulationObjects.end(), pObject,
			static_cast<JoltPhysicsObject *>( nullptr ) );
}

void JoltPhysicsMotionController::Destroy()
{
	// The environment unregisters us immediately. Keep this and its iteration
	// storage alive until a callback which destroys its own controller returns.
	if ( m_bSimulating )
		m_bDeleteRequested = true;
	else
		delete this;
}

void JoltPhysicsMotionController::OnPreSimulate( float flDeltaTime )
{
	if ( !m_pMotionEvent )
		return;

	if ( m_bObjectListDirty )
	{
		m_pSimulationObjects = m_pObjects;
		m_bObjectListDirty = false;
	}

	m_bSimulating = true;
	for ( JoltPhysicsObject *pObject : m_pSimulationObjects )
	{
		if ( !m_pMotionEvent )
			break;
		if ( !pObject || ( pObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) || !pObject->IsMoveable() )
			continue;

		Vector speed = vec3_origin;
		AngularImpulse rot = vec3_origin;
		IMotionEvent::simresult_e simResult = m_pMotionEvent->Simulate( this, pObject, flDeltaTime, speed, rot );
		if ( m_bDeleteRequested )
			break;
		if ( simResult == IMotionEvent::SIM_NOTHING || ( pObject->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE ) )
			continue;

		speed *= flDeltaTime;
		rot *= flDeltaTime;

		Vector worldLinear = speed;
		if ( simResult == IMotionEvent::SIM_LOCAL_ACCELERATION || simResult == IMotionEvent::SIM_LOCAL_FORCE )
			pObject->LocalToWorldVector( &worldLinear, speed );

		switch ( simResult )
		{
			case IMotionEvent::SIM_GLOBAL_ACCELERATION:
			case IMotionEvent::SIM_LOCAL_ACCELERATION:
				pObject->AddVelocity( &worldLinear, &rot );
				break;

			case IMotionEvent::SIM_GLOBAL_FORCE:
			case IMotionEvent::SIM_LOCAL_FORCE:
			{
				Vector worldAngular;
				pObject->LocalToWorldVector( &worldAngular, rot );
				pObject->ApplyForceCenter( worldLinear );
				pObject->ApplyTorqueCenter( worldAngular );
				break;
			}

			default:
				Log_Warning( LOG_VJolt, "Invalid motion event\n" );
				break;
		}
	}
	m_bSimulating = false;
	if ( m_bDeleteRequested )
		delete this;
}
