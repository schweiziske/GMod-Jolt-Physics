
#pragma once

#include "vjolt_object.h"
#include "vjolt_environment.h"

class JoltPhysicsMotionController : public IPhysicsMotionController, public IJoltObjectDestroyedListener, public IJoltPhysicsController
{
public:
	JoltPhysicsMotionController( JoltPhysicsEnvironment *pEnvironment, IMotionEvent *pHandler );
	~JoltPhysicsMotionController() override;

	void SetEventHandler( IMotionEvent *handler ) override;
	void AttachObject( IPhysicsObject *pObject, bool checkIfAlreadyAttached ) override;
	void DetachObject( IPhysicsObject *pObject ) override;

	int CountObjects( void ) override;
	void GetObjects( IPhysicsObject **pObjectList ) override;
	void ClearObjects( void ) override;
	void WakeObjects( void ) override;
	bool ContainsObject( JoltPhysicsObject *object ) const
	{
		return std::find( m_pObjects.begin(), m_pObjects.end(), object ) != m_pObjects.end();
	}

	void SetPriority( priority_t priority ) override;
	priority_t GetPriority() const
	{
		return m_nSimulationPriority == VJoltControllerPriority::ConstraintsMin ? LOW_PRIORITY
			: m_nSimulationPriority == VJoltControllerPriority::HighMotion ? HIGH_PRIORITY : MEDIUM_PRIORITY;
	}

public:

	void OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject ) override;
	int GetSimulationPriority() const override { return m_nSimulationPriority; }
	void OnPreSimulate( float flDeltaTime ) override;
	void Destroy();

private:
	void RemoveFromSimulation( JoltPhysicsObject *pObject );

	JoltPhysicsEnvironment *m_pEnvironment = nullptr;
	IMotionEvent *m_pMotionEvent = nullptr;
	int m_nSimulationPriority = VJoltControllerPriority::Motion;

	std::vector< JoltPhysicsObject * > m_pObjects;
	// Rebuilt only when membership changes. Callbacks may edit m_pObjects, but must
	// not reallocate or erase from the list currently being traversed.
	std::vector< JoltPhysicsObject * > m_pSimulationObjects;
	bool m_bObjectListDirty = true;
	bool m_bSimulating = false;
	bool m_bDeleteRequested = false;
};
