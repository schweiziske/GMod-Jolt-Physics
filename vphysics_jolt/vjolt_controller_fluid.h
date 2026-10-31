
#pragma once

#include "vjolt_internal_listeners.h"
#include <Jolt/Core/UnorderedMap.h>

class JoltPhysicsFluidController final : public IPhysicsFluidController, public IJoltObjectDestroyedListener, public IJoltPhysicsController
{
public:
	JoltPhysicsFluidController( JPH::PhysicsSystem *pPhysicsSystem, JoltPhysicsObject *pFluidObject, const fluidparams_t *pParams );
	~JoltPhysicsFluidController() override;

	void	SetGameData( void *pGameData ) override;
	void *	GetGameData() const override;

	void	GetSurfacePlane( Vector *pNormal, float *pDist ) const override;
	float	GetDensity() const override;
	void	WakeAllSleepingObjects() override;
	int		GetContents() const override;

public:

	// IJoltObjectDestroyedListener
	void OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject ) override;
	// IJoltPhysicsController
	int GetSimulationPriority() const override { return VJoltControllerPriority::ForceFields; }
	void OnPreSimulate( float flDeltaTime ) override;

	// A transferred object keeps its Source wrapper alive, so it will not emit the normal
	// destroyed-listener callback. Explicitly remove any source-environment fluid cache state.
	void RemoveObjectForTransfer( JoltPhysicsObject *pObject );

private:
	struct FluidObjectState
	{
		JoltPhysicsObject *pObject = nullptr;
		JPH::Vec3 vRelativeCurrent = JPH::Vec3::sZero();
		float flVisibleAreaOld = 0.0f;
		float flVisibleArea = 0.0f;
		const JPH::Shape *pFallbackTriangleShape = nullptr;
		std::vector<JPH::Float3> fallbackTriangles;
		bool bSeenThisFrame = false;
	};

	cplane_t GetSurfacePlane() const;
	Vector GetCurrentVelocity() const;
	FluidObjectState &FindOrCreateObjectState( JoltPhysicsObject *pObject );
	void ClearCachedObjectsInShape();

	JPH::PhysicsSystem *				m_pPhysicsSystem = nullptr;
	JoltPhysicsObject *					m_pFluidObject = nullptr;
	std::vector<JoltPhysicsObject *>	m_ObjectsInShape;
	JPH::UnorderedMap<JoltPhysicsObject *, FluidObjectState> m_ObjectStates;

	fluidparams_t						m_Params = {};
	cplane_t							m_LocalPlane = {};
	Vector								m_LocalCurrentVelocity = {};
	float								m_flDensity = 0.0f;
};
