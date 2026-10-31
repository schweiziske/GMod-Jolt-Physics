
#pragma once

class JoltPhysicsObject;

namespace VJoltControllerPriority
{
	// Match IVP_CONTROLLER_PRIORITY. IVP sorts ascending, then executes the
	// controller list in reverse, so larger values run first.
	inline constexpr int ConstraintsMin = 400;
	inline constexpr int ConstraintsMax = 410;
	inline constexpr int Motion = 500;
	inline constexpr int Player = 501;
	inline constexpr int ForceFields = 1600;
	inline constexpr int HighMotion = ForceFields + 1;
}

abstract_class IJoltPhysicsController
{
public:
	virtual ~IJoltPhysicsController() {}

	virtual int GetSimulationPriority() const { return VJoltControllerPriority::Motion; }

	// Called before the simulation is run
	virtual void OnPreSimulate( float flDeltaTime ) {};
	// Called after the simulation is run
	virtual void OnPostSimulate( float flDeltaTime ) {};
};

abstract_class IJoltObjectDestroyedListener
{
public:
	virtual ~IJoltObjectDestroyedListener() {}

	// Called whenever a physics object is destroyed
	virtual void OnJoltPhysicsObjectDestroyed( JoltPhysicsObject *pObject ) = 0;
};
