#pragma once

#include <vjolt/gravity.h>

class JoltPhysicsEnvironment;
class JoltPhysicsObject;

// Backend lifecycle and simulation hooks; not part of the public DLL interface.
void VJoltGravityBeforeSimulation(JoltPhysicsEnvironment *);
void VJoltGravityObjectDestroyed(JoltPhysicsObject *);
void VJoltGravityEnvironmentDestroyed(JoltPhysicsEnvironment *);
bool VJoltGravityEnableObject(JoltPhysicsObject *, bool enabled);
bool VJoltGravityIsObjectEnabled(const JoltPhysicsObject *, bool &enabled);
void VJoltGravityManageVehicle(JoltPhysicsObject *, bool managed);
VJoltGravityVector VJoltGravityForObject(JoltPhysicsObject *);
void VJoltGravityWorldChanged(JoltPhysicsEnvironment *);
void VJoltGravityRefreshObject(JoltPhysicsObject *);
