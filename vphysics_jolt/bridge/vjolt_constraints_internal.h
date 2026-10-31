#pragma once

#include <vjolt/constraints.h>

class JoltPhysicsObject;
class JoltPhysicsEnvironment;

void VJoltConstraintsBeforeSimulation( JoltPhysicsEnvironment *environment );
void VJoltConstraintsReleaseObject( JoltPhysicsObject *object );
void VJoltConstraintsEnvironmentDestroyed( JoltPhysicsEnvironment *environment );
