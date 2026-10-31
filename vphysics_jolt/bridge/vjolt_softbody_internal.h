#pragma once

class JoltPhysicsEnvironment;

void VJoltSoftBodyBeforeSimulation( JoltPhysicsEnvironment *environment );
void VJoltSoftBodyAfterSimulation( JoltPhysicsEnvironment *environment, float deltaTime );
void VJoltSoftBodyEnvironmentDestroyed( JoltPhysicsEnvironment *environment );
