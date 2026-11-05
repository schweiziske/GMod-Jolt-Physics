#pragma once

class JoltPhysicsEnvironment;

void VJoltSoftBodyBeforeSimulation( JoltPhysicsEnvironment *environment );
void VJoltSoftBodySolveAttachments( JoltPhysicsEnvironment *environment, float deltaTime );
void VJoltSoftBodyAfterSimulation( JoltPhysicsEnvironment *environment, float deltaTime );
void VJoltSoftBodyEnvironmentDestroyed( JoltPhysicsEnvironment *environment );
