#pragma once

class JoltPhysicsObject;
class JoltPhysicsEnvironment;

void VJoltPathBeforeSimulation(JoltPhysicsEnvironment *environment);
void VJoltPathReleaseObject(JoltPhysicsObject *object);
void VJoltPathEnvironmentDestroyed(JoltPhysicsEnvironment *environment);
