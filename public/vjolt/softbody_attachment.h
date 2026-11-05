#pragma once

#include "softbody.h"
class IPhysicsObject;

// Optional extension. Calls run on the environment owner thread outside Simulate.
// Compliant point joints: a movable soft-body vertex exchanges impulses with a
// rigid body at a local anchor. Existing soft-body interface vtables are unchanged.
#define VJOLT_SOFTBODY_ATTACHMENT_VERSION "VJoltSoftBodyAttachment001"
inline constexpr uint32_t kVJoltSoftBodyAttachmentVersion = 1;

class IVJoltSoftBodyAttachment
{
public:
	virtual uint32_t GetVersion() const = 0;
	// Replaces a vertex's attachment atomically. The point is in Source world
	// units; its position relative to the rigid body is retained as it moves.
	virtual bool AttachVertex( VJoltSoftBodyHandle handle, uint32_t vertex,
		IPhysicsObject *target, const VJoltSoftBodyVector &worldPoint ) = 0;
	// Restores the vertex's pre-attachment pin state. Missing attachments fail.
	virtual bool DetachVertex( VJoltSoftBodyHandle handle, uint32_t vertex ) = 0;
	virtual bool IsVertexAttached( VJoltSoftBodyHandle handle, uint32_t vertex ) const = 0;
};
