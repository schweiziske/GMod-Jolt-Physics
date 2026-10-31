#pragma once

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyFilter.h>

// Suspension constraints operate on rigid bodies. A soft surface can still
// collide with the chassis through the soft-body solver, but cannot be a wheel support.
class JoltRigidBodyFilter final : public JPH::BodyFilter
{
public:
	void IgnoreBody( const JPH::BodyID &body )
	{
		m_IgnoredBody = body;
	}

	bool ShouldCollide( const JPH::BodyID &body ) const override
	{
		return body != m_IgnoredBody;
	}

	bool ShouldCollideLocked( const JPH::Body &body ) const override
	{
		return body.IsRigidBody();
	}

private:
	JPH::BodyID m_IgnoredBody;
};
