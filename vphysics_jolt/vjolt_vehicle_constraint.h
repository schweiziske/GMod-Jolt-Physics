#pragma once

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>

// Source can pin a vehicle or temporarily drive its chassis kinematically. Jolt
// vehicles assume their chassis can be activated when a moving body reaches a
// wheel. A pinned chassis has no active-body index, so admitting that constraint
// to an island would index UINT32_MAX. Keep wheel queries/state alive, but only
// solve the suspension when the chassis is dynamic; unpinning resumes normally.
class JoltPinnableVehicleConstraint final : public JPH::VehicleConstraint
{
public:
	using JPH::VehicleConstraint::VehicleConstraint;
	bool IsActive() const override
	{
		return GetVehicleBody()->IsDynamic() && JPH::VehicleConstraint::IsActive();
	}
};
