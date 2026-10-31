// Shared pre-step/native constraint wake propagation. Connectivity and invalidation
// belong to Jolt: the adapter must not copy and rebuild the same graph every PSI.
#pragma once

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <cstdint>

class VJoltConstraintWakePropagator final
{
public:
	struct Statistics
	{
		std::uint32_t scannedConstraints = 0;
		std::uint32_t eligibleConstraints = 0;
		std::uint32_t dynamicBodies = 0;
		std::uint32_t mixedComponents = 0;
		std::uint32_t activationBatchBodies = 0;
		std::uint32_t newlyActivatedBodies = 0;
	};

	Statistics Propagate( JPH::PhysicsSystem &physicsSystem )
	{
		const auto stats = physicsSystem.PropagateConstraintWake();
		return { stats.mScannedConstraints, stats.mEligibleConstraints, stats.mDynamicBodies,
			stats.mMixedComponents, stats.mActivationBatchBodies, stats.mNewlyActivatedBodies };
	}
};
