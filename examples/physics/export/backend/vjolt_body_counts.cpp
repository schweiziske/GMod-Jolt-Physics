// Compile into vphysics_jolt, alongside the other bridge implementations.
#include "cbase.h"
#include "vjolt_environment.h"
#include "vjolt_interface.h"
#include <vjolt/body_counts.h>

#include "tier0/memdbgon.h"

namespace
{
class BodyCountsBridge final : public IVJoltBodyCounts
{
public:
    uint32_t GetVersion() const override { return kVJoltBodyCountsVersion; }

    bool Read(int32_t index, VJoltBodyCounts &out) const override
    {
        if (index < 0 || out.size != sizeof(out)) return false;
        auto *environment = static_cast<JoltPhysicsEnvironment *>(
            JoltPhysicsInterface::GetInstance().GetActiveEnvironmentByIndex(index));
        if (!environment || environment->IsInSimulation() || environment->IsInContactCallback())
            return false;

        // The existing environment owns Jolt. This is where new Jolt calls belong.
        const auto *physics = environment->GetPhysicsSystem();
        VJoltBodyCounts counts;
        counts.total = physics->GetNumBodies();
        counts.activeRigid = physics->GetNumActiveBodies(JPH::EBodyType::RigidBody);
        counts.activeSoft = physics->GetNumActiveBodies(JPH::EBodyType::SoftBody);
        out = counts;
        return true;
    }
};

BodyCountsBridge g_BodyCounts;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR(
    BodyCountsBridge, IVJoltBodyCounts, VJOLT_BODY_COUNTS_VERSION, g_BodyCounts);
}
