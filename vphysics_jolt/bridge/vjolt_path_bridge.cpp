#include "cbase.h"
#include "bridge/vjolt_path_internal.h"
#include "bridge/vjolt_path_spline.h"
#include "vjolt_environment.h"
#include "vjolt_interface.h"
#include "vjolt_internal_listeners.h"
#include "vjolt_object.h"
#include <vjolt/path.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Physics/Constraints/PathConstraint.h>
#include <array>
#include <limits>
#include <memory>
#include <thread>
#include <unordered_map>

#include "tier0/memdbgon.h"

JPH_IMPLEMENT_RTTI_VIRTUAL(VJoltPathSpline)
{
    JPH_ADD_BASE_CLASS(VJoltPathSpline, JPH::PathConstraintPath)
}

#if GAME_GMOD
extern bool IsValidPhyiscsObject(IPhysicsObject *object);
#endif

namespace
{
using Error = VJoltPathError;
bool Result(Error *out, Error error) { if (out) *out = error; return error == Error::None; }
bool Finite(float value, float max) { return std::isfinite(value) && std::abs(value) <= max; }
bool ValidMotor(const VJoltPathMotor &s)
{
    return s.size == sizeof(s) && s.mode <= VJoltPathMotorMode::Position
        && Finite(s.speed, 32767) && Finite(s.target, 1) && s.target >= 0
        && Finite(s.maxForce, 1.0e9f) && s.maxForce >= 0 && Finite(s.friction, 1.0e9f) && s.friction >= 0
        && Finite(s.frequency, 20) && s.frequency >= .1f && Finite(s.damping, 5) && s.damping >= 0;
}
bool Live(IPhysicsObject *object)
{
    if (!object) return false;
#if GAME_GMOD
    return IsValidPhyiscsObject(object);
#else
    auto &physics = JoltPhysicsInterface::GetInstance();
    for (int i = 0; auto *env = physics.GetActiveEnvironmentByIndex(i); ++i)
    {
        int count = 0; const auto **objects = env->GetObjectList(&count);
        for (int j = 0; j < count; ++j) if (objects[j] == object) return true;
    }
    return false;
#endif
}
bool Supported(JoltPhysicsObject *object, bool follower)
{
    return (!follower || !object->IsStatic()) && !object->IsTrigger() && !object->IsFluid()
        && !object->IsControlledByGame() && !object->GetShadowController()
        && !(object->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE)
        && !object->GetJoltEnvironment()->IsVehicleControlledObject(object);
}
VJoltConstraintVector SourceVector(JPH::Vec3Arg v, bool direction = false)
{
    Vector result = JoltToSource::Distance(v);
    if (direction) result.NormalizeInPlace();
    return {result.x, result.y, result.z};
}
class PathBridge;
struct Track final : IJoltObjectDestroyedListener
{
    PathBridge *owner = nullptr;
    VJoltPathHandle handle = 0;
    JoltPhysicsEnvironment *environment = nullptr;
    std::array<JoltPhysicsObject *, 2> objects{}; // follower, optional reference
    std::array<JPH::RefConst<JPH::Shape>, 2> shapes;
    JPH::Ref<VJoltPathSpline> spline;
    JPH::Ref<JPH::PathConstraint> constraint;
    bool retiring = false;
    bool CanSimulate() const
    {
        for (size_t i = 0; i < objects.size(); ++i)
            if (objects[i] && (shapes[i].GetPtr() != objects[i]->GetBody()->GetShape()
                || !Supported(objects[i], i == 0))) return false;
        return !retiring;
    }
    void Wake() const { for (auto *object : objects) if (object) object->Wake(); }
    void OnJoltPhysicsObjectDestroyed(JoltPhysicsObject *object) override;
};

class PathBridge final : public IVJoltPathBridge
{
public:
    uint32_t GetVersion() const override { return kVJoltPathBridgeVersion; }
    VJoltPathHandle Create(IPhysicsObject *follower, IPhysicsObject *reference,
        const VJoltConstraintVector *nodes, uint32_t count, const VJoltPathSettings &settings,
        const VJoltPathMotor &motor, Error *error) override
    {
        auto fail = [&](Error e) -> VJoltPathHandle { Result(error, e); return 0; };
        if (!Owner()) return fail(Error::WrongThread);
        if (settings.size != sizeof(settings) || (settings.flags & ~(VJoltPathClosed | VJoltPathSnapToPath))
            || settings.rotation > VJoltPathRotation::Fixed || !Finite(settings.smoothing, 1)
            || settings.smoothing < .05f || !ValidMotor(motor)) return fail(Error::InvalidSettings);
        const bool closed = (settings.flags & VJoltPathClosed) != 0;
        if (!nodes || count < (closed ? 3u : 2u) || count > kVJoltPathMaxNodes) return fail(Error::InvalidNodes);
        if (!Live(follower) || (reference && !Live(reference)) || follower == reference) return fail(Error::InvalidBody);
        auto *moving = static_cast<JoltPhysicsObject *>(follower);
        auto *base = static_cast<JoltPhysicsObject *>(reference);
        auto *env = moving->GetJoltEnvironment();
        if (!env || (base && env != base->GetJoltEnvironment())) return fail(Error::DifferentEnvironment);
        if (env->IsInSimulation() || env->IsInContactCallback()) return fail(Error::Busy);
        if (!Supported(moving, true) || (base && !Supported(base, false))) return fail(Error::UnsupportedBody);
        if (m_Tracks.size() >= 256 || m_NextHandle == std::numeric_limits<uint64_t>::max()) return fail(Error::LimitExceeded);

        auto track = std::make_unique<Track>();
        track->owner = this; track->environment = env; track->objects = {moving, base};
        auto &body1 = base ? *base->GetBody() : JPH::Body::sFixedToWorld;
        auto &body2 = *moving->GetBody();
        const auto inverseBase = body1.GetWorldTransform().InversedRotationTranslation();
        std::vector<JPH::Vec3> points; points.reserve(count);
        for (uint32_t i = 0; i < count; ++i)
        {
            const auto &p = nodes[i];
            if (!Finite(p.x, 1048576) || !Finite(p.y, 1048576) || !Finite(p.z, 1048576)) return fail(Error::InvalidNodes);
            points.push_back(JPH::Vec3(inverseBase * JPH::RVec3(SourceToJolt::Distance(Vector(p.x, p.y, p.z)))));
        }
        const auto origin = points[0];
        for (auto &p : points) p -= origin;
        track->spline = new VJoltPathSpline;
        if (!track->spline->Build(points, settings.smoothing, closed)) return fail(Error::DegeneratePath);
        if (JoltToSource::Distance(track->spline->GetPathMaxFraction()) > 262144) return fail(Error::LimitExceeded);
        // RTTI is registered lazily, after the backend initialized Jolt's factory.
        if (!JPH::Factory::sInstance->Find(JPH_RTTI(VJoltPathSpline)->GetHash()))
            JPH::Factory::sInstance->Register(JPH_RTTI(VJoltPathSpline));

        JPH::PathConstraintSettings config;
        config.mPath = track->spline;
        config.mPathPosition = origin;
        config.mRotationConstraintType = static_cast<JPH::EPathRotationConstraintType>(settings.rotation);
        const auto localCOM = JPH::Vec3(inverseBase * body2.GetCenterOfMassPosition()) - origin;
        config.mPathFraction = track->spline->GetClosestPoint(localCOM, 0);
        if (settings.flags & VJoltPathSnapToPath)
        {
            JPH::Vec3 p, t, n, b; track->spline->GetPointOnPath(config.mPathFraction, p, t, n, b);
            const auto desiredCOM = body1.GetWorldTransform() * (p + origin);
            Vector position; QAngle angles; moving->GetPosition(&position, &angles);
            moving->SetPosition(position + JoltToSource::Distance(JPH::Vec3(desiredCOM - body2.GetCenterOfMassPosition())), angles, true);
        }
        track->constraint = static_cast<JPH::PathConstraint *>(config.Create(body1, body2));
        ApplyMotor(*track, motor);
        track->handle = m_NextHandle++;
        const auto handle = track->handle;
        for (size_t i = 0; i < track->objects.size(); ++i) if (auto *object = track->objects[i])
        {
            track->shapes[i] = object->GetBody()->GetShape();
            object->AddDestroyedListener(track.get()); object->AddConstraintAttachment();
        }
        env->GetPhysicsSystem()->AddConstraint(track->constraint);
        track->Wake(); m_Tracks.emplace(handle, std::move(track));
        Result(error, Error::None); return handle;
    }
    bool IsValid(VJoltPathHandle handle) const override
    {
        if (!Owner()) return false;
        auto *track = Find(handle); return track && track->CanSimulate();
    }
    bool Destroy(VJoltPathHandle handle) override
    {
        if (!Owner()) return false;
        auto *track = Find(handle); if (!track) return false;
        track->retiring = true;
        auto *env = track->environment;
        if (env->DeferPhysicsMutation([this, handle] { Remove(handle); })) return true;
        if (!env->IsInSimulation()) Remove(handle);
        return true;
    }
    bool SetMotor(VJoltPathHandle handle, const VJoltPathMotor &motor, Error *error) override
    {
        if (!Owner()) return Result(error, Error::WrongThread);
        auto *track = Find(handle); if (!track || !track->CanSimulate()) return Result(error, Error::Stale);
        if (track->environment->IsInSimulation() || track->environment->IsInContactCallback()) return Result(error, Error::Busy);
        // Do not overwrite motor state while this constraint is disabled.
        if (!track->constraint->GetEnabled()) return Result(error, Error::Busy);
        if (!ValidMotor(motor)) return Result(error, Error::InvalidSettings);
        ApplyMotor(*track, motor); track->Wake(); return Result(error, Error::None);
    }
    bool GetState(VJoltPathHandle handle, VJoltPathState &state) const override
    {
        if (!Owner() || state.size != sizeof(state)) return false;
        const auto *track = Find(handle);
        if (!track || !track->CanSimulate() || track->environment->IsInSimulation()) return false;
        const auto &c = *track->constraint;
        state = {}; state.enabled = c.GetEnabled(); state.closed = track->spline->IsLooping();
        state.nodeCount = uint32_t(track->spline->nodes.size());
        const float length = track->spline->GetPathMaxFraction();
        state.progress = c.GetPathFraction() / length;
        state.distance = JoltToSource::Distance(c.GetPathFraction()); state.length = JoltToSource::Distance(length);
        state.motor.mode = static_cast<VJoltPathMotorMode>(c.GetPositionMotorState());
        state.motor.speed = JoltToSource::Distance(c.GetTargetVelocity());
        state.motor.target = c.GetTargetPathFraction() / length;
        state.motor.maxForce = JoltToSource::Distance(c.GetPositionMotorSettings().mMaxForceLimit);
        state.motor.friction = JoltToSource::Distance(c.GetMaxFrictionForce());
        state.motor.frequency = c.GetPositionMotorSettings().mSpringSettings.mFrequency;
        state.motor.damping = c.GetPositionMotorSettings().mSpringSettings.mDamping;
        JPH::Vec3 p, t, n, b; track->spline->GetPointOnPath(c.GetPathFraction(), p, t, n, b);
        const auto transform = c.GetBody1()->GetCenterOfMassTransform() * c.GetConstraintToBody1Matrix();
        const auto anchor1 = transform * p;
        const auto anchor2 = (c.GetBody2()->GetCenterOfMassTransform() * c.GetConstraintToBody2Matrix()).GetTranslation();
        const auto relative = c.GetBody2()->GetPointVelocity(anchor2) - c.GetBody1()->GetPointVelocity(anchor1);
        state.velocity = JoltToSource::Distance(relative.Dot(transform.Multiply3x3(t)));
        return true;
    }
    bool Sample(VJoltPathHandle handle, uint32_t count, VJoltPathSample *samples) const override
    {
        if (!Owner() || !samples || count < 2 || count > 513) return false;
        const auto *track = Find(handle);
        if (!track || !track->CanSimulate() || track->environment->IsInSimulation()) return false;
        const auto &c = *track->constraint;
        const auto transform = c.GetBody1()->GetCenterOfMassTransform() * c.GetConstraintToBody1Matrix();
        for (uint32_t i = 0; i < count; ++i)
        {
            JPH::Vec3 p, t, n, b; track->spline->GetPointOnPath(track->spline->GetPathMaxFraction() * float(i) / float(count - 1), p, t, n, b);
            samples[i] = {SourceVector(JPH::Vec3(transform * p)), SourceVector(transform.Multiply3x3(t), true),
                SourceVector(transform.Multiply3x3(n), true)};
        }
        return true;
    }
    void BeforeSimulation(JoltPhysicsEnvironment *env)
    {
        for (auto it = m_Tracks.begin(); it != m_Tracks.end(); )
        {
            auto current = it++;
            if (current->second->environment == env && !current->second->CanSimulate()) Remove(current->first);
        }
    }
    void ReleaseObject(JoltPhysicsObject *object)
    {
        for (auto it = m_Tracks.begin(); it != m_Tracks.end(); )
        {
            auto current = it++; const auto &objects = current->second->objects;
            if (objects[0] == object || objects[1] == object) Remove(current->first, object);
        }
    }
    void EnvironmentDestroyed(JoltPhysicsEnvironment *env)
    {
        for (auto it = m_Tracks.begin(); it != m_Tracks.end(); )
        {
            auto current = it++;
            if (current->second->environment == env) Remove(current->first, nullptr, false);
        }
    }
private:
    friend struct Track;
    bool Owner() const { return std::this_thread::get_id() == m_Owner; }
    Track *Find(VJoltPathHandle handle) const
    {
        const auto it = m_Tracks.find(handle);
        return it == m_Tracks.end() || it->second->retiring ? nullptr : it->second.get();
    }
    void ApplyMotor(Track &track, const VJoltPathMotor &motor)
    {
        auto &c = *track.constraint;
        auto &settings = c.GetPositionMotorSettings();
        settings.SetForceLimit(SourceToJolt::Distance(motor.maxForce));
        settings.mSpringSettings = JPH::SpringSettings(JPH::ESpringMode::FrequencyAndDamping, motor.frequency, motor.damping);
        c.SetMaxFrictionForce(SourceToJolt::Distance(motor.friction));
        c.SetTargetVelocity(SourceToJolt::Distance(motor.speed));
        c.SetTargetPathFraction(motor.target * track.spline->GetPathMaxFraction());
        c.SetPositionMotorState(motor.maxForce == 0 ? JPH::EMotorState::Off : static_cast<JPH::EMotorState>(motor.mode));
    }
    void Remove(VJoltPathHandle handle, JoltPhysicsObject *dying = nullptr, bool wake = true)
    {
        const auto it = m_Tracks.find(handle); if (it == m_Tracks.end()) return;
        auto &track = *it->second;
        track.environment->GetPhysicsSystem()->RemoveConstraint(track.constraint);
        for (auto *object : track.objects) if (object)
        {
            object->RemoveDestroyedListener(&track); object->RemoveConstraintAttachment();
            if (wake && object != dying && !(object->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE)) object->Wake();
        }
        m_Tracks.erase(it);
    }
    std::unordered_map<VJoltPathHandle, std::unique_ptr<Track>> m_Tracks;
    VJoltPathHandle m_NextHandle = 1;
    const std::thread::id m_Owner = std::this_thread::get_id();
};
void Track::OnJoltPhysicsObjectDestroyed(JoltPhysicsObject *object) { owner->Remove(handle, object); }
PathBridge g_PathBridge;
}

EXPOSE_SINGLE_INTERFACE_GLOBALVAR(PathBridge, IVJoltPathBridge, VJOLT_PATH_BRIDGE_VERSION, g_PathBridge);
void VJoltPathBeforeSimulation(JoltPhysicsEnvironment *env) { g_PathBridge.BeforeSimulation(env); }
void VJoltPathReleaseObject(JoltPhysicsObject *object) { g_PathBridge.ReleaseObject(object); }
void VJoltPathEnvironmentDestroyed(JoltPhysicsEnvironment *env) { g_PathBridge.EnvironmentDestroyed(env); }
