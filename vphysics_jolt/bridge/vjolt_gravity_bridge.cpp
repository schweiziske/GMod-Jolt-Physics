#include "cbase.h"
#include "vjolt_environment.h"
#include "vjolt_object.h"
#include "vjolt_interface.h"
#include "bridge/vjolt_gravity_internal.h"

#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseQuery.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <vector>

#include "tier0/memdbgon.h"

#if GAME_GMOD
extern bool IsValidPhyiscsObject(IPhysicsObject *);
#endif

namespace
{
Vector Source(const VJoltGravityVector &v) { return Vector(v.x, v.y, v.z); }
VJoltGravityVector Payload(const Vector &v) { return {v.x, v.y, v.z}; }
bool Finite(const VJoltGravityVector &v)
{
	return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z)
		&& std::abs(v.x) <= 1.0e7f && std::abs(v.y) <= 1.0e7f && std::abs(v.z) <= 1.0e7f;
}
JoltPhysicsEnvironment *Environment(int index)
{
	return static_cast<JoltPhysicsEnvironment *>(JoltPhysicsInterface::GetInstance().GetActiveEnvironmentByIndex(index));
}
bool Idle(JoltPhysicsEnvironment *env) { return env && !env->IsInSimulation() && !env->IsInContactCallback(); }
JoltPhysicsObject *Object(IPhysicsObject *ptr)
{
	if (!ptr) return nullptr;
#if GAME_GMOD
	if (!IsValidPhyiscsObject(ptr)) return nullptr;
#else
	bool found = false;
	for (int i = 0; auto *env = Environment(i); ++i)
	{
		int count = 0;
		const auto **objects = env->GetObjectList(&count);
		for (int j = 0; j < count; ++j) found |= objects[j] == ptr;
	}
	if (!found) return nullptr;
#endif
	auto *object = static_cast<JoltPhysicsObject *>(ptr);
	if (!Idle(object->GetJoltEnvironment()) || (object->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE)) return nullptr;
	return object;
}
bool Valid(const VJoltGravityField &field)
{
	return field.size == sizeof(field) && field.shape <= VJoltGravityBox && field.mode <= VJoltGravityRepel
		&& Finite(field.center) && Finite(field.halfExtent) && Finite(field.gravity)
		&& field.halfExtent.x > 0 && field.halfExtent.y > 0 && field.halfExtent.z > 0
		&& std::isfinite(field.strength) && field.strength >= 0 && field.strength <= 1.0e7f;
}
bool Contains(const VJoltGravityField &field, const Vector &point)
{
	const Vector delta = point - Source(field.center);
	if (field.shape == VJoltGravitySphere) return delta.LengthSqr() <= field.halfExtent.x * field.halfExtent.x;
	return std::abs(delta.x) <= field.halfExtent.x && std::abs(delta.y) <= field.halfExtent.y
		&& std::abs(delta.z) <= field.halfExtent.z;
}
Vector Acceleration(const VJoltGravityField &field, const Vector &point)
{
	if (field.mode == VJoltGravityUniform) return Source(field.gravity);
	Vector delta = Source(field.center) - point;
	const float length = delta.Length();
	if (length < 1.0e-5f) return vec3_origin;
	return delta * ((field.mode == VJoltGravityRepel ? -field.strength : field.strength) / length);
}
Vector Position(JoltPhysicsObject *object)
{
	return JoltToSource::Distance(object->GetBody()->GetCenterOfMassPosition());
}
struct Record
{
	VJoltGravityObject config;
	bool vehicle = false;
};
struct FieldSlot
{
	JoltPhysicsEnvironment *environment = nullptr;
	VJoltGravityField config;
	uint32_t generation = 1;
	bool pendingDestroy = false;
};

class GravityBridge final : public IVJoltGravityBridge
{
public:
	uint32_t GetVersion() const override { return kVJoltGravityBridgeVersion; }
	uint64_t GetRevision() const override { return mRevision; }
	bool GetFieldAt(int32_t environment, uint32_t ordinal, VJoltGravityFieldHandle &handle, VJoltGravityField &out) const override
	{
		auto *env = Environment(environment);
		if (!Idle(env) || out.size != sizeof(out)) return false;
		for (size_t i = 0; i < mFields.size(); ++i) if (mFields[i].environment == env)
		{
			if (ordinal-- == 0) { handle = Handle(i); out = mFields[i].config; return true; }
		}
		return false;
	}
	bool GetWorld(int32_t index, VJoltGravityVector &out) const override
	{
		auto *env = Environment(index);
		if (!Idle(env)) return false;
		Vector gravity; env->GetGravity(&gravity); out = Payload(gravity); return true;
	}
	bool SetWorld(int32_t index, const VJoltGravityVector &gravity) override
	{
		auto *env = Environment(index);
		if (!Idle(env) || !Finite(gravity)) return false;
		env->SetGravity(Source(gravity)); // WorldChanged also handles physenv.SetGravity callers.
		return true;
	}
	Record &Ensure(JoltPhysicsObject *object)
	{
		auto [it, inserted] = mObjects.try_emplace(object);
		if (inserted)
		{
			const auto *motion = object->GetBody()->GetMotionPropertiesUnchecked();
			const float factor = motion ? motion->GetGravityFactor() : 0.0f;
			it->second.config.flags = factor != 0.0f ? VJoltGravityEnabled : 0;
			it->second.config.factor = factor != 0.0f ? factor : 1.0f;
		}
		return it->second;
	}
	VJoltGravityObject Read(JoltPhysicsObject *object) const
	{
		const auto it = mObjects.find(object);
		if (it != mObjects.end()) return it->second.config;
		VJoltGravityObject config;
		const auto *motion = object->GetBody()->GetMotionPropertiesUnchecked();
		const float factor = motion ? motion->GetGravityFactor() : 0.0f;
		config.flags = factor != 0 ? VJoltGravityEnabled : 0;
		config.factor = factor != 0 ? factor : 1;
		return config;
	}
	bool GetObjectState(IPhysicsObject *ptr, VJoltGravityObject &out) const override
	{
		auto *object = Object(ptr);
		if (!object || out.size != sizeof(out)) return false;
		out = Read(object); return true;
	}
	void ApplyFactor(JoltPhysicsObject *object, const Record &record)
	{
		if (!object->GetBody()->IsStatic())
			object->GetBody()->GetMotionProperties()->SetGravityFactor(
				record.vehicle ? 0.0f : ((record.config.flags & VJoltGravityEnabled) ? record.config.factor : 0.0f));
	}
	bool SetObject(IPhysicsObject *ptr, const VJoltGravityObject &config) override
	{
		auto *object = Object(ptr);
		if (!object || object->IsStatic() || config.size != sizeof(config)
			|| (config.flags & ~(VJoltGravityEnabled | VJoltGravityOverride)) != 0
			|| !std::isfinite(config.factor) || std::abs(config.factor) > 1024.0f || !Finite(config.gravity)) return false;
		auto &record = Ensure(object);
		record.config = config;
		ApplyFactor(object, record);
		object->Wake(); ++mRevision; return true;
	}
	VJoltGravityFieldHandle Handle(size_t index) const
	{
		return (uint64_t(mFields[index].generation) << 32) | uint64_t(index + 1);
	}
	FieldSlot *Find(VJoltGravityFieldHandle handle)
	{
		const uint32_t slot = uint32_t(handle);
		if (!slot || slot > mFields.size()) return nullptr;
		auto &field = mFields[slot - 1];
		return field.environment && field.generation == uint32_t(handle >> 32) ? &field : nullptr;
	}
	const FieldSlot *Find(VJoltGravityFieldHandle handle) const { return const_cast<GravityBridge *>(this)->Find(handle); }
	template <typename Visitor>
	void Visit(const FieldSlot &field, Visitor &&visitor)
	{
		const auto &c = field.config;
		const Vector half = c.shape == VJoltGravitySphere ? Vector(c.halfExtent.x, c.halfExtent.x, c.halfExtent.x) : Source(c.halfExtent);
		const JPH::Vec3 center = SourceToJolt::Distance(Source(c.center));
		const JPH::Vec3 extent = SourceToJolt::Distance(half).Abs();
		// The only visitors gather candidates or queue wakes; neither enters game
		// callbacks. Keep their mutations outside the broadphase collector itself.
		mCollector.Reset();
		auto *system = field.environment->GetPhysicsSystem();
		system->GetBroadPhaseQuery().CollideAABox(JPH::AABox(center - extent, center + extent), mCollector);
		for (const JPH::BodyID &id : mCollector.mHits)
		{
			auto *body = system->GetBodyLockInterfaceNoLock().TryGetBody(id);
			if (!body || !body->IsDynamic() || !body->GetUserData()) continue;
			auto *object = reinterpret_cast<JoltPhysicsObject *>(body->GetUserData());
			if (!(object->GetCallbackFlags() & CALLBACK_MARKED_FOR_DELETE) && Contains(c, Position(object))) visitor(object);
		}
	}
	void Wake(const FieldSlot &field) { Visit(field, [](JoltPhysicsObject *object) { object->Wake(); }); }
	VJoltGravityFieldHandle CreateField(int32_t index, const VJoltGravityField &config) override
	{
		auto *env = Environment(index);
		if (!Idle(env) || !Valid(config)) return 0;
		size_t slot = 0;
		while (slot < mFields.size() && mFields[slot].environment) ++slot;
		if (slot == mFields.size())
		{
			if (slot >= 1024) return 0;
			mFields.emplace_back();
		}
		auto &field = mFields[slot]; field.environment = env; field.config = config; field.pendingDestroy = false;
		Wake(field); ++mRevision; return Handle(slot);
	}
	bool UpdateField(VJoltGravityFieldHandle handle, const VJoltGravityField &config) override
	{
		auto *field = Find(handle);
		if (!field || !Idle(field->environment) || !Valid(config)) return false;
		Wake(*field); field->config = config; Wake(*field); ++mRevision; return true;
	}
	bool GetField(VJoltGravityFieldHandle handle, VJoltGravityField &out) const override
	{
		const auto *field = Find(handle);
		if (!field || !Idle(field->environment) || out.size != sizeof(out)) return false;
		out = field->config; return true;
	}
	void Retire(FieldSlot &field)
	{
		field.environment = nullptr;
		field.pendingDestroy = false;
		if (++field.generation == 0) ++field.generation;
		++mRevision;
	}
	bool DestroyField(VJoltGravityFieldHandle handle) override
	{
		auto *field = Find(handle);
		if (!field) return true; // Idempotent, including handles retired by Reset.
		if (!Idle(field->environment)) { field->pendingDestroy = true; return true; }
		Wake(*field); Retire(*field); return true;
	}
	Vector SampleInternal(JoltPhysicsEnvironment *env, const Vector &position) const
	{
		Vector gravity; env->GetGravity(&gravity);
		const FieldSlot *selected = nullptr;
		// Equal priority: older slot wins, stable across steps and network snapshots.
		for (const auto &field : mFields)
			if (field.environment == env && (!selected || field.config.priority > selected->config.priority)
				&& Contains(field.config, position)) selected = &field;
		return selected ? Acceleration(selected->config, position) : gravity;
	}
	bool Sample(int32_t index, const VJoltGravityVector &position, VJoltGravityVector &out) const override
	{
		auto *env = Environment(index);
		if (!Idle(env) || !Finite(position)) return false;
		out = Payload(SampleInternal(env, Source(position))); return true;
	}
	Vector Effective(JoltPhysicsObject *object) const
	{
		const auto config = Read(object);
		if (!(config.flags & VJoltGravityEnabled)) return vec3_origin;
		return ((config.flags & VJoltGravityOverride) ? Source(config.gravity)
			: SampleInternal(object->GetJoltEnvironment(), Position(object))) * config.factor;
	}
	bool GetEffectiveGravity(IPhysicsObject *ptr, VJoltGravityVector &out) const override
	{
		auto *object = Object(ptr); if (!object) return false;
		out = Payload(Effective(object)); return true;
	}
	bool Reset(int32_t index) override
	{
		auto *env = Environment(index); if (!Idle(env)) return false;
		for (auto &field : mFields) if (field.environment == env) { Wake(field); Retire(field); }
		for (auto it = mObjects.begin(); it != mObjects.end();)
		{
			auto *object = it->first;
			if (object->GetJoltEnvironment() != env) { ++it; continue; }
			// Preserve the user's current gravity enable switch when clearing configuration.
			const uint32_t enabled = it->second.config.flags & VJoltGravityEnabled;
			it->second.config = {}; it->second.config.flags = enabled;
			ApplyFactor(object, it->second); object->Wake();
			if (it->second.vehicle) ++it; else it = mObjects.erase(it);
		}
		++mRevision; return true;
	}
	void Step(JoltPhysicsEnvironment *env)
	{
		for (auto &field : mFields) if (field.environment == env && field.pendingDestroy) { Wake(field); Retire(field); }
		// Broadphase selects field candidates. No Lua callback, full entity scan or per-step wake.
		auto &selected = mSelected;
		for (const auto &field : mFields) if (field.environment == env)
			Visit(field, [&](JoltPhysicsObject *object)
			{
				if (!object->GetBody()->IsActive()) return;
				auto [it, inserted] = selected.try_emplace(object, &field);
				if (!inserted && field.config.priority > it->second->config.priority) it->second = &field;
			});
		for (const auto &[object, record] : mObjects)
			if (object->GetJoltEnvironment() == env && (record.config.flags & VJoltGravityOverride)
				&& object->GetBody()->IsDynamic() && object->GetBody()->IsActive()) selected[object] = nullptr;
		Vector world; env->GetGravity(&world);
		for (const auto &[object, field] : selected)
		{
			const auto it = mObjects.find(object);
			if (it != mObjects.end() && it->second.vehicle) continue; // VehicleConstraint applies the complete acceleration.
			auto *body = object->GetBody();
			const auto config = Read(object);
			if (!(config.flags & VJoltGravityEnabled) || !body->IsDynamic()) continue;
			const Vector gravity = (config.flags & VJoltGravityOverride) ? Source(config.gravity) : Acceleration(field->config, Position(object));
			const JPH::Vec3 delta = SourceToJolt::Distance((gravity - world) * config.factor);
			const float inverseMass = body->GetMotionProperties()->GetInverseMass();
			if (inverseMass > 0 && !delta.IsNearZero()) body->AddForce(delta / inverseMass);
		}
		// Drop references before fields or bodies can be destroyed, but retain the
		// dense storage instead of allocating one unordered_map node per body/PSI.
		if (!selected.empty()) selected.clear();
	}
	void WorldChanged(JoltPhysicsEnvironment *env)
	{
		++mRevision;
		JPH::BodyIDVector bodies; env->GetPhysicsSystem()->GetBodies(bodies);
		for (auto id : bodies)
		{
			auto *body = env->GetPhysicsSystem()->GetBodyLockInterfaceNoLock().TryGetBody(id);
			if (body && body->IsDynamic()) env->GetPhysicsSystem()->GetBodyInterfaceNoLock().ActivateBody(id);
		}
	}
	std::unordered_map<JoltPhysicsObject *, Record> mObjects;
	std::vector<FieldSlot> mFields;
	uint64_t mRevision = 0;
private:
	// Gravity bridge operations and environment stepping run on the owner thread.
	ankerl::unordered_dense::map<JoltPhysicsObject *, const FieldSlot *> mSelected;
	JPH::AllHitCollisionCollector<JPH::CollideShapeBodyCollector> mCollector;
};
GravityBridge sGravity;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR(GravityBridge, IVJoltGravityBridge, VJOLT_GRAVITY_BRIDGE_VERSION, sGravity);
}

void VJoltGravityBeforeSimulation(JoltPhysicsEnvironment *env) { sGravity.Step(env); }
void VJoltGravityWorldChanged(JoltPhysicsEnvironment *env) { sGravity.WorldChanged(env); }
void VJoltGravityObjectDestroyed(JoltPhysicsObject *object) { sGravity.mObjects.erase(object); }
void VJoltGravityRefreshObject(JoltPhysicsObject *object)
{
	auto it = sGravity.mObjects.find(object);
	if (it != sGravity.mObjects.end()) sGravity.ApplyFactor(object, it->second);
}
void VJoltGravityEnvironmentDestroyed(JoltPhysicsEnvironment *env)
{
	for (auto &field : sGravity.mFields) if (field.environment == env) sGravity.Retire(field);
}
bool VJoltGravityEnableObject(JoltPhysicsObject *object, bool enabled)
{
	auto it = sGravity.mObjects.find(object); if (it == sGravity.mObjects.end()) return false;
	if (enabled) it->second.config.flags |= VJoltGravityEnabled;
	else it->second.config.flags &= ~VJoltGravityEnabled;
	sGravity.ApplyFactor(object, it->second);
	object->Wake(); ++sGravity.mRevision; return true;
}
bool VJoltGravityIsObjectEnabled(const JoltPhysicsObject *object, bool &enabled)
{
	auto it = sGravity.mObjects.find(const_cast<JoltPhysicsObject *>(object));
	if (it == sGravity.mObjects.end()) return false;
	enabled = (it->second.config.flags & VJoltGravityEnabled) != 0; return true;
}
void VJoltGravityManageVehicle(JoltPhysicsObject *object, bool managed)
{
	if (!object) return;
	auto &record = sGravity.Ensure(object); record.vehicle = managed;
	sGravity.ApplyFactor(object, record);
}
VJoltGravityVector VJoltGravityForObject(JoltPhysicsObject *object) { return Payload(sGravity.Effective(object)); }
