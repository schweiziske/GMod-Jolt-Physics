#include <GarrysMod/Lua/Interface.h>
#include <vjolt/gravity.h>
#include <vjolt/loader.h>
#ifdef GetObject
#undef GetObject
#endif
#include <cstdio>
#include <cmath>
#include <unordered_set>
#include <initializer_list>

using namespace GarrysMod::Lua;
static IVJoltGravityBridge *bridge;
static int fieldMeta;
static std::unordered_set<VJoltGravityFieldHandle> ownedFields;
static int Fail(ILua *lua, const char *why = "invalid/stale object or field, invalid input, or physics world is stepping")
{
	lua->PushNil(); lua->PushString(bridge ? why : "matching VJoltGravityBridge001 backend is not loaded"); return 2;
}
static VJoltGravityVector ReadVector(ILua *lua, int index)
{
	lua->CheckType(index, Type::Vector);
	const auto *v = lua->GetUserType<VJoltGravityVector>(index, Type::Vector);
	return v ? *v : VJoltGravityVector{};
}
static void PushVector(ILua *lua, const VJoltGravityVector &v)
{
	// LuaBase::PushVector's ABI is the engine Vector: three float components.
	lua->PushVector(*reinterpret_cast<const Vector *>(&v));
}
static float NumberField(ILua *lua, int index, const char *key, float fallback)
{
	lua->GetField(index, key);
	const float value = lua->IsType(-1, Type::Nil) ? fallback : float(lua->CheckNumber(-1));
	lua->Pop();
	if (!std::isfinite(value)) lua->ThrowError("gravity configuration numbers must be finite");
	return value;
}
static void VectorField(ILua *lua, int index, const char *key, VJoltGravityVector &v)
{
	lua->GetField(index, key);
	if (!lua->IsType(-1, Type::Nil)) v = ReadVector(lua, -1);
	lua->Pop();
}
static int EnvironmentIndex(ILua *lua, int index)
{
	const double value = lua->Top() >= index && !lua->IsType(index, Type::Nil) ? lua->CheckNumber(index) : 0;
	if (!std::isfinite(value) || value < 0 || value > 4096 || value != std::floor(value)) lua->ThrowError("invalid environment index");
	return int(value);
}
static IPhysicsObject *Phys(ILua *lua)
{
	lua->CheckType(1, Type::PhysObj); return lua->GetUserType<IPhysicsObject>(1, Type::PhysObj);
}
static void ParseField(ILua *lua, int index, VJoltGravityField &field)
{
	lua->CheckType(index, Type::Table);
	const float shape = NumberField(lua, index, "shape", float(field.shape));
	const float mode = NumberField(lua, index, "mode", float(field.mode));
	const float priority = NumberField(lua, index, "priority", float(field.priority));
	if (shape < 0 || shape > 1 || shape != std::floor(shape) || mode < 0 || mode > 2 || mode != std::floor(mode)
		|| std::abs(priority) > 1000000 || priority != std::floor(priority)) lua->ThrowError("invalid field shape, mode or priority");
	field.shape = uint32_t(shape); field.mode = uint32_t(mode); field.priority = int32_t(priority);
	VectorField(lua, index, "center", field.center);
	VectorField(lua, index, "halfExtent", field.halfExtent);
	VectorField(lua, index, "gravity", field.gravity);
	field.strength = NumberField(lua, index, "strength", field.strength);
	if (field.shape == VJoltGravitySphere)
	{
		field.halfExtent.x = NumberField(lua, index, "radius", field.halfExtent.x);
		field.halfExtent.y = field.halfExtent.z = field.halfExtent.x;
	}
}
static void PushField(ILua *lua, VJoltGravityFieldHandle handle, const VJoltGravityField &field)
{
	lua->CreateTable();
	auto number = [&](const char *name, double value) { lua->PushNumber(value); lua->SetField(-2, name); };
	number("shape", field.shape); number("mode", field.mode); number("priority", field.priority);
	number("strength", field.strength); number("radius", field.halfExtent.x); number("slot", uint32_t(handle));
	char identity[32]; std::snprintf(identity, sizeof(identity), "%llu", static_cast<unsigned long long>(handle));
	lua->PushString(identity); lua->SetField(-2, "id");
	PushVector(lua, field.center); lua->SetField(-2, "center");
	PushVector(lua, field.halfExtent); lua->SetField(-2, "halfExtent");
	PushVector(lua, field.gravity); lua->SetField(-2, "gravity");
}
LUA_FUNCTION_STATIC(Available) { LUA->PushBool(bridge != nullptr); return 1; }
LUA_FUNCTION_STATIC(GetWorld)
{
	VJoltGravityVector value;
	if (!bridge || !bridge->GetWorld(EnvironmentIndex(LUA, 1), value)) return Fail(LUA);
	PushVector(LUA, value); return 1;
}
LUA_FUNCTION_STATIC(SetWorld)
{
	const auto value = ReadVector(LUA, 1);
	if (!bridge || !bridge->SetWorld(EnvironmentIndex(LUA, 2), value)) return Fail(LUA);
	LUA->PushBool(true); return 1;
}
LUA_FUNCTION_STATIC(GetObject)
{
	auto *object = Phys(LUA); VJoltGravityObject state;
	if (!bridge || !bridge->GetObjectState(object, state)) return Fail(LUA);
	LUA->CreateTable();
	LUA->PushNumber(state.factor); LUA->SetField(-2, "factor");
	LUA->PushBool((state.flags & VJoltGravityEnabled) != 0); LUA->SetField(-2, "enabled");
	if (state.flags & VJoltGravityOverride) PushVector(LUA, state.gravity); else LUA->PushNil();
	LUA->SetField(-2, "gravity"); return 1;
}
LUA_FUNCTION_STATIC(Configure)
{
	auto *object = Phys(LUA); LUA->CheckType(2, Type::Table); VJoltGravityObject state;
	state.factor = NumberField(LUA, 2, "factor", 1);
	LUA->GetField(2, "enabled");
	if (LUA->IsType(-1, Type::Bool) && !LUA->GetBool(-1)) state.flags = 0;
	LUA->Pop();
	LUA->GetField(2, "gravity");
	if (!LUA->IsType(-1, Type::Nil)) { state.gravity = ReadVector(LUA, -1); state.flags |= VJoltGravityOverride; }
	LUA->Pop();
	if (!bridge || !bridge->SetObject(object, state)) return Fail(LUA);
	LUA->PushBool(true); return 1;
}
LUA_FUNCTION_STATIC(SetFactor)
{
	auto *object = Phys(LUA); const float factor = float(LUA->CheckNumber(2)); VJoltGravityObject state;
	if (!bridge || !bridge->GetObjectState(object, state)) return Fail(LUA);
	state.factor = factor;
	if (!bridge->SetObject(object, state)) return Fail(LUA);
	LUA->PushBool(true); return 1;
}
LUA_FUNCTION_STATIC(SetGravity)
{
	auto *object = Phys(LUA); VJoltGravityObject state;
	if (!bridge || !bridge->GetObjectState(object, state)) return Fail(LUA);
	if (LUA->IsType(2, Type::Nil)) state.flags &= ~VJoltGravityOverride;
	else { state.gravity = ReadVector(LUA, 2); state.flags |= VJoltGravityOverride; }
	if (!bridge->SetObject(object, state)) return Fail(LUA);
	LUA->PushBool(true); return 1;
}
LUA_FUNCTION_STATIC(GetEffectiveGravity)
{
	auto *object = Phys(LUA); VJoltGravityVector value;
	if (!bridge || !bridge->GetEffectiveGravity(object, value)) return Fail(LUA);
	PushVector(LUA, value); return 1;
}
LUA_FUNCTION_STATIC(Sample)
{
	const auto position = ReadVector(LUA, 1); VJoltGravityVector value;
	if (!bridge || !bridge->Sample(EnvironmentIndex(LUA, 2), position, value)) return Fail(LUA);
	PushVector(LUA, value); return 1;
}
LUA_FUNCTION_STATIC(Reset)
{
	if (!bridge || !bridge->Reset(EnvironmentIndex(LUA, 1))) return Fail(LUA);
	LUA->PushBool(true); return 1;
}
LUA_FUNCTION_STATIC(GetRevision) { LUA->PushNumber(bridge ? double(bridge->GetRevision()) : 0); return 1; }
static VJoltGravityFieldHandle *Handle(ILua *lua) { return lua->GetUserType<VJoltGravityFieldHandle>(1, fieldMeta); }
LUA_FUNCTION_STATIC(FieldDestroy)
{
	auto *handle = Handle(LUA);
	if (!handle || !*handle) { LUA->PushBool(false); return 1; }
	const bool removed = bridge && bridge->DestroyField(*handle);
	if (!removed) return Fail(LUA);
	ownedFields.erase(*handle); *handle = 0; LUA->PushBool(removed); return 1;
}
LUA_FUNCTION_STATIC(FieldValid)
{
	auto *handle = Handle(LUA); VJoltGravityField config;
	LUA->PushBool(handle && bridge && bridge->GetField(*handle, config)); return 1;
}
LUA_FUNCTION_STATIC(FieldGet)
{
	auto *handle = Handle(LUA); VJoltGravityField config;
	if (!handle || !bridge || !bridge->GetField(*handle, config)) return Fail(LUA);
	PushField(LUA, *handle, config); return 1;
}
LUA_FUNCTION_STATIC(FieldUpdate)
{
	auto *handle = Handle(LUA); VJoltGravityField config;
	if (!handle || !bridge || !bridge->GetField(*handle, config)) return Fail(LUA);
	ParseField(LUA, 2, config);
	if (!bridge->UpdateField(*handle, config)) return Fail(LUA);
	LUA->PushBool(true); return 1;
}
LUA_FUNCTION_STATIC(CreateField)
{
	VJoltGravityField config; ParseField(LUA, 1, config);
	const auto handle = bridge ? bridge->CreateField(EnvironmentIndex(LUA, 2), config) : 0;
	if (!handle) return Fail(LUA);
	ownedFields.insert(handle); LUA->PushUserType_Value(handle, fieldMeta); return 1;
}
LUA_FUNCTION_STATIC(GetFields)
{
	const int environment = EnvironmentIndex(LUA, 1);
	VJoltGravityVector gravity;
	if (!bridge || !bridge->GetWorld(environment, gravity)) return Fail(LUA);
	LUA->CreateTable();
	for (uint32_t i = 0; i < 1024; ++i)
	{
		VJoltGravityField config; VJoltGravityFieldHandle handle = 0;
		if (!bridge->GetFieldAt(environment, i, handle, config)) break;
		LUA->PushNumber(i + 1); PushField(LUA, handle, config); LUA->SetTable(-3);
	}
	return 1;
}
GMOD_MODULE_OPEN()
{
	bridge = VJoltSDK::DiscoverVersionedInterface<IVJoltGravityBridge>(
		VJOLT_GRAVITY_BRIDGE_VERSION, kVJoltGravityBridgeVersion).api;
	fieldMeta = LUA->CreateMetaTable("JoltGravityField");
	LUA->Push(-1); LUA->SetField(-2, "__index");
	LUA->PushCFunction(FieldDestroy); LUA->SetField(-2, "__gc");
	LUA->PushCFunction(FieldDestroy); LUA->SetField(-2, "Destroy");
	LUA->PushCFunction(FieldValid); LUA->SetField(-2, "IsValid");
	LUA->PushCFunction(FieldGet); LUA->SetField(-2, "GetConfig");
	LUA->PushCFunction(FieldUpdate); LUA->SetField(-2, "Update"); LUA->Pop();
	LUA->PushSpecial(SPECIAL_GLOB); LUA->CreateTable();
#define FUNCTION(name) LUA->PushCFunction(name); LUA->SetField(-2, #name)
	FUNCTION(Available); FUNCTION(GetWorld); FUNCTION(SetWorld); FUNCTION(GetObject); FUNCTION(Configure);
	FUNCTION(SetFactor); FUNCTION(SetGravity); FUNCTION(GetEffectiveGravity); FUNCTION(Sample); FUNCTION(Reset);
	FUNCTION(CreateField); FUNCTION(GetRevision); FUNCTION(GetFields);
#undef FUNCTION
	LUA->SetField(-2, "joltgravity"); LUA->Pop(); return 0;
}
GMOD_MODULE_CLOSE()
{
	if (bridge) for (const auto handle : ownedFields) bridge->DestroyField(handle);
	ownedFields.clear(); bridge = nullptr; return 0;
}
