#include <GarrysMod/Lua/Interface.h>
#include <vjolt/path.h>
#include <vjolt/loader.h>
#include <array>
#include <cmath>
#include <cstring>
#include <unordered_set>

using namespace GarrysMod::Lua;
namespace
{
IVJoltPathBridge *bridge = nullptr;
int pathMeta = 0;
std::unordered_set<VJoltPathHandle> owned;
const char *const modes[] = {"off", "velocity", "position"};
const char *const rotations[] = {"free", "tangent", "normal", "binormal", "follow", "fixed"};
const char *Reason(VJoltPathError error)
{
    switch (error)
    {
    case VJoltPathError::None: return "ok";
    case VJoltPathError::WrongThread: return "path calls must use the physics owner thread";
    case VJoltPathError::Busy: return "physics is stepping or this path is being rewound";
    case VJoltPathError::InvalidBody: return "invalid, stale or identical follower/reference PhysObj";
    case VJoltPathError::DifferentEnvironment: return "path members must share one physics environment";
    case VJoltPathError::UnsupportedBody: return "path members cannot be players, vehicles, triggers or shadow-controlled bodies";
    case VJoltPathError::InvalidSettings: return "invalid path settings or motor value";
    case VJoltPathError::InvalidNodes: return "path requires 2..32 finite nodes; a loop requires at least 3";
    case VJoltPathError::DegeneratePath: return "adjacent path nodes are too close or make a near-180-degree reversal";
    case VJoltPathError::LimitExceeded: return "path count or length limit exceeded";
    default: return "path is no longer valid";
    }
}
int Failure(ILua *lua, VJoltPathError error)
{
    lua->PushNil(); lua->PushString(bridge ? Reason(error) : "matching VJoltPathBridge001 backend is not loaded"); return 2;
}
float Number(ILua *lua, int index, const char *key, float fallback)
{
    if (lua->IsType(index, Type::Nil) || lua->IsType(index, Type::None)) return fallback;
    lua->GetField(index, key);
    const float value = lua->IsType(-1, Type::Nil) ? fallback : float(lua->CheckNumber(-1));
    lua->Pop(); if (!std::isfinite(value)) lua->ThrowError("path numbers must be finite"); return value;
}
bool Boolean(ILua *lua, int index, const char *key, bool fallback)
{
    if (lua->IsType(index, Type::Nil) || lua->IsType(index, Type::None)) return fallback;
    lua->GetField(index, key);
    if (lua->IsType(-1, Type::Nil)) { lua->Pop(); return fallback; }
    lua->CheckType(-1, Type::Bool); const bool value = lua->GetBool(-1); lua->Pop(); return value;
}
uint32_t Enum(ILua *lua, int index, const char *key, const char *const *names, uint32_t count, uint32_t fallback)
{
    if (lua->IsType(index, Type::Nil) || lua->IsType(index, Type::None)) return fallback;
    lua->GetField(index, key);
    if (lua->IsType(-1, Type::Nil)) { lua->Pop(); return fallback; }
    const char *value = lua->CheckString(-1);
    for (uint32_t i = 0; i < count; ++i) if (std::strcmp(value, names[i]) == 0) { lua->Pop(); return i; }
    lua->ThrowError("unknown path rotation or motor mode"); return fallback;
}
VJoltPathMotor Motor(ILua *lua, int index)
{
    VJoltPathMotor m;
    m.mode = VJoltPathMotorMode(Enum(lua, index, "mode", modes, 3, 0));
    m.speed = Number(lua, index, "speed", m.speed); m.target = Number(lua, index, "target", m.target);
    m.maxForce = Number(lua, index, "maxForce", m.maxForce); m.friction = Number(lua, index, "friction", m.friction);
    m.frequency = Number(lua, index, "frequency", m.frequency); m.damping = Number(lua, index, "damping", m.damping);
    return m;
}
VJoltPathHandle Handle(ILua *lua)
{
    const auto *value = lua->GetUserType<VJoltPathHandle>(1, pathMeta); return value ? *value : 0;
}
void NumberField(ILua *lua, const char *key, double value) { lua->PushNumber(value); lua->SetField(-2, key); }
void BoolField(ILua *lua, const char *key, bool value) { lua->PushBool(value); lua->SetField(-2, key); }
void Function(ILua *lua, const char *name, CFunc f) { lua->PushCFunction(f); lua->SetField(-2, name); }
LUA_FUNCTION_STATIC(PathsAvailable) { LUA->PushBool(bridge != nullptr); return 1; }
LUA_FUNCTION_STATIC(GetPathVersion) { if (bridge) LUA->PushNumber(bridge->GetVersion()); else LUA->PushNil(); return 1; }
LUA_FUNCTION_STATIC(CreatePath)
{
    LUA->CheckType(1, Type::PhysObj); LUA->CheckType(2, Type::Table);
    auto *follower = LUA->GetUserType<IPhysicsObject>(1, Type::PhysObj);
    if (!LUA->IsType(3, Type::Nil) && !LUA->IsType(3, Type::None)) LUA->CheckType(3, Type::Table);
    IPhysicsObject *reference = nullptr;
    if (!LUA->IsType(4, Type::Nil) && !LUA->IsType(4, Type::None))
    {
        LUA->CheckType(4, Type::PhysObj); reference = LUA->GetUserType<IPhysicsObject>(4, Type::PhysObj);
    }
    const int count = LUA->ObjLen(2);
    if (count < 2 || count > int(kVJoltPathMaxNodes)) return Failure(LUA, VJoltPathError::InvalidNodes);
    std::array<VJoltConstraintVector, kVJoltPathMaxNodes> nodes{};
    for (int i = 0; i < count; ++i)
    {
        LUA->PushNumber(i + 1); LUA->GetTable(2); LUA->CheckType(-1, Type::Vector);
        const auto *v = LUA->GetUserType<VJoltConstraintVector>(-1, Type::Vector);
        if (!v) return Failure(LUA, VJoltPathError::InvalidNodes);
        nodes[i] = *v; LUA->Pop();
    }
    VJoltPathSettings s;
    s.flags = (Boolean(LUA, 3, "closed", false) ? VJoltPathClosed : 0)
        | (Boolean(LUA, 3, "snap", true) ? VJoltPathSnapToPath : 0);
    s.rotation = VJoltPathRotation(Enum(LUA, 3, "rotation", rotations, 6, 4));
    s.smoothing = Number(LUA, 3, "smoothing", s.smoothing);
    const auto m = Motor(LUA, 3);
    VJoltPathError error{};
    const auto handle = bridge ? bridge->Create(follower, reference, nodes.data(), uint32_t(count), s, m, &error) : 0;
    if (!handle) return Failure(LUA, error);
    owned.insert(handle); LUA->PushUserType_Value(handle, pathMeta); return 1;
}
LUA_FUNCTION_STATIC(PathIsValid) { LUA->PushBool(bridge && bridge->IsValid(Handle(LUA))); return 1; }
LUA_FUNCTION_STATIC(PathDestroy)
{
    auto *h = LUA->GetUserType<VJoltPathHandle>(1, pathMeta);
    const bool result = h && *h && bridge && bridge->Destroy(*h);
    if (h) { owned.erase(*h); *h = 0; }
    LUA->PushBool(result); return 1;
}
LUA_FUNCTION_STATIC(PathSetMotor)
{
    LUA->CheckType(2, Type::Table); const auto m = Motor(LUA, 2); VJoltPathError error{};
    if (!bridge || !bridge->SetMotor(Handle(LUA), m, &error)) return Failure(LUA, error);
    LUA->PushBool(true); return 1;
}
LUA_FUNCTION_STATIC(PathGetState)
{
    VJoltPathState s;
    if (!bridge || !bridge->GetState(Handle(LUA), s)) return Failure(LUA, VJoltPathError::Stale);
    LUA->CreateTable(); LUA->PushString("path"); LUA->SetField(-2, "kind");
    BoolField(LUA, "enabled", s.enabled != 0); BoolField(LUA, "closed", s.closed != 0);
    NumberField(LUA, "nodeCount", s.nodeCount); NumberField(LUA, "progress", s.progress);
    NumberField(LUA, "distance", s.distance); NumberField(LUA, "length", s.length); NumberField(LUA, "velocity", s.velocity);
    LUA->CreateTable(); LUA->PushString(modes[uint32_t(s.motor.mode)]); LUA->SetField(-2, "mode");
    NumberField(LUA, "speed", s.motor.speed); NumberField(LUA, "target", s.motor.target);
    NumberField(LUA, "maxForce", s.motor.maxForce); NumberField(LUA, "friction", s.motor.friction);
    NumberField(LUA, "frequency", s.motor.frequency); NumberField(LUA, "damping", s.motor.damping);
    LUA->SetField(-2, "motor"); return 1;
}
LUA_FUNCTION_STATIC(PathSample)
{
    const double count = LUA->IsType(2, Type::None) || LUA->IsType(2, Type::Nil) ? 129 : LUA->CheckNumber(2);
    if (!std::isfinite(count) || count < 2 || count > 513 || count != std::floor(count))
        return Failure(LUA, VJoltPathError::InvalidSettings);
    std::array<VJoltPathSample, 513> samples;
    if (!bridge || !bridge->Sample(Handle(LUA), uint32_t(count), samples.data())) return Failure(LUA, VJoltPathError::Stale);
    LUA->CreateTable();
    for (int i = 0; i < int(count); ++i)
    {
        LUA->PushNumber(i + 1); LUA->CreateTable();
        LUA->PushVector(*reinterpret_cast<const Vector *>(&samples[i].position)); LUA->SetField(-2, "position");
        LUA->PushVector(*reinterpret_cast<const Vector *>(&samples[i].tangent)); LUA->SetField(-2, "tangent");
        LUA->PushVector(*reinterpret_cast<const Vector *>(&samples[i].normal)); LUA->SetField(-2, "normal");
        LUA->SetTable(-3);
    }
    return 1;
}
}

// Called with the existing joltconstraints table on top of the Lua stack.
void VJoltPathLuaOpen(ILua *lua)
{
    bridge = VJoltSDK::DiscoverVersionedInterface<IVJoltPathBridge>(VJOLT_PATH_BRIDGE_VERSION, kVJoltPathBridgeVersion).api;
    pathMeta = lua->CreateMetaTable("JoltPath"); lua->Push(-1); lua->SetField(-2, "__index");
    Function(lua, "__gc", PathDestroy); Function(lua, "Destroy", PathDestroy); Function(lua, "IsValid", PathIsValid);
    Function(lua, "SetMotor", PathSetMotor); Function(lua, "GetState", PathGetState); Function(lua, "Sample", PathSample);
    lua->Pop();
    Function(lua, "PathsAvailable", PathsAvailable); Function(lua, "GetPathVersion", GetPathVersion); Function(lua, "CreatePath", CreatePath);
}
void VJoltPathLuaClose()
{
    if (bridge) for (auto h : owned) bridge->Destroy(h);
    owned.clear(); bridge = nullptr;
}
