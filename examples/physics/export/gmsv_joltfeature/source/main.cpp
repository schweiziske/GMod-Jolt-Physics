#include <GarrysMod/Lua/Interface.h>
#include <vjolt/body_counts.h>
#include <vjolt/loader.h>

#include <cmath>

using namespace GarrysMod::Lua;

namespace
{
IVJoltBodyCounts *g_Bridge = nullptr;
const char *g_DiscoveryError = "module has not been initialized";

int Failure(ILua *lua, const char *reason)
{
    lua->PushNil();
    lua->PushString(reason);
    return 2;
}

LUA_FUNCTION_STATIC(Available)
{
    LUA->PushBool(g_Bridge != nullptr);
    if (g_Bridge) return 1;
    LUA->PushString(g_DiscoveryError);
    return 2;
}

LUA_FUNCTION_STATIC(GetBodyCounts)
{
    double index = 0;
    if (LUA->Top() >= 1 && !LUA->IsType(1, Type::Nil)) index = LUA->CheckNumber(1);
    if (!std::isfinite(index) || index < 0 || index > 4096 || index != std::floor(index))
        return Failure(LUA, "environment must be an integer between 0 and 4096");
    if (!g_Bridge) return Failure(LUA, g_DiscoveryError);

    VJoltBodyCounts counts;
    if (!g_Bridge->Read(static_cast<int32_t>(index), counts))
        return Failure(LUA, "environment is missing, stepping, or in a contact callback");

    LUA->CreateTable();
    LUA->PushNumber(counts.total); LUA->SetField(-2, "total");
    LUA->PushNumber(counts.activeRigid); LUA->SetField(-2, "activeRigid");
    LUA->PushNumber(counts.activeSoft); LUA->SetField(-2, "activeSoft");
    return 1;
}
}

GMOD_MODULE_OPEN()
{
    const auto found = VJoltSDK::DiscoverVersionedInterface<IVJoltBodyCounts>(
        VJOLT_BODY_COUNTS_VERSION, kVJoltBodyCountsVersion);
    g_Bridge = found.api;
    g_DiscoveryError = found.ErrorMessage();
    LUA->PushSpecial(SPECIAL_GLOB);
    LUA->CreateTable();
    LUA->PushCFunction(Available); LUA->SetField(-2, "Available");
    LUA->PushCFunction(GetBodyCounts); LUA->SetField(-2, "GetBodyCounts");
    LUA->SetField(-2, "joltfeature");
    LUA->Pop();
    return 0;
}

GMOD_MODULE_CLOSE()
{
    g_Bridge = nullptr; // The physics backend owns the singleton.
    return 0;
}
