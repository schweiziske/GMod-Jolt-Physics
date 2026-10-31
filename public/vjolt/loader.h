#pragma once

// Discovery for Windows GMod modules. Only borrows already loaded backends;
// never loads a second physics DLL, initializes Jolt, or owns an interface.
#include <cstdint>
#include <windows.h>

namespace VJoltSDK
{
using InterfaceFactory = void *(*)(const char *, int *);

inline constexpr const char *BackendModuleNames[] = {
    "vphysics_jolt_avx2.dll",
    "vphysics_jolt_sse42.dll",
    "vphysics_jolt_sse2.dll",
    "vphysics_jolt.dll",
    "vphysics.dll",
};

enum class DiscoveryError
{
    None,
    ModuleNotLoaded,
    FactoryNotFound,
    InterfaceNotFound,
    VersionMismatch,
};

inline const char *Describe(DiscoveryError error)
{
    switch (error)
    {
    case DiscoveryError::None: return "";
    case DiscoveryError::ModuleNotLoaded: return "no vphysics backend is loaded";
    case DiscoveryError::FactoryNotFound: return "loaded backend has no CreateInterface export";
    case DiscoveryError::InterfaceNotFound: return "loaded backend does not expose the requested interface";
    case DiscoveryError::VersionMismatch: return "backend interface version mismatch; rebuild the backend and Lua module together";
    }
    return "unknown backend discovery error";
}

inline InterfaceFactory GetFactory(HMODULE module)
{
    return module ? reinterpret_cast<InterfaceFactory>(GetProcAddress(module, "CreateInterface")) : nullptr;
}

// Compound discoveries (Trace plus Source collision/material interfaces) can
// inspect each loaded backend together. Returning true stops the traversal.
template <typename Visitor>
bool VisitLoadedBackends(Visitor visitor)
{
    for (const char *name : BackendModuleNames)
        if (const HMODULE module = GetModuleHandleA(name))
            if (visitor(module, name)) return true;
    return false;
}

// Also usable with Source's standard interfaces or an explicitly chosen DLL.
// Interfaces such as IVJoltCandidates have no numeric GetVersion method.
template <typename Interface>
Interface *QueryInterface(HMODULE module, const char *name)
{
    const auto factory = GetFactory(module);
    return factory ? static_cast<Interface *>(factory(name, nullptr)) : nullptr;
}

template <typename Interface>
struct DiscoveryResult
{
    Interface *api = nullptr;
    HMODULE module = nullptr;
    const char *moduleName = nullptr;
    DiscoveryError error = DiscoveryError::ModuleNotLoaded;

    explicit operator bool() const { return api != nullptr; }
    const char *ErrorMessage() const { return Describe(error); }
};

namespace Detail
{
template <typename Interface, typename Validate>
DiscoveryResult<Interface> Discover(const char *name, Validate validate)
{
    DiscoveryResult<Interface> result;
    VisitLoadedBackends([&](HMODULE module, const char *moduleName) {
        DiscoveryError error = DiscoveryError::FactoryNotFound;
        if (const auto factory = GetFactory(module))
        {
            error = DiscoveryError::InterfaceNotFound;
            if (auto *api = static_cast<Interface *>(factory(name, nullptr)))
            {
                if (validate(*api))
                {
                    result = {api, module, moduleName, DiscoveryError::None};
                    return true;
                }
                error = DiscoveryError::VersionMismatch;
            }
        }
        // Preserve the most informative failure if a later DLL lacks a factory
        // or interface. A compatible later backend can still satisfy the request.
        if (error > result.error)
        {
            result.module = module;
            result.moduleName = moduleName;
            result.error = error;
        }
        return false;
    });
    return result;
}
}

template <typename Interface>
DiscoveryResult<Interface> DiscoverInterface(const char *name)
{
    return Detail::Discover<Interface>(name, [](const Interface &) { return true; });
}

template <typename Interface>
DiscoveryResult<Interface> DiscoverVersionedInterface(const char *name, uint32_t expectedVersion)
{
    return Detail::Discover<Interface>(name, [expectedVersion](const Interface &api) {
        return static_cast<uint32_t>(api.GetVersion()) == expectedVersion;
    });
}
}
