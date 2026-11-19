#pragma once

// Borrows a backend already loaded by the engine or the physics wrapper.
#include <cstdint>
#ifdef _WIN32
#include <windows.h>
#elif defined(__linux__)
#include <array>
#include <dlfcn.h>
#include <link.h>
#include <string>
#include <string_view>
#endif

namespace VJoltSDK
{
using InterfaceFactory = void *(*)(const char *, int *);

#ifdef _WIN32
using BackendModule = HMODULE;
inline constexpr const char *BackendModuleNames[] = {
    "vphysics_jolt_avx2.dll",
    "vphysics_jolt_sse42.dll",
    "vphysics_jolt_sse2.dll",
    "vphysics_jolt.dll",
    "vphysics.dll",
};
#elif defined(__linux__)
using BackendModule = void *;
inline constexpr const char *BackendModuleNames[] = {
    "vphysics_jolt_avx2.so",
    "vphysics_jolt_sse42.so",
    "vphysics_jolt_sse2.so",
    "vphysics_jolt.so",
    "vphysics.so",
    "vphysics_srv.so",
};
#endif

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

inline InterfaceFactory GetFactory(BackendModule module)
{
    if (!module) return nullptr;
#ifdef _WIN32
    return reinterpret_cast<InterfaceFactory>(GetProcAddress(module, "CreateInterface"));
#elif defined(__linux__)
    return reinterpret_cast<InterfaceFactory>(dlsym(module, "CreateInterface"));
#endif
}

// Compound discoveries (Trace plus Source collision/material interfaces) can
// inspect each loaded backend together. Returning true stops the traversal.
template <typename Visitor>
bool VisitLoadedBackends(Visitor visitor)
{
#ifdef _WIN32
    for (const char *name : BackendModuleNames)
        if (const BackendModule module = GetModuleHandleA(name))
            if (visitor(module, name)) return true;
#elif defined(__linux__)
    // The wrapper loads by path. Use the dynamic linker's recorded path to
    // find it even when the library does not have a SONAME.
    using Paths = std::array<std::string, std::size(BackendModuleNames)>;
    Paths paths;
    dl_iterate_phdr([](dl_phdr_info *info, size_t, void *data) {
        auto &loaded = *static_cast<Paths *>(data);
        const std::string_view path(info->dlpi_name);
        const auto filename = path.substr(path.find_last_of('/') + 1);
        for (size_t i = 0; i < loaded.size(); ++i)
            if (filename == BackendModuleNames[i] && loaded[i].empty())
                loaded[i] = path;
        return 0;
    }, &paths);

    for (size_t i = 0; i < paths.size(); ++i)
    {
        if (paths[i].empty()) continue;
        if (const BackendModule module = dlopen(paths[i].c_str(), RTLD_NOW | RTLD_NOLOAD))
        {
            const bool found = visitor(module, BackendModuleNames[i]);
            // Release our reference. The engine or wrapper owns the module.
            dlclose(module);
            if (found) return true;
        }
    }
#endif
    return false;
}

// Also usable with Source's standard interfaces or an explicitly chosen DLL.
// Interfaces such as IVJoltCandidates have no numeric GetVersion method.
template <typename Interface>
Interface *QueryInterface(BackendModule module, const char *name)
{
    const auto factory = GetFactory(module);
    return factory ? static_cast<Interface *>(factory(name, nullptr)) : nullptr;
}

template <typename Interface>
struct DiscoveryResult
{
    Interface *api = nullptr;
    BackendModule module = nullptr;
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
    VisitLoadedBackends([&](BackendModule module, const char *moduleName) {
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
