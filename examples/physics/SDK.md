<!-- Installed as share/doc/VJoltSDK/README.md by the SDK component (see gmsv_jolt/CMakeLists.txt). Links are relative to that installed layout and do not resolve from examples/physics/ in the source tree. -->
# VJolt Physics SDK

[Home](https://github.com/schweiziske/GMod-Jolt-Physics)

This package contains the public C++ interfaces of GMod Jolt Physics: the headers in `include/vjolt/` and the CMake target `VJolt::SDK`. Add the package prefix to `CMAKE_PREFIX_PATH`, then:

```cmake
find_package(VJoltSDK CONFIG REQUIRED)
target_link_libraries(my_module PRIVATE VJolt::SDK)
```

- [Physics API](docs/physics-api.md): use the existing features from Lua and C++.
- [Public interfaces](public/vjolt/README.md): headers, versions and lifetime rules.
- [Examples](examples/physics/README.md): Lua and C++ consumers.
- [Feature export example](examples/physics/export/README.md): add a new interface, from the backend to a Lua module.

The headers only describe interfaces; the physics backend running on the server implements them. The Lua examples need the matching backend, the feature's `gmsv_jolt*` module and `addons/jolt` on the server. The export example changes the backend itself, so it needs a checkout of the source repository; point its module build at it with `-DVJOLT_ROOT=<checkout>`.
