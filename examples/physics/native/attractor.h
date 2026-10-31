#pragma once
#include <vjolt/gravity.h>

// Owner thread, outside Simulate. Destroy before the physics backend unloads.
class Attractor
{
public:
    Attractor() = default;
    ~Attractor() { Destroy(); }
    Attractor(const Attractor &) = delete;
    Attractor &operator=(const Attractor &) = delete;
    bool Create(int32_t environment, const VJoltGravityField &settings, const char *&error);
    bool SetCenter(const VJoltGravityVector &center);
    bool Destroy();
private:
    IVJoltGravityBridge *api_ = nullptr; // Borrowed from the loaded backend.
    VJoltGravityFieldHandle handle_ = 0;
    VJoltGravityField settings_;
};
