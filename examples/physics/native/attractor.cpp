#include "attractor.h"
#include <vjolt/loader.h>

bool Attractor::Create(int32_t environment, const VJoltGravityField &settings, const char *&error)
{
    error = nullptr;
    if (handle_) { error = "destroy the existing field before creating another"; return false; }
    const auto bridge = VJoltSDK::DiscoverVersionedInterface<IVJoltGravityBridge>(
        VJOLT_GRAVITY_BRIDGE_VERSION, kVJoltGravityBridgeVersion);
    if (!bridge) { error = bridge.ErrorMessage(); return false; }
    auto attractor = settings;
    attractor.shape = VJoltGravitySphere;
    attractor.mode = VJoltGravityAttract;
    const auto handle = bridge.api->CreateField(environment, attractor);
    if (!handle) { error = "backend rejected the field; check environment, settings and simulation timing"; return false; }
    api_ = bridge.api;
    handle_ = handle;
    settings_ = attractor;
    return true;
}

bool Attractor::SetCenter(const VJoltGravityVector &center)
{
    if (!handle_) return false;
    auto settings = settings_;
    settings.center = center;
    if (!api_->UpdateField(handle_, settings)) return false;
    settings_ = settings;
    return true;
}

bool Attractor::Destroy()
{
    if (!handle_) return true;
    if (!api_->DestroyField(handle_)) {
        VJoltGravityField state;
        if (api_->GetField(handle_, state)) return false; // Keep ownership for a retry.
    }
    handle_ = 0;
    api_ = nullptr;
    return true;
}
