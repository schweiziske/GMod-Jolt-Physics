#pragma once

#include <cstdint>
#include <type_traits>

// Example contract. Copy into public/vjolt/ when integrating the backend.
// Call on the environment's owner thread, outside simulation/contact callbacks.
// No Jolt types or ownership cross the DLL boundary.
#define VJOLT_BODY_COUNTS_VERSION "VJoltBodyCounts001"
inline constexpr uint32_t kVJoltBodyCountsVersion = 1;

struct VJoltBodyCounts
{
    uint32_t size = sizeof(VJoltBodyCounts);
    uint32_t total = 0;
    uint32_t activeRigid = 0;
    uint32_t activeSoft = 0;
};
static_assert(sizeof(VJoltBodyCounts) == 16);
static_assert(std::is_trivially_copyable_v<VJoltBodyCounts>);

class IVJoltBodyCounts
{
public:
    virtual uint32_t GetVersion() const = 0;
    virtual bool Read(int32_t environment, VJoltBodyCounts &out) const = 0;
protected:
    ~IVJoltBodyCounts() = default; // Borrowed singleton; never delete through this interface.
};
