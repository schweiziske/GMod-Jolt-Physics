-- Run on the server after integrating the example bridge and installing the DLL.
require("joltfeature")
local available, reason = joltfeature.Available()
assert(available, reason)
local counts, error = joltfeature.GetBodyCounts(0)
assert(counts, error)
print("Jolt bodies:", counts.total, "active rigid:", counts.activeRigid,
    "active soft:", counts.activeSoft)
