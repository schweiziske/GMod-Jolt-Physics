-- Server-side consumer of the installed physics engine's gravity bridge.
local Attractor = {}
Attractor.__index = Attractor

function Attractor.Create(settings, environment)
    require("jolt")
    local available, reason = jolt.IsAvailable("gravity")
    if not available then return nil, reason end
    local field, detail = jolt.gravity.CreateField({
        shape = 0, mode = 1, center = settings.center,
        radius = settings.radius, strength = settings.strength,
        priority = settings.priority,
    }, environment)
    if not field then return nil, detail end
    return setmetatable({field = field, update = {}}, Attractor)
end

function Attractor:SetCenter(center)
    if not self.field then return nil, "attractor is destroyed" end
    self.update.center = center -- Reuse this table when moving the field.
    return self.field:Update(self.update)
end

function Attractor:Destroy()
    if not self.field then return false end
    local removed, reason = self.field:Destroy()
    if removed or not self.field:IsValid() then self.field = nil end
    return removed, reason
end

return Attractor
