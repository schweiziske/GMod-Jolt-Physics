-- A minimal four-vertex cloth in the existing server physics world.
local Cloth = {}
Cloth.__index = Cloth

function Cloth.Create(settings)
    local width, height = settings.width, settings.height
    if type(width) ~= "number" or type(height) ~= "number"
        or not (width > 0 and width < math.huge and height > 0 and height < math.huge) then
        return nil, "cloth needs positive finite width and height"
    end
    require("jolt")
    local available, reason = jolt.IsAvailable("softbody")
    if not available then return nil, reason end
    local mass = settings.vertexMass
    local vertices = {
        {pos = Vector(-width / 2, 0, 0), mass = mass, pinned = true},
        {pos = Vector(width / 2, 0, 0), mass = mass, pinned = true},
        {pos = Vector(-width / 2, 0, -height), mass = mass},
        {pos = Vector(width / 2, 0, -height), mass = mass},
    }
    local options = {}
    for key, value in pairs(settings) do options[key] = value end
    options.pressure = 0 -- This is an open mesh, not a pressure vessel.
    local body, detail = jolt.softbody.Create(vertices, {{1, 3, 2}, {2, 3, 4}}, options)
    if not body then return nil, detail end
    return setmetatable({body = body, snapshot = {positions = {}, pinned = {}}, hasSnapshot = false}, Cloth)
end

function Cloth:Read()
    if not self.body then return nil, "cloth is destroyed" end
    -- The first read must populate even a sleeping body. Later reads can skip it.
    local ok, reason = self.body:ReadSnapshot(self.snapshot, self.hasSnapshot)
    if not ok then return nil, reason end
    if self.snapshot.updated then self.hasSnapshot = true end
    return self.snapshot
end

function Cloth:Destroy()
    if not self.body then return false end
    local removed, reason = self.body:Destroy()
    if removed or not self.body:IsValid() then self.body = nil end
    return removed, reason
end

return Cloth
