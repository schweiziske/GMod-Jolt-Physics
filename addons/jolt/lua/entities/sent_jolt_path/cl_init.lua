include("shared.lua")
function ENT:Initialize()
    if JoltPath then JoltPath.ClientTracks[self] = true end
end
function ENT:OnReloaded()
    if JoltPath then JoltPath.ClientTracks[self] = true; self.JoltPathData = nil end
end
function ENT:OnRemove()
    if JoltPath then JoltPath.ClientTracks[self] = nil end
end
function ENT:Draw() end
