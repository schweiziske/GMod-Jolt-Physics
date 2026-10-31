include("shared.lua")

function ENT:Initialize()
	self:DrawShadow(false)
	self:EnableCustomCollisions(true)
	self:SetCustomCollisionCheck(true)
end

function ENT:Draw()
	local client = JoltSoftBodies and JoltSoftBodies.Client
	if client then client.Draw(self) end
end

function ENT:OnRemove()
	local client = JoltSoftBodies and JoltSoftBodies.Client
	if client then client.Unbind(self) end
end
