AddCSLuaFile("shared.lua")
AddCSLuaFile("cl_init.lua")
include("shared.lua")

function ENT:Initialize()
	self:SetModel("models/hunter/blocks/cube025x025x025.mdl")
	self:SetMoveType(MOVETYPE_NONE)
	-- Source registers BBOX entities in its solid partition. Custom ray/box
	-- clipping then queries the live soft mesh instead of this broadphase box.
	self:SetSolid(SOLID_BBOX)
	self:EnableCustomCollisions(true)
	self:SetCustomCollisionCheck(true)
	self:DrawShadow(false)
end

function ENT:OnTakeDamage(info)
	local record = self.JoltSoftBodyRecord
	if not record or not record.body:IsValid() then return end
	local force = info:GetDamageForce()
	local length = force:Length()
	if length < 0.01 then return end
	local magnitude = math.min(length, math.max(1, info:GetDamage()) * math.min(record.settings.mass, 50) * 3)
	local radius = math.max(16, record.settings.width * 0.18)
	record.body:AddImpulseAt(info:GetDamagePosition(), force * (magnitude / length), radius)
	record.forceSample = true
end

function ENT:OnRemove()
	if JoltSoftBodies and JoltSoftBodies.Server then JoltSoftBodies.Server.Remove(self:GetSoftBodyID()) end
end
