AddCSLuaFile("shared.lua")
AddCSLuaFile("cl_init.lua")
include("shared.lua")

local gameplay = include("jolt_gameplay/server.lua")
local fieldSettings = include("jolt_gameplay/field_settings.lua")

local fuseDuration = 1.0
local burstDuration = 0.25

function ENT:SetPhaseWindow(phase, duration)
	local now = CurTime()
	self:SetPhase(phase)
	self:SetPhaseStart(now)
	self:SetPhaseEnd(now + duration)
end

function ENT:Initialize()
	self:SetModel("models/Items/AR2_Grenade.mdl")
	self:PhysicsInit(SOLID_VPHYSICS)
	self:SetMoveType(MOVETYPE_VPHYSICS)
	self:SetSolid(SOLID_VPHYSICS)
	self:SetCollisionGroup(COLLISION_GROUP_PROJECTILE)
	local settings = fieldSettings.ResolveBlackhole(self.FieldSettings or {}, self:GetCreator())
	self:SetFieldRadius(settings.radius)
	self.PullStrength, self.BurstStrength, self.PullDuration = settings.pull, settings.burst, settings.duration
	self.FieldSettings = nil
	self:SetPhaseWindow(self.PhaseFuse, fuseDuration)

	local physics = self:GetPhysicsObject()
	if not IsValid(physics) then
		self:Remove()
		return
	end
	physics:SetMass(5)
	physics:Wake()
end

function ENT:BeginPull()
	local gravity, reason = gameplay.LoadFeature("gravity")
	if not gravity then
		gameplay.Fail(self:GetCreator(), reason)
		self:Remove()
		return false
	end

	local field, detail = gravity.CreateField({
		shape = 0,
		mode = 1,
		center = self:GetPos(),
		radius = self:GetFieldRadius(),
		strength = self.PullStrength,
		priority = 50,
	})
	if not field then
		gameplay.Fail(self:GetCreator(), detail)
		self:Remove()
		return false
	end

	self.GravityField = field
	local physics = self:GetPhysicsObject()
	if IsValid(physics) then
		physics:EnableMotion(false)
	end
	self:SetMoveType(MOVETYPE_NONE)
	self:SetSolid(SOLID_NONE)
	self:SetPhaseWindow(self.PhasePull, self.PullDuration)
	self:EmitSound("ambient/energy/weld1.wav", 70, 70, 0.7)
	return true
end

function ENT:BeginBurst()
	local updated, reason = self.GravityField:Update({mode = 2, strength = self.BurstStrength})
	if not updated then
		gameplay.Fail(self:GetCreator(), reason)
		self:Remove()
		return false
	end
	self:SetPhaseWindow(self.PhaseBurst, burstDuration)
	self:EmitSound("ambient/energy/weld2.wav", 80, 90, 0.8)
	return true
end

function ENT:Think()
	if self.GravityField and not self.GravityField:IsValid() then
		self:Remove()
		return
	end

	local now = CurTime()
	if now >= self:GetPhaseEnd() then
		local phase = self:GetPhase()
		if phase == self.PhaseFuse then
			if not self:BeginPull() then
				return
			end
		elseif phase == self.PhasePull then
			if not self:BeginBurst() then
				return
			end
		else
			self:Remove()
			return
		end
	end
	self:NextThink(now + 0.05)
	return true
end

function ENT:OnRemove()
	local field = self.GravityField
	self.GravityField = nil
	if field then
		-- The backend accepts destruction in physics callbacks and retires safely.
		field:Destroy()
	end
end
