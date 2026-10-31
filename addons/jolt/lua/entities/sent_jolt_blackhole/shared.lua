local I = JoltI18n or include("jolt_i18n/init.lua")
ENT.Type = "anim"
ENT.Base = "base_anim"
ENT.PrintName = "#jolt.jolt_black_hole"
ENT.Spawnable = false
ENT.DoNotDuplicate = true
ENT.RenderGroup = RENDERGROUP_TRANSLUCENT

ENT.PhaseFuse = 0
ENT.PhasePull = 1
ENT.PhaseBurst = 2

function ENT:SetupDataTables()
	self:NetworkVar("Int", 0, "Phase")
	self:NetworkVar("Float", 0, "PhaseStart")
	self:NetworkVar("Float", 1, "PhaseEnd")
	self:NetworkVar("Float", 2, "FieldRadius")
end
