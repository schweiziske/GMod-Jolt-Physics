local I = JoltI18n or include("jolt_i18n/init.lua")
ENT.Type = "anim"
ENT.Base = "base_anim"
ENT.PrintName = "#jolt.jolt_path_track"
ENT.Spawnable = false
ENT.DoNotDuplicate = false -- Carries the blueprint origin through the stock duplicator.
function ENT:SetupDataTables()
    self:NetworkVar("Float", 0, "Progress")
    self:NetworkVar("Float", 1, "PathVelocity")
    self:NetworkVar("Float", 2, "MotorTarget")
    self:NetworkVar("Float", 3, "MotorSpeed")
    self:NetworkVar("Int", 0, "MotorMode")
    self:NetworkVar("Bool", 0, "PathEnabled")
    self:NetworkVar("Bool", 1, "PathPaused")
end
