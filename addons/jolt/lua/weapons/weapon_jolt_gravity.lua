local I = JoltI18n or include("jolt_i18n/init.lua")
if SERVER then AddCSLuaFile() end

SWEP.Base = "weapon_base"
SWEP.PrintName = "#jolt.jolt_gravity_controller_319"
SWEP.Category = "Jolt"
SWEP.Instructions = "#jolt.hold_e_and_move_the_mouse_to_adjust_force_direction_release_e_to_sav"
SWEP.Spawnable = true
SWEP.UseHands = true
SWEP.ViewModel = "models/weapons/c_physcannon.mdl"
SWEP.WorldModel = "models/weapons/w_physics.mdl"
SWEP.HoldType = "physgun"
SWEP.DrawAmmo = false
SWEP.Primary.ClipSize = -1
SWEP.Primary.DefaultClip = -1
SWEP.Primary.Automatic = false
SWEP.Primary.Ammo = "none"
SWEP.Secondary.ClipSize = -1
SWEP.Secondary.DefaultClip = -1
SWEP.Secondary.Automatic = false
SWEP.Secondary.Ammo = "none"

local settings = include("jolt_gameplay/field_settings.lua")

function SWEP:SetupDataTables()
    self:NetworkVar("Int", 0, "GravityMode")
    self:NetworkVar("Bool", 0, "BoxField")
    self:NetworkVar("Angle", 0, "ForceAngle")
end
function SWEP:Initialize()
    self:SetHoldType("physgun")
    if SERVER then self:SetForceAngle(Angle(90, 0, 0)) end
end

function SWEP:Holster()
    if CLIENT and JoltGravity and JoltGravity.Editor then JoltGravity.Editor.Cancel(self) end
    return true
end

function SWEP:OnRemove()
    if CLIENT and JoltGravity and JoltGravity.Editor then JoltGravity.Editor.Cancel(self) end
end

function SWEP:OwnerChanged()
    if CLIENT and JoltGravity and JoltGravity.Editor then JoltGravity.Editor.Cancel(self) end
    self:RestrictMode()
end

function SWEP:RestrictMode()
    local owner = self:GetOwner()
    if SERVER and IsValid(owner) and not owner:IsAdmin() and self:GetGravityMode() ~= 2 then self:SetGravityMode(2) end
end

function SWEP:Deploy()
    self:RestrictMode()
    return true
end

function SWEP:Think()
    self:RestrictMode()
end
local function tell(player, message)
    if IsValid(player) and message then
        I.Chat(player, I.Token("jolt.jolt_gravity") .. tostring(message))
    end
end

function SWEP:Allowed(trace)
    local owner = self:GetOwner()
    if not IsValid(owner) or not owner:IsPlayer() or not owner:Alive() then return false end
    if not JoltGravity or not joltgravity or not joltgravity.Available() then
        tell(owner, I.Phrase("jolt.gravity_module_unavailable_check_that_the_server_has_the_matching_mo"))
        return false
    end
    self:RestrictMode()
    if hook.Run("CanTool", owner, trace, "jolt_gravity", nil, 1) == false then return false end
    return true
end

function SWEP:PrimaryAttack()
    self:SetNextPrimaryFire(CurTime() + 0.25)
    if CLIENT then return end
    local owner = self:GetOwner()
    if not IsValid(owner) or owner:KeyDown(IN_USE) then return end
    local trace = owner:GetEyeTrace()
    if not self:Allowed(trace) then return end
    local values = settings.ReadPlayer(settings.Gravity, owner)
    local gravity = self:GetForceAngle():Forward() * values.strength
    local mode = self:GetGravityMode()
    if mode == 0 then
        local previous = joltgravity.GetWorld()
        local ok, err = joltgravity.SetWorld(gravity)
        if not ok then tell(owner, err); return end
        undo.Create("jolt.jolt_world_gravity")
        undo.AddFunction(function(_, value) joltgravity.SetWorld(value) end, previous)
    elseif mode == 1 then
        if not IsValid(trace.Entity) or trace.Entity:IsWorld() then return end
        local previous, err = JoltGravity.SetEntity(trace.Entity, gravity)
        if not previous then tell(owner, err); return end
        undo.Create("jolt.jolt_object_gravity")
        undo.AddFunction(function(_, old) JoltGravity.RestoreObjects(old) end, previous)
    else
        if not trace.Hit then return end
        local radius = values.radius
        local field, err = JoltGravity.AddField(owner, {
            center = trace.HitPos, gravity = gravity, radius = radius,
            halfExtent = Vector(radius, radius, radius), shape = self:GetBoxField() and 1 or 0
        })
        if not field then tell(owner, err); return end
        undo.Create("jolt.jolt_gravity_region")
        undo.AddFunction(function(_, value) value:Destroy() end, field)
    end
    undo.SetPlayer(owner); undo.Finish()
    self:EmitSound("weapons/physcannon/energy_sing_explosion2.wav", 60, 120)
end

function SWEP:SecondaryAttack()
    self:SetNextSecondaryFire(CurTime() + 0.25)
    if CLIENT then return end
    local owner = self:GetOwner()
    if not IsValid(owner) or owner:KeyDown(IN_USE) then return end
    local trace = owner:GetEyeTrace()
    if not self:Allowed(trace) then return end
    local mode = self:GetGravityMode()
    if mode == 0 then joltgravity.SetWorld(JoltGravity.OriginalWorld or Vector(0, 0, -GetConVar("sv_gravity"):GetFloat()))
    elseif mode == 1 then
        local ok, err = JoltGravity.SetEntity(trace.Entity, nil, 1)
        if not ok then tell(owner, err) end
    else JoltGravity.RemoveLastField(owner) end
end

function SWEP:Reload()
    if CLIENT or (self.NextGravityReload or 0) > CurTime() then return end
    self.NextGravityReload = CurTime() + 0.3
    local owner = self:GetOwner()
    if not IsValid(owner) or owner:KeyDown(IN_USE) then return end
    if not owner:IsAdmin() then self:RestrictMode(); return end
    self:SetGravityMode((self:GetGravityMode() + 1) % 3)
end

if SERVER then
    util.AddNetworkString("jolt_gravity_direction_v1")
    net.Receive("jolt_gravity_direction_v1", function(_, owner)
        local weapon, sequence = net.ReadEntity(), net.ReadUInt(16)
        local pitch, yaw = net.ReadFloat(), net.ReadFloat()
        if not IsValid(owner) or not owner:IsPlayer() or not owner:Alive()
            or not IsValid(weapon) or weapon:GetClass() ~= "weapon_jolt_gravity"
            or weapon:GetOwner() ~= owner or owner:GetActiveWeapon() ~= weapon then return end

        local now = CurTime()
        local accepted = pitch == pitch and yaw == yaw and math.abs(pitch) <= 90 and math.abs(yaw) <= 180
            and now >= (owner.JoltNextDirectionUpdate or 0)
        if accepted then
            owner.JoltNextDirectionUpdate = now + 0.05
            weapon:SetForceAngle(Angle(pitch, yaw, 0))
        end
        -- Acknowledge even rejected values so the client never fires with an
        -- unconfirmed preview, or stays stuck waiting on a rate-limited edit.
        local angle = weapon:GetForceAngle()
        net.Start("jolt_gravity_direction_v1")
        net.WriteEntity(weapon); net.WriteUInt(sequence, 16)
        net.WriteFloat(angle.p); net.WriteFloat(angle.y); net.WriteBool(accepted)
        net.Send(owner)
    end)
    hook.Add("PlayerUse", "JoltGravity.Direction", function(owner)
        if not owner:Alive() then return end
        local weapon = owner:GetActiveWeapon()
        if IsValid(weapon) and weapon:GetClass() == "weapon_jolt_gravity" then return false end
    end)
else
    function SWEP:DrawHUD()
        if JoltGravity and JoltGravity.Editor then JoltGravity.Editor.DrawHUD(self) end
    end
end
