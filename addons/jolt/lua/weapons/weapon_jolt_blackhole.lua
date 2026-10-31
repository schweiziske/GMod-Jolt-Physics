local I = JoltI18n or include("jolt_i18n/init.lua")
if SERVER then
	AddCSLuaFile()
end

SWEP.Base = "weapon_base"
SWEP.PrintName = "#jolt.jolt_black_hole_grenade"
SWEP.Category = "Jolt"
SWEP.Spawnable = true
SWEP.AdminOnly = false
SWEP.UseHands = true
SWEP.ViewModel = "models/weapons/c_grenade.mdl"
SWEP.WorldModel = "models/weapons/w_grenade.mdl"
SWEP.HoldType = "grenade"
SWEP.Instructions = "#jolt.lmb_throws_pulls_nearby_objects_in_then_pushes_them_away_r_sets_radi"
SWEP.DrawAmmo = false
SWEP.Primary = {ClipSize = -1, DefaultClip = -1, Automatic = false, Ammo = "none"}
SWEP.Secondary = {ClipSize = -1, DefaultClip = -1, Automatic = false, Ammo = "none"}

if CLIENT then
	I.RegisterAlias("Cleanup_jolt_blackholes", "jolt.jolt_black_hole")
	I.RegisterAlias("Cleaned_jolt_blackholes", "jolt.jolt_black_holes_cleaned_up")
	I.RegisterAlias("SBoxLimit_jolt_blackholes", "jolt.black_hole_limit_reached")
end

local gameplay = SERVER and include("jolt_gameplay/server.lua")
local blackhole = SERVER and include("jolt_gameplay/blackhole.lua")
local settings = include("jolt_gameplay/field_settings.lua")

function SWEP:Initialize()
	self:SetHoldType(self.HoldType)
end

function SWEP:PrimaryAttack()
	self:SetNextPrimaryFire(CurTime() + 1)
	if CLIENT then
		return
	end

	local owner = self:GetOwner()
	if not IsValid(owner) or not owner:IsPlayer() or not owner:Alive() then
		return false
	end

	local direction = owner:GetAimVector()
	local origin = owner:GetShootPos()
	local trace = util.TraceHull({
		start = origin,
		endpos = origin + direction * 24,
		mins = Vector(-4, -4, -4),
		maxs = Vector(4, 4, 4),
		filter = owner,
		mask = MASK_SOLID,
	})
	if trace.StartSolid then
		return false
	end

	local velocity = owner:GetVelocity() + direction * 700 + Vector(0, 0, 120)
	local grenade, reason = blackhole.Spawn(owner, trace.HitPos, velocity, settings.ReadPlayer(settings.Blackhole, owner))
	if not grenade then
		return gameplay.Fail(owner, reason)
	end
	self:SendWeaponAnim(ACT_VM_THROW)
	self.RedrawAt = CurTime() + 0.6
	owner:SetAnimation(PLAYER_ATTACK1)
	return true
end

function SWEP:SecondaryAttack()
end

function SWEP:Reload()
	if (self.NextSettingsOpen or 0) > CurTime() then return end
	if CLIENT then
		if not IsFirstTimePredicted() then return end
		self:OpenFieldSettings()
	elseif game.SinglePlayer() then
		-- Predicted SWEP hooks do not run clientside in singleplayer.
		self:CallOnClient("OpenFieldSettings")
	end
	self.NextSettingsOpen = CurTime() + 0.3
end

function SWEP:OpenFieldSettings()
	if CLIENT and self:GetOwner() == LocalPlayer() then settings.OpenBlackholePanel() end
end

if CLIENT then
	local hudColor = Color(130, 210, 255)
	function SWEP:DrawHUD()
		local width, height = ScrW(), ScrH()
		local x, y = width < 1280 and 12 or 24, math.floor(height * 0.5) - 12
		local values = settings.ReadPlayer(settings.Blackhole, LocalPlayer())
		draw.SimpleText(I.Phrase("jolt.jolt_black_hole_grenade_lmb_throw_r_radius_duration"), "DermaDefaultBold", x, y, hudColor)
		draw.SimpleText(I.Format("jolt.radius_0f_pull_1f_s_z_undo",
			values.radius, values.duration),
			"DermaDefault", x, y + 24, color_white)
	end
end

function SWEP:Think()
	if SERVER and self.RedrawAt and CurTime() >= self.RedrawAt then
		self.RedrawAt = nil
		self:SendWeaponAnim(ACT_VM_DRAW)
	end
end
