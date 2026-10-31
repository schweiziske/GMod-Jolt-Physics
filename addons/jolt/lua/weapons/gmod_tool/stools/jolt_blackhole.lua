local I = JoltI18n or include("jolt_i18n/init.lua")
TOOL.Category = "Jolt"
TOOL.Name = "#tool.jolt_blackhole.name"
local settings = include("jolt_gameplay/field_settings.lua")
TOOL.ClientConVar = {}
for _, definition in ipairs(settings.Blackhole) do
	TOOL.ClientConVar[definition.key] = tostring(definition.default)
end
TOOL.Information = {
	{name = "left"},
	{name = "right"},
}

if CLIENT then
	I.RegisterAlias("tool.jolt_blackhole.name", "jolt.black_hole")
	I.RegisterAlias("tool.jolt_blackhole.desc", "jolt.pulls_nearby_objects_in_then_pushes_them_away")
	I.RegisterAlias("tool.jolt_blackhole.left", "jolt.place_a_black_hole_on_the_targeted_surface")
	I.RegisterAlias("tool.jolt_blackhole.right", "jolt.remove_your_most_recently_placed_black_hole")
end

local gameplay = SERVER and include("jolt_gameplay/server.lua")
local blackhole = SERVER and include("jolt_gameplay/blackhole.lua")

function TOOL:LeftClick(trace)
	if not trace.Hit or trace.HitSky then
		return false
	end
	if CLIENT then
		return true
	end

	local settings = {
		radius = self:GetClientNumber("radius"),
		pull = self:GetClientNumber("pull"),
		burst = self:GetClientNumber("burst"),
		duration = self:GetClientNumber("duration"),
	}
	local position = trace.HitPos + trace.HitNormal * 8
	local grenade, reason = blackhole.Spawn(self:GetOwner(), position, nil, settings)
	if not grenade then
		return gameplay.Fail(self:GetOwner(), reason)
	end
	self.LastBlackhole = grenade
	return true
end

function TOOL:RightClick()
	if CLIENT then
		return true
	end
	local grenade = self.LastBlackhole
	if not IsValid(grenade) or grenade:GetCreator() ~= self:GetOwner() then
		return false
	end
	grenade:Remove()
	self.LastBlackhole = nil
	return true
end

function TOOL.BuildCPanel(panel)
	settings.BuildBlackholePanel(panel)
	I.Control(panel, "Help", I.Token("jolt.lmb_places_a_black_hole_on_a_surface_rmb_or_z_removes_it_you_can_als"))
end

if CLIENT then
	local accent = Color(130, 210, 255)
	surface.CreateFont("JoltBlackholeHudTitle", {font = "Microsoft YaHei", size = 25, weight = 700, extended = true, antialias = true})
	surface.CreateFont("JoltBlackholeHudText", {font = "Microsoft YaHei", size = 19, weight = 400, extended = true, antialias = true})
	function TOOL:DrawHUD()
		local x, y = ScrW() < 1280 and 12 or 24, math.floor(ScrH() * 0.5) - 24
		local values = settings.ReadPlayer(settings.Blackhole, LocalPlayer())
		draw.SimpleText(I.Phrase("jolt.jolt_black_hole"), "JoltBlackholeHudTitle", x, y, accent)
		draw.SimpleText(I.Phrase("jolt.lmb_place_black_hole_rmb_remove_latest_black_hole_z_undo"), "JoltBlackholeHudText", x, y + 34, color_white)
		draw.SimpleText(I.Format("jolt.pull_0f_radius_0f_q_panel_adjust_settings", values.pull, values.radius), "JoltBlackholeHudText", x, y + 62, accent)
	end
end
