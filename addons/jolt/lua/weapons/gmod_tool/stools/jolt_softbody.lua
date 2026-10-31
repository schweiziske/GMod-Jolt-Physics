local I = JoltI18n or include("jolt_i18n/init.lua")
TOOL.Category = "Jolt"
TOOL.Name = "#tool.jolt_softbody.name"
TOOL.RequiresTraceHit = false
TOOL.ClientConVar = {
	mode = "create", kind = "1", quality = "2", width = "160", height = "128", mass = "4",
	softness = "40", inflation = "45", pins = "1", palette = "1", rotation = "0", playercollision = "0",
	bodyiterations = "5", bodydamping = "0.35", bodyfriction = "0.6", bodyrestitution = "0.05", vertexradius = "0.5",
	maxvelocity = "2000", gravity = "1",
}
TOOL.Information = {
	{name = "left", stage = 0}, {name = "left_1", stage = 1},
	{name = "right", stage = 1}, {name = "reload", stage = 1},
}

if CLIENT then
	-- Server limits share the process with client cvars in a listen server.
	-- Keep tool userinfo names distinct, preserving saved values on migration.
	for key, legacy in pairs({bodyiterations = "iterations", bodydamping = "damping",
		bodyfriction = "friction", bodyrestitution = "restitution"}) do
		local previous = GetConVar("jolt_softbody_" .. legacy)
		if previous and not GetConVar("jolt_softbody_" .. key) then
			TOOL.ClientConVar[key] = previous:GetString()
		end
	end
	I.RegisterAlias("tool.jolt_softbody.name", "jolt.soft_body_workshop")
	I.RegisterAlias("tool.jolt_softbody.desc", "jolt.create_cloth_and_soft_balls_drag_or_pin_vertices")
	I.RegisterAlias("tool.jolt_softbody.left", "jolt.place_soft_body_z_undo")
	I.RegisterAlias("tool.jolt_softbody.left_1", "jolt.hold_to_drag_a_vertex_release_to_restore_its_original_pin_state")
	I.RegisterAlias("tool.jolt_softbody.right", "jolt.toggle_selected_vertex_s_pin_state")
	I.RegisterAlias("tool.jolt_softbody.reload", "jolt.finish_dragging")
	I.RegisterAlias("Cleanup_jolt_softbodies", "jolt.jolt_soft_body")
	I.RegisterAlias("Cleaned_jolt_softbodies", "jolt.jolt_soft_bodies_cleaned_up")
	I.RegisterAlias("SBoxLimit_jolt_softbodies", "jolt.soft_body_limit_reached")
end

local gameplay = SERVER and include("jolt_gameplay/server.lua")

function TOOL:LeftClick(trace)
	if self:GetClientInfo("mode") == "edit" then return true end
	local owner, api = self:GetOwner(), JoltSoftBodies
	if not api or not trace.Hit or trace.HitSky
		or owner:GetShootPos():DistToSqr(trace.HitPos) > api.Config.EditDistance ^ 2 then return false end
	if CLIENT then return true end
	if CurTime() < (self.NextSpawn or 0) then return false end
	self.NextSpawn = CurTime() + 0.5
	local input = {}
	for key in pairs(self.ClientConVar) do
		if key ~= "mode" then input[key] = self:GetClientNumber(key, self.ClientConVar[key]) end
	end
	local settings = api.Config.Sanitize(input)
	local hit, normal = trace.HitPos, trace.HitNormal
	local x, y, z, yaw = api.Geometry.Placement(settings, hit.x, hit.y, hit.z, normal.x, normal.y, normal.z, owner:EyeAngles().y)
	local record, reason = api.Server.Spawn(owner, settings, Vector(x, y, z), yaw)
	if not record then return gameplay.Fail(owner, reason) end
	return true
end

function TOOL:RightClick()
	return self:GetClientInfo("mode") == "edit"
end

function TOOL:Think()
	local api = JoltSoftBodies
	if not api then return end
	if SERVER then
		self:SetStage(self:GetClientInfo("mode") == "edit" and 1 or 0)
		-- Read button edges here so held drags also end when aiming off the mesh.
		api.Editing.Input(self)
	else
		api.ToolClient.Think(self)
	end
end

function TOOL:Reload()
	if SERVER and JoltSoftBodies then JoltSoftBodies.Editing.Release(self:GetOwner()) end
	return true
end

function TOOL:Holster()
	if SERVER and JoltSoftBodies then JoltSoftBodies.Editing.Release(self:GetOwner()) end
	return true
end

function TOOL:DrawHUD()
	if JoltSoftBodies and JoltSoftBodies.ToolClient then JoltSoftBodies.ToolClient.DrawHUD() end
end

function TOOL.BuildCPanel(panel)
	if JoltSoftBodies and JoltSoftBodies.ToolClient then JoltSoftBodies.ToolClient.BuildCPanel(panel) end
end
