local api = JoltSoftBodies
local config = api.Config
local physgun = {}
local inputs = setmetatable({}, {__mode = "k"})

-- Soft bodies have no rigid IPhysicsObject for the engine grab controller.
-- Intercept only gestures that begin on their actual mesh, before weapon attack
-- handling. All ordinary physgun gestures keep their original buttons.
hook.Add("StartCommand", "JoltSoftBodyPhysgun", function(owner, command)
	if CLIENT and owner ~= LocalPlayer() then return end
	local state = inputs[owner]
	if not state then state = {} inputs[owner] = state end
	local weapon = owner:GetActiveWeapon()
	local active = IsValid(weapon) and weapon:GetClass() == "weapon_physgun" and owner:Alive() and not owner:InVehicle()
	local left, right = command:KeyDown(IN_ATTACK), command:KeyDown(IN_ATTACK2)
	if not active then
		if SERVER then api.Editing.PhysgunInput(owner, false, 0, false) end
		state.captured, state.left = false, left
		return
	end
	if left and not state.left then
		local origin, direction = owner:GetShootPos(), command:GetViewAngles():Forward()
		local trace = util.TraceLine({start = origin, endpos = origin + direction * config.EditDistance, filter = owner, mask = MASK_SOLID})
		state.captured = IsValid(trace.Entity) and trace.Entity:GetClass() == config.EntityClass
		if SERVER and state.captured then api.Editing.BeginPhysgun(owner, direction) end
	end
	if SERVER then api.Editing.PhysgunInput(owner, left, command:GetMouseWheel(), right) end
	-- Consume the whole captured gesture, even after a rejection/freeze/deletion.
	-- Otherwise a held button could unexpectedly grab or freeze a prop behind it.
	if state.captured then
		command:RemoveKey(IN_ATTACK)
		command:RemoveKey(IN_ATTACK2)
		command:RemoveKey(IN_RELOAD)
	end
	if not left then state.captured = false end
	state.left = left
end)

if CLIENT then
	local beam = Material("cable/physbeam")
	local glow = Material("sprites/light_glow02_add")
	local endpoint = Vector()
	hook.Add("DrawPhysgunBeam", "JoltSoftBodyPhysgun", function(owner)
		if owner:GetNW2Bool("jolt_softbody_physgun", false) then return false end
	end)
	local function drawBeam(owner)
		if not owner:GetNW2Bool("jolt_softbody_physgun", false) then return end
		local weapon = owner:GetActiveWeapon()
		if not IsValid(weapon) or weapon:GetClass() ~= "weapon_physgun" or not owner:Alive() then return end
		local record = api.Client.Records[owner:GetNW2Int("jolt_softbody_drag_id", 0)]
		local vertex = owner:GetNW2Int("jolt_softbody_drag_vertex", 0)
		if not record or record.render.revision == 0 or vertex < 1 or vertex > record.geometry.vertexCount then return end
		local positions, offset = record.buffer.display, (vertex - 1) * 3
		endpoint:SetUnpacked(positions[offset + 1], positions[offset + 2], positions[offset + 3])
		local model = weapon
		if owner == LocalPlayer() and not owner:ShouldDrawLocalPlayer() then model = owner:GetViewModel() end
		local attachment = IsValid(model) and model:GetAttachment(1)
		local start = attachment and attachment.Pos or owner:GetShootPos()
		local color = owner:GetWeaponColor():ToColor()
		render.SetMaterial(beam)
		render.DrawBeam(start, endpoint, 2, 0, start:Distance(endpoint) / 128, color)
		render.SetMaterial(glow)
		render.DrawSprite(endpoint, 7, 7, color)
	end
	hook.Add("PostDrawTranslucentRenderables", "JoltSoftBodyPhysgun", function(depth, skybox)
		if depth or skybox then return end
		for _, owner in ipairs(player.GetAll()) do drawBeam(owner) end
	end)
	-- Physgun scroll normally also selects weapons. Preserve scroll while holding
	-- a soft mesh, including the short interval before server acknowledgement.
	hook.Add("PlayerBindPress", "JoltSoftBodyPhysgun", function(owner, bind)
		local state = inputs[owner]
		if state and state.captured and (bind == "invnext" or bind == "invprev") then return true end
	end)
else
	for _, event in ipairs({"PlayerDeath", "PlayerDisconnected", "PlayerSwitchWeapon"}) do
		hook.Add(event, "JoltSoftBodyPhysgun", function(owner)
			local drag = api.Editing.Drags[owner]
			if drag and drag.physgun then api.Editing.Release(owner) end
		end)
	end
end

function physgun.Shutdown()
	for _, event in ipairs({"StartCommand", "DrawPhysgunBeam", "PostDrawTranslucentRenderables", "PlayerBindPress",
		"PlayerDeath", "PlayerDisconnected", "PlayerSwitchWeapon"}) do
		hook.Remove(event, "JoltSoftBodyPhysgun")
	end
end

return physgun
