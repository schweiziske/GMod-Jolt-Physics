local I = JoltI18n or include("jolt_i18n/init.lua")
local api = JoltSoftBodies
local config, geometry, renderer = api.Config, api.Geometry, api.Renderer
local toolClient = {}
local keys = {"kind", "quality", "width", "height", "mass", "softness", "inflation", "pins", "palette", "rotation", "playercollision", "iterations", "damping", "friction", "restitution", "vertexradius", "maxvelocity", "gravity"}
local state = {
	raw = {}, nextSettings = 0, nextPick = 0, matrix = Matrix(), position = Vector(), angle = Angle(),
	marker = Vector(), valid = false, title = I.Phrase("jolt.jolt_soft_body"), detail = "", hint = "", mode = "create",
}
local colors = {
	background = Color(20, 25, 31, 222), text = Color(232, 238, 241), muted = Color(167, 184, 193),
	accent = Color(103, 201, 188), warning = Color(237, 163, 91), pinned = Color(243, 184, 87), selected = Color(160, 224, 255),
}
surface.CreateFont("JoltSoftBodyHudText", {font = "Microsoft YaHei", size = 24, weight = 400, extended = true, antialias = true})
surface.CreateFont("JoltSoftBodyHudTitle", {font = "Microsoft YaHei", size = 32, weight = 600, extended = true, antialias = true})
local pointMaterial = Material("sprites/light_glow02_add")
local wire = CreateClientConVar("jolt_softbody_wireframe", "0", true, false, "Show selected soft-body topology", 0, 1)

local function active()
	local owner = LocalPlayer()
	if not IsValid(owner) or not owner:Alive() then return false end
	local weapon = owner:GetActiveWeapon()
	return IsValid(weapon) and weapon:GetClass() == "gmod_tool" and weapon:GetMode() == "jolt_softbody"
end

local function settingsChanged(tool)
	local changed = not state.settings
	for index = 1, #keys do
		local key = keys[index]
		local value = tool:GetClientNumber(key)
		if state.raw[key] ~= value then state.raw[key], changed = value, true end
	end
	if not changed then return end
	local settings = config.Sanitize(state.raw)
	local previous = state.settings
	state.settings = settings
	if previous and previous.kind == settings.kind and previous.quality == settings.quality
		and previous.width == settings.width and previous.height == settings.height
		and previous.palette == settings.palette and previous.pins == settings.pins then return end
	if state.preview then renderer.Destroy(state.preview) end
	local topology = geometry.Generate(settings)
	state.preview = renderer.New(topology, settings.palette, settings.kind)
	renderer.Update(state.preview, topology.positions)
end

local function updateText(owner)
	if state.mode == "create" then
		local settings = state.settings
		state.title = settings.kind == 1 and I.Phrase("jolt.create_cloth") or I.Phrase("jolt.create_soft_ball")
		local count = config.VertexCount(settings.kind, settings.quality)
		state.detail = I.Format("jolt.d_vertices_1f_kg_approx_1f_kib_s", count, settings.mass,
			config.SnapshotBytes(count) / config.NearInterval / 1024)
		state.hint = state.valid and I.Phrase("jolt.lmb_place_z_undo_switch_to_edit_in_the_panel") or I.Phrase("jolt.aim_at_a_surface_within_600_units")
	else
		local record, vertex = state.selected, state.vertex
		local dragging = owner:GetNW2Int("jolt_softbody_drag_id", 0) ~= 0
		state.title = dragging and I.Phrase("jolt.dragging_vertex") or I.Phrase("jolt.edit_soft_body")
		if record and vertex then
			local latest = record.buffer:Latest()
			state.detail = I.Format("jolt.vertex_d_s_d_vertices", vertex,
				latest and latest.pinned[vertex] and I.Phrase("jolt.pinned") or I.Phrase("jolt.free"), record.geometry.vertexCount)
			state.hint = dragging and I.Phrase("jolt.release_lmb_or_press_r_to_finish_restores_original_pin_state") or I.Phrase("jolt.hold_lmb_drag_rmb_pin_release")
		else
			state.detail = I.Phrase("jolt.aim_at_a_soft_body_surface_to_select_a_nearby_vertex")
			state.hint = I.Phrase("jolt.gold_vertices_are_pinned_blue_is_selected")
		end
	end
end

function toolClient.Think(tool)
	if tool:GetOwner() ~= LocalPlayer() then return end
	local now, owner = RealTime(), LocalPlayer()
	local refresh = now >= state.nextSettings
	if refresh then
		settingsChanged(tool)
		state.nextSettings = now + 0.15
	end
	if not state.settings then return end
	state.mode = tool:GetClientInfo("mode") == "edit" and "edit" or "create"
	local trace, origin = owner:GetEyeTrace(), owner:GetShootPos()
	state.valid = trace.Hit and not trace.HitSky and origin:DistToSqr(trace.HitPos) <= config.EditDistance ^ 2
	if state.mode == "create" and state.valid then
		local position, normal = trace.HitPos, trace.HitNormal
		local x, y, z, yaw = geometry.Placement(state.settings, position.x, position.y, position.z,
			normal.x, normal.y, normal.z, owner:EyeAngles().y)
		state.position:SetUnpacked(x, y, z)
		state.angle:SetUnpacked(0, yaw, 0)
		state.matrix:SetAngles(state.angle)
		state.matrix:SetTranslation(state.position)
	elseif state.mode == "edit" and now >= state.nextPick then
		local direction = owner:GetAimVector()
		local limit = math.min(config.EditDistance, math.sqrt(origin:DistToSqr(trace.HitPos)) + 1)
		state.selected, state.vertex = api.Client.Pick(origin.x, origin.y, origin.z, direction.x, direction.y, direction.z, limit)
		local dragged = api.Client.Records[owner:GetNW2Int("jolt_softbody_drag_id", 0)]
		if dragged then state.selected, state.vertex = dragged, owner:GetNW2Int("jolt_softbody_drag_vertex", 0) end
		state.nextPick = now + 0.05
	end
	if refresh then updateText(owner) end
end

local function drawSelection()
	local record, selected = state.selected, state.vertex
	if not record or api.Client.Records[record.id] ~= record or record.render.revision == 0 then return end
	local latest, positions = record.buffer:Latest(), record.buffer.display
	if not latest then return end
	if wire:GetBool() then renderer.DrawWire(record.render) end
	render.SetMaterial(pointMaterial)
	local shown = 0
	for vertex = 1, record.geometry.vertexCount do
		if latest.pinned[vertex] and shown < 64 then
			local offset = (vertex - 1) * 3
			state.marker:SetUnpacked(positions[offset + 1], positions[offset + 2], positions[offset + 3])
			render.DrawSprite(state.marker, 5, 5, colors.pinned)
			shown = shown + 1
		end
	end
	if selected and selected >= 1 and selected <= record.geometry.vertexCount then
		local offset = (selected - 1) * 3
		state.marker:SetUnpacked(positions[offset + 1], positions[offset + 2], positions[offset + 3])
		render.DrawSprite(state.marker, 9, 9, colors.selected)
	end
end

hook.Add("PostDrawTranslucentRenderables", "JoltSoftBodyTool", function(depth, skybox)
	if depth or skybox or not active() then return end
	if state.mode == "create" then
		if state.valid and state.preview then renderer.DrawPreview(state.preview, state.matrix) end
	else
		drawSelection()
	end
end)

function toolClient.DrawHUD()
	if not active() then return end
	local x, y = ScrW() < 1280 and 12 or 24, math.floor(ScrH() * 0.5) - 64
	draw.SimpleText(state.title, "JoltSoftBodyHudTitle", x, y + 10,
		state.mode == "create" and not state.valid and colors.warning or colors.accent)
	draw.SimpleText(state.detail, "JoltSoftBodyHudText", x, y + 52, colors.text)
	draw.SimpleText(state.hint, "JoltSoftBodyHudText", x, y + 96, colors.muted)
	draw.SimpleText(I.Phrase("jolt.lmb_spawn_drag_vertex_rmb_pin_release_vertex_r_cancel_drag"), "JoltSoftBodyHudText", x, y + 140, colors.text)
end

function toolClient.Shutdown()
	if state.preview then renderer.Destroy(state.preview) state.preview = nil end
	state.selected, state.vertex, state.valid = nil, nil, false
	hook.Remove("PostDrawTranslucentRenderables", "JoltSoftBodyTool")
end

toolClient.BuildCPanel = include("jolt_softbody/panel.lua")
return toolClient
