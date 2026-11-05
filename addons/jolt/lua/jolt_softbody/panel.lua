local I = JoltI18n or include("jolt_i18n/init.lua")
local config = JoltSoftBodies.Config
local colors = {
	active = Color(43, 81, 91), inactive = Color(232, 237, 239), hover = Color(212, 228, 232),
	text = Color(43, 53, 61), selectedText = Color(243, 249, 250),
}
surface.CreateFont("JoltSoftBodyText", {font = "Microsoft YaHei", size = 16, weight = 400, extended = true, antialias = true})
surface.CreateFont("JoltSoftBodyTitle", {font = "Microsoft YaHei", size = 20, weight = 600, extended = true, antialias = true})

local function help(panel, text)
	local label = I.Control(panel, "Help", text)
	label:SetFont("JoltSoftBodyText")
	label:SetAutoStretchVertical(true)
	return label
end

local function slider(panel, label, variable, minimum, maximum, decimals)
	local control = I.Control(panel, "NumSlider", label, "jolt_softbody_" .. variable, minimum, maximum, decimals)
	control.Label:SetFont("JoltSoftBodyText")
	control.TextArea:SetFont("JoltSoftBodyText")
	return control
end

local function modeButton(panel, text, mode)
	local button = panel:Add("DButton")
	I.Bind(button, "SetText", text)
	button:SetFont("JoltSoftBodyText")
	function button:DoClick() RunConsoleCommand("jolt_softbody_mode", mode) end
	function button:Paint(width, height)
		draw.RoundedBox(4, 0, 0, width, height, self.JoltActive and colors.active or self:IsHovered() and colors.hover or colors.inactive)
	end
	return button
end

local function choice(panel, label, variable, labels, first)
	local combo, caption = I.Control(panel, "ComboBox", label, "jolt_softbody_" .. variable)
	combo:SetFont("JoltSoftBodyText")
	caption:SetFont("JoltSoftBodyText")
	for index = 1, #labels do I.Choice(combo, labels[index], index + (first or 1) - 1) end
	return combo
end

return function(panel)
	help(panel, I.Token("jolt.let_cloth_drape_and_soft_balls_deform_build_structures_with_pinned_v"))
	local modes = vgui.Create("DPanel", panel)
	modes:SetTall(108)
	modes:SetPaintBackground(false)
	local create, edit = modeButton(modes, I.Token("jolt.create_soft_body"), "create"), modeButton(modes, I.Token("jolt.edit_vertices"), "edit")
	local attach, cut = modeButton(modes, I.Token("jolt.mount_points"), "attach"), modeButton(modes, I.Token("jolt.cut_softbody"), "cut")
	local pressure = modeButton(modes, I.Token("jolt.pressure_tool"), "pressure")
	local buttons = {create, edit, attach, cut, pressure}
	local modeNames = {"create", "edit", "attach", "cut", "pressure"}
	function modes:PerformLayout(width, height)
		for index, button in ipairs(buttons) do
			button:SetPos((index - 1) % 2 * (width + 6) * 0.5, math.floor((index - 1) / 2) * 38)
			button:SetSize((width - 6) * 0.5, 32)
		end
	end
	panel:AddItem(modes)
	local budget = help(panel, "")
	local form = vgui.Create("DForm", panel)
	I.Control(form, "SetName", I.Token("jolt.shape_and_feel"))
	panel:AddItem(form)
	local shape = choice(form, I.Token("jolt.shape"), "kind", {I.Token("jolt.cloth"), I.Token("jolt.soft_ball"), I.Token("jolt.soft_rope"), I.Token("jolt.fabric_strap")})
	function shape:OnSelect(_, _, value)
		RunConsoleCommand("jolt_softbody_kind", tostring(value))
		if value >= 3 then
			RunConsoleCommand("jolt_softbody_width", value == 3 and "8" or "24")
			RunConsoleCommand("jolt_softbody_height", "256")
		end
	end
	local materialLabels = {}
	for _, preset in ipairs(config.Materials) do materialLabels[#materialLabels + 1] = I.Token(preset.label) end
	local material = choice(form, I.Token("jolt.material_preset"), "material", materialLabels)
	function material:OnSelect(_, _, value)
		RunConsoleCommand("jolt_softbody_material", tostring(value))
		for key, setting in pairs(config.Materials[value].values or {}) do
			RunConsoleCommand("jolt_softbody_" .. key, tostring(setting))
		end
	end
	choice(form, I.Token("jolt.mesh_quality"), "quality", {I.Token("jolt.low"), I.Token("jolt.standard"), I.Token("jolt.high")})
	local width = slider(form, I.Token("jolt.softbody_width"), "width", 2, 512, 0)
	local height = slider(form, I.Token("jolt.softbody_length"), "height", 32, 1024, 0)
	slider(form, I.Token("jolt.total_mass_kg"), "mass", 1, 50, 1)
	slider(form, I.Token("jolt.softness"), "softness", 0, 100, 0)
	local inflation = slider(form, I.Token("jolt.ball_inflation"), "inflation", 0, 100, 0)
	local pins = choice(form, I.Token("jolt.cloth_pins"), "pins", {I.Token("jolt.none"), I.Token("jolt.top_corners"), I.Token("jolt.entire_top_edge")}, 0)
	choice(form, I.Token("jolt.color"), "palette", {I.Token("jolt.ocean_blue"), I.Token("jolt.terracotta"), I.Token("jolt.teal"), I.Token("jolt.ivory")})
	slider(form, I.Token("jolt.rotation_angle"), "rotation", -180, 180, 0)
	I.Control(form, "CheckBox", I.Token("jolt.player_collision"), "jolt_softbody_playercollision")
	slider(form, I.Token("jolt.solver_iterations"), "bodyiterations", 1, 32, 0)
	slider(form, I.Token("jolt.damping"), "bodydamping", 0, 10, 2)
	slider(form, I.Token("jolt.friction"), "bodyfriction", 0, 1, 2)
	slider(form, I.Token("jolt.restitution"), "bodyrestitution", 0, 1, 2)
	slider(form, I.Token("jolt.vertex_collision_radius"), "vertexradius", 0, 64, 1)
	slider(form, I.Token("jolt.max_velocity"), "maxvelocity", 1, 100000, 0)
	slider(form, I.Token("jolt.gravity_factor"), "gravity", -100, 100, 2)
	I.Control(form, "CheckBox", I.Token("jolt.dihedral_bending"), "jolt_softbody_dihedral")
	slider(form, I.Token("jolt.plastic_yield"), "plasticyield", 0.001, 1, 3)
	slider(form, I.Token("jolt.plastic_creep"), "plasticcreep", 0, 20, 1)
	slider(form, I.Token("jolt.plastic_strain"), "plasticstrain", 0, 0.95, 2)
	local tearing = I.Control(form, "CheckBox", I.Token("jolt.automatic_tearing"), "jolt_softbody_tearing")
	local tearstrain = slider(form, I.Token("jolt.tear_strain_percent"), "tearstrain", 5, 300, 0)
	help(form, I.Token("jolt.material_preset_help"))
	help(form, I.Token("jolt.softbody_rope_foil_disabled"))
	I.Control(form, "CheckBox", I.Token("jolt.impact_feedback"), "jolt_softbody_feedback")
	I.Control(form, "CheckBox", I.Token("jolt.impact_break"), "jolt_softbody_impactbreak")
	slider(form, I.Token("jolt.impact_threshold"), "impactthreshold", 20, 1000, 0)
	slider(form, I.Token("jolt.material_toughness"), "toughness", 0.1, 10, 1)
	help(form, I.Token("jolt.impact_feedback_help"))
	local createHelp = help(panel, I.Token("jolt.lmb_to_place_z_to_undo_top_pins_are_fixed_in_world_space_switch_to_e"))
	local editHelp = help(panel, I.Token("jolt.hold_lmb_to_drag_the_selected_vertex_releasing_restores_its_original"))
	editHelp:SetVisible(false)
	local mountHelp = help(panel, I.Token("jolt.mount_help"))
	local cutHelp = help(panel, I.Token("jolt.cut_help"))
	mountHelp:SetVisible(false)
	cutHelp:SetVisible(false)
	local pressureHelp = help(panel, I.Token("jolt.pressure_help"))
	pressureHelp:SetVisible(false)
	local advanced = vgui.Create("DForm", panel)
	I.Control(advanced, "SetName", I.Token("jolt.display_and_performance"))
	advanced:SetExpanded(false)
	slider(advanced, I.Token("jolt.view_distance"), "viewdistance", 256, 3000, 0)
	slider(advanced, I.Token("jolt.interpolation_rate"), "drawrate", 0, 144, 0)
	I.Control(advanced, "CheckBox", I.Token("jolt.show_selected_object_s_wireframe"), "jolt_softbody_wireframe")
	help(advanced, I.Token("jolt.syncs_at_up_to_15_hz_nearby_and_5_hz_at_distance_the_client_interpol"))
	panel:AddItem(advanced)
	local nextUpdate, previousMode, previousKind, previousQuality, previousLocale = 0
	function modes:Think()
		local now = RealTime()
		if now < nextUpdate then return end
		nextUpdate = now + 0.2
		local mode = GetConVar("jolt_softbody_mode"):GetString()
		local kind = math.Clamp(GetConVar("jolt_softbody_kind"):GetInt(), 1, 4)
		local quality = math.Clamp(GetConVar("jolt_softbody_quality"):GetInt(), 1, 3)
		if mode ~= previousMode or kind ~= previousKind then
			local creating = mode == "create"
			form:SetVisible(creating)
			createHelp:SetVisible(creating)
			budget:SetVisible(creating)
			editHelp:SetVisible(mode == "edit")
			mountHelp:SetVisible(mode == "attach")
			cutHelp:SetVisible(mode == "cut")
			pressureHelp:SetVisible(mode == "pressure")
			for index, button in ipairs(buttons) do
				button.JoltActive = mode == modeNames[index]
				button:SetTextColor(button.JoltActive and colors.selectedText or colors.text)
			end
			width:SetMinMax(kind >= 3 and 2 or 32, kind >= 3 and 128 or 512)
			height:SetEnabled(kind ~= 2)
			pins:SetEnabled(kind ~= 2)
			tearing:SetEnabled(kind ~= 2)
			tearstrain:SetEnabled(kind ~= 2)
			inflation:SetEnabled(kind == 2)
			panel:InvalidateLayout(true)
		end
		if kind ~= previousKind or quality ~= previousQuality or previousLocale ~= I.GetLocale() then
			local vertices = config.VertexCount(kind, quality)
			local triangles = config.TriangleCount(kind, quality)
			budget:SetText(I.Format("jolt.d_vertices_d_triangles_approx_1f_kib_s_per_nearby_player_while_activ", vertices, triangles,
				config.SnapshotBytes(vertices) / config.NearInterval / 1024))
			budget:SizeToContents()
		end
		previousMode, previousKind, previousQuality, previousLocale = mode, kind, quality, I.GetLocale()
	end
end
