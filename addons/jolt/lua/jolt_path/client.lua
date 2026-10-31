local I = JoltI18n or include("jolt_i18n/init.lua")
local P = JoltPath
P.ClientTracks = P.ClientTracks or {}
P.ClientDraft = P.ClientDraft or {nodes = {}, settings = P.Settings(), node = 0, referenceBone = 0}
P.VisibleTracks = P.VisibleTracks or {}
surface.CreateFont("JoltPathHudTitle", {font = "Microsoft YaHei", size = 25, weight = 700, extended = true, antialias = true})
surface.CreateFont("JoltPathHudText", {font = "Microsoft YaHei", size = 19, weight = 400, extended = true, antialias = true})
surface.CreateFont("JoltPathHudNode", {font = "Microsoft YaHei", size = 20, weight = 700, extended = true, antialias = true})
local show = CreateClientConVar("jolt_path_show", "1", true, false, I.Phrase("jolt.show_jolt_track_lines"))
local rail, selected, errorColor = Color(236, 176, 65), Color(255, 238, 184), Color(230, 102, 82)
local background, textColor, muted = Color(26, 24, 19, 225), Color(244, 239, 223), Color(187, 178, 157)
local markerMin, markerMax = Vector(-2, -2, -2), Vector(2, 2, 2)
local metrics
local function layout()
    local w, h = ScrW(), ScrH()
    metrics = {x = w < 1280 and 12 or 24, y = math.floor(h * 0.5) - 82, w = math.min(510, w - 48), compact = w < 1280}
end
layout()
hook.Add("OnScreenSizeChanged", "JoltPathLayout", layout)
function P.Editing()
    local player = LocalPlayer()
    if not IsValid(player) or not player:Alive() then return false end
    local weapon = player:GetActiveWeapon()
    return IsValid(weapon) and weapon:GetClass() == "gmod_tool" and weapon:GetMode() == "jolt_path"
end
local function panelRefresh()
    if IsValid(P.Panel) and P.Panel.RefreshPath then P.Panel:RefreshPath() end
end
hook.Add("JoltLanguageChanged", "JoltPath.PanelLocale", panelRefresh)
local function feedback(message, failed)
    notification.AddLegacy(I.Resolve(message), failed and NOTIFY_ERROR or NOTIFY_GENERIC, 4)
end
function P.SendAction(action, value)
    if not P.Editing() then feedback(I.Phrase("jolt.equip_the_path_track_tool_first"), true); return false end
    if P.Pending then return false end
    local index = P.EnumIndex(P.Actions, action)
    if not index then return false end
    net.Start("jolt_path_action")
    net.WriteUInt(index - 1, 4)
    if action == "select_node" then net.WriteUInt(value, 6)
    elseif action == "set_node" then net.WriteUInt(value.index, 6); P.WriteVector(value.position)
    elseif action == "import" then P.WriteData(value) end
    net.SendToServer()
    P.Pending = RealTime() + 2
    panelRefresh()
    return true
end
local function readSettings()
    local input = {}
    for key, fallback in pairs(P.Defaults) do
        local cvar = GetConVar("jolt_path_" .. string.lower(key))
        if cvar then
            if type(fallback) == "boolean" then input[key] = cvar:GetBool()
            elseif type(fallback) == "number" then input[key] = cvar:GetFloat()
            else input[key] = cvar:GetString() end
        else input[key] = fallback end
    end
    if GetConVar("jolt_path_target") then input.target = input.target / 100 end
    return P.Settings(input)
end
local function applySettings(settings)
    for key, value in pairs(settings) do
        if type(value) == "boolean" then value = value and "1" or "0"
        elseif key == "target" then value = value * 100 end
        RunConsoleCommand("jolt_path_" .. string.lower(key), tostring(value))
    end
end
local function build(data)
    local curve, reason = P.BuildCurve(data.nodes, data.settings)
    data.curve, data.error = curve, reason
    if curve then
        curve.matrix = Matrix()
        local low, high = Vector(math.huge, math.huge, math.huge), Vector(-math.huge, -math.huge, -math.huge)
        for _, v in ipairs(curve.points) do
            low.x, low.y, low.z = math.min(low.x, v.x), math.min(low.y, v.y), math.min(low.z, v.z)
            high.x, high.y, high.z = math.max(high.x, v.x), math.max(high.y, v.y), math.max(high.z, v.z)
        end
        curve.center, curve.radius = (low + high) * .5, (high - low):Length() * .5
        -- Arrow vertices are immutable until nodes or smoothing change.
        curve.arrows = {}
        local stride = math.max(2, math.floor(#curve.points / 12))
        for i = 2, #curve.points, stride do
            local point = curve.points[i]
            local tangent = (point - curve.points[i - 1]):GetNormalized()
            local side = tangent:Cross(Vector(0, 0, 1))
            if side:LengthSqr() < .01 then side = tangent:Cross(Vector(0, 1, 0)) end
            side:Normalize()
            curve.arrows[#curve.arrows + 1] = {point, point - tangent * 8 + side * 4, point - tangent * 8 - side * 4}
        end
    end
end
net.Receive("jolt_path_data", function()
    local entity, data = net.ReadEntity(), P.ReadData()
    if not IsValid(entity) or not data then return end
    build(data); entity.JoltPathData = data; P.ClientTracks[entity] = true
end)
net.Receive("jolt_path_draft", function()
    local data = P.ReadData()
    if not data then return end
    data.selected, data.node, data.moving = net.ReadEntity(), net.ReadUInt(6), net.ReadBool()
    local replace = net.ReadBool()
    if data.node > #data.nodes then data.node, data.moving = 0, false end
    P.ClientDraft, P.Pending = data, nil
    if replace then applySettings(data.settings) end
    build(data); panelRefresh()
end)
for key in pairs(P.Defaults) do
    cvars.AddChangeCallback("jolt_path_" .. string.lower(key), function() P.DraftDirty = RealTime() + .06 end, "JoltPathPreview")
end
local nextRefresh, nextRequest, wasEditing = 0, 0, false
hook.Add("Think", "JoltPathClientCache", function()
    local now = RealTime()
    if P.Pending and now > P.Pending then P.Pending = nil; feedback(I.Phrase("jolt.action_not_confirmed_please_try_again"), true); panelRefresh() end
    local editing = P.Editing()
    if editing and not wasEditing and not P.Pending then P.SendAction("sync") end
    wasEditing = editing
    if P.DraftDirty and now >= P.DraftDirty then
        P.DraftDirty = nil
        local settings, reason = readSettings()
        if settings then P.ClientDraft.settings = settings; build(P.ClientDraft) else P.ClientDraft.error = reason end
        panelRefresh()
    end
    if now < nextRefresh then return end
    nextRefresh = now + .25
    local player = LocalPlayer()
    if not IsValid(player) then return end
    local eye, visible = player:EyePos(), {}
    for entity in pairs(P.ClientTracks) do
        if not IsValid(entity) then P.ClientTracks[entity] = nil
        elseif not entity.JoltPathData then
            if now >= nextRequest and now >= (entity.JoltPathNextRequest or 0) then
                entity.JoltPathNextRequest, nextRequest = now + 2, now + .12
                net.Start("jolt_path_request"); net.WriteEntity(entity); net.SendToServer()
            end
        else
            local data = entity.JoltPathData
            local position, angles = P.Frame(data)
            if data.curve and position and data.settings.visible then
                local center = LocalToWorld(data.curve.center, angle_zero, position, angles)
                local distance = eye:Distance(center)
                if distance < 8192 + data.curve.radius then visible[#visible + 1] = {entity = entity, distance = distance} end
            end
        end
    end
    table.sort(visible, function(a, b) return a.distance < b.distance end)
    for i = #visible, 25, -1 do visible[i] = nil end
    P.VisibleTracks = visible
end)
hook.Add("NetworkEntityCreated", "JoltPathClientRegister", function(entity)
    if IsValid(entity) and entity:GetClass() == "sent_jolt_path" then P.ClientTracks[entity] = true end
end)
hook.Add("OnReloaded", "JoltPathClientReload", function()
    for _, entity in ipairs(ents.FindByClass("sent_jolt_path")) do P.ClientTracks[entity] = true; entity.JoltPathData = nil end
    P.DraftDirty = RealTime()
end)
local function drawCurve(data, color, nodes, progress, target)
    local position, angles = P.Frame(data)
    if not position then return end
    local curve = data.curve
    if curve then
        curve.matrix:SetTranslation(position); curve.matrix:SetAngles(angles)
        cam.PushModelMatrix(curve.matrix)
        for i = 2, #curve.points do render.DrawLine(curve.points[i - 1], curve.points[i], color, true) end
        for _, arrow in ipairs(curve.arrows) do
            render.DrawLine(arrow[1], arrow[2], color, true); render.DrawLine(arrow[1], arrow[3], color, true)
        end
        if progress then render.DrawWireframeSphere(P.PointAt(curve, progress), 5, 8, 6, selected, true) end
        if target then render.DrawWireframeBox(P.PointAt(curve, target), angle_zero, markerMin * 2, markerMax * 2, rail, true) end
        cam.PopModelMatrix()
    end
    if nodes then
        for i, point in ipairs(data.nodes) do
            local world = LocalToWorld(point, angle_zero, position, angles)
            render.DrawWireframeBox(world, angle_zero, markerMin, markerMax, i == data.node and selected or rail, true)
        end
    end
end
hook.Add("PostDrawTranslucentRenderables", "JoltPathTracks", function(depth, skybox)
    if depth or skybox then return end
    local editing, draft = P.Editing(), P.ClientDraft
    if show:GetBool() then
        for _, item in ipairs(P.VisibleTracks) do
            local entity = item.entity
            if IsValid(entity) and (not editing or entity ~= draft.selected) and entity.JoltPathData then
                drawCurve(entity.JoltPathData, rail, false, entity:GetProgress(), entity:GetMotorMode() == 3 and entity:GetMotorTarget() or nil)
            end
        end
    end
    if editing then drawCurve(draft, draft.error and errorColor or selected, true) end
end)
function P.DrawHUD()
    local data, m = P.ClientDraft, metrics
    local nodes = P.WorldNodes(data)
    if nodes then
        for i, point in ipairs(nodes) do
            local screen = point:ToScreen()
            if screen.visible then draw.SimpleTextOutlined(string.format("%02d", i), "JoltPathHudNode", screen.x, screen.y - 12,
                i == data.node and selected or rail, TEXT_ALIGN_CENTER, TEXT_ALIGN_BOTTOM, 1, background) end
        end
    end
    local reference = data.referenceIndex and data.referenceIndex > 0 or IsValid(data.reference)
    draw.SimpleText(I.Phrase("jolt.path_track") .. #data.nodes .. I.Phrase("jolt.32_nodes") .. (reference and I.Phrase("jolt.following_reference_object") or I.Phrase("jolt.fixed_in_world")),
        "JoltPathHudTitle", m.x, m.y + 10, rail)
    local hint = data.moving and (I.Phrase("jolt.node") .. data.node .. I.Phrase("jolt.ready_to_move_e_lmb_to_place")) or I.Phrase("jolt.lmb_add_node_rmb_attach_object")
    draw.SimpleText(hint, "JoltPathHudText", m.x, m.y + 44, muted)
    local controller = data.selected
    local status = I.Phrase("jolt.no_track_selected_e_rmb_to_select_an_existing_track")
    if IsValid(controller) then
        local mode = not controller:GetPathEnabled() and I.Phrase("jolt.rewinding") or controller:GetPathPaused() and I.Phrase("jolt.holding_position") or I.Phrase("jolt.selected")
        status = I.Format("jolt.s_d_5_1f_6_1f_units_s", mode, controller:EntIndex(), controller:GetProgress() * 100, controller:GetPathVelocity())
    end
    draw.SimpleText(status, "JoltPathHudText", m.x, m.y + 72, textColor)
    local finalLine = data.error and (#data.nodes >= 2 and I.Phrase("jolt.invalid_nodes_or_curve_see_the_q_panel_for_details") or I.Phrase("jolt.at_least_2_nodes_loops_need_at_least_3")) or I.Phrase("jolt.q_panel_apply_edits_pause_reverse_save_preset")
    draw.SimpleText(finalLine, "JoltPathHudText", m.x, m.y + 100, data.error and errorColor or muted)
    draw.SimpleText(I.Phrase("jolt.e_lmb_select_move_node_shift_lmb_insert_node"), "JoltPathHudText", m.x, m.y + 128, textColor)
    draw.SimpleText(I.Phrase("jolt.e_rmb_select_track_shift_rmb_set_reference_object"), "JoltPathHudText", m.x, m.y + 156, textColor)
    draw.SimpleText(I.Phrase("jolt.r_delete_node_h_new_track_z_undo"), "JoltPathHudText", m.x, m.y + 184, textColor)
end

function P.PresetName(name)
    return type(name) == "string" and #name >= 1 and #name <= 48 and name:match("^[%w_-]+$") and name or nil
end
function P.EncodePreset(data, yaw)
    local settings, reason = P.Settings(data.settings)
    if not settings then return nil, reason end
    local nodes = P.WorldNodes(data)
    if not nodes or not P.BuildCurve(nodes, settings) then return nil, I.Phrase("jolt.complete_a_valid_track_first") end
    local serial = {version = 1, nodes = {}, settings = settings}
    for i, node in ipairs(nodes) do
        local v = WorldToLocal(node, angle_zero, nodes[1], Angle(0, yaw, 0))
        serial.nodes[i] = {x = v.x, y = v.y, z = v.z}
    end
    return util.TableToJSON(serial, true)
end
function P.DecodePreset(text)
    if type(text) ~= "string" or #text > 16384 then return nil, I.Phrase("jolt.preset_too_large_or_invalid_file") end
    local serial = util.JSONToTable(text)
    if type(serial) ~= "table" or serial.version ~= 1 or type(serial.nodes) ~= "table" then return nil, I.Phrase("jolt.unsupported_preset_format") end
    local settings, reason = P.Settings(serial.settings)
    if not settings then return nil, reason end
    local data = {nodes = {}, settings = settings, referenceBone = 0}
    if #serial.nodes > P.MaxNodes then return nil, I.Phrase("jolt.too_many_preset_nodes") end
    for i, node in ipairs(serial.nodes) do
        if type(node) ~= "table" or not P.Finite(node.x) or not P.Finite(node.y) or not P.Finite(node.z) then return nil, I.Phrase("jolt.invalid_preset_node_coordinates") end
        data.nodes[i] = Vector(node.x, node.y, node.z)
    end
    local curve, error = P.BuildCurve(data.nodes, settings)
    if not curve then return nil, error end
    return data
end

function P.BuildPanel(panel)
    P.Panel = panel
    I.Control(panel, "Help", I.Token("jolt.draw_a_track_then_attach_an_object_with_rmb_keep_the_object_frozen_s"))
    I.Control(panel, "Help", I.Token("jolt.e_lmb_selects_a_node_e_lmb_again_places_it_shift_lmb_inserts_after_t"))
    local stateLabel = panel:Help(I.Text("jolt.waiting_for_tool_state"))
    local errorLabel = panel:Help("")
    local list = vgui.Create("DListView", panel)
    list:SetTall(145); list:SetMultiSelect(false)
    I.Control(list, "AddColumn", I.Token("jolt.node_79")):SetFixedWidth(32); list:AddColumn("X"); list:AddColumn("Y"); list:AddColumn("Z")
    panel:AddItem(list)
    local buttons = {}
    local function button(label, action, selection)
        local b = I.Control(panel, "Button", label)
        b.DoClick = function() P.SendAction(action) end
        buttons[#buttons + 1] = {panel = b, selection = selection}
        return b
    end
    button(I.Token("jolt.move_node_up"), "move_up"); button(I.Token("jolt.move_node_down"), "move_down"); button(I.Token("jolt.delete_selected_node"), "remove_node")
    I.Control(panel, "Help", I.Token("jolt.node_coordinates_world_coordinates_for_world_tracks_reference_local"))
    local xyz = {}
    for _, axis in ipairs({"x", "y", "z"}) do
        local row = vgui.Create("DPanel", panel); row:SetTall(24); row.Paint = nil
        local label = vgui.Create("DLabel", row); label:Dock(LEFT); label:SetWide(20); label:SetText(string.upper(axis)); label:SetDark(true)
        local value = vgui.Create("DNumberWang", row); value:Dock(FILL); value:SetMinMax(-P.MaxLocalCoordinate, P.MaxLocalCoordinate); value:SetDecimals(2)
        xyz[axis] = value; panel:AddItem(row)
    end
    local coordinateButton = I.Control(panel, "Button", I.Token("jolt.apply_node_coordinates"))
    coordinateButton.DoClick = function()
        P.SendAction("set_node", {index = P.ClientDraft.node, position = Vector(xyz.x:GetValue(), xyz.y:GetValue(), xyz.z:GetValue())})
    end
    button(I.Token("jolt.new_track_clear_draft"), "clear")
    I.Control(panel, "Help", I.Token("jolt.shift_rmb_on_an_object_attaches_the_track_to_it_click_the_map_to_fix"))
    I.Control(panel, "CheckBox", I.Token("jolt.closed_track"), "jolt_path_closed")
    I.Control(panel, "CheckBox", I.Token("jolt.snap_object_center_of_mass_to_track_when_attaching_rebuilding"), "jolt_path_snap")
    I.Control(panel, "CheckBox", I.Token("jolt.show_this_track_to_players"), "jolt_path_visible")
    I.Control(panel, "CheckBox", I.Token("jolt.show_tracks_locally_drafts_always_visible"), "jolt_path_show")
    I.Control(panel, "NumSlider", I.Token("jolt.curve_smoothness"), "jolt_path_smoothing", .05, 1, 2)
    I.Control(panel, "NumSlider", I.Token("jolt.surface_offset_when_placing_nodes"), "jolt_path_offset", -128, 512, 1)
    local rotation = I.Control(panel, "ComboBox", I.Token("jolt.object_rotation"), "jolt_path_rotation")
    local rotationLabels = {I.Token("jolt.free_rotation"), I.Token("jolt.rotate_around_track_tangent"), I.Token("jolt.rotate_around_normal"), I.Token("jolt.rotate_around_binormal"), I.Token("jolt.follow_track_direction"), I.Token("jolt.keep_orientation_relative_to_reference")}
    for i, mode in ipairs(P.Rotations) do I.Choice(rotation, rotationLabels[i], mode) end
    I.Control(panel, "Help", I.Token("jolt.following_the_track_preserves_the_original_orientation_offset_object"))
    local mode = I.Control(panel, "ComboBox", I.Token("jolt.track_motion"), "jolt_path_mode")
    local modeLabels = {I.Token("jolt.free_slide_motor_off"), I.Token("jolt.constant_speed"), I.Token("jolt.move_to_target_position"), I.Token("jolt.automatic_end_to_end_travel")}
    for i, name in ipairs(P.Modes) do I.Choice(mode, modeLabels[i], name) end
    I.Control(panel, "NumSlider", I.Token("jolt.speed_units_second"), "jolt_path_speed", -32767, 32767, 1)
    I.Control(panel, "Help", I.Token("jolt.speed_range_is_32767_negative_values_move_against_the_arrows_closed"))
    I.Control(panel, "NumSlider", I.Token("jolt.target_position_of_track_length"), "jolt_path_target", 0, 100, 1)
    I.Control(panel, "NumSlider", I.Token("jolt.maximum_motor_force"), "jolt_path_maxforce", 0, 1000000, 0)
    I.Control(panel, "NumSlider", I.Token("jolt.free_slide_friction"), "jolt_path_friction", 0, 100000, 0)
    I.Control(panel, "NumSlider", I.Token("jolt.position_spring_frequency_hz"), "jolt_path_frequency", .1, 20, 1)
    I.Control(panel, "NumSlider", I.Token("jolt.position_damping_ratio"), "jolt_path_damping", 0, 5, 2)
    button(I.Token("jolt.apply_track_and_all_settings"), "apply", true)
    button(I.Token("jolt.update_motion_and_visibility_only"), "motor", true)
    button(I.Token("jolt.hold_position_resume_motion"), "pause", true)
    button(I.Token("jolt.reverse_motion_mirror_target_position"), "reverse", true)
    button(I.Token("jolt.remove_selected_track_keep_object"), "delete", true)
    I.Control(panel, "Help", I.Token("jolt.e_rmb_selects_an_existing_track_after_editing_nodes_click_apply_trac"))
    I.Control(panel, "Help", I.Token("jolt.presets_are_saved_locally_in_data_jolt_paths_they_load_at_the_crossh"))
    local name = I.Control(panel, "TextEntry", I.Token("jolt.preset_name_letters_numbers_underscore"))
    name:SetText("my_track")
    local preset = I.Control(panel, "ComboBox", I.Token("jolt.saved_presets"))
    local function refreshPresets()
        preset:Clear()
        local files = file.Find("jolt_paths/*.json", "DATA")
        table.sort(files)
        for _, filename in ipairs(files) do
            local stem = filename:sub(1, -6)
            if P.PresetName(stem) then preset:AddChoice(stem, stem) end
        end
    end
    preset.OnSelect = function(_, _, text) name:SetText(text) end
    local save = I.Control(panel, "Button", I.Token("jolt.save_current_draft_as_preset"))
    save.DoClick = function()
        local filename = P.PresetName(name:GetValue())
        if not filename then feedback(I.Phrase("jolt.use_only_letters_numbers_underscores_and_hyphens_maximum_48_bytes"), true); return end
        local settings, error = readSettings()
        if not settings then feedback(error, true); return end
        P.ClientDraft.settings = settings
        local text, reason = P.EncodePreset(P.ClientDraft, LocalPlayer():EyeAngles().y)
        if not text then feedback(reason, true); return end
        file.CreateDir("jolt_paths"); file.Write("jolt_paths/" .. filename .. ".json", text)
        refreshPresets(); feedback(I.Phrase("jolt.track_preset_saved") .. filename)
    end
    local load = I.Control(panel, "Button", I.Token("jolt.load_preset_at_crosshair"))
    load.DoClick = function()
        local filename = P.PresetName(name:GetValue())
        if not filename then feedback(I.Phrase("jolt.invalid_preset_name"), true); return end
        local path = "jolt_paths/" .. filename .. ".json"
        if (file.Size(path, "DATA") or -1) > 16384 then feedback(I.Phrase("jolt.preset_file_too_large"), true); return end
        local data, reason = P.DecodePreset(file.Read(path, "DATA"))
        if not data then feedback(reason, true); return end
        P.SendAction("import", data)
    end
    I.Control(panel, "NumSlider", I.Token("jolt.extra_rotation_when_loading_preset_degrees"), "jolt_path_presetyaw", -180, 180, 0)
    refreshPresets()
    function panel:RefreshPath()
        local data = P.ClientDraft
        stateLabel:SetText(I.Format("jolt.d_32_nodes_s_s", #data.nodes, IsValid(data.selected) and I.Phrase("jolt.track_selected") or I.Phrase("jolt.editing_draft"), P.Pending and I.Phrase("jolt.waiting_for_server") or ""))
        errorLabel:SetText(I.Resolve(#data.nodes >= 2 and (data.error or "") or I.Token("jolt.place_at_least_2_nodes_closed_tracks_need_at_least_3")))
        list.Updating = true; list:Clear()
        for i, v in ipairs(data.nodes) do
            local line = list:AddLine(i, string.format("%.1f", v.x), string.format("%.1f", v.y), string.format("%.1f", v.z))
            if i == data.node then list:SelectItem(line) end
        end
        list.Updating = false
        local point = data.nodes[data.node]
        coordinateButton:SetEnabled(point ~= nil and not P.Pending)
        if point then for axis, input in pairs(xyz) do if not input:HasFocus() then input:SetValue(point[axis]) end end end
        for _, b in ipairs(buttons) do b.panel:SetEnabled(not P.Pending and (not b.selection or IsValid(data.selected))) end
    end
    list.OnRowSelected = function(_, index) if not list.Updating then P.SendAction("select_node", index) end end
    panel:RefreshPath()
end
