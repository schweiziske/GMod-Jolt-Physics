local I = JoltI18n or include("jolt_i18n/init.lua")
local G, settings = JoltGravity, JoltFieldSettings
local editor = {sequence = 0}
G.Editor = editor
local areaVisible = GetConVar("vjolt_gravity_show_fields")
local directionVisible = GetConVar("vjolt_gravity_show_direction")
local radiusConVar = GetConVar("vjolt_gravity_radius")
local strengthConVar = GetConVar("vjolt_gravity_strength")
local areaColor = Color(80, 210, 255, 180)
local previewColor = Color(120, 220, 255)
local directionColor = Color(255, 160, 70)
local mutedColor = Color(185, 195, 205)
local zeroAngle = Angle(0, 0, 0)
local modes = {I.Token("jolt.world_gravity"), I.Token("jolt.object_gravity_all_physics_bones"), I.Token("jolt.region_gravity")}
surface.CreateFont("JoltGravityHudTitle", {font = "Microsoft YaHei", size = 25, weight = 700, extended = true, antialias = true})
surface.CreateFont("JoltGravityHudText", {font = "Microsoft YaHei", size = 19, weight = 400, extended = true, antialias = true})

local function activeWeapon()
    local player = LocalPlayer()
    if not IsValid(player) or not player:Alive() or player:InVehicle() then return end
    local weapon = player:GetActiveWeapon()
    if IsValid(weapon) and weapon:GetClass() == "weapon_jolt_gravity" then return weapon, player end
end

local function inputAllowed(player)
    return not gui.IsGameUIVisible() and not vgui.CursorVisible() and not player:IsTyping()
end

local function useHeld()
    local binding = input.LookupBinding("+use", true)
    local code = binding and input.GetKeyCode(binding)
    return code ~= nil and code ~= KEY_NONE and input.IsButtonDown(code)
end

function editor.Cancel(weapon)
    if weapon ~= nil and editor.weapon ~= weapon then return end
    editor.weapon, editor.angle, editor.view, editor.anchor = nil, nil, nil, nil
    editor.editing, editor.pending, editor.expires = false, nil, nil
end

local function beginEdit(weapon, player, view)
    if editor.weapon ~= weapon then editor.Cancel() end
    if editor.editing or editor.pending then return end
    local angle = editor.angle or weapon:GetForceAngle()
    editor.angle = Angle(math.Clamp(math.NormalizeAngle(angle.p), -90, 90), math.NormalizeAngle(angle.y), 0)
    -- A vertical force has no meaningful yaw. Start its first tilt towards the
    -- current view so dragging up/down is useful whichever way the player faces.
    if math.abs(editor.angle.p) > 89.99 then editor.angle.y = view.y end
    editor.weapon, editor.editing = weapon, true
    editor.view = Angle(view.p, view.y, view.r)
    local trace = player:GetEyeTrace()
    editor.anchor = trace.Hit and trace.HitPos or player:GetShootPos() + view:Forward() * 128
end

local function finishEdit()
    editor.editing = false
    editor.sequence = (editor.sequence + 1) % 65536
    editor.pending, editor.expires = editor.sequence, RealTime() + 3
    net.Start("jolt_gravity_direction_v1")
    net.WriteEntity(editor.weapon); net.WriteUInt(editor.sequence, 16)
    net.WriteFloat(editor.angle.p); net.WriteFloat(math.NormalizeAngle(editor.angle.y))
    net.SendToServer()
end

hook.Add("InputMouseApply", "JoltGravity.Direction", function(command, x, y, view)
    local weapon, player = activeWeapon()
    if not weapon or not inputAllowed(player) or not (command:KeyDown(IN_USE) or useHeld()) then return end
    beginEdit(weapon, player, view)
    if editor.editing then
        editor.angle.p = math.Clamp(editor.angle.p + y * 0.15, -90, 90)
        editor.angle.y = math.NormalizeAngle(editor.angle.y - x * 0.15)
        command:SetViewAngles(editor.view)
    end
    return true -- Consume mouse look only while this weapon is using E.
end)

hook.Add("CreateMove", "JoltGravity.Direction", function(command)
    local weapon, player = activeWeapon()
    if not weapon or not inputAllowed(player) then editor.Cancel(); return end
    if editor.weapon and editor.weapon ~= weapon then editor.Cancel() end
    if editor.pending and RealTime() > editor.expires then
        editor.Cancel()
        player:ChatPrint(I.Phrase("jolt.jolt_gravity_direction_sync_timed_out_the_server_direction_was_resto"))
    end
    if command:KeyDown(IN_USE) then
        beginEdit(weapon, player, command:GetViewAngles())
    elseif editor.editing then
        finishEdit()
    end
    if editor.editing then command:SetViewAngles(editor.view) end
    if editor.editing or editor.pending then
        -- Wait for the authoritative acknowledgement before allowing a shot;
        -- usercmds and net messages may arrive at the server in different ticks.
        command:RemoveKey(IN_ATTACK)
        command:RemoveKey(IN_ATTACK2)
    end
    -- Keep IN_USE so the server also blocks attacks/reload while editing.
end)

net.Receive("jolt_gravity_direction_v1", function()
    local weapon, sequence = net.ReadEntity(), net.ReadUInt(16)
    local pitch, yaw, accepted = net.ReadFloat(), net.ReadFloat(), net.ReadBool()
    if weapon ~= editor.weapon or sequence ~= editor.pending then return end
    editor.angle = Angle(pitch, yaw, 0)
    editor.pending, editor.expires, editor.view, editor.anchor = nil, nil, nil, nil
    if not accepted then
        LocalPlayer():ChatPrint(I.Phrase("jolt.jolt_gravity_adjustment_too_fast_or_invalid_direction_the_server_dir"))
    end
end)

hook.Add("OnReloaded", "JoltGravity.Direction", function() editor.Cancel() end)

local function newArrow()
    return {origin = Vector(), tip = Vector(), a = Vector(), b = Vector(), c = Vector(), d = Vector()}
end

local function setVector(vector, x, y, z)
    vector.x, vector.y, vector.z = x, y, z
end

local function fillArrow(arrow, center, direction, length)
    local x, y, z = direction.x, direction.y, direction.z
    local magnitude = math.sqrt(x * x + y * y + z * z)
    arrow.visible = magnitude > 0.00001
    if not arrow.visible then return end
    x, y, z = x / magnitude, y / magnitude, z / magnitude
    local rx, ry, rz = 0, 0, 0
    if math.abs(z) < 0.95 then
        local horizontal = math.sqrt(x * x + y * y)
        rx, ry = -y / horizontal, x / horizontal
    else
        local vertical = math.sqrt(y * y + z * z)
        ry, rz = -z / vertical, y / vertical
    end
    local ux, uy, uz = y * rz - z * ry, z * rx - x * rz, x * ry - y * rx
    local tipX, tipY, tipZ = center.x + x * length, center.y + y * length, center.z + z * length
    local head, width = math.min(length * 0.3, 22), math.min(length * 0.15, 11)
    local bx, by, bz = tipX - x * head, tipY - y * head, tipZ - z * head
    setVector(arrow.origin, center.x, center.y, center.z)
    setVector(arrow.tip, tipX, tipY, tipZ)
    setVector(arrow.a, bx + rx * width, by + ry * width, bz + rz * width)
    setVector(arrow.b, bx - rx * width, by - ry * width, bz - rz * width)
    setVector(arrow.c, bx + ux * width, by + uy * width, bz + uz * width)
    setVector(arrow.d, bx - ux * width, by - uy * width, bz - uz * width)
end

local function drawArrow(arrow, writeZ)
    if not arrow.visible then return end
    render.DrawLine(arrow.origin, arrow.tip, directionColor, writeZ == true)
    render.DrawLine(arrow.tip, arrow.a, directionColor, writeZ == true)
    render.DrawLine(arrow.tip, arrow.b, directionColor, writeZ == true)
    render.DrawLine(arrow.tip, arrow.c, directionColor, writeZ == true)
    render.DrawLine(arrow.tip, arrow.d, directionColor, writeZ == true)
end

local placed, snapshot = {}, nil
local function rebuildFields()
    placed, snapshot = {}, G.State
    for _, field in ipairs(snapshot.fields) do
        -- Radial fields (including black holes) have their own entity effects.
        if field.mode == 0 then
            local half = field.halfExtent
            local record = {
                center = field.center, shape = field.shape, radius = field.radius,
                mins = Vector(-half.x, -half.y, -half.z), maxs = half,
                bound = field.shape == 0 and field.radius or half:Length(), arrow = newArrow(),
            }
            fillArrow(record.arrow, field.center, field.gravity, math.Clamp(half.x * 0.4, 32, 144))
            placed[#placed + 1] = record
        end
    end
end
hook.Add("JoltGravityChanged", "JoltGravity.Visuals", rebuildFields)

local function inView(field, eye, forward)
    local x, y, z = field.center.x - eye.x, field.center.y - eye.y, field.center.z - eye.z
    local distance = field.bound + 4096
    return x * x + y * y + z * z <= distance * distance
        and x * forward.x + y * forward.y + z * forward.z >= -field.bound
end

local previewArrow, previewMins, previewMaxs = newArrow(), Vector(), Vector()
local editArrow, editCenter = newArrow(), Vector()
hook.Remove("PostDrawTranslucentRenderables", "JoltGravity.Preview")
hook.Add("PostDrawTranslucentRenderables", "JoltGravity.Visuals", function(depth, skybox, skybox3D)
    if depth or skybox or skybox3D then return end
    local showArea, showDirection = areaVisible:GetBool(), directionVisible:GetBool()
    if not showArea and not showDirection and not editor.editing and not editor.pending then return end
    local player = LocalPlayer()
    if not IsValid(player) then return end
    if G.State ~= snapshot then rebuildFields() end
    local eye, forward = player:EyePos(), player:GetAimVector()
    if showArea or showDirection then
        local drawn = 0
        for _, field in ipairs(placed) do
            if inView(field, eye, forward) then
                if showArea then
                    if field.shape == 1 then
                        render.DrawWireframeBox(field.center, zeroAngle, field.mins, field.maxs, areaColor, true)
                    else
                        render.DrawWireframeSphere(field.center, field.radius, 16, 8, areaColor, true)
                    end
                end
                if showDirection then drawArrow(field.arrow) end
                drawn = drawn + 1
                if drawn >= 64 then break end
            end
        end
    end
    local weapon = activeWeapon()
    if not weapon then return end
    local trace = player:GetEyeTrace()
    local center = editor.weapon == weapon and editor.anchor or trace.HitPos
    local mode = weapon:GetGravityMode()
    if not center or (mode == 2 and not trace.Hit and not editor.anchor) then return end
    local radius = settings.Clamp(radiusConVar:GetFloat(), settings.Gravity[2], player)
    if mode == 2 and showArea then
        if weapon:GetBoxField() then
            setVector(previewMins, -radius, -radius, -radius)
            setVector(previewMaxs, radius, radius, radius)
            render.DrawWireframeBox(center, zeroAngle, previewMins, previewMaxs, previewColor, true)
        else
            render.DrawWireframeSphere(center, radius, 16, 8, previewColor, true)
        end
    end
    if showDirection or editor.editing or editor.pending then
        local angle = editor.weapon == weapon and editor.angle or weapon:GetForceAngle()
        fillArrow(previewArrow, center, angle:Forward(), math.Clamp(radius * 0.4, 48, 144))
        drawArrow(previewArrow)
        if editor.editing or editor.pending then
            -- Also keep a small gizmo near the camera: a far-away target or an
            -- arrow pointing into the floor must not make editing invisible.
            setVector(editCenter, eye.x + forward.x * 96, eye.y + forward.y * 96, eye.z + forward.z * 96)
            fillArrow(editArrow, editCenter, angle:Forward(), 28)
            render.DrawWireframeSphere(editCenter, 2, 8, 4, directionColor, false)
            drawArrow(editArrow)
        end
    end
end)

local hud = {}
local function layoutHUD()
    local width, height = ScrW(), ScrH()
    hud.x, hud.y = width < 1280 and 12 or 24, math.floor(height * 0.5) - 50
end
layoutHUD()
hook.Add("OnScreenSizeChanged", "JoltGravity.HUD", layoutHUD)
hook.Add("JoltLanguageChanged", "JoltGravity.HUDLocale", function()
    hud.mode, hud.strength, hud.status = nil, nil, nil
end)

function editor.DrawHUD(weapon)
    local mode, box = weapon:GetGravityMode(), weapon:GetBoxField()
    local player = LocalPlayer()
    local strength = settings.Clamp(strengthConVar:GetFloat(), settings.Gravity[1], player)
    local radius = settings.Clamp(radiusConVar:GetFloat(), settings.Gravity[2], player)
    local angle = editor.weapon == weapon and editor.angle or weapon:GetForceAngle()
    local status = editor.editing and I.Phrase("jolt.adjusting_direction_release_e_to_save") or editor.pending and I.Phrase("jolt.syncing_direction") or I.Phrase("jolt.direction_locked_hold_e_to_adjust")
    if hud.mode ~= mode then
        hud.mode, hud.title = mode, I.Text("jolt.jolt_gravity_controller") .. I.Resolve(modes[mode + 1] or modes[1])
    end
    if hud.strength ~= strength or hud.radius ~= radius or hud.box ~= box then
        hud.strength, hud.radius, hud.box = strength, radius, box
        hud.values = I.Format("jolt.strength_0f_radius_half_size_0f_s", strength, radius, box and I.Phrase("jolt.box") or I.Phrase("jolt.sphere"))
    end
    if hud.pitch ~= angle.p or hud.yaw ~= angle.y or hud.status ~= status then
        hud.pitch, hud.yaw, hud.status = angle.p, angle.y, status
        local direction = angle:Forward()
        hud.direction = string.format("%s  [%.2f, %.2f, %.2f]", status, direction.x, direction.y, direction.z)
    end
    local x, y = hud.x, hud.y
    draw.SimpleText(hud.title, "JoltGravityHudTitle", x, y, previewColor)
    draw.SimpleText(hud.direction, "JoltGravityHudTitle", x, y + 30, directionColor)
    local keys = IsValid(player) and player:IsAdmin()
        and I.Phrase("jolt.lmb_apply_place_rmb_restore_remove_r_change_mode")
        or I.Phrase("jolt.lmb_place_gravity_field_rmb_remove_gravity_field_region_mode_only")
    draw.SimpleText(keys, "JoltGravityHudText", x, y + 64, color_white)
    draw.SimpleText(I.Phrase("jolt.hold_e_mouse_adjust_direction_h_sphere_box_z_undo"), "JoltGravityHudText", x, y + 89, color_white)
    draw.SimpleText(hud.values, "JoltGravityHudText", x, y + 114, color_white)
    draw.SimpleText(I.Phrase("jolt.q_options_jolt_gravity_fields_display_strength_and_range"), "JoltGravityHudText", x, y + 139, mutedColor)
end
