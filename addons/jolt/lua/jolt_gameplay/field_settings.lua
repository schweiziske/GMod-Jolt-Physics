local I = JoltI18n or include("jolt_i18n/init.lua")
if SERVER then AddCSLuaFile() end

-- The SWEPs, Sandbox controls and server validation share these limits.
JoltFieldSettings = JoltFieldSettings or {}
local settings = JoltFieldSettings
settings.MaxDistance = 32767
settings.Gravity = {
    {key = "strength", cvar = "vjolt_gravity_strength", label = I.Token("jolt.gravity_acceleration"), default = 600, min = 0, max = 10000, playerMax = 1500},
    {key = "radius", cvar = "vjolt_gravity_radius", label = I.Token("jolt.region_radius_box_half_size"), default = 256, min = 16, max = settings.MaxDistance, playerMax = 512},
}
settings.Blackhole = {
    {key = "radius", cvar = "jolt_blackhole_radius", label = I.Token("jolt.effect_radius"), default = 450, min = 64, max = settings.MaxDistance, playerMax = 1024},
    {key = "pull", cvar = "jolt_blackhole_pull", label = I.Token("jolt.pull_acceleration"), default = 2200, min = 100, max = 10000, playerMax = 2000},
    {key = "burst", cvar = "jolt_blackhole_burst", label = I.Token("jolt.burst_acceleration"), default = 6500, min = 100, max = 20000},
    {key = "duration", cvar = "jolt_blackhole_duration", label = I.Token("jolt.pull_duration_seconds"), default = 2.5, min = 0.1, max = 300, decimals = 1},
}

function settings.Maximum(definition, owner)
    if definition.playerMax and not (IsValid(owner) and owner:IsAdmin()) then return definition.playerMax end
    return definition.max
end

function settings.Clamp(value, definition, owner)
    value = tonumber(value)
    if not value or value ~= value or value == math.huge or value == -math.huge then
        value = definition.default
    end
    return math.Clamp(value, definition.min, settings.Maximum(definition, owner))
end

function settings.ReadPlayer(definitions, owner)
    local values = {}
    for _, definition in ipairs(definitions) do
        local fallback = definition.default
        if definitions == settings.Blackhole then
            local convar = GetConVar("jolt_blackhole_default_" .. definition.key)
            if convar then fallback = convar:GetFloat() end
        end
        values[definition.key] = settings.Clamp(owner:GetInfoNum(definition.cvar, fallback), definition, owner)
    end
    return values
end

function settings.ResolveBlackhole(values, owner)
    local result = {}
    for _, definition in ipairs(settings.Blackhole) do
        local value = values[definition.key]
        if value == nil then
            local convar = GetConVar("jolt_blackhole_default_" .. definition.key)
            value = convar and convar:GetFloat() or definition.default
        end
        result[definition.key] = settings.Clamp(value, definition, owner)
    end
    return result
end

if SERVER then
    for _, definition in ipairs(settings.Blackhole) do
        CreateConVar("jolt_blackhole_default_" .. definition.key, tostring(definition.default), FCVAR_ARCHIVE,
            I.Text("jolt.black_hole_default") .. I.Resolve(definition.label), definition.min, definition.max)
    end
else
    for _, definitions in ipairs({settings.Gravity, settings.Blackhole}) do
        for _, definition in ipairs(definitions) do
            CreateClientConVar(definition.cvar, tostring(definition.default), true, true,
                I.Resolve(definition.label), definition.min, definition.max)
        end
    end
    CreateClientConVar("vjolt_gravity_show_fields", "1", true, false, I.Phrase("jolt.show_gravity_region_outlines"), 0, 1)
    CreateClientConVar("vjolt_gravity_show_direction", "1", true, false, I.Phrase("jolt.show_gravity_direction_arrows"), 0, 1)

    local function sliders(panel, definitions)
        for _, definition in ipairs(definitions) do
            I.Control(panel, "NumSlider", definition.label, definition.cvar, definition.min, settings.Maximum(definition, LocalPlayer()), definition.decimals or 0)
        end
    end

    function settings.BuildGravityPanel(panel)
        I.Control(panel, "Help", I.Token("jolt.hold_e_and_move_the_mouse_to_rotate_the_orange_arrow_release_e_to_sa"))
        sliders(panel, settings.Gravity)
        I.Control(panel, "CheckBox", I.Token("jolt.show_region_outlines_including_placed_regions"), "vjolt_gravity_show_fields")
        I.Control(panel, "CheckBox", I.Token("jolt.show_force_direction_arrows"), "vjolt_gravity_show_direction")
        I.Control(panel, "Help", I.Token("jolt.display_toggles_affect_only_your_view_arrows_appear_temporarily_whil"))
    end

    function settings.BuildBlackholePanel(panel)
        I.Control(panel, "Help", I.Token("jolt.the_grenade_and_black_hole_tool_share_these_settings_changes_apply_t"))
        sliders(panel, settings.Blackhole)
        I.Control(panel, "Help", I.Token("jolt.the_fuse_lasts_1_second_after_pulling_for_the_set_duration_the_black"))
    end

    function settings.OpenBlackholePanel()
        if IsValid(settings.BlackholePanel) then
            settings.BlackholePanel:MakePopup()
            return
        end
        local screenWidth, screenHeight = ScrW(), ScrH()
        local frame = vgui.Create("DFrame")
        settings.BlackholePanel = frame
        I.Control(frame, "SetTitle", I.Token("jolt.jolt_black_hole_settings"))
        frame:SetSize(math.min(440, screenWidth - 24), math.min(390, screenHeight - 24))
        frame:Center()
        frame:SetDeleteOnClose(true)
        frame.OnRemove = function(panel)
            if settings.BlackholePanel == panel then settings.BlackholePanel = nil end
        end
        local scroll = vgui.Create("DScrollPanel", frame)
        scroll:Dock(FILL)
        local form = vgui.Create("DForm", scroll)
        form:Dock(TOP)
        I.Control(form, "SetName", I.Token("jolt.next_black_hole"))
        settings.BuildBlackholePanel(form)
        frame:MakePopup()
    end

    hook.Add("OnScreenSizeChanged", "JoltFieldSettings.Layout", function()
        local frame = settings.BlackholePanel
        if not IsValid(frame) then return end
        local screenWidth, screenHeight = ScrW(), ScrH()
        frame:SetSize(math.min(440, screenWidth - 24), math.min(390, screenHeight - 24))
        frame:Center()
    end)
    hook.Add("PopulateToolMenu", "JoltFieldSettings.Menu", function()
        spawnmenu.AddToolMenuOption("Options", "Jolt", "JoltGravitySettings", "#jolt.gravity_fields", "", "", settings.BuildGravityPanel)
        spawnmenu.AddToolMenuOption("Options", "Jolt", "JoltBlackholeSettings", "#jolt.black_hole", "", "", settings.BuildBlackholePanel)
    end)
    hook.Add("OnReloaded", "JoltFieldSettings.Close", function()
        if IsValid(settings.BlackholePanel) then settings.BlackholePanel:Remove() end
    end)
end

return settings
