if SERVER then return end

local I = JoltI18n or include("jolt_i18n/init.lua")
local api = JoltSoftBodies
local config = api.Config
local serverValues, serverControls = {}, {}
local serverAdmin = false

local function addSection(panel, title)
	local form = vgui.Create("DForm", panel)
	I.Control(form, "SetName", title)
	form:SetExpanded(true)
	panel:AddItem(form)
	return form
end

local function addServerSlider(form, definition)
	local control = vgui.Create("DNumSlider", form)
	control:SetText(I.Text(definition.label))
	control:SetMin(definition.min)
	control:SetMax(definition.max)
	control:SetDecimals(definition.decimals or 0)
	control:SetValue(serverValues[definition.cvar] or definition.default)
	control:SetTooltip(definition.cvar)
	control.OnValueChanged = function(self, value)
		if self.JoltLoading or not serverAdmin then return end
		net.Start(config.SettingsSnapshotMessage)
		net.WriteString(definition.cvar)
		net.WriteFloat(value)
		net.SendToServer()
	end
	control.JoltCvar = definition.cvar
	serverControls[definition.cvar] = control
	form:AddItem(control)
	return control
end

local function applyServerValues(values, admin)
	serverAdmin = admin
	for cvar, value in pairs(values) do
		serverValues[cvar] = value
		local control = serverControls[cvar]
		if IsValid(control) then
			control.JoltLoading = true
			control:SetValue(value)
			control.JoltLoading = nil
			control:SetEnabled(serverAdmin)
		end
	end
	for _, control in pairs(serverControls) do
		if IsValid(control) then control:SetEnabled(serverAdmin) end
	end
end

net.Receive(config.SettingsSnapshotMessage, function()
	serverAdmin = net.ReadBool()
	local values = {}
	for _ = 1, net.ReadUInt(8) do values[net.ReadString()] = net.ReadFloat() end
	applyServerValues(values, serverAdmin)
end)

local function requestServerValues()
	net.Start(config.SettingsRequestMessage)
	net.SendToServer()
end

local function buildClientSection(panel)
	local form = addSection(panel, I.Token("jolt.softbody_local_settings"))
	for _, definition in ipairs(config.ClientSettings) do
		local control = I.Control(form, "NumSlider", definition.label, definition.cvar,
			definition.min, definition.max, definition.decimals or 0)
		control:SetTooltip(definition.cvar)
	end
	I.Control(form, "CheckBox", I.Token("jolt.show_selected_object_s_wireframe"), "jolt_softbody_wireframe")
	I.Control(form, "CheckBox", I.Token("jolt.softbody_collision_profile"), "jolt_softbody_collision_profile")
	I.Control(form, "Help", I.Token("jolt.softbody_local_settings_help"))
end

local function buildServerSection(panel)
	local form = addSection(panel, I.Token("jolt.softbody_server_settings"))
	I.Control(form, "Help", I.Token("jolt.softbody_server_settings_help"))
	for _, definition in ipairs(config.ServerSettings) do addServerSlider(form, definition) end
end

local function build(panel)
	panel:Clear()
	buildClientSection(panel)
	buildServerSection(panel)
	requestServerValues()
end

hook.Add("PopulateToolMenu", "JoltSoftBodySettings.Menu", function()
	spawnmenu.AddToolMenuOption("Options", "Jolt", "JoltSoftBodySettings", "#jolt.softbody_settings", "", "", build)
end)

return true
