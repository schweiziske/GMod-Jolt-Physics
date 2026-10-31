-- Shared phrase IDs travel unchanged until they reach a player's UI/chat.
JoltI18n = JoltI18n or {}
local I = JoltI18n
if I.Ready then return I end
if SERVER then
    AddCSLuaFile()
    for _, file in ipairs({"en", "zh-CN", "aliases"}) do AddCSLuaFile("jolt_i18n/" .. file .. ".lua") end
end
I.Catalogs = {en = include("jolt_i18n/en.lua"), ["zh-CN"] = include("jolt_i18n/zh-CN.lua")}
I.Aliases = include("jolt_i18n/aliases.lua")
I.Bindings = setmetatable({}, {__mode = "k"})
I.Ready = true

function I.Normalize(locale)
    locale = tostring(locale or ""):lower():gsub("_", "-")
    if locale == "zh" or locale == "zh-cn" or locale == "zh-hans" or locale == "schinese" then return "zh-CN" end
    return "en"
end

function I.GetLocale(player)
    if SERVER and player and player.GetInfo then return I.Normalize(player:GetInfo("jolt_language")) end
    return I.Locale or "en"
end

function I.InLocale(locale, key, ...)
    local catalog = I.Catalogs[I.Normalize(locale)] or I.Catalogs.en
    key = I.Aliases[key] or key
    local text = catalog[key] or I.Catalogs.en[key] or key
    if select("#", ...) == 0 then return text end
    return string.format(text, ...)
end

function I.Text(key, ...) return I.InLocale(I.GetLocale(), key, ...) end
function I.ForPlayer(player, key, ...) return I.InLocale(I.GetLocale(player), key, ...) end
function I.Token(key) return "{" .. key .. "}" end
function I.Phrase(key) return SERVER and I.Token(key) or I.Text(key) end

-- Data-only format arguments; never evaluate network text as Lua.
local function hex(text)
    return (tostring(text):gsub(".", function(char) return string.format("%02x", char:byte()) end))
end
local function unhex(text)
    return (text:gsub("%x%x", function(pair) return string.char(tonumber(pair, 16)) end))
end
function I.Format(key, ...)
    if not SERVER then return I.Text(key, ...) end
    local encoded = {}
    for index = 1, select("#", ...) do
        local value = select(index, ...)
        encoded[index] = (type(value) == "number" and "n" or "s") .. hex(value)
    end
    return "{" .. key .. "|" .. table.concat(encoded, ",") .. "}"
end

function I.Resolve(text, player)
    if type(text) ~= "string" then return text end
    return (text:gsub("{(jolt%.[%w_.]+)([^{}]*)}", function(key, suffix)
        if not I.Catalogs.en[key] then return nil end
        local locale = I.GetLocale(player)
        if suffix == "" then return I.InLocale(locale, key) end
        if suffix:sub(1, 1) ~= "|" or #suffix > 8192 then return nil end
        local args, payload = {}, suffix:sub(2)
        if payload ~= "" then
            for argument in (payload .. ","):gmatch("(.-),") do
                local kind, data = argument:sub(1, 1), argument:sub(2)
                if (kind ~= "n" and kind ~= "s") or #data % 2 ~= 0 or data:find("[^%x]") then return nil end
                local decoded = unhex(data)
                if kind == "n" then decoded = tonumber(decoded); if not decoded then return nil end end
                args[#args + 1] = decoded
                if #args > 32 then return nil end
            end
        end
        local ok, result = pcall(I.InLocale, locale, key, unpack(args))
        return ok and result or nil
    end))
end

function I.Chat(player, text)
    if player and player.ChatPrint then player:ChatPrint(I.Resolve(text, player)) end
end

function I.RegisterAlias(alias, key)
    I.Aliases[alias] = key
    if CLIENT and language then language.Add(alias, I.Text(key)) end
end

-- Sandbox prefixes undo names with Undone_ before translating notifications.
function I.UndoName(token)
    local key = token:match("^{(jolt%.[%w_.]+)}$")
    return key or token
end

-- Only update labels: preserve values, drafts, selection and collapsed state.
function I.Bind(control, method, token)
    if not control then return control end
    control[method](control, I.Resolve(token))
    if not token:find("{jolt.", 1, true) then return control end
    local bindings = I.Bindings[control] or {}
    I.Bindings[control] = bindings
    bindings[method] = token
    return control
end

function I.Control(panel, method, token, ...)
    if method == "SetTitle" or method == "SetName" or method == "SetText" then
        return I.Bind(panel, method, token)
    end
    local control, caption = panel[method](panel, I.Resolve(token), ...)
    if method == "ComboBox" or method == "TextEntry" then
        if caption then I.Bind(caption, "SetText", token) end
    elseif method == "NumSlider" then
        I.Bind(control, "SetText", token)
    elseif method == "AddColumn" then
        I.Bind(control, "SetName", token)
    else
        I.Bind(control, "SetText", token)
    end
    return control, caption
end

function I.Choice(combo, token, data)
    local index = combo:AddChoice(I.Resolve(token), data)
    local bindings = I.Bindings[combo] or {}
    I.Bindings[combo] = bindings
    bindings.choices = bindings.choices or {}
    bindings.choices[index] = token
    return index
end

function I.Refresh()
    if not CLIENT then return end
    local previous = I.Locale
    local convar = GetConVar("gmod_language")
    I.Locale = I.Normalize(convar and convar.GetString and convar:GetString() or "en")
    if RunConsoleCommand then RunConsoleCommand("jolt_language", I.Locale) end
    if language then
        for key in pairs(I.Catalogs.en) do language.Add(key, I.Text(key)) end
        for alias, key in pairs(I.Aliases) do language.Add(alias, I.Text(key)) end
    end
    for control, bindings in pairs(I.Bindings) do
        if IsValid(control) then
            for method, token in pairs(bindings) do
                if method == "choices" then
                    if control.CloseMenu then control:CloseMenu() end
                    for index, phrase in pairs(token) do control.Choices[index] = I.Resolve(phrase) end
                    local selected = control:GetSelectedID()
                    if selected and token[selected] then control:SetText(I.Resolve(token[selected])) end
                else control[method](control, I.Resolve(token)) end
            end
            control:InvalidateLayout(true)
        end
    end
    if previous and previous ~= I.Locale then hook.Run("JoltLanguageChanged", I.Locale) end
end

if CLIENT then
    if CreateClientConVar then CreateClientConVar("jolt_language", "en", false, true, "Jolt locale synchronized with gmod_language") end
    if cvars then
        cvars.AddChangeCallback("gmod_language", I.Refresh, "JoltI18n.Language")
    end
    I.Refresh()
    if hook then
        hook.Add("InitPostEntity", "JoltI18n.Sync", I.Refresh)
        hook.Add("JoltLanguageChanged", "JoltI18n.WeaponInfo", function()
            for _, class in ipairs({"weapon_jolt_blackhole", "weapon_jolt_gravity"}) do
                for _, weapon in ipairs(ents.FindByClass(class)) do weapon.InfoMarkup = nil end
            end
        end)
    end
end
return I
