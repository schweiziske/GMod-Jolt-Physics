-- Shared server helpers for the bundled Sandbox toys. Native features stay lazy.
local I = JoltI18n or include("jolt_i18n/init.lua")
local gameplay = {}

function gameplay.LoadFeature(name)
	local loaded, reason = pcall(require, "jolt")
	if not loaded then
		return nil, tostring(reason)
	end

	local available, detail = jolt.IsAvailable(name)
	if not available then
		return nil, detail
	end
	return jolt.Load(name)
end

function gameplay.Fail(player, reason)
	if IsValid(player) and reason then
		I.Chat(player, "[Jolt] " .. tostring(reason))
	end
	return false
end

function gameplay.ClampNumber(value, fallback, minimum, maximum)
	value = tonumber(value)
	if not value or value ~= value or value == math.huge or value == -math.huge then
		value = fallback
	end
	return math.Clamp(value, minimum, maximum)
end

function gameplay.TrackEntity(player, entity, category, label)
	player:AddCount(category, entity)
	player:AddCleanup(category, entity)
	undo.Create(I.UndoName(label))
	undo.AddEntity(entity)
	undo.SetPlayer(player)
	undo.Finish()
end

return gameplay
