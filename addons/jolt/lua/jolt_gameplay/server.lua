-- Shared server helpers for the bundled Sandbox toys. Native features stay lazy.
local I = JoltI18n or include("jolt_i18n/init.lua")
local gameplay = {}

function gameplay.NativeFailure(reason)
	return I.Phrase("jolt.native_download_help")
		.. " https://github.com/schweiziske/GMod-Jolt-Physics/releases/latest"
		.. "\n" .. I.Phrase("jolt.native_install_help")
		.. "\n" .. tostring(reason or I.Phrase("jolt.the_matching_physics_backend_is_not_loaded"))
end

function gameplay.LoadFeature(name)
	local loaded, reason = pcall(require, "jolt")
	if not loaded then
		return nil, gameplay.NativeFailure(reason)
	end

	local available, detail = jolt.IsAvailable(name)
	if not available then
		return nil, gameplay.NativeFailure(detail)
	end
	local native, error = jolt.Load(name)
	if not native then return nil, gameplay.NativeFailure(error) end
	return native
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
