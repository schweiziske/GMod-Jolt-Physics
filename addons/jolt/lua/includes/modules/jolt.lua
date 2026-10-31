-- Unified entry point. Requiring this file does not load native modules
-- or take ownership of native handles.
local features = {
	gravity = {module = "joltgravity", global = "joltgravity", probe = "Available"},
	constraints = {module = "joltconstraints", global = "joltconstraints", probe = "Available"},
	softbody = {module = "joltsoftbody", global = "joltsoftbody", probe = "Available"},
}
local order = {"gravity", "constraints", "softbody"}
local loading = {}
local api = {}

function api.ListFeatures()
	local result = {}
	for index, name in ipairs(order) do
		result[index] = name
	end
	return result
end

-- Explicit optional loading: returns the original native table, or nil/reason.
-- GMod native require() publishes globals and need not return a module table.
function api.Load(name)
	local feature = features[name]
	if not feature then
		return nil, "unknown Jolt feature: " .. tostring(name)
	end
	if CLIENT then
		return nil, "Jolt native features are server-only"
	end

	local value = rawget(api, name) or rawget(_G, feature.global)
	if type(value) == "table" then
		rawset(api, name, value)
		return value
	end
	if loading[feature.module] then
		return nil, "recursive Jolt module load: " .. feature.module
	end

	loading[feature.module] = true
	local loaded, reason = pcall(require, feature.module)
	loading[feature.module] = nil
	if not loaded then
		return nil, "cannot load " .. feature.module .. ": " .. tostring(reason)
	end

	value = rawget(_G, feature.global)
	if type(value) ~= "table" then
		return nil, feature.module .. " did not publish " .. feature.global
	end
	rawset(api, name, value)
	return value
end

-- Checks only the requested feature. Unlike ListFeatures, this can load it.
function api.IsAvailable(name)
	local value, reason = api.Load(name)
	if not value then
		return false, reason
	end

	local probe = features[name].probe
	if not probe then
		return true
	end
	if type(value[probe]) ~= "function" then
		return false, "missing native capability probe: " .. probe
	end

	local called, available, detail = pcall(value[probe])
	if not called then
		return false, tostring(available)
	end
	if not available then
		return false, detail or "matching Jolt backend interface is unavailable"
	end
	return true
end

setmetatable(api, {
	__index = function(_, name)
		if not features[name] then
			return nil
		end
		local value, reason = api.Load(name)
		if not value then
			error(reason, 2)
		end
		return value
	end,
})

_G.jolt = api
return api
