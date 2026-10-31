local I = JoltI18n or include("jolt_i18n/init.lua")
local config = {
	Protocol = 1,
	MetaMessage = "jolt_softbody_meta_v1",
	SnapshotMessage = "jolt_softbody_frame_v1",
	RemoveMessage = "jolt_softbody_remove_v1",
	ReadyMessage = "jolt_softbody_ready_v1",
	SettingsRequestMessage = "jolt_softbody_settings_request_v1",
	SettingsSnapshotMessage = "jolt_softbody_settings_snapshot_v1",
	EntityClass = "sent_jolt_softbody",
	MaxVertices = 1024,
	MaxRecords = 32,
	SnapshotSlots = 4,
	NearInterval = 1 / 15,
	FarInterval = 1 / 5,
	InterestInterval = 0.25,
	NearDistance = 1200,
	ViewDistance = 3000,
	EditDistance = 600,
	Quantization = 1 / 16,
	CoordinateLimit = 1048576,
	ClothResolution = {8, 16, 24},
	SphereSubdivisions = {1, 2, 3},
	Palette = {
		{name = I.Token("jolt.ocean_blue"), r = 75, g = 148, b = 210},
		{name = I.Token("jolt.terracotta"), r = 218, g = 127, b = 76},
		{name = I.Token("jolt.teal"), r = 84, g = 181, b = 151},
		{name = I.Token("jolt.ivory"), r = 221, g = 215, b = 194},
	},
}

-- Server settings are kept as data so the options page, validation and
-- descriptions stay in sync without duplicating ranges in client Lua.
config.ServerSettings = {
	{cvar = "sbox_maxjolt_softbodies", label = "jolt.softbody_player_limit", default = 3, min = 0, max = 128},
	{cvar = "jolt_softbody_maxrecords", label = "jolt.softbody_live_limit", default = 32, min = 0, max = 256},
	{cvar = "jolt_softbody_maxvertices", label = "jolt.softbody_vertex_budget", default = 4096, min = 0, max = 131072},
	{cvar = "jolt_softbody_maxvertices_per_body", label = "jolt.softbody_vertices_per_body", default = 1024, min = 3, max = 8192},
	{cvar = "jolt_softbody_maxtriangles_per_body", label = "jolt.softbody_triangles_per_body", default = 2048, min = 1, max = 16384},
	{cvar = "jolt_softbody_iterations", label = "jolt.solver_iterations", default = 5, min = 1, max = 32},
	{cvar = "jolt_softbody_damping", label = "jolt.damping", default = 0.35, min = 0, max = 10, decimals = 2},
	{cvar = "jolt_softbody_friction", label = "jolt.friction", default = 0.6, min = 0, max = 1, decimals = 2},
	{cvar = "jolt_softbody_restitution", label = "jolt.restitution", default = 0.05, min = 0, max = 1, decimals = 2},
	{cvar = "jolt_softbody_vertex_radius", label = "jolt.vertex_collision_radius", default = 0.5, min = 0, max = 64, decimals = 1},
	{cvar = "jolt_softbody_max_velocity", label = "jolt.max_velocity", default = 2000, min = 1, max = 100000},
	{cvar = "jolt_softbody_gravity_factor", label = "jolt.gravity_factor", default = 1, min = -100, max = 100, decimals = 2},
	{cvar = "jolt_softbody_player_push_rate", label = "jolt.softbody_player_push_rate", default = 15, min = 1, max = 66},
	{cvar = "jolt_softbody_player_push_budget", label = "jolt.softbody_player_push_budget", default = 32, min = 1, max = 256},
	{cvar = "jolt_softbody_player_push_impulse", label = "jolt.softbody_player_push_impulse", default = 1, min = 0, max = 10, decimals = 2},
	{cvar = "jolt_softbody_net_kib", label = "jolt.softbody_net_budget", default = 96, min = 8, max = 512, decimals = 0},
	{cvar = "jolt_softbody_near_distance", label = "jolt.softbody_near_distance", default = 1200, min = 256, max = 8192},
	{cvar = "jolt_softbody_view_distance", label = "jolt.softbody_replication_distance", default = 3000, min = 256, max = 16384},
	{cvar = "jolt_softbody_near_rate", label = "jolt.softbody_near_rate", default = 15, min = 1, max = 60},
	{cvar = "jolt_softbody_far_rate", label = "jolt.softbody_far_rate", default = 5, min = 1, max = 30},
	{cvar = "jolt_softbody_interest_rate", label = "jolt.softbody_interest_rate", default = 4, min = 1, max = 20},
}

config.ClientSettings = {
	{cvar = "jolt_softbody_viewdistance", label = "jolt.view_distance", default = 3000, min = 256, max = 16384},
	{cvar = "jolt_softbody_drawrate", label = "jolt.interpolation_rate", default = 0, min = 0, max = 144},
}

function config.IsFinite(value)
	return type(value) == "number" and value == value and value > -math.huge and value < math.huge
end

function config.ClampNumber(value, fallback, minimum, maximum)
	value = tonumber(value)
	if not config.IsFinite(value) then return fallback end
	return math.max(minimum, math.min(value, maximum))
end

function config.Sanitize(input)
	input = input or {}
	local function value(lower, camel)
		return input[lower] ~= nil and input[lower] or input[camel]
	end
	return {
		kind = input.kind == 2 and 2 or 1,
		quality = math.floor(config.ClampNumber(input.quality, 2, 1, 3)),
		width = config.ClampNumber(input.width, 160, 32, 512),
		height = config.ClampNumber(input.height, 128, 32, 512),
		mass = config.ClampNumber(input.mass, 4, 1, 50),
		softness = config.ClampNumber(input.softness, 40, 0, 100),
		inflation = config.ClampNumber(input.inflation, 45, 0, 100),
		pins = math.floor(config.ClampNumber(input.pins, 1, 0, 2)),
		palette = math.floor(config.ClampNumber(input.palette, 1, 1, #config.Palette)),
		rotation = config.ClampNumber(input.rotation, 0, -180, 180),
		playerCollision = input.playercollision == 1 or input.playerCollision == true,
		iterations = math.floor(config.ClampNumber(input.iterations, 5, 1, 32)),
		damping = config.ClampNumber(input.damping, 0.35, 0, 10),
		friction = config.ClampNumber(input.friction, 0.6, 0, 1),
		restitution = config.ClampNumber(input.restitution, 0.05, 0, 1),
		vertexRadius = config.ClampNumber(value("vertexradius", "vertexRadius"), 0.5, 0, 64),
		maxVelocity = config.ClampNumber(value("maxvelocity", "maxVelocity"), 2000, 1, 100000),
		gravityFactor = config.ClampNumber(value("gravity", "gravityFactor"), 1, -100, 100),
	}
end

function config.VertexCount(kind, quality)
	if kind == 2 then return 10 * 4 ^ config.SphereSubdivisions[quality] + 2 end
	local resolution = config.ClothResolution[quality]
	return resolution * resolution
end

-- Payload only; reserve extra bytes when accounting for engine framing.
function config.SnapshotBytes(vertexCount)
	return math.ceil((225 + vertexCount * 49) / 8)
end

return config
