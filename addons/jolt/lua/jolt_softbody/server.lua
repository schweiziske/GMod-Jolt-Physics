local I = JoltI18n or include("jolt_i18n/init.lua")
local api = JoltSoftBodies
local config, geometry, protocol = api.Config, api.Geometry, api.Protocol
local gameplay = include("jolt_gameplay/server.lua")
local server = {Records = {}, Order = {}, VertexCount = 0}
local nextID = _G.JoltSoftBodyNextID or 1
local limit = CreateConVar("sbox_maxjolt_softbodies", "3", FCVAR_ARCHIVE, "Soft bodies per player", 0, 128)
local recordLimit = CreateConVar("jolt_softbody_maxrecords", "32", FCVAR_ARCHIVE, "Maximum live soft bodies", 0, 256)
local vertexLimit = CreateConVar("jolt_softbody_maxvertices", "4096", FCVAR_ARCHIVE, "Server soft-body vertex budget", 0, 131072)
local bodyVertexLimit = CreateConVar("jolt_softbody_maxvertices_per_body", "1024", FCVAR_ARCHIVE, "Maximum vertices in one soft body", 3, 8192)
local bodyTriangleLimit = CreateConVar("jolt_softbody_maxtriangles_per_body", "2048", FCVAR_ARCHIVE, "Maximum triangles in one soft body", 1, 16384)
local iterations = CreateConVar("jolt_softbody_iterations", "5", FCVAR_ARCHIVE, "Default soft-body solver iterations", 1, 32)
local damping = CreateConVar("jolt_softbody_damping", "0.35", FCVAR_ARCHIVE, "Default soft-body damping", 0, 10)
local friction = CreateConVar("jolt_softbody_friction", "0.6", FCVAR_ARCHIVE, "Default soft-body friction", 0, 1)
local restitution = CreateConVar("jolt_softbody_restitution", "0.05", FCVAR_ARCHIVE, "Default soft-body restitution", 0, 1)
local vertexRadius = CreateConVar("jolt_softbody_vertex_radius", "0.5", FCVAR_ARCHIVE, "Soft-body vertex collision radius", 0, 64)
local maxVelocity = CreateConVar("jolt_softbody_max_velocity", "2000", FCVAR_ARCHIVE, "Soft-body vertex speed limit", 1, 100000)
local gravityFactor = CreateConVar("jolt_softbody_gravity_factor", "1", FCVAR_ARCHIVE, "Maximum absolute soft-body gravity multiplier", -100, 100)
local playerPushRate = CreateConVar("jolt_softbody_player_push_rate", "15", FCVAR_ARCHIVE, "Player to soft-body checks per second", 1, 66)
local playerPushBudget = CreateConVar("jolt_softbody_player_push_budget", "32", FCVAR_ARCHIVE, "Maximum player-soft-body checks per tick", 1, 256)
local playerPushImpulse = CreateConVar("jolt_softbody_player_push_impulse", "1", FCVAR_ARCHIVE, "Player impulse multiplier", 0, 10)
cleanup.Register("jolt_softbodies")

util.AddNetworkString(config.SettingsRequestMessage)
util.AddNetworkString(config.SettingsSnapshotMessage)

local settingsByCvar = {}
for _, definition in ipairs(config.ServerSettings) do settingsByCvar[definition.cvar] = definition end

local function sendSettings(owner)
	net.Start(config.SettingsSnapshotMessage)
	net.WriteBool(IsValid(owner) and owner:IsAdmin() or game.SinglePlayer())
	net.WriteUInt(#config.ServerSettings, 8)
	for _, definition in ipairs(config.ServerSettings) do
		local convar = GetConVar(definition.cvar)
		net.WriteString(definition.cvar)
		net.WriteFloat(convar and convar:GetFloat() or definition.default)
	end
	if IsValid(owner) then net.Send(owner) else net.Broadcast() end
end

net.Receive(config.SettingsRequestMessage, function(_, owner)
	if not IsValid(owner) then return end
	sendSettings(owner)
end)

net.Receive(config.SettingsSnapshotMessage, function(_, owner)
	if not IsValid(owner) or not (owner:IsAdmin() or game.SinglePlayer()) then return end
	local cvar = net.ReadString()
	local value = net.ReadFloat()
	local definition = settingsByCvar[cvar]
	if not definition or value ~= value or value == math.huge or value == -math.huge then return end
	value = math.Clamp(value, definition.min, definition.max)
	local convar = GetConVar(cvar)
	if not convar then return end
	convar:SetString(string.format("%.*f", definition.decimals or 0, value))
	sendSettings(owner)
end)

local function updateBounds(record, state)
	state = state or record.snapshot
	local x, y, z = (state.minX + state.maxX) * 0.5, (state.minY + state.maxY) * 0.5,
		(state.minZ + state.maxZ) * 0.5
	record.center:SetUnpacked(x, y, z)
	record.minimum:SetUnpacked(state.minX - x - 4, state.minY - y - 4, state.minZ - z - 4)
	record.maximum:SetUnpacked(state.maxX - x + 4, state.maxY - y + 4, state.maxZ - z + 4)
	record.entity:SetPos(record.center)
	record.entity:SetCollisionBounds(record.minimum, record.maximum)
end

function server.Sample(record, now, force)
	local skip = not force and not record.snapshot.active and now < record.nextFullRead
	local ok = record.body:ReadSnapshot(record.snapshot, skip)
	if not ok then return false end
	if not record.snapshot.updated then return true end
	record.nextFullRead = now + 1
	local valid, changed = protocol.Prepare(record.snapshot, record.frame)
	if not valid then return false end
	if changed or record.snapshot.active or record.sequence == 0 then
		record.sequence = record.sequence % 65535 + 1
		record.time = now
		updateBounds(record)
	end
	return true
end

local function nativeMesh(mesh, mass)
	local vertices, triangles = {}, {}
	for index = 1, mesh.vertexCount do
		local offset = (index - 1) * 3
		vertices[index] = {pos = Vector(mesh.positions[offset + 1], mesh.positions[offset + 2],
			mesh.positions[offset + 3]), mass = mass / mesh.vertexCount, pinned = mesh.pinned[index]}
	end
	for corner = 1, #mesh.indices, 3 do
		triangles[#triangles + 1] = {mesh.indices[corner], mesh.indices[corner + 1], mesh.indices[corner + 2]}
	end
	return vertices, triangles
end

function server.Spawn(owner, input, position, yaw)
	return server.SpawnPrepared(owner, config.Sanitize(input), position, Angle(0, yaw, 0))
end

function server.SpawnPrepared(owner, settings, position, angles, velocity)
	if not IsValid(owner) or not owner:Alive() then return end
	local count = config.VertexCount(settings.kind, settings.quality)
	local owned = 0
	for index = 1, #server.Order do
		if server.Order[index].owner == owner then owned = owned + 1 end
	end
	if not owner:CheckLimit("jolt_softbodies") then return nil end
	-- Preserve the original admin/singleplayer safety cap; multiplayer player
	-- quotas are decided by Sandbox's PlayerCheckLimit hook above.
	if (owner:IsAdmin() or game.SinglePlayer()) and owned >= limit:GetInt() then
		owner:LimitHit("jolt_softbodies")
		return nil
	end
	if count > bodyVertexLimit:GetInt() or (settings.kind == 2 and 20 * 4 ^ config.SphereSubdivisions[settings.quality] > bodyTriangleLimit:GetInt())
		or (settings.kind == 1 and 2 * (config.ClothResolution[settings.quality] - 1) ^ 2 > bodyTriangleLimit:GetInt()) then
		return nil, I.Phrase("jolt.the_soft_body_exceeds_the_configured_mesh_limit")
	end
	if #server.Order >= math.min(recordLimit:GetInt(), config.MaxRecords) or server.VertexCount + count > vertexLimit:GetInt() then
		return nil, I.Phrase("jolt.the_server_s_soft_body_vertex_budget_is_exhausted_lower_quality_or_r")
	end
	if nextID > 2147483647 then return nil, I.Phrase("jolt.soft_body_ids_exhausted_change_the_map") end
	local native, reason = gameplay.LoadFeature("softbody")
	if not native then return nil, reason end
	local mesh = geometry.Generate(settings)
	local vertices, triangles = nativeMesh(mesh, settings.mass)
	local softness = settings.softness / 100
	local body, detail = native.Create(vertices, triangles, {
		pos = position, angles = angles, velocity = velocity or Vector(),
		iterations = math.Clamp(settings.iterations or iterations:GetInt(), 1, iterations:GetInt()),
		dihedralBend = false,
		edgeCompliance = 10 ^ (-7 + softness * 4),
		bendCompliance = 10 ^ (-5 + softness * 4),
		pressure = settings.kind == 2 and settings.mass * 600 * settings.width * 0.5 * settings.inflation / 100 or 0,
		damping = math.Clamp(settings.damping or damping:GetFloat(), 0, damping:GetFloat()),
		friction = math.Clamp(settings.friction or friction:GetFloat(), 0, friction:GetFloat()),
		restitution = math.Clamp(settings.restitution or restitution:GetFloat(), 0, restitution:GetFloat()),
		vertexRadius = math.Clamp(settings.vertexRadius or vertexRadius:GetFloat(), 0, vertexRadius:GetFloat()),
		maxVelocity = math.Clamp(settings.maxVelocity or maxVelocity:GetFloat(), 1, maxVelocity:GetFloat()),
		gravityFactor = math.Clamp(settings.gravityFactor or gravityFactor:GetFloat(),
			-math.max(1, math.abs(gravityFactor:GetFloat())), math.max(1, math.abs(gravityFactor:GetFloat()))),
	})
	if not body then return nil, detail end
	if not body.ReadSnapshot or not body.ReadVertex or not body.ReadState or not body.Trace then
		body:Destroy()
		return nil, I.Phrase("jolt.update_the_soft_body_dll_and_physics_backend_together_live_trace_sta")
	end
	local entity = ents.Create(config.EntityClass)
	if not IsValid(entity) then
		body:Destroy()
		return nil, I.Phrase("jolt.unable_to_create_the_soft_body_entity")
	end
	local id = nextID
	nextID = nextID + 1
	_G.JoltSoftBodyNextID = nextID
	entity:SetPos(position)
	entity:SetCreator(owner)
	entity:SetSoftBodyID(id)
	entity:SetPlayerCollision(settings.playerCollision == true)
	entity:Spawn()
	if not IsValid(entity) then body:Destroy() return nil, I.Phrase("jolt.soft_body_entity_creation_failed") end
	local record = {
		id = id, entity = entity, owner = owner, body = body, settings = settings, geometry = mesh,
		snapshot = api.Buffer.Native(count), frame = protocol.Frame(count),
		sequence = 0, time = 0, nextSample = 0, nextFullRead = 0, forceSample = false,
		center = Vector(), minimum = Vector(), maximum = Vector(), recipients = {}, reliableRecipients = {},
		bounds = {}, playerPushTimes = {},
		pushCenter = Vector(), pushExtents = Vector(), pushDelta = Vector(),
	}
	entity.JoltSoftBodyRecord = record
	server.Records[id] = record
	server.Order[#server.Order + 1] = record
	server.VertexCount = server.VertexCount + count
	if not server.Sample(record, CurTime(), true) then
		entity:Remove()
		return nil, I.Phrase("jolt.unable_to_read_the_initial_soft_body_snapshot")
	end
	if entity.CPPISetOwner then entity:CPPISetOwner(owner) end
	gameplay.TrackEntity(owner, entity, "jolt_softbodies", I.Token("jolt.jolt_soft_body"))
	return record
end

function server.Remove(id)
	local record = server.Records[id]
	if not record then return end
	server.Records[id] = nil
	if api.Editing then api.Editing.ReleaseRecord(record) end
	if api.Replication then api.Replication.Remove(record) end
	record.body:Destroy()
	if record.restore then record.restore() end
	server.VertexCount = server.VertexCount - record.geometry.vertexCount
	for index = #server.Order, 1, -1 do
		if server.Order[index] == record then table.remove(server.Order, index) break end
	end
end

-- Collision partition bounds follow every physics tick, independently of the
-- 15 Hz visual snapshots. A fast moving body must not leave its trace proxy.

local pushCursor = 1
local function pushPlayersIntoSoftBodies(now)
	local players = player.GetAll()
	local total = #players * #server.Order
	if total == 0 then return end
	local budget = playerPushBudget:GetInt()
	for step = 1, math.min(budget, total) do
		local slot = (pushCursor - 1) % total + 1
		local playerIndex = math.floor((slot - 1) / math.max(1, #server.Order)) + 1
		local bodyIndex = (slot - 1) % math.max(1, #server.Order) + 1
		local owner = players[playerIndex]
		local record = server.Order[bodyIndex]
		pushCursor = pushCursor + 1
		local nextPush = record and record.playerPushTimes[owner] or 0
		if IsValid(owner) and owner:Alive() and record and not record.settings.playerCollision and now >= nextPush then
			local minimum, maximum = owner:OBBMins(), owner:OBBMaxs()
			local position = owner:GetPos()
			local halfX, halfY, halfZ = (maximum.x - minimum.x) * 0.5,
				(maximum.y - minimum.y) * 0.5, (maximum.z - minimum.z) * 0.5
			local playerMinX, playerMaxX = position.x + minimum.x, position.x + maximum.x
			local playerMinY, playerMaxY = position.y + minimum.y, position.y + maximum.y
			local playerMinZ, playerMaxZ = position.z + minimum.z, position.z + maximum.z
			local bounds = record.bounds
			if playerMaxX >= bounds.minX and playerMinX <= bounds.maxX
				and playerMaxY >= bounds.minY and playerMinY <= bounds.maxY
				and playerMaxZ >= bounds.minZ and playerMinZ <= bounds.maxZ then
				record.pushExtents:SetUnpacked(halfX, halfY, halfZ)
				record.pushCenter:SetUnpacked(position.x + (minimum.x + maximum.x) * 0.5,
					position.y + (minimum.y + maximum.y) * 0.5, position.z + (minimum.z + maximum.z) * 0.5)
				local hit = record.body:Trace(record.pushCenter, record.pushDelta, record.pushExtents)
				if hit and hit.Hit then
					local velocity = owner:GetVelocity()
					if velocity:Length() > 1 then
						local physics = owner:GetPhysicsObject()
						local mass = IsValid(physics) and physics:GetMass() or 80
						local impulse = velocity * (mass * playerPushImpulse:GetFloat())
						if record.body.AddImpulseAt then
							record.body:AddImpulseAt(hit.HitPos, impulse, math.max(8, record.pushExtents:Length()))
						else
							record.body:AddImpulse(impulse)
						end
						record.forceSample = true
					end
				end
			end
			record.playerPushTimes[owner] = now + 1 / playerPushRate:GetFloat()
		end
	end
end

hook.Add("Tick", "JoltSoftBodyBounds", function()
	for index = 1, #server.Order do
		local record = server.Order[index]
		if IsValid(record.entity) and record.body:ReadState(record.bounds) then updateBounds(record, record.bounds) end
	end
	pushPlayersIntoSoftBodies(CurTime())
end)

local function think()
	local now = CurTime()
	if api.Editing then api.Editing.Think(now) end
	for index = #server.Order, 1, -1 do
		local record = server.Order[index]
		if not IsValid(record.entity) or not record.body:IsValid() then
			if IsValid(record.entity) then record.entity:Remove() else server.Remove(record.id) end
		elseif now >= record.nextSample or record.forceSample then
			local ok = server.Sample(record, now, record.forceSample)
			record.forceSample = false
			record.nextSample = now + config.NearInterval
			if not ok then record.entity:Remove() end
		end
	end
	if api.Replication then api.Replication.Think(now) end
end

function api.Shutdown()
	for index = #server.Order, 1, -1 do
		local record = server.Order[index]
		if IsValid(record.entity) then record.entity:Remove() end
		server.Remove(record.id)
	end
	hook.Remove("Think", "JoltSoftBodies")
	hook.Remove("Tick", "JoltSoftBodyBounds")
	hook.Remove("PlayerInitialSpawn", "JoltSoftBodies")
	hook.Remove("PlayerDisconnected", "JoltSoftBodies")
	hook.Remove("ShutDown", "JoltSoftBodies")
end

hook.Add("Think", "JoltSoftBodies", think)
hook.Add("ShutDown", "JoltSoftBodies", api.Shutdown)
return server
