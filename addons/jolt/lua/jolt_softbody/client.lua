local api = JoltSoftBodies
local config, renderer = api.Config, api.Renderer
local client = {Records = {}, Order = {}, Frames = 0, Bytes = 0, Rejected = 0, nextUpdate = 0}
local distance = CreateClientConVar("jolt_softbody_viewdistance", "3000", true, false, "Soft-body draw distance", 256, 16384)
local drawRate = CreateClientConVar("jolt_softbody_drawrate", "0", true, false, "Soft-body interpolation rate (0 follows client FPS)", 0, 144)
local nextReady, readyAttempts = 0, 0

function client.Remove(id)
	local record = client.Records[id]
	if not record then return end
	if IsValid(record.entity) and record.entity.JoltSoftBodyRecord == record then record.entity.JoltSoftBodyRecord = nil end
	renderer.Destroy(record.render)
	client.Records[id] = nil
	for index = #client.Order, 1, -1 do
		if client.Order[index] == record then table.remove(client.Order, index) break end
	end
end

function client.Unbind(entity)
	local record = entity.JoltSoftBodyRecord
	if record and record.entity == entity then
		record.entity = nil
		record.unboundAt = RealTime()
		renderer.Destroy(record.render)
	end
	entity.JoltSoftBodyRecord = nil
end

local function bind(record, now)
	if IsValid(record.entity) then
		if record.entity:GetSoftBodyID() == record.id then return true end
		client.Remove(record.id)
		return false
	end
	local entity = Entity(record.entityIndex)
	if IsValid(entity) and entity:GetClass() == config.EntityClass and entity:GetSoftBodyID() == record.id then
		record.entity = entity
		entity.JoltSoftBodyRecord = record
		if record.render.revision > 0 then entity:SetRenderBoundsWS(record.render.minimum, record.render.maximum) end
		return true
	end
	if now - record.unboundAt > 10 then client.Remove(record.id) end
	return false
end

local function receiveMeta(bits, reader)
	local id, entityIndex, settings = reader(bits)
	if not id then client.Rejected = client.Rejected + 1 return end
	local existing = client.Records[id]
	if existing and existing.entityIndex == entityIndex then return end
	if existing then client.Remove(id) end
	if #client.Order >= config.MaxRecords then client.Rejected = client.Rejected + 1 return end
	local topology = api.Geometry.Generate(settings)
	local record = {
		id = id, entityIndex = entityIndex, settings = settings, geometry = topology,
		collision = api.Collision.New(topology),
		buffer = api.Buffer.New(topology.vertexCount), render = renderer.New(topology, settings.palette, settings.kind, settings),
		receivedAt = RealTime(), unboundAt = RealTime(), nextLight = 0, visible = false, settled = false,
	}
	client.Records[id] = record
	client.Order[#client.Order + 1] = record
end
net.Receive(config.MetaMessage, function(bits) receiveMeta(bits, api.Protocol.ReadMeta) end)

net.Receive(config.SnapshotMessage, function(bits)
	local record = api.Protocol.ReadFrame(client.Records, bits, RealTime(), CurTime())
	if record then
		client.Frames = client.Frames + 1
		client.Bytes = client.Bytes + math.ceil(bits / 8)
	else
		client.Rejected = client.Rejected + 1
	end
end)

net.Receive(config.RemoveMessage, function(bits)
	if bits == 32 then client.Remove(net.ReadUInt(32)) end
end)

function client.Pick(ox, oy, oz, dx, dy, dz, limit)
	local selected, vertex
	for index = 1, #client.Order do
		local record = client.Order[index]
		if record.visible and record.render.revision > 0 and api.Geometry.RayBounds(record.render, ox, oy, oz, dx, dy, dz, limit) then
			local point, depth = api.Geometry.Raycast(record.buffer.display, record.geometry.indices, ox, oy, oz, dx, dy, dz, limit)
			if point then selected, vertex, limit = record, point, depth end
		end
	end
	return selected, vertex, limit
end

function client.Draw(entity)
	local record = entity.JoltSoftBodyRecord
	if not record or record.id ~= entity:GetSoftBodyID() or not record.visible then return end
	renderer.Draw(record.render, record.settled)
end

local function think()
	local now = RealTime()
	-- Repeated handshakes cover initial sign-on and a client-only Lua refresh.
	if readyAttempts < 4 and now >= nextReady and IsValid(LocalPlayer()) then
		net.Start(config.ReadyMessage)
		net.SendToServer()
		readyAttempts, nextReady = readyAttempts + 1, now + 3
	end
	local frequency = drawRate:GetFloat()
	if frequency > 0 then
		if now < client.nextUpdate then return end
		client.nextUpdate = math.max(client.nextUpdate + 1 / math.max(15, frequency), now)
	else
		client.nextUpdate = now
	end
	if #client.Order == 0 then return end
	local owner = LocalPlayer()
	if not IsValid(owner) then return end
	-- Global EyePos() describes the current render pass and can be zero in Think.
	local eye, maxDistance = owner:EyePos(), distance:GetFloat() ^ 2
	-- Game-time deltas preserve pause/timescale; the buffer owns the remote-time origin.
	local playbackAt = CurTime()
	for index = #client.Order, 1, -1 do
		local record = client.Order[index]
		local latest = record.buffer:Latest()
		if bind(record, now) and latest then
			local changed, settled = record.buffer:Play(playbackAt)
			record.settled = settled
			if changed then
				renderer.UpdateBounds(record.render, record.buffer.display)
			end
			record.visible = eye:DistToSqr(record.render.center) <= maxDistance
			if record.visible and not settled and now >= record.nextLight then
				local light = render.GetLightColor(record.render.center)
				renderer.SetLight(record.render, light.x, light.y, light.z)
				record.nextLight = now + 0.5
			end
			local redraw = record.visible and record.renderedRevision ~= record.buffer.revision
			if redraw then
				renderer.Update(record.render, record.buffer.display, true)
				record.renderedRevision = record.buffer.revision
			end
			if changed or redraw then
				-- Bounds update outside Draw so a culled body can re-enter the view.
				record.entity:SetRenderBoundsWS(record.render.minimum, record.render.maximum)
			end
		end
	end
end

function api.Shutdown()
	if api.ToolClient then api.ToolClient.Shutdown() end
	for index = #client.Order, 1, -1 do client.Remove(client.Order[index].id) end
	hook.Remove("Think", "JoltSoftBodies")
	hook.Remove("ShutDown", "JoltSoftBodies")
end

hook.Add("Think", "JoltSoftBodies", think)
hook.Add("ShutDown", "JoltSoftBodies", api.Shutdown)
concommand.Add("jolt_softbody_client_stats", function()
	local vertices = 0
	for index = 1, #client.Order do vertices = vertices + client.Order[index].geometry.vertexCount end
	Msg(string.format("Jolt soft bodies: %d, vertices: %d, frames: %d, received: %.1f KiB, rejected: %d\n",
		#client.Order, vertices, client.Frames, client.Bytes / 1024, client.Rejected))
	Msg(string.format("DX9 meshes: %d, static writes: %d, dynamic draws: %d, Lua heap: %.1f KiB\n",
		renderer.LiveMeshes, renderer.StaticBuilds, renderer.DynamicDraws, collectgarbage("count")))
	local stats = api.Collision.Stats
	Msg(string.format("Client collision profile: %d traces, mean %.3f ms, max %.3f ms; busiest frame %d traces, max frame total %.3f ms\n",
		stats.calls, stats.seconds*1000/math.max(1,stats.calls), stats.maximum*1000,
		stats.maximumFrameCalls, stats.maximumFrameSeconds*1000))
	if stats.calls == 0 then Msg("Enable jolt_softbody_collision_profile 1, touch a soft body, then run this command again.\n") end
end)
return client
