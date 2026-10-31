local api = JoltSoftBodies
local config, server, protocol = api.Config, api.Server, api.Protocol
local replication = {Peers = {}, Order = {}, Bytes = 0, Frames = 0}
local rate = CreateConVar("jolt_softbody_net_kib", "96", FCVAR_ARCHIVE, "Soft-body KiB/s budget per player", 8, 512)
local nearDistance = CreateConVar("jolt_softbody_near_distance", "1200", FCVAR_ARCHIVE, "Soft-body near replication distance", 256, 8192)
local viewDistance = CreateConVar("jolt_softbody_view_distance", "3000", FCVAR_ARCHIVE, "Soft-body replication distance", 256, 16384)
local nearRate = CreateConVar("jolt_softbody_near_rate", "15", FCVAR_ARCHIVE, "Soft-body near snapshots per second", 1, 60)
local farRate = CreateConVar("jolt_softbody_far_rate", "5", FCVAR_ARCHIVE, "Soft-body far snapshots per second", 1, 30)
local interestRate = CreateConVar("jolt_softbody_interest_rate", "4", FCVAR_ARCHIVE, "Soft-body interest refreshes per second", 1, 20)

util.AddNetworkString(config.MetaMessage)
util.AddNetworkString(config.SnapshotMessage)
util.AddNetworkString(config.RemoveMessage)
util.AddNetworkString(config.ReadyMessage)

local function addPlayer(owner)
	if replication.Peers[owner] then return replication.Peers[owner] end
	local peer = {player = owner, entries = {}, tokens = 0, time = CurTime(), nextInterest = 0, nextReady = 0, ready = false, cursor = 1}
	replication.Peers[owner] = peer
	replication.Order[#replication.Order + 1] = peer
	return peer
end

local function removeMessage(id, owner)
	net.Start(config.RemoveMessage)
	net.WriteUInt(id, 32)
	net.Send(owner)
end

local function refreshInterest(peer, now)
	local owner = peer.player
	local eye = owner:EyePos()
	local nearSquared, farSquared = nearDistance:GetFloat() ^ 2, viewDistance:GetFloat() ^ 2
	for index = 1, #server.Order do
		local record = server.Order[index]
		local entry = peer.entries[record.id]
		if not entry then
			entry = {visible = false, known = false, sequence = -1, nextSend = 0, interval = config.FarInterval, send = false, reliable = false}
			peer.entries[record.id] = entry
		end
		local distance = eye:DistToSqr(record.center)
		local visible = distance <= farSquared and owner:TestPVS(record.entity)
		if not visible and entry.known then
			removeMessage(record.id, owner)
			entry.known, entry.sequence = false, -1
		end
		entry.visible = visible
		entry.interval = distance <= nearSquared and 1 / nearRate:GetFloat() or 1 / farRate:GetFloat()
	end
	peer.nextInterest = now + 1 / interestRate:GetFloat()
end

local function sendFrame(record, recipients, reliable)
	if #recipients == 0 then return end
	net.Start(config.SnapshotMessage, not reliable)
	protocol.WriteFrame(record.id, record.sequence, record.time, record.frame)
	net.Send(recipients)
	replication.Frames = replication.Frames + 1
	replication.Bytes = replication.Bytes + (config.SnapshotBytes(record.geometry.vertexCount) + 24) * #recipients
end

local function schedulePeer(peer, now)
	local count = #server.Order
	if not peer.ready or count == 0 then return end
	local start = (peer.cursor - 1) % count + 1
	for offset = 0, count - 1 do
		local index = (start + offset - 1) % count + 1
		local record = server.Order[index]
		local entry = peer.entries[record.id]
		if entry and entry.visible and entry.sequence ~= record.sequence and now >= entry.nextSend then
			local cost = config.SnapshotBytes(record.geometry.vertexCount) + 96
			if not entry.known then cost = cost + protocol.MetaBytes(record) end
			if peer.tokens < cost then
				-- Reserve credit for this body; small packets must not starve a large mesh.
				peer.cursor = index
				return
			end
			entry.send, entry.reliable = true, not entry.known or not record.frame.active
			if not entry.known then
				net.Start(config.MetaMessage)
				protocol.WriteMeta(record)
				net.Send(peer.player)
			end
			entry.known, entry.sequence, entry.nextSend = true, record.sequence, now + entry.interval
			peer.tokens = peer.tokens - cost
			peer.cursor = index % count + 1
		end
	end
end

local function sendRecord(record)
	local reliable, dynamic = record.reliableRecipients, record.recipients
	local reliableCount, dynamicCount = 0, 0
	for index = 1, #replication.Order do
		local peer = replication.Order[index]
		local entry = peer.entries[record.id]
		if entry and entry.send then
			-- The last sleeping pose must arrive even when the last active packet was lost.
			if entry.reliable then
				reliableCount = reliableCount + 1
				reliable[reliableCount] = peer.player
			else
				dynamicCount = dynamicCount + 1
				dynamic[dynamicCount] = peer.player
			end
			entry.send = false
		end
	end
	for index = #reliable, reliableCount + 1, -1 do reliable[index] = nil end
	for index = #dynamic, dynamicCount + 1, -1 do dynamic[index] = nil end
	sendFrame(record, reliable, true)
	sendFrame(record, dynamic, false)
end

function replication.Think(now)
	local bytes = rate:GetFloat() * 1024
	for index = 1, #replication.Order do
		local peer = replication.Order[index]
		peer.tokens = math.min(bytes * 0.5, peer.tokens + math.max(0, now - peer.time) * bytes)
		peer.time = now
		if peer.ready and now >= peer.nextInterest then refreshInterest(peer, now) end
		schedulePeer(peer, now)
	end
	for index = 1, #server.Order do sendRecord(server.Order[index]) end
end

function replication.Remove(record)
	for index = 1, #replication.Order do
		local peer = replication.Order[index]
		local entry = peer.entries[record.id]
		if entry and entry.known and IsValid(peer.player) then removeMessage(record.id, peer.player) end
		peer.entries[record.id] = nil
	end
end

net.Receive(config.ReadyMessage, function(bits, owner)
	if bits ~= 0 or not IsValid(owner) then return end
	local peer = addPlayer(owner)
	local now = CurTime()
	if now < peer.nextReady then return end
	peer.nextReady, peer.nextInterest, peer.ready = now + 2, 0, true
	for _, entry in pairs(peer.entries) do entry.known, entry.sequence = false, -1 end
end)

hook.Add("PlayerInitialSpawn", "JoltSoftBodies", function(owner)
	-- Returning the peer would stop other hooks and gamemode initialization.
	addPlayer(owner)
end)
hook.Add("PlayerDisconnected", "JoltSoftBodies", function(owner)
	local peer = replication.Peers[owner]
	replication.Peers[owner] = nil
	for index = #replication.Order, 1, -1 do
		if replication.Order[index] == peer then table.remove(replication.Order, index) break end
	end
	if api.Editing then api.Editing.Release(owner) end
	for index = #server.Order, 1, -1 do
		if server.Order[index].owner == owner then server.Order[index].entity:Remove() end
	end
end)
-- Existing players already have the addon during a server-only Lua refresh.
for _, owner in ipairs(player.GetAll()) do addPlayer(owner).ready = true end

concommand.Add("jolt_softbody_server_stats", function(owner)
	if IsValid(owner) and not owner:IsAdmin() then return end
	local message = string.format("Jolt soft bodies: %d, vertices: %d, peers: %d, frames: %d, sent: %.1f KiB\n",
		#server.Order, server.VertexCount, #replication.Order, replication.Frames, replication.Bytes / 1024)
	if IsValid(owner) then owner:PrintMessage(HUD_PRINTCONSOLE, message) else Msg(message) end
end)
return replication
