local config = JoltSoftBodies.Config
local protocol = {}
local floor, max, abs = math.floor, math.max, math.abs
local writeUInt, writeInt, writeFloat, writeBool = net.WriteUInt, net.WriteInt, net.WriteFloat, net.WriteBool
local readUInt, readInt, readFloat, readBool = net.ReadUInt, net.ReadInt, net.ReadFloat, net.ReadBool

function protocol.Frame(vertexCount)
	local frame = {positions = {}, pinned = {}, vertexCount = vertexCount, x = 0, y = 0, z = 0, scale = 1}
	for index = 1, vertexCount * 3 do frame.positions[index] = 0 end
	for index = 1, vertexCount do frame.pinned[index] = false end
	return frame
end

function protocol.Prepare(snapshot, frame)
	local x, y, z = (snapshot.minX + snapshot.maxX) * 0.5,
		(snapshot.minY + snapshot.maxY) * 0.5, (snapshot.minZ + snapshot.maxZ) * 0.5
	local extent = max(snapshot.maxX - snapshot.minX, snapshot.maxY - snapshot.minY,
		snapshot.maxZ - snapshot.minZ) * 0.5
	local magnitude = max(abs(x), abs(y), abs(z))
	if not config.IsFinite(extent) or not config.IsFinite(magnitude)
		or extent < 0 or magnitude > config.CoordinateLimit then return false end
	local scale = max(config.Quantization, extent / 32760, magnitude / 8388608)
	local changed = frame.x ~= x or frame.y ~= y or frame.z ~= z or frame.scale ~= scale
		or frame.active ~= snapshot.active
	local source, output = snapshot.positions, frame.positions
	for index = 1, frame.vertexCount * 3, 3 do
		local px, py, pz = source[index], source[index + 1], source[index + 2]
		if not config.IsFinite(px) or not config.IsFinite(py) or not config.IsFinite(pz) then return false end
		local qx, qy, qz = floor((px - x) / scale + 0.5), floor((py - y) / scale + 0.5),
			floor((pz - z) / scale + 0.5)
		if max(abs(qx), abs(qy), abs(qz)) > 32767 then return false end
		changed = changed or output[index] ~= qx or output[index + 1] ~= qy or output[index + 2] ~= qz
		output[index], output[index + 1], output[index + 2] = qx, qy, qz
	end
	for index = 1, frame.vertexCount do
		changed = changed or frame.pinned[index] ~= snapshot.pinned[index]
		frame.pinned[index] = snapshot.pinned[index]
	end
	frame.x, frame.y, frame.z, frame.scale = x, y, z, scale
	frame.active = snapshot.active
	return true, changed
end

function protocol.WriteFrame(id, sequence, time, frame)
	writeUInt(id, 32)
	writeUInt(sequence, 16)
	writeFloat(time)
	writeFloat(frame.x)
	writeFloat(frame.y)
	writeFloat(frame.z)
	writeFloat(frame.scale)
	writeUInt(frame.vertexCount, 16)
	writeBool(frame.active)
	local positions, pinned = frame.positions, frame.pinned
	for vertex = 1, frame.vertexCount do
		local offset = (vertex - 1) * 3
		writeInt(positions[offset + 1], 16)
		writeInt(positions[offset + 2], 16)
		writeInt(positions[offset + 3], 16)
		writeBool(pinned[vertex])
	end
end

function protocol.ReadFrame(records, bits, receivedAt, playbackAt)
	if bits < 225 then return end
	local id, sequence, time = readUInt(32), readUInt(16), readFloat()
	local x, y, z, scale = readFloat(), readFloat(), readFloat(), readFloat()
	local count, active = readUInt(16), readBool()
	local record = records[id]
	if not record or count ~= record.geometry.vertexCount then return end
	local expected = 225 + count * 49
	if bits < expected or bits >= expected + 8 or not config.IsFinite(time)
		or not config.IsFinite(x) or not config.IsFinite(y) or not config.IsFinite(z)
		or not config.IsFinite(scale) or scale < config.Quantization or scale > 128
		or max(abs(x), abs(y), abs(z)) > config.CoordinateLimit then return end
	local slot = record.buffer:Begin(sequence, time)
	if not slot then return end
	local positions, pinned = slot.positions, slot.pinned
	local minX, minY, minZ, maxX, maxY, maxZ = math.huge, math.huge, math.huge, -math.huge, -math.huge, -math.huge
	for vertex = 1, count do
		local offset = (vertex - 1) * 3
		local px, py, pz = x + readInt(16) * scale, y + readInt(16) * scale, z + readInt(16) * scale
		positions[offset + 1], positions[offset + 2], positions[offset + 3] = px, py, pz
		pinned[vertex] = readBool()
		minX, minY, minZ = math.min(minX, px), math.min(minY, py), math.min(minZ, pz)
		maxX, maxY, maxZ = max(maxX, px), max(maxY, py), max(maxZ, pz)
	end
	slot.minX, slot.minY, slot.minZ, slot.maxX, slot.maxY, slot.maxZ = minX, minY, minZ, maxX, maxY, maxZ
	slot.active = active
	record.buffer:Commit(sequence, time, playbackAt or receivedAt)
	record.receivedAt = receivedAt
	return record
end

function protocol.WriteMeta(record)
	local settings = record.settings
	writeUInt(config.Protocol, 8)
	writeUInt(record.id, 32)
	writeUInt(record.entity:EntIndex(), 16)
	writeUInt(settings.kind, 2)
	writeUInt(settings.quality, 2)
	writeFloat(settings.width)
	writeFloat(settings.height)
	writeUInt(settings.palette, 3)
	writeUInt(settings.pins, 2)
	writeBool(settings.playerCollision == true)
end

function protocol.ReadMeta(bits)
	if bits < 130 or bits >= 138 or readUInt(8) ~= config.Protocol then return end
	local id, entityIndex, kind, quality = readUInt(32), readUInt(16), readUInt(2), readUInt(2)
	local width, height, palette, pins, playerCollision = readFloat(), readFloat(), readUInt(3), readUInt(2), readBool()
	if id == 0 or id > 2147483647 or entityIndex == 0 or kind < 1 or kind > 2 or quality < 1 or quality > 3
		or not config.IsFinite(width) or width < 32 or width > 512
		or not config.IsFinite(height) or height < 32 or height > 512
		or palette < 1 or palette > #config.Palette or pins > 2 then return end
	return id, entityIndex, {kind = kind, quality = quality, width = width, height = height, palette = palette, pins = pins,
		playerCollision = playerCollision}
end

function protocol.MetaBytes(record)
	return 18
end

return protocol
