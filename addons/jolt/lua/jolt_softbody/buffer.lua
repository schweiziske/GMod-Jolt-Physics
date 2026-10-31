local config = JoltSoftBodies.Config
local buffer = {}
local methods = {}
methods.__index = methods

local function numbers(count)
	local values = {}
	for index = 1, count do values[index] = 0 end
	return values
end

function buffer.Native(vertexCount)
	local pinned = {}
	for index = 1, vertexCount do pinned[index] = false end
	return {
		positions = numbers(vertexCount * 3), pinned = pinned,
		vertexCount = vertexCount, triangleCount = 0, pinnedCount = 0, simulationTime = 0,
		minX = 0, minY = 0, minZ = 0, maxX = 0, maxY = 0, maxZ = 0, pressure = 0, active = false, updated = false,
	}
end

function buffer.New(vertexCount)
	local self = setmetatable({
		vertexCount = vertexCount, slots = {}, count = 0, head = 0, sequence = -1,
		display = numbers(vertexCount * 3), revision = 0, interval = config.NearInterval,
		leftSequence = -1, rightSequence = -1, alpha = -1, target = -math.huge,
		clockReady = false, clockTime = 0, clockRate = 1, clockDrift = 0,
		playbackTime = 0, playbackDelay = 0.1, lastArrival = 0, jitter = 0,
	}, methods)
	for index = 1, config.SnapshotSlots do
		self.slots[index] = buffer.Native(vertexCount)
		self.slots[index].time = 0
		self.slots[index].sequence = 0
	end
	return self
end

function buffer.IsNewer(sequence, previous)
	local difference = (sequence - previous) % 65536
	return previous == -1 or difference > 0 and difference < 32768
end

function methods:Begin(sequence, time)
	if not buffer.IsNewer(sequence, self.sequence) or not config.IsFinite(time) then return end
	-- Newer packets can share a float timestamp within one server tick.
	if self.count > 0 and time < self.slots[self.head].time then return end
	return self.slots[self.head % config.SnapshotSlots + 1]
end

local function advanceClock(self, now)
	if not self.clockReady then return end
	local elapsed = math.max(0, now - self.clockTime)
	self.clockTime = math.max(now, self.clockTime)
	-- Stop at the available pose without accumulating time ahead of the stream.
	self.playbackTime = math.min(self.slots[self.head].time, self.playbackTime + elapsed * self.clockRate)
end

function methods:Commit(sequence, time, receivedAt)
	if receivedAt then advanceClock(self, receivedAt) end
	local gap = 0
	if self.count > 0 then
		gap = time - self.slots[self.head].time
		if gap > 0.75 then
			self.count, self.target = 0, -math.huge
			self.clockReady, self.clockDrift, self.jitter = false, 0, 0
		elseif gap > 0 then
			self.interval = self.interval * 0.8 + gap * 0.2
			if receivedAt and self.clockReady then
				local variation = math.abs(receivedAt - self.lastArrival - gap)
				self.jitter = self.jitter * 0.9 + variation * 0.1
			end
		end
	end
	self.head = self.head % config.SnapshotSlots + 1
	self.count = math.min(self.count + 1, config.SnapshotSlots)
	self.sequence = sequence
	local slot = self.slots[self.head]
	slot.sequence, slot.time = sequence, time
	if receivedAt then
		-- The two clocks can have unrelated origins. Anchor once at packet arrival;
		-- then adjust playback speed gently instead of jumping at every packet.
		local delay = math.max(0.1, self.interval * 1.4 + self.jitter * 2)
		self.playbackDelay = math.min(0.35, self.interval * (config.SnapshotSlots - 1), delay)
		if not self.clockReady then
			self.playbackTime, self.clockTime = time - self.playbackDelay, receivedAt
			self.clockReady, self.clockRate = true, 1
		elseif gap > 0 then
			local drift = time - self.playbackDelay - self.playbackTime
			self.clockDrift = self.clockDrift * 0.9 + drift * 0.1
			self.clockRate = math.max(0.95, math.min(1.05, 1 + self.clockDrift * 2))
		end
		self.lastArrival = receivedAt
	end
end

function methods:Latest()
	return self.count > 0 and self.slots[self.head] or nil
end

function methods:Play(now)
	if not self.clockReady then return false, false end
	advanceClock(self, now)
	return self:Sample(self.playbackTime)
end

function methods:Sample(time)
	if self.count == 0 then return false, false end
	-- Increasing the interpolation delay must not move the display backwards.
	time = math.max(time, self.target)
	self.target = time
	local oldest = (self.head - self.count) % config.SnapshotSlots + 1
	local left, right = self.slots[oldest], self.slots[oldest]
	for offset = 1, self.count - 1 do
		local candidate = self.slots[(oldest + offset - 1) % config.SnapshotSlots + 1]
		if time < candidate.time then right = candidate break end
		left, right = candidate, candidate
	end
	local duration = right.time - left.time
	local alpha = duration > 0 and math.max(0, math.min(1, (time - left.time) / duration)) or 0
	local latest = self.slots[self.head]
	local settled = not latest.active and time >= latest.time
	if left.sequence == self.leftSequence and right.sequence == self.rightSequence and alpha == self.alpha then
		return false, settled
	end
	local first, second, output = left.positions, right.positions, self.display
	for index = 1, self.vertexCount * 3 do
		output[index] = first[index] + (second[index] - first[index]) * alpha
	end
	self.leftSequence, self.rightSequence, self.alpha = left.sequence, right.sequence, alpha
	self.revision = self.revision + 1
	return true, settled
end

return buffer
