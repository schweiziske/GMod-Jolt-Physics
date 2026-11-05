local I = JoltI18n or include("jolt_i18n/init.lua")
local api = JoltSoftBodies
local gameplay = include("jolt_gameplay/server.lua")
local attachments = {Pending = {}}
local zero = Vector()

function attachments.Cancel(owner)
	attachments.Pending[owner] = nil
	if IsValid(owner) then
		owner:SetNW2Int("jolt_softbody_mount_id", 0)
		owner:SetNW2Int("jolt_softbody_mount_vertex", 0)
	end
end

function attachments.Detach(record, vertex)
	local mount = record.mounts and record.mounts[vertex]
	if not mount then return false end
	record.mounts[vertex] = nil
	if record.body:IsValid() then
		if mount.native then record.body:DetachVertex(vertex) end
		record.body:SetVertexVelocity(vertex, zero)
		record.body:SetVertexPinned(vertex, mount.wasPinned)
		record.forceSample = true
	end
	return true
end

local function allowed(owner, entity, trace)
	if not IsValid(owner) then return false end
	if entity.CPPICanTool and entity:CPPICanTool(owner, "jolt_softbody") == false then return false end
	return hook.Run("CanTool", owner, trace, "jolt_softbody", nil, 1) ~= false
end

function attachments.Click(owner)
	local pending = attachments.Pending[owner]
	if not pending then
		local record, vertex = api.Editing.Pick(owner, 1)
		if not record then if vertex then gameplay.Fail(owner, vertex) end return end
		if record.dragOwner then return gameplay.Fail(owner, I.Phrase("jolt.release_the_vertex_you_are_dragging_first")) end
		attachments.Pending[owner] = {record = record, vertex = vertex}
		owner:SetNW2Int("jolt_softbody_mount_id", record.id)
		owner:SetNW2Int("jolt_softbody_mount_vertex", vertex)
		return true
	end
	local record, vertex = pending.record, pending.vertex
	if api.Server.Records[record.id] ~= record or record.dragOwner
		or (record.owner ~= owner and not owner:IsAdmin()) then attachments.Cancel(owner) return end
	local origin = owner:GetShootPos()
	local trace = util.TraceLine({start = origin, endpos = origin + owner:GetAimVector() * api.Config.EditDistance,
		filter = {owner, record.entity}, mask = MASK_SOLID})
	if not trace.Hit or trace.HitSky then return end
	local target = trace.Entity
	if not IsValid(target) and not trace.HitWorld then return end
	local physics = not trace.HitWorld and target:GetPhysicsObjectNum(trace.PhysicsBone or 0)
	if not trace.HitWorld and (target:IsPlayer() or target:IsNPC() or target.JoltSoftBodyRecord or not IsValid(physics)) then
		return gameplay.Fail(owner, I.Phrase("jolt.mount_target_invalid"))
	end
	if not allowed(owner, target, trace) or not allowed(owner, record.entity,
		{Entity = record.entity, Hit = true, HitPos = record.center}) then return end
	local point = {}
	if not record.body:ReadVertex(vertex, point) then attachments.Cancel(owner) return end
	if origin:DistToSqr(Vector(point.x, point.y, point.z)) > (api.Config.EditDistance + 100) ^ 2 then
		attachments.Cancel(owner) return
	end
	record.mounts = record.mounts or {}
	local count = 0
	for _ in pairs(record.mounts) do count = count + 1 end
	if count >= 64 and not record.mounts[vertex] then return gameplay.Fail(owner, I.Phrase("jolt.mount_limit")) end
	local wasPinned = record.mounts[vertex] and record.mounts[vertex].wasPinned
	if wasPinned == nil then wasPinned = point.pinned end
	local position = trace.HitPos + trace.HitNormal * math.max(0.5, record.settings.vertexRadius or 0.5)
	if trace.HitWorld then
		local previous = record.mounts[vertex]
		if previous and previous.native then record.body:DetachVertex(vertex) end
		if not record.body:SetVertexPinned(vertex, true) then return end
	else
		if not record.body.AttachVertex then return gameplay.Fail(owner, I.Phrase("jolt.softbody_update_native")) end
		if not record.body:AttachVertex(vertex, physics, position) then
			return gameplay.Fail(owner, I.Phrase("jolt.mount_target_invalid"))
		end
	end
	record.mounts[vertex] = {
		owner = owner, target = target, physics = physics, world = trace.HitWorld,
		native = not trace.HitWorld,
		position = trace.HitWorld and position or physics:WorldToLocal(position),
		wasPinned = wasPinned, point = point, velocity = Vector(), nextPermission = 0, undoToken = {},
	}
	local token, family = record.mounts[vertex].undoToken, record.family
	undo.Create(I.UndoName(I.Token("jolt.mount_points")))
	undo.AddFunction(function()
		-- The token survives splitting, but never matches a replacement mount.
		local records = family and family.records or {[record.id] = record}
		for _, member in pairs(records) do
			if api.Server.Records[member.id] == member then
				for index, mount in pairs(member.mounts or {}) do
					if mount.undoToken == token then attachments.Detach(member, index) end
				end
			end
		end
	end)
	undo.SetPlayer(owner)
	undo.Finish()
	record.forceSample = true
	attachments.Cancel(owner)
	return true
end

function attachments.Unmount(owner)
	if attachments.Pending[owner] then attachments.Cancel(owner) return end
	local record, vertex = api.Editing.Pick(owner, 2)
	if not record then if vertex then gameplay.Fail(owner, vertex) end return end
	if not record.dragOwner then attachments.Detach(record, vertex) end
end

function attachments.ReleaseRecord(record)
	for owner, pending in pairs(attachments.Pending) do
		if pending.record == record then attachments.Cancel(owner) end
	end
	record.mounts = nil
end

function attachments.Copy(parent, child, sources)
	child.mounts = {}
	child.frozenPins = {}
	for index, source in ipairs(sources) do
		if parent.frozenPins then child.frozenPins[index] = parent.frozenPins[source] end
		local mount = parent.mounts and parent.mounts[source]
		if mount then
			local copy = {}
			for key, value in pairs(mount) do copy[key] = value end
			copy.point, copy.velocity = {}, Vector()
			child.mounts[index] = copy
		end
	end
end

local function move(record, vertex, mount, now)
	if not IsValid(mount.owner) or (record.owner ~= mount.owner and not mount.owner:IsAdmin()) then return false end
	if not mount.world and (not IsValid(mount.target) or not IsValid(mount.physics)) then return false end
	local target = mount.world and mount.position or mount.physics:LocalToWorld(mount.position)
	if now >= mount.nextPermission then
		if not allowed(mount.owner, mount.target, {Entity = mount.target, HitWorld = mount.world, Hit = true, HitPos = target})
			or not allowed(mount.owner, record.entity, {Entity = record.entity, Hit = true, HitPos = target}) then return false end
		mount.nextPermission = now + 0.5
	end
	-- Prop joints exchange impulses at each native physics step; Lua only
	-- checks permissions/lifetime. Never overwrite their simulated velocity.
	if mount.native then return record.body:IsVertexAttached(vertex) end
	local point = mount.point
	if not record.body:ReadVertex(vertex, point) then return false end
	local velocity = (target - Vector(point.x, point.y, point.z)) / engine.TickInterval()
	-- Include rigid-body motion at this surface point (including rotation).
	if not mount.world then velocity = velocity + mount.physics:GetVelocityAtPoint(target) end
	local speed, maximum = velocity:Length(), record.settings.maxVelocity * 0.999
	if speed > maximum then velocity = velocity * (maximum / speed) end
	return record.body:SetVertexVelocity(vertex, velocity)
end

function attachments.Think(now)
	for owner in pairs(attachments.Pending) do
		local weapon = IsValid(owner) and owner:GetActiveWeapon()
		if not IsValid(owner) or not owner:Alive() or not IsValid(weapon) or weapon:GetClass() ~= "gmod_tool"
			or weapon:GetMode() ~= "jolt_softbody" or owner:GetInfo("jolt_softbody_mode") ~= "attach" then attachments.Cancel(owner) end
	end
	for _, record in ipairs(api.Server.Order) do
		if record.body:IsValid() then
			for vertex, mount in pairs(record.mounts or {}) do
				if not move(record, vertex, mount, now) then attachments.Detach(record, vertex) end
			end
		end
	end
end

function attachments.Shutdown()
	for owner in pairs(attachments.Pending) do attachments.Cancel(owner) end
end
return attachments
