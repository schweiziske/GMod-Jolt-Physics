local I = JoltI18n or include("jolt_i18n/init.lua")
local api = JoltSoftBodies
local config, geometry, server = api.Config, api.Geometry, api.Server
local gameplay = include("jolt_gameplay/server.lua")
local editing = {Drags = {}}
local zero = Vector()

local function activeTool(owner)
	if not IsValid(owner) or not owner:Alive() then return end
	local weapon = owner:GetActiveWeapon()
	if not IsValid(weapon) or weapon:GetClass() ~= "gmod_tool" or weapon:GetMode() ~= "jolt_softbody"
		or owner:GetInfo("jolt_softbody_mode") ~= "edit" then return end
	local tool = weapon:GetToolObject("jolt_softbody")
	return tool and tool:Allowed() and tool or nil
end

local function canEdit(owner, record, trace, button)
	local tool = activeTool(owner)
	if not tool or not IsValid(record.entity) then return false end
	if record.owner ~= owner and not owner:IsAdmin() then return false end
	if record.entity.CPPICanTool and not record.entity:CPPICanTool(owner, "jolt_softbody") then return false end
	return hook.Run("CanTool", owner, trace, "jolt_softbody", tool, button) ~= false
end

function editing.Pick(owner, button)
	local origin, direction = owner:GetShootPos(), owner:GetAimVector()
	local trace = util.TraceLine({start = origin, endpos = origin + direction * config.EditDistance, filter = owner, mask = MASK_SOLID})
	local selected = IsValid(trace.Entity) and trace.Entity.JoltSoftBodyRecord
	if not selected then return end
	local hit = selected.body:Trace(origin, direction * config.EditDistance)
	if not hit or not hit.Hit or hit.Vertex < 1 then return end
	if not canEdit(owner, selected, trace, button) then return nil, I.Phrase("jolt.you_do_not_have_permission_to_edit_this_soft_body") end
	return selected, hit.Vertex, hit.Fraction * config.EditDistance, trace
end

function editing.Release(owner)
	local drag = editing.Drags[owner]
	if not drag then return end
	editing.Drags[owner] = nil
	local record = drag.record
	record.dragOwner = nil
	if record.body:IsValid() then
		record.body:SetVertexVelocity(drag.vertex, zero)
		record.body:SetVertexPinned(drag.vertex, drag.wasPinned)
		record.forceSample = true
	end
	if IsValid(owner) then
		owner:SetNW2Int("jolt_softbody_drag_id", 0)
		owner:SetNW2Int("jolt_softbody_drag_vertex", 0)
	end
end

function editing.ReleaseRecord(record)
	if record.dragOwner then editing.Release(record.dragOwner) end
end

function editing.Begin(owner)
	editing.Release(owner)
	local record, vertex, depth, hit = editing.Pick(owner, 1)
	if not record then if vertex then gameplay.Fail(owner, vertex) end return end
	if record.dragOwner then return gameplay.Fail(owner, I.Phrase("jolt.another_player_is_dragging_this_soft_body")) end
	local point = {}
	if not record.body:ReadVertex(vertex, point) or not record.body:SetVertexPinned(vertex, true) then return end
	local drag = {
		record = record, vertex = vertex, depth = depth, wasPinned = point.pinned, point = point,
		offsetX = point.x - hit.HitPos.x, offsetY = point.y - hit.HitPos.y, offsetZ = point.z - hit.HitPos.z,
		velocity = Vector(), permission = hit, nextPermission = 0,
		trace = {start = Vector(), endpos = Vector(), filter = {owner, record.entity}, mask = MASK_SOLID, output = {}},
	}
	editing.Drags[owner], record.dragOwner = drag, owner
	record.forceSample = true
	owner:SetNW2Int("jolt_softbody_drag_id", record.id)
	owner:SetNW2Int("jolt_softbody_drag_vertex", vertex)
end

function editing.TogglePin(owner)
	local record, vertex = editing.Pick(owner, 2)
	if not record then if vertex then gameplay.Fail(owner, vertex) end return end
	if record.dragOwner then return gameplay.Fail(owner, I.Phrase("jolt.release_the_vertex_you_are_dragging_first")) end
	local point = {}
	if record.body:ReadVertex(vertex, point) and record.body:SetVertexPinned(vertex, not point.pinned) then
		record.forceSample = true
	end
end

function editing.Input(tool)
	local owner = tool:GetOwner()
	local left, right = owner:KeyDown(IN_ATTACK), owner:KeyDown(IN_ATTACK2)
	if activeTool(owner) then
		if left and not tool.JoltLeftDown then editing.Begin(owner) end
		if right and not tool.JoltRightDown then editing.TogglePin(owner) end
	end
	tool.JoltLeftDown, tool.JoltRightDown = left, right
end

local function moveVertex(owner, drag, now)
	local record, point = drag.record, drag.point
	if not activeTool(owner) or not owner:KeyDown(IN_ATTACK) or not record.body:ReadVertex(drag.vertex, point) then return false end
	if now >= drag.nextPermission then
		drag.permission.HitPos:SetUnpacked(point.x, point.y, point.z)
		if not canEdit(owner, record, drag.permission, 1) then return false end
		drag.nextPermission = now + 0.25
	end
	local origin, direction = owner:GetShootPos(), owner:GetAimVector()
	local dx, dy, dz = point.x - origin.x, point.y - origin.y, point.z - origin.z
	if dx * dx + dy * dy + dz * dz > (config.EditDistance + 100) ^ 2 then return false end
	local trace = drag.trace
	trace.start:SetUnpacked(origin.x, origin.y, origin.z)
	trace.endpos:SetUnpacked(origin.x + direction.x * drag.depth + drag.offsetX,
		origin.y + direction.y * drag.depth + drag.offsetY, origin.z + direction.z * drag.depth + drag.offsetZ)
	util.TraceLine(trace)
	local target = trace.output.Hit and trace.output.HitPos or trace.endpos
	local x, y, z = target.x, target.y, target.z
	if trace.output.Hit then
		local normal = trace.output.HitNormal
		x, y, z = x + normal.x * 3, y + normal.y * 3, z + normal.z * 3
	end
	local vx, vy, vz = (x - point.x) * 18, (y - point.y) * 18, (z - point.z) * 18
	local speed = math.sqrt(vx * vx + vy * vy + vz * vz)
	local scale = speed > 900 and 900 / speed or 1
	drag.velocity:SetUnpacked(vx * scale, vy * scale, vz * scale)
	return record.body:SetVertexVelocity(drag.vertex, drag.velocity)
end

function editing.Think(now)
	for owner, drag in pairs(editing.Drags) do
		if not moveVertex(owner, drag, now) then editing.Release(owner) end
	end
end

return editing
