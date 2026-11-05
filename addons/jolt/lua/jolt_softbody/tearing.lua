local I = JoltI18n or include("jolt_i18n/init.lua")
local api = JoltSoftBodies
local gameplay = include("jolt_gameplay/server.lua")
local tearing = {nextCheck = 0}

function tearing.Split(record, seam)
	if api.Server.Records[record.id] ~= record or not record.body:IsValid() then return false end
	if not record.body.CreateFragment then return false, I.Phrase("jolt.softbody_update_native") end
	local pieces = api.Geometry.Split(record.settings, seam)
	if not pieces then return false, I.Phrase("jolt.cut_interior_seam") end
	if not api.Server.CanSplit(record, pieces) then return false, I.Phrase("jolt.tear_budget") end
	api.Editing.ReleaseRecord(record)
	local created = {}
	for part, piece in ipairs(pieces) do
		local body, reason = record.body:CreateFragment(piece.sources, piece.fractions, piece.triangles)
		if body then
			local state = body:GetState()
			if state then
				piece.settings.mass = state.mass
				local child
				child, reason = api.Server.Register(record.owner, piece.settings, piece.mesh, body, state.pos)
				if child then
					api.Attachments.Copy(record, child, piece.sources)
					child.nextTear = CurTime() + 0.5
					created[#created + 1] = child
				end
			else body:Destroy() end
		end
		if #created < part then
			for _, child in ipairs(created) do child.entity:Remove() end
			return false, reason or I.Phrase("jolt.tear_failed")
		end
	end
	-- All allocations and snapshots succeeded. Only now replace the original.
	local family = record.family
	family.pieces = family.pieces + 1
	for _, child in ipairs(created) do api.Server.Track(child, family) end
	if api.Feedback then api.Feedback.Inherit(record, created) end
	record.entity:Remove()
	return true, created
end

function tearing.Cut(owner)
	if CurTime() < (owner.JoltNextSoftCut or 0) then return end
	owner.JoltNextSoftCut = CurTime() + 0.3
	local record, vertex = api.Editing.Pick(owner, 1)
	if not record then if vertex then gameplay.Fail(owner, vertex) end return end
	if record.dragOwner and record.dragOwner ~= owner then return gameplay.Fail(owner, I.Phrase("jolt.another_player_is_dragging_this_soft_body")) end
	local mesh = record.geometry
	if not mesh.columns then return gameplay.Fail(owner, I.Phrase("jolt.cut_interior_seam")) end
	local seam = math.floor((vertex - 1) / mesh.columns) + mesh.rowFrom
	local ok, reason = tearing.Split(record, seam)
	if not ok then gameplay.Fail(owner, reason) end
end

-- Inspect longitudinal edges in the latest native snapshot; require two
-- consecutive overstretched samples and fracture at an adjacent interior row.
function tearing.FindStrain(record)
	local mesh, positions = record.geometry, record.snapshot.positions
	if not mesh.columns or mesh.rowTo - mesh.rowFrom < 2 then return end
	local columns = mesh.columns
	local threshold = (1 + record.settings.tearstrain / 100) ^ 2
	local restSquared = (record.settings.height / api.Config.Rows(record.settings.kind, record.settings.quality)) ^ 2
	local best, bestStrain = nil, threshold
	for row = mesh.rowFrom, mesh.rowTo - 1 do
		for column = 1, columns do
			local a = ((row - mesh.rowFrom) * columns + column - 1) * 3
			local b = a + columns * 3
			local x, y, z = positions[b + 1] - positions[a + 1], positions[b + 2] - positions[a + 2], positions[b + 3] - positions[a + 3]
			local strain = (x * x + y * y + z * z) / restSquared
			if strain > bestStrain then
				best, bestStrain = math.min(row + 1, mesh.rowTo - 1), strain
			end
		end
	end
	return best
end

function tearing.Think(now)
	if now < tearing.nextCheck then return end
	tearing.nextCheck = now + 0.1
	for index = #api.Server.Order, 1, -1 do
		local record = api.Server.Order[index]
		if record.settings.tearing and record.settings.kind ~= 2 and now >= (record.nextTear or 0)
			and record.family and record.family.pieces < api.Config.MaxPieces then
			-- A fresh sample avoids counting the same sleeping/old frame twice.
			if api.Server.Sample(record, now, true) then
				local seam = tearing.FindStrain(record)
				if seam and seam == record.strainedSeam then
					record.nextTear = now + 0.5
					tearing.Split(record, seam)
					record.strainedSeam = nil
				else record.strainedSeam = seam end
			end
		end
	end
end
return tearing
