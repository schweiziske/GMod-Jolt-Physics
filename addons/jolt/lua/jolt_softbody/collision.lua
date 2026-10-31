-- Client prediction uses the displayed mesh. Authoritative server queries go
-- through the existing native Jolt Trace integration, never this Lua sweep.
local collision = {}
local geometry = JoltSoftBodies.Geometry
local tree = include("jolt_softbody/collision_tree.lua")
local abs, min, max, sqrt = math.abs, math.min, math.max, math.sqrt
collision.New = tree.New
local profile = CreateClientConVar("jolt_softbody_collision_profile", "0", false, false,
	"Measure client soft-body collision queries for jolt_softbody_client_stats", 0, 1)
collision.Stats = {calls = 0, seconds = 0, maximum = 0, frame = -1, frameCalls = 0, frameSeconds = 0,
	maximumFrameCalls = 0, maximumFrameSeconds = 0}

local function faceNormal(p, indices, corner)
	local a, b, c = (indices[corner]-1)*3, (indices[corner+1]-1)*3, (indices[corner+2]-1)*3
	local ux, uy, uz = p[b+1]-p[a+1], p[b+2]-p[a+2], p[b+3]-p[a+3]
	local vx, vy, vz = p[c+1]-p[a+1], p[c+2]-p[a+2], p[c+3]-p[a+3]
	return uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx
end

local function contains(p, indices, x, y, z, context)
	if context and not tree.ContainsBounds(context,x,y,z) then return false end
	local candidates = context and tree.Query(context,x,y,z,1,0.371,0.529,0,0,0,math.huge,context.rays)
	local _, _, corner = geometry.Raycast(p, indices, x, y, z, 1, 0.371, 0.529, math.huge, candidates)
	if not corner then return false end
	local nx, ny, nz = faceNormal(p, indices, corner)
	return nx + ny*0.371 + nz*0.529 > 0
end

-- A moving AABB and a triangle overlap over one interval. Test the 13
-- separating axes using scalars; no temporary Vectors per triangle or axis.
local function axis(q, nx, ny, nz, ax, ay, az, ux, uy, uz, vx, vy, vz)
	local length2 = nx*nx + ny*ny + nz*nz
	if length2 < 1e-18 then return true end
	local a = ax*nx + ay*ny + az*nz
	local b, c = a+ux*nx+uy*ny+uz*nz, a+vx*nx+vy*ny+vz*nz
	local radius = q.ex*abs(nx) + q.ey*abs(ny) + q.ez*abs(nz)
	local low, high = min(a,b,c)-radius, max(a,b,c)+radius
	local epsilon = 1e-6*sqrt(length2)
	if low >= -epsilon or high <= epsilon then q.overlap = false end
	local speed = q.dx*nx + q.dy*ny + q.dz*nz
	if abs(speed) < 1e-12*sqrt(length2) then return low <= 0 and high >= 0 end
	local enter, leave = low/speed, high/speed
	if enter > leave then enter, leave = leave, enter end
	if enter > q.enter then
		q.enter = enter
		local sign = speed > 0 and -1 or 1
		q.nx, q.ny, q.nz = nx*sign, ny*sign, nz*sign
	end
	q.leave = min(q.leave, leave)
	return q.enter <= q.leave
end

local function triangle(q, p, indices, corner, start)
	local a, b, c = (indices[corner]-1)*3, (indices[corner+1]-1)*3, (indices[corner+2]-1)*3
	local ax, ay, az = p[a+1]-start.x, p[a+2]-start.y, p[a+3]-start.z
	local ux, uy, uz = p[b+1]-p[a+1], p[b+2]-p[a+2], p[b+3]-p[a+3]
	local vx, vy, vz = p[c+1]-p[a+1], p[c+2]-p[a+2], p[c+3]-p[a+3]
	q.enter, q.leave, q.overlap = -math.huge, math.huge, true
	q.nx, q.ny, q.nz = 0, 0, 0
	if not axis(q,1,0,0,ax,ay,az,ux,uy,uz,vx,vy,vz)
		or not axis(q,0,1,0,ax,ay,az,ux,uy,uz,vx,vy,vz)
		or not axis(q,0,0,1,ax,ay,az,ux,uy,uz,vx,vy,vz)
		or not axis(q,uy*vz-uz*vy,uz*vx-ux*vz,ux*vy-uy*vx,ax,ay,az,ux,uy,uz,vx,vy,vz) then return false end
	local wx, wy, wz = vx-ux, vy-uy, vz-uz
	return axis(q,0,uz,-uy,ax,ay,az,ux,uy,uz,vx,vy,vz)
		and axis(q,-uz,0,ux,ax,ay,az,ux,uy,uz,vx,vy,vz)
		and axis(q,uy,-ux,0,ax,ay,az,ux,uy,uz,vx,vy,vz)
		and axis(q,0,vz,-vy,ax,ay,az,ux,uy,uz,vx,vy,vz)
		and axis(q,-vz,0,vx,ax,ay,az,ux,uy,uz,vx,vy,vz)
		and axis(q,vy,-vx,0,ax,ay,az,ux,uy,uz,vx,vy,vz)
		and axis(q,0,wz,-wy,ax,ay,az,ux,uy,uz,vx,vy,vz)
		and axis(q,-wz,0,wx,ax,ay,az,ux,uy,uz,vx,vy,vz)
		and axis(q,wy,-wx,0,ax,ay,az,ux,uy,uz,vx,vy,vz)
		and q.enter < 1 and q.leave >= 0
end

local function earlier(a, b) return a[1] < b[1] end

local function covered(p, indices, intervals, start, delta, closed, context)
	table.sort(intervals, earlier)
	local reach = 0
	for index = 1, #intervals+1 do
		local interval = intervals[index]
		local nextStart = interval and interval[1] or 1
		if nextStart > reach+1e-7 then
			local middle = (reach+nextStart)*0.5
			if not closed or not contains(p, indices, start.x+delta.x*middle, start.y+delta.y*middle, start.z+delta.z*middle, context) then return false end
		end
		reach = max(reach, interval and interval[2] or 1)
		if reach >= 1 then return true end
	end
	return true
end

local function stationaryHit(start)
	return {Hit=true,HitPos=Vector(start.x,start.y,start.z),Normal=Vector(),Fraction=0,
		StartSolid=true,AllSolid=true,FractionLeftSolid=0}
end

local function trace(p, indices, start, delta, extents, closed, context, revision)
	if context then tree.Refit(context,p,indices,revision) end
	local fraction, nx, ny, nz, left = 1, 0, 0, 0, 0
	local starts = closed and contains(p, indices, start.x, start.y, start.z, context) or false
	local all = false
	local stationary = context and delta.x == 0 and delta.y == 0 and delta.z == 0
	if stationary and starts then return stationaryHit(start) end
	local box = extents and (extents.x > 0 or extents.y > 0 or extents.z > 0)
	if not box then
		local candidates = context and tree.Query(context,start.x,start.y,start.z,delta.x,delta.y,delta.z,0,0,0,1,context.rays)
		local _, distance, corner = geometry.Raycast(p, indices, start.x, start.y, start.z, delta.x, delta.y, delta.z, 1, candidates)
		if corner then fraction = distance nx, ny, nz = faceNormal(p, indices, corner) end
		if starts then left, all, fraction = corner and fraction or 0, not corner, 0 end
	else
		local q = context and context.query or {}
		q.dx, q.dy, q.dz, q.ex, q.ey, q.ez = delta.x, delta.y, delta.z, extents.x, extents.y, extents.z
		local intervals, used = context and context.intervals or {}, 0
		local candidates = context and tree.Query(context,start.x,start.y,start.z,q.dx,q.dy,q.dz,q.ex,q.ey,q.ez,1,context.candidates)
		for at = 1, candidates and #candidates or #indices / 3 do
			local corner = candidates and candidates[at] or (at-1)*3+1
			if triangle(q, p, indices, corner, start) then
				-- A strict overlap covers the whole [0,1] interval when the hull
				-- does not move. Ground/stuck probes need no remaining triangles.
				if stationary and q.overlap then return stationaryHit(start) end
				used = used+1
				local interval = context and context.intervalPool[used] or {}
				if context then context.intervalPool[used] = interval end
				interval[1], interval[2] = max(0,q.enter), min(1,q.leave)
				intervals[used] = interval
				starts = starts or q.overlap
				if q.enter < fraction then fraction, nx, ny, nz = max(0,q.enter), q.nx, q.ny, q.nz end
			end
		end
		for at = #intervals, used+1, -1 do intervals[at] = nil end
		if starts then fraction, all = 0, covered(p, indices, intervals, start, delta, closed, context) end
	end
	if fraction >= 1 and not starts then return false end
	local length = sqrt(nx*nx+ny*ny+nz*nz)
	if length > 1e-9 then nx, ny, nz = nx/length, ny/length, nz/length end
	local approach = nx*delta.x+ny*delta.y+nz*delta.z
	if approach > 0 then nx, ny, nz, approach = -nx, -ny, -nz, -approach end
	if box and not starts and approach < -1e-9 then fraction = max(0,fraction+0.03125/approach) end
	return {Hit=true,HitPos=start+delta*fraction,Normal=Vector(nx,ny,nz),Fraction=fraction,
		StartSolid=starts,AllSolid=all,FractionLeftSolid=left}
end

function collision.Trace(p, indices, start, delta, extents, closed, context, revision)
	if not profile:GetBool() then return trace(p,indices,start,delta,extents,closed,context,revision) end
	local before = SysTime()
	local hit = trace(p,indices,start,delta,extents,closed,context,revision)
	local elapsed, frame, stats = SysTime()-before, FrameNumber(), collision.Stats
	stats.calls, stats.seconds, stats.maximum = stats.calls+1, stats.seconds+elapsed, max(stats.maximum,elapsed)
	if stats.frame ~= frame then stats.frame, stats.frameCalls, stats.frameSeconds = frame, 0, 0 end
	stats.frameCalls, stats.frameSeconds = stats.frameCalls+1, stats.frameSeconds+elapsed
	stats.maximumFrameCalls = max(stats.maximumFrameCalls,stats.frameCalls)
	stats.maximumFrameSeconds = max(stats.maximumFrameSeconds,stats.frameSeconds)
	return hit
end

return collision
