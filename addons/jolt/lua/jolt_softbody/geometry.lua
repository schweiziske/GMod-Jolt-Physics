local config = JoltSoftBodies.Config
local geometry = {}
local abs, sqrt, min, max = math.abs, math.sqrt, math.min, math.max
local pi, atan2, asin = math.pi, math.atan2, math.asin

local function appendVertex(mesh, x, y, z)
	local index = mesh.vertexCount + 1
	local offset = (index - 1) * 3
	mesh.positions[offset + 1] = x
	mesh.positions[offset + 2] = y
	mesh.positions[offset + 3] = z
	mesh.pinned[index] = false
	mesh.vertexCount = index
	return index
end

local function appendTriangle(mesh, first, second, third, shade)
	local offset = #mesh.indices
	mesh.indices[offset + 1] = first
	mesh.indices[offset + 2] = second
	mesh.indices[offset + 3] = third
	mesh.shade[offset + 1] = shade or 1
	mesh.shade[offset + 2] = shade or 1
	mesh.shade[offset + 3] = shade or 1
end

local function cloth(mesh, settings)
	local resolution = config.ClothResolution[settings.quality]
	local last = resolution - 1
	for row = 0, last do
		for column = 0, last do
			local index = appendVertex(mesh,
				(column / last - 0.5) * settings.width, 0, (0.5 - row / last) * settings.height)
			mesh.pinned[index] = row == 0 and (settings.pins == 2
				or settings.pins == 1 and (column == 0 or column == last))
		end
	end
	for row = 0, last - 1 do
		for column = 0, last - 1 do
			local first = row * resolution + column + 1
			local below = first + resolution
			local shade = (math.floor(row / 2) + math.floor(column / 2)) % 2 == 0 and 1 or 0.88
			appendTriangle(mesh, first, first + 1, below, shade)
			appendTriangle(mesh, first + 1, below + 1, below, shade)
		end
	end
	for corner = 1, #mesh.indices do
		local index = mesh.indices[corner] - 1
		mesh.u[corner] = (index % resolution) / last
		mesh.v[corner] = math.floor(index / resolution) / last
	end
end

local function sphereVertex(mesh, x, y, z, radius)
	local scale = radius / sqrt(x * x + y * y + z * z)
	return appendVertex(mesh, x * scale, y * scale, z * scale)
end

local function sphere(mesh, settings)
	local radius, golden = settings.width * 0.5, (1 + sqrt(5)) * 0.5
	local seeds = {
		-1, golden, 0, 1, golden, 0, -1, -golden, 0, 1, -golden, 0,
		0, -1, golden, 0, 1, golden, 0, -1, -golden, 0, 1, -golden,
		golden, 0, -1, golden, 0, 1, -golden, 0, -1, -golden, 0, 1,
	}
	for offset = 1, #seeds, 3 do
		sphereVertex(mesh, seeds[offset], seeds[offset + 1], seeds[offset + 2], radius)
	end
	mesh.indices = {
		1, 12, 6, 1, 6, 2, 1, 2, 8, 1, 8, 11, 1, 11, 12,
		2, 6, 10, 6, 12, 5, 12, 11, 3, 11, 8, 7, 8, 2, 9,
		4, 10, 5, 4, 5, 3, 4, 3, 7, 4, 7, 9, 4, 9, 10,
		5, 10, 6, 3, 5, 12, 7, 3, 11, 9, 7, 8, 10, 9, 2,
	}
	for subdivision = 1, config.SphereSubdivisions[settings.quality] do
		local previous, midpoint = mesh.indices, {}
		mesh.indices, mesh.shade = {}, {}
		local function split(first, second)
			local key = min(first, second) * 2048 + max(first, second)
			local index = midpoint[key]
			if index then return index end
			local a, b, positions = (first - 1) * 3, (second - 1) * 3, mesh.positions
			index = sphereVertex(mesh, positions[a + 1] + positions[b + 1],
				positions[a + 2] + positions[b + 2], positions[a + 3] + positions[b + 3], radius)
			midpoint[key] = index
			return index
		end
		for corner = 1, #previous, 3 do
			local a, b, c = previous[corner], previous[corner + 1], previous[corner + 2]
			local ab, bc, ca = split(a, b), split(b, c), split(c, a)
			appendTriangle(mesh, a, ab, ca)
			appendTriangle(mesh, b, bc, ab)
			appendTriangle(mesh, c, ca, bc)
			appendTriangle(mesh, ab, bc, ca)
		end
	end
	for corner = 1, #mesh.indices, 3 do
		local low, high = 1, 0
		for offset = corner, corner + 2 do
			local index = (mesh.indices[offset] - 1) * 3
			local u = atan2(mesh.positions[index + 2], mesh.positions[index + 1]) / (2 * pi) + 0.5
			mesh.u[offset] = u
			mesh.v[offset] = 0.5 - asin(max(-1, min(1, mesh.positions[index + 3] / radius))) / pi
			low, high = min(low, u), max(high, u)
		end
		if high - low > 0.5 then
			for offset = corner, corner + 2 do
				if mesh.u[offset] < 0.5 then mesh.u[offset] = mesh.u[offset] + 1 end
			end
		end
	end
end

function geometry.Generate(settings)
	local mesh = {positions = {}, indices = {}, pinned = {}, u = {}, v = {}, shade = {}, vertexCount = 0}
	if settings.kind == 2 then sphere(mesh, settings) else cloth(mesh, settings) end
	mesh.triangleCount = #mesh.indices / 3
	return mesh
end

function geometry.Transform(positions, output, x, y, z, yaw)
	local cosine, sine = math.cos(yaw * pi / 180), math.sin(yaw * pi / 180)
	for index = 1, #positions, 3 do
		local localX, localY = positions[index], positions[index + 1]
		output[index] = x + localX * cosine - localY * sine
		output[index + 1] = y + localX * sine + localY * cosine
		output[index + 2] = z + positions[index + 2]
	end
end

function geometry.Placement(settings, x, y, z, nx, ny, nz, eyeYaw)
	local yaw = eyeYaw + 90 + settings.rotation
	local clearance
	if settings.kind == 2 then
		clearance = settings.width * 0.5 + 8
	else
		local angle = yaw * pi / 180
		clearance = abs(nx * math.cos(angle) + ny * math.sin(angle)) * settings.width * 0.5
			+ abs(nz) * settings.height * 0.5 + 8
	end
	return x + nx * clearance, y + ny * clearance, z + nz * clearance, yaw
end

-- Flat numeric arrays keep these loops traceable and avoid Vector arithmetic.
function geometry.Normals(positions, indices, output)
	for index = 1, #positions do output[index] = 0 end
	for corner = 1, #indices, 3 do
		local a, b, c = (indices[corner] - 1) * 3, (indices[corner + 1] - 1) * 3, (indices[corner + 2] - 1) * 3
		local ux, uy, uz = positions[b + 1] - positions[a + 1], positions[b + 2] - positions[a + 2],
			positions[b + 3] - positions[a + 3]
		local vx, vy, vz = positions[c + 1] - positions[a + 1], positions[c + 2] - positions[a + 2],
			positions[c + 3] - positions[a + 3]
		local nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
		output[a + 1], output[a + 2], output[a + 3] = output[a + 1] + nx, output[a + 2] + ny, output[a + 3] + nz
		output[b + 1], output[b + 2], output[b + 3] = output[b + 1] + nx, output[b + 2] + ny, output[b + 3] + nz
		output[c + 1], output[c + 2], output[c + 3] = output[c + 1] + nx, output[c + 2] + ny, output[c + 3] + nz
	end
	for index = 1, #positions, 3 do
		local x, y, z = output[index], output[index + 1], output[index + 2]
		local length = sqrt(x * x + y * y + z * z)
		if length > 1e-12 then
			output[index], output[index + 1], output[index + 2] = x / length, y / length, z / length
		else
			output[index], output[index + 1], output[index + 2] = 0, 0, 1
		end
	end
end

local function rayTriangle(p, a, b, c, ox, oy, oz, dx, dy, dz, limit)
	local ax, ay, az = p[a + 1], p[a + 2], p[a + 3]
	local ux, uy, uz = p[b + 1] - ax, p[b + 2] - ay, p[b + 3] - az
	local vx, vy, vz = p[c + 1] - ax, p[c + 2] - ay, p[c + 3] - az
	local px, py, pz = dy * vz - dz * vy, dz * vx - dx * vz, dx * vy - dy * vx
	local determinant = ux * px + uy * py + uz * pz
	if abs(determinant) < 1e-9 then return end
	local tx, ty, tz = ox - ax, oy - ay, oz - az
	local inverse = 1 / determinant
	local u = (tx * px + ty * py + tz * pz) * inverse
	if u < 0 or u > 1 then return end
	local qx, qy, qz = ty * uz - tz * uy, tz * ux - tx * uz, tx * uy - ty * ux
	local v = (dx * qx + dy * qy + dz * qz) * inverse
	if v < 0 or u + v > 1 then return end
	local distance = (vx * qx + vy * qy + vz * qz) * inverse
	if distance >= 0 and distance < limit then return distance end
end

function geometry.Raycast(positions, indices, ox, oy, oz, dx, dy, dz, limit, candidates)
	local hitCorner, nearest
	for at = 1, candidates and #candidates or #indices / 3 do
		local corner = candidates and candidates[at] or (at-1)*3+1
		local a, b, c = (indices[corner] - 1) * 3, (indices[corner + 1] - 1) * 3, (indices[corner + 2] - 1) * 3
		local distance = rayTriangle(positions, a, b, c, ox, oy, oz, dx, dy, dz, limit)
		if distance then limit, hitCorner = distance, corner end
	end
	if not hitCorner then return end
	local x, y, z = ox + dx * limit, oy + dy * limit, oz + dz * limit
	local best = math.huge
	for corner = hitCorner, hitCorner + 2 do
		local index = indices[corner]
		local offset = (index - 1) * 3
		local vx, vy, vz = positions[offset + 1] - x, positions[offset + 2] - y, positions[offset + 3] - z
		local distance = vx * vx + vy * vy + vz * vz
		if distance < best then best, nearest = distance, index end
	end
	return nearest, limit, hitCorner
end

local function slab(origin, direction, low, high, entry, exit)
	if abs(direction) < 1e-12 then
		if origin < low or origin > high then return end
		return entry, exit
	end
	local first, second = (low - origin) / direction, (high - origin) / direction
	entry, exit = max(entry, min(first, second)), min(exit, max(first, second))
	if entry <= exit then return entry, exit end
end

function geometry.RayBounds(bounds, ox, oy, oz, dx, dy, dz, limit)
	local entry, exit = slab(ox, dx, bounds.minX, bounds.maxX, 0, limit)
	if not entry then return false end
	entry, exit = slab(oy, dy, bounds.minY, bounds.maxY, entry, exit)
	if not entry then return false end
	return slab(oz, dz, bounds.minZ, bounds.maxZ, entry, exit) ~= nil
end

return geometry
