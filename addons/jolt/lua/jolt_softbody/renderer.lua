local geometry, config = JoltSoftBodies.Geometry, JoltSoftBodies.Config
local renderer = {StaticBuilds = 0, DynamicDraws = 0, LiveMeshes = 0}
local identity = Matrix()
local min, max, abs = math.min, math.max, math.abs
local position, normal, color = mesh.Position, mesh.Normal, mesh.Color
local texCoord, advance = mesh.TexCoord, mesh.AdvanceVertex

-- These DX9 shaders need position, normal, vertex color and one UV channel only.
-- Lighting is baked into vertex colors; no tangent stream, bump map or render target.
-- Vertex alpha makes UnlitGeneric disable opaque depth writes, even at alpha 255.
local material = CreateMaterial("jolt_softbody_surface_v1", "UnlitGeneric", {
	["$basetexture"] = "color/white", ["$vertexcolor"] = "1", ["$nocull"] = "1",
	["$translucent"] = "0",
})
local previewMaterial = CreateMaterial("jolt_softbody_preview_v1", "UnlitGeneric", {
	["$basetexture"] = "color/white", ["$vertexcolor"] = "1", ["$vertexalpha"] = "1",
	["$nocull"] = "1", ["$translucent"] = "1", ["$alpha"] = "0.24", ["$wireframe"] = "1",
})
renderer.Material, renderer.PreviewMaterial = material, previewMaterial

function renderer.New(topology, palette, kind, settings)
	assert(#topology.indices <= 65535, "soft-body mesh exceeds the DX9 vertex limit")
	local data = {
		geometry = topology, palette = config.Palette[palette], kind = kind,
		positions = {}, normals = {}, numericNormals = {}, red = {}, green = {}, blue = {},
		minimum = Vector(), maximum = Vector(), center = Vector(), revision = 0, staticRevision = -1,
		minX = 0, minY = 0, minZ = 0, maxX = 0, maxY = 0, maxZ = 0,
		lightR = 1, lightG = 1, lightB = 1,
	}
	for index = 1, topology.vertexCount do
		data.positions[index], data.normals[index] = Vector(), Vector()
	end
	for index = 1, topology.vertexCount * 3 do data.numericNormals[index] = 0 end
	for index = 1, #topology.indices do data.red[index], data.green[index], data.blue[index] = 0, 0, 0 end
	return data
end

function renderer.SetLight(data, r, g, b)
	data.lightR, data.lightG, data.lightB = min(1, 0.35 + r * 0.85), min(1, 0.35 + g * 0.85), min(1, 0.35 + b * 0.85)
end

-- Bounds/positions also serve picking and engine culling outside the draw range.
-- Keep them cheap: normals and colors are only needed to draw.
function renderer.UpdateBounds(data, positions)
	local minX, minY, minZ, maxX, maxY, maxZ = math.huge, math.huge, math.huge, -math.huge, -math.huge, -math.huge
	for index = 1, data.geometry.vertexCount do
		local offset = (index - 1) * 3
		local x, y, z = positions[offset + 1], positions[offset + 2], positions[offset + 3]
		data.positions[index]:SetUnpacked(x, y, z)
		minX, minY, minZ = min(minX, x), min(minY, y), min(minZ, z)
		maxX, maxY, maxZ = max(maxX, x), max(maxY, y), max(maxZ, z)
	end
	data.minX, data.minY, data.minZ, data.maxX, data.maxY, data.maxZ = minX, minY, minZ, maxX, maxY, maxZ
	data.minimum:SetUnpacked(minX - 4, minY - 4, minZ - 4)
	data.maximum:SetUnpacked(maxX + 4, maxY + 4, maxZ + 4)
	data.center:SetUnpacked((minX + maxX) * 0.5, (minY + maxY) * 0.5, (minZ + maxZ) * 0.5)
end

function renderer.Update(data, positions, boundsReady)
	if not boundsReady then renderer.UpdateBounds(data, positions) end
	local topology, normals = data.geometry, data.numericNormals
	geometry.Normals(positions, topology.indices, normals)
	for index = 1, topology.vertexCount do
		local offset = (index - 1) * 3
		data.normals[index]:SetUnpacked(normals[offset + 1], normals[offset + 2], normals[offset + 3])
	end
	local palette = data.palette
	for corner = 1, #topology.indices do
		local offset = (topology.indices[corner] - 1) * 3
		local diffuse = normals[offset + 1] * -0.35 + normals[offset + 2] * -0.3 + normals[offset + 3] * 0.88
		-- Thin cloth is lit on both sides. The sphere keeps a directional silhouette.
		local shade = (data.kind == 1 and 0.6 + 0.4 * abs(diffuse) or 0.48 + 0.52 * max(0, diffuse)) * topology.shade[corner]
		data.red[corner] = min(255, palette.r * shade * data.lightR)
		data.green[corner] = min(255, palette.g * shade * data.lightG)
		data.blue[corner] = min(255, palette.b * shade * data.lightB)
	end
	data.revision = data.revision + 1
end

local function emit(data)
	local topology = data.geometry
	for corner = 1, #topology.indices do
		local index = topology.indices[corner]
		position(data.positions[index])
		normal(data.normals[index])
		texCoord(0, topology.u[corner], topology.v[corner])
		color(data.red[corner], data.green[corner], data.blue[corner], 255)
		advance()
	end
end

function renderer.Cache(data, surface)
	if data.staticRevision == data.revision then return end
	if not data.static then
		data.static = Mesh(surface or material)
		renderer.LiveMeshes = renderer.LiveMeshes + 1
	end
	-- Topology never changes: the same IMesh can be rewritten on the next sleep.
	mesh.Begin(data.static, MATERIAL_TRIANGLES, data.geometry.triangleCount)
	emit(data)
	mesh.End()
	data.staticRevision = data.revision
	renderer.StaticBuilds = renderer.StaticBuilds + 1
end

function renderer.Draw(data, settled)
	if data.revision == 0 then return end
	render.SetMaterial(material)
	cam.PushModelMatrix(identity)
	if settled then
		renderer.Cache(data)
		data.static:Draw()
	else
		mesh.Begin(MATERIAL_TRIANGLES, data.geometry.triangleCount)
		emit(data)
		mesh.End()
		renderer.DynamicDraws = renderer.DynamicDraws + 1
	end
	cam.PopModelMatrix()
end

function renderer.DrawPreview(data, matrix)
	render.SetMaterial(previewMaterial)
	cam.PushModelMatrix(matrix)
	renderer.Cache(data, previewMaterial)
	data.static:Draw()
	cam.PopModelMatrix()
end

function renderer.DrawWire(data)
	render.SetMaterial(previewMaterial)
	cam.PushModelMatrix(identity)
	mesh.Begin(MATERIAL_TRIANGLES, data.geometry.triangleCount)
	emit(data)
	mesh.End()
	cam.PopModelMatrix()
end

function renderer.Destroy(data)
	if data.static then
		data.static:Destroy()
		data.static = nil
		data.staticRevision = -1
		renderer.LiveMeshes = renderer.LiveMeshes - 1
	end
end

return renderer
