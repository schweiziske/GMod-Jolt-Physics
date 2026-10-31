-- Topology is fixed. Build the partition once and refit it only when a queried
-- display pose changes; prediction can then reuse it for every movement trace.
local tree = {}
local min, max = math.min, math.max
local leafSize = 8

function tree.New(topology)
	local data = {nodes = {}, order = {}, revision = -1, candidates = {}, rays = {},
		intervals = {}, intervalPool = {}, query = {}}
	local centers, p, indices = {}, topology.positions, topology.indices
	for corner = 1, #indices, 3 do
		local a, b, c = (indices[corner]-1)*3, (indices[corner+1]-1)*3, (indices[corner+2]-1)*3
		centers[corner] = {(p[a+1]+p[b+1]+p[c+1])/3, (p[a+2]+p[b+2]+p[c+2])/3,
			(p[a+3]+p[b+3]+p[c+3])/3}
		data.order[#data.order+1] = corner
	end
	local nodes, order = data.nodes, data.order
	local function build(first, last)
		local index = #nodes+1
		local node = {first = first, last = last}
		nodes[index] = node
		if last-first+1 > leafSize then
			local lo, hi = {math.huge,math.huge,math.huge}, {-math.huge,-math.huge,-math.huge}
			for at = first, last do
				local center = centers[order[at]]
				for axis = 1, 3 do lo[axis], hi[axis] = min(lo[axis],center[axis]), max(hi[axis],center[axis]) end
			end
			local axis = 1
			for candidate = 2, 3 do if hi[candidate]-lo[candidate] > hi[axis]-lo[axis] then axis = candidate end end
			local pivot, split = (lo[axis]+hi[axis])*0.5, first
			for at = first, last do
				if centers[order[at]][axis] < pivot then
					order[at], order[split] = order[split], order[at]
					split = split+1
				end
			end
			-- Degenerate centroid ranges still produce a bounded tree.
			if split == first or split == last+1 then split = math.floor((first+last)/2)+1 end
			node.left, node.right = build(first,split-1), build(split,last)
		end
		node.escape = #nodes+1
		return index
	end
	if #order > 0 then build(1,#order) end
	return data
end

function tree.Refit(data, p, indices, revision)
	if revision ~= nil and data.revision == revision and data.positions == p then return end
	local nodes, order = data.nodes, data.order
	for index = #nodes, 1, -1 do
		local node = nodes[index]
		local lx, ly, lz, hx, hy, hz
		if node.left then
			local a, b = nodes[node.left], nodes[node.right]
			lx, ly, lz = min(a[1],b[1]), min(a[2],b[2]), min(a[3],b[3])
			hx, hy, hz = max(a[4],b[4]), max(a[5],b[5]), max(a[6],b[6])
		else
			lx, ly, lz, hx, hy, hz = math.huge, math.huge, math.huge, -math.huge, -math.huge, -math.huge
			for at = node.first, node.last do
				local corner = order[at]
				for offset = corner, corner+2 do
					local vertex = (indices[offset]-1)*3
					local x, y, z = p[vertex+1], p[vertex+2], p[vertex+3]
					lx, ly, lz, hx, hy, hz = min(lx,x), min(ly,y), min(lz,z), max(hx,x), max(hy,y), max(hz,z)
				end
			end
		end
		node[1], node[2], node[3], node[4], node[5], node[6] = lx, ly, lz, hx, hy, hz
	end
	data.revision, data.positions = revision, p
end

function tree.ContainsBounds(data, x, y, z)
	local root = data.nodes[1]
	return root and x >= root[1]-1e-6 and y >= root[2]-1e-6 and z >= root[3]-1e-6
		and x <= root[4]+1e-6 and y <= root[5]+1e-6 and z <= root[6]+1e-6
end

local function slab(origin, delta, low, high, enter, leave)
	if delta == 0 then
		if origin < low or origin > high then return end
		return enter, leave
	end
	local a, b = (low-origin)/delta, (high-origin)/delta
	enter, leave = max(enter,min(a,b)), min(leave,max(a,b))
	if enter <= leave then return enter, leave end
end

function tree.Query(data, x, y, z, dx, dy, dz, ex, ey, ez, limit, output)
	local nodes, order, count, index = data.nodes, data.order, 0, 1
	-- Padding is conservative; only the original triangle test decides a hit.
	ex, ey, ez = ex+1e-6, ey+1e-6, ez+1e-6
	while index <= #nodes do
		local node = nodes[index]
		local enter, leave = slab(x,dx,node[1]-ex,node[4]+ex,0,limit)
		if enter then enter, leave = slab(y,dy,node[2]-ey,node[5]+ey,enter,leave) end
		if enter then enter, leave = slab(z,dz,node[3]-ez,node[6]+ez,enter,leave) end
		if enter then
			if not node.left then
				for at = node.first, node.last do count = count+1 output[count] = order[at] end
			end
			index = index+1
		else
			index = node.escape
		end
	end
	for at = #output, count+1, -1 do output[at] = nil end
	-- Preserve the old tie-breaking order for shared edges and contact normals.
	table.sort(output)
	return output
end

return tree
