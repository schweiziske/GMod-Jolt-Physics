local I = JoltI18n or include("jolt_i18n/init.lua")
JoltPath = JoltPath or {}
local P = JoltPath
P.MaxNodes = 32
P.MaxCoordinate = 32767
P.MaxLocalCoordinate = 131068 -- A rotated reference may span the world's full diagonal.
P.Modes = {"off", "velocity", "position", "pingpong"}
P.Rotations = {"free", "tangent", "normal", "binormal", "follow", "fixed"}
P.Actions = {"sync", "clear", "select_node", "remove_node", "move_up", "move_down", "apply", "motor", "pause", "reverse", "delete", "import", "set_node"}
P.Defaults = {closed = false, snap = true, visible = true, smoothing = 0.75, rotation = "follow",
    mode = "pingpong", speed = 128, target = 0.5, maxForce = 100000, friction = 0, frequency = 2, damping = 1}
P.NumberFields = {
    {"smoothing", .05, 1}, {"speed", -32767, 32767}, {"target", 0, 1},
    {"maxForce", 0, 1000000000}, {"friction", 0, 1000000000}, {"frequency", .1, 20}, {"damping", 0, 5},
}
P.BoolFields = {"closed", "snap", "visible"}
local function enumIndex(values, value)
    for i, candidate in ipairs(values) do if candidate == value then return i end end
end
P.EnumIndex = enumIndex
function P.Finite(value)
    return type(value) == "number" and value == value and value ~= math.huge and value ~= -math.huge
end
function P.ValidVector(value, maximum)
    return isvector(value) and P.Finite(value.x) and P.Finite(value.y) and P.Finite(value.z)
        and math.abs(value.x) <= maximum and math.abs(value.y) <= maximum and math.abs(value.z) <= maximum
end
function P.Settings(input)
    if input ~= nil and type(input) ~= "table" then return nil, I.Token("jolt.invalid_track_settings_format") end
    input = input or {}
    local out = {}
    for _, key in ipairs(P.BoolFields) do
        local value = input[key]
        if value == nil then value = P.Defaults[key] end
        if type(value) ~= "boolean" then return nil, key .. I.Token("jolt.must_be_a_boolean") end
        out[key] = value
    end
    for _, field in ipairs(P.NumberFields) do
        local key, minimum, maximum = unpack(field)
        local value = input[key]
        if value == nil then value = P.Defaults[key] end
        if not P.Finite(value) or value < minimum or value > maximum then return nil, key .. I.Token("jolt.exceeds_the_allowed_range") end
        out[key] = value
    end
    out.mode = input.mode == nil and P.Defaults.mode or input.mode
    out.rotation = input.rotation == nil and P.Defaults.rotation or input.rotation
    if not enumIndex(P.Modes, out.mode) or not enumIndex(P.Rotations, out.rotation) then return nil, I.Token("jolt.invalid_motion_rotation_mode") end
    return out
end
function P.CopyNodes(nodes)
    local out = {}
    for i, v in ipairs(nodes) do out[i] = Vector(v.x, v.y, v.z) end
    return out
end
function P.ValidateNodes(nodes, closed, draft)
    if type(nodes) ~= "table" or #nodes > P.MaxNodes then return nil, I.Token("jolt.at_most_32_nodes_are_allowed") end
    local count = #nodes
    for i = 1, count do if not P.ValidVector(nodes[i], P.MaxLocalCoordinate) then return nil, I.Token("jolt.node_list_must_be_contiguous_and_coordinates_valid") end end
    for key, v in pairs(nodes) do
        if type(key) ~= "number" or key % 1 ~= 0 or key < 1 or key > count
            or not P.ValidVector(v, P.MaxLocalCoordinate) then return nil, I.Token("jolt.incomplete_node_data_or_invalid_coordinates") end
    end
    if not draft and count < (closed and 3 or 2) then return nil, closed and I.Token("jolt.a_closed_track_needs_at_least_3_nodes") or I.Token("jolt.place_at_least_2_nodes_first") end
    for i = 1, count do
        local nextIndex = i + 1
        if nextIndex > count then nextIndex = closed and 1 or nil end
        if nextIndex and count > 1 and nodes[i]:DistToSqr(nodes[nextIndex]) < 1 then return nil, I.Token("jolt.adjacent_nodes_must_be_at_least_1_unit_apart") end
    end
    return true
end
function P.Tangents(nodes, smoothing, closed)
    local ok, reason = P.ValidateNodes(nodes, closed, false)
    if not ok then return nil, reason end
    local tangents, count = {}, #nodes
    for i, position in ipairs(nodes) do
        local before = position - nodes[(i - 2) % count + 1]
        local after = nodes[i % count + 1] - position
        if not closed and i == 1 then before = after end
        if not closed and i == count then after = before end
        local a, b = before:Length(), after:Length()
        local incoming, outgoing = before / a, after / b
        if incoming:Dot(outgoing) < -.98 then return nil, I.Token("jolt.node") .. i .. I.Token("jolt.nearly_doubles_back_adjust_its_position_to_form_a_smooth_bend") end
        tangents[i] = (incoming + outgoing):GetNormalized() * (math.min(a, b) * smoothing)
    end
    return tangents
end
function P.BuildCurve(nodes, settings)
    local tangents, reason = P.Tangents(nodes, settings.smoothing, settings.closed)
    if not tangents then return nil, reason end
    local points, distances, length = {nodes[1]}, {0}, 0
    local segments = settings.closed and #nodes or #nodes - 1
    for i = 1, segments do
        local nextIndex = i % #nodes + 1
        local chord = nodes[nextIndex] - nodes[i]
        for sample = 1, 16 do
            local t = sample / 16
            local t2, t3 = t * t, t * t * t
            local point = nodes[i] + tangents[i] * (t3 - 2*t2 + t)
                + chord * (-2*t3 + 3*t2) + tangents[nextIndex] * (t3 - t2)
            length = length + (point - points[#points]):Length()
            points[#points + 1], distances[#distances + 1] = point, length
        end
    end
    if length > 262144 then return nil, I.Token("jolt.total_track_length_cannot_exceed_262144_units") end
    return {points = points, distances = distances, length = length}
end
function P.PointAt(curve, progress)
    local target = math.Clamp(progress, 0, 1) * curve.length
    local low, high = 2, #curve.points
    while low < high do
        local mid = math.floor((low + high) / 2)
        if curve.distances[mid] < target then low = mid + 1 else high = mid end
    end
    local before, after = curve.distances[low - 1], curve.distances[low]
    return LerpVector(after > before and (target - before) / (after - before) or 0, curve.points[low - 1], curve.points[low])
end
function P.Frame(data)
    if data.referenceIndex and data.referenceIndex > 0 and not IsValid(data.reference) then data.reference = Entity(data.referenceIndex) end
    if data.referenceIndex and data.referenceIndex > 0 and not IsValid(data.reference) then return end
    if IsValid(data.reference) then
        local physics = data.reference:GetPhysicsObjectNum(data.referenceBone or 0)
        if IsValid(physics) then return physics:GetPos(), physics:GetAngles() end
        return data.reference:GetPos(), data.reference:GetAngles()
    end
    return vector_origin, angle_zero
end
function P.WorldNodes(data)
    local position, angles = P.Frame(data)
    if not position then return end
    local nodes = {}
    for i, node in ipairs(data.nodes) do nodes[i] = LocalToWorld(node, angle_zero, position, angles) end
    return nodes
end
function P.NativeSettings(settings)
    local out = P.Settings(settings)
    if out.mode == "pingpong" then out.mode = "velocity" end
    return out
end
-- Source's coordinate-compressed WriteVector is unsuitable for editable tracks
-- near the map limit or large reference-local offsets. Keep all three floats.
function P.WriteVector(value)
    net.WriteFloat(value.x); net.WriteFloat(value.y); net.WriteFloat(value.z)
end
function P.ReadVector() return Vector(net.ReadFloat(), net.ReadFloat(), net.ReadFloat()) end
function P.WriteData(data)
    net.WriteUInt(IsValid(data.reference) and data.reference:EntIndex() or 0, 16)
    net.WriteUInt(data.referenceBone or 0, 8)
    net.WriteUInt(#data.nodes, 6)
    for _, point in ipairs(data.nodes) do P.WriteVector(point) end
    local s = data.settings
    for _, key in ipairs(P.BoolFields) do net.WriteBool(s[key]) end
    for _, field in ipairs(P.NumberFields) do net.WriteFloat(s[field[1]]) end
    net.WriteUInt(enumIndex(P.Modes, s.mode) - 1, 2)
    net.WriteUInt(enumIndex(P.Rotations, s.rotation) - 1, 3)
end
function P.ReadData()
    local index = net.ReadUInt(16)
    local data = {referenceIndex = index, reference = index > 0 and Entity(index) or nil,
        referenceBone = net.ReadUInt(8), nodes = {}, settings = {}}
    local count = net.ReadUInt(6)
    if count > P.MaxNodes then return nil end
    for i = 1, count do data.nodes[i] = P.ReadVector() end
    for _, key in ipairs(P.BoolFields) do data.settings[key] = net.ReadBool() end
    for _, field in ipairs(P.NumberFields) do data.settings[field[1]] = net.ReadFloat() end
    data.settings.mode = P.Modes[net.ReadUInt(2) + 1]
    data.settings.rotation = P.Rotations[net.ReadUInt(3) + 1]
    if not data.settings.mode or not data.settings.rotation then return nil end
    local settings = P.Settings(data.settings)
    if not settings or not P.ValidateNodes(data.nodes, false, true) then return nil end
    data.settings = settings
    return data
end
return P
