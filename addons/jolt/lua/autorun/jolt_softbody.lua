if JoltSoftBodies and JoltSoftBodies.Shutdown then JoltSoftBodies.Shutdown() end
JoltSoftBodies = {}

local sharedFiles = {"config", "geometry", "buffer", "protocol"}
local clientFiles = {"collision_tree", "collision", "renderer", "client", "tool_client", "panel", "options"}
if SERVER then
	AddCSLuaFile()
	for _, name in ipairs(sharedFiles) do AddCSLuaFile("jolt_softbody/" .. name .. ".lua") end
	for _, name in ipairs(clientFiles) do AddCSLuaFile("jolt_softbody/" .. name .. ".lua") end
end

JoltSoftBodies.Config = include("jolt_softbody/config.lua")
JoltSoftBodies.Geometry = include("jolt_softbody/geometry.lua")
JoltSoftBodies.Buffer = include("jolt_softbody/buffer.lua")
JoltSoftBodies.Protocol = include("jolt_softbody/protocol.lua")

if SERVER then
	JoltSoftBodies.Server = include("jolt_softbody/server.lua")
	JoltSoftBodies.Replication = include("jolt_softbody/replication.lua")
	JoltSoftBodies.Editing = include("jolt_softbody/editing.lua")
else
	JoltSoftBodies.Collision = include("jolt_softbody/collision.lua")
	JoltSoftBodies.Renderer = include("jolt_softbody/renderer.lua")
	JoltSoftBodies.Client = include("jolt_softbody/client.lua")
	JoltSoftBodies.ToolClient = include("jolt_softbody/tool_client.lua")
	JoltSoftBodies.Options = include("jolt_softbody/options.lua")
end
