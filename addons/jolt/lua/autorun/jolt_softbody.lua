if JoltSoftBodies and JoltSoftBodies.Shutdown then JoltSoftBodies.Shutdown() end
JoltSoftBodies = {}

local sharedFiles = {"config", "geometry", "buffer", "protocol"}
local clientFiles = {"collision_tree", "collision", "renderer", "client", "tool_client", "panel", "options"}
if SERVER then
	AddCSLuaFile()
	AddCSLuaFile("jolt_softbody/physgun.lua")
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
	JoltSoftBodies.Attachments = include("jolt_softbody/attachments.lua")
	JoltSoftBodies.Tearing = include("jolt_softbody/tearing.lua")
	JoltSoftBodies.Feedback = include("jolt_softbody/feedback.lua")
else
	JoltSoftBodies.Collision = include("jolt_softbody/collision.lua")
	JoltSoftBodies.Renderer = include("jolt_softbody/renderer.lua")
	JoltSoftBodies.Client = include("jolt_softbody/client.lua")
	JoltSoftBodies.ToolClient = include("jolt_softbody/tool_client.lua")
	JoltSoftBodies.Options = include("jolt_softbody/options.lua")
end

JoltSoftBodies.Physgun = include("jolt_softbody/physgun.lua")
