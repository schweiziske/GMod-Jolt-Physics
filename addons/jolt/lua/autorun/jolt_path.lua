if SERVER then
    AddCSLuaFile("jolt_path/shared.lua")
    AddCSLuaFile("jolt_path/client.lua")
end
include("jolt_path/shared.lua")
if SERVER then include("jolt_path/server.lua") else include("jolt_path/client.lua") end
