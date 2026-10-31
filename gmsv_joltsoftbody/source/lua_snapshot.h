#pragma once

#include <GarrysMod/Lua/Interface.h>
#include <vjolt/softbody.h>

namespace JoltSoftBodyLua
{
int ReadStateIntoTable( GarrysMod::Lua::ILua *lua, IVJoltSoftBodyBridge *bridge, VJoltSoftBodyHandle handle );
int ReadSnapshotIntoTable( GarrysMod::Lua::ILua *lua, IVJoltSoftBodyBridge *bridge, VJoltSoftBodyHandle handle );
int ReadVertexIntoTable( GarrysMod::Lua::ILua *lua, IVJoltSoftBodyBridge *bridge, VJoltSoftBodyHandle handle );
} // namespace JoltSoftBodyLua
