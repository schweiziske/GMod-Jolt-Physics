# Soft-Body Client

[Home](../../../README.md)

How the Soft Body Workshop sends soft bodies to clients and draws them. Controls, settings and the Lua API are in the [soft-body guide](../../../gmsv_joltsoftbody/README.md). Clients run only the addon's Lua; the native module stays on the server.

## Replication

- **Sampling.** The server reads each body with `ReadSnapshot` up to 15 times per second, and a sleeping body in full at most once per second. A moving body gets a new frame every sample; a sleeping one only when its quantized pose or pins change. Each tick the server also refreshes the entity's collision bounds from `ReadState`, so traces keep finding fast bodies.
- **Interest.** Every 1/`jolt_softbody_interest_rate` s, a body counts as visible to a player if it is within `jolt_softbody_view_distance` of their eyes and passes `TestPVS`; it is sent at `jolt_softbody_near_rate` within `jolt_softbody_near_distance` and at `jolt_softbody_far_rate` beyond. A body that drops out gets a remove message, and its metadata is resent when it returns.
- **Budget.** Each player has a token bucket refilled at `jolt_softbody_net_kib` KiB/s that holds at most half a second's worth. A frame costs its payload plus 96 bytes, plus 18 bytes of metadata the first time. Bodies are served round-robin; when the next one doesn't fit, the scheduler waits on it until enough credit builds up, so small meshes can't starve a large one.
- **Reliability.** Metadata, a player's first frame of a body and the final sleeping pose are reliable; other frames are unreliable and may be dropped.
- **Joining and reloads.** A client sends a ready message up to four times, 3 s apart. The server then forgets what that player has received and resends everything.

## Wire format

| Message | Contents |
| --- | --- |
| `jolt_softbody_meta_v1` | Protocol version, body ID, entity index, shape, quality, width, height, color, pin preset and player-collision flag. The client regenerates the topology from these, so frames carry positions only. |
| `jolt_softbody_frame_v1` | Body ID (32 bits), sequence (16), server time, center (3 floats), scale (float), vertex count (16), active flag, then three signed 16-bit coordinates and a pinned bit per vertex: 225 + 49 bits per vertex |
| `jolt_softbody_remove_v1` | Body ID |

A coordinate decodes as `center + q * scale` with `scale = max(1/16, halfSize / 32760, maxCoord / 8388608)`, where `halfSize` is half the longest side of the bounds and `maxCoord` the largest absolute center coordinate. The step is normally 1/16 unit and grows for very large bodies or far from the map origin, where float precision runs out.

## Interpolation

- Each body keeps a ring of four snapshots. Only newer sequence numbers (with 16-bit wraparound) are accepted, so late and duplicate frames are dropped. A gap of more than 0.75 s of server time resets the buffer and clock.
- The first frame maps server time to client game time (`CurTime()`); playback then advances with game-time deltas, so pause and time scale behave normally. Its rate is nudged within ±5% to follow drift instead of jumping.
- The delay is `max(100 ms, 1.4 × packet interval + 2 × jitter)`, capped at 350 ms and at three packet intervals: about 100 ms at 15 Hz.
- Playback never passes the newest snapshot, so it holds the last pose when data stops. It never extrapolates or steps back when the delay grows, and there is no local physics prediction.

## Rendering

- A moving body is drawn every frame with a dynamic `mesh.Begin(MATERIAL_TRIANGLES, ...)`. Once it sleeps and interpolation settles, the pose is written once to a static `IMesh` that is drawn until the body wakes; the next sleep rewrites the same mesh. It is destroyed explicitly when the body stops replicating, is removed, or Lua reloads.
- The material is `UnlitGeneric` with vertex colors and one UV channel. Lighting is baked into the vertex colors from smoothed normals and `render.GetLightColor`, sampled every 0.5 s while the body moves. This avoids `IMesh` lighting differences under `VertexLitGeneric` and needs no tangents, normal maps, PBR, compute shaders or extra render targets.
- Surfaces are opaque. Don't enable `$vertexalpha`: it stops `UnlitGeneric` from writing depth even at alpha 255. The creation preview and the wireframe use a separate translucent wireframe material.
- A body has at most 3840 render corners (1280 triangles), far below the mesh API's 65535-vertex limit.
- The creation preview is rebuilt only when its shape, quality, size, color or pins change; moving it only updates a reused `Matrix`.

## Allocation and culling

- `ReadSnapshot` and `ReadVertex` fill caller-owned tables, so routine replication creates no per-vertex tables or Vectors. Render caches, normals, quantized arrays, receive buffers and recipient lists are reused, and loops stay numeric so LuaJIT can trace them. The addon never stops, forces or tunes garbage collection; engine calls such as eye position and lighting still allocate a little.
- Bounds update in `Think`, not `Draw`, so a culled body can come back into view. Distance culling uses `LocalPlayer():EyePos()`, because the global `EyePos()` describes the current render pass and can be zero in `Think`.

## Client collision

On the client, the entity's `TestCollision` traces the displayed mesh, so movement prediction matches what the player sees; the server traces the native body. Topology never changes, so each body builds a bounding-volume tree once and refits it when the displayed pose changes. `jolt_softbody_collision_profile 1` records trace timings for `jolt_softbody_client_stats`.

## Diagnostics

- `jolt_softbody_server_stats` (admins or the server console): bodies, vertices, connected players, frames and KiB sent.
- `jolt_softbody_client_stats`: bodies, vertices, frames, KiB received and rejected packets; DX9 meshes, static writes and dynamic draws; collision timings. Its Lua heap figure covers the whole client Lua state, not just soft bodies.

## Code map

Scripts are in `addons/jolt/lua/`, loaded by `autorun/jolt_softbody.lua`.

| File | Responsibility |
| --- | --- |
| `jolt_softbody/config.lua` | Limits, presets, ConVar definitions, bandwidth estimate |
| `jolt_softbody/geometry.lua` | Cloth and icosphere generation, placement, normals, ray tests |
| `jolt_softbody/buffer.lua` | Reusable snapshot buffers, the four-slot ring, the playback clock |
| `jolt_softbody/protocol.lua` | Quantization and message format |
| `jolt_softbody/server.lua` | Native bodies, entity lifetime, sampling, player push, server settings |
| `jolt_softbody/replication.lua` | Interest, late joining, bandwidth scheduling, `jolt_softbody_server_stats` |
| `jolt_softbody/editing.lua` | Edit permissions, vertex picking, pinning, dragging |
| `jolt_softbody/client.lua` | Receiving, entity binding, interpolation, culling, `jolt_softbody_client_stats` |
| `jolt_softbody/renderer.lua` | Vertex lighting and DX9 mesh lifetime |
| `jolt_softbody/collision.lua`, `collision_tree.lua` | Client traces against the displayed mesh |
| `jolt_softbody/tool_client.lua`, `panel.lua` | Tool preview, vertex markers, HUD, tool panel |
| `jolt_softbody/options.lua` | The **Q → Options → Jolt → Soft body** page |
| `entities/sent_jolt_softbody/` | Entity: traces, damage impulses, drawing |
| `weapons/gmod_tool/stools/jolt_softbody.lua` | Sandbox tool |

Only the workshop's preset cloth grids and icosphere balls replicate, because clients rebuild them from metadata. Bodies created directly with `jolt.softbody.Create` aren't networked or drawn.
