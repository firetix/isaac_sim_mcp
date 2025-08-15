# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an Isaac Sim MCP (Model Context Protocol) Extension and Server that enables natural language control of NVIDIA Isaac Sim through AI assistants. The project consists of two main components:

1. **MCP Server** (`isaac_mcp/`) - A FastMCP server that communicates with AI assistants
2. **Isaac Sim Extension** (`isaac.sim.mcp_extension/`) - An Omniverse extension that runs inside Isaac Sim

The extension creates a socket server (localhost:8766) that accepts commands from the MCP server, enabling remote control of Isaac Sim simulations through natural language.

## Development Commands

### Environment Setup
```bash
# Install uv/uvx (required dependency manager)
# See: https://github.com/astral-sh/uv

# Install MCP CLI tools
uv pip install "mcp[cli]"
```

### Running the MCP Server
```bash
# Test the MCP server directly
uv run /path/to/isaac-sim-mcp/isaac_mcp/server.py

# Run with MCP inspector for debugging
uv run mcp dev ~/Documents/isaac-sim-mcp/isaac_mcp/server.py
# Visit debug interface at http://localhost:5173
```

### Isaac Sim Extension Setup
```bash
# Start Isaac Sim with extension enabled
cd ~/.local/share/ov/pkg/isaac-sim-4.2.0
./isaac-sim.sh --ext-folder /path/to/isaac-sim-mcp/ --enable isaac.sim.mcp_extension
```

Required environment variables:
```bash
export MESHY_API_KEY=<Your Meshy API Key from https://app.meshy.ai/>
export NVIDIA_API_KEY="<your nvidia api key>"
```

## Architecture Overview

### MCP Server Architecture (`isaac_mcp/server.py`)

**Core Components:**
- `FastMCP` server with lifespan management
- `IsaacConnection` class for socket communication with Isaac Sim
- Global connection management with automatic reconnection
- Tool decorators for MCP tool registration

**Key Tools:**
- `get_scene_info()` - Ping Isaac Sim and verify connection
- `create_physics_scene()` - Initialize physics environment with objects
- `create_robot()` - Add robots (franka, jetbot, carter, g1, go1) to scene
- `execute_script()` - Run arbitrary Python code in Isaac Sim
- `generate_3d_from_text_or_image()` - Use Meshy API for 3D model generation
- `search_3d_usd_by_text()` - Search USD asset libraries
- `transform()` - Apply position/scale transformations to objects

**Communication Protocol:**
- JSON-based command/response over TCP socket
- Commands: `{"type": "command_name", "params": {...}}`
- Responses: `{"status": "success|error", "result": {...}}`
- Chunked response handling with 300s timeout

### Isaac Sim Extension Architecture (`isaac.sim.mcp_extension/extension.py`)

**Core Components:**
- `MCPExtension` class implementing `omni.ext.IExt`
- Multi-threaded socket server (localhost:8766)
- Command routing system with async execution
- Integration with Omniverse Kit command system

**Command Handlers:**
- `execute_script()` - Execute Python in Isaac Sim context
- `create_physics_scene()` - Scene setup with objects and physics
- `create_robot()` - Robot instantiation with positioning
- `generate_3d_from_text_or_image()` - 3D model generation workflow
- `search_3d_usd_by_text()` - USD asset search and loading

**Threading Model:**
- Main server thread accepts connections
- Separate client handler threads for each connection
- Commands executed in Isaac Sim main thread using `run_coroutine()`

### Supporting Modules

**3D Generation (`isaac_sim_mcp_extension/gen3d.py`):**
- `Meshy3d` class for Meshy API integration
- Async task monitoring and model download
- Cache management for generated models

**USD Loading (`isaac_sim_mcp_extension/usd.py`):**
- `USDLoader` for loading and transforming USD models
- `USDSearch3d` for searching USD asset libraries
- Material and texture binding support

## Robot Simulation Patterns

### Basic Robot Creation
Always call in this order:
1. `get_scene_info()` - Verify connection
2. `create_physics_scene()` - Initialize physics
3. `create_robot()` - Add robot to scene

### Advanced Robot Control
For complex robot behaviors, use `execute_script()` with these patterns:

**Franka Robot Setup:**
```python
from omni.isaac.core import SimulationContext, World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.nucleus import get_assets_root_path

# Initialize World and physics
my_world = World(stage_units_in_meters=1.0)
simulation_context = SimulationContext()

# Load robot asset
assets_root_path = get_assets_root_path()
asset_path = assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd"
add_reference_to_stage(asset_path, "/Franka")

# Initialize physics and articulation
simulation_context.initialize_physics()
art = Articulation("/Franka")
art.initialize(my_world.physics_sim_view)

# Control robot
simulation_context.play()
for i in range(1000):
    art.set_joint_positions([-1.5], [art.get_dof_index("panda_joint2")])
    simulation_context.step(render=True)
```

### Physics Simulation Best Practices

1. **Async Patterns:** Use `World.step_async()` instead of blocking operations
2. **Physics Initialization:** Always call `create_physics_scene()` before robot operations
3. **Error Handling:** The extension includes retry logic and connection recovery
4. **Resource Management:** Clean up articulations and physics contexts properly

## Example Workflows

### Multi-Robot Factory Setup
```python
# 1. Initialize scene
get_scene_info()
create_physics_scene(floor=True, objects=[...])

# 2. Create robots in grid
for row in range(3):
    for col in range(3):
        create_robot("franka", [row*2, col*2, 0])
```

### 3D Model Generation and Placement
```python
# Generate from text/image using Meshy API
generate_3d_from_text_or_image(
    text_prompt="rusty desk",
    position=[0, 5, 0], 
    scale=[3, 3, 3]
)

# Or search existing USD assets
search_3d_usd_by_text("rusty desk", "/World/desk")
```

## Development Notes

- Extension listens on port 8766 by default (configurable in extension.toml)
- MCP server maintains persistent connection with automatic reconnection
- All robot assets loaded from Isaac Sim's nucleus server (configurable path)
- 3D generation requires valid Meshy API key from https://app.meshy.ai/
- Physics simulation runs at 60 FPS (1.0/60.0 dt) by default
- Socket communication uses 16KB buffer with chunked response handling

## Debugging

- Use MCP inspector at http://localhost:5173 when running with `mcp dev`
- Extension logs appear in Isaac Sim console
- Server logs include detailed connection and command execution info
- Check physics context initialization if robot control fails