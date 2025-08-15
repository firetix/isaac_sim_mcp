# Isaac Sim MCP Extension and MCP Server

The MCP Server and its extension leverage the Model Context Protocol (MCP) framework to enable natural language control of NVIDIA Isaac Sim, transforming conversational AI inputs into precise simulation manipulation. This expansion bridges the MCP ecosystem with embodied intelligence applications.

## Features

- Natural language control of Isaac Sim
- Dynamic robot positioning and movement
- Custom lighting and scene creation
- Advanced robot simulations with obstacle navigation
- Interactive code preview before execution

## Requirements

- NVIDIA Isaac Sim 4.2.0 or higher
- Python 3.9+
- Cursor AI editor for MCP integration

## **Mandatory** Pre-requisite

- Install uv/uvx: [https://github.com/astral-sh/uv](https://github.com/astral-sh/uv)
- Install mcp[cli] to base env: [uv pip install "mcp[cli]"](https://pypi.org/project/mcp/)

## Installation

```bash
cd ~/Documents
git clone https://github.com/firetix/isaac_sim_mcp.git
cd isaac_sim_mcp
```

### Setup API Keys

Set up your API keys for Meshy and NVIDIA services:

```bash
# Option 1: Using environment variables
export MESHY_API_KEY=<Your Meshy API Key from https://app.meshy.ai/>
export NVIDIA_API_KEY="<your nvidia api key from https://ngc.nvidia.com/signout>"

# Option 2: Using .env file (recommended for development)
# Edit the .env file in the project root with your API keys
```

### Install MCP Dependencies

```bash
# Install uv if not already installed
curl -LsSf https://astral.sh/uv/install.sh | sh

# Install MCP CLI tools
uv pip install "mcp[cli]"
```

### Test MCP Server

```bash
# Test the MCP server runs correctly
uv run isaac_mcp/server.py

# Or run with MCP inspector for debugging
uv run mcp dev isaac_mcp/server.py
# Visit debug interface at http://localhost:5173
```

### Install and Enable Isaac Sim Extension

```bash
# Find your Isaac Sim installation (adjust path as needed)
# Common locations:
# ~/.local/share/ov/pkg/isaac-sim-4.2.0  (Linux)
# ~/Library/Application Support/ov/pkg/isaac-sim-4.2.0  (macOS)

# Enable extension in Isaac Simulation
cd ~/.local/share/ov/pkg/isaac-sim-4.2.0
./isaac-sim.sh --ext-folder ~/Documents/isaac_sim_mcp/ --enable isaac.sim.mcp_extension 
```

Verify the extension starts successfully. The output should look like:

```
[7.160s] [ext: isaac.sim.mcp_extension-0.1.0] startup
trigger  on_startup for:  isaac.sim.mcp_extension-0.1.0
settings:  {'envPath': '/home/ubuntu/.local/share/ov/data/Kit/Isaac-Sim/4.2/pip3-envs/default', 'archiveDirs': {}, 'installCheckIgnoreVersion': False, 'allowOnlineIndex': True, 'tryUpgradePipOnFirstUse': False}
Server thread startedIsaac Sim MCP server started on localhost:8766
```

The extension should be listening at **localhost:8766** by default.



### Configure Cursor AI (Optional)

To use with Cursor AI editor:

1. Start Cursor and open the project folder
2. Go to Cursor preferences, choose MCP and add a global MCP server:

```json
{
    "mcpServers": {
        "isaac-sim": {
            "command": "uv run /absolute/path/to/isaac_sim_mcp/isaac_mcp/server.py"
        }
    }
}
```

### Verify Installation

1. **MCP Server**: Should start without errors and show available tools
2. **Isaac Sim Extension**: Should load and show "Isaac Sim MCP server started on localhost:8766"
3. **API Keys**: Test 3D generation with a simple prompt to verify Meshy API connectivity

## Example Prompts for Simulation
Notice: Switch to Agent mode in top left of Chat dialog before you type prompt and choose sonnet 3.7 for better coding.

### Robot Party
```
# Create robots and improve lighting
create  3x3 frankas robots in these current stage across location [3, 0, 0] and [6, 3, 0]
always check connection with get_scene_info before execute code.
add more light in the stage


# Add specific robots at positions
create a g1 robot at [3, 9, 0]
add Go1 robot at location [2, 1, 0]
move go1 robot to [1, 1, 0]
```

### Factory Setup
```
# Create multiple robots in a row
acreate  3x3 frankas robots in these current stage across location [3, 0, 0] and [6, 3, 0]
always check connection with get_scene_info before execute code.
add more light in the stage


```
### Vibe Coding from scratch
```
reference to g1.py to create an new g1 robot simulation and allow robot g1 walk straight  from [0, 0, 0] to [3, 0, 0] and [3, 3, 0]
create more obstacles in the stage

```
### Gen3D with Meshy model support

```
Use following images to generate 3D objects and place them into a grid area across [0, 0, 0] to [40, 40, 0] with scale [3, 3, 3]

<your image url here, could be multiple images urls>
```

### USD search
```
search a rusty desk and place it at [0, 5, 0] with scale [3, 3, 3]
```

## MCP Tools

The Isaac Sim MCP Extension provides several specialized tools that can be accessed through natural language in Cursor AI. These tools enable you to control and manipulate NVIDIA Isaac Sim with simple commands:

### Connection and Scene Management

- **get_scene_info** - Pings the Isaac Sim Extension Server to verify connection status and retrieve basic scene information. Always use this first to ensure the connection is active.

### Physics and Environment Creation

- **create_physics_scene** - Creates a physics scene with configurable parameters:
  - `objects`: List of objects to create (each with type and position)
  - `floor`: Whether to create a ground plane (default: true)
  - `gravity`: Vector defining gravity direction and magnitude (default: [0, -0.981, 0])
  - `scene_name`: Name for the scene (default: "physics_scene")

### Robot Creation and Control

- **create_robot** - Creates a robot in the scene at a specified position:
  - `robot_type`: Type of robot to create (options: "franka", "jetbot", "carter", "g1", "go1")
  - `position`: [x, y, z] position coordinates

### Omniverse Kit and Scripting

- **omni_kit_command** - Executes an Omni Kit command:
  - `command`: The Omni Kit command to execute (e.g., "CreatePrim")
  - `prim_type`: The primitive type for the command (e.g., "Sphere")

- **execute_script** - Executes arbitrary Python code in Isaac Sim:
  - `code`: Python code to execute

### Usage Best Practices

1. Always check connection with `get_scene_info` before executing any commands
2. Initialize a physics scene with `create_physics_scene` before adding robots
3. Use `create_robot` for standard robot placement before trying custom scripts
4. For complex simulations, use `execute_script` with proper async patterns
5. Preview code in chat before execution for verification

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Video Demonstrations

Below are demonstrations of the Isaac Sim MCP Extension in action:

### Robot Party Demo

![Robot Party Demo](media/add_more_robot_into_party.gif)

*GIF: Adding more robots to the simulation using natural language commands*


### Video Format (MP4)

For higher quality video, you can access the MP4 version directly:

- [Robot Party Demo (MP4)](media/add_more_robot_into_party.mp4)

When viewing on GitHub, you can click the link above to view or download the MP4 file.
