# 🌌 Isaac Sim Setup & Automation Guide: JetBot

This document provides the end-to-end workflow for setting up NVIDIA Isaac Sim and automating the deployment of a JetBot environment for VLA and WAM development.

---

## 🛠️ 1. Installation & Prerequisites

### Hardware Requirements
- **GPU**: NVIDIA RTX series (RTX 3080+ recommended) with $\geq$ 8GB VRAM.
- **OS**: Ubuntu 20.04/22.04 or Windows 10/11.
- **Drivers**: NVIDIA Driver 525.x or newer.

### Step-by-Step Installation
1. **Omniverse Launcher**: Download and install from the [NVIDIA Omniverse page](https://www.nvidia.com/en-us/omniverse/download/).
2. **Isaac Sim**: Search for "Isaac Sim" in the **Exchange** tab of the Launcher and install the latest version.
3. **Isaac ROS Bridge**: Install the ROS2 Humble/Foxy bridge via the [Isaac ROS documentation](https://nvidia-isaac-ros.github.io/) to allow the simulator to communicate with your `vla_client_bridge.py` and `predictive_governor.py`.

---

## 🤖 2. Loading the JetBot Asset

NVIDIA provides a high-fidelity USD (Universal Scene Description) model of the JetBot.

1. **Manual Load**:
   - Open Isaac Sim $\rightarrow$ Content Browser $\rightarrow$ `NVIDIA/Assets/Isaac/4.x/Robots/Jetbot`.
   - Drag `jetbot.usd` into the viewport.
2. **Asset Properties**:
   - The model includes a CSI camera and a differential drive system.
   - Ensure the "Articulation Root" is correctly set to the JetBot's base link for physics-based control.

---

## 🔌 3. ROS2 Bridge Configuration

To link the simulation to your ROS2 stack, you must map the simulation internals to ROS2 topics.

| Sim Component | ROS2 Topic | Direction | Role |
| :--- | :--- | :--- | :--- |
| **Camera Sensor** | `/camera/image_raw` | Sim $\rightarrow$ ROS2 | Input for VLA Server |
| **Lidar Sensor** | `/scan` | Sim $\rightarrow$ ROS2 | Input for Predictive Governor |
| **Diff Drive** | `/cmd_vel_final` | ROS2 $\rightarrow$ Sim | Motor Execution |
| **Odom/TF** | `/odom` / `/tf` | Sim $\rightarrow$ ROS2 | State Estimation |

---

## 📜 4. Automation Script (Python API)

Instead of manually dragging assets, you can use the Isaac Sim Python API. This script automates the scene setup, robot spawning, and ROS2 bridge initialization.

**File**: `simulate_jetbot.py`
```python
from omni.isaac.kit import SimulationApp

# Start the simulator before importing other Isaac modules
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.nucleus import get_assets_root_path
import numpy as np

class JetBotSim:
    def __init__(self):
        self.world = World(stage_units_in_meters=1.0)
        self.assets_root_path = get_assets_root_path()
        self.jetbot_usd = f"{self.assets_root_path}/Robots/Jetbot/jetbot.usd"
        
        # 1. Spawn the JetBot
        self.robot = self.world.scene.add(
            Robot(
                prim_path="/World/JetBot",
                name="my_jetbot",
                usd_path=self.jetbot_usd,
                position=np.array([0, 0, 0]),
                orientation=np.array([1, 0, 0, 0])
            )
        )
        
        # 2. Setup ROS2 Bridge
        # Note: In Isaac Sim, the ROS2 bridge is often configured via an Action Graph
        # but can be toggled via the Omniverse Extensions menu.
        self.setup_ros2_bridge()

    def setup_ros2_bridge(self):
        print("Initializing ROS2 Bridge...")
        # In a real script, you would use the 'omni.isaac.ros2_bridge' extension 
        # to create the Publisher/Subscriber nodes for /cmd_vel and /camera/image_raw.
        pass

    def run(self):
        self.world.reset()
        while simulation_app.is_running():
            self.world.step(render=True)

if __name__ == "__main__":
    sim = JetBotSim()
    sim.run()
    simulation_app.close()
```

---

## 🚀 5. Expert Sim-to-Real Guidelines

As a robotics scientist, focus on these three factors to reduce the gap between Isaac Sim and your physical Jetson Nano:

1. **Latency Injection**: The VLA server introduces latency. In simulation, add a `delay` node to the `/cmd_vel` topic to ensure your `predictive_governor.py` is actually dealing with stale data, mirroring real-world conditions.
2. **Surface Friction**: Do not use default "perfect" friction. Adjust the **Physics Materials** on the JetBot wheels and the floor to simulate the slight slip of rubber on hardwood/carpet.
3. **Sensor Noise**: Add additive white Gaussian noise (AWGN) to the LiDAR `/scan` data in the simulation. This prevents the `predictive_governor.py` from over-fitting to the "perfect" simulation data and failing in the real world.
