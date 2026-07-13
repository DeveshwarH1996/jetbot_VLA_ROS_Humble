# ⚙️ JetBot ROS2 Configuration & Launch Guide

This document provides the necessary configuration and launch logic to bring up the JetBot system with VLA-integration and priority-based control.

## 1. `twist_mux` Configuration
The `twist_mux` node is critical for allowing seamless switching between Joystick (Manual) and VLA (Auto) modes.

**File**: `config/twist_mux.yaml`
```yaml
twist_mux:
  ros__parameters:
    # Priority: Lower number = Higher priority
    # 1. Emergency/Safety Veto (Highest)
    # 2. Joystick/Manual Control
    # 3. VLA Autonomous Output (Lowest)
    topics:
      - "/cmd_vel_safety"     # Priority 1
      - "/cmd_vel_joy"        # Priority 2
      - "/cmd_vel_final"      # Priority 3
    locks:
      - "/twist_mux/lock"
```

## 2. Master Launch File
This launch file brings up the entire pipeline. In a production environment, this would be a `.launch.py` file. 

**Logical Sequence**:
1. `jetbot_hardware_driver`: Initializes the Pico W communication and motor PWM.
2. `lidar_node`: Starts the LiDAR driver and publishes to `/scan`.
3. `twist_mux`: Initializes the priority multiplexer.
4. `predictive_governor`: Starts the System 1 safety valve.
5. `vla_client_bridge`: Starts the System 2 server connection.

**Suggested Launch Command (Conceptual)**:
```bash
ros2 launch jetbot_project bringup.launch.py \
    server_url:="http://<your-server-ip>:8000/predict" \
    safety_threshold:=0.4
```

## 3. Manual $\rightarrow$ Auto Switch Logic
To implement the "press of a button" switch, you should run a simple bridge node that monitors the `/joy` topic:

```python
# Conceptual logic for the Mode Switcher
def joy_callback(msg):
    if msg.buttons[0] == 1: # Assuming button 0 is the Toggle
        self.auto_mode_enabled = not self.auto_mode_enabled
        self.get_logger().info(f"Mode Switched: {'AUTO' if self.auto_mode_enabled else 'MANUAL'}")
        # In a real system, you could publish to /twist_mux/lock to 
        # explicitly disable the VLA priority level.
```

## 4. Distributed Deployment (Simulation Server $\rightarrow$ Local Client)
Since Isaac Sim is compute-intensive, it is recommended to run it on a dedicated GPU server while running your ROS2 logic on the Jetson Nano.

### 🌐 Network Configuration
ROS2 uses DDS (Data Distribution Service) for discovery. For two machines to communicate, they must be on the same subnet and share the same Domain ID.

1. **Set the Domain ID**: 
   Add this to the `.bashrc` of both the server and the client to ensure they are on the same virtual network:
   ```bash
   export ROS_DOMAIN_ID=10  # Choose any integer between 0-232
   ```
2. **Firewall Requirements**: 
   DDS uses UDP multicast for discovery. Ensure your firewalls allow UDP traffic. On Ubuntu, you can temporarily disable the firewall to test connectivity:
   ```bash
   sudo ufw disable
   ```
3. **Verification**: 
   Run the simulator on the server, then run this on the client to verify discovery:
   ```bash
   ros2 topic list
   ```
   You should see `/scan` and `/odom` appearing automatically from the remote server.

### ⚠️ Latency & Jitter Considerations
Running the simulator remotely introduces network latency. This transforms your system from a "perfect" simulation into a "realistic" one. This makes the `predictive_governor` critical, as it must handle "stale" packets arriving from the server.

---

## 5. Summary of Nodes for a "Moving Robot"
To get the vehicle moving, the following processes must be active:
1. **Hardware**: `jetbot_driver` $\rightarrow$ publishes `/odom`, subscribes `/cmd_vel_mux`
2. **Sensing**: `lidar_driver` $\rightarrow$ publishes `/scan`
3. **Multiplexing**: `twist_mux` $\rightarrow$ subscribes `/cmd_vel_joy`, `/cmd_vel_final`, `/cmd_vel_safety` $\rightarrow$ publishes `/cmd_vel_mux`
4. **Cognition**: `vla_client_bridge` $\rightarrow$ publishes `/cmd_vel_vla`
5. **Safety**: `predictive_governor` $\rightarrow$ subscribes `/cmd_vel_vla` $\rightarrow$ publishes `/cmd_vel_final`
6. **Control**: `teleop_twist_joy` $\rightarrow$ publishes `/cmd_vel_joy`

