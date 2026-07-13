# 🚀 Action Plan: JetBot VLA-ROS2 Project

This plan outlines the step-by-step process to transform a standard JetBot into a VLA-controlled autonomous agent.

## Phase 1: Foundation (The Edge)
**Goal**: Get the robot moving and sensing within a ROS2 environment.
1. **OS Installation**: Flash the latest NVIDIA JetPack image to the SD card.
2. **ROS2 Setup**: Install ROS2 (Foxy or Humble) on the Jetson Nano.
3. **Driver Integration**: 
   - Install the hardware drivers for the motors and camera.
   - Implement the ROS2 wrapper (referencing `jetbot-ros2`) to expose motor controls as ROS2 topics.
4. **LiDAR Setup**: 
   - Connect the LiDAR and install the corresponding ROS2 driver.
   - Verify that the `/scan` topic is publishing correct distance data.

## Phase 2: Navigation & Mapping (The "Body")
**Goal**: Enable the robot to know where it is and how to move safely.
1. **SLAM**: Use `slam_toolbox` to drive the robot around the room and create a high-resolution 2D map.
2. **Nav2 Config**: Configure the Navigation 2 stack with the JetBot's footprint and the LiDAR as the primary costmap sensor.
3. **Testing**: Successfully command the robot to move to a specific coordinate on the map without hitting walls.

## Phase 3: Intelligence Infrastructure (The "Brain")
**Goal**: Set up the server to provide high-level reasoning.
1. **Server Setup**: Prepare a PC with an NVIDIA GPU (24GB+ VRAM recommended) and install CUDA/PyTorch.
2. **VLA Deployment**: 
   - Deploy **OpenVLA** or a similar model.
   - Set up a REST API (FastAPI/Flask) that accepts an image and a text prompt and returns an action sequence.
3. **API Bridge**: Create a ROS2 node on the Jetson Nano that acts as the "Bridge"—it captures images and communicates with the server.

## Phase 4: Application Development (The "Cool Stuff")
**Goal**: Implement actual high-level tasks.
1. **Semantic Navigation**: Use the VLA to tell the robot: *"Find the coffee machine"* $\rightarrow$ Server identifies the object $\rightarrow$ Nano navigates to it.
2. **Reactive-VLA Loop**: Implement a loop where the VLA provides high-level goals, but the local YOLO/Nav2 system handles immediate obstacle avoidance.
3. **Project Polish**: Document the process and record a demo of the robot following complex linguistic instructions.

## 📉 Success Metrics
- [ ] Robot can be teleoperated via ROS2.
- [ ] Robot can generate a map of a room via LiDAR.
- [ ] Robot can navigate to a goal coordinate autonomously.
- [ ] Robot successfully executes a command sent from the VLA server.
