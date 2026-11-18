# rov_physics_sim

Physics-accurate ROV simulation (Gazebo Harmonic + ROS 2 Jazzy).
Separate from `rov_gazebo_sim`, which is the velocity-controlled nav demo.

Status: M0 (environment only). The world is empty; no vehicle yet.

## Prerequisites
- Docker Desktop (WSL 2 backend on Windows) running
- VS Code with the Dev Containers extension
- Foxglove desktop app

## Setup
1. Clone the repo and open the folder in VS Code (on Windows, clone inside WSL).
2. Press Ctrl+Shift+P, run **Dev Containers: Reopen in Container**.
   The first build downloads several GB and takes 5 to 15 minutes.

## Launch
In a terminal inside the container:

```bash
cd /workspace
colcon build --packages-select rov_physics_sim
source install/setup.bash
ros2 launch rov_physics_sim sim.launch.py headless:=true
```

Omit `headless:=true` to open the Gazebo GUI (requires display passthrough).

## View in Foxglove
1. Open Foxglove, click **Open connection**, choose **Foxglove WebSocket**.
2. Enter `ws://localhost:8765` and click Open.
3. Add a **Raw Messages** panel and enter `/clock`. Simulation time should count up.

## Tuning
Not applicable yet. Physics parameters arrive in M1 and later.
