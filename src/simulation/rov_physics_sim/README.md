# rov_physics_sim

Physics-accurate ROV simulation (Gazebo Harmonic + ROS 2 Jazzy). The ROV is a rigid body with
real gravity, buoyancy and hydrodynamic damping, in a model of the competition diving well.

It is separate from `rov_gazebo_sim`, the velocity-controlled Nav2 demo, which stays untouched.

Status: M1 in progress. The ROV floats, trims and can be dropped at any of the launch-station
layouts. Thrusters come in M2. Mass properties are placeholders until Mech's numbers arrive.

## Prerequisites

- Docker Desktop (WSL 2 backend on Windows) running
- VS Code with the Dev Containers extension
- Foxglove desktop app

## Setup

1. Clone the repo and open the folder in VS Code (on Windows, clone inside WSL).
2. Press Ctrl+Shift+P, run **Dev Containers: Reopen in Container**.
   The first build downloads several GB and takes 5 to 15 minutes. Rebuild the container
   after pulling changes to `.devcontainer/`.

## Launch

In a terminal inside the container:

```bash
cd /workspace
colcon build --packages-select rov_physics_sim
source install/setup.bash
ros2 launch rov_physics_sim sim.launch.py headless:=true
```

| Argument | Default | Description |
|----------|---------|-------------|
| `headless` | `false` | Run Gazebo without its GUI (needed on Windows for now) |
| `station` | `no_wall` | Launch station layout: `no_wall`, `wall_left`, `wall_right`, `both_walls` |
| `spawn_z` | from `pool.yaml` (0) | Spawn height of the ROV's centre; e.g. `-1.0` to drop it in submerged |
| `show_volumes` | `false` | Draw the buoyancy boxes in the Gazebo GUI |
| `vehicle_config`, `pool_config` | packaged files | Use another `vehicle.yaml` or `pool.yaml` |
| `foxglove` | `true` | Start a Foxglove bridge |
| `foxglove_port` | `8766` | Its port. 8765 belongs to the team's infrastructure container and `rov_gazebo_sim` |
| `zenoh_router` | `true` under Zenoh | Start a Zenoh router (the team's middleware) |

## View in Foxglove

1. Open Foxglove, click **Open connection**, choose **Foxglove WebSocket**.
2. Enter `ws://localhost:8766` and click Open.
3. Add a **Plot** panel with `/sim/ground_truth.pose.pose.position.z` to watch the ROV settle,
   or a **3D** panel with its display frame set to `world` to follow `base_link`.

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/clock` | `rosgraph_msgs/Clock` | Sim time |
| `/sim/ground_truth` | `nav_msgs/Odometry` | True pose in `world` (surface at z = 0, up is +z) and body-frame velocity, 50 Hz. Same name as `rov_gazebo_sim` |
| `/tf` | `tf2_msgs/TFMessage` | `world` -> `base_link` |

Thruster commands (M2) are waiting on the real robot's interface from Controls / ROS infra.

## The world

`config/pool.yaml` describes the King County diving well (22.86 x 15.54 m, 5.18 m deep), which
MATE told us to plan around, and the team's launch station on one wall. MATE says stations sit
along a pool wall and may have a sidewall on either side, or none in the middle. The `station`
argument picks the layout; the sidewall size and spacing are guesses until MATE's Prop Building
Instructions arrive (Jan/Feb 2027). The ROV starts 0.6 m off the wall, facing into the pool.

## The vehicle and tuning

Everything tunable lives in `config/vehicle.yaml`: mass, centre of mass, inertia, the buoyant
volumes, and the hydrodynamic coefficients. The model is generated from it at launch, so
there is no SDF to edit. `base_link` is x forward, y left, z up, at the centre of Mech's envelope.

Gazebo computes buoyancy from collision geometry, and its surface (graded) mode only handles
boxes and spheres. So each part that displaces water is a box with the right volume and centre:
the frame rails, the sealed electronics tube, the thrusters and the flotation blocks. Boxes
must not overlap. The visual is the CAD STL (tether removed, decimated) and has no effect on
the physics.

After any change, check the numbers without starting Gazebo:

```bash
ros2 run rov_physics_sim hydrostatics
```

It prints the mass, displaced volume, net buoyancy, centre of buoyancy, BG (how far the centre
of buoyancy sits above the centre of mass; positive means self-righting), the expected trim, and
where the ROV should float.

## Checking trim (M1 acceptance)

With the sim running, in a second terminal:

```bash
ros2 run rov_physics_sim trim_check --settle 30
```

It waits 30 s of sim time, averages the last 5 s of `/sim/ground_truth`, and compares the
depth, roll and pitch with what `hydrostatics` predicts. It exits 0 on a pass. Try it from a
submerged start too: `spawn_z:=-1.0`.

## Joining the team's watod stack

The container uses the team's middleware, Zenoh (`RMW_IMPLEMENTATION=rmw_zenoh_cpp`). Alone, the
launch file starts its own Zenoh router and Foxglove bridge. To share topics with containers
started by `watod up`, which already run both:

1. In `.devcontainer/devcontainer.json`, add `"runArgs": ["--network=host"]` (on Docker Desktop,
   also enable host networking under Settings, Resources, Network) and rebuild the container.
2. Set `ROS_DOMAIN_ID` to watod's value (your UID % 230) before opening the container.
3. Launch with `zenoh_router:=false foxglove:=false`, and use the Foxglove address `watod up`
   prints.

## Tests

```bash
colcon test --packages-select rov_physics_sim && colcon test-result --verbose
```

The unit tests check the configs and the generated SDF without Gazebo: no overlapping volumes,
slightly positive buoyancy, a positive BG, and a valid world for every station layout.
