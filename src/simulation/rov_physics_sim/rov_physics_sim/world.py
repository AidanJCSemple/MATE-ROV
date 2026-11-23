# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""The pool world, its launch station and the ROV's spawn pose, from config/pool.yaml.

Plain Python with no ROS imports, so it can be unit tested and run anywhere.
"""

import math

import yaml

STATIONS = ("no_wall", "wall_left", "wall_right", "both_walls")

# Inward normal of each pool wall (pointing into the pool).
_WALL_NORMALS = {
    "west": (1.0, 0.0),
    "east": (-1.0, 0.0),
    "south": (0.0, 1.0),
    "north": (0.0, -1.0),
}


def load(path):
    with open(path) as f:
        return yaml.safe_load(f)


def _box(name, size, pos, yaw=0.0, color="0.8 0.85 0.9 1"):
    geom = f"<geometry><box><size>{_v(size)}</size></box></geometry>"
    pose = f"<pose>{_v(pos)} 0 0 {yaw:.6g}</pose>"
    return (
        f'<collision name="{name}_collision">{pose}{geom}</collision>'
        f'<visual name="{name}_visual">{pose}{geom}'
        f"<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>"
    )


def station_frame(config):
    """The station's origin on the inner face of its wall, its inward normal and its left."""
    pool, st = config["pool"], config["station"]
    nx, ny = _WALL_NORMALS[st["wall"]]
    half = pool["length"] / 2 if nx else pool["width"] / 2
    left = (-ny, nx)  # facing into the pool
    origin = (-nx * half + left[0] * st["offset"], -ny * half + left[1] * st["offset"])
    return origin, (nx, ny), left


def spawn_pose(config, z=None):
    """x, y, z, roll, pitch, yaw of the ROV at the station, facing into the pool."""
    (ox, oy), (nx, ny), _ = station_frame(config)
    d = config["spawn"]["standoff"]
    z = config["spawn"]["z"] if z is None else z
    return (ox + nx * d, oy + ny * d, z, 0.0, 0.0, math.atan2(ny, nx))


def pool_sdf(config):
    p = config["pool"]
    length, width, depth, t, fb = (
        p["length"],
        p["width"],
        p["depth"],
        p["wall_thickness"],
        p["freeboard"],
    )
    h = depth + fb
    zc = fb - h / 2
    body = "".join(
        [
            _box(
                "floor",
                (length + 2 * t, width + 2 * t, t),
                (0, 0, -depth - t / 2),
                color="0.6 0.7 0.8 1",
            ),
            _box("wall_west", (t, width + 2 * t, h), (-(length + t) / 2, 0, zc)),
            _box("wall_east", (t, width + 2 * t, h), ((length + t) / 2, 0, zc)),
            _box("wall_south", (length, t, h), (0, -(width + t) / 2, zc)),
            _box("wall_north", (length, t, h), (0, (width + t) / 2, zc)),
        ]
    )
    # The water surface, for the Gazebo GUI only: no collision, and buoyancy ignores it.
    water = (
        '<visual name="water_surface"><pose>0 0 0 0 0 0</pose>'
        f"<geometry><plane><normal>0 0 1</normal><size>{length} {width}</size></plane></geometry>"
        "<transparency>0.7</transparency>"
        "<material><ambient>0.1 0.4 0.8 1</ambient><diffuse>0.1 0.4 0.8 1</diffuse></material></visual>"
    )
    return f'<model name="pool"><static>true</static><link name="pool">{body}{water}</link></model>'


def station_sdf(config, station):
    if station not in STATIONS:
        raise ValueError(f"station must be one of {STATIONS}, not {station!r}")
    st, pool = config["station"], config["pool"]
    (ox, oy), (nx, ny), (lx, ly) = station_frame(config)
    sides = {
        "no_wall": (),
        "wall_left": (1,),
        "wall_right": (-1,),
        "both_walls": (1, -1),
    }[station]
    if not sides:
        return ""
    depth = (
        pool["depth"] if st["sidewall_depth"] == "full" else float(st["sidewall_depth"])
    )
    top = pool["freeboard"]
    length, thick = st["sidewall_length"], st["sidewall_thickness"]
    yaw = math.atan2(ny, nx)
    boxes = []
    for s in sides:
        along = s * st["width"] / 2
        cx = ox + lx * along + nx * length / 2
        cy = oy + ly * along + ny * length / 2
        name = "sidewall_left" if s > 0 else "sidewall_right"
        boxes.append(
            _box(
                name,
                (length, thick, top + depth),
                (cx, cy, (top - depth) / 2),
                yaw,
                color="0.9 0.6 0.2 1",
            )
        )
    return f'<model name="station"><static>true</static><link name="station">{"".join(boxes)}</link></model>'


def world_sdf(
    pool_config, vehicle_model_sdf, station="no_wall", water_density=1000.0, step=0.001
):
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="rov_world">
    <physics name="default" type="dart">
      <max_step_size>{step}</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.81</gravity>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <!-- Water below z = 0, air above, so the ROV can float at the surface. -->
    <plugin filename="gz-sim-buoyancy-system" name="gz::sim::systems::Buoyancy">
      <graded_buoyancy>
        <default_density>{water_density}</default_density>
        <density_change>
          <above_depth>0</above_depth>
          <density>1.225</density>
        </density_change>
      </graded_buoyancy>
    </plugin>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <direction>-0.5 0.1 -0.9</direction>
      <diffuse>0.8 0.8 0.8 1</diffuse>
    </light>
    {pool_sdf(pool_config)}
    {station_sdf(pool_config, station)}
    {vehicle_model_sdf}
  </world>
</sdf>
"""


def _v(values):
    return " ".join(f"{float(x):.6g}" for x in values)
