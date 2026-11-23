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
"""Checks on the vehicle and pool configs and the SDF generated from them. No Gazebo needed."""

import math
import os
import xml.etree.ElementTree as ET

import pytest
from rov_physics_sim import world
from rov_physics_sim.vehicle import Box, Vehicle

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VEHICLE = os.path.join(PKG, "config", "vehicle.yaml")
POOL = os.path.join(PKG, "config", "pool.yaml")


@pytest.fixture
def vehicle():
    return Vehicle.from_file(VEHICLE)


@pytest.fixture
def pool():
    return world.load(POOL)


def test_volumes_do_not_overlap(vehicle):
    assert vehicle.overlapping_boxes() == []


def test_volumes_fit_in_envelope(vehicle):
    env = vehicle.config["envelope"]
    half = (env["length"] / 2, env["width"] / 2, env["height"] / 2)
    for b in vehicle.boxes:
        for i in range(3):
            assert abs(b.pos[i]) + b.size[i] / 2 <= half[i] + 1e-6, b.name


def test_slightly_positive_and_stable(vehicle):
    assert 0.005 <= vehicle.reserve_buoyancy <= 0.05  # floats, but not by much
    assert vehicle.bg > 0.01  # self-righting


def test_waterline_balances_weight(vehicle):
    z = vehicle.waterline()
    submerged = sum(b.volume_below(z) for b in vehicle.boxes)
    assert submerged * vehicle.rho == pytest.approx(vehicle.mass, rel=1e-6)


def test_sinks_without_waterline():
    cfg = {
        "mass": {"total": 2.0, "center": [0, 0, 0]},
        "water": {"density": 1000.0},
        "volumes": [{"name": "b", "size": [0.1, 0.1, 0.1], "pos": [0, 0, 0]}],
    }
    assert Vehicle(cfg).waterline() is None


def test_trim_follows_cob_offset():
    cfg = {
        "mass": {"total": 1.0, "center": [0, 0, 0]},
        "water": {"density": 1000.0},
        "volumes": [{"name": "b", "size": [0.1, 0.1, 0.1], "pos": [0.01, 0, 0.1]}],
    }
    roll, pitch = Vehicle(cfg).submerged_trim()
    assert roll == pytest.approx(0)
    assert pitch == pytest.approx(
        -math.atan2(0.01, 0.1)
    )  # buoyancy forward lifts the nose


def test_box_overlap():
    a = Box("a", (1, 1, 1), (0, 0, 0))
    assert a.overlaps(Box("b", (1, 1, 1), (0.9, 0, 0)))
    assert not a.overlaps(Box("c", (1, 1, 1), (1.0, 0, 0)))  # touching is fine


@pytest.mark.parametrize("station", world.STATIONS)
def test_world_is_valid_sdf(vehicle, pool, station):
    sdf = world.world_sdf(
        pool,
        vehicle.sdf(pose=world.spawn_pose(pool), mesh_uri="file:///x.stl"),
        station,
    )
    root = ET.fromstring(sdf)
    w = root.find("world")
    models = {m.get("name") for m in w.findall("model")}
    assert {"pool", "rov"} <= models
    assert ("station" in models) == (station != "no_wall")
    plugins = {p.get("name") for p in w.iter("plugin")}
    assert "gz::sim::systems::Buoyancy" in plugins
    assert "gz::sim::systems::Hydrodynamics" in plugins


def test_spawn_is_inside_pool_facing_in(pool):
    x, y, _, _, _, yaw = world.spawn_pose(pool)
    assert abs(x) < pool["pool"]["length"] / 2 and abs(y) < pool["pool"]["width"] / 2
    assert x == pytest.approx(
        -pool["pool"]["length"] / 2 + pool["spawn"]["standoff"]
    )  # west wall
    assert yaw == pytest.approx(0.0)


def test_sidewalls_on_correct_side(pool):
    left = ET.fromstring(world.station_sdf(pool, "wall_left"))
    pose = left.find(".//collision/pose").text.split()
    assert float(pose[1]) > 0  # facing +x from the west wall, left is +y
    right = ET.fromstring(world.station_sdf(pool, "wall_right"))
    assert float(right.find(".//collision/pose").text.split()[1]) < 0


def test_unknown_station_rejected(pool):
    with pytest.raises(ValueError):
        world.station_sdf(pool, "middle")
