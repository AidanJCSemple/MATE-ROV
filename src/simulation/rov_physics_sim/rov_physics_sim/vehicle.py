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
"""The ROV's mass properties, hydrostatics and SDF model, all from config/vehicle.yaml.

Plain Python with no ROS imports, so it can be unit tested and run anywhere.
"""

import math
from dataclasses import dataclass

import yaml

GRAVITY = 9.81


@dataclass
class Box:
    name: str
    size: tuple
    pos: tuple

    @property
    def volume(self):
        return self.size[0] * self.size[1] * self.size[2]

    @property
    def bottom(self):
        return self.pos[2] - self.size[2] / 2

    def volume_below(self, z):
        """Volume of the box below the horizontal plane at height z (vehicle upright)."""
        depth = min(max(z - self.bottom, 0.0), self.size[2])
        return self.size[0] * self.size[1] * depth

    def overlaps(self, other, tol=1e-9):
        return all(
            abs(self.pos[i] - other.pos[i]) < (self.size[i] + other.size[i]) / 2 - tol
            for i in range(3)
        )


class Vehicle:
    def __init__(self, config):
        self.config = config
        self.mass = float(config["mass"]["total"])
        self.com = tuple(config["mass"]["center"])
        self.rho = float(config["water"]["density"])
        self.boxes = [
            Box(v["name"], tuple(v["size"]), tuple(v["pos"])) for v in config["volumes"]
        ]

    @classmethod
    def from_file(cls, path):
        with open(path) as f:
            return cls(yaml.safe_load(f))

    # ---------------------------------------------------------------- mass properties

    @property
    def inertia(self):
        given = self.config["mass"].get("inertia")
        if given:
            return {
                k: float(given.get(k, 0.0))
                for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
            }
        env = self.config["envelope"]
        fill = float(self.config["mass"].get("inertia_fill", 1.0))
        a, b, c = (env["length"] * fill, env["width"] * fill, env["height"] * fill)
        m = self.mass
        return {
            "ixx": m * (b * b + c * c) / 12,
            "iyy": m * (a * a + c * c) / 12,
            "izz": m * (a * a + b * b) / 12,
            "ixy": 0.0,
            "ixz": 0.0,
            "iyz": 0.0,
        }

    # ---------------------------------------------------------------- hydrostatics

    @property
    def volume(self):
        return sum(b.volume for b in self.boxes)

    @property
    def center_of_buoyancy(self):
        v = self.volume
        return tuple(sum(b.volume * b.pos[i] for b in self.boxes) / v for i in range(3))

    @property
    def net_buoyancy(self):
        """Buoyancy minus weight when fully submerged, in newtons. Positive floats."""
        return (self.rho * self.volume - self.mass) * GRAVITY

    @property
    def reserve_buoyancy(self):
        """Net buoyancy as a fraction of weight."""
        return self.rho * self.volume / self.mass - 1.0

    @property
    def bg(self):
        """Height of the centre of buoyancy above the centre of mass (m). Must be positive."""
        return self.center_of_buoyancy[2] - self.com[2]

    def submerged_trim(self):
        """Roll and pitch (rad) at rest when fully submerged: CoB ends up directly above CoM."""
        cob = self.center_of_buoyancy
        dx, dy, dz = (cob[i] - self.com[i] for i in range(3))
        return math.atan2(dy, dz), math.atan2(-dx, dz)

    def waterline(self):
        """Height of the water surface in base_link when floating upright, or None if it sinks.

        Upright only: it ignores the small tilt a CoM/CoB offset would add.
        """
        target = self.mass / self.rho
        if self.volume <= target:
            return None
        lo = min(b.bottom for b in self.boxes)
        hi = max(b.pos[2] + b.size[2] / 2 for b in self.boxes)
        for _ in range(60):
            mid = (lo + hi) / 2
            if sum(b.volume_below(mid) for b in self.boxes) < target:
                lo = mid
            else:
                hi = mid
        return (lo + hi) / 2

    def overlapping_boxes(self):
        return [
            (a.name, b.name)
            for i, a in enumerate(self.boxes)
            for b in self.boxes[i + 1 :]
            if a.overlaps(b)
        ]

    def report(self):
        cob = self.center_of_buoyancy
        roll, pitch = self.submerged_trim()
        wl = self.waterline()
        lines = [
            f"mass                 {self.mass:.3f} kg  (weight {self.mass * GRAVITY:.1f} N)",
            f"displaced volume     {self.volume * 1000:.3f} L",
            f"net buoyancy         {self.net_buoyancy:+.2f} N  ({self.reserve_buoyancy * 100:+.1f} % of weight)",
            f"centre of mass       [{self.com[0]:+.4f}, {self.com[1]:+.4f}, {self.com[2]:+.4f}] m",
            f"centre of buoyancy   [{cob[0]:+.4f}, {cob[1]:+.4f}, {cob[2]:+.4f}] m",
            f"BG (CoB above CoM)   {self.bg * 1000:+.1f} mm",
            f"submerged trim       roll {math.degrees(roll):+.2f} deg, pitch {math.degrees(pitch):+.2f} deg",
            (
                f"floating waterline   z = {wl:+.4f} m in base_link "
                f"(base_link rests at world z = {-wl:+.4f} m)"
                if wl is not None
                else "floating waterline   none: the vehicle sinks"
            ),
        ]
        overlaps = self.overlapping_boxes()
        if overlaps:
            lines.append(f"WARNING overlapping volumes (counted twice): {overlaps}")
        if self.bg <= 0:
            lines.append(
                "WARNING the centre of buoyancy is not above the centre of mass: unstable"
            )
        return "\n".join(lines)

    # ---------------------------------------------------------------- SDF

    def sdf(
        self, name="rov", pose=(0, 0, 0, 0, 0, 0), mesh_uri=None, show_volumes=False
    ):
        """The <model> element for this vehicle, as a string."""
        inertia = self.inertia
        parts = []
        for b in self.boxes:
            geom = f"<geometry><box><size>{_v(b.size)}</size></box></geometry>"
            parts.append(
                f'<collision name="{b.name}"><pose>{_v(b.pos)} 0 0 0</pose>{geom}</collision>'
            )
            if show_volumes:
                parts.append(
                    f'<visual name="{b.name}_volume"><pose>{_v(b.pos)} 0 0 0</pose>{geom}'
                    "<transparency>0.6</transparency>"
                    "<material><ambient>0.1 0.6 1 1</ambient><diffuse>0.1 0.6 1 1</diffuse></material>"
                    "</visual>"
                )
        visual = self.config.get("visual", {})
        if mesh_uri:
            parts.append(
                '<visual name="cad"><geometry><mesh>'
                f"<uri>{mesh_uri}</uri><scale>{_v(visual.get('scale', [1, 1, 1]))}</scale>"
                "</mesh></geometry>"
                "<material><ambient>0.7 0.7 0.75 1</ambient><diffuse>0.7 0.7 0.75 1</diffuse></material>"
                "</visual>"
            )

        hydro = self.config.get("hydrodynamics", {})
        coeffs = {
            **hydro.get("linear", {}),
            **hydro.get("quadratic", {}),
            **hydro.get("added_mass", {}),
        }
        hydro_xml = "".join(f"<{k}>{float(v)}</{k}>" for k, v in coeffs.items())

        return f"""<model name="{name}">
  <pose>{_v(pose)}</pose>
  <link name="base_link">
    <inertial>
      <pose>{_v(self.com)} 0 0 0</pose>
      <mass>{self.mass}</mass>
      <inertia>{"".join(f"<{k}>{v:.6f}</{k}>" for k, v in inertia.items())}</inertia>
    </inertial>
    {"".join(parts)}
  </link>
  <plugin filename="gz-sim-hydrodynamics-system" name="gz::sim::systems::Hydrodynamics">
    <link_name>base_link</link_name>
    <water_density>{self.rho}</water_density>
    {hydro_xml}
  </plugin>
  <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
    <dimensions>3</dimensions>
    <odom_frame>world</odom_frame>
    <robot_base_frame>base_link</robot_base_frame>
    <odom_topic>/model/{name}/odometry</odom_topic>
    <tf_topic>/model/{name}/tf</tf_topic>
    <odom_publish_frequency>50</odom_publish_frequency>
  </plugin>
</model>"""


def _v(values):
    return " ".join(f"{float(x):.6g}" for x in values)
