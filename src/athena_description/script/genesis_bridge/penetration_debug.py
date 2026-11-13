"""Wheel-vs-terrain penetration diagnostics.

Every ``PENETRATION_DEBUG_EVERY`` physics steps we report, per wheel:

* ``pen`` — the deepest penetration in Genesis' own contact set for that wheel
  against the terrain, straight from the physics solver. Positive = overlap.
  This is the authoritative "is the wheel inside the terrain?" answer.
* ``vis`` — an independent, solver-free cross-check: the wheel centre's height
  above the *visual* terrain STL directly below it, minus the wheel radius.
  Negative means the wheel centre is closer to the rendered ground than a
  tangent wheel, i.e. the wheel looks sunk. It also catches a wheel that
  tunneled through without generating a contact. Read it with care: Genesis
  collides mesh-vs-mesh through an SDF of the terrain whose horizontal cell is
  ~0.28 m (its 128-node budget spread over a 30 m mesh), while the camera
  renders the raw STL. Around facet edges the SDF surface is up to ~3 cm below
  the rendered one, so ``vis`` legitimately reads -20..-35 mm while ``pen`` is
  ~0.1 mm. Only a drop well beyond that (or ``pen`` > ``PENETRATION_MAX_ALLOWED``)
  means the physics itself is wrong.
* ``f`` — net contact force on the wheel (vector sum over its contacts) and
  ``f1`` the largest single-contact force. ``f1`` matters: a sum of magnitudes
  over a 20-point manifold can read tens of kN while the net force is a few
  hundred N.
* ``nz`` — vertical component of the deepest contact's normal (near 1 = the
  wheel is being pushed up from below, low = an edge/side hit).

A one-line worst-case summary is logged at shutdown.

This module is diagnostic only: it never writes to the simulation.
"""

import numpy as np
import trimesh

from steering.wheel_config import WHEEL_RADIUS

from . import config

WHEEL_LINKS = [
    "wheel_front_left",
    "wheel_front_right",
    "wheel_rear_left",
    "wheel_rear_right",
]


class WheelPenetrationMonitor:
    def __init__(self, rover, terrain_entity):
        self.rover = rover
        self.terrain_entity = terrain_entity
        self.links = [rover.get_link(name) for name in WHEEL_LINKS]
        # Reuse the same STL the physics uses so the raycast surface matches the
        # collision surface (the terrain has no rotation or scale in scene.py).
        self.terrain = trimesh.load(config.TERRAIN_PATH, force="mesh")
        self.terrain_xyz = np.asarray(config.TERRAIN_POS, dtype=float)
        self._step = 0
        # Running worst case, reported once at shutdown.
        self._samples = 0
        self._worst_pen = np.zeros(len(WHEEL_LINKS))
        self._worst_vis = np.zeros(len(WHEEL_LINKS))
        self._worst_force = np.zeros(len(WHEEL_LINKS))
        self._samples_over = np.zeros(len(WHEEL_LINKS), dtype=int)

    def _terrain_z(self, xy):
        """World terrain height under each (x, y), NaN where there is no hit."""
        origins = np.column_stack(
            (
                xy[:, 0] - self.terrain_xyz[0],
                xy[:, 1] - self.terrain_xyz[1],
                np.full(len(xy), 100.0),
            )
        )
        directions = np.tile([0.0, 0.0, -1.0], (len(xy), 1))
        hits, ray_idx, _ = self.terrain.ray.intersects_location(
            origins, directions, multiple_hits=False
        )
        ground = np.full(len(xy), np.nan)
        ground[ray_idx] = hits[:, 2] + self.terrain_xyz[2]
        return ground

    def _solver_contacts(self):
        """Per-wheel (deepest penetration [m], n, net force [N], max force [N], nz)."""
        data = self.rover.get_contacts(
            with_entity=self.terrain_entity, exclude_self_contact=True
        )
        no_contacts = [(0.0, 0, 0.0, 0.0, 0.0) for _ in self.links]
        penetration = data.get("penetration")
        if penetration is None or len(penetration) == 0:
            return no_contacts

        pen = penetration.cpu().numpy()
        link_a = data["link_a"].cpu().numpy()
        link_b = data["link_b"].cpu().numpy()
        # Genesis stores an equal-and-opposite pair per contact (force_a = -force,
        # force_b = +force), so summing the two is always zero. Pick the vector
        # that acts on the wheel, then sum vectors (not magnitudes).
        force_a = data["force_a"].cpu().numpy()
        force_b = data["force_b"].cpu().numpy()
        normal_z = data["normal"].cpu().numpy()[:, 2]

        out = []
        for link in self.links:
            mask = (link_a == link.idx) | (link_b == link.idx)
            if not mask.any():
                out.append((0.0, 0, 0.0, 0.0, 0.0))
                continue
            acting = np.where(
                (link_b[mask] == link.idx)[:, None], force_b[mask], force_a[mask]
            )
            deepest = int(np.argmax(pen[mask]))
            out.append(
                (
                    float(pen[mask].max()),
                    int(mask.sum()),
                    float(np.linalg.norm(acting.sum(axis=0))),
                    float(np.linalg.norm(acting, axis=1).max()),
                    abs(float(normal_z[mask][deepest])),
                )
            )
        return out

    def update(self, logger):
        self._step += 1
        if not config.PENETRATION_DEBUG:
            return
        if self._step % config.PENETRATION_DEBUG_EVERY:
            return

        centers = np.array([link.get_pos().cpu().numpy() for link in self.links])
        ground = self._terrain_z(centers[:, :2])
        # Centre-height-above-the-surface minus radius: < 0 means sunk.
        clearance = centers[:, 2] - ground - WHEEL_RADIUS
        solver = self._solver_contacts()

        # Body position/attitude, so a wheel that leaves the ground because the
        # rover is tipping over is not mistaken for penetration, and so a logged
        # event can be located on the map.
        body_pos = self.rover.get_pos().cpu().numpy()
        body_z = float(body_pos[2])
        w, x, y, z = self.rover.get_quat().cpu().numpy()
        roll = np.degrees(np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))
        pitch = np.degrees(np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0)))

        parts = []
        for i, name in enumerate(WHEEL_LINKS):
            pen, n_contacts, net_force, max_force, nz = solver[i]
            sink = -clearance[i]
            self._worst_pen[i] = max(self._worst_pen[i], pen)
            self._worst_vis[i] = max(self._worst_vis[i], sink)
            self._worst_force[i] = max(self._worst_force[i], net_force)
            if pen > config.PENETRATION_MAX_ALLOWED:
                self._samples_over[i] += 1
            parts.append(
                f"{name.replace('wheel_', '')}: "
                f"pen={pen * 1000.0:+.1f}mm (n={n_contacts}) "
                f"vis={clearance[i] * 1000.0:+.1f}mm "
                f"f={net_force:.0f}/{max_force:.0f}N nz={nz:.2f}"
            )
        self._samples += 1

        deepest = max(c[0] for c in solver)
        # One call site only: rclpy rejects changing the severity of a logger
        # call from the same source line, so the depth is reported in the tag.
        tag = "PENETRATION" if deepest > config.PENETRATION_MAX_ALLOWED else "penetration"
        logger.info(
            f"{tag} | pos=({body_pos[0]:.2f}, {body_pos[1]:.2f}) z={body_z:.3f}m "
            f"roll={roll:+.1f}deg pitch={pitch:+.1f}deg "
            f"| deepest={deepest * 1000.0:+.1f}mm | " + " | ".join(parts)
        )

    def log_summary(self, logger):
        """Log the worst values seen over the whole session (safe to call once)."""
        if not config.PENETRATION_DEBUG or not self._samples:
            return
        wheels = ", ".join(
            f"{name.replace('wheel_', '')}={pen * 1000.0:+.1f}mm "
            f"({over} samples >{config.PENETRATION_MAX_ALLOWED * 1000.0:.0f}mm)/"
            f"vis {vis * 1000.0:+.1f}mm/f {force:.0f}N"
            for name, pen, vis, force, over in zip(
                WHEEL_LINKS,
                self._worst_pen,
                self._worst_vis,
                self._worst_force,
                self._samples_over,
            )
        )
        logger.info(
            f"penetration summary | {self._samples} samples | worst {wheels}"
        )
