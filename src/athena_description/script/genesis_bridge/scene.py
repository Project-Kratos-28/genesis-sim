from dataclasses import dataclass
from typing import Any

import genesis as gs

from . import config

@dataclass
class SimHandles:
    scene: Any
    rover: Any
    sensors: Any = None


def _add_terrain(scene):
    scene.add_entity(
        morph=gs.morphs.Mesh(
            file=config.TERRAIN_PATH,
            pos=config.TERRAIN_POS,
            euler=(0.0, 0.0, 0.0),
            scale=1.0,
            fixed=True,
            convexify=False,
        ),
        material=gs.materials.Rigid(friction=config.TERRAIN_FRICTION),
    )


def _add_rover(scene):
    return scene.add_entity(
        gs.morphs.URDF(
            file=config.URDF_PATH,
            fixed=False,
            pos=(10.0, 0.0, config.SPAWN_Z),
            merge_fixed_links=True,
            links_to_keep=config.LINKS_TO_KEEP,
        )
    )


def _add_visual_landmarks(scene, rover_xy):
    """Static, non-colliding boulders used as visual features by the cameras.

    The terrain mesh has no UVs, so texturing it is impossible; without any
    structure the stereo odometry has nothing to track. These boulders are
    visual only for the physics (`collision=False`, `fixed=True`) and so leave
    the dynamics untouched.

    The material also opts into `use_visual_raycasting`, which adds the merged
    boulder mesh to Genesis' visual raycast BVH. Without it the simulated Livox
    would only ever see the collision geometry (terrain + rover) and the rocks
    would be missing from `/livox/lidar`; the flag changes nothing for physics
    or camera rendering. The mesh is static, so that BVH is built once and is
    not rebuilt per step.
    """
    import numpy as np
    import trimesh

    rng = np.random.default_rng(config.LANDMARK_SEED)
    terrain = trimesh.load(config.TERRAIN_PATH, force="mesh")
    lo, hi = terrain.bounds[0][:2], terrain.bounds[1][:2]
    margin = 1.0

    local, radii = [], []
    while len(local) < config.LANDMARK_COUNT:
        n = config.LANDMARK_COUNT * 2
        lx = rng.uniform(lo[0] + margin, hi[0] - margin, n)
        ly = rng.uniform(lo[1] + margin, hi[1] - margin, n)
        rs = rng.uniform(*config.LANDMARK_RADIUS, n)
        keep = np.hypot(
            lx + config.TERRAIN_POS[0] - rover_xy[0],
            ly + config.TERRAIN_POS[1] - rover_xy[1],
        ) > config.LANDMARK_EXCLUSION
        local.extend(zip(lx[keep], ly[keep]))
        radii.extend(rs[keep])

    local = np.asarray(local[: config.LANDMARK_COUNT])
    radii = np.asarray(radii[: config.LANDMARK_COUNT])

    local_positions, local_radii = [], []

    # Cast down in terrain-local coordinates to find each boulder's ground height.
    origins = np.column_stack([local[:, 0], local[:, 1], np.full(len(local), 100.0)])
    hits, ray_idx, _ = terrain.ray.intersects_location(
        origins, np.tile([0.0, 0.0, -1.0], (len(local), 1)), multiple_hits=False
    )
    ground = np.full(len(local), np.nan)
    ground[ray_idx] = hits[:, 2]
    valid = np.isfinite(ground)

    for (lx, ly), lz, r in zip(local[valid], ground[valid], radii[valid]):
        local_positions.append((lx + config.TERRAIN_POS[0],
                                ly + config.TERRAIN_POS[1],
                                lz + config.TERRAIN_POS[2] + 0.45 * r))
        local_radii.append(r)

    # One merged visual mesh instead of ~160 entities: the rasterizer re-uploads
    # the scene graph on every sensor read, so entity count directly costs
    # rendering time. Genesis only accepts files for `morphs.Mesh`, so the merged
    # mesh is exported to a temporary STL.
    import os
    import tempfile

    boulders = trimesh.util.concatenate([
        trimesh.creation.icosphere(subdivisions=1, radius=r).apply_translation(pos)
        for pos, r in zip(local_positions, local_radii)
    ])
    stl_path = os.path.join(tempfile.gettempdir(), "athena_visual_landmarks.stl")
    boulders.export(stl_path)
    # `Kinematic` (rather than `Rigid`) keeps these visual-only boulders in the
    # kinematic solver: every link there is fixed, so Genesis treats the visual
    # raycast BVH as static and builds it once. A `Rigid` material would put them
    # in the rover's rigid solver, whose movable links force that (large) visual
    # BVH to rebuild every physics step.
    scene.add_entity(
        gs.morphs.Mesh(file=stl_path, fixed=True, collision=False),
        material=gs.materials.Kinematic(use_visual_raycasting=True),
        surface=gs.surfaces.Rough(color=(0.33, 0.27, 0.22)),
    )


def build_scene(sensor_factory=None, show_viewer=None) -> SimHandles:
    """Build terrain + rover.

    `sensor_factory(scene, rover)` runs after the entities are added and before
    `scene.build()`, which is the only window in which Genesis sensors can be
    created. Its return value is stashed on `SimHandles.sensors`.
    """
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=config.SIM_DT),
        show_viewer=config.SHOW_VIEWER if show_viewer is None else show_viewer,
    )

    _add_terrain(scene)
    rover = _add_rover(scene)
    if config.VISUAL_LANDMARKS:
        _add_visual_landmarks(scene, rover_xy=(10.0, 0.0))
    sensors = sensor_factory(scene, rover) if sensor_factory else None
    scene.build()  
    return SimHandles(scene, rover, sensors)
