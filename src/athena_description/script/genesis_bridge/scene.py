from dataclasses import dataclass
from typing import Any

import genesis as gs
import numpy as np

from . import config

@dataclass
class SimHandles:
    scene: Any
    rover: Any
    left_cam: Any
    right_cam: Any
    imu: Any


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

def _add_stereo_cameras(scene, rover):
    res = (config.WIDTH, config.HEIGHT)
    left = scene.add_camera(res=res, fov=config.FOV_DEG)
    right = scene.add_camera(res=res, fov=config.FOV_DEG)
    left.attach(rover.get_link(config.LEFT_CAM_FRAME), offset_T=np.eye(4))
    right.attach(rover.get_link(config.RIGHT_CAM_FRAME), offset_T=np.eye(4))
    return left, right


def _add_imu(scene, rover):
    link = rover.get_link(config.IMU_FRAME)
    return scene.add_sensor(
        gs.sensors.IMU(entity_idx=rover.idx, link_idx_local=link.idx_local)
    )


def build_scene() -> SimHandles:
    scene = gs.Scene(
        sim_options=gs.options.SimOptions(dt=config.SIM_DT),
        show_viewer=config.SHOW_VIEWER,
    )

    _add_terrain(scene)
    rover = _add_rover(scene)
    left_cam, right_cam = _add_stereo_cameras(scene, rover)
    imu = _add_imu(scene, rover)
    scene.build()  
    return SimHandles(scene, rover, left_cam, right_cam, imu)
