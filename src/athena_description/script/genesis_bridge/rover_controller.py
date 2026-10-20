from steering.wheel_config import STEER_JOINTS, WHEEL_JOINTS

from . import config

class RoverController:
    def __init__(self, rover):
        self.rover = rover
        self.steer_dofs = [rover.get_joint(n).dofs_idx_local[0] for n in STEER_JOINTS]
        self.wheel_dofs = [rover.get_joint(n).dofs_idx_local[0] for n in WHEEL_JOINTS]
        self._configure_gains()

    def _configure_gains(self):
        n = len(self.steer_dofs)
        self.rover.set_dofs_kp(kp=[config.STEER_KP] * n, dofs_idx_local=self.steer_dofs)
        self.rover.set_dofs_kv(kv=[config.STEER_KV] * n, dofs_idx_local=self.steer_dofs)
        self.rover.control_dofs_position(position=[0.0] * n, dofs_idx_local=self.steer_dofs)
 
        n = len(self.wheel_dofs)
        self.rover.set_dofs_kp(kp=[config.WHEEL_KP] * n, dofs_idx_local=self.wheel_dofs)
        self.rover.set_dofs_kv(kv=[config.WHEEL_KV] * n, dofs_idx_local=self.wheel_dofs)
 
    def apply(self, steer_angles, wheel_ang_vels):
        self.rover.control_dofs_position(position=steer_angles, dofs_idx_local=self.steer_dofs)
        self.rover.control_dofs_velocity(velocity=wheel_ang_vels, dofs_idx_local=self.wheel_dofs)
 
    def pose(self):
        """Returns (pos, quat[w, x, y, z])."""
        return self.rover.get_pos(), self.rover.get_quat()
 
    def wheel_velocities(self):
        return self.rover.get_dofs_velocity(dofs_idx_local=self.wheel_dofs)
