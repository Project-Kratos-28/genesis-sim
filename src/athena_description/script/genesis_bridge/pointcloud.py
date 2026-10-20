import numpy as np
 
from . import config
 
 
def depth_to_pointcloud(depth, model, stride=config.POINTCLOUD_STRIDE):
    h, w = depth.shape
    us, vs = np.meshgrid(np.arange(0, w, stride), np.arange(0, h, stride))
    z = depth[vs, us]
    valid = (z > config.DEPTH_MIN) & (z < config.DEPTH_MAX) & np.isfinite(z)
    # The original double negation on x cancels out; this is the equivalent form.
    x = (us - model.cx) * z / model.fx
    y = -(vs - model.cy) * z / model.fy
    return np.stack([x[valid], y[valid], z[valid]], axis=-1).astype(np.float32)
 