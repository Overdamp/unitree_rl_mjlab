"""Head depth camera (Intel RealSense D455) for the G1-23DoF stairs student.

doc/g1_23dof_stairs_camera_plan.md part B. The camera is simulated with ray
casts in a pinhole pattern (mjlab RayCastSensor + PinholeCameraPatternCfg),
not by rendering, so it runs in the same physics step on the GPU.

The mount pose is a PLACEHOLDER: Unitree's G1 URDF puts its head camera
(d435_link) on torso_link at xyz (0.0576, 0.0175, 0.4299), pitched down
0.8308 rad (47.6 deg). Our robot carries a D455f in the head; measure the real
pose (plan step 0.2) and update D455_POS / D455_PITCH_RAD.
"""

from __future__ import annotations

import math

import mujoco
import numpy as np
import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.sensor import ObjRef, RayCastSensorCfg
from mjlab.sensor.raycast_sensor import PinholeCameraPatternCfg, RayCastSensor
from src.assets.robots.unitree_g1.g1_23dof_constants import get_spec

D455_SITE = "d455_depth"
D455_BODY = "torso_link"
D455_POS = (0.0576235, 0.01753, 0.42987)  # placeholder, Unitree URDF d435_link
D455_PITCH_RAD = 0.8307767  # placeholder, down is positive

# D455 depth: 87 x 58 deg field of view, good from ~0.6 m, invalid below ~0.4 m.
# 40 x 24 pixels gives a horizontal FOV of ~85 deg at fovy 58 (aspect 1.67).
DEPTH_WIDTH = 40
DEPTH_HEIGHT = 24
DEPTH_FOVY_DEG = 58.0
DEPTH_NEAR_M = 0.4
DEPTH_FAR_M = 3.0


def _camera_quat(pitch_rad: float) -> np.ndarray:
  """Site orientation whose -Z looks forward (+X of the body) pitched down.

  The pinhole pattern casts rays along -Z of the site, +X right, +Y image
  rows. Columns of R are the site axes in body coordinates.
  """
  base = np.array(
    [
      [0.0, 0.0, -1.0],  # x_site = -Y_body (right), y_site = +Z_body, z_site = -X_body
      [-1.0, 0.0, 0.0],
      [0.0, 1.0, 0.0],
    ]
  )
  c, s = math.cos(pitch_rad), math.sin(pitch_rad)
  pitch = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])  # about body +Y
  rot = pitch @ base
  quat = np.zeros(4)
  mujoco.mju_mat2Quat(quat, rot.flatten())
  return quat


def get_spec_with_d455() -> mujoco.MjSpec:
  """The G1-23DoF spec plus a site where the head depth camera sits."""
  spec = get_spec()
  body = spec.body(D455_BODY)
  body.add_site(
    name=D455_SITE,
    pos=D455_POS,
    quat=_camera_quat(D455_PITCH_RAD),
    size=(0.01, 0.01, 0.01),
    group=5,
  )
  return spec


def d455_sensor_cfg(name: str = "depth_cam") -> RayCastSensorCfg:
  return RayCastSensorCfg(
    name=name,
    frame=ObjRef(type="site", name=D455_SITE, entity="robot"),
    pattern=PinholeCameraPatternCfg(
      width=DEPTH_WIDTH, height=DEPTH_HEIGHT, fovy=DEPTH_FOVY_DEG
    ),
    ray_alignment="base",
    max_distance=DEPTH_FAR_M,
    exclude_parent_body=True,
  )


def depth_image(
  env: ManagerBasedRlEnv,
  sensor_name: str,
  near: float = DEPTH_NEAR_M,
  far: float = DEPTH_FAR_M,
  noise_per_m2: float = 0.01,
  dropout_prob: float = 0.02,
) -> torch.Tensor:
  """Depth image (B, H, W) in [0, 1] = distance / far, 0 = no data.

  Like a real D455: closer than `near` is invalid (0), nothing within `far`
  reads as `far` (1), the error grows with distance squared, and random pixels
  drop out (0). Row 0 is the top of the view (far), the last row the ground
  ~0.3 m ahead of the feet at the placeholder mount. With history_length = k and flatten_history_dim = False the
  group becomes (B, k, H, W): k frames as CNN channels.
  """
  sensor: RayCastSensor = env.scene[sensor_name]
  pattern = sensor.cfg.pattern
  assert isinstance(pattern, PinholeCameraPatternCfg)
  d = sensor.data.distances.clone()
  d[d < 0] = far
  d = d.clamp(max=far)
  if noise_per_m2 > 0:
    d = d + torch.randn_like(d) * noise_per_m2 * d * d
    d = d.clamp(0.0, far)
  img = d / far
  invalid = d < near
  if dropout_prob > 0:
    invalid |= torch.rand_like(d) < dropout_prob
  img[invalid] = 0.0
  # The pattern's first rows look DOWN (site +Y = body up, rays from -1 to 1);
  # flip so row 0 is the top of the view, like a real depth image.
  return img.reshape(d.shape[0], pattern.height, pattern.width).flip(1)
