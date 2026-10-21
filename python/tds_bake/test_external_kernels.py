"""Checks the kernels of worlds with the solver "external"
(src/bake/world_model.hpp) on the CPU in double precision:

  dof_axis   the link velocities from the world motion vectors of the degrees
             of freedom match finite differences of the link poses
  factor     M^-1 from the factorization M = L^T D L (CRBA) matches the
             response of the forward dynamics (ABA) to joint torques
  advance    integrates the positions with the given velocities

  python -m tds_bake.test_external_kernels      (or python -m unittest)
"""

import unittest

import numpy as np

import tds_bake
from tds_bake import examples


def quat_mul(a, b):
  ax, ay, az, aw = a
  bx, by, bz, bw = b
  return np.array([aw * bx + ax * bw + ay * bz - az * by,
                   aw * by - ax * bz + ay * bw + az * bx,
                   aw * bz + ax * by - ay * bx + az * bw,
                   aw * bw - ax * bx - ay * by - az * bz])


def link_twists(body, dof_axis, qd, links):
  """Angular velocity and velocity of the link origins from dof_axis."""
  axes = dof_axis.reshape(-1, 6)
  w_out, v_out = [], []
  for l, dofs in enumerate(body.link_dofs):
    p = links[l, :3]
    w = np.zeros(3)
    v = np.zeros(3)
    for d in dofs:
      a = axes[body.velocity_offset + d]
      q = qd[d]
      w += a[:3] * q
      v += (a[3:] + np.cross(a[:3], p)) * q
    w_out.append(w)
    v_out.append(v)
  return np.array(w_out), np.array(v_out)


def factor_inverse(body, factor):
  """M^-1 from the factorization of a body, column by column."""
  n = len(body.parent)
  f = factor[body.factor_offset:body.factor_offset + body.factor_size]
  out = np.zeros((n, n))
  for col in range(n):
    x = np.zeros(n)
    x[col] = 1.
    for i in range(n - 1, -1, -1):
      o = body.factor_index[i] + 1
      j = body.parent[i]
      while j >= 0:
        x[j] -= f[o] * x[i]
        o += 1
        j = body.parent[j]
    for i in range(n):
      x[i] /= f[body.factor_index[i]]
    for i in range(n):
      o = body.factor_index[i] + 1
      j = body.parent[i]
      while j >= 0:
        x[i] -= f[o] * x[j]
        o += 1
        j = body.parent[j]
    out[:, col] = x
  return out


def make_world(name, urdf, floating, position, control="torque"):
  world = tds_bake.World(name, dt=1. / 240., substeps=1, solver="external")
  world.add_urdf("robot", urdf, floating=floating, position=position,
                 control=control, kp=100., kd=2., armature=0.01)
  return world


ROBOTS = [
    ("external_test_g1", examples.G1_URDF, True, (0., 0., 1.)),
    ("external_test_laikago", examples.LAIKAGO_URDF, True, (0., 0., 0.6)),
    ("external_test_piper", examples.PIPER_URDF, False, (0., 0., 0.)),
]


class ExternalKernelsTest(unittest.TestCase):

  def run_robot(self, name, urdf, floating, position):
    baked = tds_bake.bake(make_world(name, urdf, floating, position),
                          backends=("cpu",), precisions=("double",),
                          verbose=False)
    self.assertEqual(baked.solver, "external")
    body = baked.body("robot")
    batch = baked.batch(1, backend="cpu", precision="double")
    rng = np.random.default_rng(1)
    state = baked.default_state()
    nq = body.q.stop - body.q.start
    nqd = body.qd.stop - body.qd.start
    # a random pose and velocity
    joints = body.joint_q
    state[joints] += rng.uniform(-0.3, 0.3, len(joints))
    if floating:
      axis = rng.normal(size=3)
      axis /= np.linalg.norm(axis)
      angle = 0.4
      state[body.base_orientation] = np.concatenate(
          [np.sin(angle / 2) * axis, [np.cos(angle / 2)]])
    qd = rng.uniform(-1., 1., nqd)
    state[body.qd] = qd
    batch.set("state", state)

    # dof_axis against finite differences of the link poses
    batch._batch.launch("articulation")
    dof_axis = batch.get("dof_axis")[0]
    factor = batch.get("factor")[0]
    batch.set("velocity", qd)
    batch._batch.launch("advance")
    links0 = batch.get("links")[0].reshape(-1, 7)  # after one step of dt
    batch.set("state", state)
    links_start = batch.links()[0]
    w, v = link_twists(body, dof_axis, qd, links_start)
    v_fd = (links0[:, :3] - links_start[:, :3]) / baked.dt
    np.testing.assert_allclose(v, v_fd, atol=2e-2 * (1 + np.abs(v).max()))
    for l in range(len(links_start)):
      dq = quat_mul(links0[l, 3:], links_start[l, 3:] * [-1, -1, -1, 1])
      if dq[3] < 0:
        dq = -dq
      w_fd = 2. * dq[:3] / baked.dt
      np.testing.assert_allclose(w[l], w_fd, atol=2e-2 * (1 + np.abs(w).max()))
    # exactly at the integrated velocity: advance integrates with qd
    next_state = batch.get("next_state")[0]
    np.testing.assert_allclose(next_state[body.qd], qd)
    if body.joint_q.size:
      np.testing.assert_allclose(
          next_state[body.joint_q],
          state[body.joint_q] + baked.dt * qd[body.joint_qd - body.qd.start])

    # M^-1 from the factor against the forward dynamics' response to torques
    minv = factor_inverse(body, factor)
    np.testing.assert_allclose(minv, minv.T, atol=1e-9 * np.abs(minv).max())
    self.assertGreater(np.linalg.eigvalsh(minv).min(), 0.)
    control = np.zeros(baked.control_dim)
    batch.set("control", control)
    batch._batch.launch("predict")
    base = batch.get("velocity")[0]
    for a in range(0, body.num_actuators, max(1, body.num_actuators // 7)):
      control[:] = 0.
      control[body.torques.start + a] = 1.
      batch.set("control", control)
      batch._batch.launch("predict")
      column = (batch.get("velocity")[0] - base) / baked.dt
      dof = body.actuator_qd[a] - body.qd.start
      np.testing.assert_allclose(column, minv[:, dof],
                                 atol=1e-6 * np.abs(minv).max())
    return baked

  def test_g1(self):
    self.run_robot(*ROBOTS[0])

  def test_laikago(self):
    self.run_robot(*ROBOTS[1])

  def test_piper(self):
    self.run_robot(*ROBOTS[2])


if __name__ == "__main__":
  unittest.main()
