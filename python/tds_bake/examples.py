"""Example worlds, also a starting point for new ones:

  python -m tds_bake example g1_walk > g1_walk.json

Each function returns a tds_bake.World.
"""

import math

from .world import World

G1_URDF = "unitree_g1/g1_29dof_with_hand_rev_1_0.urdf"
G1_FEET = ["left_ankle_roll_link", "right_ankle_roll_link"]

# standing pose of the legs, as in unitree_rl_gym
G1_LEG_POSE = {
    "left_hip_pitch_joint": -0.1,
    "left_knee_joint": 0.3,
    "left_ankle_pitch_joint": -0.2,
    "right_hip_pitch_joint": -0.1,
    "right_knee_joint": 0.3,
    "right_ankle_pitch_joint": -0.2,
}
# pelvis height with the legs in G1_LEG_POSE and the feet on the ground
G1_LEG_POSE_HEIGHT = 0.78

# joint gains (kp, kd) for walking, the legs as in unitree_rl_gym
G1_WALK_GAINS = {
    ".*_hip_.*": (100., 2.),
    ".*_knee_joint": (150., 4.),
    ".*_ankle_.*": (40., 2.),
    "waist_.*": (200., 5.),
    ".*_shoulder_.*": (100., 2.),
    ".*_elbow_joint": (100., 2.),
    ".*_wrist_.*": (20., 1.),
    ".*_hand_.*": (5., 0.2),
}


def g1_stand(name="g1_stand"):
  """The G1 of examples/g1_opengl_example.cpp: stiff PD (kp 1000, kd 10)
  holding the zero joint pose, dropped from 0.2 m above standing height,
  feet-only contacts, 360 Hz with 6 substeps per frame."""
  world = World(name, dt=1. / 360., substeps=6, iterations=50)
  world.add_urdf("g1", G1_URDF, position=(0., 0., 0.993), kp=1000., kd=10.,
                 armature=0.01, collision_links=G1_FEET)
  return world


# contact spheres of the G1 body (in link frames), so that it lies on the
# ground after a fall instead of sinking through it with only the feet
# colliding
G1_BODY_SPHERES = [
    ("pelvis", 0.07, (0., 0., -0.08)),
    ("torso_link", 0.1, (0.01, 0., 0.12)),
    ("torso_link", 0.09, (0.01, 0., 0.40)),
    ("left_hip_yaw_link", 0.055, (-0.03, 0., -0.1)),
    ("right_hip_yaw_link", 0.055, (-0.03, 0., -0.1)),
    ("left_knee_link", 0.05, (0.01, 0., -0.02)),
    ("right_knee_link", 0.05, (0.01, 0., -0.02)),
    ("left_knee_link", 0.045, (0.01, 0., -0.2)),
    ("right_knee_link", 0.045, (0.01, 0., -0.2)),
    ("left_shoulder_yaw_link", 0.045, (0., 0., -0.05)),
    ("right_shoulder_yaw_link", 0.045, (0., 0., -0.05)),
    ("left_elbow_link", 0.04, (0.03, 0., 0.)),
    ("right_elbow_link", 0.04, (0.03, 0., 0.)),
    ("left_wrist_yaw_link", 0.04, (0.085, 0., 0.)),
    ("right_wrist_yaw_link", 0.04, (0.085, 0., 0.)),
]


def g1_body_spheres(collide_with=("ground",)):
  return [{"link": link, "radius": radius, "position": list(position),
           "with": list(collide_with)}
          for link, radius, position in G1_BODY_SPHERES]


def g1_walk(name="g1_walk", body_spheres=False):
  """The G1 for learning to walk: 200 Hz physics, 50 Hz control (4 substeps),
  gains and standing pose of unitree_rl_gym, URDF effort limits. Only the
  feet collide with the ground, so a robot that falls over sinks through it.
  body_spheres adds 15 spheres on the pelvis, torso, legs and arms that
  collide with the ground, so that a fallen robot lies on it. They make a
  time step about 10 times slower (23 instead of 8 contacts, all of them in
  the contact problem of every step), training that ends episodes at a fall
  does not need them."""
  world = World(name, dt=0.005, substeps=4, iterations=20)
  world.add_urdf("g1", G1_URDF,
                 position=(0., 0., G1_LEG_POSE_HEIGHT),
                 joint_positions=G1_LEG_POSE,
                 gains=G1_WALK_GAINS,
                 armature=0.01,
                 effort_limits="urdf",
                 collision_links=G1_FEET,
                 collision_spheres=g1_body_spheres() if body_spheres else [])
  return world


# contact spheres of the G1 hands, for manipulation: the palms and the tips of
# the thumb, index and middle fingers (link frames of the Dex3 hand)
def g1_hand_spheres(radius=0.012, collide_with=("ball",)):
  spheres = []
  for side, sign in (("left", 1.), ("right", -1.)):
    spheres += [
        {"link": side + "_wrist_yaw_link", "radius": 0.025,
         "position": [0.09, 0.0, 0.0]},
        {"link": side + "_hand_thumb_2_link", "radius": radius,
         "position": [0.0, sign * 0.045, 0.0]},
        {"link": side + "_hand_index_1_link", "radius": radius,
         "position": [0.045, 0.0, 0.0]},
        {"link": side + "_hand_middle_1_link", "radius": radius,
         "position": [0.045, 0.0, 0.0]},
    ]
  for s in spheres:
    s["with"] = list(collide_with)
  return spheres


def g1_ball(name="g1_ball", ball_radius=0.1, ball_mass=0.3,
            ball_position=(0.38, 0., 0.85), body_spheres=False):
  """The G1 standing in front of a ball that rests on a nest of three fixed
  spheres, for manipulation: the feet collide with the ground, contact
  spheres on the hands with the ball, the ball also with the nest and the
  ground. body_spheres: see g1_walk."""
  world = World(name, dt=0.005, substeps=4, iterations=20)
  world.add_urdf("g1", G1_URDF,
                 position=(0., 0., G1_LEG_POSE_HEIGHT),
                 joint_positions=G1_LEG_POSE,
                 gains=G1_WALK_GAINS,
                 armature=0.01,
                 effort_limits="urdf",
                 collision_links=G1_FEET,
                 collision_spheres=g1_hand_spheres(collide_with=["ball"]) +
                 (g1_body_spheres() if body_spheres else []),
                 collide_with=["ground"])
  # three spheres below the ball, touching it
  nest_radius = 0.04
  x, y, z = ball_position
  spread = 0.07
  drop = math.sqrt((ball_radius + nest_radius)**2 - spread**2)
  for i in range(3):
    angle = 2. * math.pi * i / 3.
    world.add_sphere("nest%d" % i, radius=nest_radius, floating=False,
                     position=(x + spread * math.cos(angle),
                               y + spread * math.sin(angle), z - drop),
                     color=(0.5, 0.5, 0.5, 1.), collide_with=["ball"])
  world.add_sphere("ball", radius=ball_radius, mass=ball_mass,
                   position=ball_position, color=(0.9, 0.4, 0.1, 1.))
  return world


PIPER_URDF = "piper/piper_description.urdf"

# the AgileX Piper above the cube, with the open gripper pointing down: the
# joint angles for the tool point (between the fingertips) at (0.25, 0, 0.12),
# see python/examples/piper_pick.py for the inverse kinematics
PIPER_HOVER_POSE = {
    "joint1": 0.0,
    "joint2": 1.4607,
    "joint3": -1.0159,
    "joint4": 0.0,
    "joint5": 1.2126,
    "joint6": 0.0,
    "joint7": 0.035,
    "joint8": -0.035,
}
PIPER_GAINS = {
    "joint[123]": (500., 25.),
    "joint[456]": (100., 5.),
    "joint[78]": (500., 10.),
}


def piper_finger_spheres(radius=0.003, collide_with=("cube",)):
  """Contact spheres on the flat pads of the two fingers (link7, link8, which
  have the same shape): the pad is the face z = 0 of the finger frame, 26 x 11
  mm at the tip of the finger, so two spheres touching it."""
  spheres = []
  for link in ("link7", "link8"):
    for x in (-0.0065, 0.0065):
      spheres.append({"link": link, "radius": radius,
                      "position": [x, -0.0055, radius],
                      "with": list(collide_with)})
  return spheres


def piper_pick(name="piper_pick", cube_size=0.03, cube_mass=0.05,
               cube_position=(0.25, 0.)):
  """The AgileX Piper arm with its gripper (data/piper, from piper_ros,
  converted by convert_piper_urdf.py) on the ground, above a cube that rests
  on it, to pick it up: 240 Hz physics, 60 Hz control (4 substeps), PD joint
  control with the URDF effort limits, the gripper open and pointing down. Only
  the pads of the fingers collide (two contact spheres on each, with the cube)
  and the cube with the ground. python/examples/piper_pick.py moves the arm:
  down, close the gripper, up."""
  world = World(name, dt=1. / 240., substeps=4, iterations=30)
  world.add_urdf("piper", PIPER_URDF,
                 floating=False,
                 joint_positions=PIPER_HOVER_POSE,
                 gains=PIPER_GAINS,
                 armature=0.01,
                 effort_limits="urdf",
                 collision_links="none",
                 collision_spheres=piper_finger_spheres())
  world.add_box("cube", size=(cube_size,) * 3, mass=cube_mass,
                position=(cube_position[0], cube_position[1], cube_size / 2),
                color=(0.9, 0.4, 0.1, 1.), friction=1.0)
  return world


LAIKAGO_URDF = "laikago/laikago_toes_zup.urdf"

# standing pose of the Laikago, as examples/environments/laikago_environment.h:
# abduction 0.2, hip 0, knee -0.7 on every leg, the base 0.48 m above the
# ground
LAIKAGO_POSE = {}
for _leg in ("FR", "FL", "RR", "RL"):
  LAIKAGO_POSE[_leg + "_hip_motor_2_chassis_joint"] = 0.2
  LAIKAGO_POSE[_leg + "_upper_leg_2_hip_motor_joint"] = 0.
  LAIKAGO_POSE[_leg + "_lower_leg_2_upper_leg_joint"] = -0.7
LAIKAGO_POSE_HEIGHT = 0.48


# Worlds with the solver "external": the kernels compute the robots' dynamics,
# and another engine finds and solves the contacts of their links (the
# collision settings of a body say which links collide, here all).


def g1_external(name="g1_external", dt=1. / 360., substeps=6):
  """The G1 of g1_stand (kp 1000, kd 10, 360 Hz, 6 substeps per frame) with
  the contacts of all its links solved by another engine."""
  world = World(name, dt=dt, substeps=substeps, solver="external")
  world.add_urdf("g1", G1_URDF, position=(0., 0., 0.793), kp=1000., kd=10.,
                 armature=0.01, collision_links="all")
  return world


def laikago_external(name="laikago_external", kp=200., kd=4.):
  """The Laikago quadruped standing (LAIKAGO_POSE) with PD joint control, the
  contacts of all its links solved by another engine, 240 Hz with 4 substeps
  per frame."""
  world = World(name, dt=1. / 240., substeps=4, solver="external")
  world.add_urdf("laikago", LAIKAGO_URDF,
                 position=(0., 0., LAIKAGO_POSE_HEIGHT),
                 joint_positions=LAIKAGO_POSE, kp=kp, kd=kd, armature=0.01,
                 collision_links="all")
  return world


def piper_external(name="piper_external"):
  """The Piper of piper_pick, hovering above where the cube goes, with the
  contacts of all its links solved by another engine (the URDF has no
  collision shapes: the engine can use the visual meshes)."""
  world = World(name, dt=1. / 240., substeps=4, solver="external")
  world.add_urdf("piper", PIPER_URDF,
                 floating=False,
                 joint_positions=PIPER_HOVER_POSE,
                 gains=PIPER_GAINS,
                 armature=0.01,
                 effort_limits="urdf",
                 collision_links="all")
  return world


EXAMPLES = {
    "g1_stand": g1_stand,
    "g1_walk": g1_walk,
    "g1_walk_body": lambda: g1_walk("g1_walk_body", body_spheres=True),
    "g1_ball": g1_ball,
    "piper_pick": piper_pick,
    "g1_external": g1_external,
    "laikago_external": laikago_external,
    "piper_external": piper_external,
}
