
from __future__ import absolute_import
from __future__ import division
#from __future__ import google_type_annotations
from __future__ import print_function

import os
import inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
os.sys.path.insert(0, parentdir)

from absl import app
from absl import flags
import scipy.interpolate
import numpy as np

import time
import pybullet
import random

import com_velocity_estimator
import gait_generator as gait_generator_lib
import locomotion_controller
import openloop_gait_generator
import raibert_swing_leg_controller
import torque_stance_leg_controller

#uncomment the robot of choice
import laikago_tds_sim_clean as robot_sim

FLAGS = flags.FLAGS


_NUM_SIMULATION_ITERATION_STEPS = 300


_STANCE_DURATION_SECONDS = [
    0.3
] * 4  # For faster trotting (v > 1.5 ms reduce this to 0.13s).

# Standing
#_DUTY_FACTOR = [1.] * 4
#_INIT_PHASE_FULL_CYCLE = [0., 0., 0., 0.]
#_MAX_TIME_SECONDS = 50
#_INIT_LEG_STATE = (
#    gait_generator_lib.LegState.STANCE,
#    gait_generator_lib.LegState.STANCE,
#    gait_generator_lib.LegState.STANCE,
#    gait_generator_lib.LegState.STANCE,
#)

#Tripod
#_DUTY_FACTOR = [.8] * 4
#_INIT_PHASE_FULL_CYCLE = [0., 0.25, 0.5, 0.]
#_MAX_TIME_SECONDS = 5
#
#_INIT_LEG_STATE = (
#     gait_generator_lib.LegState.STANCE,
#     gait_generator_lib.LegState.STANCE,
#     gait_generator_lib.LegState.STANCE,
#     gait_generator_lib.LegState.SWING,
#)

# Trotting
_DUTY_FACTOR = [0.6] * 4
_INIT_PHASE_FULL_CYCLE = [0.9, 0, 0, 0.9]
#_INIT_PHASE_FULL_CYCLE = [1, 0, 0, 1]
_MAX_TIME_SECONDS = 100

_INIT_LEG_STATE = (
    gait_generator_lib.LegState.SWING,
    gait_generator_lib.LegState.STANCE,
    gait_generator_lib.LegState.STANCE,
    gait_generator_lib.LegState.SWING,
)


vx = 0.6 * robot_sim.MPC_VELOCITY_MULTIPLIER
vy = 0.2 * robot_sim.MPC_VELOCITY_MULTIPLIER
wz = 0.8 * robot_sim.MPC_VELOCITY_MULTIPLIER

time_points = np.array(range (0,_MAX_TIME_SECONDS))
print("time_points=",time_points)
speed_points=[]
for t in time_points:
  if (t%4==0):
    speed_points.append((vx*random.uniform(-1, 1),vy*random.uniform(-1, 1),0,wz*random.uniform(-1, 1)))
  if (t%4==1):
    speed_points.append((0,0,0,wz*random.uniform(-1, 1)))
  if (t%4==2):
    speed_points.append((vx*random.uniform(-1, 1),0,0,0))
  if (t%4==3):
    speed_points.append((0,vy*random.uniform(-1, 1),0,0))
print("speed_points=",speed_points)

def _generate_example_linear_angular_speed(t):
  """Creates an example speed profile based on time for demo purpose."""
  
  time_points = (0, 5, 20, 25, 30, 35,40)
  speed_points = ((vx, 0, 0, 0),(0, 0, 0, 0), (0, 0, 0, wz), (0, 0, 0, -wz), (0, -vy, 0, 0),
                  (0, 0, 0, 0), (0, 0, 0, wz))
  

  speed = scipy.interpolate.interp1d(
      time_points,
      speed_points,
      kind="previous",
      fill_value="extrapolate",
      axis=0)(
          t)

  return speed[0:3], speed[3]


def _setup_controller(robot):
  """Demonstrates how to create a locomotion controller."""
  desired_speed = (0, 0)
  desired_twisting_speed = 0

  gait_generator = openloop_gait_generator.OpenloopGaitGenerator(
      robot,
      stance_duration=_STANCE_DURATION_SECONDS,
      duty_factor=_DUTY_FACTOR,
      initial_leg_phase=_INIT_PHASE_FULL_CYCLE,
      initial_leg_state=_INIT_LEG_STATE)
  state_estimator = com_velocity_estimator.COMVelocityEstimator(robot,
                                                                window_size=20)
  sw_controller = raibert_swing_leg_controller.RaibertSwingLegController(
      robot,
      gait_generator,
      state_estimator,
      desired_speed=desired_speed,
      desired_twisting_speed=desired_twisting_speed,
      desired_height=robot_sim.MPC_BODY_HEIGHT,
      foot_clearance=0.01)

  st_controller = torque_stance_leg_controller.TorqueStanceLegController(
      robot,
      gait_generator,
      state_estimator,
      desired_speed=desired_speed,
      desired_twisting_speed=desired_twisting_speed,
      desired_body_height=robot_sim.MPC_BODY_HEIGHT,
      body_mass=robot_sim.MPC_BODY_MASS,
      body_inertia=robot_sim.MPC_BODY_INERTIA)

  controller = locomotion_controller.LocomotionController(
      robot=robot,
      gait_generator=gait_generator,
      state_estimator=state_estimator,
      swing_leg_controller=sw_controller,
      stance_leg_controller=st_controller,
      clock=robot.GetTimeSinceReset)
  return controller



def _update_controller_params(controller, lin_speed, ang_speed):
  controller.swing_leg_controller.desired_speed = lin_speed
  controller.swing_leg_controller.desired_twisting_speed = ang_speed
  controller.stance_leg_controller.desired_speed = lin_speed
  controller.stance_leg_controller.desired_twisting_speed = ang_speed


def _run_example(max_time=_MAX_TIME_SECONDS):
  """Runs the locomotion controller example."""
  
  simulation_time_step = 0.001
  #random.seed(10)
  heightPerturbationRange = 0.06
  

  robot = robot_sim.SimpleRobot(simulation_time_step=simulation_time_step)
  
  controller = _setup_controller(robot)
  controller.reset()
  
  current_time = robot.GetTimeSinceReset()
  
  while current_time < max_time:
    
    # Updates the controller behavior parameters.
    lin_speed, ang_speed = _generate_example_linear_angular_speed(current_time)
    #lin_speed, ang_speed = (0., 0., 0.), 0.
    _update_controller_params(controller, lin_speed, ang_speed)

    # Needed before every call to get_action().
    controller.update()
    hybrid_action, info = controller.get_action()
    
    robot.Step(hybrid_action)
    
    #if record_video:
    time.sleep(0.003)
    current_time = robot.GetTimeSinceReset()
  
  timestr = time.strftime("%Y%m%d-%H%M%S")
  controller._stance_leg_controller.dump("mpc_"+timestr)

def main(argv):
  del argv
  _run_example()


if __name__ == "__main__":
  app.run(main)
