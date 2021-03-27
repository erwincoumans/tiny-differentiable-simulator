import re
import numpy as np
import pytinydiffsim as dp
import meshcat_utils_dp
import meshcat

import time
np.set_printoptions(precision=3, suppress=True, linewidth=200)

def vecx_to_np(a):
  lst = []
  for i in range (a.size()):
    lst.append(a[i])
  return np.array(lst)

def vec3_to_np(a):
  lst = []
  for i in range(3):
    lst.append(a[i])
  return np.array(lst)
def vec4_to_np(a):
  lst = []
  for i in range(4):
    lst.append(a[i])
  return np.array(lst)


URDF_NAME = "laikago/laikago_toes_zup.urdf"
START_POS = [1.5, -2.0, 0.50]
START_ORN = [0,0,0,1]
MPC_BODY_MASS = 215/9.8
MPC_BODY_INERTIA = (0.07335, 0, 0, 0, 0.25068, 0, 0, 0, 0.25447)
MPC_BODY_HEIGHT = 0.42

MPC_VELOCITY_MULTIPLIER = 1.0


ACTION_REPEAT = 10

_IDENTITY_ORIENTATION=[0,0,0,1]
CHASSIS_NAME_PATTERN = re.compile(r"\w+_chassis_\w+")
MOTOR_NAME_PATTERN = re.compile(r"\w+_hip_motor_\w+")
KNEE_NAME_PATTERN = re.compile(r"\w+_lower_leg_\w+")
TOE_NAME_PATTERN = re.compile(r"jtoe\d*")
_DEFAULT_HIP_POSITIONS = (
    (0.21, -0.1157, 0),
    (0.21, 0.1157, 0),
    (-0.21, -0.1157, 0),
    (-0.21, 0.1157, 0),
)
_BODY_B_FIELD_NUMBER = 2
_LINK_A_FIELD_NUMBER = 3

HIP_JOINT_OFFSET = 0.0
UPPER_LEG_JOINT_OFFSET = 0#-0.6
KNEE_JOINT_OFFSET = 0#0.66


LAIKAGO_DEFAULT_ABDUCTION_ANGLE = 0.0#0.2
LAIKAGO_DEFAULT_HIP_ANGLE = 0.67-0.6
LAIKAGO_DEFAULT_KNEE_ANGLE = -1.25 +0.66
NUM_LEGS = 4
NUM_MOTORS = 12
# Bases on the readings from Laikago's default pose.
INIT_MOTOR_ANGLES = np.array([
    LAIKAGO_DEFAULT_ABDUCTION_ANGLE,
    LAIKAGO_DEFAULT_HIP_ANGLE,
    LAIKAGO_DEFAULT_KNEE_ANGLE
] * NUM_LEGS)
MOTOR_NAMES = [
    "FR_hip_motor_2_chassis_joint",
    "FR_upper_leg_2_hip_motor_joint",
    "FR_lower_leg_2_upper_leg_joint",
    "FL_hip_motor_2_chassis_joint",
    "FL_upper_leg_2_hip_motor_joint",
    "FL_lower_leg_2_upper_leg_joint",
    "RR_hip_motor_2_chassis_joint",
    "RR_upper_leg_2_hip_motor_joint",
    "RR_lower_leg_2_upper_leg_joint",
    "RL_hip_motor_2_chassis_joint",
    "RL_upper_leg_2_hip_motor_joint",
    "RL_lower_leg_2_upper_leg_joint",
]

#Use a PD controller
MOTOR_CONTROL_POSITION = 1 
# Apply motor torques directly.
MOTOR_CONTROL_TORQUE = 2
# Apply a tuple (q, qdot, kp, kd, tau) for each motor. Here q, qdot are motor
# position and velocities. kp and kd are PD gains. tau is the additional
# motor torque. This is the most flexible control mode.
MOTOR_CONTROL_HYBRID = 3
MOTOR_CONTROL_PWM = 4 #only for Minitaur



MOTOR_COMMAND_DIMENSION = 5

# These values represent the indices of each field in the motor command tuple
POSITION_INDEX = 0
POSITION_GAIN_INDEX = 1
VELOCITY_INDEX = 2
VELOCITY_GAIN_INDEX = 3
TORQUE_INDEX = 4


class LaikagoMotorModel(object):
  """A simple motor model for Laikago.

    When in POSITION mode, the torque is calculated according to the difference
    between current and desired joint angle, as well as the joint velocity.
    For more information about PD control, please refer to:
    https://en.wikipedia.org/wiki/PID_controller.

    The model supports a HYBRID mode in which each motor command can be a tuple
    (desired_motor_angle, position_gain, desired_motor_velocity, velocity_gain,
    torque).

  """

  def __init__(self,
               kp,
               kd,
               torque_limits=None,
               motor_control_mode=MOTOR_CONTROL_POSITION):
    self._kp = kp
    self._kd = kd
    self._torque_limits = torque_limits
    if torque_limits is not None:
      if isinstance(torque_limits, (collections.Sequence, np.ndarray)):
        self._torque_limits = np.asarray(torque_limits)
      else:
        self._torque_limits = np.full(NUM_MOTORS, torque_limits)
    self._motor_control_mode = motor_control_mode
    self._strength_ratios = np.full(NUM_MOTORS, 1)

  def set_strength_ratios(self, ratios):
    """Set the strength of each motors relative to the default value.

    Args:
      ratios: The relative strength of motor output. A numpy array ranging from
        0.0 to 1.0.
    """
    self._strength_ratios = ratios

  def set_motor_gains(self, kp, kd):
    """Set the gains of all motors.

    These gains are PD gains for motor positional control. kp is the
    proportional gain and kd is the derivative gain.

    Args:
      kp: proportional gain of the motors.
      kd: derivative gain of the motors.
    """
    self._kp = kp
    self._kd = kd

  def set_voltage(self, voltage):
    pass

  def get_voltage(self):
    return 0.0

  def set_viscous_damping(self, viscous_damping):
    pass

  def get_viscous_dampling(self):
    return 0.0

  def convert_to_torque(self,
                        motor_commands,
                        motor_angle,
                        motor_velocity,
                        true_motor_velocity,
                        motor_control_mode=None):
    """Convert the commands (position control or torque control) to torque.

    Args:
      motor_commands: The desired motor angle if the motor is in position
        control mode. The pwm signal if the motor is in torque control mode.
      motor_angle: The motor angle observed at the current time step. It is
        actually the true motor angle observed a few milliseconds ago (pd
        latency).
      motor_velocity: The motor velocity observed at the current time step, it
        is actually the true motor velocity a few milliseconds ago (pd latency).
      true_motor_velocity: The true motor velocity. The true velocity is used to
        compute back EMF voltage and viscous damping.
      motor_control_mode: A MotorControlMode enum.

    Returns:
      actual_torque: The torque that needs to be applied to the motor.
      observed_torque: The torque observed by the sensor.
    """
    del true_motor_velocity
    if not motor_control_mode:
      motor_control_mode = self._motor_control_mode

    # No processing for motor torques
    if motor_control_mode is MOTOR_CONTROL_TORQUE:
      assert len(motor_commands) == NUM_MOTORS
      motor_torques = self._strength_ratios * motor_commands
      return motor_torques, motor_torques

    desired_motor_angles = None
    desired_motor_velocities = None
    kp = None
    kd = None
    additional_torques = np.full(NUM_MOTORS, 0)
    if motor_control_mode is MOTOR_CONTROL_POSITION:
      assert len(motor_commands) == NUM_MOTORS
      kp = self._kp
      kd = self._kd
      desired_motor_angles = motor_commands
      desired_motor_velocities = np.full(NUM_MOTORS, 0)
    elif motor_control_mode is MOTOR_CONTROL_HYBRID:
      # The input should be a 60 dimension vector
      assert len(motor_commands) == MOTOR_COMMAND_DIMENSION * NUM_MOTORS
      kp = motor_commands[POSITION_GAIN_INDEX::MOTOR_COMMAND_DIMENSION]
      kd = motor_commands[VELOCITY_GAIN_INDEX::MOTOR_COMMAND_DIMENSION]
      desired_motor_angles = motor_commands[
          POSITION_INDEX::MOTOR_COMMAND_DIMENSION]
      desired_motor_velocities = motor_commands[
          VELOCITY_INDEX::MOTOR_COMMAND_DIMENSION]
      additional_torques = motor_commands[TORQUE_INDEX::MOTOR_COMMAND_DIMENSION]
    motor_torques = -1 * (kp * (motor_angle - desired_motor_angles)) - kd * (
        motor_velocity - desired_motor_velocities) + additional_torques
    motor_torques = self._strength_ratios * motor_torques
    if self._torque_limits is not None:
      if len(self._torque_limits) != len(motor_torques):
        raise ValueError(
            "Torque limits dimension does not match the number of motors.")
      motor_torques = np.clip(motor_torques, -1.0 * self._torque_limits,
                              self._torque_limits)

    return motor_torques, motor_torques

  

class SimpleRobot(object):
  def __init__(self, simulation_time_step):
    self.MPC_BODY_MASS  = MPC_BODY_MASS
    self.MPC_BODY_INERTIA = MPC_BODY_INERTIA
    self.MPC_BODY_HEIGHT = MPC_BODY_HEIGHT

    self.time_step = simulation_time_step
    
    self.num_legs = NUM_LEGS
    self.num_motors = NUM_MOTORS
 
    self.skip_sync=0
    self.world = dp.TinyWorld()
    self.world.friction = 10.0

    self.mb_solver = dp.TinyMultiBodyConstraintSolver()
    self.vis = meshcat.Visualizer(zmq_url='tcp://127.0.0.1:6000')
    self.vis.delete()

    urdf_parser = dp.TinyUrdfParser()

    plane_urdf_data = urdf_parser.load_urdf("../../../data/plane_implicit.urdf")
    plane2vis = meshcat_utils_dp.convert_visuals(plane_urdf_data, "../../../data/checker_purple.png", self.vis, "../../../data/")
    self.plane_mb = dp.TinyMultiBody(False)
    plane2mb = dp.UrdfToMultiBody2()
    res = plane2mb.convert2(plane_urdf_data, self.world, self.plane_mb)

    self.urdf_data = urdf_parser.load_urdf("../../../data/laikago/laikago_toes_zup_joint_order.urdf")
    print("robot_name=",self.urdf_data.robot_name)
    self.b2vis = meshcat_utils_dp.convert_visuals(self.urdf_data, "../../../data/laikago/laikago_tex.jpg", self.vis, "../../../data/laikago/")
    is_floating=True
    self.mb = dp.TinyMultiBody(is_floating)
    urdf2mb = dp.UrdfToMultiBody2()
    self.res = urdf2mb.convert2(self.urdf_data, self.world, self.mb)
    self.mb.set_base_position(dp.Vector3(0,0,0.6))

    knee_angle = -0.5
    abduction_angle = 0.2

    self.initial_poses = [abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle,
                     abduction_angle, 0., knee_angle, abduction_angle, 0., knee_angle]

    qcopy = self.mb.q
    print("mb.q=",self.mb.q)
    print("qcopy=",qcopy)
    for q_index in range (12):
      qcopy[q_index+7]=self.initial_poses[q_index]
    print("qcopy=",qcopy)
    self.mb.set_q(qcopy)
    print("2 mb.q=",self.mb.q)
    print("self.mb.links=", self.mb.links)    
    print("TDS:")
    for i in range (len(self.mb.links)):
      print("i=",i)
      l = self.mb.links[i]
      #print("l.link_name=", l.link_name)
      print("l.joint_name=", l.joint_name)
    dp.forward_kinematics(self.mb, self.mb.q, self.mb.qd)

    
    self._BuildJointNameToIdDict()
    self._BuildUrdfIds()
    self._BuildMotorIdList()
    self.ResetPose()
    self._motor_enabled_list = [True] * self.num_motors
    self._step_counter = 0
    self._state_action_counter = 0
    self._motor_offset= np.array([0.]*12)
    self._motor_direction= np.array([1.]*12)
    self.ReceiveObservation()
    self._kp = self.GetMotorPositionGains()
    self._kd = self.GetMotorVelocityGains()
    self._motor_model = LaikagoMotorModel(kp=self._kp, kd=self._kd, motor_control_mode=MOTOR_CONTROL_HYBRID)
    
    self.ReceiveObservation()
    self.mb.set_base_position(dp.Vector3(START_POS[0],START_POS[1],START_POS[2]))
    self.mb.set_base_orientation(dp.Quaternion(START_ORN[0],START_ORN[1],START_ORN[2],START_ORN[3]))
    
    dp.forward_kinematics(self.mb, self.mb.q, self.mb.qd)
    meshcat_utils_dp.sync_visual_transforms(self.mb, self.b2vis, self.vis)
    
    
    self._SettleDownForReset(reset_time=1.0)
    

    
  def ResetPose(self):
    pass

  def _SettleDownForReset(self, reset_time):
    self.ReceiveObservation()
    if reset_time <= 0:
      return
    for _ in range(1500):
      self._StepInternal(
          INIT_MOTOR_ANGLES,
          motor_control_mode=MOTOR_CONTROL_POSITION)
        
  def _GetMotorNames(self):
    return MOTOR_NAMES
    
  def _BuildMotorIdList(self):
    self._motor_id_list = [
        self._joint_name_to_id[motor_name]
        for motor_name in self._GetMotorNames()
    ]
  
  def GetMotorPositionGains(self):
    return [220.]*self.num_motors
    
  def GetMotorVelocityGains(self):
    return np.array([2.]*self.num_motors)#np.array([1., 2., 2., 1., 2., 2., 1., 2., 2., 1., 2., 2.])
    
  def compute_jacobian(self, robot, link_id):
    """Computes the Jacobian matrix for the given link.

    Args:
      robot: A robot instance.
      link_id: The link id as returned from loadURDF.

    Returns:
      The 3 x N transposed Jacobian matrix. where N is the total DoFs of the
      robot. For a quadruped, the first 6 columns of the matrix corresponds to
      the CoM translation and rotation. The columns corresponds to a leg can be
      extracted with indices [6 + leg_id * 3: 6 + leg_id * 3 + 3].
    """
    
    world_point = self.mb.links[link_id].world_transform.translation
    local_point = self.mb.get_world_transform(-1).apply_inverse(world_point)
    jacdp = dp.point_jacobian(self.mb, link_id, local_point, True)
    #jacdp.print("jacdp")
    #print("number of rows=", jacdp.num_rows)
    #print("number of columns=", jacdp.num_columns)
    jakkes=[]
    diff=[]
    for r in range (jacdp.num_rows):
      for c in range (jacdp.num_columns):
        v = jacdp.get_at(r,c)
        jakkes.append(v)
        
    jacobian2 = np.array(jakkes).reshape(jacdp.num_rows,jacdp.num_columns)
    #print("jacobian2=", jacobian2)  
    #print("diffjac[",link_id,"]=",diffjac)
    assert jacobian2.shape[0] == 3
    return jacobian2
    
  def ComputeJacobian(self, leg_id):
    """Compute the Jacobian for a given leg."""
    # Does not work for Minitaur which has the four bar mechanism for now.
    assert len(self._foot_link_ids) == self.num_legs
    return self.compute_jacobian(
        robot=self,
        link_id=self._foot_link_ids[leg_id],
    )
    
  def MapContactForceToJointTorques(self, leg_id, contact_force):
    """Maps the foot contact force to the leg joint torques."""
    jv = self.ComputeJacobian(leg_id)
    all_motor_torques = np.matmul(contact_force, jv)
    motor_torques = {}
    motors_per_leg = self.num_motors // self.num_legs
    com_dof = 6
    for joint_id in range(leg_id * motors_per_leg,
                          (leg_id + 1) * motors_per_leg):
      motor_torques[joint_id] = all_motor_torques[
          com_dof + joint_id] * self._motor_direction[joint_id]

    return motor_torques
    
  def GetBaseRollPitchYaw(self):
    """Get minitaur's base orientation in euler angle in the world frame.
  
    Returns:
      A tuple (roll, pitch, yaw) of the base in world frame.
    """
    orn = self.mb.get_base_orientation()
    
    roll_pitch_yaw2 = vec3_to_np(orn.get_euler_rpy())
    #print("roll_pitch_yaw=",roll_pitch_yaw)
    #print("roll_pitch_yaw2=",roll_pitch_yaw2)
    
    return np.asarray(roll_pitch_yaw2)
    
  
  def joint_angles_from_link_position(  self,    robot,    link_position,    link_id,    joint_ids,    position_in_world_frame,
              base_translation = (0, 0, 0),    base_rotation = (0, 0, 0, 1)):
    """Uses Inverse Kinematics to calculate joint angles.

    Args:
      robot: A robot instance.
      link_position: The (x, y, z) of the link in the body or the world frame,
        depending on whether the argument position_in_world_frame is true.
      link_id: The link id as returned from loadURDF.
      joint_ids: The positional index of the joints. This can be different from
        the joint unique ids.
      position_in_world_frame: Whether the input link_position is specified
        in the world frame or the robot's base frame.
      base_translation: Additional base translation.
      base_rotation: Additional base rotation.

    Returns:
      A list of joint angles.
    """
    
    if not position_in_world_frame:
      #tds
      base_world_tr = self.mb.get_world_transform(-1)
      local_base_tr = dp.TinySpatialTransform()
      local_base_tr.translation = dp.Vector3(base_translation[0],base_translation[1],base_translation[2])
      local_base_tr.rotation = dp.Matrix3(dp.Quaternion(base_rotation[0],base_rotation[1],base_rotation[2],base_rotation[3]))
      world_tr = base_world_tr*local_base_tr
      local_link_tr = dp.TinySpatialTransform()
      local_link_tr.rotation = dp.Matrix3(dp.Quaternion(0,0,0,1))
      local_link_tr.translation = dp.Vector3(link_position[0],link_position[1],link_position[2])
      link_tr = world_tr* local_link_tr
      world_link_pos = [link_tr.translation[0],link_tr.translation[1],link_tr.translation[2]]
        
    else:
      world_link_pos = link_position

    ik_solver = 0
    
    target_world_pos = dp.Vector3(world_link_pos[0],world_link_pos[1],world_link_pos[2])
    #target_local_pos = self.mb.get_world_transform(-1).apply_inverse(target_world_pos)
    
    all_joint_angles2 = dp.inverse_kinematics(self.mb, link_id, target_world_pos)
    #print("--")
    #print("len(all_joint_angles)=",len(all_joint_angles))
    #print("all_joint_angles=",all_joint_angles)
    
    vec = []
    for i in range (all_joint_angles2.size()-7):
      vec.append(all_joint_angles2[i+7])
    #print("len(all_joint_angles2)=",len(vec))
    #print("all_joint_angles2=",vec)
    vec = np.array(vec)
    #diff = vec - all_joint_angles
    #print("diff=",diff)
    # Extract the relevant joint angles.
    joint_angles = [vec[i] for i in joint_ids]
    return joint_angles
  
  def ComputeMotorAnglesFromFootLocalPosition(self, leg_id,
                                              foot_local_position):
    """Use IK to compute the motor angles, given the foot link's local position.

    Args:
      leg_id: The leg index.
      foot_local_position: The foot link's position in the base frame.

    Returns:
      A tuple. The position indices and the angles for all joints along the
      leg. The position indices is consistent with the joint orders as returned
      by GetMotorAngles API.
    """
    return self._EndEffectorIK(
        leg_id, foot_local_position, position_in_world_frame=False)
  def _EndEffectorIK(self, leg_id, position, position_in_world_frame):
    """Calculate the joint positions from the end effector position."""
    assert len(self._foot_link_ids) == self.num_legs
    toe_id = self._foot_link_ids[leg_id]
    motors_per_leg = self.num_motors // self.num_legs
    joint_position_idxs = [
        i for i in range(leg_id * motors_per_leg, leg_id * motors_per_leg +
                         motors_per_leg)
    ]
    joint_angles = self.joint_angles_from_link_position(
        robot=self,
        link_position=position,
        link_id=toe_id,
        joint_ids=joint_position_idxs,
        position_in_world_frame=position_in_world_frame)
    # Joint offset is necessary for Laikago.
    joint_angles = np.multiply(
        np.asarray(joint_angles) -
        np.asarray(self._motor_offset)[joint_position_idxs],
        self._motor_direction[joint_position_idxs])
    # Return the joing index (the same as when calling GetMotorAngles) as well
    # as the angles.
    return joint_position_idxs, joint_angles.tolist()
    
  def GetTimeSinceReset(self):
    return self._step_counter * self.time_step
    
  def GetHipPositionsInBaseFrame(self):
    return _DEFAULT_HIP_POSITIONS
    
  def GetBaseAngularVelocity(self):
    angvel = [self.mb.qd[0],self.mb.qd[1],self.mb.qd[2]]
    return angvel
  def GetBaseVelocity(self):
    """Get the linear velocity of minitaur's base.

    Returns:
      The velocity of minitaur's base.
    """
    #print("tds linvel")
    linvel = [self.mb.qd[3],self.mb.qd[4],self.mb.qd[5]]
    return linvel

  def GetTrueBaseOrientation(self):
    rn = self.mb.get_base_orientation()
    orn=[rn.x,rn.y,rn.z,rn.w]
    return orn
    
  def TransformAngularVelocityToLocalFrame(self, angular_velocity, orientation):
    """Transform the angular velocity from world frame to robot's frame.

    Args:
      angular_velocity: Angular velocity of the robot in world frame.
      orientation: Orientation of the robot represented as a quaternion.

    Returns:
      angular velocity of based on the given orientation.
    """
    
    
    # Treat angular velocity as a position vector, then transform based on the
    # orientation given by dividing (or multiplying with inverse).
    # Get inverse quaternion assuming the vector is at 0,0,0 origin.
    tr = dp.TinySpatialTransform()
    tr.translation = dp.Vector3(1,2,3)
    tr.translation.set_zero()
    orn = dp.Quaternion(orientation[0],orientation[1],orientation[2],orientation[3])
    tr.rotation = dp.Matrix3(orn)
    tr_inv = tr.get_inverse()
    rel_vel = tr.apply_inverse(dp.Vector3(angular_velocity[0],angular_velocity[1],angular_velocity[2]))
    #tr.print("tds trans")
    #print("tds rel_vel=",rel_vel)
    
    #print(tr)
    
    return vec3_to_np(rel_vel)
    
  def GetBaseRollPitchYawRate(self):
    """Get the rate of orientation change of the minitaur's base in euler angle.
  
    Returns:
      rate of (roll, pitch, yaw) change of the minitaur's base.
    """
    angular_velocity = self.GetBaseAngularVelocity()
    orientation = self.GetTrueBaseOrientation()
    
    
    return self.TransformAngularVelocityToLocalFrame(angular_velocity,orientation)
                                                     
  def GetFootContacts(self):
    contacts = [False, False, False, False]
    return contacts

    #all_contacts = self.pybullet_client.getContactPoints(bodyA=self.quadruped)
    #  
    #print("len(all_contacts)=", len(all_contacts))
    #for contact in all_contacts:
    #  # Ignore self contacts
    #  if contact[_BODY_B_FIELD_NUMBER] == self.quadruped:
    #    continue
    #  try:
    #    toe_link_index = self._foot_link_ids.index(
    #        contact[_LINK_A_FIELD_NUMBER])
    #    contacts[toe_link_index] = True
    #  except ValueError:
    #    continue
    #return contacts
    
  def GetTrueMotorAngles(self):
    """Gets the eight motor angles at the current moment, mapped to [-pi, pi].

    Returns:
      Motor angles, mapped to [-pi, pi].
    """
    self.ReceiveObservation()
    
    #motor_angles = [state[0] for state in self._joint_states]
    motor_angles = np.multiply(
        np.asarray(self.motor_angles) - np.asarray(self._motor_offset),
        self._motor_direction)
    return motor_angles

  def GetPDObservation(self):
    self.ReceiveObservation()
    observation = []
    observation.extend(self.GetTrueMotorAngles())
    observation.extend(self.GetTrueMotorVelocities())
    q = observation[0:self.num_motors]
    qdot = observation[self.num_motors:2 * self.num_motors]
    return (np.array(q), np.array(qdot))


  def GetTrueMotorVelocities(self):
    """Get the velocity of all eight motors.

    Returns:
      Velocities of all eight motors.
    """
    #motor_velocities = [state[1] for state in self._joint_states]

    motor_velocities = np.multiply(self.motor_velocities, self._motor_direction)
    return motor_velocities
    

  #def GetTrueObservation(self):
  #  self.ReceiveObservation()
  #  observation = []
  #  observation.extend(self.GetTrueMotorAngles())
  #  observation.extend(self.GetTrueMotorVelocities())
  #  observation.extend(self.GetTrueMotorTorques())
  #  observation.extend(self.GetTrueBaseOrientation())
  #  observation.extend(self.GetTrueBaseRollPitchYawRate())
  #  return observation
    
  def ApplyAction(self, motor_commands, motor_control_mode):
    """Apply the motor commands using the motor model.

    Args:
      motor_commands: np.array. Can be motor angles, torques, hybrid commands
      motor_control_mode: A MotorControlMode enum.
    """
    
    dp.forward_kinematics(self.mb, self.mb.q, self.mb.qd)
    dp.forward_dynamics(self.mb, dp.Vector3(0.,0.,-10.))

    mb_q = vecx_to_np(self.mb.q)[7:]
    mb_qd = vecx_to_np(self.mb.qd)[6:]

    actual_torque, observed_torque = self._motor_model.convert_to_torque(
                          motor_commands,
                          mb_q,
                          mb_qd,
                          mb_qd,
                          motor_control_mode=motor_control_mode)
    #print("actual_torque=",actual_torque)
    for i in range(len(actual_torque)):
      self.mb.tau[i] = actual_torque[i]

  def _SetMotorTorqueByIds(self, motor_ids, torques):
    pass
        
  def ReceiveObservation(self):    
    self.motor_angles = vecx_to_np(self.mb.q)[7:]
    #print("tds motor_angles=",self.motor_angles)
    self.motor_velocities = vecx_to_np(self.mb.qd)[6:]
    #print("tds motor_velocities=",self.motor_velocities)
    

  def _StepInternal(self, action, motor_control_mode):
    self.ApplyAction(action, motor_control_mode)
      
    dp.integrate_euler_qdd(self.mb, self.time_step)
    
    multi_bodies = [self.plane_mb, self.mb]
    dispatcher = self.world.get_collision_dispatcher()
    contacts = self.world.compute_contacts_multi_body(multi_bodies,dispatcher)
      
    sync_to_pybullet = False
    
    #collision solver
    for cps in contacts:
      self.mb_solver.resolve_collision(cps, self.time_step)
    dp.integrate_euler(self.mb, self.time_step)
    
    joint_states = []
    for q_index in range (12):
      #qcopy[q_index+7]=self.initial_poses[q_index]
      joint_position = self.mb.q[q_index+7]
      joint_velocity = self.mb.qd[q_index+6]
      if sync_to_pybullet:
        self.pybullet_client.resetJointState(self.quadruped, self._motor_id_list[q_index], joint_position,joint_velocity)
    
    ps = self.mb.get_base_position()
    rn = self.mb.get_base_orientation()
    angvel = [self.mb.qd[0],self.mb.qd[1],self.mb.qd[2]]
    linvel = [self.mb.qd[3],self.mb.qd[4],self.mb.qd[5]]
    pos=[ps[0],ps[1],ps[2]]
    orn=[rn.x,rn.y,rn.z,rn.w]
    
    if sync_to_pybullet:
      self.pybullet_client.resetBasePositionAndOrientation(self.quadruped, pos, orn)
      self.pybullet_client.resetBaseVelocity(self.quadruped, linvel, angvel)
    self.ReceiveObservation()
    
    self.skip_sync-=1
    if self.skip_sync<=0:
      #print("frame=",frame)
      self.skip_sync=16
      meshcat_utils_dp.sync_visual_transforms(self.mb, self.b2vis, self.vis)
    
    self._state_action_counter += 1
    
  def Step(self, action):
    """Steps simulation."""
    #if self._enable_action_filter:
    #  action = self._FilterAction(action)

    for i in range(ACTION_REPEAT):
      #proc_action = self.ProcessAction(action, i)
      proc_action = action
      self._StepInternal(proc_action, motor_control_mode=MOTOR_CONTROL_HYBRID)
      self._step_counter += 1
    
  def _BuildJointNameToIdDict(self):
    self._joint_name_to_id = {}
    print("TDS:")
    for joint_id in range(len(self.urdf_data.joints)):
      j = self.urdf_data.joints[joint_id]
      print("joint_id=",joint_id)
      print(j.joint_name)
      self._joint_name_to_id[j.joint_name] = joint_id
    
  def _BuildUrdfIds(self):
    """Build the link Ids from its name in the URDF file.

    Raises:
      ValueError: Unknown category of the joint name.
    """
    self._chassis_link_ids = [-1]
    self._leg_link_ids = []
    self._motor_link_ids = []
    self._knee_link_ids = []
    self._foot_link_ids = []
    
    print("TDS")
    num_joints = len(self.urdf_data.joints)
    for joint_id in range(num_joints):
      j = self.urdf_data.joints[joint_id]
      joint_name = j.joint_name
      print("link index=", joint_id)
      print("joint_name=",joint_name)
      joint_id = self._joint_name_to_id[joint_name]
      if CHASSIS_NAME_PATTERN.match(joint_name):
        self._chassis_link_ids.append(joint_id)
      elif MOTOR_NAME_PATTERN.match(joint_name):
        self._motor_link_ids.append(joint_id)
      # We either treat the lower leg or the toe as the foot link, depending on
      # the urdf version used.
      elif KNEE_NAME_PATTERN.match(joint_name):
        self._knee_link_ids.append(joint_id)
      elif TOE_NAME_PATTERN.match(joint_name):
        #assert self._urdf_filename == URDF_WITH_TOES
        self._foot_link_ids.append(joint_id)
      else:
        raise ValueError("Unknown category of joint %s" % joint_name)
      
    
    self._leg_link_ids.extend(self._knee_link_ids)
    self._leg_link_ids.extend(self._foot_link_ids)

    
    #assert len(self._foot_link_ids) == NUM_LEGS
    self._chassis_link_ids.sort()
    self._motor_link_ids.sort()
    self._knee_link_ids.sort()
    self._foot_link_ids.sort()
    self._leg_link_ids.sort()

    return

  def link_position_in_base_frame( self,   link_id ):
    """Computes the link's local position in the robot frame.

    Args:
      robot: A robot instance.
      link_id: The link to calculate its relative position.

    Returns:
      The relative position of the link.
    """
    link_local_frame = dp.link_transform_base_frame(self.mb, self.mb.q, link_id)
    link_local_position = vec3_to_np(link_local_frame.translation)
      
    return np.array(link_local_position)


  def GetFootLinkIDs(self):
    """Get list of IDs for all foot links."""
    return self._foot_link_ids
    
  def GetFootPositionsInBaseFrame(self):
    """Get the robot's foot position in the base frame."""
    assert len(self._foot_link_ids) == self.num_legs
    foot_positions = []
    for foot_id in self.GetFootLinkIDs():
      foot_positions.append(
          self.link_position_in_base_frame(link_id=foot_id)
          )
    return np.array(foot_positions)
    
