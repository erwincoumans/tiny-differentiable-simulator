import pytinydiffsim as dp

def apply(mb, desired_q):
    #use PD controller to compute tau
    qd_offset = 6
    q_offset = 7
    num_targets = mb.tau.size()
    #print("num_targets=",num_targets)
    kp = 220
    kd = 2
    max_force = 550
    pose_index = 0
    tau=mb.tau
    for l in mb.links:
      #print("pose_index=",pose_index)
      #print("l.joint_type=",l.joint_type)
      if (l.joint_type != dp.JOINT_FIXED):
        q_desired = desired_q[pose_index]
        q_actual = mb.q[pose_index+7]
        qd_actual = mb.qd[pose_index+6]
        position_error = q_desired-q_actual
        desired_velocity=0
        velocity_error = (desired_velocity - qd_actual)
        force = kp * position_error + kd * velocity_error;
        if (force < -max_force):
          force = -max_force
        if (force > max_force):
          force = max_force
        tau[pose_index] = force
        pose_index +=1
    mb.tau=tau
