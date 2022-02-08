#ifndef LOCOMOTION_CONTACT_SIMULATION_H
#define LOCOMOTION_CONTACT_SIMULATION_H

#undef min
#undef max

#include "world.hpp"
#include "urdf/urdf_cache.hpp"
#include "dynamics/integrator.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "plane_implicit_urdf.h"
#include "utils/file_utils.hpp"
#include "math/matrix_utils.hpp"


template <typename Algebra, int VARIABLE_SIZE>
struct LocomotionContactSimulation {
  using Scalar = typename Algebra::Scalar;
  using Vector3 = typename Algebra::Vector3;
  using Quaternion = typename Algebra::Quaternion;

  tds::UrdfCache<Algebra> cache;
  std::string urdf_filename_;
  std::string urdf_full_path_;
  tds::World<Algebra> world;
  tds::MultiBody<Algebra>* mb_ = nullptr;

  int num_timesteps{1};
  Scalar dt{Algebra::from_double(1e-3)};
  
  
  std::vector<Scalar> initial_poses_;
  bool is_floating_{false};
  int action_dim_{0};
  int num_visual_links_{0};

  Scalar kp_{Algebra::zero()};
  Scalar kd_{Algebra::zero()};
  Scalar max_force_{Algebra::zero()};

  Vector3 m_start_base_position{Scalar(0),Scalar(0),Scalar(0.48)};
  Quaternion m_start_base_orientation{Scalar(0),Scalar(0),Scalar(0),Scalar(1)};


  void set_kp(const Scalar new_kp) { 
      kp_ = new_kp; 
  }

  void set_kd(const Scalar new_kd) { 
      kd_ = new_kd; 
  }
  
  void set_max_force(const Scalar new_max_force) { 
      max_force_ = new_max_force; 
  }


  int input_dim() const { return mb_->dof() + mb_->dof_qd(); }

  int input_dim_with_action2() const {
    return mb_->dof() + mb_->dof_qd() + action_dim();
  }


  int input_dim_with_action_and_variables() const {
      int dof = mb_->dof();
      int dofq = mb_->dof_qd();
      int adim = action_dim() ;
      int varsize = VARIABLE_SIZE;
      return dof+dofq+adim+varsize;
    //return mb_->dof() + mb_->dof_qd() + action_dim() + VARIABLE_SIZE;
  }

  // input, world transforms for all links and up_dot_world_z
  int state_dim() const {
    return mb_->dof() + mb_->dof_qd() + mb_->num_links() * num_visual_links_ + 1;
  }
  int output_dim() const { return state_dim(); }

  
  int action_dim() const { return action_dim_;}
  bool is_floating() const { return is_floating_;};
  
  int variables_dim_{VARIABLE_SIZE};

  int base_dof_{6};
  
  LocomotionContactSimulation(bool urdf_from_file, 
          const std::string& urdf_filename, 
          const std::string& urdf_string,
          const std::vector<Scalar>& initial_poses,
          bool floating,
          Scalar delta_time)
: dt(delta_time),
  initial_poses_(initial_poses),
  is_floating_(floating),
  action_dim_((int)initial_poses.size())
  {
    
    std::string plane_filename = "plane_impl";

    if (urdf_from_file) {

      tds::FileUtils::find_file("plane_implicit.urdf", plane_filename);
      cache.construct(plane_filename, world, false, false);

      tds::FileUtils::find_file(urdf_filename, urdf_filename_);
      char urdf_full_path[TINY_MAX_EXE_PATH_LEN];
      tds::FileUtils::extract_path(urdf_filename_.c_str(),
                                   urdf_full_path, TINY_MAX_EXE_PATH_LEN);
      urdf_full_path_ = urdf_full_path;

      mb_ = cache.construct(urdf_filename_, world, false,
                            is_floating());
    } else {
      std::string plane_string = plane_implicit_urdf;
      cache.construct_from_string(plane_filename, plane_string, world, false,
                                  false);

      urdf_filename_ = urdf_filename;
      mb_ = cache.construct_from_string(urdf_filename_, urdf_string,
                                        world, false, is_floating());
    }
     mb_->initialize();
     for (const auto& link : *mb_) {
            for (int v=0;v<link.X_visuals.size();v++)
            {
                num_visual_links_++;
            }
        }

    mb_->base_X_world().set_identity();
    world.default_friction = 1;
    //world.set_gravity(Vector3(Algebra::zero(),Algebra::zero(),Algebra::zero()));
    world.get_mb_constraint_solver()->keep_all_points_ = true;
  }

   void prepare_sim_state_with_action_and_variables(std::vector<Scalar>& v, const std::vector<Scalar>& actions) const {
        
        for (int i=0;i<actions.size();i++)
        {
            v[i+this->input_dim()] = actions[i];
        }

        v[this->input_dim_with_action2()+0] = kp_;
        v[this->input_dim_with_action2()+1] = kd_;
        v[this->input_dim_with_action2()+2] = max_force_;
    }

  //v is the input with action and variables
  void step_forward_original(const std::vector<Scalar>& v, std::vector<Scalar>& result) {
    mb_->initialize();
    // copy input into q, qd
    for (int i = 0; i < mb_->dof(); ++i) {
      mb_->q(i) = v[i];
    }
    for (int i = 0; i < mb_->dof_qd(); ++i) {
      mb_->qd(i) = v[i + mb_->dof()];
    }

    int action_offset = input_dim();
    int variable_index = input_dim()+action_dim();

    Scalar kp = v[variable_index+0];
    Scalar kd = v[variable_index+1];
    Scalar max_force = v[variable_index+2];
    
    for (int t = 0; t < num_timesteps; ++t) {
      bool usePD = true;
      if (usePD) {
        // use PD controller to compute tau
        
        int param_index = 0;

        for (int i = 0; i < mb_->tau_.size(); i++) {
          mb_->tau_[i] = 0;
        }

        int pose_index = 0;
        int start_link = mb_->is_floating() ? 0 : base_dof_;//skip 3 prismatic and 3 revolute or 4 spherical scalars
        for (int i = start_link; i < mb_->links_.size(); i++) {

            switch (mb_->links_[i].joint_type) {
            case tds::JOINT_FIXED:
                {
                    break;
                }
                case tds::JOINT_SPHERICAL:
                {
                    
                    int q_offset = mb_->links()[i].q_index;
                    int qd_offset = mb_->links()[i].qd_index;
                    int tau_index = mb_->is_floating() ? mb_->links()[i].qd_index - 6 : mb_->links()[i].qd_index;
                    //if (pose_index < initial_poses.size()) 
                    Quaternion q_desired = Algebra::quat_from_xyzw(Algebra::zero(),Algebra::zero(),Algebra::zero(),Algebra::one());
                    //  Quaternion q_desired = Algebra::quat_from_xyzw(initial_poses[pose_index],
                    //          initial_poses[pose_index+1],initial_poses[pose_index+2],initial_poses[pose_index+3]);

                    Quaternion q_actual = Algebra::quat_from_xyzw(mb_->q_[q_offset+0],mb_->q_[q_offset+1],mb_->q_[q_offset+2],mb_->q_[q_offset+3]);

                    Vector3 qd_actual(mb_->qd_[qd_offset],mb_->qd_[qd_offset+1],mb_->qd_[qd_offset+2]);
                    Vector3 qd_desired(Algebra::zero(),Algebra::zero(),Algebra::zero());
                    Vector3 position_error = tds::get_axis_difference_quaternion<Algebra>( q_desired, q_actual);

                    Vector3 velocity_error = (qd_desired - qd_actual);
                    Vector3 force = kp * position_error + kd * velocity_error;
                    force = Vector3(
                        Algebra::min(Algebra::max(force.x(), -max_force), max_force),
                        Algebra::min(Algebra::max(force.y(), -max_force), max_force),
                        Algebra::min(Algebra::max(force.z(), -max_force), max_force)
                    );

                    if (mb_->is_floating() || i>=4) {
                        mb_->tau_[tau_index++] = force.x();
                        mb_->tau_[tau_index++] = force.y();
                        mb_->tau_[tau_index++] = force.z();
                    } else {
                        tau_index+=3;
                    }
                    pose_index+=4;
                    
                    break;
                }
                default:
                {
                    assert(pose_index < initial_poses_.size());
                    {
                        int q_offset = mb_->links()[i].q_index;
                        int qd_offset = mb_->links()[i].qd_index;
                        int tau_index = mb_->is_floating() ? mb_->links()[i].qd_index - 6 : mb_->links()[i].qd_index;
                      
                        
                        Scalar clamped_action = v[action_offset+pose_index];
                        Scalar ACTION_LIMIT(0.4);
                        clamped_action = Algebra::min(clamped_action, ACTION_LIMIT);
                        clamped_action = Algebra::max(clamped_action, -ACTION_LIMIT);

                        Scalar q_desired = initial_poses_[pose_index++] + clamped_action ;

                        Scalar q_actual = mb_->q_[q_offset];
                        Scalar qd_actual = mb_->qd_[qd_offset];
                        Scalar position_error = (q_desired - q_actual);
                        Scalar desired_velocity(0);
                        Scalar velocity_error = (desired_velocity - qd_actual);
                        Scalar force = kp * position_error + kd * velocity_error;

                        force = Algebra::min(Algebra::max(force, -max_force), max_force);
                        mb_->tau_[tau_index] = force;
                        q_offset++;
                        qd_offset++;
                        param_index++;
                        tau_index++;
                    }
                }
            }

        }
      }

      
     tds::forward_dynamics(*mb_, world.get_gravity());

      mb_->clear_forces();

      integrate_euler_qdd(*mb_, dt);

      world.step(dt);

      tds::integrate_euler(*mb_, dt);
      
    }

    // copy q, qd, link world poses (for rendering) to output
    int j = 0;
    for (int i = 0; i < mb_->dof(); ++i, ++j) {
      result[j] = mb_->q(i);
    }
    for (int i = 0; i < mb_->dof_qd(); ++i, ++j) {
      result[j] = mb_->qd(i);
    }
    for (const auto& link : *mb_) {
        //just copy the link world transform. Still have to multiply with visual transform for each instance.
        for (int v=0;v<link.X_visuals.size();v++)
        {
            auto visual_X_world = link.X_world * link.X_visuals[v];//
            {
                result[j++] = visual_X_world.translation[0];
                result[j++] = visual_X_world.translation[1];
                result[j++] = visual_X_world.translation[2];
            }
            auto orn = Algebra::matrix_to_quat(visual_X_world.rotation);
            {
                result[j++] = orn.x();
                result[j++] = orn.y();
                result[j++] = orn.z();
                result[j++] = orn.w();
            }
        }
    }

    auto base_tr = mb_->get_world_transform(-1);
    Scalar up_dot_world_z = base_tr.rotation(2, 2);
    result[j++] = up_dot_world_z;
  }
};

#endif //LOCOMOTION_CONTACT_SIMULATION_H
