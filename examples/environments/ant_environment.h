#ifndef ANT_ENVIRONMENT_H
#define ANT_ENVIRONMENT_H

#include "math/neural_network.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"
#include "urdf/urdf_cache.hpp"
#include "utils/file_utils.hpp"
#include "urdf/urdf_parser.hpp"

#include "math.h"


#include "math/neural_network.hpp"

static double ant_knee_angle{-0.5};
static double ant_hip = 0;
static std::vector<double> ant_initial_poses = {
    ant_hip, ant_knee_angle,
    ant_hip, ant_knee_angle,
    ant_hip, ant_knee_angle,
    ant_hip, ant_knee_angle,
};


template <typename Algebra>
struct AntContactSimulation {
    using Scalar = typename Algebra::Scalar;
    tds::UrdfCache<Algebra> cache;
    std::string m_urdf_filename;
    tds::World<Algebra> world;
    tds::MultiBody<Algebra>* mb_ = nullptr;

    int num_timesteps{ 1 };
    int num_visual_links_{0};

    Scalar dt{ Algebra::from_double(1e-2) };

    int input_dim() const { 
        int id = mb_->dof() + mb_->dof_qd();
        return id; 
    }

    int input_dim_with_action() const { 
        int id = mb_->dof() + mb_->dof_qd() + action_dim_;
        return id; 
    }

    int state_dim() const {
        int sd = mb_->dof() + mb_->dof_qd() + num_visual_links_ * 7;
        return sd;
    }
    int output_dim() const { 
        return num_timesteps * state_dim(); 
    }

    int action_dim_{(int)ant_initial_poses.size()};
    std::vector<Scalar> action_;

    AntContactSimulation() {
        std::string plane_filename;
        tds::FileUtils::find_file("plane_implicit.urdf", plane_filename);
        cache.construct(plane_filename, world, false, false);
        std::string urdf_name = "gym/ant_org_xyz_xyzrot.urdf";//ant_org.urdf";
        bool is_floating = false;
        //std::string urdf_name = "gym/ant_org.urdf";
        //bool is_floating = true;
        
        tds::FileUtils::find_file(urdf_name, m_urdf_filename);
        
        
        mb_ = cache.construct(m_urdf_filename, world, false, is_floating);
        mb_->base_X_world().set_identity();
        
        world.default_friction = 1;
        world.get_mb_constraint_solver()->keep_all_points_ = true;
        world.get_mb_constraint_solver()->pgs_iterations_ = 1;
        
        Scalar combinedContactStiffness ( 1.);
        Scalar combinedContactDamping ( .1);

        Scalar denom = (dt * combinedContactStiffness + combinedContactDamping);
        if (denom < 1e-6)
		{
		    denom = 1e-6;
        }
	    Scalar cfm = Scalar(1) / denom;
        Scalar erp = (dt * combinedContactStiffness) / denom;


        world.get_mb_constraint_solver()->cfm_ = cfm;
        world.get_mb_constraint_solver()->erp_ = erp;

        for (const auto link : *mb_) {
        //just copy the link world transform. Still have to multiple with visual transform for each instance.
            if (link.X_visuals.size())
            {
                num_visual_links_++;
            }
        }

        action_.resize(action_dim_);

        //world.set_gravity(Algebra::Vector3(Algebra::zero(),Algebra::zero(),Algebra::zero()));
        //ant_initial_poses.resize(mb_->q_.size());
        //for (int i=0;i<mb_->q_.size();i++)
        //{
        //    ant_initial_poses[i] = mb_->q_[i];
        //}
    }

    virtual ~AntContactSimulation()
    {
        printf("~AntContactSimulation\n");
    }
    std::vector<Scalar> operator()(const std::vector<Scalar>& v) {
        assert(static_cast<int>(v.size()) == input_dim_with_action());

        
        
        mb_->initialize();
        //copy input into q, qd
        for (int i = 0; i < mb_->dof(); ++i) {
            mb_->q(i) = v[i];
        }
        for (int i = 0; i < mb_->dof_qd(); ++i) {
            mb_->qd(i) = v[i + mb_->dof()];
        }
        for (int i=0;i<action_dim_;i++)
        {
            action_[i] = v[i+ mb_->dof()+mb_->dof_qd()];
        }

        std::vector<Scalar> result(output_dim());
        //for (int i=0;i<10;i++)
        {
            
            for (int t = 0; t < num_timesteps; ++t) {

                // pd control
                if (1) {
                    // use PD controller to compute tau
                    int qd_offset = mb_->is_floating() ? 6 : 6;
                    int q_offset = mb_->is_floating() ? 7 : 6;
                    int num_targets = mb_->tau_.size() - qd_offset;
                    std::vector<double> q_targets;
                    q_targets.resize(mb_->tau_.size());

                    Scalar kp (15);
                    Scalar kd (0.3);
                    Scalar max_force ( 3.);
                    int param_index = 0;

                    for (int i = 0; i < mb_->tau_.size(); i++) {
                        mb_->tau_[i] = 0;
                    }
                    int tau_index =  mb_->is_floating() ? 0 : 6;
                    int pose_index = 0;
                    int start_link = mb_->is_floating() ? 0 : 6;
                    for (int i = start_link; i < mb_->links_.size(); i++) {
                        if (mb_->links_[i].joint_type != tds::JOINT_FIXED) {

                            if (pose_index<ant_initial_poses.size())
                            {
                                //clamp action 
                                Scalar ACTION_LIMIT_LOW ( (i%2==0)? -0.5 : -1.1+0.5);
                                Scalar ACTION_LIMIT_HIGH ( (i%2==0)? 0.5 : 0);
                                Scalar clamped_action = action_[pose_index]*Scalar(10.);
                                clamped_action = Algebra::min(clamped_action, ACTION_LIMIT_HIGH);
                                clamped_action = Algebra::max(clamped_action, ACTION_LIMIT_LOW);

                                Scalar q_desired = ant_initial_poses[pose_index++] + clamped_action;
                            
                                Scalar q_actual = mb_->q_[q_offset];
                                Scalar qd_actual = mb_->qd_[qd_offset];
                                Scalar position_error = (q_desired - q_actual);
                                Scalar desired_velocity ( 0.);
                                Scalar velocity_error = (desired_velocity - qd_actual);
                                Scalar force = kp * position_error + kd * velocity_error;

                                force = Algebra::max(force, -max_force);
                                force = Algebra::min(force, max_force);
                                mb_->tau_[tau_index] = force;
                                q_offset++;
                                qd_offset++;
                                param_index++;
                                tau_index++;
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
            //copy q, qd, link world poses (for rendering) to output
            int j = 0;
            for (int i = 0; i < mb_->dof(); ++i, ++j) {
                result[j] = mb_->q(i);
            }
            for (int i = 0; i < mb_->dof_qd(); ++i, ++j) {
                result[j] = mb_->qd(i);
            }

            for (const auto link : *mb_) {
                //just copy the link world transform. Still have to multiple with visual transform for each instance.
                if (link.X_visuals.size())
                {
                    auto visual_X_world = link.X_world * link.X_visuals[0];//
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
        }
        return result;
    }
};

struct AntEnvOutput
{
    std::vector<double> obs;
    double reward;
    bool done;
};

struct AntRolloutOutput
{
    AntRolloutOutput()
        :total_reward(0),
        num_steps(0)
    {
    }
    double total_reward;
    int num_steps;
};



template <typename Algebra>
struct AntEnv
{
    AntContactSimulation<Algebra> contact_sim_;
    using Scalar = typename Algebra::Scalar;
    using Vector3 = typename Algebra::Vector3;
    using Transform = typename Algebra::Transform;
    using Quaternion = typename Algebra::Quaternion;

    AntEnv()
    {
        int observation_size = contact_sim_.input_dim();
        bool use_input_bias = false;
        neural_network.set_input_dim(observation_size, use_input_bias);
        //network.add_linear_layer(tds::NN_ACT_RELU, 32);
        //neural_network.add_linear_layer(tds::NN_ACT_RELU, 64);
        bool learn_bias = true;
        neural_network.add_linear_layer(tds::NN_ACT_IDENTITY, ant_initial_poses.size(),learn_bias);
    }
    virtual ~AntEnv()
    {
    }

    void init_neural_network(const std::vector<double> &x)
    {
        neural_network.set_parameters(x);
    }

    std::vector<Scalar> sim_state;
    std::vector<Scalar> sim_state_with_action;
    std::vector<Scalar> sim_state_with_graphics;

    tds::NeuralNetwork<Algebra> neural_network;

    void seed(long long int s) {
      //std::cout<<"seed:" << s << std::endl;
      std::srand(s);
    }
    
    
    int observation_dim_{28};//??

    std::vector<double> reset()
    {
        sim_state.resize(0);
        sim_state.resize(contact_sim_.input_dim(), Scalar(0));
        Vector3 start_pos(0,0,.48);//0.4002847
        Quaternion start_orn (0,0,0,1);

        if (contact_sim_.mb_->is_floating())
        {
            
            //sim_state[0] = start_orn.x();
            //sim_state[1] = start_orn.y();
            //sim_state[2] = start_orn.z();
            //sim_state[3] = start_orn.w();
            sim_state[4] = start_pos.x();
            sim_state[5] = start_pos.y();
            sim_state[6] = start_pos.z();
            int qoffset = 7;
            for(int j=0;j<ant_initial_poses.size();j++)
            {
                sim_state[j+qoffset] = ant_initial_poses[j];//0.05*((std::rand() * 1. / RAND_MAX)-0.5)*2.0;
            }
        }
        else
        {
            sim_state[0] = start_pos.x();
            sim_state[1] = start_pos.y();
            sim_state[2] = start_pos.z();
            sim_state[3] = 0;
            sim_state[4] = 0;
            sim_state[5] = 0;
            int qoffset = 6;
            for(int j=0;j<ant_initial_poses.size();j++)
            {
                sim_state[j+qoffset] = ant_initial_poses[j]+0.05*((std::rand() * 1. / RAND_MAX)-0.5)*2.0;
            }

        }

        std::vector<double> zero_action(ant_initial_poses.size(), Scalar(0));
        std::vector<double> observation;
        double reward;
        bool done;
        //todo: tune this
        int settle_down_steps= 50;
        for (int i=0;i<settle_down_steps;i++)
        {
            step(zero_action, observation, reward, done);
        }
        
        //for (auto v : sim_state)
        //    std::cout << v << std::endl;
        return observation;
    }
    void step(std::vector<double>& action,std::vector<double>& obs,double& reward,bool& done)
    {
        int simstate_size = sim_state.size();
        sim_state_with_action = sim_state;
        sim_state_with_action.resize(simstate_size+action.size());

        for (int i=0;i<action.size();i++)
        {
            sim_state_with_action[i+simstate_size] = action[i];
        }
        sim_state_with_graphics = contact_sim_(sim_state_with_action);
        sim_state = sim_state_with_graphics;

        sim_state.resize(contact_sim_.input_dim());
        obs = sim_state;
        
        //reward forward along x-axis
        reward = sim_state[0];
        static double max_reward = -1e30;
        static double min_reward = 1e30;
        if (reward < min_reward)
        {
            min_reward = reward;
            //printf("min_reward = %f\n",min_reward);
        }
        if (reward > max_reward)
        {
            max_reward = reward;
            //printf("max_reward = %f\n",max_reward);
        }

        //Ant height needs to stay above 0.25
        if (sim_state[2] < 0.26)
        {
            done =  true;
        }  else
        {
            done = false;
        }
    }
    
    AntEnvOutput step2(std::vector<double>& action)
    {
        AntEnvOutput output;
        step(action, output.obs, output.reward, output.done);
        return output;
    }
    
    AntRolloutOutput rollout(int rollout_length, double shift)
    {
      AntRolloutOutput rollout_out;
      std::vector<double> obs = reset();
      bool done = false;
      int steps=0;
      while (rollout_out.num_steps<rollout_length && !done)
      {
         //double action = 0.f;
         auto action = policy(obs);
         double reward;
         step(action, obs, reward, done);
         rollout_out.total_reward += reward;
         rollout_out.num_steps++;
      }
      return rollout_out;
    }

    inline const std::vector<double> policy(const std::vector<double>& obs)
    {
        std::vector<double> action (ant_initial_poses.size(), Scalar(0));
    
        neural_network.compute(obs, action);
                
        return action;
    }

};




#endif //ANT_ENVIRONMENT_H
