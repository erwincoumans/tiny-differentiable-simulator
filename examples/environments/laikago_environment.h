#ifndef CARTPOLE_ENVIRONMENT_H
#define CARTPOLE_ENVIRONMENT_H


#include "math/neural_network.hpp"


template <typename Algebra>
struct ContactSimulation {
    using Scalar = typename Algebra::Scalar;
    tds::UrdfCache<Algebra> cache;
    std::string m_urdf_filename;
    tds::World<Algebra> world;
    tds::MultiBody<Algebra>* mb_ = nullptr;

    int num_timesteps{ 1 };
    Scalar dt{ Algebra::from_double(1e-3) };

    int input_dim() const { return mb_->dof() + mb_->dof_qd(); }
    int state_dim() const {
        return mb_->dof() + mb_->dof_qd() + mb_->num_links() * 7;
    }
    int output_dim() const { return num_timesteps * state_dim(); }

    ContactSimulation() {
        std::string plane_filename;
        tds::FileUtils::find_file("plane_implicit.urdf", plane_filename);
        cache.construct(plane_filename, world, false, false);

        tds::FileUtils::find_file(urdf_name, m_urdf_filename);
        
        
        mb_ = cache.construct(m_urdf_filename, world, false, is_floating);
        mb_->base_X_world().translation = start_pos;
        mb_->base_X_world().rotation = Algebra::quat_to_matrix(start_orn);
        world.default_friction = 1;
        //world.set_gravity(Algebra::Vector3(0,0,0));
        //initial_poses.resize(mb_->q_.size());
        //for (int i=0;i<mb_->q_.size();i++)
        //{
        //    initial_poses[i] = mb_->q_[i];
        //}
    }

    std::vector<Scalar> operator()(const std::vector<Scalar>& v, const std::vector<Scalar>& action) {
        assert(static_cast<int>(v.size()) == input_dim());
        mb_->initialize();
        //copy input into q, qd
        for (int i = 0; i < mb_->dof(); ++i) {
            mb_->q(i) = v[i];
        }
        for (int i = 0; i < mb_->dof_qd(); ++i) {
            mb_->qd(i) = v[i + mb_->dof()];
        }
        std::vector<Scalar> result(output_dim());
        for (int t = 0; t < num_timesteps; ++t) {

            // pd control
            if (1) {
                // use PD controller to compute tau
                int qd_offset = mb_->is_floating() ? 6 : 0;
                int q_offset = mb_->is_floating() ? 7 : 0;
                int num_targets = mb_->tau_.size() - qd_offset;
                std::vector<double> q_targets;
                q_targets.resize(mb_->tau_.size());

                double kp = 150;
                double kd = 3;
                double max_force = 50;
                int param_index = 0;

                for (int i = 0; i < mb_->tau_.size(); i++) {
                    mb_->tau_[i] = 0;
                }
                int tau_index = 0;
                int pose_index = 0;
                for (int i = 0; i < mb_->links_.size(); i++) {
                    if (mb_->links_[i].joint_type != JOINT_FIXED) {

                        if (pose_index<initial_poses.size())
                        {
                            //clamp action 
                            double clamped_action = action[pose_index];
                            double ACTION_LIMIT = 1;
                            clamped_action = Algebra::min(clamped_action, ACTION_LIMIT);
                            clamped_action = Algebra::max(clamped_action, -ACTION_LIMIT);


                            double q_desired = initial_poses[pose_index++] + clamped_action;
                            
                            double q_actual = mb_->q_[q_offset];
                            double qd_actual = mb_->qd_[qd_offset];
                            double position_error = (q_desired - q_actual);
                            double desired_velocity = 0;
                            double velocity_error = (desired_velocity - qd_actual);
                            double force = kp * position_error + kd * velocity_error;

                            if (force < -max_force) force = -max_force;
                            if (force > max_force) force = max_force;
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
                Transform visual_X_world = link.X_world * link.X_visuals[0];//
                result[j++] = visual_X_world.translation[0];
                result[j++] = visual_X_world.translation[1];
                result[j++] = visual_X_world.translation[2];
                auto orn = Algebra::matrix_to_quat(visual_X_world.rotation);
                result[j++] = orn[0];
                result[j++] = orn[1];
                result[j++] = orn[2];
                result[j++] = orn[3];
            }
        }
        return result;
    }
};




struct LaikagoEnv
{
    ContactSimulation<MyAlgebra>& contact_sim;

    LaikagoEnv(ContactSimulation<MyAlgebra>& cartpole)
        :contact_sim(cartpole)
    {
        int observation_size = contact_sim.input_dim();
        neural_network.set_input_dim(observation_size);
        neural_network.add_linear_layer(tds::NN_ACT_RELU, 128);
        neural_network.add_linear_layer(tds::NN_ACT_RELU, 64);
        neural_network.add_linear_layer(tds::NN_ACT_IDENTITY, initial_poses.size());
        
        

    }
    virtual ~LaikagoEnv()
    {
    }

    std::vector<MyScalar> sim_state;
    std::vector<MyScalar> sim_state_with_graphics;

    NeuralNetwork<MyAlgebra> neural_network;

    std::vector<double> reset()
    {
        sim_state.resize(0);
        sim_state.resize(contact_sim.input_dim(), MyScalar(0));

        if (contact_sim.mb_->is_floating())
        {
            sim_state[0] = start_orn.x();
            sim_state[1] = start_orn.y();
            sim_state[2] = start_orn.z();
            sim_state[3] = start_orn.w();
            sim_state[4] = start_pos.x();
            sim_state[5] = start_pos.y();
            sim_state[6] = start_pos.z();
            int qoffset = 7;
            for(int j=0;j<initial_poses.size();j++)
            {
                sim_state[j+qoffset] = initial_poses[j];//0.05*((std::rand() * 1. / RAND_MAX)-0.5)*2.0;
            }
        }
        else
        {
            for(int i=0;i<sim_state.size();i++)
            {
                sim_state[i] = 0.05*((std::rand() * 1. / RAND_MAX)-0.5)*2.0;
            }
        }

        std::vector<double> zero_action(initial_poses.size(), MyScalar(0));
        std::vector<double> observation;
        double reward;
        bool done;
        //todo: tune this
        int settle_down_steps= 100;
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
        
        sim_state_with_graphics = contact_sim(sim_state,action);
        sim_state = sim_state_with_graphics;

        sim_state.resize(contact_sim.input_dim());
        obs = sim_state;
        reward = 1;
        double x = sim_state[0];
        double theta = sim_state[1];

        double theta_threshold_radians = 12. * 2. * M_PI / 360.;
        double x_threshold = 0.4;//  #2.4
        auto base_tr = contact_sim.mb_->get_world_transform(-1);
        
        //reward forward along x-axis
        reward = base_tr.translation[0];
        MyScalar up_dot_world_z = base_tr.rotation(2,2);
        //Laikago needs to point up, angle > 30 degree (0.523599 radians)
        if (up_dot_world_z < 0.85 || base_tr.translation.z() < 0.3)
        {
            done =  true;
        }  else
        {
            done = false;
        }
    }
    
    inline const std::vector<double> policy(const std::vector<double> &x,const std::vector<double>& obs)
    {
        //todo, copy the weights in the constructor?
        int num_weights = neural_network.num_weights();
        int num_biases = neural_network.num_biases();

        assert(num_weights + num_biases == x.size());
        neural_network.set_parameters(x);
        
        std::vector<double> action (initial_poses.size(), MyScalar(0));
    
        neural_network.compute(obs, action);
                
        return action;
    }

};




#endif //CARTPOLE_ENVIRONMENT_H