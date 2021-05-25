#include <iostream>

#include <pagmo/algorithm.hpp>
#include <pagmo/algorithms/sade.hpp>
#include <pagmo/algorithms/cmaes.hpp>
#include <pagmo/archipelago.hpp>
#include <pagmo/problem.hpp>
#include <pagmo/problems/schwefel.hpp>




#include <chrono>
#include <fstream>
#include <iostream>
#include <streambuf>
#include <string>
#include <thread>


#include "math/tiny/tiny_double_utils.h"
#include "utils/file_utils.hpp"
#include "urdf/urdf_parser.hpp"
#include "urdf/urdf_to_multi_body.hpp"
#include "dynamics/forward_dynamics.hpp"
#include "dynamics/integrator.hpp"

#include "urdf/urdf_cache.hpp"
#include "tiny_visual_instance_generator.h"

using namespace TINY;
using namespace tds;

typedef double TinyDualScalar;
typedef double MyScalar;
typedef ::TINY::DoubleUtils MyTinyConstants;
#include "math/tiny/tiny_algebra.hpp"
typedef TinyAlgebra<double,MyTinyConstants> MyAlgebra;

typedef TinyVector3<double,DoubleUtils> Vector3;
typedef TinyQuaternion<double,DoubleUtils> Quaternion;

using namespace pagmo;


#ifdef USE_LAIKAGO
static MyAlgebra::Vector3 start_pos(0,0,.48);//0.4002847
//MyAlgebra::Quaternion start_orn = MyAlgebra::quat_from_euler_rpy(MyAlgebra::Vector3(-3.14/2.,0,0));
//MyAlgebra::Quaternion start_orn(0.23364591,0,0,0.97232174932);
//MyAlgebra::Quaternion start_orn(0,0,0,1);//0.23364591,0,0,0.97232174932);
//MyAlgebra::Quaternion start_orn = MyAlgebra::quat_from_euler_rpy(MyAlgebra::Vector3(3.14/5.,0,0));
//MyAlgebra::Quaternion start_orn = MyAlgebra::quat_from_euler_rpy(MyAlgebra::Vector3(0,3.14/2.,0));
static MyAlgebra::Quaternion start_orn (0,0,0,1);

static std::string urdf_name = "laikago/laikago_toes_zup.urdf";
static bool is_floating = true;
static double hip_angle = 0.07;//0
static double knee_angle = -0.59;//-0.5;
static double abduction_angle = 0.2;
static std::vector<double> initial_poses = {
    abduction_angle, hip_angle, knee_angle, abduction_angle, hip_angle, knee_angle,
    abduction_angle, hip_angle, knee_angle, abduction_angle, hip_angle, knee_angle,
};
#include "environments/laikago_environment.h"
typedef LaikagoEnv Environment;

ContactSimulation<MyAlgebra> m_laikago_sim;

struct laikago_problem {

    /// Constructor from dimension
    laikago_problem()
        //check environment_eval.cpp 
        //int num_params = env.neural_network.num_weights() + env.neural_network.num_biases();
        : m_dim(7409)//
    {
        std::cout << "laikago_problem\n" << std::endl;
    }

    virtual ~laikago_problem()
    {
        std::cout << "~laikago_problem\n" << std::endl;
    }

    
    

    // Fitness computation
    vector_double fitness(const vector_double &x) const
    {
        NeuralNetwork<MyAlgebra> neural_network;
        
        neural_network.set_parameters(x);

        int num_steps = 1000;
        double avg_reward = 0;
        int num_rollouts = 1;
        
        LaikagoEnv env;
        env.init_neural_network(x);

        for(int k=0;k<num_rollouts;k++)
        {
            
            
            auto obs = env.reset();
            env.neural_network.set_parameters(x);
            double total_reward = 0;
            for(int i =0; i< num_steps ; i++)
            {
                
                auto action = env.policy(obs);
                double reward;
                bool  done;
                env.step(action,obs,reward,done);
                total_reward += reward;
                int num_contacts = 0;
                  for (int c=0;c<env.contact_sim.world.mb_contacts_.size();c++)
                  {
                      for (int j=0;j<env.contact_sim.world.mb_contacts_[c].size();j++)
                      {
                        if (env.contact_sim.world.mb_contacts_[c][j].distance<0.01)
                        {
                            num_contacts++;
                        }
                      }
                  }
          
                if(done || num_contacts<3)
                {
                    break;
                }
            }
            avg_reward+=total_reward;
        }
        avg_reward /= double(num_rollouts);
        double total_reward = avg_reward;

        static double max_total_reward = 0;
        if(total_reward > max_total_reward)
        {
            max_total_reward = total_reward;
            std::string x_str;
            for(int i=0;i<x.size();i++)
            {
                x_str = x_str + std::to_string(x[i]) + std::string(",");
            }
            //std::cout << "-----------------\n" << "for " << x_str << std::endl;
            std::cout << "max_total_reward=" << std::to_string(max_total_reward) << "for " << x_str << std::endl;
            //std::cout << "------------------\nmax_total_reward=" << std::to_string(max_total_reward) << "for " << x_str << std::endl;
            std::cout << "max_total_reward=" << std::to_string(max_total_reward) << "------------------\n" << std::endl;
        }

        vector_double f(1,0.);
        f[0] = -total_reward;
        return f;
    }
    // Box-bounds
    std::pair<vector_double,vector_double> get_bounds() const
    {
        vector_double lb(m_dim,-5);
        vector_double ub(m_dim,5);
        return {lb, ub};
    }
    /// Problem name
    /**
     * @return a string containing the problem name
     */
    std::string get_name() const
    {
        return "cartpole_problem Function";
    }
    // Optimal solution
    vector_double best_known() const
    {
        return vector_double(m_dim,95);
    }
    // Object serialization
    template <typename Archive>
    void serialize(Archive &ar,unsigned)
    {
        ar &m_dim;
    }
    /// Problem dimensions
    unsigned m_dim;
};

typedef laikago_problem tds_environment_problem;


#else
#include "environments/cartpole_environment.h"
#include "math/neural_network.hpp"

typedef CartpoleEnv<MyAlgebra> Environment;
//std::vector<double> x={5.236089,11.197456,8.838370,12.149057,-0.488806};



struct cartpole_problem {

    

    /// Constructor from dimension
    cartpole_problem()
        : m_dim(5u)
    {
        std::cout << "cartpole_problem\n" << std::endl;
    }

    virtual ~cartpole_problem()
    {
        std::cout << "~cartpole_problem\n" << std::endl;
    }

    
    

    // Fitness computation
    vector_double fitness(const vector_double &x) const
    {
        NeuralNetwork<MyAlgebra> neural_network;
        
        neural_network.set_parameters(x);

        int num_steps = 1000;
        double avg_reward = 0;
        int num_rollouts = 20;
        
        //CartpoleContactSimulation<MyAlgebra> m_cartpole_sim;

        for(int k=0;k<num_rollouts;k++)
        {
            CartpoleEnv<MyAlgebra> env;
            env.init_neural_network(x);
            
            auto obs = env.reset();
            env.neural_network.set_parameters(x);
            double total_reward = 0;
            for(int i =0; i< num_steps ; i++)
            {
                
                double action = env.policy(obs);
                double reward;
                bool  done;
                env.step(action,obs,reward,done);
                total_reward += reward;
                if(done)
                {
                    break;
                }
            }
            avg_reward+=total_reward;
        }
        avg_reward /= double(num_rollouts);
        double total_reward = avg_reward;

        static double max_total_reward = 0;
        if(total_reward > max_total_reward)
        {
            max_total_reward = total_reward;
            std::string x_str;
            for(int i=0;i<x.size();i++)
            {
                x_str = x_str + std::to_string(x[i]) + std::string(",");
            }
            std::cout << "max_total_reward=" << std::to_string(max_total_reward) << "for " << x_str << std::endl;
        }

        vector_double f(1,0.);
        f[0] = -total_reward;
        return f;
    }
    // Box-bounds
    std::pair<vector_double,vector_double> get_bounds() const
    {
        vector_double lb(m_dim,-5);
        vector_double ub(m_dim,5);
        return {lb, ub};
    }
    /// Problem name
    /**
     * @return a string containing the problem name
     */
    std::string get_name() const
    {
        return "cartpole_problem Function";
    }
    // Optimal solution
    vector_double best_known() const
    {
        return vector_double(m_dim,95);
    }
    // Object serialization
    template <typename Archive>
    void serialize(Archive &ar,unsigned)
    {
        ar &m_dim;
    }
    /// Problem dimensions
    unsigned m_dim;
};

typedef cartpole_problem tds_environment_problem;


#endif








int main()
{

    // 1 - Instantiate a pagmo problem constructing it from a UDP
    // (i.e., a user-defined problem, in this case the 30-dimensional
    // generalised Schwefel test function).

    tds_environment_problem cart;
    problem prob{cart};

    //self-adaptive differential evolution

    // 2 - Instantiate a pagmo algorithm (self-adaptive differential
    // evolution, 100 generations).

    //cmaes(unsigned gen = 1,double cc = -1,double cs = -1,double c1 = -1,double cmu = -1,double sigma0 = 0.5,
    //    double ftol = 1e-6,double xtol = 1e-6,bool memory = false,bool force_bounds = false,
    //    unsigned seed = pagmo::random_device::next());

    cmaes c(100);
    c.set_verbosity(1);
    //sade s(1000);
    //c.set_verbosity(1);
    algorithm algo(c);

    // 3 - Instantiate an archipelago with 16 islands having each 20 individuals.
    archipelago archi{16u, algo, prob, 100u};

    // 4 - Run the evolution in parallel on the 16 separate islands 10 times.
    archi.evolve(100);

    // 5 - Wait for the evolutions to finish.
    archi.wait_check();

    // 6 - Print the fitness of the best solution in each island.
    for(const auto &isl : archi) {

        std::cout << "champion:" << isl.get_population().champion_f()[0] << '\n';
    }
}

