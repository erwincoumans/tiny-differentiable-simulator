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

#include "environments/cartpole_environment.h"



CartpoleContactSimulation<MyAlgebra> m_cartpole_sim;

struct cartpole_problem {

    

    /// Constructor from dimension
    cartpole_problem(unsigned dim = 5u)
        : m_dim(dim)
    {
        std::cout << "cartpole_problem\n" << std::endl;
    }

    virtual ~cartpole_problem()
    {
        std::cout << "~cartpole_problem\n" << std::endl;
    }

    double policy(const vector_double &x,const std::vector<double>& obs) const
    {
        double action = 0;

        for(int i=0;i<4;i++)
        {
            action+=x[i]*obs[i];
        }
        action+=x[4];
        if(action<-1)
            action=-1;
        if(action>1)
            action=1;
        return action;
    }

    // Fitness computation
    vector_double fitness(const vector_double &x) const
    {
        
        int num_steps = 1000;
        double avg_reward = 0;
        int num_rollouts = 20;
        for(int k=0;k<num_rollouts;k++)
        {
            CartpoleEnv<MyAlgebra> env;//(m_cartpole_sim);
            auto obs = env.reset();

            double total_reward = 0;
            for(int i =0; i< num_steps ; i++)
            {
                double action = 10.*policy(x,obs);
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





int main()
{

    // 1 - Instantiate a pagmo problem constructing it from a UDP
    // (i.e., a user-defined problem, in this case the 30-dimensional
    // generalised Schwefel test function).

    cartpole_problem cart( 5);
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
    archipelago archi{1u, algo, prob, 100u};

    // 4 - Run the evolution in parallel on the 16 separate islands 10 times.
    archi.evolve(100);

    // 5 - Wait for the evolutions to finish.
    archi.wait_check();

    // 6 - Print the fitness of the best solution in each island.
    for(const auto &isl : archi) {

        std::cout << "champion:" << isl.get_population().champion_f()[0] << '\n';
    }
}

