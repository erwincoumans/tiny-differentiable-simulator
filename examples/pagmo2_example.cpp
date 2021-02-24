#include <iostream>

#include <pagmo/algorithm.hpp>
#include <pagmo/algorithms/sade.hpp>
#include <pagmo/algorithms/cmaes.hpp>
#include <pagmo/archipelago.hpp>
#include <pagmo/problem.hpp>
#include <pagmo/problems/schwefel.hpp>

using namespace pagmo;


struct cartpole_problem {
    /// Constructor from dimension
    cartpole_problem(unsigned dim = 1u)
        : m_dim(dim)
    {
    }
    // Fitness computation
    vector_double fitness(const vector_double &x) const
    {
        vector_double f(1,0.);
        auto n = x.size();
        //for(decltype(n) i = 0u; i < n; i++) {
        f[0] = (x[0]-1.)*(x[0]-1.)+100.;
        //}

        //std::cout << "f(" << std::to_string(x[0]) << ")=" << std::to_string(f[0]) <<std::endl;
        
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
    problem prob{cartpole_problem(1)};

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
        std::cout << isl.get_population().champion_f()[0] << '\n';
    }
}

