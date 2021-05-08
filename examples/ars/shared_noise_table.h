#ifndef SHARED_NOISE_TABLE_H
#define SHARED_NOISE_TABLE_H

#include <random>

struct SharedNoiseTable
{
    const std::vector<double> deltas_;
    int seed_;
     // obtain a random number from hardware
    std::mt19937 gen_;
    std::uniform_int_distribution<> distr_;
    int params_dim_;

    SharedNoiseTable(const std::vector<double>& deltas, int seed, int params_dim)
        :deltas_(deltas),
        seed_(seed),
        params_dim_(params_dim)
    {
        std::random_device rd_;
        std::seed_seq seed2;
        // seed the generator
        gen_ = std::mt19937(seed2);//rd_());
        //std::mt19937 gen(rd()); 
        distr_ = std::uniform_int_distribution<>(0, deltas.size()-params_dim_); // define the range
    }
    virtual ~SharedNoiseTable()
    {
    }

    //def get(self, i, dim):
    //    return self.noise[i:i + dim]
    void get(int start_index, double delta_std, std::vector<double>& noise)
    {
        noise.resize(params_dim_);
        for (int i=0;i<params_dim_;i++)
        {
            noise[i] = deltas_[start_index+i]*delta_std;
        }
    }

    //def sample_index(self, dim):
    //    return self.rg.randint(0, len(self.noise) - dim + 1)
    int sample_index()
    {
        return distr_(gen_);
    }

    //    idx = self.sample_index(dim)
    //    return idx, self.get(idx, dim)
    int get_delta(double delta_std, std::vector<double>& delta) {
        int index = sample_index();
        get(index, delta_std, delta);
        return index;
    }
};

#endif //SHARED_NOISE_TABLE_H