#ifndef ARS_CONFIG_H
#define ARS_CONFIG_H

struct ARSConfig {
    int rollout_length_train_{3000};
    int rollout_length_eval_{3000};
    double delta_std_{0.03};
    double sgd_step_size { 0.02};
    //what size is reasonable?
    int deltas_count{25000000}; //250000
    int env_seed{12345};
    int batch_size{128};//number of parallel rollouts/robots
    int eval_interval{10};
    int num_iter{1024*1024};
    bool auto_reset_when_done{false};
};
    
#endif //ARS_CONFIG_H