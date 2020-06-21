#ifndef __OPTIMIZER_HPP__
#define __OPTIMIZER_HPP__

#include <limbo/limbo.hpp>

namespace limbo
{
namespace bayes_opt {

template <class Params,
          class A1 = boost::parameter::void_,
          class A2 = boost::parameter::void_,
          class A3 = boost::parameter::void_,
          class A4 = boost::parameter::void_,
          class A5 = boost::parameter::void_,
          class A6 = boost::parameter::void_>
class Optimizer : public BOptimizer<Params, A1, A2, A3, A4, A5, A6> {
public:

using base_t = BOptimizer<Params, A1, A2, A3, A4, A5, A6>;
using model_t = typename base_t::model_t;
using acquisition_function_t = typename base_t::acquisition_function_t;
using acqui_optimizer_t = typename base_t::acqui_optimizer_t;

// Constructors
Optimizer() {
}
Optimizer(std::string filepath) {
        // Remove directory created by base
        std::string directory = tools::hostname() + "_" + tools::date() + "_" + tools::getpid();
        boost::filesystem::path path(directory);
        boost::filesystem::remove(path);
        // Create directory to filepath
        boost::filesystem::path new_path(filepath);
        boost::filesystem::create_directory(new_path);
        this->_res_dir = filepath;
}

template <typename AggregatorFunction = FirstElem>
Eigen::VectorXd sample(const AggregatorFunction& afun = AggregatorFunction(), bool reset = false)
{
        this->_init(NULL, afun, reset);

        // Get Model
        if (!this->_observations.empty())
                this->_model.compute(this->_samples, this->_observations);
        else
                this->_model = model_t(Params::model::dim_in(), Params::model::dim_out());

        // Acquisition function optimizer
        acqui_optimizer_t acqui_optimizer;

        // Acqusition function
        acquisition_function_t acqui(this->_model, this->_current_iteration);

        // Get best sample point
        auto acqui_optimization =
                [&](const Eigen::VectorXd& x, bool g) {
                        return acqui(x, afun, g);
                };
        Eigen::VectorXd starting_point = tools::random_vector(Params::model::dim_in(), Params::bayes_opt_bobase::bounded());
        Eigen::VectorXd new_sample = acqui_optimizer(acqui_optimization, starting_point, Params::bayes_opt_bobase::bounded());

        return new_sample;
}

template <typename AggregatorFunction = FirstElem>
void update(const Eigen::VectorXd& s, const Eigen::VectorXd& v, const AggregatorFunction& afun = AggregatorFunction())
{
        // Evaluate new sample point
        this->add_new_sample(s, v);
        this->_update_stats(*this, afun);

        // Remove oldest samples
        if (this->_samples.size() > 5) {
                this->_samples.erase(this->_samples.begin());
                this->_observations.erase(this->_observations.begin());
        }

        this->_total_iterations++;
}


};
}
}

#endif
