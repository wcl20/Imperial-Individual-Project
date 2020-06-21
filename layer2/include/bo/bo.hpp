#ifndef __BO_HPP__
#define __BO_HPP__

#include <limbo/limbo.hpp>
#include "include/bo/optimizer.hpp"
#include <fstream>
#include <sstream>
#include <string>

namespace BO
{
using namespace limbo;
/*************************************************************************************************************
* Params
*************************************************************************************************************/
struct Params {
        // Model
        struct model {
                BO_PARAM(size_t, dim_in, 1);
                BO_PARAM(size_t, dim_out, 1);
        };

        // Optimizer
        struct bayes_opt_boptimizer : public defaults::bayes_opt_boptimizer {};

        struct bayes_opt_bobase : public defaults::bayes_opt_bobase {
                BO_PARAM(int, stats_enabled, true);
        };

        struct opt_gridsearch : public defaults::opt_gridsearch {};

        // Stop criterion
        struct stop_maxiterations {
                BO_PARAM(int, iterations, 100);
        };

        // Acquisition function
        struct acqui_ucb : public defaults::acqui_ucb {
                BO_PARAM(double, alpha, 0.2);
        };

        // Kernel
        struct kernel_exp {
                BO_PARAM(double, sigma_sq, 1.0);
                BO_PARAM(double, l, 0.2);
        };

        struct kernel : public defaults::kernel {
                BO_PARAM(double, noise, 0.1);
        };

        // Stats
        struct stat_gp {
                BO_PARAM(size_t, bins, 100);
        };
};

/*************************************************************************************************************
* Prior
*************************************************************************************************************/
template <typename Params>
struct MeanModel : mean::BaseMean<Params> {
        MeanModel(size_t dim_out = 1) {
        }

        template <typename GP>
        Eigen::VectorXd operator()(const Eigen::VectorXd& x, const GP&) const
        {
                return x;
        }
};

/*************************************************************************************************************
* Typedefs
*************************************************************************************************************/
// Gaussian Process
using kernel_t = kernel::Exp<Params>;
using mean_t = MeanModel<Params>;
using model_t = model::GP<Params, kernel_t, mean_t>;

using acqui_t = acqui::UCB<Params, model_t>;
using init_t = init::NoInit<Params>;
using stat_t = boost::fusion::vector<stat::GP<Params>>;
bayes_opt::Optimizer<Params,
                     initfun<init_t>,
                     modelfun<model_t>,
                     acquifun<acqui_t>,
                     statsfun<stat_t> > optimizer("/git/sferes2/exp/layer2/results/temp");
}

#endif
