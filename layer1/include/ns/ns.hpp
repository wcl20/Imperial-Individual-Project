#ifndef __NS_HPP__
#define __NS_HPP__

// Genotype
#include <sferes/gen/evo_float.hpp>
// Phenotype
#include <sferes/phen/parameters.hpp>
// Evaluator
#include <sferes/eval/eval.hpp>         // Simple Evaluator
#include <sferes/eval/parallel.hpp>     // Parallel Evaluator
// Statistics
#include <sferes/stat/best_fit.hpp>     // Best Fitness
#include <sferes/stat/mean_fit.hpp>     // Mean Fitness
#include "include/qd/stat.hpp"          // All individuals in the container
#include <sferes/stat/qd_progress.hpp>  // Overall statistics
// Modifiers
#include <sferes/modif/dummy.hpp>
// QD Algorithm
#include <sferes/qd/quality_diversity.hpp>
// QD Algorithm (Fitness)
#include <sferes/fit/fit_qd.hpp>
// QD Algorithm (Selectors)
#include <sferes/qd/selector/population_based.hpp>
#include <sferes/qd/selector/score_proportionate.hpp>
#include <sferes/qd/selector/value_selector.hpp>
// QD Algorithm (Container)
#include <sferes/qd/container/archive.hpp>
// QD Algorithm (Storage)
#include <sferes/qd/container/kdtree_storage.hpp>
#include <sferes/run.hpp>

namespace NS
{
/*************************************************************************************************************
 * Typedefs
 **************************************************************************************************************/
 // Typedefs of Novelty Search algorithm
using namespace sferes;
template <typename Params>
struct NSAlgorithm
{
        // Define Genotype
        typedef gen::EvoFloat<Params::gen_size, Params> gen_t;
        // Define Phenotype
        typedef phen::Parameters<gen_t, typename Params::fit_t, Params> phen_t;
        // Define Evaluator
   #ifdef GRAPHIC
        typedef eval::Eval<Params> eval_t;
   #else
        typedef eval::Parallel<Params> eval_t;
   #endif
        // Define Statistics
        typedef boost::fusion::vector<
                        stat::BestFit<phen_t, Params>,
                        stat::MeanFit<Params>,
                        stat::QdContainer<phen_t, Params>,
                        stat::QdProgress<phen_t, Params> > stat_t;
        // Define Modifier
        typedef modif::Dummy<> modifier_t;
        // Selector
        typedef qd::selector::ScoreProportionate<phen_t, qd::selector::getNovelty, Params> score_t;
        typedef qd::selector::PopulationBased<phen_t, score_t, Params> select_t;
        // Container
        typedef qd::container::KdtreeStorage<boost::shared_ptr<phen_t>, Params::qd::behav_dim> storage_t;
        typedef qd::container::Archive<phen_t, storage_t, Params> container_t;
        typedef qd::QualityDiversity<phen_t, eval_t, stat_t, modifier_t, select_t, container_t, Params> qd_t;
};

/*************************************************************************************************************
 * Common Params
 **************************************************************************************************************/
using namespace sferes::gen::evo_float;
FIT_QD(DummyFitness) {
};
struct Params
{
        // Size of genotype
        SFERES_CONST int gen_size = 3;
        // Define Fitness
        typedef DummyFitness<Params> fit_t;
        struct evo_float
        {
                // we choose the polynomial mutation type
                SFERES_CONST mutation_t mutation_type = polynomial;
                // we choose the polynomial cross-over type
                SFERES_CONST cross_over_t cross_over_type = sbx;
                // the mutation rate of the real-valued vector
                SFERES_CONST float mutation_rate = 0.03f;
                // the cross rate of the real-valued vector
                SFERES_CONST float cross_rate = 0.0f;
                // a parameter of the polynomial mutation
                SFERES_CONST float eta_m = 10.0f;
                // a parameter of the polynomial cross-over
                SFERES_CONST float eta_c = 10.0f;
        };
        struct pop
        {
                // number of initial random points
                SFERES_CONST size_t init_size = 100;
                // size of the population
                SFERES_CONST unsigned size = 2000;
                // number of generations
                SFERES_CONST unsigned nb_gen = 2000;
                // how often should the result file be written (here, every 100 generation)
                SFERES_CONST int dump_period = 100;
        };
        // Novelty Score
        struct nov {
                SFERES_CONST size_t deep = 3;
                SFERES_CONST double l = 0.01;
                SFERES_CONST double k = 15;
                SFERES_CONST double eps = 0.1;
        };
        struct parameters
        {
                // maximum value of the phenotype parameters
                SFERES_CONST float min = -1.0f;
                // minimum value
                SFERES_CONST float max = 1.0f;
        };
        struct qd {
                // Behavior space dimension
                SFERES_CONST size_t behav_dim = 3;
        };
};

using Layer0 = NS::NSAlgorithm<Params>;
}

#endif
