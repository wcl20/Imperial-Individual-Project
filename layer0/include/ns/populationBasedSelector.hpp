#ifndef __POPULATION_BASED_SELECTOR_HPP__
#define __POPULATION_BASED_SELECTOR_HPP__

#include <sferes/qd/selector/score_proportionate.hpp>

namespace sferes {
    namespace qd {
        namespace selector {
            template <typename Phen, typename Selector, typename Params>
            struct PopulationBased {

                typedef boost::shared_ptr<Phen> indiv_t;

                template <typename EA>
                void operator()(std::vector<indiv_t>& pop, const EA& ea) const
                {
                    std::vector<indiv_t> temp;

                    for (auto& indiv : ea.parents()) {
                        if (!indiv->fit().dead()) {
                          temp.push_back(indiv);
                        }
                    }
                    for (auto& indiv : ea.offspring()) {
                        if (!indiv->fit().dead()) {
                          temp.push_back(indiv);
                        }
                    }
                    _selector(pop, temp);
                }
                Selector _selector;
            };
        } // namespace selector
    } // namespace qd
} // namespace sferes
#endif
