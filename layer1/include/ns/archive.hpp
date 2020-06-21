#ifndef __ARCHIVE_HPP__
#define __ARCHIVE_HPP__

#include <fstream>
#include <sstream>
#include <string>
#include "include/ns/ns.hpp"

namespace archive
{
/*************************************************************************************************************
* Archives
*************************************************************************************************************/
NS::Layer0::qd_t qd;
std::vector<std::shared_ptr<typename NS::Layer0::container_t> > archives = {
        std::make_shared<typename NS::Layer0::container_t>(qd.container()),
        std::make_shared<typename NS::Layer0::container_t>(qd.container()),
        std::make_shared<typename NS::Layer0::container_t>(qd.container()),
        std::make_shared<typename NS::Layer0::container_t>(qd.container()),
        std::make_shared<typename NS::Layer0::container_t>(qd.container()),
        std::make_shared<typename NS::Layer0::container_t>(qd.container())
};

/*************************************************************************************************************
* Load archive (from .dat files)
*************************************************************************************************************/
template <typename Archive>
void load_archive(std::string filename, std::shared_ptr<Archive> archive)
{
        // Get archive dimension
        size_t dim = NS::Params::qd::behav_dim;
        // Read data file
        std::string line;
        std::ifstream f (filename);
        if (f.is_open()) {
                while (getline(f, line)) {
                        // Tokenize line
                        std::stringstream ss(line);
                        std::vector<double> tokens;
                        std::string token;
                        while (getline(ss, token, ' ')) {
                                if (!token.empty()) {
                                        tokens.push_back(std::stod(token));
                                }
                        }
                        // Tokens: "Gen" "Desc1" ... "DescN" "Fitness" "Gen1" ... "GenN"
                        // Create indiviudal
                        typename Archive::indiv_t ind (new typename Archive::indiv_t::element_type);
                        // Set behavior descriptor
                        std::vector<double> descriptor;
                        for (unsigned i = 0; i < dim; ++i) {
                                descriptor.push_back(tokens[i + 1]);
                        }
                        ind->fit().set_desc(descriptor);
                        // Set indiviudal fitness
                        double fitness = tokens[dim + 1];
                        ind->fit().set_value(fitness);
                        // Set indiviudal genotype
                        for (unsigned i = 0; i < ind->gen().size(); ++i) {
                                ind->gen().data(i, tokens[i + dim + 2]);
                        }
                        ind->develop();
                        // Add individual to container
                        archive->add(ind);
                }
                f.close();
        }
}

/*************************************************************************************************************
* Get controllers (Layer 0)
*************************************************************************************************************/
template <typename Archive>
std::tuple<std::vector<double>, std::vector<double> > get_leg_ctrl(Eigen::Vector3d& descriptor, std::shared_ptr<Archive> archive)
{
        typedef typename Archive::indiv_t indiv_t;
        typedef typename Archive::storage_t storage_t;
        // Get storage
        storage_t storage = archive->archive();
        // Find nearest solution in archive
        auto data = storage.nearest(descriptor);
        indiv_t ind = data.second;
        // Get behavior descriptor
        std::vector<double> desc = ind->fit().desc();
        // Return controller
        std::vector<float> gen = ind->gen().data();
        std::vector<double> ctrl (gen.begin(), gen.end());
        return { ctrl, desc };
}
}

#endif
