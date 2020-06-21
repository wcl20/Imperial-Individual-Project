#ifndef __L1_ARCHIVE_HPP__
#define __L1_ARCHIVE_HPP__

#include <fstream>
#include <sstream>
#include <string>
#include <limbo/tools.hpp>
#include "include/qd/qd.hpp"

namespace archive
{
/*************************************************************************************************************
* Archives
*************************************************************************************************************/
// Archive for layer 1
QD::Layer1::qd_t qd;
auto L1_archive = std::make_shared<typename QD::Layer1::container_t>(qd.container());

/*************************************************************************************************************
* Load archive (from .dat files)
*************************************************************************************************************/
template <typename Archive>
void load_grid(std::string filename, std::shared_ptr<Archive> archive)
{
        // Get archive dimension
        size_t dim = Archive::dim;
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
* Get controller (Layer 1)
*************************************************************************************************************/
template <typename Archive>
typename QD::Layer1::container_t::indiv_t get_random_ctrl(std::shared_ptr<Archive> archive)
{
        using namespace limbo;
        const size_t dim = Archive::dim;
        typedef typename Archive::indiv_t indiv_t;
        typedef typename Archive::behav_index_t behav_index_t;
        // Get dimensions of archive
        int size = archive->archive().shape()[0];
        // Create random number generator
        static tools::rgen_int_t generator(0, size - 1);
        // Check random cell until individual is found
        indiv_t ind;
        do {
                // Create random index
                behav_index_t index;
                index[0] = generator.rand();
                index[1] = generator.rand();
                // Get individual in cell
                ind = archive->archive()(index);
        } while(!ind);
        // Return individual
        return ind;
}

template <typename Archive>
typename QD::Layer1::container_t::indiv_t get_ctrl(std::vector<double> descriptor, std::shared_ptr<Archive> archive)
{
        const size_t dim = Archive::dim;
        typedef typename QD::Layer1::phen_t phen_t;
        typedef typename Archive::indiv_t indiv_t;
        typedef typename Archive::behav_index_t behav_index_t;
        typedef boost::multi_array<bool, dim> bool_array_t;
        // Create individual with behavior descriptor
        indiv_t ind (new phen_t);
        ind->fit().set_desc(descriptor);
        // Get dimensions of archive
        int size1 = archive->archive().shape()[0];
        int size2 = archive->archive().shape()[1];
        // Create multiarray to store visited cells
        bool_array_t visited(boost::extents[size1][size2]);
        std::fill_n(visited.data(), visited.num_elements(), false);
        // Get individual in MAP grid with closest behavior
        behav_index_t start = archive->get_index(ind);
        // Create a queue for Breadth First Search
        std::vector<behav_index_t> queue { start };
        // Breadth first search
        while (!queue.empty()) {
                // Get first element from queue
                behav_index_t index = queue.back();
                queue.pop_back();
                // Check index in in bound
                if (visited(index)) continue;
                // Get individual in the cell
                indiv_t ind = archive->archive()(index);
                if (ind) return ind;
                // If there is no individual in cell, continue searching
                visited(index) = true;
                for (int i = index[0] - 1; i <= index[0] + 1; ++i) {
                        for (int j = index[1] - 1; j <= index[1] + 1; ++j) {
                                // Check index inbound
                                if (i < 0 || i >= size1 ||
                                    j < 0 || j >= size2) continue;
                                // Add neighbour to queue
                                behav_index_t neighbour = {{ i, j }};
                                queue.insert(queue.begin(), neighbour);
                        }
                }
        }
}
}

#endif
