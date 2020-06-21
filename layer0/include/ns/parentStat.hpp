#ifndef __PARENT_STAT_HPP__
#define __PARENT_STAT_HPP__

#include <numeric>
#include <sferes/stat/stat.hpp>

namespace sferes {
namespace stat {
SFERES_STAT(QdParent, Stat){
public:
        typedef std::vector<boost::shared_ptr<Phen> > archive_t;

        template <typename E> void refresh(const E& ea)
        {
                _container.clear();
                for (auto it = ea.pop().begin(); it != ea.pop().end(); ++it)
                        _container.push_back(*it);

                if (ea.gen() % Params::pop::dump_period == 0)
                        _write_container(std::string("parent_"), ea);
        }

        template <typename EA>
        void _write_container(const std::string& prefix, const EA& ea) const
        {
                std::cout << "writing..." << prefix << ea.gen() << std::endl;
                std::string fname = ea.res_dir() + "/" + prefix
                                    + boost::lexical_cast<std::string>(ea.gen()) + std::string(".dat");

                std::ofstream ofs(fname.c_str());

                size_t offset = 0;
                ofs.precision(17);
                // Use offspring for randomly generated solutions
                // for (auto it = ea.offspring().begin(); it != ea.offspring().end(); ++it) {
                for (auto it = ea.parents().begin(); it != ea.parents().end(); ++it) {

                        ofs << offset << "    ";
                        // Behavior descriptor
                        for (size_t dim = 0; dim < (*it)->fit().desc().size(); ++dim)
                                ofs << (*it)->fit().desc()[dim] << " ";

                        // Fitness
                        ofs << " " << (*it)->fit().value() << "         ";

                        // Controller
                        for (size_t dim = 0; dim < (*it)->size(); ++dim)
                                ofs << (*it)->data(dim) << " ";
                        ofs << std::endl;
                        ++offset;
                }
        }

        template<class Archive>
        void serialize(Archive & ar, const unsigned int version) {
                ar & BOOST_SERIALIZATION_NVP(_container);
        }

        const archive_t& archive() const {
                return _container;
        }
protected:
        archive_t _container;
};
}
}

#endif
