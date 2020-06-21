//| This file is a part of the sferes2 framework.
//| Copyright 2016, ISIR / Universite Pierre et Marie Curie (UPMC)
//| Main contributor(s): Jean-Baptiste Mouret, mouret@isir.fr
//|
//| This software is a computer program whose purpose is to facilitate
//| experiments in evolutionary computation and evolutionary robotics.
//|
//| This software is governed by the CeCILL license under French law
//| and abiding by the rules of distribution of free software.  You
//| can use, modify and/ or redistribute the software under the terms
//| of the CeCILL license as circulated by CEA, CNRS and INRIA at the
//| following URL "http://www.cecill.info".
//|
//| As a counterpart to the access to the source code and rights to
//| copy, modify and redistribute granted by the license, users are
//| provided only with a limited warranty and the software's author,
//| the holder of the economic rights, and the successive licensors
//| have only limited liability.
//|
//| In this respect, the user's attention is drawn to the risks
//| associated with loading, using, modifying and/or developing or
//| reproducing the software by the user in light of its specific
//| status of free software, that may mean that it is complicated to
//| manipulate, and that also therefore means that it is reserved for
//| developers and experienced professionals having in-depth computer
//| knowledge. Users are therefore encouraged to load and test the
//| software's suitability as regards their requirements in conditions
//| enabling the security of their systems and/or data to be ensured
//| and, more generally, to use and operate it in the same conditions
//| as regards security.
//|
//| The fact that you are presently reading this means that you have
//| had knowledge of the CeCILL license and that you accept its terms.

#include <iostream>
#include "include/ns/archive.hpp"
#include "include/qd/qd.hpp"

int main(int argc, char **argv)
{
        // Restrict number of CPUs to 48
        tbb::task_scheduler_init init(48);

        // Initialize Robot
        global::hexapod = std::make_shared<robot_dart::Robot>("/git/sferes2/exp/layer1/resources/hexapod_v2.urdf");
        global::hexapod->set_position_enforced(true);
        global::hexapod->set_actuator_types(dart::dynamics::Joint::SERVO);
        global::hexapod->skeleton()->enableSelfCollisionCheck();

        // Load Layer 0 archive
        for (int leg = 0; leg < 6; ++leg) {
                archive::load_archive("/git/sferes2/exp/layer1/data/archive1900/leg" + std::to_string(leg) + ".dat", archive::archives[leg]);
        }

        // If arguments are given, run a speciific controller
        if (argc == QD::Params::gen_size + 1) {
                std::vector<double> ctrl;
                for (int i = 0; i < QD::Params::gen_size; ++i) {
                        ctrl.push_back(std::stod(argv[i + 1]));
                }
                // Run controller
                typename QD::Params::fit_t fit;
                fit.simulate(ctrl, true);
                return 0;
        }

        // Run QD Algorithm
        QD::Layer1::qd_t qd;
        sferes::run_ea(argc, argv, qd);

        // Reset robot
        global::hexapod.reset();
        return 0;
}
