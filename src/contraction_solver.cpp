
#include <iostream>
#include <memory>
#include <time.h>
#include <fstream>

#include "Utils/IOUtils.h"
#include "CH_solver.h"

#include <boost/program_options.hpp>
#include<boost/tokenizer.hpp>

using namespace std;

std::string alg_variant = "";


// Simple example to demonstarte the usage of the algorithm

int main(int argc, char** argv){
    namespace po = boost::program_options;

    std::vector<string> objective_files;
    std::vector<double> eps_vec;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("output,o", po::value<std::string>()->required(), "Name of the output file")
        ("input,i", po::value<std::string>()->required(), "Name of the input file")
        ("start,s", po::value<size_t>()->default_value(0), "start location")
        ("goal,g", po::value<size_t>()->default_value(0), "goal location")
        ("query,q", po::value<std::string>()->default_value(""), "query file")
        ("eps,e", po::value<std::vector<double>>(&eps_vec)->multitoken(), "approximation factor")
        ("partial" , po::value<bool>()->default_value(true), "using partial exp")
        ("timelimit,t", po::value<size_t>()->default_value(300), "time limit for each query")
        ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    po::notify(vm);
    srand((int)time(0));

    // Load files

    CHGraphRT chg(vm["input"].as<std::string>());

    std::cout << "graph loaded" << std::endl;

    std::vector<std::pair<size_t, size_t>> queries;
    if (vm["query"].as<std::string>() != ""){

        std::cout << "run query file: " << vm["query"].as<std::string>() << std::endl;
        if (load_queries(vm["query"].as<std::string>(), queries) == false) {
            std::cout << "Failed to load queries file" << std::endl;
            return -1;
        }
    } else{
        size_t start = vm["start"].as<size_t>();
        size_t goal = vm["goal"].as<size_t>();

        queries.push_back({start, goal});

    }

    std::ofstream stats;
    stats.open(vm["output"].as<std::string>(), std::fstream::app);
    for (auto & query: queries){

        size_t start = query.first;
        size_t goal = query.second;


        SolutionSet sols;

        if (! vm["partial"].as<bool>()){
            BOAStarCH boach(chg);
            boach(start, goal,sols, vm["timelimit"].as<size_t>());
            std::cout << "time:" << (double)boach.total_time/CLOCKS_PER_SEC << std::endl;
            std::cout << "sols:" << sols.size() << std::endl;
            std::cout << "exp:" << boach.num_expansion << std::endl;

            stats << boach.get_solver_name() << "\t"
                  << start << "\t" << goal<< "\t"
                  << boach.num_generation << "\t"
                  << boach.num_expansion << "\t"
                  << sols.size() << "\t"
                  << (double) boach.total_time/CLOCKS_PER_SEC << "\t"
                  << (double) boach.preprocess_time/CLOCKS_PER_SEC
                  << std::endl;
        } else {
            std::cout << "partial expansion;" << std::endl;
            BOAStarCHPartial boach(chg);
            boach(start, goal,sols, vm["timelimit"].as<size_t>());
            std::cout << "time:" << (double)boach.total_time/CLOCKS_PER_SEC << std::endl;
            std::cout << "sols:" << sols.size() << std::endl;
            std::cout << "exp:" << boach.num_expansion << std::endl;

            stats << boach.get_solver_name() << "\t"
                  << start << "\t" << goal<< "\t"
                  << boach.num_generation << "\t"
                  << boach.num_expansion << "\t"
                  << sols.size() << "\t"
                  << (double) boach.total_time/CLOCKS_PER_SEC << "\t"
                  << (double) boach.preprocess_time/CLOCKS_PER_SEC
                  << std::endl;
        }
    }

    return 0;
}
