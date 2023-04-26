#include <iostream>
#include <memory>
#include <time.h>
#include <fstream>

#include "Utils/IOUtils.h"
#include "ContractionHierarchy.h"

#include <boost/program_options.hpp>
#include<boost/tokenizer.hpp>

using namespace std;

int main(int argc, char** argv){
    namespace po = boost::program_options;

    std::vector<string> objective_files;

    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("output,o", po::value<std::string>()->required(), "Name of the output file")
        ("map,m",po::value< std::vector<string> >(&objective_files)->multitoken(), "files for edge weight")
        ("approach,a", po::value<std::string>()->default_value("batched"), "approach for preprocess: basic, batched, or supportpoint.")
        ("limit,l", po::value<double>()->default_value(0), "contraction limit")
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
    size_t graph_size;
    std::vector<Edge> edges;

    for (auto file:objective_files){
        std::cout << file << std::endl;
    }

    if (load_gr_files(objective_files, edges, graph_size) == false) {
        std::cout << "Failed to load gr files" << std::endl;
        return -1;
    }

    for (int i = 0; i < edges.size(); i++){
        assert(edges[i].apex.size() == 2);
    }

    std::cout << "Graph Size: " << graph_size << std::endl;


    std::ofstream stats;
    stats.open("contraction_log.txt", std::fstream::app);
    ContractionHierarchy ch(graph_size, edges);

    if (vm["approach"].as<std::string>() == "batched"){
    } else if (vm["approach"].as<std::string>() == "basic"){
        ch.set_naive(true);
    } else if (vm["approach"].as<std::string>() == "supportpoint"){
        ch.set_lcs(true);
    } else {
        std::cerr << "unknown approach: " << vm["approach"].as<std::string>() << std::endl;
        exit(1);
    }



    if (vm["limit"].as<double>() == 0){
        std:: cout << "full contraction" << std::endl;
        ch.contract();
    } else {
        size_t contract_limit = vm["limit"].as<double>() * graph_size;
        std:: cout << "partial contraction: " << contract_limit << std::endl;
        ch.contract(contract_limit);
    }

    for (auto s : objective_files){
        std::vector<std::string> decomposed_line;
        split_string(s, "/", decomposed_line);
        stats << decomposed_line.back() << "-";
    }
    stats << "\t" ;
    ch.write_to(vm["output"].as<std::string>());
    stats << "\t" ;
    stats << ch.get_time() / CLOCKS_PER_SEC << "\t" ;
    stats << ch.get_num_shortcuts() << "\t" << vm["limit"].as<double>();
    stats<< "\n" ;


    return 0;
}
