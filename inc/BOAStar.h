#ifndef BI_CRITERIA_BOA_STAR_H
#define BI_CRITERIA_BOA_STAR_H

#include <vector>
#include "Utils/Definitions.h"
#include "Utils/Logger.h"
#include "AbstractSolver.h"
#include <set>


class BOAStar: public AbstractSolver {
protected:
    std::clock_t start_time;

    std::vector<std::pair<std::clock_t, NodePtr>> solution_log;
    void log_solution(NodePtr);

public:
    virtual std::string get_solver_name() {return "BOA*"; }

    BOAStar(const AdjacencyMatrix &adj_matrix, Pair<double> eps, const LoggerPtr logger=nullptr);

    void operator()(size_t source, size_t target, Heuristic &heuristic, SolutionSet &solutions, unsigned int time_limit=UINT_MAX);

    std::vector<std::pair<std::clock_t, NodePtr>> get_sol_log(){return solution_log;}
};


class BOD: public BOAStar {
protected:
public:
    std::string get_solver_name() override{
        return "BOD";
    }
    BOD(const AdjacencyMatrix &adj_matrix,  const LoggerPtr logger=nullptr): BOAStar(adj_matrix, {0,0}, logger) {};


    virtual void operator()(size_t source, size_t target, Heuristic &heuristic, SolutionSet &solutions, unsigned int time_limit=UINT_MAX) override {
        std::cout << "this function is not implemented for DPex" << std::endl;
    }
    void operator()(size_t source, std::vector<SolutionSet> &solutions, unsigned int time_limit=UINT_MAX) override;

};


bool is_dominated(std::set<std::pair<int, int>>& pair_set, std::pair<int, int> vec);

#endif //BI_CRITERIA_BOA_STAR_H
