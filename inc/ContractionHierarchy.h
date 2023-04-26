#include "Utils/Definitions.h"

class CHState{
public:
    size_t state;

    size_t delet_edge_num = 0;
    size_t add_edge_num = 0;
    size_t height = 0;
    double priority = 0;

    std::vector<Edge> shortcuts;
    bool flag=false;

    CHState(size_t state): state(state) {};

    struct smaller_weight {
        bool operator()(const CHState &a, const CHState &b) const;
    };

};

using CHStatePtr   = std::shared_ptr<CHState>;


struct ContractionOrdering{
    bool operator()(const CHStatePtr &a, const CHStatePtr &b) const;
};


class ContractionHierarchy{
private:
    size_t cnt_edge_diff = 0;
    size_t cnt_witness_search = 0;
    unsigned long long cnt_node_exp = 0;

    clock_t total_time;
    std::vector<size_t> contraction_order;

    void update_priority(size_t state);

    size_t verbal=2;

    std::vector<double> epsilon = {0, 0};

    AdjacencyMatrix graph;
    AdjacencyMatrix inv_graph;

    std::vector<Edge> all_shortcuts;
    std::vector<CHStatePtr> states;


    void delete_state(size_t state);
    void contract_state(size_t state);
    void witness_search(size_t source, size_t target, std::vector<Edge> &edges, size_t mid) ;
    bool witness_search(size_t source, size_t target, Edge &edge) ;
    bool witness_search_lcs(size_t source, size_t target, Edge &edge) ;
    void witness_search_lcs(size_t source, size_t target, std::vector<Edge> &edges) ;
    std::vector<size_t> witness_search_lcs(size_t source, size_t target, std::vector<Edge> &edges, std::vector<Edge> &undetermined, double w) ;
    std::vector<size_t> min_cost_path_w(size_t source, size_t target, double w);

    // update the state, unset its flag etc.
    void edge_difference(size_t state);
    void merge_edges(std::vector<Edge> & edges);


    bool naive = false;
    bool lcs_witness = false;

    // if found a edge that can be merged
    // return True, update new_edge, and update the ptr
    bool can_merge(Edge& new_edge, int & index);


public:
    void set_naive(bool input){naive = input;}
    void set_lcs(bool input){lcs_witness = input;}

    clock_t get_time() {return total_time;}
    clock_t get_num_shortcuts() {return all_shortcuts.size();}

    ContractionHierarchy(size_t graph_size, std::vector<Edge> & edges): graph(graph_size, edges), inv_graph(graph_size, edges, true){};

    void contract(size_t contract_limit=0);

    void write_to(std::string fname);

};
