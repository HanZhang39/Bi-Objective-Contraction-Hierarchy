#ifndef UTILS_DEFINITIONS_H
#define UTILS_DEFINITIONS_H

#include <map>
#include <vector>
#include <array>
#include <list>
#include <iostream>
#include <limits>
#include <functional>
#include <memory>
#include <climits>
#include "boost/heap/pairing_heap.hpp"
#include "boost/heap/priority_queue.hpp"


#ifndef DEBUG
#define DEBUG 1
#endif


const size_t MAX_COST = std::numeric_limits<size_t>::max();


template<typename T>
using Pair      = std::array<T, 2>;

template<typename T>
std::ostream& operator<<(std::ostream &stream, const Pair<T> pair) {
    stream << "[" << pair[0] << ", " << pair[1] << "]";
    return stream;
}


using Heuristic = std::function<std::vector<size_t>(size_t)>;

// Structs and classes
struct Edge {
    size_t          source;
    size_t          target;
    std::vector<size_t>    cost;
    std::vector<size_t>    apex;

    Edge(size_t source, size_t target, std::vector<size_t> cost) : source(source), target(target), cost(cost), apex(cost) {}
    Edge(size_t source, size_t target, std::vector<size_t> cost, std::vector<size_t> apex) : source(source), target(target), cost(cost), apex(apex) {}

    Edge inverse() const {
        return Edge(this->target, this->source, this->cost, this->apex);
    }
};

std::ostream& operator<<(std::ostream &stream, const Edge &edge);


// Graph representation as adjacency matrix
class AdjacencyMatrix {
private:
    size_t                         graph_size;
    size_t num_of_objectives = 0;

public:


    // TODO bad design!
    std::vector<std::vector<Edge>> matrix;


    AdjacencyMatrix() = default;
    AdjacencyMatrix(size_t graph_size, std::vector<Edge> &edges, bool inverse=false);
    void add(Edge edge);
    size_t size(void) const;
    size_t get_num_of_objectives() const;
    const std::vector<Edge>& operator[](size_t vertex_id) const;
  
    friend std::ostream& operator<<(std::ostream &stream, const AdjacencyMatrix &adj_matrix);

    void update_edges(size_t state, std::vector<Edge>& edges){
        matrix[state] = edges;
    }

    void update_edge(size_t state, int idx, Edge edge){
        assert(idx < matrix[state].size());
        matrix[state][idx] = edge;
    }

    void add_edge(Edge& edge){
        matrix[edge.source].push_back(edge);
    }
};


struct Node;
struct PathPair;
struct ApexPathPair;
using NodePtr       = std::shared_ptr<Node>;
using PathPairPtr   = std::shared_ptr<PathPair>;
using ApexPathPairPtr   = std::shared_ptr<ApexPathPair>;
using SolutionSet   = std::vector<NodePtr>;
using PPSolutionSet = std::vector<PathPairPtr>;
using ApexPathSolutionSet = std::vector<ApexPathPairPtr>;


struct NodePE;
using NodePEPtr       = std::shared_ptr<NodePE>;

using EPS = std::vector<double>;

struct Node {
    size_t          id;
    std::vector<size_t>    g;
    std::vector<size_t>    h;
    std::vector<size_t>    f;
    NodePtr         parent;

    Node(size_t id, std::vector<size_t> g, std::vector<size_t> h, NodePtr parent=nullptr)
        : id(id), g(g), h(h), f(g.size()), parent(parent) {
        for (int i = 0; i < g.size(); i++){
            f[i] = g[i] + h[i];
        }
    };

    void update_h(std::vector<size_t> new_h){
        h = new_h;
        for (int i = 0; i < g.size(); i++){
            f[i] = g[i] + h[i];
        }
    }

    struct more_than_specific_heurisitic_cost {
        size_t cost_idx;

        more_than_specific_heurisitic_cost(size_t cost_idx) : cost_idx(cost_idx) {};
        bool operator()(const NodePtr &a, const NodePtr &b) const;
    };

    struct more_than_combined_heurisitic {
        double factor;

        more_than_combined_heurisitic(double factor) : factor(factor) {};
        bool operator()(const NodePtr &a, const NodePtr &b) const;
    };

    struct more_than_full_cost_alpha_beta {
        double alpha;
        double beta;
        more_than_full_cost_alpha_beta(std::pair<double, double> alpha_beta):alpha(alpha_beta.first), beta(alpha_beta.second){};
        bool operator()(const NodePtr &a, const NodePtr &b) const;
    };

    struct more_than_full_cost_w {
        double w;
        more_than_full_cost_w(double w):w(w){};
        bool operator()(const NodePtr &a, const NodePtr &b) const{
            return (w*a->f[0] + (1-w)*a->f[1]  > w*b->f[0] + (1-w) * b->f[1]);
        }
    };

    struct more_than_full_cost {
        bool operator()(const NodePtr &a, const NodePtr &b) const;
    };

    enum LEX_ORDER {LEX0, LEX1};
    struct more_than_lex{
        Node::LEX_ORDER order;
        more_than_lex(Node::LEX_ORDER order) : order(order) {};
        bool operator()(const NodePtr &a, const NodePtr &b) const;
    };


    struct compare_lex1
    {
        bool operator()(const NodePtr n1, const NodePtr n2) const
        {
            if (n1->f[0] != n2->f[0]){
                return n1->f[0] > n2->f[0];
            }
            return n1->f[1] > n2->f[1];
        }
    };
    friend std::ostream& operator<<(std::ostream &stream, const Node &node);
};

struct NodePE{
    size_t          id;
    std::vector<size_t>    g;
    std::vector<size_t>    h;
    std::vector<size_t>    f;
    NodePEPtr         parent;
    size_t state_idx;
    size_t cost_idx;

    NodePE(size_t id, std::vector<size_t> g, std::vector<size_t> h, NodePEPtr parent=nullptr, size_t state_idx=0, size_t cost_idx=0):
        id(id), g(g), h(h), f(g.size()), parent(parent),
        state_idx(state_idx), cost_idx(cost_idx)
    {
        for (int i = 0; i < g.size(); i++){
            f[i] = g[i] + h[i];
        }
    };

    struct more_than_full_cost {
        bool operator()(const NodePEPtr &a, const NodePEPtr &b) const {
            for (size_t i = 0; i + 1 < a->f.size(); i++){
                if (a->f[i] != b->f[i]) {
                    return (a->f[i] > b->f[i]);
                }
            }
            return (a->f.back() > b->f.back());

        }
    };

};



bool is_bounded(NodePtr apex, NodePtr node,  const EPS eps);
bool is_bounded(NodePtr apex, NodePtr node);
bool is_dominated_dr(NodePtr apex, NodePtr node);
bool is_dominated_dr(NodePtr apex, NodePtr node, const EPS eps);


typedef boost::heap::priority_queue<NodePtr , boost::heap::compare<Node::compare_lex1> > heap_open_t;


#endif //UTILS_DEFINITIONS_H
