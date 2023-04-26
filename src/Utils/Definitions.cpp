#include <iostream>
#include <set>
#include <string>
#include "Utils/Definitions.h"

bool is_bounded(NodePtr apex, NodePtr node){
  for (int i = 0; i < apex->f.size(); i ++ ){
    if (node->f[i] > apex->f[i]){
      return false;
    }
  }
  return true;
}


AdjacencyMatrix::AdjacencyMatrix(size_t graph_size, std::vector<Edge> &edges, bool inverse)
    : matrix((graph_size+1), std::vector<Edge>()), graph_size(graph_size) {

  num_of_objectives = edges[0].cost.size();

    for (auto iter = edges.begin(); iter != edges.end(); ++iter) {
        if (inverse) {
            this->add(iter->inverse());
        } else {
            this->add((*iter));
        }
    }
}



size_t AdjacencyMatrix::get_num_of_objectives() const {
  return num_of_objectives;
}

void AdjacencyMatrix::add(Edge edge) {
    (this->matrix[edge.source]).push_back(edge);
}


size_t AdjacencyMatrix::size() const {return this->graph_size;}


const std::vector<Edge>& AdjacencyMatrix::operator[](size_t vertex_id) const {
    return this->matrix.at(vertex_id);
}


std::ostream& operator<<(std::ostream &stream, const AdjacencyMatrix &adj_matrix) {
    size_t  i = 0;

    stream << "{\n";
    for (auto vertex_iter = adj_matrix.matrix.begin(); vertex_iter != adj_matrix.matrix.end(); ++vertex_iter) {
        stream << "\t\"" << i++ << "\": [";

        std::vector<Edge> edges = *vertex_iter;
        for (auto edge_iter = edges.begin(); edge_iter != edges.end(); ++edge_iter) {
            stream << "\"" << edge_iter->source << "->" << edge_iter->target << "\", ";
        }

        stream << "],\n";
    }
    stream << "}";
    return stream;
}

std::ostream& operator<<(std::ostream &stream, const Edge &edge) {
    // Printed in JSON format
    stream
        << "{"
        <<  "\"edge_source\": " << edge.source << ", "
        <<  "\"edge_target\": " << edge.target << ", "
        <<  "\"edge_cost\": ";

    for (auto c: edge.cost){
      stream << c << ", ";
    }
    stream << "}";

    return stream;
}


bool Node::more_than_specific_heurisitic_cost::operator()(const NodePtr &a, const NodePtr &b) const {
    return (a->h[cost_idx] > b->h[cost_idx]);
}

bool Node::more_than_combined_heurisitic::operator()(const NodePtr &a, const NodePtr &b) const {
  return (a->g[0] + this->factor * a->g[1] > b->g[0] + this->factor * b->g[1]);
}


bool Node::more_than_full_cost::operator()(const NodePtr &a, const NodePtr &b) const {
  for (int i = 0; i + 1 < a->f.size(); i++){
    if (a->f[i] != b->f[i]) {
      return (a->f[i] > b->f[i]);
    }
  }
  return (a->f.back() > b->f.back());
}

bool Node::more_than_full_cost_alpha_beta::operator()(const NodePtr &a, const NodePtr &b) const {
  if (alpha * a->f[0] + (1 - alpha) * a->f[1] != alpha * b->f[0] + (1 - alpha) * b->f[1]){
    return alpha * a->f[0] + (1 - alpha) * a->f[1] > alpha * b->f[0] + (1 - alpha) * b->f[1];
  }
  return beta * a->f[1] + (1 - beta) * a->f[0] > beta * b->f[1] + (1 - beta) * b->f[0];
}

bool Node::more_than_lex::operator()(const NodePtr &a, const NodePtr &b) const {
  if (order == Node::LEX_ORDER::LEX0){
    if (a->f[0] != b->f[0]) {
      return (a->f[0] > b->f[0]);
    } else {
      return (a->f[1] > b->f[1]);
    }
  }else{
      if (a->f[1] != b->f[1]) {
        return (a->f[1] > b->f[1]);
    } else {
        return (a->f[0] > b->f[0]);
    }
  }
}


std::ostream& operator <<(std::ostream &stream, const std::vector<size_t> &vec){
  stream << "[";
  for (size_t i = 0 ;  i < vec.size(); i ++){
    stream << vec[i];
    if (i + 1 <vec.size()){
      stream << ", ";
    }
  }
  stream << "]";
  return stream;
}

std::ostream& operator<<(std::ostream &stream, const Node &node) {
    // Printed in JSON format
    std::string parent_id = node.parent == nullptr ? "-1" : std::to_string(node.parent->id);
    stream
        << "{"
        <<      "\"id\": " << node.id << ", "
        <<      "\"parent\": " << parent_id << ", "
        <<      "\"cost_until_now\": " << node.g << ", "
        <<      "\"heuristic_cost\": " << node.h << ", "
        <<      "\"full_cost\": " << node.f
        << "}";
    return stream;
}
