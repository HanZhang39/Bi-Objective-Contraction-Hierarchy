#pragma once

#include <set>
#include <vector>
#include <unordered_map>
#include <list>
#include "Utils/Definitions.h"

template<class Node, class more_than_full_cost>
class MapQueue
{
private:
    std::vector<Node>                heap;
    more_than_full_cost           more_than;

    std::vector<std::list<Node>>   open_map;

public:
    MapQueue(size_t graph_size);
    bool empty();
    // Node top();
    Node pop();
    void insert(Node &pp);
    std::list<Node> &get_open(size_t id);

};


template<class Node, class more_than_full_cost>
class MapQueueHash
{
private:
    std::vector<Node>                heap;
    more_than_full_cost           more_than;

    std::unordered_map<size_t, std::list<Node> >  open_map;

public:
    MapQueueHash();
    bool empty();
    // Node top();
    Node pop();
    void insert(Node &pp);
    std::list<Node> &get_open(size_t id);

};

using NodeQueue = MapQueue<NodePtr, Node::more_than_full_cost>;
