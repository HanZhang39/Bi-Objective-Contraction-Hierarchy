#include "CH_solver.h"
#include "Utils/IOUtils.h"
#include <fstream>
#include <boost/unordered_map.hpp>


CHGraphRT::CHGraphRT(std::string fname, size_t num_obj){
    this->num_obj = num_obj;

    std::ifstream  file(fname.c_str());

    // assert (file.is_open() == false);


    std::string line;

    size_t graph_size, contraction_order_size;
    std::getline(file, line);

    std::vector<std::string> decomposed_line_1;
    split_string(line, "\t", decomposed_line_1);
    if (decomposed_line_1.size() == 1){
        std::cout << "loading old version file...\n";
        graph_size = std::stoul(decomposed_line_1[0]);
        contraction_order_size = std::stoul(decomposed_line_1[0]);
    } else {
        graph_size = std::stoul(decomposed_line_1[1]);
        contraction_order_size = std::stoul(decomposed_line_1[0]);
    }


    std::cout << "graph size:" << graph_size << std::endl;
    std::cout << "contract size:" << contraction_order_size << std::endl;

    state_to_order.resize(graph_size, graph_size + 10);

    for (size_t i = 0; i < contraction_order_size; i++){
        std::getline(file, line);

        if (line == "") {
            break;
        } else if (line[0] == '#') {
            continue; // Commented out queries
        }

        std::vector<std::string> decomposed_line;
        split_string(line, "\t", decomposed_line);
        size_t node_id = std::stoul(decomposed_line[0]);
        state_to_order[node_id] = i + 1;

    }

    std::cout << state_to_order.size() << std::endl;

    std::getline(file, line);
    size_t shortcut_num = std::stoul(line);
    std::cout << "shortcuts:" << shortcut_num << std::endl;


    // std::vector<Edge> up_edges;
    // std::vector<Edge> down_edges;
    // std::vector<Edge> equal_edges;
    boost::unordered_map<std::pair<size_t, size_t>, MultiCostEdge> all_edges;

    for (size_t i = 0; i < shortcut_num; i++){
        std::getline(file, line);

        if (line == "") {
            break;
        } else if (line[0] == '#') {
            continue; // Commented out queries
        }

        std::vector<std::string> decomposed_line;
        split_string(line, "\t", decomposed_line);
        if (decomposed_line.size() < 6){
            std::cout << "errors" << i << std::endl;
        }
        size_t state_from = std::stoul(decomposed_line[0]);
        size_t state_to = std::stoul(decomposed_line[1]);

        std::vector<size_t> cost;
        std::vector<size_t> apex;
        for (size_t j = 0 ; j < num_obj; j++){
            cost.push_back(std::stoul(decomposed_line[2 + j]));
        }
        for (size_t j = 0 ; j < num_obj; j++){
            apex.push_back(std::stoul(decomposed_line[2 + num_obj + j]));
        }
        // Edge e(state_from, state_to, cost, apex);

        if ( all_edges.find({state_from, state_to}) == all_edges.end() ){
            all_edges[{state_from, state_to}] = MultiCostEdge(state_from, state_to, cost);
        } else {
            all_edges[{state_from, state_to}].insert(cost);
        }

        // if (state_to_order[state_to] > state_to_order[state_from]){
        //     up_edges.push_back(e);
        // } else if (state_to_order[state_to] < state_to_order[state_from]){
        //     down_edges.push_back(e);
        // } else{
        //     equal_edges.push_back(e);
        // }

    }

    up_graph.resize(graph_size + 1);
    down_graph.resize(graph_size + 1);
    up_inv_graph.resize(graph_size + 1);
    down_inv_graph.resize(graph_size + 1);

    for (auto it: all_edges){
        MultiCostEdge me = it.second;
        me.sort();
        size_t state_from = me.source;
        size_t state_to = me.target;
        if (state_to_order[state_to] > state_to_order[state_from]){
            up_graph[state_from].push_back(me);
            up_inv_graph[state_to].push_back(me.inverse());
        } else if (state_to_order[state_to] < state_to_order[state_from]){
            down_graph[state_from].push_back(me);
            down_inv_graph[state_to].push_back(me.inverse());
        } else{
            up_graph[state_from].push_back(me);
            down_inv_graph[state_to].push_back(me.inverse());
            // equal_edges[state_from].push_back(it.second);
        }
    }


    // up_graph = AdjacencyMatrix(graph_size, up_edges);
    // up_inv_graph = AdjacencyMatrix(graph_size, up_edges, true);

    // down_graph = AdjacencyMatrix(graph_size, down_edges);
    // down_inv_graph = AdjacencyMatrix(graph_size, down_edges, true);

    // if (equal_edges.size() != 0){
    //     std::cout << "equal weight edge: " << equal_edges.size() << std::endl;
    // }

    // for (const auto& edge:equal_edges){
    //     up_graph.add(edge);
    //     down_inv_graph.add(edge.inverse());
    // }

}



std::unordered_set<size_t> CHGraphRT::all_up_states(size_t state){
    std::unordered_set<size_t> closed;

    std::list<size_t> open;
    open.push_back(state);
    closed.insert(state);

    while (!open.empty()){
        size_t curr = open.back();
        open.pop_back();

        for (const auto&e:up_graph[curr]){
            if ( closed.find(e.target) == closed.end() ){
                closed.insert(e.target);
                open.push_back(e.target);
            }
        }
    }

    return closed;
}
std::unordered_set<size_t> CHGraphRT::all_up_states_r(size_t state){
    std::unordered_set<size_t> closed;

    std::list<size_t> open;
    open.push_back(state);
    closed.insert(state);

    while (!open.empty()){
        size_t curr = open.back();
        open.pop_back();

        for (const auto&e : down_inv_graph[curr]){
            if ( closed.find(e.target) == closed.end() ){
                closed.insert(e.target);
                open.push_back(e.target);
            }
        }
    }
    return closed;
}


CHShortestPathHeuristic::CHShortestPathHeuristic(size_t source,
                                                 const CHGraphRT& chg,
                                                 const std::unordered_set<size_t>& up_states,
                                                 const std::unordered_set<size_t>& down_states)
    : source(source)  {
    num_obj = chg.get_num_of_objectives();

    for (size_t j=0; j < num_obj; j ++){
        data.push_back(std::unordered_map<size_t,size_t>());
        compute(j, chg, up_states, down_states);
    }
}



// Implements Dijkstra shortest path algorithm per cost_idx cost function
void CHShortestPathHeuristic::compute(size_t cost_idx, const CHGraphRT& chg, const std::unordered_set<size_t>& up_states, const std::unordered_set<size_t>& down_states){
    // Init all heuristics to MAX_COST

    // Init open heap
    std::vector<std::pair<size_t, size_t>> open;
    data[cost_idx][source] = 0;
    open.push_back({0, source});
    auto order = std::greater<std::pair<size_t, size_t>>();
    std::push_heap(open.begin(), open.end(), order);

    std::unordered_map<size_t, size_t> generated;
    std::unordered_set<size_t> closed;
    size_t previous = 0;

    while (open.empty() == false) {
        // Pop min from queue and process
        std::pop_heap(open.begin(), open.end(), order);
        std::pair<size_t,size_t> curr = open.back();
        open.pop_back();
        size_t cost = curr.first;
        size_t node_id = curr.second;
        assert(cost >= previous);
        previous = cost;

        if (closed.find(node_id) != closed.end()){
            continue;
        }

        closed.insert(node_id);
        data[cost_idx][node_id] = cost;

        // Check to which neighbors we should extend the paths
        if (down_states.find(node_id) != down_states.end()){
            const std::vector<MultiCostEdge> &outgoing_edges = chg.down_inv_edges(node_id);
            for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
                size_t edge_cost;
                if (cost_idx == 0){
                    edge_cost = p_edge->costs.front().first;
                } else {
                    
                    edge_cost = p_edge->costs.back().second;
                }

                size_t ch_id = p_edge->target;
                if (up_states.find(ch_id) == up_states.end() && down_states.find(ch_id) == down_states.end()){
                    continue;
                }

                // Dominance check
                if (generated.find(ch_id)!= generated.end() &&
                    generated[ch_id]<= (cost+ edge_cost)) {
                    continue;
                }

                generated[ch_id]= (cost + edge_cost);
                open.push_back({cost + edge_cost, ch_id});
                std::push_heap(open.begin(), open.end(), order);
            }
        }
        if (up_states.find(node_id) != up_states.end()){
            const std::vector<MultiCostEdge> &outgoing_edges = chg.up_inv_edges(node_id);
            for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
                size_t edge_cost;
                if (cost_idx == 0){
                    edge_cost = p_edge->costs.front().first;
                } else {
                    
                    edge_cost = p_edge->costs.back().second;
                }

                size_t ch_id = p_edge->target;
                if (up_states.find(ch_id) == up_states.end() && down_states.find(ch_id) == down_states.end()){
                    continue;
                }

                // Dominance check
                if (generated.find(ch_id)!= generated.end() &&
                    generated[ch_id]<= (cost+ edge_cost)) {
                    continue;
                }

                generated[ch_id]= (cost + edge_cost);
                open.push_back({cost + edge_cost, ch_id});
                std::push_heap(open.begin(), open.end(), order);
            }
        }
    }
}

std::vector<size_t> CHShortestPathHeuristic::operator()(size_t node_id) {
    auto v = std::vector<size_t>(data.size(), 0);
    for (size_t cost_idx = 0; cost_idx < v.size(); cost_idx++)
    if (data[cost_idx].find(node_id) != data[cost_idx].end()){
        v[cost_idx] = data[cost_idx][node_id];
    }
    return v;
}


void BOAStarCH::operator()(size_t source, size_t target, SolutionSet &solutions, unsigned int time_limit) {
    start_time = std::clock();
    auto up_states = chg.all_up_states(source);
    auto down_states = chg.all_up_states_r(target);
    // if (source != 0 and target != 0){
        // std::cout << up_states.size() << std::endl;
        // std::cout << down_states.size() << std::endl;
    // }

    auto heuristic = CHShortestPathHeuristic(target, chg, up_states, down_states);

    // for (auto &l: heuristic.data){
    //     std::cout << l.size() << std::endl;
    // }

    preprocess_time = std::clock() - start_time;

    NodePtr node;
    NodePtr next;

    // Saving all the unused NodePtrs in a vector improves performace for some reason
    // std::vector<NodePtr> closed;

    // Vector to hold mininum cost of 2nd criteria per node
    std::unordered_map<size_t, size_t> min_g2;

    // Init open heap
    Node::more_than_full_cost more_than;
    std::vector<NodePtr> open;
    std::make_heap(open.begin(), open.end(), more_than);

    node = std::make_shared<Node>(source, std::vector<size_t>(2,0), heuristic(source));
    open.push_back(node);
    std::push_heap(open.begin(), open.end(), more_than);

    std::cout <<"hinit "<< heuristic(source)[0] << ", "<< heuristic(source)[1] << std::endl;

    while (open.empty() == false) {
        if ((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){

            total_time = std::clock() - start_time;
            return;
        }

        // Pop min from queue and process
        std::pop_heap(open.begin(), open.end(), more_than);
        node = open.back();
        open.pop_back();
        num_generation +=1;

        // Dominance check
        if (min_g2.find(target) != min_g2.end()  &&
            node->f[1] >= min_g2[target]){
            continue;
        }
        if (min_g2.find(node->id) != min_g2.end()  &&
            node->g[1] >= min_g2[node->id]){
            continue;
        }

        min_g2[node->id] = node->g[1];
        num_expansion += 1;


        if (node->id == target) {
            solutions.push_back(node);
            continue;
        }

        // Check to which neighbors we should extend the paths
        if (up_states.find(node->id) != up_states.end()){
            const std::vector<MultiCostEdge> &outgoing_edges = chg.up_edges(node->id);
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            size_t next_id = p_edge->target;
            if (up_states.find(next_id) == up_states.end() && down_states.find(next_id) == down_states.end()){
                continue;
            }

            for (const auto & cost:p_edge->costs){
                
                std::vector<size_t> next_g = {node->g[0]+cost.first, node->g[1]+cost.second};
                auto next_h = heuristic(next_id);

                // Dominance check
                if (min_g2.find(target) != min_g2.end()  &&
                    next_g[1] + next_h[1] >= min_g2[target]){
                    continue;
                }
                if (min_g2.find(next_id) != min_g2.end()  &&
                    next_g[1] >= min_g2[next_id]){
                    continue;
                }

                // next = std::make_shared<Node>(next_id, next_g, next_h, node);
                next = std::make_shared<Node>(next_id, next_g, next_h, nullptr);

                open.push_back(next);
                std::push_heap(open.begin(), open.end(), more_than);
            }
        }
        }
        if (down_states.find(node->id) != down_states.end()){
            const std::vector<MultiCostEdge> &outgoing_edges = chg.down_edges(node->id);
            for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
                size_t next_id = p_edge->target;
                if (up_states.find(next_id) == up_states.end() && down_states.find(next_id) == down_states.end()){
                    continue;
                }


                for (const auto & cost:p_edge->costs){
                
                    std::vector<size_t> next_g = {node->g[0]+cost.first, node->g[1]+cost.second};
                    auto next_h = heuristic(next_id);

                    // Dominance check
                    if (min_g2.find(target) != min_g2.end()  &&
                        next_g[1] + next_h[1] >= min_g2[target]){
                        continue;
                    }
                    if (min_g2.find(next_id) != min_g2.end()  &&
                        next_g[1] >= min_g2[next_id]){
                        continue;
                    }

                    // next = std::make_shared<Node>(next_id, next_g, next_h, node);
                    next = std::make_shared<Node>(next_id, next_g, next_h, nullptr);

                    open.push_back(next);
                    std::push_heap(open.begin(), open.end(), more_than);
                }
            }
        }
    }

    total_time = std::clock() - start_time;


}




NodePEPtr BOAStarCHPartial::next_node(NodePEPtr parent, size_t ch_idx, size_t cost_idx){
    if (parent == nullptr){return nullptr;}
    size_t parent_state = parent->id;
    MultiCostEdge* me;
    if (ch_idx < chg.up_graph[parent_state].size()){
        me = & chg.up_graph[parent_state][ch_idx];
    } else {
        me = & chg.down_graph[parent_state][ch_idx - chg.up_graph[parent_state].size()];
    }
    if ((cost_idx >= me->costs.size())){
        return nullptr;
    }
    size_t state = me->target;
    auto new_h = heuristic(state);

    if (min_g2.find(target) != min_g2.end()  &&
        parent->g[1] + me->costs.back().second + new_h[1]>= min_g2[target]){
        return nullptr;
    }

    if (min_g2.find(state) != min_g2.end()  &&
        parent->g[1] + me->costs.back().second >= min_g2[state]){
        return nullptr;
    }

    for (size_t idx = cost_idx; idx < me->costs.size(); idx++){
        std::vector<size_t> next_g = {parent->g[0]+me->costs[idx].first, parent->g[1]+me->costs[idx].second};
        if (min_g2.find(target) != min_g2.end()  &&
            next_g[1] + new_h[1]>= min_g2[target]){
            continue;
        }

        if (min_g2.find(state) != min_g2.end()  &&
            next_g[1] >= min_g2[state]){
            continue;
        }
        NodePEPtr res = std::make_shared<NodePE>(me->target, next_g, new_h, parent, ch_idx, idx);
        return res;
    }
    return nullptr;
}

void BOAStarCHPartial::operator()(size_t source, size_t target, SolutionSet &solutions, unsigned int time_limit) {
    start_time = std::clock();
    this->target = target;
    auto up_states = chg.all_up_states(source);
    auto down_states = chg.all_up_states_r(target);
    // if (source != 0 and target != 0){
        // std::cout << up_states.size() << std::endl;
        // std::cout << down_states.size() << std::endl;
    // }

    heuristic = CHShortestPathHeuristic(target, chg, up_states, down_states);

    // for (auto &l: heuristic.data){
    //     std::cout << l.size() << std::endl;
    // }

    preprocess_time = std::clock() - start_time;

    NodePEPtr node;
    NodePEPtr next;

    // Saving all the unused NodePtrs in a vector improves performace for some reason
    std::vector<NodePEPtr> closed;

    // Vector to hold mininum cost of 2nd criteria per node
    min_g2.clear();

    // Init open heap
    NodePE::more_than_full_cost more_than;
    std::vector<NodePEPtr> open;
    std::make_heap(open.begin(), open.end(), more_than);

    node = std::make_shared<NodePE>(source, std::vector<size_t>(2,0), heuristic(source));
    open.push_back(node);
    std::push_heap(open.begin(), open.end(), more_than);

    std::cout <<"hinit "<< heuristic(source)[0] << ", "<< heuristic(source)[1] << std::endl;

    while (open.empty() == false) {
        if ((std::clock() - start_time)/CLOCKS_PER_SEC > time_limit){

            total_time = std::clock() - start_time;
            return;
        }

        // Pop min from queue and process
        std::pop_heap(open.begin(), open.end(), more_than);
        node = open.back();
        open.pop_back();
        num_generation +=1;


        next = next_node(node->parent, node->state_idx, node->cost_idx + 1);
        if (next != nullptr){
            open.push_back(next);
            std::push_heap(open.begin(), open.end(), more_than);
        }


        // Dominance check
        if (min_g2.find(target) != min_g2.end()  &&
            node->f[1] >= min_g2[target]){
            continue;
        }
        if (min_g2.find(node->id) != min_g2.end()  &&
            node->g[1] >= min_g2[node->id]){
            continue;
        }

        min_g2[node->id] = node->g[1];
        num_expansion += 1;


        if (node->id == target) {
            solutions.push_back( std::make_shared<Node>(node->id, node->g, node->h));
            continue;
        }

        // Check to which neighbors we should extend the paths
        if (up_states.find(node->id) != up_states.end()){
            const std::vector<MultiCostEdge> &outgoing_edges = chg.up_edges(node->id);
            for (size_t i =0; i< outgoing_edges.size(); i++){
                size_t next_id = outgoing_edges[i].target;
                if (up_states.find(next_id) == up_states.end() && down_states.find(next_id) == down_states.end()){
                    continue;
                }
                // next = std::make_shared<NodePE>(next_id, next_g, next_h, node);
                next = next_node(node, i, 0);
                if (next == nullptr){continue;}
                open.push_back(next);
                std::push_heap(open.begin(), open.end(), more_than);
            }

        }
        if (down_states.find(node->id) != down_states.end()){
            const std::vector<MultiCostEdge> &outgoing_edges = chg.down_edges(node->id);
            for (size_t i =0; i< outgoing_edges.size(); i++){
                size_t next_id = outgoing_edges[i].target;
                if (up_states.find(next_id) == up_states.end() && down_states.find(next_id) == down_states.end()){
                    continue;
                }
                // next = std::make_shared<NodePE>(next_id, next_g, next_h, node);
                next = next_node(node, i + chg.up_graph[node->id].size(), 0);
                if (next == nullptr){continue;}
                open.push_back(next);
                std::push_heap(open.begin(), open.end(), more_than);
            }
        }
    }

    total_time = std::clock() - start_time;


}
