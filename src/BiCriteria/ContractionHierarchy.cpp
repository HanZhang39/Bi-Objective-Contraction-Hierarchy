#include "ContractionHierarchy.h"
#include <unordered_set>
#include <unordered_map>
#include <fstream>
#include <algorithm>
#include "ShortestPathHeuristic.h"


Edge briding(const Edge & a, const Edge &b ){
    Edge e = a.inverse();
    assert(e.target == b. source);
    e.target = b.target;
    for (size_t i = 0; i < e.cost.size(); i++){
        e.cost[i] += b.cost[i];
        e.apex[i] += b.apex[i];
    }
    return e;
}

struct EdgeOrdering{
    
    bool operator()(const Edge&a, const Edge&b) const{
        for (int i = 0; i + 1 < a.apex.size(); i++){
            if (a.apex[i] != b.apex[i]) {
                return (a.apex[i] > b.apex[i]);
            }
        }
        return (a.apex.back() > b.apex.back());
    }
};

void remove_dominated(std::vector<Edge> & edges){
    // WARNING: only for biobjective
    EdgeOrdering eo;
    size_t prev_c1 = 0;
    std::vector<Edge> new_edges;
    std::make_heap(edges.begin(), edges.end(), eo);
    size_t min_g2 = edges.front().apex[1] + 1;
    while(!edges.empty()){
        std::pop_heap(edges.begin(), edges.end(), eo);
        Edge edge = edges.back();
        assert(edge.apex[0] >= prev_c1);
        prev_c1 = edge.apex[0];
        edges.pop_back();
        if ( edge.apex[1] < min_g2 ){
            new_edges.push_back(edge);
            min_g2 = edge.apex[1];
        }
    }
    edges = new_edges;
}

void ContractionHierarchy::edge_difference(size_t state){
    cnt_edge_diff += 1;
    size_t edge_cnt = graph[state].size() + inv_graph[state].size();
    size_t new_edge_cnt = 0;

    std::vector<Edge> shortcuts;
    std::unordered_set<size_t> from_states;
    for (const Edge & edge: inv_graph[state]){
        from_states.insert(edge.target);
    }
    std::unordered_set<size_t> to_states;
    for (const Edge & edge: graph[state]){
        to_states.insert(edge.target);
    }

    for (auto u:from_states){
        std::vector<Edge> from_edges;
        for (const Edge & edge: inv_graph[state]){
            if (edge.target == u){
                from_edges.push_back(edge);
            }
        }

        for (auto v:to_states){
            if (u == v){
                continue;
            }
            std::vector<Edge> to_edges;
            for (const Edge & edge: graph[state]){
                if (edge.target == v){
                    to_edges.push_back(edge);
                }
            }

            cnt_witness_search += 1;
            std::vector<Edge> all_edge_combination;
            all_edge_combination.reserve(from_edges.size() * to_edges.size());
            for (auto& from_e: from_edges){
                for (auto& to_e: to_edges){
                    all_edge_combination.push_back(briding(from_e, to_e));
                }
            }
            if (naive && !lcs_witness){
                // remove_dominated(all_edge_combination);
                std::vector<Edge> all_edge_combination_;
                for (Edge & e : all_edge_combination){
                    if (! witness_search(u, v, e)){
                        all_edge_combination_.push_back(e);
                    }
                }
                all_edge_combination = all_edge_combination_;
            }else if (lcs_witness && naive){
                std::vector<Edge> all_edge_combination_;
                for (Edge & e : all_edge_combination){
                    if (! witness_search_lcs(u, v, e)){
                        all_edge_combination_.push_back(e);
                    }
                }
                all_edge_combination = all_edge_combination_;
            } else if (lcs_witness){
                remove_dominated(all_edge_combination);
                witness_search_lcs(u, v, all_edge_combination);
            } else {
                remove_dominated(all_edge_combination);
                witness_search(u, v, all_edge_combination, state);
            }


            for (Edge & edge: all_edge_combination){
                shortcuts.push_back(edge);
            }
        }
    }

    CHStatePtr state_ptr = states[state];
    state_ptr->shortcuts = shortcuts;
    state_ptr->delet_edge_num = 1 > edge_cnt? 1:edge_cnt;
    state_ptr->add_edge_num = shortcuts.size();

    update_priority(state);
}

void ContractionHierarchy::update_priority(size_t state){
    CHStatePtr a = states[state];
    a->priority = 10.0 * ((double)(a->add_edge_num)) / a->delet_edge_num + a->height;
}

bool is_bounded(const std::vector<size_t>& apex, const std::vector<size_t> & cost,  const EPS eps){
    for (int i = 0; i < apex.size(); i ++ ){
        if ((double) cost[i] > (1 + eps[i]) * apex[i]){
            return false;
        }
    }
    return true;
}

bool update_edges_by_merging(Edge & edge, Edge & other, const std::vector<double> eps){
    assert(edge.source == other.source);
    assert(edge.target == other.target);

    std::vector<size_t> new_apex = edge.apex;
    for (size_t i = 0; i < new_apex.size(); i ++){
        new_apex[i] = std::min(new_apex[i], other.apex[i]);
    }


    auto lex_cost_1= &edge.cost;
    auto lex_cost_2= &other.cost;

    for (int i = lex_cost_1->size() -1; i >=0; i--){
        if (lex_cost_1->at(i) != lex_cost_2->at(i)){
            if (lex_cost_1->at(i) > lex_cost_2->at(i)){
                std::swap(lex_cost_1, lex_cost_2);
            }
            break;
        }
    }
    if (is_bounded(new_apex, *lex_cost_1, eps)){
        edge.apex = new_apex;
        edge.cost = *lex_cost_1;
        return true;
    }

    if (is_bounded(new_apex, *lex_cost_2, eps)){
        edge.apex = new_apex;
        edge.cost = *lex_cost_2;
        return true;
    }
    return false;
}

void ContractionHierarchy::contract(size_t contract_limit){

    auto start_time = std::clock();
    if (contract_limit == 0){
        contract_limit = graph.size() + 10;
    }

    states.reserve(graph.size() + 1);
    std::vector<CHStatePtr> heap;

    for (size_t s=0; s < graph.size() + 1; s ++){
        states.push_back(std::make_shared<CHState>(s));
        heap.push_back(states[s]);
        states[s]->flag = false;
        edge_difference(s);

        // if (s < 200){
        //     std::cout << s << "- " << states[s]->add_edge_num << " - " << states[s]->delet_edge_num << std::endl;
        // }
    }
    // return;

    ContractionOrdering co;
    std::make_heap(heap.begin(), heap.end(), co);

    size_t cnt = 0;

    while(!heap.empty()){
        if (cnt == contract_limit){
            break;
        }

        std::pop_heap(heap.begin(), heap.end(), co);
        CHStatePtr state_ptr = heap.back();
        heap.pop_back();
        if (state_ptr->flag){
            edge_difference(state_ptr->state);
            heap.push_back(state_ptr);
            std::push_heap(heap.begin(), heap.end(), co);
            state_ptr->flag = false;
            continue;
        }
        contraction_order.push_back(state_ptr->state);

        contract_state(state_ptr->state);
        cnt ++;

        if ( cnt % 10000 == 0 && verbal > 0){
            std::cout << "Total contract " << cnt << " states, add shortcuts: " << all_shortcuts.size() << std::endl;
        }
    }

    if (cnt == contract_limit){
        // partial contraction
        for (CHStatePtr state_ptr: heap){
            size_t state = state_ptr->state;
            for (const auto& to_edge: graph[state]){
                all_shortcuts.push_back(to_edge);
            }
        }
    }


    total_time = std::clock() - start_time;
    std::cout << "Total time: " << total_time / CLOCKS_PER_SEC << std::endl;
    std::cout << "cnt edge diff: " << cnt_edge_diff << "  cnt witness search: " << cnt_witness_search << std::endl;
    std::cout << "cnt node exp: " << cnt_node_exp << std::endl;
}

// std::ostream& operator<<(std::ostream &stream, const std::vector<size_t> &vec){
    // if (vec.size() > 0){
    //     stream << vec.front();
    // }
    // for (size_t i = 0 ; i < vec.size(); i ++){
    //     stream << "\t" << vec[i];
    // }
    // return stream;
// }



void ContractionHierarchy::write_to(std::string fname){
 
    std::ofstream output;
    output.open(fname, std::fstream::out);

    output <<  contraction_order.size() << "\t" << graph.size()<< std::endl;
    for (size_t i = 0; i < contraction_order.size(); i ++){
        output << contraction_order[i] << "\t" << states[contraction_order[i]]->height << std::endl;
    }

    output <<  all_shortcuts.size() << std::endl;
    for (size_t i = 0; i < all_shortcuts.size(); i ++){
        output << all_shortcuts[i].source << "\t"
               << all_shortcuts[i].target << "\t";
        for (size_t j = 0 ; j < all_shortcuts[i].cost.size(); j ++){
            output << all_shortcuts[i].cost[j]<< "\t";
        }
        for (size_t j = 0 ; j < all_shortcuts[i].apex.size(); j ++){
            output << all_shortcuts[i].apex[j];
            if (j== all_shortcuts[i].apex.size() - 1){
                output << "\n";
            } else {
                output << "\t";
            }
        }
        // << all_shortcuts[i].cost << "\t"
               // << all_shortcuts[i].apex << std::endl;
    }
    output.close();
}

void ContractionHierarchy::witness_search(size_t source, size_t target, std::vector<Edge> &edges, size_t mid){
    NodePtr node;
    NodePtr next;

    // Saving all the unused NodePtrs in a vector improves performace for some reason

    // Vector to hold mininum cost of 2nd criteria per node
    std::unordered_map<size_t, size_t> min_g2;

    size_t min_f2 = edges[0].apex[1];
    size_t max_f1 = edges[0].apex[0];
    size_t i_check = 0 ;

    PartialShortestPathHeuristic sp_heuristic(target, source, inv_graph);
    cnt_node_exp += sp_heuristic.node_exp;

    using std::placeholders::_1;
    Heuristic heuristic = std::bind( &PartialShortestPathHeuristic::operator(), sp_heuristic, _1);
    // Init open heap

    Node::more_than_full_cost more_than;
    std::vector<NodePtr> open;
    std::make_heap(open.begin(), open.end(), more_than);

    node = std::make_shared<Node>(source, std::vector<size_t>(2,0), heuristic(source));
    open.push_back(node);
    std::push_heap(open.begin(), open.end(), more_than);

    std::vector<Edge> nd_sols;

    while (open.empty() == false) {

        // Pop min from queue and process
        std::pop_heap(open.begin(), open.end(), more_than);
        node = open.back();
        open.pop_back();


        while (node->f[0] > max_f1){
            nd_sols.push_back(edges[i_check]);

            i_check += 1;
            if ( i_check >= edges.size()){
                break;
            }
            max_f1 = edges[i_check].apex[0];
            min_f2 = edges[i_check].apex[1];
        }
        if ( i_check >= edges.size()){
            break;
        }

        // Dominance check
        if (((node->f[1]) > min_f2) ||
            (min_g2.find(node->id) != min_g2.end() && node->g[1] >= min_g2[node->id])) {
            continue;
        }

        // cnt_node_exp += 1;

        if (node->id == target) {
            while (node->f[1] <= min_f2){
                i_check += 1;
                if ( i_check >= edges.size()){
                    break;
                }
                max_f1 = edges[i_check].apex[0];
                min_f2 = edges[i_check].apex[1];
            }
            if ( i_check >= edges.size()){
                break;
            }
            continue;
        }

        min_g2[node->id] = node->g[1];

        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = graph[node->id];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            size_t next_id = p_edge->target;
            if (next_id == mid){
                continue;
            }


            std::vector<size_t> next_g = {node->g[0]+p_edge->apex[0], node->g[1]+p_edge->apex[1]};
            auto next_h = heuristic(next_id);

            // Dominance check
            if ((((next_g[1]+next_h[1])) > min_f2) ||
                (min_g2.find(next_id) != min_g2.end() && next_g[1] >= min_g2[next_id])) {
                continue;
            }

            // If not dominated create node and push to queue
            // Creation is defered after dominance check as it is
            // relatively computational heavy and should be avoided if possible
            next = std::make_shared<Node>(next_id, next_g, next_h, nullptr);

            open.push_back(next);
            std::push_heap(open.begin(), open.end(), more_than);
        }
    }
    while (i_check < edges.size()){
        nd_sols.push_back(edges[i_check]);
        i_check += 1;
    }
    edges = nd_sols;
}


bool ContractionHierarchy::witness_search(size_t source, size_t target, Edge &edge){
    NodePtr node;
    NodePtr next;

    // Saving all the unused NodePtrs in a vector improves performace for some reason

    // Vector to hold mininum cost of 2nd criteria per node
    std::unordered_map<size_t, size_t> min_g2;

    min_g2[target] = edge.apex[1];
    size_t max_f1 = edge.apex[0];
    size_t i_check = 0 ;

    PartialShortestPathHeuristic sp_heuristic(target, source, inv_graph);

    using std::placeholders::_1;
    Heuristic heuristic = std::bind( &PartialShortestPathHeuristic::operator(), sp_heuristic, _1);
    // Init open heap

    Node::more_than_full_cost more_than;
    std::vector<NodePtr> open;
    std::make_heap(open.begin(), open.end(), more_than);

    node = std::make_shared<Node>(source, std::vector<size_t>(2,0), heuristic(source));
    open.push_back(node);
    std::push_heap(open.begin(), open.end(), more_than);

    std::vector<Edge> nd_sols;

    while (open.empty() == false) {

        // Pop min from queue and process
        std::pop_heap(open.begin(), open.end(), more_than);
        node = open.back();
        open.pop_back();


        if (node->f[0] > max_f1){
            return false;
        }

        // Dominance check
        if (((node->f[1]) >= min_g2[target]) ||
            (min_g2.find(node->id) != min_g2.end() && node->g[1] >= min_g2[node->id])) {
            continue;
        }

        min_g2[node->id] = node->g[1];


        if (node->id == target) {
            return true;
            continue;
        }

        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = graph[node->id];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            size_t next_id = p_edge->target;
            std::vector<size_t> next_g = {node->g[0]+p_edge->apex[0], node->g[1]+p_edge->apex[1]};
            auto next_h = heuristic(next_id);

            // Dominance check
            if ((((next_g[1]+next_h[1])) >= min_g2[target]) ||
                (min_g2.find(next_id) != min_g2.end() && next_g[1] >= min_g2[next_id])) {
                continue;
            }

            // If not dominated create node and push to queue
            // Creation is defered after dominance check as it is
            // relatively computational heavy and should be avoided if possible
            next = std::make_shared<Node>(next_id, next_g, next_h, node);

            open.push_back(next);
            std::push_heap(open.begin(), open.end(), more_than);
        }
    }
    return false;
}

struct SimpleNode{
    size_t id;
    size_t g1;
    size_t g2;
    SimpleNode(size_t id, size_t g1, size_t g2):id(id), g1(g1), g2(g2){};

    struct more_than_full_cost_w {
        double w;
        more_than_full_cost_w(double w):w(w){};
        bool operator()(const SimpleNode &a, const SimpleNode &b) const{
            return (w*a.g1 + (1-w)*a.g2  > w*b.g1 + (1-w) * b.g2);
        }
    
    };
};

std::vector<size_t> ContractionHierarchy::min_cost_path_w(size_t source, size_t target, double w){

    // Init open heap
    SimpleNode::more_than_full_cost_w more_than(w);
    std::vector<SimpleNode> open;
    std::make_heap(open.begin(), open.end(), more_than);

    // node = std::make_shared<Node>(source, std::vector<size_t>(2,0), std::vector<size_t>(2,0));
    SimpleNode node(source, 0, 0);
    open.push_back(node);
    std::push_heap(open.begin(), open.end(), more_than);

    std::unordered_map<size_t, double> generated;
    std::unordered_set<size_t> closed;

    while (open.empty() == false) {


        // Pop min from queue and process
        std::pop_heap(open.begin(), open.end(), more_than);
        node = open.back();
        open.pop_back();


        if (closed.find(node.id) != closed.end()){
            continue;
        }

        closed.insert(node.id);
        cnt_node_exp += 1;

        if (node.id == target) {
            return {node.g1, node.g2};
        }

        // Check to which neighbors we should extend the paths
        const std::vector<Edge> &outgoing_edges = graph[node.id];
        for(auto p_edge = outgoing_edges.begin(); p_edge != outgoing_edges.end(); p_edge++) {
            size_t next_id = p_edge->target;
            size_t next_g1 = node.g1 + p_edge->apex[0];
            size_t next_g2 = node.g2 + p_edge->apex[1];


            if (generated.find(next_id)!= generated.end() &&
                generated[next_id]<= w*next_g1 + (1-w)*next_g2 ) {
                continue;
            }

            generated[next_id] = w*next_g1+ (1-w)*next_g2; 
            // next = std::make_shared<Node>(next_id, next_g, std::vector<size_t>(2,0), nullptr);

            SimpleNode next(next_id, next_g1, next_g2);
            open.push_back(next);
            std::push_heap(open.begin(), open.end(), more_than);
        }
    }
    return {std::numeric_limits<size_t>::max(), std::numeric_limits<size_t>::max()};
}



bool ContractionHierarchy::witness_search_lcs(size_t source, size_t target, Edge &edge){
    auto cost_1 = min_cost_path_w(source, target, 0);
    if (cost_1[0] == edge.cost[0] && cost_1[1] == edge.cost[1]){
        return false;
    } else if (cost_1[0] <= edge.cost[0] && cost_1[1] <= edge.cost[1]){
        return true;
    }

    auto cost_2 = min_cost_path_w(source, target, 1);
    if (cost_2[0] == edge.cost[0] && cost_2[1] == edge.cost[1]){
        return false;
    } else if (cost_2[0] <= edge.cost[0] && cost_2[1] <= edge.cost[1]){
        return true;
    }

    double w = ((double )cost_2[1] - cost_1[1]) / (cost_1[0] - cost_2[0] + cost_2[1] -cost_1[1] );
    if (w > 0 && w < 1){
        auto cost_3 = min_cost_path_w(source, target, w);

        if (cost_3[0] == edge.cost[0] && cost_3[1] == edge.cost[1]){
            return false;
        } else if (cost_3[0] <= edge.cost[0] && cost_3[1] <= edge.cost[1]){
            return true;
        }
    }
    return false;

}

void ContractionHierarchy::witness_search_lcs(size_t source, size_t target, std::vector<Edge> & edges){
    auto cost_1 = min_cost_path_w(source, target, 0);
    auto cost_2 = min_cost_path_w(source, target, 1);
    double w = ((double )cost_2[1] - cost_1[1]) / (cost_1[0] - cost_2[0] + cost_2[1] -cost_1[1] );
    std::vector<size_t> cost_3;
    if (w > 0 && w < 1){
        cost_3 = min_cost_path_w(source, target, w);
    }

    std::vector<Edge>  edges_;
    for (auto & edge:edges){
        if (cost_1[0] == edge.cost[0] && cost_1[1] == edge.cost[1]){
            edges_.push_back(edge);
            continue;
        } else if (cost_1[0] <= edge.cost[0] && cost_1[1] <= edge.cost[1]){
            continue;
        }

        if (cost_2[0] == edge.cost[0] && cost_2[1] == edge.cost[1]){
            edges_.push_back(edge);
            continue;
        } else if (cost_2[0] <= edge.cost[0] && cost_2[1] <= edge.cost[1]){
            continue;
        }


        if (w > 0 && w < 1){
            if (cost_3[0] == edge.cost[0] && cost_3[1] == edge.cost[1]){
                edges_.push_back(edge);
                continue;
            } else if (cost_3[0] <= edge.cost[0] && cost_3[1] <= edge.cost[1]){
                continue;
            }
        }
        edges_.push_back(edge);
        continue;

    }

    edges = edges_;
}

bool is_reverse(const Edge & e1, const Edge& e2){
    if (e1.source != e2.target){return false;}
    if (e2.source != e1.target){return false;}

    for (size_t i =0; i < e1.cost.size(); i++){
        if (e1.cost[i] != e2.cost[i]){ return false;}
        if (e1.apex[i] != e2.apex[i]){ return false;}
    }

    return true;
}


bool ContractionHierarchy::can_merge(Edge& new_edge, int & index){
    size_t s = new_edge.source;
    for (size_t i=0; i < graph[s].size(); i++){
        if (new_edge.target != graph[s][i].target){
            continue;
        }
        if (update_edges_by_merging(new_edge, graph.matrix[s][i], epsilon)){
            index = i;
            return true;
        }

    }
    return false;

}

void ContractionHierarchy::contract_state(size_t state){
    CHStatePtr ch_state = states[state];

    for (const Edge& edge : graph[state]){
        auto other = edge.target;
        states[other]->height = std::min((size_t)40, std::max(states[other]->height, ch_state->height + 1));
        states[other]->flag = true;
    }
    for (const Edge& edge : inv_graph[state]){
        auto other = edge.target;
        states[other]->height = std::min((size_t)40, std::max(states[other]->height, ch_state->height + 1));
        states[other]->flag = true;
    }
    delete_state(state);
    assert(ch_state->shortcuts.size() == ch_state->add_edge_num);
    for (auto & edge: ch_state->shortcuts){
        int index_fwd = -1;
        if (can_merge(edge, index_fwd)){
            int index= -1;
            for (size_t i=0; i < inv_graph[edge.target].size(); i++){
                if (is_reverse(graph[edge.source][index_fwd], inv_graph[edge.target][i])){
                    index = i;
                    break;
                }
            }
            assert (index >= 0);
            graph.update_edge(edge.source, index_fwd, edge);
            inv_graph.update_edge(edge.target, index, edge.inverse());


        } else {
            // assert(edge.cost.size() == 2);
            // assert(edge.apex.size() == 2);

            // all_shortcuts.push_back(edge);

            graph.add_edge(edge);
            auto inv_edge = edge.inverse();
            // assert(inv_edge.cost.size() == 2);
            // assert(inv_edge.apex.size() == 2);
            inv_graph.add_edge(inv_edge);
        }
    }
}


bool ContractionOrdering::operator()(const CHStatePtr &a, const CHStatePtr &b) const{
    // return 10.0 * ((double)(a->add_edge_num)) / a->delet_edge_num + a->height >
    //     10.0 * ((double)(b->add_edge_num)) / b->delet_edge_num + b->height ;
    return a->priority > b->priority;
}




void ContractionHierarchy::delete_state(size_t state){
    size_t edge_cnt = graph[state].size()  + inv_graph[state].size();
    size_t delete_cnt = 0;

    std::unordered_set<size_t> from_states;
    for (const auto& edge: inv_graph[state]){
        auto other = edge.target;
        if (from_states.find(other) != from_states.end()){
            continue;
        }
        from_states.insert(other);
        std::vector<Edge> filtered_edges;
        for (const auto& to_edge: graph[other]){
            if (to_edge.target != state){
                filtered_edges.push_back(to_edge);
            }else{
                delete_cnt += 1;
                all_shortcuts.push_back(to_edge);
            }
        }
        graph.update_edges(other, filtered_edges);
    }


    from_states.clear();
    for (const auto& edge: graph[state]){
        auto other = edge.target;
        if (from_states.find(other) != from_states.end()){
            continue;
        }
        from_states.insert(other);
        std::vector<Edge> filtered_edges;
        for (const auto& to_edge: inv_graph[other]){
            if (to_edge.target != state){
                filtered_edges.push_back(to_edge);
            }else{
                delete_cnt += 1;
                all_shortcuts.push_back(to_edge.inverse());
            }
        }
        inv_graph.update_edges(other, filtered_edges);
    }


    assert(edge_cnt == delete_cnt);
}
