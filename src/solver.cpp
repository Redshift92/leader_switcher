#include <boost/config.hpp>
#include <iostream>
#include <fstream>
#include <cmath>

#include <boost/python.hpp>

#include <algorithm>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>

#include <boost/heap/priority_queue.hpp>

using namespace boost;

template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {
  if ( !v.empty() ) {
    out << '[';
    std::copy (v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
    out << "\b\b]";
  }
  return out;
}

template <typename T1, typename T2>
std::ostream& operator<< (std::ostream& out,  const std::pair<T1, T2>& v) {
    out << "(" << v.first << "," << v.second << ")";
  return out;
}

#define get_int (int)python::extract<int>
#define py python
#define INF 10000000


typedef std::pair<int, int> coord_t;
typedef std::pair<int, int> Edge;

// bundled property
struct vertex_coord_t {
    coord_t coord;
    // long index;
};

typedef property<edge_weight_t, int> EdgeWeightProperty;

typedef adjacency_list<listS, vecS, undirectedS, vertex_coord_t, EdgeWeightProperty> graph_t;

typedef graph_traits<graph_t>::vertex_descriptor vertex_t;

typedef std::vector<int> state_t;

// std::vector<vertex_t> index_to_vertex;
// long vertex_counter;
graph_t slv_graph;
state_t start_state, goal_state, leaders;
std::vector<coord_t> formation_conf;

static inline coord_t coord_diff(coord_t c1, coord_t c2) {
    return std::make_pair(c1.first-c2.first, c1.second-c2.second);
}

static inline coord_t coord_sum(coord_t c1, coord_t c2) {
    return std::make_pair(c1.first+c2.first, c1.second+c2.second);
}


void add_vertex_(vertex_t *v, std::vector<py::tuple> *a_n, py::tuple coord,
    std::vector<std::pair<py::tuple, int> >* start_pos, std::vector<std::pair<py::tuple, int> >* goal_pos) {
    *v = add_vertex(slv_graph);
    // vertex_counter += 1;
    slv_graph[*v].coord = std::make_pair(get_int(coord[0]),get_int(coord[1]));
    // slv_graph[*v].index = vertex_counter;
    (*a_n).push_back(coord);
    // index_to_vertex.push_back(*v);

    int j, found = -1;
    for (j = 0; j < start_pos->size(); j++) {
        if ((*start_pos)[j].first == coord) {
            found = j;
            //std::cout << "start: " << get_int(coord[0]) << " " << get_int(coord[1]) << " " << (*start_pos)[j].second << "\n";
            break;
        }
    }
    if (found != -1) {
        // start_state[(*start_pos)[found].second] = vertex_counter;
        start_state[(*start_pos)[found].second] = num_vertices(slv_graph)-1;
        start_pos->erase(start_pos->begin() + found);
    }
    else {
        for (j = 0; j < goal_pos->size(); j++) {
            if ((*goal_pos)[j].first == coord) {
                //std::cout << "goal: " << get_int(coord[0]) << " " << get_int(coord[1]) << " " << (*goal_pos)[j].second << "\n";
                // goal_state[(*goal_pos)[j].second] = vertex_counter;
                goal_state[(*goal_pos)[j].second] = num_vertices(slv_graph)-1;
                goal_pos->erase(goal_pos->begin() + j);
                break;
            }
        }
    }
}


void init_graph(py::list free_space, py::list obs, py::list start_positions, py::list ldrs,
                py::list goal_positions, py::tuple map_limit)  {

    long i,j;

    py::list obstacles = py::extract<py::list>(obs[2]);
    py::list obs_crns_edges  = py::extract<py::list>(obs[3]);
    std::vector<py::tuple> processed_nodes;
    std::vector<py::tuple> added_nodes;
    coord_t nh_dirs[4] = { std::make_pair(0,1), std::make_pair(0,-1),
                        std::make_pair(1,0), std::make_pair(-1,0) };

    // vertex_counter = 0;
    std::vector< std::pair<py::tuple, int> > start_pos, goal_pos;
    for (i = 0; i < py::len(start_positions); i++) {
        py::tuple xx = py::extract<py::tuple>(start_positions[i]);
        start_pos.push_back(std::make_pair(xx, i));
        xx = py::extract<py::tuple>(goal_positions[i]);
        goal_pos.push_back(std::make_pair(xx, i));
        start_state.push_back(-1);
        goal_state.push_back(-1);
    }
    // leader info
    start_state.push_back(-1);
    goal_state.push_back(-1);

    for (i = 0; i < py::len(ldrs); i++) {
        leaders.push_back(get_int(ldrs[i]));
    }

    //std::cout << "free_space: " << py::len(free_space) << "\n";

    for (i = 0; i < py::len(free_space); i++) {
        py::tuple tuple_coord = py::extract<py::tuple>(free_space[i]);
        vertex_t u;
        std::vector<py::tuple>::iterator pos = std::find(added_nodes.begin(), added_nodes.end(), tuple_coord);
        if (pos == added_nodes.end()) {
            add_vertex_(&u, &added_nodes, tuple_coord, &start_pos, &goal_pos);
        }
        else {
            // u = index_to_vertex[std::distance(added_nodes.begin(), pos)];
            // u = vertex(std::distance(added_nodes.begin(), pos), slv_graph);
            u = std::distance(added_nodes.begin(), pos);
        }
        for (j = 0; j < 4; j++) {
            int nh_first = get_int(tuple_coord[0]) + nh_dirs[j].first;
            int nh_second = get_int(tuple_coord[1]) + nh_dirs[j].second;
            if (nh_first < 0 || nh_second < 0 || nh_first >= get_int(map_limit[0]) || nh_second >= get_int(map_limit[1])) {
                continue;
            }
            py::tuple neighbour = py::make_tuple(nh_first, nh_second);
            // processed are nodes added and checked for neighbours
            // if neighbour is already processed, edge has been already added
            if (std::find(processed_nodes.begin(), processed_nodes.end(), neighbour) == processed_nodes.end()) {
                if (!PySequence_Contains(obs_crns_edges.ptr(),  neighbour.ptr())) {
                    vertex_t q;
                    std::vector<py::tuple>::iterator pos = std::find(added_nodes.begin(), added_nodes.end(), neighbour);
                    if (pos == added_nodes.end()) {
                        add_vertex_(&q, &added_nodes, neighbour, &start_pos, &goal_pos);
                    }
                    else {
                        // q = index_to_vertex[std::distance(added_nodes.begin(), pos)];
                        // q = vertex(std::distance(added_nodes.begin(), pos), slv_graph);
                        q = std::distance(added_nodes.begin(), pos);
                    }
                    add_edge(u, q, 1, slv_graph);
                }
            }
        }
        processed_nodes.push_back(tuple_coord);
    }

    for (i = 1; i < start_state.size()-1; i++) {
        formation_conf.push_back(coord_diff(slv_graph[start_state[i]].coord, slv_graph[start_state[i-1]].coord));
    }

    //std::cout << "Number of vertices: " << num_vertices(slv_graph) << "\n";
    //std::cout << "Number of edges: " << num_edges(slv_graph) << "\n";

    // for (j=0; j < start_state.size(); j++) {
        //std::cout << j << ": " << start_state[j] << " " << goal_state[j] << "\n";
    // }

}

typedef std::pair<state_t, long> state_cost_t;

// from least to most expensive
struct state_cost_compare {
    bool operator()(const state_cost_t &sc1, const state_cost_t &sc2) const {
        return sc1.second > sc2.second;
    }
};

typedef heap::priority_queue<state_cost_t, heap::compare<state_cost_compare> > state_queue_t;


std::vector< std::pair<state_t, state_t> > predecessor_successor;

void update_predecessor_successor(std::pair<state_t, state_t> ps) {
    //std::cout << "predecessor " << ps.first << " for " << ps.second << "\n";
    bool updated = false;
    std::vector< std::pair<state_t, state_t> >::iterator ops;
    for (ops = predecessor_successor.begin(); ops != predecessor_successor.end(); ops++) {
        if (ops->second == ps.second) {
            ops->first = ps.first;
            updated = true;
            break;
        }
    }
    if (!updated) {
        predecessor_successor.push_back(ps);
    }
}

state_t get_predecessor(state_t successor) {
    state_t predecessor;
    std::vector< std::pair<state_t, state_t> >::iterator ops;
    for (ops = predecessor_successor.begin(); ops != predecessor_successor.end(); ops++) {
        if (ops->second == successor) {
            predecessor = ops->first;
            break;
        }
    }
    return predecessor;
}

std::vector<int> get_neighbours(int v, int pred) {
    std::vector<int> n;
    graph_traits<graph_t>::out_edge_iterator e, e_end;
    for (boost::tie(e, e_end) = out_edges(v, slv_graph); e != e_end; ++e) {
        vertex_t leader_new_vertex = target(*e, slv_graph);
        if (leader_new_vertex != pred) { // avoid going back to previous leader position
            n.push_back(leader_new_vertex);
        }
    }
    return n;
}

coord_t get_formation_diff(int u, int v) {
    // formation_conf x1-x0, x2-x1, x3-x2...
    coord_t res;
    if (u > v) {
        res = formation_conf[u-1];
        for (int i = 2; i <= u-v; i++) {
            res = coord_sum(res, formation_conf[u-i]);
        }
    }
    else {
        res = std::make_pair(0,0);
        for (int i = 0; i < v-u; i++) {
            res = coord_diff(res, formation_conf[u+i]);
        }
    }
    return res;
}

float get_succ_points(int successor, int leader_v, int successor_i, int leader_i) {
    // //std::cout << "coord diff\n";
    coord_t ndiff = coord_diff(slv_graph[successor].coord, slv_graph[leader_v].coord);
    // //std::cout << "formation diff\n";
    coord_t fdiff = get_formation_diff(successor_i, leader_i);
    // //std::cout << "coord diff\n";
    coord_t error = coord_diff(ndiff, fdiff);
    //std::cout << "error: " << sqrt((error.first*error.first) + (error.second*error.second)) << "\n";
    return (1.0f / (sqrt((error.first*error.first) + (error.second*error.second))+0.001));
}

state_t policy_eval(int leader_new_vertex, state_t state) {
    int leader_i = state.back();
    //std::cout << "policy under leader " << leader_i << "\n";
    state[leader_i] = leader_new_vertex;
    for (int i=0; i < state.size()-1; i++) {
        if (i == leader_i) continue;
        // get all possible successors for a non leader agent
        //std::cout << "possible successors for: " << state[i] << " " << slv_graph[state[i]].coord << "\n";
        std::vector<int> possible_succ = get_neighbours(state[i], -1);
        int best_succ = -1;
        float best_succ_val;
        for (int j=0; j < possible_succ.size(); j++) {
            //std::cout << "getting points for successor " << possible_succ[j] << " " << slv_graph[possible_succ[j]].coord << "\n";
            // more points for successors far from obstacles, keeping formation,
            // far from agents collision (just keeping formation at the moment)
            float succ_points = get_succ_points(possible_succ[j], leader_new_vertex, i, leader_i);
            if (best_succ == -1 || succ_points > best_succ_val) {
                best_succ = j;
                best_succ_val = succ_points;
            }
        }
        state[i] = possible_succ[best_succ];
    }
    //std::cout << "got state " << state << " from policy\n";
    // for (int j=0; j < state.size(); j++) {
        //std::cout << slv_graph[state[j]].coord << "\n";
    // }
    return state;
}

void successors_fixed_leader(state_t state, std::vector<state_t>* succs) {
    int leader_i = state.back();
    int leader_vertex = state[leader_i];
    //std::cout << "get neighbours for: " << slv_graph[leader_vertex].coord << "\n";
    state_t predecessor = get_predecessor(state);
    int pred = ((predecessor.size() == 0) ? -1 : predecessor[leader_i]);
    std::vector<int> leader_successors = get_neighbours(leader_vertex, pred);
    for (int j = 0; j < leader_successors.size(); j++) {
        //std::cout << "getting successor from neighbour: " << leader_successors[j] << ": " << slv_graph[leader_successors[j]].coord << "\n";
        succs->push_back(policy_eval(leader_successors[j], state));
    }

}

std::vector<state_t> successors(int i, state_t state) {
    std::vector<state_t> succ;
    if (i != 0) {
        //std::cout << "single fixed leader\n";
        successors_fixed_leader(state, &succ);
    }
    else {
        //std::cout << "all leaders\n";
        for (int j = 0; j < leaders.size(); j++) {
            //std::cout << "new leader\n";
            state[state.size()-1] = leaders[j];
            successors_fixed_leader(state, &succ);
        }
    }
    //std::cout << "found successors: \n";
    // for (int j=0; j < succ.size(); j++) {
        //std::cout << succ[j] << "\n";
    // }
    return succ;
}


long dijkstra_dist(int u, int v) {
    std::vector<vertex_t> p(num_vertices(slv_graph));
    std::vector<int> d(num_vertices(slv_graph));
    // dijkstra_shortest_paths(slv_graph, index_to_vertex[u],
    //     predecessor_map(make_iterator_property_map(p.begin(), get(vertex_index, slv_graph))).
    //     distance_map(make_iterator_property_map(d.begin(), get(vertex_index, slv_graph))));
    // return d[index_to_vertex[v]];
    dijkstra_shortest_paths(slv_graph, u,
        predecessor_map(make_iterator_property_map(p.begin(), get(vertex_index, slv_graph))).
        distance_map(make_iterator_property_map(d.begin(), get(vertex_index, slv_graph))));
    // std::cout << "dijkstra_dist (" << slv_graph[u].coord << ", " << slv_graph[v].coord << "): " << d[v] << "\n";<
    return d[v];
}

long heuristic(int hidx, state_t state) {
    if (hidx == 0) {
        std::vector<long> costs(state.size()-1);
        for (int i=0; i < state.size()-1; i++) {
            costs[i] = dijkstra_dist(goal_state[i], state[i]);
            // std::cout << "dijkstra_dist (" << slv_graph[goal_state[i]].coord << ", " << slv_graph[state[i]].coord << "): " << costs[i] << "\n";
        }
        // std::cout << "returning: " << *std::max_element(costs.begin(),costs.end()) << "\n";
        return *std::max_element(costs.begin(),costs.end());
    }
    else {
        return dijkstra_dist(goal_state[leaders[hidx-1]], state[leaders[hidx-1]]);
    }
}

float wh = 60;
float wa = 1.5;

long min_key(int i, std::vector<state_queue_t>* open) {
    return  ((state_cost_t) open->at(i).top()).second;
}

state_t get_top(int i, std::vector<state_queue_t>* open) {
    return  ((state_cost_t) open->at(i).top()).first;
}

void update_open(int i, state_cost_t new_state_cost, std::vector<state_queue_t>* open) {
    state_queue_t new_open_i;
    state_cost_t old_el;

    // std::cout << "queue: " << i << "\n";

    // std::cout << "placing " << slv_graph[(new_state_cost.first)[0]].coord << " in queue with cost: " << new_state_cost.second << "\n";

    while (!open->at(i).empty()) {
        old_el = open->at(i).top();
        // std::cout << "already in queue " << slv_graph[(old_el.first)[0]].coord << " with cost: "<< old_el.second <<".\n";
        if (old_el.first != new_state_cost.first) {
            new_open_i.push(old_el);
        }
        open->at(i).pop();
    }
    new_open_i.push(new_state_cost);
    open->at(i) = new_open_i;
}

long get_g_i_cost(state_t state, std::vector<state_cost_t> g_i) {
    long g_i_cost = -1;
    std::vector<state_cost_t>::iterator g_i_it;
    for (g_i_it = g_i.begin(); g_i_it != g_i.end(); g_i_it++) {
        if (g_i_it->first == state) g_i_cost = g_i_it->second;
    }
    return g_i_cost;
}

std::pair<long, int> get_g_i_cost_pos(state_t state, std::vector<state_cost_t> g_i) {
    long g_i_cost = INF;
    int pos = -1;
    for (std::vector<state_cost_t>::iterator g_i_it = g_i.begin() ; g_i_it != g_i.end(); g_i_it++) {
        if (g_i_it->first == state) {
            g_i_cost = g_i_it->second;
            pos = std::distance(g_i.begin(), g_i_it);
        }
    }
    return std::make_pair(g_i_cost, pos);
}

int leader_transfer_cost;

long edge_cost(state_t src, state_t dst) {
    long cost = 0;
    int i;

    for (i = 0; i < src.size()-1; i++) {
        // cost to move
        if (src[i] != dst[i]) {
            cost += 1;
        }
    }

    if (src.back() != dst.back()) {
        cost += (((float)leader_transfer_cost / 100) * src.size());
    }

    // std::vector<coord_t> dst_formation_conf;
    // std::vector<coord_t> dst_formation_err;
    coord_t dst_rel_coord, dst_f_err;
    long f_err = 0;

    for (i = 1; i < dst.size()-1; i++) {
        // dst_formation_conf.push_back(coord_diff(slv_graph[dst[i]].coord, slv_graph[dst[i-1]].coord));
        dst_rel_coord = coord_diff(slv_graph[dst[i]].coord, slv_graph[dst[i-1]].coord);
        // dst_formation_err.push_back(coord_diff(dst_rel_coord, formation_conf[i-1]));
        dst_f_err = coord_diff(dst_rel_coord, formation_conf[i-1]);
        f_err += (dst_f_err.first*dst_f_err.first + dst_f_err.second*dst_f_err.second);
    }
    f_err = (long) sqrt(f_err);

    std::cout << "f_err: " << f_err << "\n";

    if (f_err == 0) {
        std::cout << "error zero for: \n";
        for (i = 0; i < dst.size()-1; i++) {
            std::cout << "state " << i << ": " << slv_graph[dst[i]].coord << " ref: " << slv_graph[start_state[i]].coord << "\n";
            if (i>0) {
                dst_rel_coord = coord_diff(slv_graph[dst[i]].coord, slv_graph[dst[i-1]].coord);
                dst_f_err = coord_diff(dst_rel_coord, formation_conf[i-1]);
                std::cout << "dst rel: " << dst_rel_coord;
                std::cout << " f err: " << dst_f_err << "\n";
            }
        }
    }

    return cost + 2*f_err;
}

float goal_err_th = 10.0;
state_t satisfying_goal;

bool satisfies_goal(state_t state) {
    float error = 0;
    //std::cout << state << " satisfying " << goal_state << "?\n";
    for (int i=0; i < goal_state.size()-1; i++) {
        //std::cout << slv_graph[state[i]].coord << " " << slv_graph[goal_state[i]].coord << "\n";
        coord_t crd_e = coord_diff(slv_graph[state[i]].coord, slv_graph[goal_state[i]].coord);
        error += sqrt(crd_e.first*crd_e.first + crd_e.second*crd_e.second);
    }
    printf("error %f\n", error);
    return (error < goal_err_th);
}

long eval_key(int hidx, state_t state, std::vector<state_cost_t> g_i) {
    long new_cost = get_g_i_cost(state, g_i) + wh*heuristic(hidx, state);
    // long new_cost = wh*heuristic(hidx, state);
    // std::cout << "key val: " << new_cost << "\n";
    // return get_g_i_cost(state, g_i) + wh*heuristic(hidx, state);
    return new_cost;
}

state_cost_t transfer_func(int i, state_t state) {
    state[state.size()-1] = i;
    return std::make_pair(state, leader_transfer_cost);
}

std::vector<std::vector<state_t> > already_expanded;

bool is_already_expanded(int i, state_t state) {
    return (std::find(already_expanded[i].begin(), already_expanded[i].end(), state) != already_expanded[i].end());
}

long global_goal_cost;

void update_succ(int i, state_t state, state_t parent, long cost, std::vector<std::vector<state_cost_t> > *g, std::vector<state_queue_t>* open) {
    //std::cout << "update succ " << i << "\n";
    // should check for any inadmissible, but since i-th inadmissible contains only
    // states with i-th leader it's ok to check only for i-th inadmissible
    if (!is_already_expanded(i, state)) {
        std::pair<long,int> cost_pos = get_g_i_cost_pos(state, g->at(i));
        //std::cout << "not already_expanded: " << state << " current cost: " << cost_pos.first << " new cost: " << cost << "\n";
        if (cost_pos.first > cost) {
            // std::cout << "parent: " << parent << " with child: " << state << " and cost: " << cost << "\n";

            update_predecessor_successor(std::make_pair(parent, state));

            state_cost_t new_state_cost = std::make_pair(state, cost);
            if (cost_pos.second != -1) {
                g->at(i)[cost_pos.second] = new_state_cost;
            }
            else {
                g->at(i).push_back(new_state_cost);
            }
            new_state_cost.second = eval_key(i, state, g->at(i));
            update_open(i, new_state_cost, open);
            if (satisfies_goal(state)) {
                satisfying_goal = state;
                global_goal_cost = cost;
            }
        }
    }
}

void expand(int i, state_t state, std::vector<state_queue_t>* open, std::vector<std::vector<state_cost_t> > *g) {
    // std::cout << "expanding state " << slv_graph[state[0]].coord << " from list " << i << "\n";
    // for (int j=0; j < state.size(); j++) {
        //std::cout << state[j] << " " << slv_graph[state[j]].coord << "\n";
    // }
    open->at(i).pop();

    already_expanded[i].push_back(state);

    long pred_cost = get_g_i_cost(state, g->at(i));

    std::vector<state_t> succ = successors(i, state);
    std::vector<state_t>::iterator succ_state;
    for (succ_state = succ.begin(); succ_state != succ.end(); succ_state++) {
        //std::cout << "cycling successors\n";
        if (get_g_i_cost(*succ_state, g->at(i)) == -1) { // never seend by i-th queue
            g->at(i).push_back(std::make_pair(*succ_state, INF));
        }
        long cost = pred_cost + edge_cost(state, *succ_state);
        //std::cout << "edge cost: " << cost << "\n";

        update_succ(i, *succ_state, state, cost, g, open);
        update_succ(0, *succ_state, state, cost, g, open);
        for (int j=1; j < leaders.size()+1; j++) {
            if (j==i) continue;
            state_cost_t transfered = transfer_func(j-1, *succ_state);
            if (get_g_i_cost(transfered.first, g->at(j)) == -1) { // never seend by i-th queue
                g->at(j).push_back(std::make_pair(transfered.first, INF));
            }
            // long cost_j = get_g_i_cost(*succ_state, g->at(i)) + transfered.second;

            update_succ(j, transfered.first, state, cost + transfered.second, g, open);
            update_succ(0, transfered.first, state, cost + transfered.second, g, open);
        }
    }
}


void run(int ltc, float wa_, float wh_) {

    leader_transfer_cost = ltc;
    wa = wa_;
    wh = wh_;

    std::vector<state_queue_t> open;
    std::vector< std::vector<state_cost_t> > g;

    global_goal_cost = INF;

    // insert start state into all heaps with key(start, i) as priority
    for (int hidx = 0; hidx < leaders.size()+1; hidx++) {
        state_t state_i = start_state;
        if (hidx == 0) {
            // arbitrary leader for anchor first step
            state_i[state_i.size()-1] = leaders[0];
        }
        else {
            state_i[state_i.size()-1] = leaders[hidx-1];
        }
        std::vector<state_cost_t> g_i;
        g_i.push_back(std::make_pair(state_i, 0));
        g.push_back(g_i);
        state_queue_t open_i;
        long key = eval_key(hidx, state_i, g_i);
        open_i.push(std::make_pair(state_i, key));
        //std::cout << "adding to list " << hidx << " with key: " << key << "\n";
        open.push_back(open_i);

        std::vector<state_t> already_expanded_i;
        already_expanded.push_back(already_expanded_i);
    }

    int q = 0;

    while (global_goal_cost > wa * min_key(0, &open)) {
        // std::cout << "min key: " << wa * min_key(0, &open) << " global_goal_cost:" << global_goal_cost << "\n";
        //std::cout << "main cycle\n";
        q = (q % leaders.size()) + 1;
        long key0 = min_key(0, &open);
        long keyq = min_key(q, &open);
        //std::cout << "key " << q << " " << keyq << "\n";
        if (keyq <= wa * key0) {
            // if (!open[q].empty()) {
                expand(q, get_top(q, &open), &open, &g);
            // }
        }
        else {
            expand(0, get_top(0, &open), &open, &g);
        }
        // if (q==2) exit(1);
    }

}

py::tuple coord_to_tuple(coord_t c) {
    return py::make_tuple(c.first, c.second);
}

py::list state_to_coord_list(state_t s) {
    py::list coord_list;
    for (int i=0; i < s.size()-1; i++) {
        coord_list.append(coord_to_tuple(slv_graph[s[i]].coord));
    }
    coord_list.append(s[s.size()-1]);
    return coord_list;
}

bool equal_except_leader(state_t s1, state_t s2) {
    for (int i=0; i<s1.size()-1; i++) {
        if (s1[i] != s2[i]) return false;
    }
    return true;
}

py::list get_states_sequence() {
    for (int i=0; i < predecessor_successor.size(); i++) {
        // std::cout << i << ": " << predecessor_successor[i] << "\n";
    }

    // int cnt = 0;

    py::list seq;
    state_t state = satisfying_goal;
    // std::cout << "start_state: " << start_state << "\n";
    bool has_predecessor = true;
    while (has_predecessor) {
        // std::cout << "appending: " << state << "\n";
        seq.append(state_to_coord_list(state));
        state = get_predecessor(state);
        if (equal_except_leader(state, start_state)) {
            has_predecessor = false;
        }
        // cnt++;
    }
    return seq;
}

py::list get_graph_edges() {
    py::list res;
    graph_traits < graph_t >::edge_iterator ei, ei_end;
    for (tie(ei, ei_end) = edges(slv_graph); ei != ei_end; ++ei) {
        graph_traits < graph_t >::edge_descriptor e = *ei;
        graph_traits < graph_t >::vertex_descriptor u = source(e, slv_graph), v = target(e, slv_graph);
        py::tuple u_coords, v_coords;
        u_coords = py::make_tuple(slv_graph[u].coord.first, slv_graph[u].coord.second);
        v_coords = py::make_tuple(slv_graph[v].coord.first, slv_graph[v].coord.second);
        res.append(py::make_tuple(u_coords, v_coords));
    }
    return res;
}


BOOST_PYTHON_MODULE(solver) {
   py::def("init_graph", init_graph);
   py::def("run", run);
   py::def("get_states_sequence", get_states_sequence);
   py::def("get_graph_edges", get_graph_edges);
}
