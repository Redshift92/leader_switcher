#include <boost/config.hpp>
#include <boost/python.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>


#define get_int (int)python::extract<int>
#define py python
#define INF 10000000

using namespace boost;

/************ graph and coords *****************/

typedef std::pair<int, int> coord_t;
typedef std::pair<int, int> Edge;

// bundled property
struct vertex_coord_t {
    coord_t coord;
};

typedef property<edge_weight_t, int> EdgeWeightProperty;

typedef adjacency_list<listS, vecS, undirectedS, vertex_coord_t, EdgeWeightProperty> graph_t;

typedef graph_traits<graph_t>::vertex_descriptor vertex_t;

typedef std::vector<int> state_t;

graph_t slv_graph;


static inline coord_t coord_diff(coord_t c1, coord_t c2) {
    return std::make_pair(c1.first-c2.first, c1.second-c2.second);
}

static inline coord_t coord_sum(coord_t c1, coord_t c2) {
    return std::make_pair(c1.first+c2.first, c1.second+c2.second);
}

static inline py::tuple coord_to_tuple(coord_t c) {
    return py::make_tuple(c.first, c.second);
}

/*********** main algorithm structures and variables **********/

state_t start_state, goal_state, leaders;
std::vector<coord_t> formation_conf;

typedef std::pair<state_t, long> state_cost_t;

// order queue from least to most expensive
struct state_cost_compare {
    bool operator()(const state_cost_t &sc1, const state_cost_t &sc2) const {
        return sc1.second > sc2.second;
    }
};

typedef heap::priority_queue<state_cost_t, heap::compare<state_cost_compare> > state_queue_t;

std::vector< std::pair<state_t, state_t> > predecessor_successor;
float wa, wh;
float leader_transfer_cost;

float goal_err_th = 10.0;
state_t satisfying_goal;

float last_error = INF;

std::vector<std::vector<state_t> > already_expanded;

long global_goal_cost;
