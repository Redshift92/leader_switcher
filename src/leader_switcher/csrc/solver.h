#include <boost/config.hpp>
#include <iostream>
#include <fstream>

#include <algorithm>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>

#include <boost/graph/adj_list_serialize.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

using namespace boost;


typedef std::pair<int, int> coord_t;
typedef std::pair<int, int> Edge;

// bundled property
struct vertex_coord_t {
    coord_t coord;
};

typedef property<edge_weight_t, int> EdgeWeightProperty;

typedef adjacency_list<listS, vecS, undirectedS, vertex_coord_t, EdgeWeightProperty> graph_t;

typedef graph_traits < graph_t >::vertex_descriptor vertex_t;


graph_t g;

std::string init_graph(py::list free_space, py::list obstacles,
                py::list start_positions, py::list goal_positions)  {

    // std::vector processed_nodes;
    long i,j;
    // py::list processed_nodes;
    // py::list added_nodes;
    int not_step = 1000;
    int cur_not  = 0;

    std::vector<py::tuple> processed_nodes;
    std::vector<py::tuple> added_nodes;
    std::vector<vertex_t> added_nodes_v;
    coord_t nh_dirs[4] = { std::make_pair(0,1), std::make_pair(0,-1),
                        std::make_pair(1,0), std::make_pair(-1,0) };

    std::cout << "free_space: " << free_space.length() << "\n";

    for (i = 0; i < free_space.length(); i++) {
        py::tuple tuple_coord = PyList_GetItem(free_space, i);
        vertex_t u;
        std::vector<py::tuple>::iterator pos = std::find(added_nodes.begin(), added_nodes.end(), tuple_coord);
        // if (num_vertices(g) > cur_not) {
        //     cur_not += not_step;
        //     std::cout << "Number of vertices: " << num_vertices(g) << "\n";
        //     std::cout << "Number of edges: " << num_edges(g) << "\n";
        //     std::cout << i << "\n";
        //     // break;
        // }
        if (pos == added_nodes.end()) {
            u = add_vertex(g);
            g[u].coord = std::make_pair(int(tuple_coord[0]), int(tuple_coord[1]));
            added_nodes.push_back(tuple_coord);
            added_nodes_v.push_back(u);
            // std::cout << "add node " << g[u].coord.first << " " << g[u].coord.second << "\n";
        }
        else {
            u = added_nodes_v[std::distance(added_nodes.begin(), pos)];
            // std::cout << "already present " << g[u].coord.first << " " << g[u].coord.second << "\n";
        }
        for (j = 0; j < 4; j++) {
            py::tuple neighbour(2);
            neighbour[0] = int(tuple_coord[0]) + nh_dirs[j].first;
            neighbour[1] = int(tuple_coord[1]) + nh_dirs[j].second;
            // std::cout << "check neigh " << int(neighbour[0]) << " " << int(neighbour[1]) << "\n";
            // processed are nodes added and checked for neighbours
            // if neighbour is already processed, edge has been already added
            if (std::find(processed_nodes.begin(), processed_nodes.end(), neighbour) == processed_nodes.end()) {
                // std::cout << "not processed\n";
                if (PySequence_Contains(free_space, neighbour)) {
                    // std::cout << "in free space, add vertex,edge\n";
                    vertex_t q;
                    std::vector<py::tuple>::iterator pos = std::find(added_nodes.begin(), added_nodes.end(), neighbour);
                    if (pos == added_nodes.end()) {
                        q = add_vertex(g);
                        g[q].coord = std::make_pair(int(neighbour[0]), int(neighbour[1]));
                        added_nodes.push_back(neighbour);
                        added_nodes_v.push_back(q);
                        // std::cout << "add node " << g[u].coord.first << " " << g[u].coord.second << "\n";
                    }
                    else {
                        q = added_nodes_v[std::distance(added_nodes.begin(), pos)];
                        // std::cout << "already present " << g[u].coord.first << " " << g[u].coord.second << "\n";
                    }

                    add_edge(u, q, 1, g);
                }
            }
        }
        processed_nodes.push_back(tuple_coord);
    }
    std::cout << "Number of vertices: " << num_vertices(g) << "\n";
    std::cout << "Number of edges: " << num_edges(g) << "\n";

    std::string res_graph;
    boost::archive::text_oarchive oa(res_graph);
    // write class instance to archive
    oa << g;
    return res_graph;

    // for (int k = 0; k < num_vertices(g); k++) {
    //     std::cout << "v" << k << " " <<g[k].coord.first << " " << g[k].coord.second << "\n";
    // }


  // boost::add_edge (0, 1, 8, g);
  // const int num_nodes = 5;
  // enum nodes { A, B, C, D, E };
  // char name[] = "ABCDE";
  // Edge edge_array[] = { Edge(A, C), Edge(B, B), Edge(B, D), Edge(B, E),
  //   Edge(C, B), Edge(C, D), Edge(D, E), Edge(E, A), Edge(E, B)
  // };
  // int weights[] = { 1, 2, 1, 2, 7, 3, 1, 1, 1 };
  // int num_arcs = sizeof(edge_array) / sizeof(Edge);
  // property_map<graph_t, edge_weight_t>::type weightmap = get(edge_weight, g);
  // std::vector<vertex_descriptor> p(num_vertices(g));
  // std::vector<int> d(num_vertices(g));
  // vertex_descriptor s = vertex(A, g);
  //
  // dijkstra_shortest_paths(g, s,
  //                         predecessor_map(boost::make_iterator_property_map(p.begin(), get(boost::vertex_index, g))).
  //                         distance_map(boost::make_iterator_property_map(d.begin(), get(boost::vertex_index, g))));
  //
  // std::cout << "distances and parents:" << std::endl;
  // graph_traits < graph_t >::vertex_iterator vi, vend;
  // for (boost::tie(vi, vend) = vertices(g); vi != vend; ++vi) {
  //   std::cout << "distance(" << name[*vi] << ") = " << d[*vi] << ", ";
  //   std::cout << "parent(" << name[*vi] << ") = " << name[p[*vi]] << std::
  //     endl;
  // }
  // std::cout << std::endl;
  //
  // std::ofstream dot_file("figs/dijkstra-eg.dot");
  //
  // dot_file << "digraph D {\n"
  //   << "  rankdir=LR\n"
  //   << "  size=\"4,3\"\n"
  //   << "  ratio=\"fill\"\n"
  //   << "  edge[style=\"bold\"]\n" << "  node[shape=\"circle\"]\n";
  //
  // graph_traits < graph_t >::edge_iterator ei, ei_end;
  // for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei) {
  //   graph_traits < graph_t >::edge_descriptor e = *ei;
  //   graph_traits < graph_t >::vertex_descriptor
  //     u = source(e, g), v = target(e, g);
  //   dot_file << name[u] << " -> " << name[v]
  //     << "[label=\"" << get(weightmap, e) << "\"";
  //   if (p[v] == u)
  //     dot_file << ", color=\"black\"";
  //   else
  //     dot_file << ", color=\"grey\"";
  //   dot_file << "]";
  // }
  // dot_file << "}";
}

py::list get_graph_edges(std::string graph) {
    boost::archive::text_iarchive ia(ifs);
    ia >> g;
    py::list res;
    graph_traits < graph_t >::edge_iterator ei, ei_end;
    for (tie(ei, ei_end) = edges(g); ei != ei_end; ++ei) {
      graph_traits < graph_t >::edge_descriptor e = *ei;
      graph_traits < graph_t >::vertex_descriptor
        u = source(e, g), v = target(e, g);
      py::tuple edge_coords(2);
      py::tuple u_coords(2), v_coords(2);
      u_coords[0] = g[u].coord.first; u_coords[1] = g[u].coord.second;
      std::cout << "append: " << g[u].coord.first << " " << g[u].coord.first;
      std::cout << " " << g[v].coord.first << " " << g[v].coord.second << "\n---\n";
      v_coords[0] = g[v].coord.first; v_coords[1] = g[v].coord.second;
      edge_coords[0] = u_coords;
      edge_coords[1] = v_coords;
      res.append(edge_coords);
    }
    std::cout << "res len: " << res.length() << "\n";
    return res;
}
