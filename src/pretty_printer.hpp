#include <iostream>
#include "solver.hpp"

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

std::vector<coord_t> state_to_coord_vctor(state_t s) {
    std::vector<coord_t> coord_vctor;
    for (int i=0; i < s.size()-1; i++) {
        coord_vctor.push_back(slv_graph[s[i]].coord);
    }
    return coord_vctor;
}

std::string state_to_str_coord_vctor(state_t s) {
    std::string str = "[";
    std::vector<coord_t> coord_vctor = state_to_coord_vctor(s);
    for (int i=0; i < coord_vctor.size(); i++) {
        std::ostringstream ss;
        ss << coord_vctor[i];
        str += ss.str() + ",";
    }
    std::ostringstream xx;
    xx << s.back();
    str += xx.str() + "]";
    return str;
}
