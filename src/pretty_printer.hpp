#include <iostream>

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
