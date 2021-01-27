#ifndef SPARSE_GRAPH_SPARSE_GRAPH_H_
#define SPARSE_GRAPH_SPARSE_GRAPH_H_

namespace spg {

class SparseGraph {
public:
  explicit SparseGraph(const vi_map::VIMap& map);
  void addVerticesToMissionGraph(const std::string& map_key);
  void compute();

private:
  const vi_map::VIMap& map_;
};

}  // namespace spg

#endif //  SPARSE_GRAPH_SPARSE_GRAPH_H_
