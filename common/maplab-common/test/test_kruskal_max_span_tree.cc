#include <list>
#include <tuple>

#include <maplab-common/kruskal-max-span-tree.h>
#include <maplab-common/test/testing-entrypoint.h>

TEST(MaplabCommon, MaxSpanKruskalTree) {
  typedef std::tuple<int, int, double> Edge;
  std::list<Edge> edges;

  edges.emplace_back(0, 1, 5);
  edges.emplace_back(2, 1, 2);
  edges.emplace_back(3, 0, 7);
  edges.emplace_back(2, 1, 1);
  edges.emplace_back(1, 3, 3);
  edges.emplace_back(3, 2, 4);

  common::KruskalMaxSpanTree(4, &edges);

  std::vector<Edge> cpy_edges;
  cpy_edges.insert(cpy_edges.begin(), edges.begin(), edges.end());

  ASSERT_EQ(cpy_edges.size(), 3u);
  EXPECT_EQ(std::get<0>(cpy_edges[0]), 3);
  EXPECT_EQ(std::get<1>(cpy_edges[0]), 0);
  EXPECT_EQ(std::get<0>(cpy_edges[1]), 0);
  EXPECT_EQ(std::get<1>(cpy_edges[1]), 1);
  EXPECT_EQ(std::get<0>(cpy_edges[2]), 3);
  EXPECT_EQ(std::get<1>(cpy_edges[2]), 2);
}
MAPLAB_UNITTEST_ENTRYPOINT
