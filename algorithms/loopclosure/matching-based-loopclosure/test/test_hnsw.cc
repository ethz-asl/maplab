#include <chrono>
#include <cstdint>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <thread>
#include <vector>
#include <unordered_set>

#include "matching-based-loopclosure/hnswlib/hnswlib.h"

namespace hnswlib {

template <class Function>
inline void ParallelFor(
    size_t start, size_t end, size_t numThreads, Function fn) {
  std::vector<std::thread> threads;
  std::atomic<size_t> current(start);

  // keep track of exceptions in threads
  // https://stackoverflow.com/a/32428427/1713196
  std::exception_ptr lastException = nullptr;
  std::mutex lastExceptMutex;

  for (size_t threadId = 0; threadId < numThreads; ++threadId) {
    threads.push_back(std::thread([&, threadId] {
      while (true) {
        size_t id = current.fetch_add(1);

        if (id >= end) {
          break;
        }

        try {
          fn(id, threadId);
        } catch (...) {
          std::unique_lock<std::mutex> lastExcepLock(lastExceptMutex);
          lastException = std::current_exception();
          /*
           * This will work even when current is the largest value that
           * size_t can fit, because fetch_add returns the previous value
           * before the increment (what will result in overflow
           * and produce 0 instead of current + 1).
           */
          current = end;
          break;
        }
      }
    }));
  }
  for (auto& thread : threads) {
    thread.join();
  }

  if (lastException) {
    std::rethrow_exception(lastException);
  }
}

TEST(HNSWTest, MultiThreadReplace) {
  int d = 16;
  int num_elements = 1000;
  int max_elements = 2 * num_elements;
  int num_threads = 50;

  std::mt19937 rng;
  rng.seed(47);
  std::uniform_real_distribution<> distrib_real;

  L2Space space(d);

  // generate batch1 and batch2 data
  float* batch1 = new float[d * max_elements];
  for (int i = 0; i < d * max_elements; i++) {
    batch1[i] = distrib_real(rng);
  }
  float* batch2 = new float[d * num_elements];
  for (int i = 0; i < d * num_elements; i++) {
    batch2[i] = distrib_real(rng);
  }

  // generate random labels to delete them from index
  std::vector<int> rand_labels(max_elements);
  for (int i = 0; i < max_elements; i++) {
    rand_labels[i] = i;
  }
  std::shuffle(rand_labels.begin(), rand_labels.end(), rng);

  int iter = 0;
  while (iter < 200) {
    HierarchicalNSW<float>* alg_hnsw =
        new HierarchicalNSW<float>(&space, max_elements, 16, 200, 123, true);

    // add batch1 data
    EXPECT_NO_THROW(ParallelFor(
        0, max_elements, num_threads, [&](size_t row, size_t threadId) {
          alg_hnsw->addPoint((void*)(batch1 + d * row), row);
        }));

    // delete half random elements of batch1 data
    for (int i = 0; i < num_elements; i++) {
      alg_hnsw->markDelete(rand_labels[i]);
    }

    // replace deleted elements with batch2 data
    EXPECT_NO_THROW(ParallelFor(
        0, num_elements, num_threads, [&](size_t row, size_t threadId) {
          int label = rand_labels[row] + max_elements;
          alg_hnsw->addPoint((void*)(batch2 + d * row), label, true);
        }));

    iter += 1;

    delete alg_hnsw;
  }

  delete[] batch1;
  delete[] batch2;
}

TEST(HNSWTest, SearchKnnCloserFirst) {
  int d = 4;
  labeltype n = 100;
  labeltype nq = 10;
  size_t k = 10;

  std::vector<float> data(n * d);
  std::vector<float> query(nq * d);

  std::mt19937 rng;
  rng.seed(47);
  std::uniform_real_distribution<> distrib;

  for (labeltype i = 0; i < n * d; ++i) {
    data[i] = distrib(rng);
  }
  for (labeltype i = 0; i < nq * d; ++i) {
    query[i] = distrib(rng);
  }

  L2Space space(d);
  AlgorithmInterface<float>* alg_brute =
      new BruteforceSearch<float>(&space, 2 * n);
  AlgorithmInterface<float>* alg_hnsw =
      new HierarchicalNSW<float>(&space, 2 * n);

  for (size_t i = 0; i < n; ++i) {
    alg_brute->addPoint(data.data() + d * i, i);
    alg_hnsw->addPoint(data.data() + d * i, i);
  }

  // test searchKnnCloserFirst of BruteforceSearch
  for (size_t j = 0; j < nq; ++j) {
    const void* p = query.data() + j * d;
    auto gd = alg_brute->searchKnn(p, k);
    auto res = alg_brute->searchKnnCloserFirst(p, k);
    EXPECT_EQ(gd.size(), res.size());
    size_t t = gd.size();
    while (!gd.empty()) {
      EXPECT_EQ(gd.top(), res[--t]);
      gd.pop();
    }
  }
  for (size_t j = 0; j < nq; ++j) {
    const void* p = query.data() + j * d;
    auto gd = alg_hnsw->searchKnn(p, k);
    auto res = alg_hnsw->searchKnnCloserFirst(p, k);
    EXPECT_EQ(gd.size(), res.size());
    size_t t = gd.size();
    while (!gd.empty()) {
      EXPECT_EQ(gd.top(), res[--t]);
      gd.pop();
    }
  }

  delete alg_brute;
  delete alg_hnsw;
}

bool test_some_filtering(
    BaseFilterFunctor& filter_func, size_t div_num, size_t label_id_start) {
  int d = 4;
  labeltype n = 100;
  labeltype nq = 10;
  size_t k = 10;

  std::vector<float> data(n * d);
  std::vector<float> query(nq * d);

  std::mt19937 rng;
  rng.seed(47);
  std::uniform_real_distribution<> distrib;

  for (labeltype i = 0; i < n * d; ++i) {
    data[i] = distrib(rng);
  }
  for (labeltype i = 0; i < nq * d; ++i) {
    query[i] = distrib(rng);
  }

  L2Space space(d);
  AlgorithmInterface<float>* alg_brute =
      new BruteforceSearch<float>(&space, 2 * n);
  AlgorithmInterface<float>* alg_hnsw =
      new HierarchicalNSW<float>(&space, 2 * n);

  for (size_t i = 0; i < n; ++i) {
    // `label_id_start` is used to ensure that the returned IDs are labels and
    // not internal IDs
    alg_brute->addPoint(data.data() + d * i, label_id_start + i);
    alg_hnsw->addPoint(data.data() + d * i, label_id_start + i);
  }

  // test searchKnnCloserFirst of BruteforceSearch with filtering
  for (size_t j = 0; j < nq; ++j) {
    const void* p = query.data() + j * d;
    auto gd = alg_brute->searchKnn(p, k, &filter_func);
    auto res = alg_brute->searchKnnCloserFirst(p, k, &filter_func);
    if (gd.size() != res.size()) {
      return false;
    }
    size_t t = gd.size();
    while (!gd.empty()) {
      if (gd.top() != res[--t]) {
        return false;
      }
      if ((gd.top().second % div_num) != 0) {
        return false;
      }
      gd.pop();
    }
  }

  // test searchKnnCloserFirst of hnsw with filtering
  for (size_t j = 0; j < nq; ++j) {
    const void* p = query.data() + j * d;
    auto gd = alg_hnsw->searchKnn(p, k, &filter_func);
    auto res = alg_hnsw->searchKnnCloserFirst(p, k, &filter_func);
    if (gd.size() != res.size()) {
      return false;
    }
    size_t t = gd.size();
    while (!gd.empty()) {
      if (gd.top() != res[--t]) {
        return false;
      }
      if ((gd.top().second % div_num) != 0) {
        return false;
      }
      gd.pop();
    }
  }

  delete alg_brute;
  delete alg_hnsw;

  return true;
}

bool test_none_filtering(
    BaseFilterFunctor& filter_func, size_t label_id_start) {
  int d = 4;
  labeltype n = 100;
  labeltype nq = 10;
  size_t k = 10;

  std::vector<float> data(n * d);
  std::vector<float> query(nq * d);

  std::mt19937 rng;
  rng.seed(47);
  std::uniform_real_distribution<> distrib;

  for (labeltype i = 0; i < n * d; ++i) {
    data[i] = distrib(rng);
  }
  for (labeltype i = 0; i < nq * d; ++i) {
    query[i] = distrib(rng);
  }

  L2Space space(d);
  AlgorithmInterface<float>* alg_brute =
      new BruteforceSearch<float>(&space, 2 * n);
  AlgorithmInterface<float>* alg_hnsw =
      new HierarchicalNSW<float>(&space, 2 * n);

  for (size_t i = 0; i < n; ++i) {
    // `label_id_start` is used to ensure that the returned IDs are labels and
    // not internal IDs
    alg_brute->addPoint(data.data() + d * i, label_id_start + i);
    alg_hnsw->addPoint(data.data() + d * i, label_id_start + i);
  }

  // test searchKnnCloserFirst of BruteforceSearch with filtering
  for (size_t j = 0; j < nq; ++j) {
    const void* p = query.data() + j * d;
    auto gd = alg_brute->searchKnn(p, k, &filter_func);
    auto res = alg_brute->searchKnnCloserFirst(p, k, &filter_func);
    if (gd.size() != res.size()) {
      return false;
    }
    if (gd.size() != 0) {
      return false;
    }
  }

  // test searchKnnCloserFirst of hnsw with filtering
  for (size_t j = 0; j < nq; ++j) {
    const void* p = query.data() + j * d;
    auto gd = alg_hnsw->searchKnn(p, k, &filter_func);
    auto res = alg_hnsw->searchKnnCloserFirst(p, k, &filter_func);
    if (gd.size() != res.size()) {
      return false;
    }
    if (gd.size() != 0) {
      return false;
    }
  }

  delete alg_brute;
  delete alg_hnsw;

  return true;
}

class PickDivisibleIds : public BaseFilterFunctor {
  unsigned int divisor = 1;

 public:
  PickDivisibleIds(unsigned int divisor) : divisor(divisor) {
    assert(divisor != 0);
  }
  bool operator()(labeltype label_id) {
    return label_id % divisor == 0;
  }
};

class PickNothing : public BaseFilterFunctor {
 public:
  bool operator()(labeltype label_id) {
    return false;
  }
};

class CustomFilterFunctor : public BaseFilterFunctor {
  std::unordered_set<labeltype> allowed_values;

 public:
  explicit CustomFilterFunctor(const std::unordered_set<labeltype>& values)
      : allowed_values(values) {}

  bool operator()(labeltype id) {
    return allowed_values.count(id) != 0;
  }
};

TEST(HNSWTest, SearchKnnWithFilter) {
  // some of the elements are filtered
  PickDivisibleIds pickIdsDivisibleByThree(3);
  EXPECT_TRUE(test_some_filtering(pickIdsDivisibleByThree, 3, 17));
  PickDivisibleIds pickIdsDivisibleBySeven(7);
  EXPECT_TRUE(test_some_filtering(pickIdsDivisibleBySeven, 7, 17));

  // all of the elements are filtered
  PickNothing pickNothing;
  EXPECT_TRUE(test_none_filtering(pickNothing, 17));

  // functor style which can capture context
  CustomFilterFunctor pickIdsDivisibleByThirteen({26, 39, 52, 65});
  EXPECT_TRUE(test_some_filtering(pickIdsDivisibleByThirteen, 13, 21));
}

}  // namespace hnswlib

MAPLAB_UNITTEST_ENTRYPOINT
