#pragma once

#include <algorithm>
#include <cmath>
#include <cwchar>
#include <limits>
#include <queue>
#include <set>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "StateSpace.h"
#include "dynotree/dynotree_macros.h"

namespace dynotree {

template <class Id, int Dimensions, std::size_t BucketSize = 32,
          typename Scalar = double,
          typename StateSpace = Rn<Scalar, Dimensions>>
class KDTree {
private:
  struct Node;
  std::vector<Node> m_nodes;
  std::set<std::size_t> waitingForSplit;
  StateSpace state_space;

public:
  using scalar_t = Scalar;
  using id_t = Id;
  using point_t = Eigen::Matrix<Scalar, Dimensions, 1>;
  using cref_t = const Eigen::Ref<const Eigen::Matrix<Scalar, Dimensions, 1>> &;
  using ref_t = Eigen::Ref<Eigen::Matrix<Scalar, Dimensions, 1>>;
  using state_space_t = StateSpace;
  int m_dimensions = Dimensions;
  static const std::size_t bucketSize = BucketSize;
  using tree_t = KDTree<Id, Dimensions, BucketSize, Scalar, StateSpace>;

  StateSpace &getStateSpace() { return state_space; }

  KDTree() = default;

  void init_tree(int runtime_dimension = -1,
                 const StateSpace &t_state_space = StateSpace()) {
    state_space = t_state_space;
    if constexpr (Dimensions == Eigen::Dynamic) {
      assert(runtime_dimension > 0);
      m_dimensions = runtime_dimension;
      m_nodes.emplace_back(BucketSize, m_dimensions);
    } else {
      m_nodes.emplace_back(BucketSize, -1);
    }
  }

  size_t size() const { return m_nodes[0].m_entries; }

  void addPoint(const point_t &x, const Id &id, bool autosplit = true) {
    std::size_t addNode = 0;

    assert(m_dimensions > 0);
    while (m_nodes[addNode].m_splitDimension != m_dimensions) {
      m_nodes[addNode].expandBounds(x);
      if (x[m_nodes[addNode].m_splitDimension] <
          m_nodes[addNode].m_splitValue) {
        addNode = m_nodes[addNode].m_children.first;
      } else {
        addNode = m_nodes[addNode].m_children.second;
      }
    }
    m_nodes[addNode].add(PointId{x, id});

    if (m_nodes[addNode].shouldSplit() &&
        m_nodes[addNode].m_entries % BucketSize == 0) {
      if (autosplit) {
        split(addNode);
      } else {
        waitingForSplit.insert(addNode);
      }
    }
  }

  void splitOutstanding() {
    std::vector<std::size_t> searchStack(waitingForSplit.begin(),
                                         waitingForSplit.end());
    waitingForSplit.clear();
    while (searchStack.size() > 0) {
      std::size_t addNode = searchStack.back();
      searchStack.pop_back();
      if (m_nodes[addNode].m_splitDimension == m_dimensions &&
          m_nodes[addNode].shouldSplit() && split(addNode)) {
        searchStack.push_back(m_nodes[addNode].m_children.first);
        searchStack.push_back(m_nodes[addNode].m_children.second);
      }
    }
  }

  struct DistanceId {
    Scalar distance;
    Id id;
    inline bool operator<(const DistanceId &dp) const {
      return distance < dp.distance;
    }
  };

  std::vector<DistanceId> searchKnn(const point_t &x,
                                    std::size_t maxPoints) const {
    return searcher().search(x, std::numeric_limits<Scalar>::max(), maxPoints,
                             state_space);
  }

  std::vector<DistanceId> searchBall(const point_t &x, Scalar maxRadius) const {
    return searcher().search(
        x, maxRadius, std::numeric_limits<std::size_t>::max(), state_space);
  }

  std::vector<DistanceId>
  searchCapacityLimitedBall(const point_t &x, Scalar maxRadius,
                            std::size_t maxPoints) const {
    return searcher().search(x, maxRadius, maxPoints, state_space);
  }

  DistanceId search(const point_t &x) const {
    DistanceId result;
    result.distance = std::numeric_limits<Scalar>::infinity();

    if (m_nodes[0].m_entries > 0) {
      std::vector<std::size_t> searchStack;
      searchStack.reserve(
          1 +
          std::size_t(1.5 * std::log2(1 + m_nodes[0].m_entries / BucketSize)));
      searchStack.push_back(0);

      while (searchStack.size() > 0) {
        std::size_t nodeIndex = searchStack.back();
        searchStack.pop_back();
        const Node &node = m_nodes[nodeIndex];
        if (result.distance > node.distance_to_rectangle(x, state_space)) {
          if (node.m_splitDimension == m_dimensions) {
            for (const auto &lp : node.m_locationId) {
              // Allow to have inactive nodes in the tree
              if (!lp.active)
                continue;
              Scalar nodeDist = state_space.distance(x, lp.x);
              if (nodeDist < result.distance) {
                result = DistanceId{nodeDist, lp.id};
              }
            }
          } else {
            node.queueChildren(x, searchStack);
          }
        }
      }
    }
    return result;
  }

  void set_inactive(const point_t &x) {
    DistanceId result;
    result.distance = std::numeric_limits<Scalar>::infinity();

    bool found = false;
    if (m_nodes[0].m_entries > 0) {
      std::vector<std::size_t> searchStack;
      searchStack.reserve(
          1 +
          std::size_t(1.5 * std::log2(1 + m_nodes[0].m_entries / BucketSize)));
      searchStack.push_back(0);

      while (!found && searchStack.size() > 0) {
        std::size_t nodeIndex = searchStack.back();
        searchStack.pop_back();
        Node &node = m_nodes[nodeIndex];
        if (result.distance > node.distance_to_rectangle(x, state_space)) {
          if (node.m_splitDimension == m_dimensions) {
            for (auto &lp : node.m_locationId) {
              // Allow to have inactive nodes in the tree
              if (!lp.active)
                continue;
              Scalar nodeDist = state_space.distance(x, lp.x);
              if (nodeDist < result.distance) {
                result = DistanceId{nodeDist, lp.id};
                if (result.distance < 1e-8) {
                  found = true;
                  lp.active = false;
                  break;
                }
              }
            }
          } else {
            node.queueChildren(x, searchStack);
          }
        }
      }
    }
    CHECK_PRETTY_DYNOTREE__(found);
    // return result;
  }

  class Searcher {
  public:
    Searcher(const tree_t &tree) : m_tree(tree) {}
    Searcher(const Searcher &searcher) : m_tree(searcher.m_tree) {}

    // NB! this method is not const. Do not call this on same instance from
    // different threads simultaneously.
    const std::vector<DistanceId> &search(const point_t &x, Scalar maxRadius,
                                          std::size_t maxPoints,
                                          const StateSpace &state_space) {
      // clear results from last time
      m_results.clear();

      // reserve capacities
      m_searchStack.reserve(
          1 + std::size_t(1.5 * std::log2(1 + m_tree.m_nodes[0].m_entries /
                                                  BucketSize)));
      if (m_prioqueueCapacity < maxPoints &&
          maxPoints < m_tree.m_nodes[0].m_entries) {
        std::vector<DistanceId> container;
        container.reserve(maxPoints);
        m_prioqueue = std::priority_queue<DistanceId, std::vector<DistanceId>>(
            std::less<DistanceId>(), std::move(container));
        m_prioqueueCapacity = maxPoints;
      }

      m_tree.searchCapacityLimitedBall(x, maxRadius, maxPoints, m_searchStack,
                                       m_prioqueue, m_results, state_space);

      m_prioqueueCapacity = std::max(m_prioqueueCapacity, m_results.size());
      return m_results;
    }

  private:
    const tree_t &m_tree;

    std::vector<std::size_t> m_searchStack;
    std::priority_queue<DistanceId, std::vector<DistanceId>> m_prioqueue;
    std::size_t m_prioqueueCapacity = 0;
    std::vector<DistanceId> m_results;
  };

  // NB! returned class has no const methods. Get one instance per thread!
  Searcher searcher() const { return Searcher(*this); }

private:
  struct PointId {
    point_t x;
    Id id;
    bool active = true;
  };
  std::vector<PointId> m_bucketRecycle;

  void searchCapacityLimitedBall(
      const point_t &x, Scalar maxRadius, std::size_t maxPoints,
      std::vector<std::size_t> &searchStack,
      std::priority_queue<DistanceId, std::vector<DistanceId>> &prioqueue,
      std::vector<DistanceId> &results, const StateSpace &state_space) const {
    std::size_t numSearchPoints = std::min(maxPoints, m_nodes[0].m_entries);

    if (numSearchPoints > 0) {
      searchStack.push_back(0);
      while (searchStack.size() > 0) {
        std::size_t nodeIndex = searchStack.back();
        searchStack.pop_back();
        const Node &node = m_nodes[nodeIndex];
        Scalar minDist = node.distance_to_rectangle(x, state_space);
        if (maxRadius > minDist && (prioqueue.size() < numSearchPoints ||
                                    prioqueue.top().distance > minDist)) {
          if (node.m_splitDimension == m_dimensions) {
            node.searchCapacityLimitedBall(x, maxRadius, numSearchPoints,
                                           prioqueue, state_space);
          } else {
            node.queueChildren(x, searchStack);
          }
        }
      }

      results.reserve(prioqueue.size());
      while (prioqueue.size() > 0) {
        results.push_back(prioqueue.top());
        prioqueue.pop();
      }
      std::reverse(results.begin(), results.end());
    }
  }

  bool split(std::size_t index) {
    if (m_nodes.capacity() < m_nodes.size() + 2) {
      m_nodes.reserve((m_nodes.capacity() + 1) * 2);
    }
    Node &splitNode = m_nodes[index];
    splitNode.m_splitDimension = m_dimensions;
    Scalar width(0);
    state_space.choose_split_dimension(splitNode.m_lb, splitNode.m_ub,
                                       splitNode.m_splitDimension, width);

    if (splitNode.m_splitDimension == m_dimensions) {
      return false;
    }

    std::vector<Scalar> splitDimVals;
    splitDimVals.reserve(splitNode.m_entries);
    for (const auto &lp : splitNode.m_locationId) {
      splitDimVals.push_back(lp.x[splitNode.m_splitDimension]);
    }
    std::nth_element(splitDimVals.begin(),
                     splitDimVals.begin() + splitDimVals.size() / 2 + 1,
                     splitDimVals.end());
    std::nth_element(splitDimVals.begin(),
                     splitDimVals.begin() + splitDimVals.size() / 2,
                     splitDimVals.begin() + splitDimVals.size() / 2 + 1);
    splitNode.m_splitValue = (splitDimVals[splitDimVals.size() / 2] +
                              splitDimVals[splitDimVals.size() / 2 + 1]) /
                             Scalar(2);

    splitNode.m_children = std::make_pair(m_nodes.size(), m_nodes.size() + 1);
    std::size_t entries = splitNode.m_entries;
    m_nodes.emplace_back(m_bucketRecycle, entries, m_dimensions);
    Node &leftNode = m_nodes.back();
    m_nodes.emplace_back(entries, m_dimensions);
    Node &rightNode = m_nodes.back();

    for (const auto &lp : splitNode.m_locationId) {
      if (lp.x[splitNode.m_splitDimension] < splitNode.m_splitValue) {
        leftNode.add(lp);
      } else {
        rightNode.add(lp);
      }
    }

    if (leftNode.m_entries ==
        0) // points with equality to splitValue go in rightNode
    {
      splitNode.m_splitValue = 0;
      splitNode.m_splitDimension = m_dimensions;
      splitNode.m_children = std::pair<std::size_t, std::size_t>(0, 0);
      std::swap(rightNode.m_locationId, m_bucketRecycle);
      m_nodes.pop_back();
      m_nodes.pop_back();
      return false;
    } else {
      splitNode.m_locationId.clear();
      // if it was a standard sized bucket, recycle the memory to reduce
      // allocator pressure otherwise clear the memory used by the bucket
      // since it is a branch not a leaf anymore
      if (splitNode.m_locationId.capacity() == BucketSize) {
        std::swap(splitNode.m_locationId, m_bucketRecycle);
      } else {
        std::vector<PointId> empty;
        std::swap(splitNode.m_locationId, empty);
      }
      return true;
    }
  }

  struct Node {
    Node(std::size_t capacity, int runtime_dimension = -1) {
      init(capacity, runtime_dimension);
    }

    Node(std::vector<PointId> &recycle, std::size_t capacity,
         int runtime_dimension) {
      std::swap(m_locationId, recycle);
      init(capacity, runtime_dimension);
    }

    void init(std::size_t capacity, int runtime_dimension) {

      if constexpr (Dimensions == Eigen::Dynamic) {
        assert(runtime_dimension > 0);
        m_lb.resize(runtime_dimension);
        m_ub.resize(runtime_dimension);
        m_splitDimension = runtime_dimension;
      }

      m_lb.setConstant(std::numeric_limits<Scalar>::max());
      m_ub.setConstant(std::numeric_limits<Scalar>::lowest());
      m_locationId.reserve(std::max(BucketSize, capacity));
    }

    void expandBounds(const point_t &x) {
      m_lb = m_lb.cwiseMin(x);
      m_ub = m_ub.cwiseMax(x);
      m_entries++;
    }

    void add(const PointId &lp) {
      expandBounds(lp.x);
      m_locationId.push_back(lp);
    }

    bool shouldSplit() const { return m_entries >= BucketSize; }

    void searchCapacityLimitedBall(const point_t &x, Scalar maxRadius,
                                   std::size_t K,
                                   std::priority_queue<DistanceId> &results,
                                   const StateSpace &state_space) const {

      std::size_t i = 0;

      // this fills up the queue if it isn't full yet
      for (; results.size() < K && i < m_entries; i++) {
        const auto &lp = m_locationId[i];
        Scalar distance = state_space.distance(x, lp.x);
        if (distance < maxRadius) {
          results.emplace(DistanceId{distance, lp.id});
        }
      }

      // this adds new things to the queue once it is full
      for (; i < m_entries; i++) {
        const auto &lp = m_locationId[i];
        Scalar distance = state_space.distance(x, lp.x);
        if (distance < maxRadius && distance < results.top().distance) {
          results.pop();
          results.emplace(DistanceId{distance, lp.id});
        }
      }
    }

    void queueChildren(const point_t &x,
                       std::vector<std::size_t> &searchStack) const {
      if (x[m_splitDimension] < m_splitValue) {
        searchStack.push_back(m_children.second);
        searchStack.push_back(m_children.first); // left is popped first
      } else {
        searchStack.push_back(m_children.first);
        searchStack.push_back(m_children.second); // right is popped first
      }
    }

    Scalar distance_to_rectangle(const point_t &x,
                                 const StateSpace &distance) const {
      return distance.distance_to_rectangle(x, m_lb, m_ub);
    }

    std::size_t m_entries = 0; /// size of the tree, or subtree

    int m_splitDimension = Dimensions; /// split dimension of this node
    Scalar m_splitValue = 0;           /// split value of this node

    // struct Range {
    //   Scalar min, max;
    // };

    // std::array<Range, Dimensions> m_bounds; /// bounding box of this node
    ///
    ///
    ///
    ///
    ///
    ///
    ///
    ///
    ///
    Eigen::Matrix<Scalar, Dimensions, 1> m_lb;
    Eigen::Matrix<Scalar, Dimensions, 1> m_ub;

    std::pair<std::size_t, std::size_t>
        m_children;                    /// subtrees of this node (if not a leaf)
    std::vector<PointId> m_locationId; /// data held in this node (if a leaf)
  };
};

} // namespace dynotree
