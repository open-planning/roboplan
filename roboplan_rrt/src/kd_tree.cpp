#include <roboplan_rrt/kd_tree.hpp>

#include <cmath>
#include <cstdint>
#include <numbers>

#include <nanoflann.hpp>

namespace roboplan {
namespace {

/// The points, in nanoflann's dataset adaptor interface.
struct Points {
  int dim = 0;
  std::vector<bool> circular;  // Per coordinate, true for an SO(2) angle in [-pi, pi].
  std::vector<double> coords;  // `dim` per point, in insertion order.

  size_t kdtree_get_point_count() const { return dim > 0 ? coords.size() / dim : 0; }
  double kdtree_get_pt(size_t idx, size_t d) const { return coords[idx * dim + d]; }
  template <class Bbox> bool kdtree_get_bbox(Bbox&) const { return false; }
};

/// Squared product metric over R^1 and SO(2) coordinates, in nanoflann's manifold metric
/// interface so that k-d tree pruning stays exact on the wrapped coordinates.
struct Metric {
  using ElementType = double;
  using DistanceType = double;
  static constexpr bool hasQuaternionBlock = false;

  const Points& points;

  explicit Metric(const Points& points_) : points(points_) {}

  static double diff(double a, double b, bool circular) {
    double d = a - b;
    if (circular) {
      if (d > std::numbers::pi) {
        d -= 2.0 * std::numbers::pi;
      } else if (d < -std::numbers::pi) {
        d += 2.0 * std::numbers::pi;
      }
    }
    return d;
  }

  double evalMetric(const double* a, uint32_t b_idx, size_t size) const {
    const double* b = &points.coords[b_idx * size];
    double result = 0.0;
    for (size_t i = 0; i < size; ++i) {
      const double d = diff(a[i], b[i], points.circular[i]);
      result += d * d;
    }
    return result;
  }

  template <typename U, typename V> double accum_dist(U a, V b, size_t i) const {
    const double d = diff(a, b, points.circular[i]);
    return d * d;
  }

  /// Lower bound on the squared distance from `v` to a tree node's interval [low, high].
  double pointToIntervalDist(double v, double low, double high, size_t i) const {
    if (v >= low && v <= high) {
      return 0.0;
    }
    if (!points.circular[i]) {
      const double d = v < low ? low - v : v - high;
      return d * d;
    }
    // Off the arc, the nearest point on it is one of its endpoints.
    const double d = std::min(std::abs(diff(v, low, true)), std::abs(diff(v, high, true)));
    return d * d;
  }
};

}  // namespace
}  // namespace roboplan

template <> struct nanoflann::is_manifold_metric<roboplan::Metric> : std::true_type {};

namespace roboplan {

struct KdTree::Impl {
  using Index = nanoflann::KDTreeSingleIndexIncrementalAdaptor<Metric, Points, -1, uint32_t>;
  Points points;
  std::unique_ptr<Index> index;
};

KdTree::KdTree() : impl_(std::make_unique<Impl>()) {}

KdTree::~KdTree() = default;

void KdTree::reset(const std::vector<bool>& circular, size_t capacity) {
  auto& points = impl_->points;
  points.dim = static_cast<int>(circular.size());
  points.circular = circular;
  points.coords.clear();
  points.coords.reserve(capacity * circular.size());
  impl_->index = std::make_unique<Impl::Index>(points.dim, points);
  impl_->index->reserve(capacity);
}

void KdTree::addPoint(const Eigen::VectorXd& x) {
  auto& points = impl_->points;
  const auto id = static_cast<uint32_t>(points.kdtree_get_point_count());
  points.coords.insert(points.coords.end(), x.data(), x.data() + x.size());
  impl_->index->addPoint(id);
}

int KdTree::nearest(const Eigen::VectorXd& x) const {
  uint32_t id = 0;
  double dist_sq = 0.0;
  return impl_->index->knnSearch(x.data(), 1, &id, &dist_sq) > 0 ? static_cast<int>(id) : -1;
}

std::vector<int> KdTree::withinRadius(const Eigen::VectorXd& x, double radius) const {
  std::vector<nanoflann::ResultItem<uint32_t, double>> matches;
  nanoflann::SearchParameters params;
  params.sorted = false;
  (void)impl_->index->radiusSearch(x.data(), radius * radius, matches, params);
  std::vector<int> ids;
  ids.reserve(matches.size());
  for (const auto& match : matches) {
    ids.push_back(static_cast<int>(match.first));
  }
  return ids;
}

}  // namespace roboplan
