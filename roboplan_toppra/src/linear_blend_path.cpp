#include <roboplan_toppra/linear_blend_path.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace roboplan {

namespace {
/// @brief Small tolerance used for degeneracy checks (coincident points, collinearity).
constexpr double kEps = 1e-9;
}  // namespace

LinearBlendPath::LinearBlendPath(const toppra::Vectors& waypoints, double max_deviation)
    : toppra::GeometricPath(static_cast<int>(waypoints.empty() ? 0 : waypoints.front().size())) {
  if (waypoints.size() < 2) {
    throw std::invalid_argument("LinearBlendPath requires at least 2 waypoints.");
  }

  double cumulative = 0.0;

  // Appends a straight segment from `a` to `b`, skipping zero-length segments.
  const auto add_linear = [&](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
    const double len = (b - a).norm();
    if (len <= kEps) {
      return;
    }
    Segment seg;
    seg.start_s = cumulative;
    seg.length = len;
    seg.is_arc = false;
    seg.point = a;
    seg.direction = (b - a) / len;
    segments_.push_back(seg);
    cumulative += len;
  };

  // `free_end` tracks the current open end of the path: the exit of the previous corner
  // blend, or the path start for the first segment.
  Eigen::VectorXd free_end = waypoints.front();

  for (size_t i = 1; i + 1 < waypoints.size(); ++i) {
    const Eigen::VectorXd& c1 = waypoints.at(i - 1);
    const Eigen::VectorXd& c2 = waypoints.at(i);
    const Eigen::VectorXd& c3 = waypoints.at(i + 1);

    // Blend across the corner at c2, using the adjacent segment midpoints as the blend's
    // reach. Midpoints guarantee a blend consumes at most half of each segment, so
    // consecutive blends never overlap.
    const Eigen::VectorXd in_vec = c2 - 0.5 * (c1 + c2);   // toward the corner
    const Eigen::VectorXd out_vec = 0.5 * (c2 + c3) - c2;  // away from the corner
    const double in_len = in_vec.norm();
    const double out_len = out_vec.norm();

    // No blending requested or a degenerate (coincident) neighbor: go straight to c2.
    if (max_deviation <= 0.0 || in_len <= kEps || out_len <= kEps) {
      add_linear(free_end, c2);
      free_end = c2;
      continue;
    }

    const Eigen::VectorXd u_in = in_vec / in_len;
    const Eigen::VectorXd u_out = out_vec / out_len;
    const double angle = std::acos(std::clamp(u_in.dot(u_out), -1.0, 1.0));

    // Nearly collinear (no real corner) or a near-reversal (tan(angle/2) blows up and the
    // blend collapses): treat as a sharp corner the trajectory must slow through.
    if (angle <= kEps || angle >= M_PI - kEps) {
      add_linear(free_end, c2);
      free_end = c2;
      continue;
    }

    const double half = 0.5 * angle;
    double distance = std::min(in_len, out_len);
    distance = std::min(distance, max_deviation * std::sin(half) / (1.0 - std::cos(half)));
    const double radius = distance / std::tan(half);

    Segment arc;
    arc.is_arc = true;
    arc.radius = radius;
    arc.center = c2 + (u_out - u_in).normalized() * (radius / std::cos(half));
    arc.x = (c2 - distance * u_in - arc.center).normalized();
    arc.y = u_in;
    arc.length = angle * radius;
    arc.start_s = 0.0;  // set after the preceding linear segment fixes `cumulative`

    const Eigen::VectorXd arc_entry = c2 - distance * u_in;
    const Eigen::VectorXd arc_exit = c2 + distance * u_out;

    // Straight run up to the blend entry, then the arc itself.
    add_linear(free_end, arc_entry);
    arc.start_s = cumulative;
    segments_.push_back(arc);
    cumulative += arc.length;

    free_end = arc_exit;
  }

  // Final straight run to the last waypoint.
  add_linear(free_end, waypoints.back());
  total_length_ = cumulative;

  // Fully degenerate (all waypoints coincident): keep a single zero-length segment so that
  // evaluation and pathInterval remain well defined.
  if (segments_.empty()) {
    Segment seg;
    seg.point = waypoints.front();
    seg.direction = Eigen::VectorXd::Zero(waypoints.front().size());
    segments_.push_back(seg);
  }
}

const LinearBlendPath::Segment& LinearBlendPath::segmentAt(double s) const {
  // Find the last segment whose start_s <= s (segments_ is ordered by start_s).
  size_t left = 0;
  size_t right = segments_.size();
  while (left < right) {
    const size_t mid = (left + right) / 2;
    if (segments_.at(mid).start_s <= s) {
      left = mid + 1;
    } else {
      right = mid;
    }
  }
  return segments_.at(left == 0 ? 0 : left - 1);
}

toppra::Vector LinearBlendPath::eval_single(toppra::value_type s, int order) const {
  s = std::clamp(s, 0.0, total_length_);
  const Segment& seg = segmentAt(s);
  const double u = s - seg.start_s;
  const int dof = configSize();

  if (!seg.is_arc) {
    switch (order) {
    case 0:
      return seg.point + u * seg.direction;
    case 1:
      return seg.direction;
    default:
      return Eigen::VectorXd::Zero(dof);
    }
  }

  const double theta = seg.radius > 0.0 ? u / seg.radius : 0.0;
  const double cos = std::cos(theta);
  const double sin = std::sin(theta);
  switch (order) {
  case 0:
    return seg.center + seg.radius * (seg.x * cos + seg.y * sin);
  case 1:
    return -seg.x * sin + seg.y * cos;
  case 2:
    return (-1.0 / seg.radius) * (seg.x * cos + seg.y * sin);
  default:
    return Eigen::VectorXd::Zero(dof);
  }
}

toppra::Bound LinearBlendPath::pathInterval() const {
  toppra::Bound bound;
  bound << 0.0, total_length_;
  return bound;
}

toppra::Vector LinearBlendPath::segmentBoundaries() const {
  toppra::Vector boundaries(static_cast<Eigen::Index>(segments_.size()) + 1);
  for (size_t i = 0; i < segments_.size(); ++i) {
    boundaries(static_cast<Eigen::Index>(i)) = segments_.at(i).start_s;
  }
  boundaries(static_cast<Eigen::Index>(segments_.size())) = total_length_;
  return boundaries;
}

}  // namespace roboplan
