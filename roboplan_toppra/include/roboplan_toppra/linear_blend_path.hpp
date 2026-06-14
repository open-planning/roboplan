#pragma once

#include <vector>

#include <Eigen/Dense>
#include <toppra/geometric_path.hpp>
#include <toppra/toppra.hpp>

namespace roboplan {

/// @brief A geometric path made of straight-line segments joined by circular corner blends,
/// parameterized by arc length and exposed through the TOPP-RA `GeometricPath` interface.
/// @details This is the path representation used by time-optimal trajectory generation
/// (Kunz & Stilman 2012, https://www.roboticsproceedings.org/rss08/p27.pdf).
/// Unlike an interpolating cubic spline, straight segments have exactly zero curvature,
/// so that densely sampled, slightly noisy waypoints no longer inflate the spline's centripetal
/// acceleration term (q'' * s_dot^2) and force TOPP-RA to crawl.
/// Curvature exists only at the corner blends and is bounded by the `max_deviation` tolerance
/// (the maximum distance the blend may stray from the sharp corner).
///
/// Each interior waypoint is rounded with a circular arc built from the midpoints of its two
/// adjacent segments; using midpoints guarantees a blend consumes at most half of each
/// segment, so adjacent blends never overlap. The path is C1 (continuous position and unit
/// tangent) with piecewise-constant curvature.
///
/// @note The waypoints are expected to be monotone along the path (no sharp reversals).
/// A near-180-degree reversal cannot be blended and becomes a tangent-discontinuous cusp,
/// which a time parameterization cannot traverse without stopping; this matches the assumptions
/// of TOTG and is satisfied by Cartesian-path-following traces.
class LinearBlendPath : public toppra::GeometricPath {
public:
  /// @brief Constructs the line+blend path through the given waypoints.
  /// @param waypoints The (collapsed) joint-space waypoints; must contain at least 2 points
  ///   all of the same dimension.
  /// @param max_deviation Maximum distance (in the waypoint's units) a corner blend may
  ///   deviate from the sharp corner. Values <= 0 disable blending (pure polyline).
  /// @throws std::invalid_argument if fewer than 2 waypoints are provided.
  LinearBlendPath(const toppra::Vectors& waypoints, double max_deviation);

  /// @brief Evaluates the path at arc-length position `s` (clamped to the path interval).
  /// @param s Arc-length parameter.
  /// @param order Derivative order: 0 = position, 1 = unit tangent, 2 = curvature vector.
  toppra::Vector eval_single(toppra::value_type s, int order = 0) const override;

  /// @brief Returns the path interval [0, total arc length].
  toppra::Bound pathInterval() const override;

  /// @brief Returns the arc-length positions of every segment boundary, including the path
  /// endpoints (0 and the total length).
  /// @details Curvature is discontinuous at line<->arc junctions, so TOPP-RA must place
  /// gridpoints on these boundaries to enforce the acceleration limit correctly. Seed
  /// `proposeGridpoints` with these (as `initialGridpoints`) before handing the grid to the
  /// solver via `setGridpoints`.
  toppra::Vector segmentBoundaries() const;

private:
  /// @brief One path piece: either a straight line or a circular arc, in arc-length terms.
  struct Segment {
    double start_s = 0.0;  ///< Cumulative arc length at the start of this segment.
    double length = 0.0;   ///< Arc length of this segment.
    bool is_arc = false;   ///< True for a circular arc, false for a straight line.

    // Linear segment: position(u) = point + u * direction, with `direction` a unit vector.
    // Circular arc:   position(u) = center + radius * (x * cos(u/radius) + y * sin(u/radius)),
    //                 with `x`, `y` orthonormal; `point`/`direction` unused.
    Eigen::VectorXd point;      ///< Linear: segment start. Arc: unused.
    Eigen::VectorXd direction;  ///< Linear: unit direction. Arc: unused.
    Eigen::VectorXd center;     ///< Arc: circle center. Linear: unused.
    Eigen::VectorXd x;          ///< Arc: unit vector from center to arc start.
    Eigen::VectorXd y;          ///< Arc: unit tangent direction at arc start.
    double radius = 0.0;        ///< Arc: blend radius.
  };

  /// @brief Returns the segment containing arc-length `s` (clamped to the path interval).
  const Segment& segmentAt(double s) const;

  /// @brief The ordered path segments.
  std::vector<Segment> segments_;

  /// @brief Total arc length of the path.
  double total_length_ = 0.0;
};

}  // namespace roboplan
