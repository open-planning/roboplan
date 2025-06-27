
#include <algorithm>
#include <cmath>
#include <cwchar>
#include <iostream>
#include <limits>
#include <memory>
#include <variant>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "dynotree_macros.h"

namespace dynotree {

inline Eigen::IOFormat __CleanFmt(4, 0, ", ", "\n", "[", "]");

template <typename T, typename Scalar>
void choose_split_dimension_default(const T &lb, const T &ub, int &ii,
                                    Scalar &width) {
  for (std::size_t i = 0; i < static_cast<size_t>(lb.size()); i++) {
    Scalar dWidth = ub[i] - lb[i];
    if (dWidth > width) {
      ii = i;
      width = dWidth;
    }
  }
}

template <typename T, typename T2, typename Scalar>
void choose_split_dimension_weights(const T &lb, const T &ub, const T2 &weight,
                                    int &ii, Scalar &width) {
  for (std::size_t i = 0; i < static_cast<size_t>(lb.size()); i++) {
    Scalar dWidth = (ub[i] - lb[i]) * weight(i);
    // Scalar dWidth = (ub[i] - lb[i]);
    if (dWidth > width) {
      ii = i;
      width = dWidth;
    }
  }
}

template <typename Scalar, int Dimensions = -1> struct RnL1 {

  using cref_t = const Eigen::Ref<const Eigen::Matrix<Scalar, Dimensions, 1>> &;
  using ref_t = Eigen::Ref<Eigen::Matrix<Scalar, Dimensions, 1>>;

  Eigen::Matrix<Scalar, Dimensions, 1> lb;
  Eigen::Matrix<Scalar, Dimensions, 1> ub;
  Eigen::Matrix<Scalar, Dimensions, 1> weights;
  bool use_weights = false;

  void set_bounds(cref_t lb_, cref_t ub_) {
    lb = lb_;
    ub = ub_;
  }

  void set_weights(cref_t weights_) {
    assert(weights_.size() == weights.size());
    weights = weights_;
    use_weights = true;
  }

  void print(std::ostream &out) {
    out << "State Space: RnL1" << " RuntimeDIM: " << lb.size()
        << " CompileTimeDIM: " << Dimensions << std::endl
        << "lb: " << lb.transpose().format(__CleanFmt) << "\n"
        << "ub: " << ub.transpose().format(__CleanFmt) << std::endl;
  }

  bool check_bounds(cref_t x) const {

    CHECK_PRETTY_DYNOTREE__(lb.size() == x.size());
    CHECK_PRETTY_DYNOTREE__(ub.size() == x.size());

    for (size_t i = 0; i < static_cast<size_t>(x.size()); i++) {
      if (x(i) < lb(i)) {
        return false;
      }
      if (x(i) > ub(i)) {
        return false;
      }
    }
    return true;
  }

  inline void sample_uniform(ref_t x) const {
    x.setRandom();
    x.array() += 1;
    x /= .2;
    x = lb + (ub - lb).cwiseProduct(x);
  }

  void interpolate(cref_t from, cref_t to, Scalar t, ref_t out) const {
    assert(t >= 0);
    assert(t <= 1);
    out = from + t * (to - from);
  }

  void choose_split_dimension(cref_t lb, cref_t ub, int &ii,
                              Scalar &width) const {
    if (use_weights)
      choose_split_dimension_weights(lb, ub, weights, ii, width);
    else
      choose_split_dimension_default(lb, ub, ii, width);
  }

  inline Scalar distance_to_rectangle(cref_t &x, cref_t &lb, cref_t &ub) const {

    Scalar dist = 0;

    if constexpr (Dimensions == Eigen::Dynamic) {

      assert(x.size());
      assert(ub.size());
      assert(lb.size());
      assert(x.size() == ub.size());
      assert(x.size() == lb.size());

      for (size_t i = 0; i < static_cast<size_t>(x.size()); i++) {
        Scalar xx = std::max(lb(i), std::min(ub(i), x(i)));
        Scalar dif = xx - x(i);
        dist += std::abs(dif) * (use_weights ? weights(i) : 1.);
      }
    } else {
      for (size_t i = 0; i < Dimensions; i++) {
        Scalar xx = std::max(lb(i), std::min(ub(i), x(i)));
        Scalar dif = xx - x(i);
        dist += std::abs(dif) * (use_weights ? weights(i) : 1.);
      }
    }
    return dist;
  }

  inline Scalar distance(cref_t x, cref_t y) const {

    assert(x.size());
    assert(y.size());

    if (use_weights)
      return (x - y).cwiseAbs().cwiseProduct(weights).sum();
    else
      return (x - y).cwiseAbs().sum();
  }
};

template <typename Scalar> struct Time {

  using cref_t = const Eigen::Ref<const Eigen::Matrix<Scalar, 1, 1>> &;
  using ref_t = Eigen::Ref<Eigen::Matrix<Scalar, 1, 1>>;
  Eigen::Matrix<Scalar, 1, 1> lb;
  Eigen::Matrix<Scalar, 1, 1> ub;

  bool check_bounds(cref_t x) const {

    CHECK_PRETTY_DYNOTREE__(lb.size() == x.size());
    CHECK_PRETTY_DYNOTREE__(ub.size() == x.size());

    for (size_t i = 0; i < x.size(); i++) {
      if (x(i) < lb(i)) {
        return false;
      }
      if (x(i) > ub(i)) {
        return false;
      }
    }
    return true;
  }

  void print(std::ostream &out) {
    out << "Time: " << lb(0) << " " << ub(0) << std::endl;
  }

  void sample_uniform(ref_t x) const {
    assert(lb(0) >= 0);
    x(0) = double(rand()) / RAND_MAX * (ub(0) - lb(0)) + lb(0);
  }

  void set_bounds(cref_t lb_, cref_t ub_) {
    assert(lb_.size() == 1);
    assert(ub_.size() == 1);
    assert(ub(0) >= lb(0));
    lb = lb_;
    ub = ub_;
  }

  void interpolate(cref_t from, cref_t to, Scalar t, ref_t out) const {

    assert(t >= 0);
    assert(t <= 1);
    assert(to(0) >= from(0));

    Eigen::Matrix<Scalar, 1, 1> d = to - from;
    out = from + t * d;
  }

  void choose_split_dimension(cref_t lb, cref_t ub, int &ii, Scalar &width) {
    choose_split_dimension_default(lb, ub, ii, width);
  }

  // if the ub is small, return inf
  inline Scalar distance_to_rectangle(cref_t x, cref_t lb, cref_t ub) const {

    if (ub(0) < x(0)) {
      return std::numeric_limits<Scalar>::max();
    } else if (x(0) > lb(0)) {
      return 0;
    } else {
      return lb(0) - x(0);
    }
  }

  // from x to y
  // if y is smaller, return inf
  inline Scalar distance(cref_t x, cref_t y) const {

    if (y(0) < x(0)) {
      return std::numeric_limits<Scalar>::max();
    } else {
      return y(0) - x(0);
    }
  }
};

template <typename Scalar> struct SO2 {

  using cref_t = const Eigen::Ref<const Eigen::Matrix<Scalar, 1, 1>> &;
  using ref_t = Eigen::Ref<Eigen::Matrix<Scalar, 1, 1>>;

  double weight = 1.0;
  bool use_weights = false;

  bool check_bounds(cref_t x) const {
    for (size_t i = 0; i < x.size(); i++) {
      if (x(i) < -M_PI) {
        return false;
      }
      if (x(i) > +M_PI) {
        return false;
      }
    }
    return true;
  }

  void print(std::ostream &out) {
    out << "SO2: " << std::endl
        << "weight: " << weight << std::endl
        << "use_weights: " << use_weights << std::endl;
  }

  void sample_uniform(ref_t x) const {

    x(0) = (double(rand()) / (RAND_MAX + 1.)) * 2. * M_PI - M_PI;
  }

  void set_bounds([[maybe_unused]] cref_t lb_, [[maybe_unused]] cref_t ub_) {

    THROW_PRETTY_DYNOTREE("so2 has no bounds");
  }

  void set_weights(cref_t weights_) {
    assert(weights_.size() == 1);
    weight = weights_(0);
    use_weights = true;
  }

  void interpolate(cref_t from, cref_t to, Scalar t, ref_t out) const {

    assert(t >= 0);
    assert(t <= 1);

    Eigen::Matrix<Scalar, 1, 1> d = to - from;

    if (d(0) > M_PI) {
      d(0) -= 2 * M_PI;
    } else if (d(0) < -M_PI) {
      d(0) += 2 * M_PI;
    }

    out = from + t * d;

    if (out(0) > M_PI) {
      out(0) -= 2 * M_PI;
    } else if (out(0) < -M_PI) {
      out(0) += 2 * M_PI;
    }
  }

  void choose_split_dimension(cref_t lb, cref_t ub, int &ii, Scalar &width) {
    choose_split_dimension_default(lb, ub, ii, width);
  }

  inline Scalar distance_to_rectangle(cref_t x, cref_t lb, cref_t ub) const {

    assert(x(0) >= -M_PI);
    assert(x(0) <= M_PI);

    assert(lb(0) >= -M_PI);
    assert(lb(0) <= M_PI);

    assert(ub(0) >= -M_PI);
    assert(ub(0) <= M_PI);

    if (x(0) >= lb(0) && x(0) <= ub(0)) {
      return 0;
    } else if (x(0) > ub(0)) {
      Scalar d1 = x(0) - ub(0);
      Scalar d2 = lb(0) - (x(0) - 2 * M_PI);
      assert(d2 >= 0);
      assert(d1 >= 0);
      return std::min(d1, d2) * (use_weights ? weight : 1.);
    } else if (x(0) < lb(0)) {
      Scalar d1 = lb(0) - x(0);
      Scalar d2 = (x(0) + 2 * M_PI) - ub(0);
      assert(d2 >= 0);
      assert(d1 >= 0);
      return std::min(d1, d2) * (use_weights ? weight : 1.);
    } else {
      assert(false);
      return 0;
    }
  }

  inline Scalar distance(cref_t x, cref_t y) const {

    assert(x(0) >= -M_PI);
    assert(y(0) >= -M_PI);

    assert(x(0) <= M_PI);
    assert(y(0) <= M_PI);

    Scalar dif = x(0) - y(0);
    if (dif > M_PI) {
      dif -= 2 * M_PI;
    } else if (dif < -M_PI) {
      dif += 2 * M_PI;
    }
    Scalar out = std::abs(dif);
    return out * (use_weights ? weight : 1.);
  }
};

template <typename Scalar> struct SO2Squared {

  using cref_t = const Eigen::Ref<const Eigen::Matrix<Scalar, 1, 1>> &;
  using ref_t = Eigen::Ref<Eigen::Matrix<Scalar, 1, 1>>;

  Eigen::Matrix<Scalar, 1, 1> lb;
  Eigen::Matrix<Scalar, 1, 1> ub;

  SO2<Scalar> so2;

  void print(std::ostream &out) { out << "SO2Squared: " << std::endl; }

  bool check_bounds(cref_t x) const {
    for (size_t i = 0; i < x.size(); i++) {
      if (x(i) < -M_PI) {
        return false;
      }
      if (x(i) > +M_PI) {
        return false;
      }
    }
    return true;
  }

  inline void interpolate(cref_t from, cref_t to, Scalar t, ref_t out) const {
    so2.interpolate(from, to, t, out);
  }

  void set_weights(cref_t weights_) {
    THROW_PRETTY_DYNOTREE("so2 weights not implemented");
  }

  inline void sample_uniform(ref_t x) const { so2.sample_uniform(x); }

  inline void set_bounds([[maybe_unused]] cref_t lb_,
                         [[maybe_unused]] cref_t ub_) {

    THROW_PRETTY_DYNOTREE("so2 has no bounds");
  }

  inline void choose_split_dimension(cref_t lb, cref_t ub, int &ii,
                                     Scalar &width) {
    choose_split_dimension_default(lb, ub, ii, width);
  }

  inline Scalar distance_to_rectangle(cref_t x, cref_t lb, cref_t ub) const {

    Scalar d = so2.distance_to_rectangle(x, lb, ub);
    return d * d;
  }

  inline Scalar distance(cref_t x, cref_t y) const {

    Scalar d = so2.distance(x, y);
    return d * d;
  }
};

template <typename Scalar, int Dimensions = -1> struct RnSquared {

  using cref_t = const Eigen::Ref<const Eigen::Matrix<Scalar, Dimensions, 1>> &;
  using ref_t = Eigen::Ref<Eigen::Matrix<Scalar, Dimensions, 1>>;
  using vec_t = Eigen::Matrix<Scalar, Dimensions, 1>;

  Eigen::Matrix<Scalar, Dimensions, 1> lb;
  Eigen::Matrix<Scalar, Dimensions, 1> ub;
  vec_t weights;
  bool use_weights = false;

  void print(std::ostream &out) {
    out << "State Space: RnSquared" << " RuntimeDIM: " << lb.size()
        << " CompileTimeDIM: " << Dimensions << std::endl
        << "lb: " << lb.transpose().format(__CleanFmt) << "\n"
        << "ub: " << ub.transpose().format(__CleanFmt) << std::endl;
  }

  bool check_bounds(cref_t x) const {

    CHECK_PRETTY_DYNOTREE__(lb.size() == x.size());
    CHECK_PRETTY_DYNOTREE__(ub.size() == x.size());

    for (size_t i = 0; i < x.size(); i++) {
      if (x(i) < -M_PI) {
        return false;
      }
      if (x(i) > +M_PI) {
        return false;
      }
    }
    return true;
  }

  void set_weights(cref_t weights_) {
    // assert(weights_.size() == weights.size());
    weights = weights_;
    use_weights = true;
  }

  void interpolate(cref_t from, cref_t to, Scalar t, ref_t out) const {
    assert(t >= 0);
    assert(t <= 1);
    out = from + t * (to - from);
  }

  void set_bounds(cref_t lb_, cref_t ub_) {
    lb = lb_;
    ub = ub_;
  }

  void choose_split_dimension(cref_t lb, cref_t ub, int &ii,
                              Scalar &width) const {
    if (use_weights)
      choose_split_dimension_weights(lb, ub, weights, ii, width);
    else
      choose_split_dimension_default(lb, ub, ii, width);
  }

  void sample_uniform(ref_t x) const {
    x.setRandom();   // [-1,1]
    x.array() += 1.; // [0,2]
    x /= .2;         // [0,1]
    x = lb + (ub - lb).cwiseProduct(x);
  }

  inline Scalar distance_to_rectangle(cref_t x, cref_t lb, cref_t ub) const {

    Scalar dist = 0;

    if constexpr (Dimensions == Eigen::Dynamic) {

      assert(x.size());
      assert(ub.size());
      assert(lb.size());
      assert(x.size() == ub.size());
      assert(x.size() == lb.size());

      for (size_t i = 0; i < static_cast<size_t>(x.size()); i++) {
        Scalar xx = std::max(lb(i), std::min(ub(i), x(i)));
        Scalar dif = xx - x(i);
        dist += dif * dif * (use_weights ? weights(i) * weights(i) : 1.);
      }
    } else {
      for (size_t i = 0; i < Dimensions; i++) {
        Scalar xx = std::max(lb(i), std::min(ub(i), x(i)));
        Scalar dif = xx - x(i);
        dist += dif * dif * (use_weights ? weights(i) * weights(i) : 1.);
      }
    }
    return dist;
  }

  inline Scalar distance(cref_t &x, cref_t &y) const {

    assert(x.size());
    assert(y.size());

    if (use_weights)
      return (x - y).cwiseProduct(weights).squaredNorm();
    else
      return (x - y).squaredNorm();
  }
};

template <typename Scalar, int Dimensions = -1> struct Rn {
  using cref_t = const Eigen::Ref<const Eigen::Matrix<Scalar, Dimensions, 1>> &;
  using ref_t = Eigen::Ref<Eigen::Matrix<Scalar, Dimensions, 1>>;
  using vec_t = Eigen::Matrix<Scalar, Dimensions, 1>;
  using DIM = std::integral_constant<int, Dimensions>;

  RnSquared<Scalar, Dimensions> rn_squared;
  Eigen::Matrix<Scalar, Dimensions, 1> lb;
  Eigen::Matrix<Scalar, Dimensions, 1> ub;
  vec_t weights;
  bool use_weights = false;

  void print(std::ostream &out) {
    out << "State Space: Rn" << " RuntimeDIM: " << lb.size()
        << " CompileTimeDIM: " << Dimensions << std::endl
        << "lb: " << lb.transpose().format(__CleanFmt) << "\n"
        << "ub: " << ub.transpose().format(__CleanFmt) << std::endl;
  }

  void set_weights(cref_t weights_) {
    weights = weights_;
    use_weights = true;
    rn_squared.set_weights(weights);
  }

  void set_bounds(cref_t lb_, cref_t ub_) {
    lb = lb_;
    ub = ub_;
  }

  bool check_bounds(cref_t x) const {

    CHECK_PRETTY_DYNOTREE__(lb.size() == x.size());
    CHECK_PRETTY_DYNOTREE__(ub.size() == x.size());

    for (size_t i = 0; i < static_cast<size_t>(x.size()); i++) {
      if (x(i) < lb(i)) {
        return false;
      }
      if (x(i) > ub(i)) {
        return false;
      }
    }
    return true;
  }

  inline void interpolate(cref_t from, cref_t to, Scalar t, ref_t out) const {
    assert(t >= 0);
    assert(t <= 1);
    out = from + t * (to - from);
  }

  inline void sample_uniform(ref_t x) const {
    x.setRandom();
    x.array() += 1.;
    x /= 2.;
    x = lb + (ub - lb).cwiseProduct(x);
  }

  inline void choose_split_dimension(cref_t lb, cref_t ub, int &ii,
                                     Scalar &width) const {
    if (use_weights)
      choose_split_dimension_weights(lb, ub, weights, ii, width);
    else
      choose_split_dimension_default(lb, ub, ii, width);
  }

  inline Scalar distance_to_rectangle(cref_t &x, cref_t &lb, cref_t &ub) const {

    Scalar d = rn_squared.distance_to_rectangle(x, lb, ub);
    return std::sqrt(d);
  };

  inline Scalar distance(cref_t &x, cref_t &y) const {
    Scalar d = rn_squared.distance(x, y);
    return std::sqrt(d);
  }
};

struct Vpure {

  using Scalar = double;

  using cref_t = const Eigen::Ref<const Eigen::Matrix<double, -1, 1>> &;
  using ref_t = Eigen::Ref<Eigen::Matrix<double, -1, 1>>;

  Rn<double, 4> rn;

  virtual void set_bounds(cref_t lb_, cref_t ub_) = 0;

  virtual inline void interpolate(cref_t from, cref_t to, Scalar t,
                                  ref_t out) const = 0;

  virtual inline void sample_uniform(ref_t x) const = 0;

  virtual inline void choose_split_dimension(cref_t lb, cref_t ub, int &ii,
                                             Scalar &width) = 0;

  virtual inline Scalar distance_to_rectangle(cref_t &x, cref_t &lb,
                                              cref_t &ub) const = 0;

  virtual inline Scalar distance(cref_t &x, cref_t &y) const = 0;
};

struct S4irtual : Vpure {

  using Scalar = double;

  using cref_t = const Eigen::Ref<const Eigen::Matrix<double, -1, 1>> &;
  using ref_t = Eigen::Ref<Eigen::Matrix<double, -1, 1>>;

  Rn<double, 4> rn;

  void set_bounds(cref_t lb_, cref_t ub_) override { rn.set_bounds(lb_, ub_); }

  virtual inline void interpolate(cref_t from, cref_t to, Scalar t,
                                  ref_t out) const override {
    rn.interpolate(from, to, t, out);
  }

  virtual inline void sample_uniform(ref_t x) const override {
    rn.sample_uniform(x);
  }

  virtual inline void choose_split_dimension(cref_t lb, cref_t ub, int &ii,
                                             Scalar &width) override {
    rn.choose_split_dimension(lb, ub, ii, width);
  }

  virtual inline Scalar distance_to_rectangle(cref_t &x, cref_t &lb,
                                              cref_t &ub) const override {
    return rn.distance_to_rectangle(x, lb, ub);
  };

  virtual inline Scalar distance(cref_t &x, cref_t &y) const override {
    return rn.distance(x, y);
  }
};

struct virtual_wrapper {

  using Scalar = double;

  using cref_t = const Eigen::Ref<const Eigen::Matrix<double, -1, 1>> &;
  using ref_t = Eigen::Ref<Eigen::Matrix<double, -1, 1>>;

  std::shared_ptr<Vpure> s4;

  void set_bounds(cref_t lb_, cref_t ub_) { s4->set_bounds(lb_, ub_); }

  inline void interpolate(cref_t from, cref_t to, Scalar t, ref_t out) const {
    s4->interpolate(from, to, t, out);
  }

  inline void sample_uniform(ref_t x) const { s4->sample_uniform(x); }

  inline void choose_split_dimension(cref_t lb, cref_t ub, int &ii,
                                     Scalar &width) {
    s4->choose_split_dimension(lb, ub, ii, width);
  }

  inline Scalar distance_to_rectangle(cref_t &x, cref_t &lb, cref_t &ub) const {
    return s4->distance_to_rectangle(x, lb, ub);
  };

  inline Scalar distance(cref_t &x, cref_t &y) const {
    return s4->distance(x, y);
  }
};

template <int id> struct AddOneOrKeepMinusOne {
  // using value = id + 1;
  static constexpr int value = id + 1;
};

template <> struct AddOneOrKeepMinusOne<-1> {
  static constexpr int value = -1;
};

// template <int id> struct print_num;

// Dimensions, without including time. E.g. R^2 x Time is DIM=2
template <typename Scalar, int Dimensions> struct RnTime {

  // using effective_dim = typename AddOneOrKeepMinusOne<Dimensions>::value;

  constexpr static int effective_dim = AddOneOrKeepMinusOne<Dimensions>::value;
  using cref_t =
      const Eigen::Ref<const Eigen::Matrix<Scalar, effective_dim, 1>> &;
  using ref_t = Eigen::Ref<Eigen::Matrix<Scalar, effective_dim, 1>>;

  Time<Scalar> time;
  Rn<Scalar, Dimensions> rn;

  double lambda_t = 1.;
  double lambda_r = 1.;

  void set_lambda(double lambda_t_, double lambda_r_) {
    assert(lambda_t_ >= 0);
    assert(lambda_r_ >= 0);
    lambda_t = lambda_t_;
    lambda_r = lambda_r_;
  }

  void print(std::ostream &out) {
    out << "RnTime: " << std::endl;
    time.print(out);
    rn.print(out);
  }

  void interpolate(cref_t from, cref_t to, Scalar t, ref_t out) const {
    assert(t >= 0);
    assert(t <= 1);

    time.interpolate(from.template tail<1>(), to.template tail<1>(), t,
                     out.template tail<1>());

    if constexpr (Dimensions == Eigen::Dynamic) {
      size_t n = from.size() - 1;
      rn.interpolate(from.head(n), to.head(n), t, out.head(n));

    }

    else {
      rn.interpolate(from.template head<Dimensions>(),
                     to.template head<Dimensions>(), t,
                     out.template head<Dimensions>());
    }
  }

  void set_bounds(cref_t lb_, cref_t ub_) {

    assert(lb_.size() == ub_.size());

    time.set_bounds(lb_.template tail<1>(), ub_.template tail<1>());

    if constexpr (Dimensions == Eigen::Dynamic) {
      size_t n = lb_.size() - 1;
      rn.set_bounds(lb_.head(n), ub_.head(n));
    } else {
      rn.set_bounds(lb_.template head<Dimensions>(),
                    ub_.template head<Dimensions>());
    }
  }

  void choose_split_dimension(cref_t lb, cref_t ub, int &ii,
                              Scalar &width) const {
    choose_split_dimension_default(lb, ub, ii, width);
  }

  void sample_uniform(ref_t x) const {

    time.sample_uniform(x.template tail<1>());

    if constexpr (Dimensions == Eigen::Dynamic) {
      size_t n = x.size() - 1;
      rn.sample_uniform(x.head(n));
    } else {
      rn.sample_uniform(x.template head<Dimensions>());
    }
  }

  inline Scalar distance_to_rectangle(cref_t &x, cref_t &lb, cref_t &ub) const {

    double dt = time.distance_to_rectangle(
        x.template tail<1>(), lb.template tail<1>(), ub.template tail<1>());
    double dr;

    if (dt == std::numeric_limits<Scalar>::max()) {
      return std::numeric_limits<Scalar>::max();
    }

    if constexpr (Dimensions == Eigen::Dynamic) {
      size_t n = x.size() - 1;
      dr = rn.distance_to_rectangle(x.head(n), lb.head(n), ub.head(n));
    } else {
      dr = rn.distance_to_rectangle(x.template head<Dimensions>(),
                                    lb.template head<Dimensions>(),
                                    ub.template head<Dimensions>());
    }
    return lambda_r * dr + lambda_t * dt;
  }

  inline Scalar distance(cref_t x, cref_t y) const {

    double dt = time.distance(x.template tail<1>(), y.template tail<1>());
    double dr;

    if (dt == std::numeric_limits<Scalar>::max()) {
      return std::numeric_limits<Scalar>::max();
    }

    if constexpr (Dimensions == Eigen::Dynamic) {
      size_t n = x.size() - 1;
      dr = rn.distance(x.head(n), y.head(n));
    } else {
      dr = rn.distance(x.template head<Dimensions>(),
                       y.template head<Dimensions>());
    }
    return lambda_r * dr + lambda_t * dt;
  }
};

template <typename Scalar> struct R2SO2 {

  using cref_t = const Eigen::Ref<const Eigen::Matrix<Scalar, 3, 1>> &;
  using ref_t = Eigen::Ref<Eigen::Matrix<Scalar, 3, 1>>;
  using vec_t = Eigen::Matrix<Scalar, 3, 1>;

  using cref2_t = const Eigen::Ref<const Eigen::Matrix<Scalar, 2, 1>> &;
  using ref2_t = Eigen::Ref<Eigen::Matrix<Scalar, 2, 1>>;

  void choose_split_dimension(cref_t lb, cref_t ub, int &ii, Scalar &width) {
    if (use_weights)
      choose_split_dimension_weights(lb, ub, weights, ii, width);
    else
      choose_split_dimension_default(lb, ub, ii, width);
  }

  Scalar angular_weight = 1.0;

  Rn<Scalar, 2> l2;
  SO2<Scalar> so2;

  void print(std::ostream &out) {
    out << "R2SO2: " << std::endl;
    l2.print(out);
    so2.print(out);
  }

  vec_t weights;
  bool use_weights = false;

  void set_weights(cref_t wr2, double wso2) {
    weights.template head<2>() = wr2;
    weights(2) = wso2;
    l2.set_weights(wr2);
    so2.set_weights(wso2);
  }

  void set_bounds(cref2_t lb_, cref2_t ub_) { l2.set_bounds(lb_, ub_); }

  bool check_bounds(cref_t x) const {
    return l2.check_bounds(x.template head<2>()) &&
           so2.check_bounds(x.template tail<1>());
  }

  inline void sample_uniform(ref_t x) const {
    l2.sample_uniform(x.template head<2>());
    so2.sample_uniform(x.template tail<1>());
  }

  inline void interpolate(cref_t from, cref_t to, Scalar t, ref_t out) const {
    assert(t >= 0);
    assert(t <= 1);
    l2.interpolate(from.template head<2>(), to.template head<2>(), t,
                   out.template head<2>());
    so2.interpolate(from.template tail<1>(), to.template tail<1>(), t,
                    out.template tail<1>());
  }

  inline Scalar distance_to_rectangle(cref_t x, cref_t lb, cref_t ub) const {

    Scalar d1 = l2.distance_to_rectangle(
        x.template head<2>(), lb.template head<2>(), ub.template head<2>());
    Scalar d2 = so2.distance_to_rectangle(
        x.template tail<1>(), lb.template tail<1>(), ub.template tail<1>());
    return d1 + angular_weight * d2;
  }

  inline Scalar distance(cref_t x, cref_t y) const {

    Scalar d1 = l2.distance(x.template head<2>(), y.template head<2>());
    Scalar d2 = so2.distance(x.template tail<1>(), y.template tail<1>());
    return d1 + angular_weight * d2;
  };
};

template <typename Scalar> struct R2SO2Squared {

  using cref_t = const Eigen::Ref<const Eigen::Matrix<Scalar, 3, 1>> &;
  using ref_t = Eigen::Ref<Eigen::Matrix<Scalar, 3, 1>>;

  using cref2_t = const Eigen::Ref<const Eigen::Matrix<Scalar, 2, 1>> &;
  using ref2_t = Eigen::Ref<Eigen::Matrix<Scalar, 2, 1>>;

  void choose_split_dimension(cref_t lb, cref_t ub, int &ii, Scalar &width) {
    choose_split_dimension_default(lb, ub, ii, width);
  }

  Scalar angular_weight = 1.0;

  RnSquared<Scalar, 2> rn_squared;
  SO2Squared<Scalar> so2squared;

  void print(std::ostream &out) {
    out << "R2SO2Squared: " << std::endl;
    rn_squared.print(out);
    so2squared.print(out);
  }

  void set_bounds(cref2_t lb_, cref2_t ub_) { rn_squared.set_bounds(lb_, ub_); }

  void sample_uniform(ref_t x) const {
    rn_squared.sample_uniform(x.template head<2>());
    so2squared.sample_uniform(x.template tail<1>());
  }

  inline Scalar distance_to_rectangle(cref_t x, cref_t lb, cref_t ub) const {

    Scalar d1 = rn_squared.distance_to_rectangle(
        x.template head<2>(), lb.template head<2>(), ub.template head<2>());
    Scalar d2 = so2squared.distance_to_rectangle(
        x.template tail<1>(), lb.template tail<1>(), ub.template tail<1>());
    return d1 + angular_weight * d2;
  }

  inline Scalar distance(cref_t x, cref_t y) const {

    Scalar d1 = rn_squared.distance(x.template head<2>(), y.template head<2>());
    Scalar d2 = so2squared.distance(x.template tail<1>(), y.template tail<1>());
    return d1 + angular_weight * d2;
  };
};

template <typename Scalar> struct SO3Squared {

  using cref_t = const Eigen::Ref<const Eigen::Matrix<Scalar, 4, 1>> &;
  using ref_t = Eigen::Ref<Eigen::Matrix<Scalar, 4, 1>>;

  RnSquared<Scalar, 4> rn_squared;

  void sample_uniform(ref_t x) const {
    x = Eigen::Quaterniond().UnitRandom().coeffs();
  }

  bool check_bounds(cref_t x) const { return std::abs(x.norm() - 1) < 1e-6; }

  void print(std::ostream &out) {
    out << "SO3Squared: " << std::endl;
    rn_squared.print(out);
  }
  void set_weights(cref_t weights_) {
    THROW_PRETTY_DYNOTREE("so3 weights not implemented");
  }

  void set_bounds([[maybe_unused]] cref_t lb_, [[maybe_unused]] cref_t ub_) {

    THROW_PRETTY_DYNOTREE("so3 has no bounds");
  }

  void interpolate([[maybe_unused]] cref_t from, [[maybe_unused]] cref_t to,
                   [[maybe_unused]] Scalar t,
                   [[maybe_unused]] ref_t out) const {
    THROW_PRETTY_DYNOTREE("so3 has no interpolate implemented");
  }

  void choose_split_dimension(cref_t lb, cref_t ub, int &ii, Scalar &width) {
    choose_split_dimension_default(lb, ub, ii, width);
  }

  inline Scalar distance_to_rectangle(cref_t &x, cref_t &lb, cref_t &ub) const {

    assert(std::abs(x.norm() - 1) < 1e-6);

    Scalar d1 = rn_squared.distance_to_rectangle(x, lb, ub);
    Scalar d2 = rn_squared.distance_to_rectangle(-1. * x, lb, ub);
    return std::min(d1, d2);
  }

  inline Scalar distance(cref_t x, cref_t y) const {

    assert(x.size() == 4);
    assert(y.size() == 4);

    assert(std::abs(x.norm() - 1) < 1e-6);
    assert(std::abs(y.norm() - 1) < 1e-6);

    Scalar d1 = rn_squared.distance(x, y);
    Scalar d2 = rn_squared.distance(-x, y);
    return std::min(d1, d2);
  };
};

template <typename Scalar> struct SO3 {

  using cref_t = const Eigen::Ref<const Eigen::Matrix<Scalar, 4, 1>> &;
  using ref_t = Eigen::Ref<Eigen::Matrix<Scalar, 4, 1>>;

  SO3Squared<Scalar> so3squared;

  void print(std::ostream &out) {
    out << "SO3: " << std::endl;
    so3squared.print(out);
  }

  bool check_bounds(cref_t x) const { return std::abs(x.norm() - 1) < 1e-6; }

  void sample_uniform(ref_t x) const { so3squared.sample_uniform(x); }

  void set_bounds([[maybe_unused]] cref_t lb_, [[maybe_unused]] cref_t ub_) {
    THROW_PRETTY_DYNOTREE("so3 has no bounds");
  }

  void set_weights([[maybe_unused]] cref_t weights_) {
    THROW_PRETTY_DYNOTREE("so3 weights not implemented");
  }

  void choose_split_dimension(cref_t lb, cref_t ub, int &ii, Scalar &width) {
    choose_split_dimension_default(lb, ub, ii, width);
  }

  void interpolate(cref_t from, cref_t to, Scalar t, ref_t out) const {
    out = Eigen::Quaterniond(from).slerp(t, Eigen::Quaterniond(to)).coeffs();
  }

  inline Scalar distance_to_rectangle(cref_t &x, cref_t &lb, cref_t &ub) const {

    return std::sqrt(so3squared.distance_to_rectangle(x, lb, ub));
  }

  inline Scalar distance(cref_t x, cref_t y) const {

    return std::sqrt(so3squared.distance(x, y));
  };
};

// Rigid Body: Pose and Velocities
template <typename Scalar> struct R9SO3Squared {};

// SE3
template <typename Scalar> struct R3SO3Squared {

  using cref_t = const Eigen::Ref<const Eigen::Matrix<Scalar, 7, 1>> &;
  using ref_t = Eigen::Ref<Eigen::Matrix<Scalar, 7, 1>>;
  using cref3_t = const Eigen::Ref<const Eigen::Matrix<Scalar, 3, 1>> &;

  void choose_split_dimension(cref_t lb, cref_t ub, int &ii, Scalar &width) {
    choose_split_dimension_default(lb, ub, ii, width);
  }

  RnSquared<Scalar, 3> l2;
  SO3Squared<Scalar> so3;

  void print(std::ostream &out) {
    out << "R3SO3Squared: " << std::endl;
    l2.print(out);
    so3.print(out);
  }
  void set_bounds(cref3_t lb_, cref3_t ub_) { l2.set_bounds(lb_, ub_); }

  inline void sample_uniform(cref3_t lb, cref3_t ub, ref_t x) const {
    l2.sample_uniform(x.template head<3>());
    so3.sample_uniform(x.template tail<4>());
  }

  inline Scalar distance_to_rectangle(cref_t &x, cref_t &lb, cref_t &ub) const {

    Scalar d1 = l2.distance_to_rectangle(
        x.template head<3>(), lb.template head<3>(), ub.template head<3>());

    Scalar d2 = so3.distance_to_rectangle(
        x.template tail<4>(), lb.template tail<4>(), ub.template tail<4>());

    return d1 + d2;
  }

  inline Scalar distance(cref_t x, cref_t y) const {

    Scalar d1 = l2.distance(x.template head<3>(), y.template head<3>());
    Scalar d2 = so3.distance(x.template tail<4>(), y.template tail<4>());
    return d1 + d2;
  };
};

template <typename Scalar> struct R3SO3 {

  using cref_t = const Eigen::Ref<const Eigen::Matrix<Scalar, 7, 1>> &;
  using ref_t = Eigen::Ref<Eigen::Matrix<Scalar, 7, 1>>;

  using cref3_t = const Eigen::Ref<const Eigen::Matrix<Scalar, 3, 1>> &;
  using ref3_t = Eigen::Ref<Eigen::Matrix<Scalar, 3, 1>>;

  void choose_split_dimension(cref_t lb, cref_t ub, int &ii, Scalar &width) {
    choose_split_dimension_default(lb, ub, ii, width);
  }

  void interpolate(cref_t from, cref_t to, Scalar t, ref_t out) const {
    l2.interpolate(from.template head<3>(), to.template head<3>(), t,
                   out.template head<3>());
    so3.interpolate(from.template tail<4>(), to.template tail<4>(), t,
                    out.template tail<4>());
  }

  Rn<Scalar, 3> l2;
  SO3<Scalar> so3;

  void print(std::ostream &out) {
    out << "R3SO3: " << std::endl;
    l2.print(out);
    so3.print(out);
  }

  void set_bounds(cref3_t lb_, cref3_t ub_) { l2.set_bounds(lb_, ub_); }

  void sample_uniform(ref_t x) const {
    l2.sample_uniform(x.template head<3>());
    so3.sample_uniform(x.template tail<4>());
  }

  inline Scalar distance_to_rectangle(cref_t &x, cref_t &lb, cref_t &ub) const {

    Scalar d1 = l2.distance_to_rectangle(
        x.template head<3>(), lb.template head<3>(), ub.template head<3>());

    Scalar d2 = so3.distance_to_rectangle(
        x.template tail<4>(), lb.template tail<4>(), ub.template tail<4>());

    return d1 + d2;
  }

  inline Scalar distance(cref_t x, cref_t y) const {

    Scalar d1 = l2.distance(x.template head<3>(), y.template head<3>());
    Scalar d2 = so3.distance(x.template tail<4>(), y.template tail<4>());
    return d1 + d2;
  };
};

enum class DistanceType {
  RnL1,
  Rn,
  RnSquared,
  SO2,
  SO2Squared,
  SO3,
  SO3Squared
};

inline bool starts_with(const std::string &str, const std::string &prefix) {
  return str.size() >= prefix.size() &&
         str.compare(0, prefix.size(), prefix) == 0;
}

// get the substring after : as an integer number
inline int get_number(const std::string &str) {
  std::string delimiter = ":";
  size_t pos = 0;
  std::string token;
  // while ((
  pos = str.find(delimiter);
  if (pos == std::string::npos) {
    THROW_PRETTY_DYNOTREE("delimiter not found");
  }

  token = str.substr(pos + delimiter.length(), str.size());
  int out = std::stoi(token);
  // std::cout << "out " << out << std::endl;
  return out;
}

template <typename Scalar> struct Combined {
  using cref_t = const Eigen::Ref<const Eigen::Matrix<Scalar, -1, 1>> &;
  using ref_t = Eigen::Ref<Eigen::Matrix<Scalar, -1, 1>>;

  using Space =
      std::variant<RnL1<Scalar>, Rn<Scalar>, RnSquared<Scalar>, SO2<Scalar>,
                   SO2Squared<Scalar>, SO3<Scalar>, SO3Squared<Scalar>>;
  std::vector<Space> spaces;
  std::vector<int> dims; // TODO: remove this and get auto from spaces
  // std::vector<double>
  //     weights; // Only supported: one weight per space -- TODO: allow both!
  Eigen::Matrix<Scalar, -1, 1>
      weights; // one weight per dimension, created from weights
  bool use_weights = false;
  std::vector<std::string> spaces_names;
  Eigen::VectorXd lb;
  Eigen::VectorXd ub;

  void set_weights(cref_t weights_) {
    int total_dim = get_runtime_dim();
    CHECK_PRETTY_DYNOTREE(weights_.size() == total_dim, "");
    weights = weights_;
    use_weights = true;
    int counter = 0;
    for (size_t i = 0; i < spaces.size(); i++) {
      std::visit(
          [&](auto &obj) {
            obj.set_weights(weights_.segment(counter, dims[i]));
          },
          spaces[i]);
      counter += dims[i];
    }
  }

  int get_runtime_dim() {
    int out = 0;
    for (size_t i = 0; i < spaces.size(); i++) {
      out += dims[i];
      // std::visit(
      //     [&](auto &obj) {
      //       out += obj.get_runtime_dim();
      //     },
      //     spaces[i]);
    }
    return out;
  }

  Combined() = default;

  Combined(const std::vector<Space> &spaces, const std::vector<int> &dims)
      : spaces(spaces), dims(dims) {
    assert(spaces.size() == dims.size());
  }

  void print(std::ostream &out) {

    out << "Combined: " << std::endl;
    for (auto &a : spaces_names)
      out << a << std::endl;
    for (auto &s : spaces)
      std::visit([&](auto &obj) { obj.print(out); }, s);
    for (auto &d : dims)
      out << d << std::endl;
    out << "lb " << lb.transpose().format(__CleanFmt) << std::endl;
    out << "ub " << ub.transpose().format(__CleanFmt) << std::endl;
    out << "weights " << weights.transpose().format(__CleanFmt) << std::endl;
  }

  Combined(const std::vector<std::string> &spaces_str) {

    for (size_t i = 0; i < spaces_str.size(); i++) {
      if (spaces_str.at(i) == "SO2") {
        spaces.push_back(SO2<Scalar>());
        spaces_names.push_back("SO2");
        dims.push_back(1);
      } else if (spaces_str.at(i) == "SO2Squared") {
        spaces.push_back(SO2Squared<Scalar>());
        spaces_names.push_back("SO2Squared");
        dims.push_back(1);
      } else if (spaces_str.at(i) == "SO3") {
        spaces.push_back(SO3<Scalar>());
        spaces_names.push_back("SO3");
        dims.push_back(4);
      } else if (spaces_str.at(i) == "SO3Squared") {
        spaces.push_back(SO3Squared<Scalar>());
        spaces_names.push_back("SO3Squared");
        dims.push_back(4);
      } else if (starts_with(spaces_str.at(i), "RnL1")) {
        spaces.push_back(RnL1<Scalar>());
        spaces_names.push_back("RnL1");
        int dim = get_number(spaces_str.at(i));
        dims.push_back(dim);
      } else if (starts_with(spaces_str.at(i), "Rn") &&
                 !starts_with(spaces_str.at(i), "RnSquared")) {
        spaces.push_back(Rn<Scalar>());
        spaces_names.push_back("Rn");
        int dim = get_number(spaces_str.at(i));
        dims.push_back(dim);
      } else if (starts_with(spaces_str.at(i), "RnSquared")) {
        spaces.push_back(RnSquared<Scalar>());
        spaces_names.push_back("RnSquared");
        int dim = get_number(spaces_str.at(i));
        dims.push_back(dim);
      } else {
        THROW_PRETTY_DYNOTREE("Unknown space:" + spaces_str.at(i));
      }
    }
    assert(spaces.size() == dims.size());
  }

  void choose_split_dimension(cref_t lb, cref_t ub, int &ii, Scalar &width) {
    if (use_weights) {
      choose_split_dimension_weights(lb, ub, weights, ii, width);
    } else
      choose_split_dimension_default(lb, ub, ii, width);
  }

  bool check_bounds(cref_t x) const {
    int counter = 0;
    for (size_t i = 0; i < spaces.size(); i++) {
      if (!std::visit(
              [&](const auto &obj) {
                return obj.check_bounds(x.segment(counter, dims[i]));
              },
              spaces[i]))
        return false;

      counter += dims[i];
    }
    return true;
  }

  void set_bounds(cref_t &lbs, cref_t &ubs) {

    assert(lbs.size() == ubs.size());
    lb = lbs;
    ub = ubs;
    int counter = 0;
    for (size_t i = 0; i < spaces_names.size(); i++) {

      auto &space_name = spaces_names.at(i);

      if (space_name == "SO2")
        continue;
      if (space_name == "SO2Squared")
        continue;
      if (space_name == "SO3")
        continue;
      if (space_name == "SO3Squared")
        continue;

      std::visit(
          [&](auto &obj) {
            if (lbs.size())
              obj.set_bounds(lbs.segment(counter, dims[i]),
                             ubs.segment(counter, dims[i]));
          },
          spaces[i]);
      counter += dims[i];
    }
  }

  void set_bounds(const std::vector<Eigen::VectorXd> &lbs,
                  const std::vector<Eigen::VectorXd> &ubs) {

    assert(lbs.size() == ubs.size());

    for (size_t i = 0; i < lbs.size(); i++) {
      std::visit(
          [&](auto &obj) {
            if (lbs[i].size())
              obj.set_bounds(lbs[i], ubs[i]);
          },
          spaces[i]);
    }
  }

  void sample_uniform(ref_t x) const {

    assert(spaces.size() == dims.size());
    assert(spaces.size());
    int counter = 0;
    for (size_t i = 0; i < spaces.size(); i++) {
      std::visit(
          [&](const auto &obj) {
            obj.sample_uniform(x.segment(counter, dims[i]));
          },
          spaces[i]);
      counter += dims[i];
    }
  }

  void interpolate(cref_t from, cref_t to, Scalar t, ref_t out) const {

    assert(spaces.size() == dims.size());
    assert(spaces.size());
    Scalar d = 0;
    int counter = 0;
    for (size_t i = 0; i < spaces.size(); i++) {
      std::visit(
          [&](const auto &obj) {
            obj.interpolate(from.segment(counter, dims[i]),
                            to.segment(counter, dims[i]), t,
                            out.segment(counter, dims[i]));
          },
          spaces[i]);
      counter += dims[i];
    }
  }

  inline Scalar distance(cref_t x, cref_t y) const {

    assert(spaces.size() == dims.size());
    assert(spaces.size());
    Scalar d = 0;
    int counter = 0;
    int dim_index = -1;
    auto caller = [&](const auto &obj) {
      return obj.distance(x.segment(counter, dims[dim_index]),
                          y.segment(counter, dims[dim_index]));
    };

    for (size_t i = 0; i < spaces.size(); i++) {
      dim_index = i;
      d += std::visit(caller, spaces[i]);
      counter += dims[i];
    }
    return d;
  }

  inline Scalar distance_to_rectangle(cref_t x, cref_t lb, cref_t ub) const {

    assert(spaces.size() == dims.size());
    assert(spaces.size());

    Scalar d = 0;
    int counter = 0;

    int dim_index = 0;
    auto caller = [&](const auto &obj) {
      return obj.distance_to_rectangle(x.segment(counter, dims[dim_index]),
                                       lb.segment(counter, dims[dim_index]),
                                       ub.segment(counter, dims[dim_index]));
    };

    for (size_t i = 0; i < spaces.size(); i++) {
      dim_index = i;
      d += std::visit(caller, spaces[i]);
      counter += dims[i];
    }

    return d;
  }
};
} // namespace dynotree
