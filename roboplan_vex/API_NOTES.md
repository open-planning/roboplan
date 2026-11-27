# roboplan_vex — third-party API notes

Per `.claude/rules/source-verification.md`, every third-party API this package codes against is
recorded here with a `file:line` citation into the pinned source.

## ProxQP sparse solver (proxsuite)

**Pin:** proxsuite v0.8.2 (`CMakeLists.txt`: `find_package(proxsuite)` first, otherwise
FetchContent `GIT_TAG v0.8.2`). Header-only, linked `PRIVATE`. The citations were checked against
the installed 0.8.2 headers of the pixi `default` environment.

Citations are relative to the proxsuite include root, `proxsuite/`.

Used from `src/qp_backend.cpp` (include: `<proxsuite/proxqp/sparse/sparse.hpp>`).

### Solver object — `proxsuite::proxqp::sparse::QP<T, I>`

| Symbol | Signature | Citation |
|---|---|---|
| Class | `template<typename T, typename I> struct QP` with members `results`, `settings` | `proxqp/sparse/wrapper.hpp:90` |
| Constructor | `QP(isize dim, isize n_eq, isize n_in)` | `proxqp/sparse/wrapper.hpp:103` |
| `init` | `void init(optional<SparseMat<T, I>> H, optional<VecRef<T>> g, optional<SparseMat<T, I>> A, optional<VecRef<T>> b, optional<SparseMat<T, I>> C, optional<VecRef<T>> l, optional<VecRef<T>> u, bool compute_preconditioner_ = true, optional<T> rho = nullopt, optional<T> mu_eq = nullopt, optional<T> mu_in = nullopt, optional<T> manual_minimal_H_eigenvalue = nullopt)` | `proxqp/sparse/wrapper.hpp:170` |
| `solve` | `void solve()` | `proxqp/sparse/wrapper.hpp:642` |
| `solve` (warm start) | `void solve(optional<VecRef<T>> x, optional<VecRef<T>> y, optional<VecRef<T>> z)` | `proxqp/sparse/wrapper.hpp:657` |

`init` reads only the upper triangle of `H`: `(H.value()).template triangularView<Eigen::Upper>()`
(`proxqp/sparse/wrapper.hpp:300-301`).

`init` must always get `H`: its `H == nullopt` branch still evaluates `H.value()`
(`proxqp/sparse/wrapper.hpp:319`, inside the `else` of `if (H != nullopt)` at `:299`), so a missing
`H` is invalid. `qp_backend.cpp` always passes `H` for this reason.

### Types

| Symbol | Definition | Citation |
|---|---|---|
| `sparse::SparseMat<T, I>` | `Eigen::SparseMatrix<T, Eigen::ColMajor, I>` | `proxqp/sparse/fwd.hpp:31` |
| `sparse::VecRef<T>` | `Eigen::Ref<Eigen::Matrix<T, DYN, 1> const>` | `proxqp/sparse/fwd.hpp:34` |
| `proxsuite::optional<T>` | `std::optional<T>` (or `tl::optional<T>` without C++17) | `helpers/optional.hpp:22`, `:40` |
| `proxsuite::nullopt` | `std::nullopt` (or the `tl::nullopt` instance) | `helpers/optional.hpp:24`, `:42` |

### Settings — `QP::settings` (`proxsuite::proxqp::Settings<T>`)

| Field | Type | Citation |
|---|---|---|
| `eps_abs` | `T` | `proxqp/settings.hpp:113` |
| `eps_rel` | `T` | `proxqp/settings.hpp:114` |
| `max_iter` | `isize` | `proxqp/settings.hpp:116` |
| `verbose` | `bool` | `proxqp/settings.hpp:122` |
| `eps_primal_inf` | `T` | `proxqp/settings.hpp:134` |

`mu_in` is an argument of `init` (above), not a settings field.

### Results — `QP::results` (`proxsuite::proxqp::Results<T>`)

| Field | Type | Citation |
|---|---|---|
| `x` | `sparse::Vec<T>` | `proxqp/results.hpp:73` |
| `info` | `Info<T>` | `proxqp/results.hpp:82` |
| `info.status` | `QPSolverOutput` | `proxqp/results.hpp:44` |

### Status — `proxsuite::proxqp::QPSolverOutput`

`enum struct QPSolverOutput` at `proxqp/status.hpp:17`. The values used here are
`PROXQP_SOLVED` (`:19`), `PROXQP_MAX_ITER_REACHED` (`:20`), `PROXQP_PRIMAL_INFEASIBLE` (`:21`) and
`PROXQP_DUAL_INFEASIBLE` (`:24`). Everything else falls into the `default` case.
