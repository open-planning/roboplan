# roboplan_aligator

Trajectory-optimization satellite wrapping the aligator proximal-DDP solver
(`aligator::SolverProxDDPTpl<double>`) over Pinocchio free-space multibody dynamics.
Authoritative spec: `docs/design/roboplan_trajopt_design_v4.md`. Public C++ namespace is
`roboplan`; Python module is `roboplan.aligator`; compiled extension is `_aligator_ext`;
solver class is `TrajectoryOptimizer`.

- Depends only on core `roboplan` (PUBLIC). aligator is resolved via
  `find_package(aligator CONFIG)` and otherwise fetched via CMake FetchContent pinned to
  commit `c54d9645` (approved dependency; acquisition recorded in `API_NOTES.md`). It is
  linked **PRIVATE** — aligator types (and pinocchio, which aligator links PUBLIC) must
  never leak into any public `include/roboplan_aligator/*.hpp` nor the exported link
  interface. `pinocchio`/`Eigen3` arrive transitively through `roboplan::roboplan`.
- aligator's FetchContent build toggles `BUILD_TESTING`; the package's own test gating
  uses `BUILD_TESTING_ALIGATOR` (captured before the fetch, mirroring `roboplan_oink`).
- Every aligator and pinocchio symbol must be verified against pinned source and recorded
  in `API_NOTES.md` with a `file:line` citation before use (`.claude/rules/source-verification.md`,
  `verify-api` skill).
- Follow the roboplan numerics rules: `nq` for configs, `nv` for velocities/torques/Jacobians
  (never assume `nq == nv`); one pre-allocated `pinocchio::Data` per solver instance;
  `tl::expected<T, std::string>` for recoverable failures, exceptions for setup/lifecycle
  invariant violations.
- Bindings mirror C++ names 1:1 (camelCase methods), return by value, use
  `unwrap_expected()` for `tl::expected`, and `import_("roboplan.core")` before referencing
  core types (`.claude/rules/bindings.md`).
