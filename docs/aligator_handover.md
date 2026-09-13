# Handover: roboplan_aligator follow-up work

Status snapshot for a fresh session picking up work on `roboplan_aligator` (trajectory
optimization satellite wrapping aligator's proximal-DDP solver). Branch: `pr-aligator-trajopt-dev`.

This note is about *what's done and what's next*. For how the code and aligator itself are
structured, read `docs/aligator_structure.md` first — this note assumes that context.

## How this work stream started

A code-quality review compared `roboplan_aligator` against upstream aligator's own example usage
(`external/aligator/examples/`) and produced a ranked list of concrete improvements. Two items on
that list (a Cartesian end-effector cost/constraint, and a collision constraint) were already
implemented once, then deleted from the package (commits `3430ba2`, `9190708`) — the user has said
those are coming back **as a separate follow-up from elsewhere**; do not implement them here.

The remaining ranked items, and their status:

1. ~~Solver diagnostics (history + raw callback registration)~~ — **done**, commit `8bfdc51`.
2. ~~Solver performance tunables (`linear_solver_choice`/`num_threads`/`rollout_type`)~~ — **done**,
   commit `1cc21eb`.
3. **Test the "direct aligator" C++ custom-cost/constraint escape hatch end-to-end** — explicitly
   **skipped for now** (user said "skip 3 for now"). Not started. **Next up.**
4. ~~Structural cleanup, borrowing from `roboplan_oink`'s conventions~~ — **done**, uncommitted.
   See "What was actually implemented for #4" below (see "Item 4: what to actually do" further down
   for the original spec this satisfies).
5. Promote the `CostHandle` multi-stage-closure behavior and the value-polymorphism
   `xyz::polymorphic` copy-on-insert gotcha from `API_NOTES.md`'s narrative into a short, explicit
   rule in `CLAUDE.md` (mirroring how `roboplan_oink/CLAUDE.md` states its own load-bearing
   gotchas as bullet rules). Not started; lowest priority, cheap to do alongside item 3.

## What was actually implemented for #1 and #2 (brief)

- `TrajOptOptions::record_history` (default `false`) + `TrajOptResult::history` (a
  `std::vector<TrajOptIterate>` with `iteration`/`cost`/`prim_infeas`/`dual_infeas`) — wraps
  aligator's own `HistoryCallbackTpl`, cleared at the start of every `solve()`.
  `TrajectoryOptimizer::registerCallback(name, callback)` is the C++-only raw-callback passthrough
  (Tier 2). See `API_NOTES.md` "Prompt 13" for full citations, and `docs/aligator_structure.md`
  §8 for the design rationale (in particular: why the history callback is never moved across
  `TrajectoryOptimizer` instances).
- `TrajOptOptions::linear_solver_choice` / `num_threads` / `rollout_type` — applied once inside
  `build()`, **not** re-applied every `solve()` like `tol`/`mu_init`/`max_iters`/`verbose` are (this
  is forced by aligator's own internals, not a style choice — see `API_NOTES.md` "Prompt 14" and
  `docs/aligator_structure.md` §8 for why). Tests cover the Parallel+NonLinear incompatibility
  throwing at `build()`, and confirm the Parallel backend actually solves in this environment
  (OpenMP support is compiled in).

Both are covered by new C++ tests in `test/test_trajectory_optimizer.cpp` and were verified to
build and pass (`pixi run build roboplan_aligator`, `pixi run test roboplan_aligator` — 41/41
passing as of commit `1cc21eb`) plus a manual Python binding smoke check.

## What was actually implemented for #4 (brief)

All four sub-points of "Item 4: what to actually do" (below) are done, via a dispatched subagent
whose diff was independently reviewed and re-verified (clean-rebuilt + retested from scratch, not
just trusting the agent's own report):

- Spec structs split one-per-file: `include/roboplan_aligator/costs/{configuration_cost,
  velocity_cost}.hpp`, `include/roboplan_aligator/constraints/torque_limit.hpp`. `CostHandle`
  (shared infra, not a cost type) moved to its own top-level `cost_handle.hpp`, mirroring how
  oink's shared `Task` base class lives outside `tasks/`. Old `costs.hpp`/`constraints.hpp`
  deleted; no umbrella header — every consumer includes the exact file it needs
  (`trajectory_optimizer.hpp`, `cost_spec.hpp`, `constraint_spec.hpp`, both factory headers,
  `test_costs.cpp`, `test_constraints.cpp`, `bindings/src/aligator.cpp`).
- Factory implementations split the same way: `src/costs/configuration_cost.cpp`,
  `src/costs/velocity_cost.cpp`, `src/constraints/torque_limit.cpp`. `cost_factory.hpp`/
  `constraint_factory.hpp` were deliberately *not* split further — they're declaration surfaces
  for a whole domain's free functions, not a single type, and there's no oink analogue for that
  indirection layer (see point 4 of the original spec below). `src/constraint_factory.cpp` was
  deleted outright (nothing was left in it once `buildTorqueLimit` moved out); `src/cost_factory.cpp`
  now holds only `CostHandle::Impl` and the public `CostHandle` method bodies.
- `attachConfigurationCost`/`attachVelocityCost` de-duplicated via a new internal helper,
  `aligator_detail::attachMaskedStateCost` (`costs/masked_state_cost.{hpp,cpp}`): both callers now
  only build their own target vector + weight mask and delegate the insert/`dynamic_cast`/closure
  plumbing to it.
- The `Options`-struct convention (point 2 of the spec) was **documented, not retrofitted** — a
  bullet was added to `roboplan_aligator/CLAUDE.md` describing the pattern for future types (e.g.
  the upcoming Cartesian pose cost), since `ConfigurationCost`/`VelocityCost`/`TorqueLimit` are too
  simple today to need it themselves.
- `CostSpec`/`ConstraintSpec` + the factory indirection layer (point 4) were left untouched, as
  specified.
- `docs/aligator_structure.md` §6 was updated to match (file paths in the class diagram, the
  sequence diagram's `cost_factory` participant, and a new paragraph describing the file layout
  and the dedup helper) — it had gone stale the moment this refactor landed.

Verified independently: clean-removed the built `.o`/`.so` artifacts and reran
`pixi run build roboplan_aligator` (0 warnings from any new/changed file) and
`pixi run test roboplan_aligator` — 41/41 passing, same count as before the refactor. Nothing from
this item is committed yet.

## Important: uncommitted documentation state

`roboplan_aligator/API_NOTES.md`, `CLAUDE.md`, and `CHANGELOG.rst` are currently **untracked** on
this branch (no git history at all here, despite `git log --all` showing commits that touched
similarly-named files elsewhere — those are not ancestors of this branch). They contain:

- The full "Prompt 0" through "Prompt 14" narrative (dependency approval, `ReducedGroupModel`,
  problem shell, cost/constraint factories, collision residual investigation, MPC-cycling
  decision, and now the diagnostics/tunables work from this session).
- `CLAUDE.md`'s package rules (two cost/constraint API paths, numerics conventions, the
  diagnostics/tunables bullets, and now the cost/constraint file-layout + `Options`-struct
  convention bullet from item 4's work).
- A `CHANGELOG.rst` with entries for the initial scaffolding plus this session's two features.

**This was flagged to the user but not resolved**: committing these files now would bundle a large
amount of pre-existing, previously-uncommitted documentation under whatever commit message
describes a much smaller change. Before doing more work that touches these three files, either
ask the user how they want them committed (one bulk "docs" commit? split by Prompt?), or at least
re-flag it — don't assume silently either way.

Also note: `API_NOTES.md` documents some things (the collision residual, `FrameAxisCost`,
`FramePoseCost`, MPC-cycling primitives) as if currently implemented, when the corresponding
source was actually deleted in `3430ba2`/`9190708`. This is pre-existing drift, not introduced this
session — worth a cleanup pass whenever the Cartesian/collision follow-up lands and reconciles it,
but not urgent on its own.

## Item 4: what to actually do

The goal (from the original review): make `roboplan_aligator`'s cost/constraint code follow the
same conventions `roboplan_oink` already uses successfully, so that when the Cartesian
cost/constraint and collision constraint land (from the separate follow-up), they have a
low-friction place to go instead of growing the current monolithic files further.

Concretely:

1. **Split `costs.hpp`/`constraints.hpp` (and `cost_factory.cpp`/`constraint_factory.cpp`) into
   one file per type**, in `costs/`/`constraints/` subdirectories — mirroring
   `roboplan_oink/include/roboplan_oink/{barriers,constraints,tasks}/*.hpp` (one class, one
   header+source pair, per concept). Currently: `ConfigurationCost`/`VelocityCost` share
   `costs.hpp`; `TorqueLimit` alone is in `constraints.hpp`. Do this split even though there are
   currently only 2-3 types total — it's cheap now and expensive to retrofit once the Cartesian/
   collision types land on top of it.
2. **Adopt an `Options`-struct convention for any cost/constraint with more than 1-2 optional
   tunables**, matching `roboplan_oink`'s `FrameTaskOptions`/`ConstraintAxisSelection` pattern
   (small POD, sensible defaults, passed by const-ref with a default `{}`). `ConfigurationCost`/
   `VelocityCost`/`TorqueLimit` are simple enough today that this isn't strictly needed for them,
   but the convention should be established before the Cartesian pose cost (which will likely want
   position/orientation weight split, a gain, a max-error clamp — exactly `FrameTaskOptions`-shaped)
   lands.
3. **De-duplicate `attachConfigurationCost`/`attachVelocityCost`** (`cost_factory.cpp`) — they are
   ~90% identical (build a padded/masked target and weight matrix, insert into the stack, recover
   the stored pointer via the item returned by `addCost`, build a `setTarget` closure). Extract the
   shared insert/cast/closure-building logic into one helper (e.g.
   `attachMaskedStateCost(stack, space, weight, target, weight_matrix) -> setter closure`), and have
   both `attachConfigurationCost`/`attachVelocityCost` just build their respective target/weight
   masks and call it. This removes the main piece of copy-paste risk for whoever adds the next
   `QuadraticStateCostTpl`-based cost type.
4. **Keep the `CostSpec`/`ConstraintSpec` variant + factory indirection layer as-is.** This is
   *not* the same problem `roboplan_oink`'s `Barrier`/`Task` inheritance solves — those are
   roboplan-owned base classes, so a new subclass just works with no serialization boundary.
   Aligator's cost/constraint types are third-party polymorphic-value types
   (`xyz::polymorphic<CostAbstractTpl<double>>`) that nanobind can't bind directly; the
   `Spec`/factory layer exists specifically to translate closed, POD, Python-bindable values into
   that foreign type system. Do not try to remove it or replace it with inheritance — that was
   considered and rejected already (see `docs/Aligator_OInK_Comparison_and_Review.md`, though note
   that document is itself stale on a different point — it argued for keeping PIMPL, which was
   removed anyway shortly after it was written; treat that whole document with a grain of salt and
   prefer `CLAUDE.md`/`API_NOTES.md` as current source of truth).

After the split, rebuild and run the full package test suite
(`pixi run build roboplan_aligator && pixi run test roboplan_aligator`) to confirm nothing broke —
this is a pure refactor, no behavior should change, so every existing test should still pass
unmodified.

## Process reminders for whoever picks this up

- This repo enforces `.claude/rules/source-verification.md`: verify every aligator/pinocchio symbol
  against the pinned source (vendored at `external/aligator/`, commit `c54d96450112a1bba0e3f798ac0d1f6887412df3`)
  and record a `file:line` citation in `API_NOTES.md` *before* writing code against it. Use the
  `verify-api` skill.
- `.claude/rules/numerics.md` and `.claude/rules/testing.md` apply throughout (nq vs nv, one
  `pinocchio::Data` per solver, `tl::expected` for recoverable failures, every test tolerance needs
  a comment justifying it, solver/planner tests must assert same-seed determinism).
- Commit messages in this session were kept to a single 72-char summary line; the user has been
  approving commits per-feature, not batching multiple features into one commit.
