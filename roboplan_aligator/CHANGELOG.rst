^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package roboplan_aligator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Initial package scaffolding: CMake/ament packaging, nanobind bindings shell, and the
  aligator dependency wiring (find_package first, pinned FetchContent fallback, linked
  PUBLIC).
* Solver diagnostics: ``TrajOptOptions.record_history`` / ``TrajOptResult.history`` (wraps
  aligator's ``HistoryCallbackTpl``) and ``TrajectoryOptimizer::registerCallback`` (C++-only
  passthrough to a raw aligator callback).
* Solver performance tunables: ``TrajOptOptions.linear_solver_choice`` (serial/parallel/staged-dense
  Riccati backend), ``num_threads``, and ``rollout_type`` (linear/nonlinear forward pass).
* Cost/constraint file layout split one type per header+source pair under ``costs/``/
  ``constraints/``, mirroring ``roboplan_oink``; ``attachConfigurationCost``/``attachVelocityCost``
  de-duplicated via a shared ``attachMaskedStateCost`` helper.
* **Breaking**: ``StageWindow`` and ``CostHandle`` removed. Cost/constraint targeting is now three
  explicit methods per kind -- ``addCost``/``addConstraint`` (every stage), ``addStageCost``/
  ``addStageConstraint(stage, ...)`` (exactly one stage index), ``addTerminalCost``/
  ``addTerminalConstraint`` (the terminal node) -- assembled once inside ``build()``, mirroring
  aligator's own per-index ``addStage`` loop rather than building empty stages and decorating them
  afterward. There is no post-build cost/constraint mutation; retargeting means
  ``resetProblem()`` + re-adding + ``build()``. Added ``TrajectoryOptimizer::setStageFactory``
  (C++-only): a caller-supplied per-index factory that builds the entire aligator ``StageModel``
  itself, for callers who want direct, unmediated access to aligator's own API.
* Contributors: Sebastian Jahr
