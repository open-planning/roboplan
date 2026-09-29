window.BENCHMARK_DATA = {
  "lastUpdate": 1790651916322,
  "repoUrl": "https://github.com/open-planning/roboplan",
  "entries": {
    "Benchmark": [
      {
        "commit": {
          "author": {
            "name": "open-planning",
            "username": "open-planning"
          },
          "committer": {
            "name": "open-planning",
            "username": "open-planning"
          },
          "id": "1390424e223adaaff416a1f42e1f9f385cf4a3e3",
          "message": "Add benchmarks and actually check CI",
          "timestamp": "2026-09-28T16:35:31Z",
          "url": "https://github.com/open-planning/roboplan/pull/357/commits/1390424e223adaaff416a1f42e1f9f385cf4a3e3"
        },
        "date": 1790651915758,
        "tool": "pytest",
        "benches": [
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[so101]",
            "value": 1.7148909204581977,
            "unit": "iter/sec",
            "range": "stddev: 0.4693640809422838",
            "extra": "mean: 583.1274678 msec\nrounds: 5"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[so101]",
            "value": 7.873719811895632,
            "unit": "iter/sec",
            "range": "stddev: 0.0683900730211882",
            "extra": "mean: 127.00477333333579 msec\nrounds: 24"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[kinova]",
            "value": 3.2206435646622773,
            "unit": "iter/sec",
            "range": "stddev: 0.12191830071476292",
            "extra": "mean: 310.4969488000023 msec\nrounds: 5"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[kinova]",
            "value": 7.625481160521627,
            "unit": "iter/sec",
            "range": "stddev: 0.09173363741332",
            "extra": "mean: 131.13926570000132 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[ur5]",
            "value": 1.8628787440932197,
            "unit": "iter/sec",
            "range": "stddev: 0.5079338647316666",
            "extra": "mean: 536.8035913076903 msec\nrounds: 13"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[ur5]",
            "value": 3.0194991327486598,
            "unit": "iter/sec",
            "range": "stddev: 0.5823827913252341",
            "extra": "mean: 331.18075417021123 msec\nrounds: 47"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[franka]",
            "value": 9.865084864154156,
            "unit": "iter/sec",
            "range": "stddev: 0.06069557887876339",
            "extra": "mean: 101.36760238460869 msec\nrounds: 13"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[franka]",
            "value": 27.890995238842123,
            "unit": "iter/sec",
            "range": "stddev: 0.010079447275121821",
            "extra": "mean: 35.8538657884592 msec\nrounds: 52"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[dual]",
            "value": 0.3922701186660971,
            "unit": "iter/sec",
            "range": "stddev: 1.3641472353644701",
            "extra": "mean: 2.5492637659999957 sec\nrounds: 5"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[dual]",
            "value": 12.471018133296969,
            "unit": "iter/sec",
            "range": "stddev: 0.024687512540151495",
            "extra": "mean: 80.18591499999924 msec\nrounds: 21"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-no_obstacles]",
            "value": 67.52681144654014,
            "unit": "iter/sec",
            "range": "stddev: 0.011356191578242874",
            "extra": "mean: 14.80893260881544 msec\nrounds: 363"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-with_obstacles]",
            "value": 88.09445513359162,
            "unit": "iter/sec",
            "range": "stddev: 0.009301362464880709",
            "extra": "mean: 11.35145223934402 msec\nrounds: 305"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-with_obstacles]",
            "value": 433.32509345864776,
            "unit": "iter/sec",
            "range": "stddev: 0.00011876416207058834",
            "extra": "mean: 2.3077361895161745 msec\nrounds: 496"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-no_obstacles]",
            "value": 397.42112312725294,
            "unit": "iter/sec",
            "range": "stddev: 0.0002153374871188758",
            "extra": "mean: 2.516222570484265 msec\nrounds: 454"
          }
        ]
      }
    ]
  }
}