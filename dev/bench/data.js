window.BENCHMARK_DATA = {
  "lastUpdate": 1791388073455,
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
      },
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
          "id": "964ebe9c3df7fc03b6fb4b4049bf91ae363957b3",
          "message": "Add benchmarks and actually check CI",
          "timestamp": "2026-10-07T11:27:07Z",
          "url": "https://github.com/open-planning/roboplan/pull/357/commits/964ebe9c3df7fc03b6fb4b4049bf91ae363957b3"
        },
        "date": 1791388072294,
        "tool": "pytest",
        "benches": [
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.Bounded]",
            "value": 464.1912300058815,
            "unit": "iter/sec",
            "range": "stddev: 0.00017644765444384982",
            "extra": "mean: 2.154284560669812 msec\nrounds: 478"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.TimeOptimal]",
            "value": 841.4304322432565,
            "unit": "iter/sec",
            "range": "stddev: 0.000045798216781182216",
            "extra": "mean: 1.1884523802330234 msec\nrounds: 860"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[so101]",
            "value": 4828.633180302709,
            "unit": "iter/sec",
            "range": "stddev: 0.000004282475630168403",
            "extra": "mean: 207.0979431776405 usec\nrounds: 4910"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-no_obstacles]",
            "value": 83.88714401654474,
            "unit": "iter/sec",
            "range": "stddev: 0.008875614509006823",
            "extra": "mean: 11.920777751150688 msec\nrounds: 434"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-no_obstacles]",
            "value": 474.48701353440856,
            "unit": "iter/sec",
            "range": "stddev: 0.00009141336008867649",
            "extra": "mean: 2.107539240223026 msec\nrounds: 537"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.Bounded]",
            "value": 290.00562434932004,
            "unit": "iter/sec",
            "range": "stddev: 0.00003590216415455501",
            "extra": "mean: 3.4482089864418337 msec\nrounds: 295"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.TimeOptimal]",
            "value": 511.56753610457633,
            "unit": "iter/sec",
            "range": "stddev: 0.000028085529654542372",
            "extra": "mean: 1.9547761134623225 msec\nrounds: 520"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[kinova]",
            "value": 1959.8303904597003,
            "unit": "iter/sec",
            "range": "stddev: 0.000008733581801888688",
            "extra": "mean: 510.2482362085623 usec\nrounds: 1994"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-with_obstacles]",
            "value": 514.3738974426136,
            "unit": "iter/sec",
            "range": "stddev: 0.0001016247100907402",
            "extra": "mean: 1.944111093062543 msec\nrounds: 591"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-with_obstacles]",
            "value": 114.26313788772649,
            "unit": "iter/sec",
            "range": "stddev: 0.006935909888080832",
            "extra": "mean: 8.751728846993396 msec\nrounds: 366"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.Bounded]",
            "value": 218.2460602984489,
            "unit": "iter/sec",
            "range": "stddev: 0.00006341876621784274",
            "extra": "mean: 4.581984200001191 msec\nrounds: 225"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.TimeOptimal]",
            "value": 385.07665939039043,
            "unit": "iter/sec",
            "range": "stddev: 0.000020539677335133854",
            "extra": "mean: 2.596885517764401 msec\nrounds: 394"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[ur5]",
            "value": 2707.1986965024853,
            "unit": "iter/sec",
            "range": "stddev: 0.00003134781374612189",
            "extra": "mean: 369.38552064609496 usec\nrounds: 2785"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.Bounded]",
            "value": 23.442420985395344,
            "unit": "iter/sec",
            "range": "stddev: 0.00024023901944389707",
            "extra": "mean: 42.65771016666756 msec\nrounds: 24"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.TimeOptimal]",
            "value": 39.49022268979967,
            "unit": "iter/sec",
            "range": "stddev: 0.00014413843506014945",
            "extra": "mean: 25.322723750005594 msec\nrounds: 40"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[franka]",
            "value": 2289.8320128686732,
            "unit": "iter/sec",
            "range": "stddev: 0.000008954930368778484",
            "extra": "mean: 436.71325860590633 usec\nrounds: 2324"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[dual]",
            "value": 1515.1991360389827,
            "unit": "iter/sec",
            "range": "stddev: 0.000012292801635800341",
            "extra": "mean: 659.9792569933675 usec\nrounds: 1537"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[so101]",
            "value": 2.3039489928280203,
            "unit": "iter/sec",
            "range": "stddev: 0.35237579700662014",
            "extra": "mean: 434.03738672727013 msec\nrounds: 11"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[so101]",
            "value": 10.430349565012298,
            "unit": "iter/sec",
            "range": "stddev: 0.053677360507299085",
            "extra": "mean: 95.87406383333624 msec\nrounds: 30"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[kinova]",
            "value": 2.836513186869488,
            "unit": "iter/sec",
            "range": "stddev: 0.23728947929123542",
            "extra": "mean: 352.5455142000055 msec\nrounds: 5"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[kinova]",
            "value": 4.295452119236961,
            "unit": "iter/sec",
            "range": "stddev: 0.3117178761455457",
            "extra": "mean: 232.80436429998872 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[ur5]",
            "value": 0.36761361571544837,
            "unit": "iter/sec",
            "range": "stddev: 5.333203426129904",
            "extra": "mean: 2.7202474479999967 sec\nrounds: 46"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[ur5]",
            "value": 4.518265847854868,
            "unit": "iter/sec",
            "range": "stddev: 0.44809321906875255",
            "extra": "mean: 221.32385160000467 msec\nrounds: 5"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[franka]",
            "value": 10.146636191097473,
            "unit": "iter/sec",
            "range": "stddev: 0.061861361709532624",
            "extra": "mean: 98.55482951851442 msec\nrounds: 27"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[franka]",
            "value": 37.861227071329,
            "unit": "iter/sec",
            "range": "stddev: 0.007015908120956612",
            "extra": "mean: 26.41224485714742 msec\nrounds: 56"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[dual]",
            "value": 0.15726026456059672,
            "unit": "iter/sec",
            "range": "stddev: 5.548792707852376",
            "extra": "mean: 6.358885398000029 sec\nrounds: 5"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[dual]",
            "value": 16.076060780135258,
            "unit": "iter/sec",
            "range": "stddev: 0.014717242614637317",
            "extra": "mean: 62.20429330770335 msec\nrounds: 26"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[tiago_pro]",
            "value": 0.06341194213598926,
            "unit": "iter/sec",
            "range": "stddev: 9.73929209842277",
            "extra": "mean: 15.769900216200018 sec\nrounds: 5"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[tiago_pro]",
            "value": 5.537008334438548,
            "unit": "iter/sec",
            "range": "stddev: 0.04934423183553806",
            "extra": "mean: 180.6029428889057 msec\nrounds: 9"
          }
        ]
      }
    ]
  }
}