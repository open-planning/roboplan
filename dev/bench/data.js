window.BENCHMARK_DATA = {
  "lastUpdate": 1793400223048,
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
      },
      {
        "commit": {
          "author": {
            "email": "7966037+eholum@users.noreply.github.com",
            "name": "Erik Holum",
            "username": "eholum"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "3ca0fdb2a121b166d8a2cb90d0e4611e33a41edf",
          "message": "Add benchmarks and actually check CI (#357)",
          "timestamp": "2026-10-11T14:40:42-04:00",
          "tree_id": "302dffaec4b81934957d67abff098464700cb44d",
          "url": "https://github.com/open-planning/roboplan/commit/3ca0fdb2a121b166d8a2cb90d0e4611e33a41edf"
        },
        "date": 1791745044165,
        "tool": "pytest",
        "benches": [
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.Bounded]",
            "value": 459.3591231832644,
            "unit": "iter/sec",
            "range": "stddev: 0.00019571201606313955",
            "extra": "mean: 2.1769459874230983 msec\nrounds: 477"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.TimeOptimal]",
            "value": 840.487977434258,
            "unit": "iter/sec",
            "range": "stddev: 0.00005226451101255634",
            "extra": "mean: 1.1897850140018438 msec\nrounds: 857"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[so101]",
            "value": 4724.128029991241,
            "unit": "iter/sec",
            "range": "stddev: 0.000017136415294578403",
            "extra": "mean: 211.6792757629505 usec\nrounds: 4881"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-no_obstacles]",
            "value": 84.22780593749323,
            "unit": "iter/sec",
            "range": "stddev: 0.008831277535523669",
            "extra": "mean: 11.872563803242311 msec\nrounds: 432"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-no_obstacles]",
            "value": 470.53067902178867,
            "unit": "iter/sec",
            "range": "stddev: 0.00008949173988982897",
            "extra": "mean: 2.1252599343340446 msec\nrounds: 533"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.Bounded]",
            "value": 291.3403622621772,
            "unit": "iter/sec",
            "range": "stddev: 0.00002509222415857987",
            "extra": "mean: 3.432411466215244 msec\nrounds: 296"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.TimeOptimal]",
            "value": 510.8202808172579,
            "unit": "iter/sec",
            "range": "stddev: 0.00002471310294282884",
            "extra": "mean: 1.9576356647392832 msec\nrounds: 519"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[kinova]",
            "value": 1872.6968106747283,
            "unit": "iter/sec",
            "range": "stddev: 0.000016700069330480697",
            "extra": "mean: 533.9892684709076 usec\nrounds: 1922"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-with_obstacles]",
            "value": 503.67196449330373,
            "unit": "iter/sec",
            "range": "stddev: 0.00010671071183341193",
            "extra": "mean: 1.985419222223346 msec\nrounds: 585"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-with_obstacles]",
            "value": 106.26629190500128,
            "unit": "iter/sec",
            "range": "stddev: 0.007479978218434312",
            "extra": "mean: 9.410321768769052 msec\nrounds: 333"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.Bounded]",
            "value": 209.4641015289486,
            "unit": "iter/sec",
            "range": "stddev: 0.0001194565749527266",
            "extra": "mean: 4.774087744394697 msec\nrounds: 223"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.TimeOptimal]",
            "value": 376.03737997047284,
            "unit": "iter/sec",
            "range": "stddev: 0.0000490940492803008",
            "extra": "mean: 2.6593100932639246 msec\nrounds: 386"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[ur5]",
            "value": 2694.5902754263957,
            "unit": "iter/sec",
            "range": "stddev: 0.000008528533283953547",
            "extra": "mean: 371.11393487893395 usec\nrounds: 2718"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.Bounded]",
            "value": 22.79423000082193,
            "unit": "iter/sec",
            "range": "stddev: 0.00025827316141688875",
            "extra": "mean: 43.870751500004225 msec\nrounds: 24"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.TimeOptimal]",
            "value": 38.347213467778204,
            "unit": "iter/sec",
            "range": "stddev: 0.00011314776288295771",
            "extra": "mean: 26.077514102563526 msec\nrounds: 39"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[franka]",
            "value": 2252.6831263697895,
            "unit": "iter/sec",
            "range": "stddev: 0.000009707257139061466",
            "extra": "mean: 443.9150754467208 usec\nrounds: 2293"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[dual]",
            "value": 1481.3217309716833,
            "unit": "iter/sec",
            "range": "stddev: 0.000011005806435306946",
            "extra": "mean: 675.0727941755388 usec\nrounds: 1511"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[so101]",
            "value": 2.861838021663376,
            "unit": "iter/sec",
            "range": "stddev: 0.23204359446219716",
            "extra": "mean: 349.4257859565278 msec\nrounds: 23"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[so101]",
            "value": 9.56588979641406,
            "unit": "iter/sec",
            "range": "stddev: 0.06127705557678178",
            "extra": "mean: 104.53810584090851 msec\nrounds: 88"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[kinova]",
            "value": 2.436744300074576,
            "unit": "iter/sec",
            "range": "stddev: 0.20782981914308532",
            "extra": "mean: 410.38364180000144 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[kinova]",
            "value": 5.869973611394079,
            "unit": "iter/sec",
            "range": "stddev: 0.13318994516346128",
            "extra": "mean: 170.35851712500403 msec\nrounds: 40"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[ur5]",
            "value": 0.3346464200765645,
            "unit": "iter/sec",
            "range": "stddev: 4.62667962363113",
            "extra": "mean: 2.9882285899583443 sec\nrounds: 24"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[ur5]",
            "value": 3.8858946179723626,
            "unit": "iter/sec",
            "range": "stddev: 0.4258042626573952",
            "extra": "mean: 257.34099822856086 msec\nrounds: 35"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[franka]",
            "value": 10.863422825580104,
            "unit": "iter/sec",
            "range": "stddev: 0.04190095662204475",
            "extra": "mean: 92.0520186000033 msec\nrounds: 20"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[franka]",
            "value": 33.87040533427335,
            "unit": "iter/sec",
            "range": "stddev: 0.008270664887962388",
            "extra": "mean: 29.524299757585226 msec\nrounds: 66"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[dual]",
            "value": 0.10908747947179147,
            "unit": "iter/sec",
            "range": "stddev: 7.759914634940738",
            "extra": "mean: 9.166954858999986 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[dual]",
            "value": 14.407495735936392,
            "unit": "iter/sec",
            "range": "stddev: 0.030780333401416556",
            "extra": "mean: 69.40831483334857 msec\nrounds: 24"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[tiago_pro]",
            "value": 0.13982359861314686,
            "unit": "iter/sec",
            "range": "stddev: 8.762913609775273",
            "extra": "mean: 7.1518685681000305 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[tiago_pro]",
            "value": 4.526365051430091,
            "unit": "iter/sec",
            "range": "stddev: 0.08620364820565403",
            "extra": "mean: 220.92782809995697 msec\nrounds: 10"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "4603398+sea-bass@users.noreply.github.com",
            "name": "Sebastian Castro",
            "username": "sea-bass"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d3201c12dcbb75510b8e8c9f6a17f95f9eccd40f",
          "message": "Switch to nanoflann for RRT K-D trees (#376)",
          "timestamp": "2026-10-14T10:05:32-04:00",
          "tree_id": "b8f23bae0237e3be8582a6ad4d01d9de39b1f1aa",
          "url": "https://github.com/open-planning/roboplan/commit/d3201c12dcbb75510b8e8c9f6a17f95f9eccd40f"
        },
        "date": 1791987744085,
        "tool": "pytest",
        "benches": [
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.Bounded]",
            "value": 363.98298443829407,
            "unit": "iter/sec",
            "range": "stddev: 0.00019890929535510196",
            "extra": "mean: 2.7473811764668623 msec\nrounds: 374"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.TimeOptimal]",
            "value": 654.1860538595031,
            "unit": "iter/sec",
            "range": "stddev: 0.000028165815064855895",
            "extra": "mean: 1.5286171175620413 msec\nrounds: 672"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[so101]",
            "value": 3740.1995106094882,
            "unit": "iter/sec",
            "range": "stddev: 0.000009707825123441527",
            "extra": "mean: 267.36541651411636 usec\nrounds: 3815"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-no_obstacles]",
            "value": 65.59562945136068,
            "unit": "iter/sec",
            "range": "stddev: 0.011250156328430674",
            "extra": "mean: 15.244918119758303 msec\nrounds: 334"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-no_obstacles]",
            "value": 368.46753888946404,
            "unit": "iter/sec",
            "range": "stddev: 0.00012212721467003317",
            "extra": "mean: 2.713943277103681 msec\nrounds: 415"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.Bounded]",
            "value": 224.40289970026095,
            "unit": "iter/sec",
            "range": "stddev: 0.000047788009177112626",
            "extra": "mean: 4.456270401744889 msec\nrounds: 229"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.TimeOptimal]",
            "value": 397.9921481331272,
            "unit": "iter/sec",
            "range": "stddev: 0.000024782594139540088",
            "extra": "mean: 2.512612383663164 msec\nrounds: 404"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[kinova]",
            "value": 1505.4932810354182,
            "unit": "iter/sec",
            "range": "stddev: 0.000020788183409843985",
            "extra": "mean: 664.2341168817704 usec\nrounds: 1540"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-with_obstacles]",
            "value": 397.98380846479284,
            "unit": "iter/sec",
            "range": "stddev: 0.00012763996936912143",
            "extra": "mean: 2.5126650349356203 msec\nrounds: 458"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-with_obstacles]",
            "value": 88.05377531225221,
            "unit": "iter/sec",
            "range": "stddev: 0.008833195957525092",
            "extra": "mean: 11.356696478417268 msec\nrounds: 278"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.Bounded]",
            "value": 164.29936749367485,
            "unit": "iter/sec",
            "range": "stddev: 0.00008395242274966219",
            "extra": "mean: 6.086450698226199 msec\nrounds: 169"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.TimeOptimal]",
            "value": 295.55306100509785,
            "unit": "iter/sec",
            "range": "stddev: 0.00005956379822963712",
            "extra": "mean: 3.3834872039533757 msec\nrounds: 304"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[ur5]",
            "value": 2140.9508013677982,
            "unit": "iter/sec",
            "range": "stddev: 0.000011685280511823536",
            "extra": "mean: 467.0821951448515 usec\nrounds: 2183"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.Bounded]",
            "value": 17.744639448915112,
            "unit": "iter/sec",
            "range": "stddev: 0.00024917064579866265",
            "extra": "mean: 56.35504755556693 msec\nrounds: 18"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.TimeOptimal]",
            "value": 29.755806858355815,
            "unit": "iter/sec",
            "range": "stddev: 0.00014963638439050405",
            "extra": "mean: 33.60688569999866 msec\nrounds: 30"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[franka]",
            "value": 1774.15668091115,
            "unit": "iter/sec",
            "range": "stddev: 0.000023049236406360754",
            "extra": "mean: 563.6480761588836 usec\nrounds: 1812"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[dual]",
            "value": 1178.5010403847093,
            "unit": "iter/sec",
            "range": "stddev: 0.000015361753072798298",
            "extra": "mean: 848.5355258350561 usec\nrounds: 1200"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[so101]",
            "value": 1.8895203215887864,
            "unit": "iter/sec",
            "range": "stddev: 0.2920372577961875",
            "extra": "mean: 529.2348478999997 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[so101]",
            "value": 7.047368137273195,
            "unit": "iter/sec",
            "range": "stddev: 0.08164527454376812",
            "extra": "mean: 141.89694372726854 msec\nrounds: 44"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[kinova]",
            "value": 1.2679961581532637,
            "unit": "iter/sec",
            "range": "stddev: 0.7640369678913431",
            "extra": "mean: 788.6459225999715 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[kinova]",
            "value": 4.56800752694066,
            "unit": "iter/sec",
            "range": "stddev: 0.11748374360863699",
            "extra": "mean: 218.91382492308017 msec\nrounds: 13"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[ur5]",
            "value": 0.24254822968652184,
            "unit": "iter/sec",
            "range": "stddev: 5.1621859692168774",
            "extra": "mean: 4.122891357700018 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[ur5]",
            "value": 4.30930634667187,
            "unit": "iter/sec",
            "range": "stddev: 0.39863080671360246",
            "extra": "mean: 232.0559086666726 msec\nrounds: 36"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[franka]",
            "value": 8.729110170045793,
            "unit": "iter/sec",
            "range": "stddev: 0.07216038653156998",
            "extra": "mean: 114.55921400001691 msec\nrounds: 22"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[franka]",
            "value": 26.950832956750585,
            "unit": "iter/sec",
            "range": "stddev: 0.01215696665498648",
            "extra": "mean: 37.10460458141507 msec\nrounds: 43"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[dual]",
            "value": 0.1385667043307481,
            "unit": "iter/sec",
            "range": "stddev: 8.279310130899741",
            "extra": "mean: 7.2167408817999785 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[dual]",
            "value": 12.012634967856727,
            "unit": "iter/sec",
            "range": "stddev: 0.02095928748751749",
            "extra": "mean: 83.24568278947864 msec\nrounds: 19"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[tiago_pro]",
            "value": 0.3732164752401204,
            "unit": "iter/sec",
            "range": "stddev: 4.227604899123313",
            "extra": "mean: 2.6794101181000087 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[tiago_pro]",
            "value": 3.8706754804654144,
            "unit": "iter/sec",
            "range": "stddev: 0.055410261903263015",
            "extra": "mean: 258.35283919998346 msec\nrounds: 10"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "4603398+sea-bass@users.noreply.github.com",
            "name": "Sebastian Castro",
            "username": "sea-bass"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "3546dd1f0f6e51dcd4d46ff452a7fdb6ae3873f2",
          "message": "Change OInK to solve cascade of QPs + add Relative pose constraint (#363)",
          "timestamp": "2026-10-14T14:24:06-04:00",
          "tree_id": "8069a6e30bc375f0001cc43fce1a0db22cb5dbc3",
          "url": "https://github.com/open-planning/roboplan/commit/3546dd1f0f6e51dcd4d46ff452a7fdb6ae3873f2"
        },
        "date": 1792002972511,
        "tool": "pytest",
        "benches": [
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.Bounded]",
            "value": 470.94791242639116,
            "unit": "iter/sec",
            "range": "stddev: 0.000018175282207558704",
            "extra": "mean: 2.1233770733749653 msec\nrounds: 477"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.TimeOptimal]",
            "value": 831.633341082093,
            "unit": "iter/sec",
            "range": "stddev: 0.00001959049300048829",
            "extra": "mean: 1.202452992924543 msec\nrounds: 848"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[so101]",
            "value": 4849.266187590551,
            "unit": "iter/sec",
            "range": "stddev: 0.000006105360355072401",
            "extra": "mean: 206.2167679223377 usec\nrounds: 4938"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-no_obstacles]",
            "value": 83.95690970167847,
            "unit": "iter/sec",
            "range": "stddev: 0.008816826494744183",
            "extra": "mean: 11.910871940776161 msec\nrounds: 439"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-no_obstacles]",
            "value": 475.07493642379376,
            "unit": "iter/sec",
            "range": "stddev: 0.00009125868269569677",
            "extra": "mean: 2.1049310820892124 msec\nrounds: 536"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.Bounded]",
            "value": 271.81602347562773,
            "unit": "iter/sec",
            "range": "stddev: 0.000030045919407659077",
            "extra": "mean: 3.678958978257824 msec\nrounds: 276"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.TimeOptimal]",
            "value": 456.14859653525303,
            "unit": "iter/sec",
            "range": "stddev: 0.000029983791142811406",
            "extra": "mean: 2.192268062634971 msec\nrounds: 463"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[kinova]",
            "value": 1933.4413521323027,
            "unit": "iter/sec",
            "range": "stddev: 0.00005137197369284953",
            "extra": "mean: 517.2124817218513 usec\nrounds: 1997"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-with_obstacles]",
            "value": 512.1246521715716,
            "unit": "iter/sec",
            "range": "stddev: 0.00010277515498052596",
            "extra": "mean: 1.9526496054421159 msec\nrounds: 588"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-with_obstacles]",
            "value": 112.61554497995019,
            "unit": "iter/sec",
            "range": "stddev: 0.007044303150145722",
            "extra": "mean: 8.879768775953957 msec\nrounds: 366"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.Bounded]",
            "value": 183.08811373254426,
            "unit": "iter/sec",
            "range": "stddev: 0.0001333180240276966",
            "extra": "mean: 5.461851015958379 msec\nrounds: 188"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.TimeOptimal]",
            "value": 319.9817168513809,
            "unit": "iter/sec",
            "range": "stddev: 0.00003845791122295374",
            "extra": "mean: 3.1251785565750345 msec\nrounds: 327"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[ur5]",
            "value": 2780.5765835654015,
            "unit": "iter/sec",
            "range": "stddev: 0.000006504542521345972",
            "extra": "mean: 359.63763987314724 usec\nrounds: 2824"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.Bounded]",
            "value": 28.73164838224113,
            "unit": "iter/sec",
            "range": "stddev: 0.0020574136507165042",
            "extra": "mean: 34.80482521212025 msec\nrounds: 33"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.TimeOptimal]",
            "value": 51.3004245879717,
            "unit": "iter/sec",
            "range": "stddev: 0.0017391299825335977",
            "extra": "mean: 19.493016052628693 msec\nrounds: 57"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[franka]",
            "value": 2295.7025810499554,
            "unit": "iter/sec",
            "range": "stddev: 0.00000855709766475309",
            "extra": "mean: 435.59649592877275 usec\nrounds: 2333"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[dual]",
            "value": 1516.58530147374,
            "unit": "iter/sec",
            "range": "stddev: 0.000010994255566610021",
            "extra": "mean: 659.37603313724 usec\nrounds: 1539"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[so101]",
            "value": 2.979894355390729,
            "unit": "iter/sec",
            "range": "stddev: 0.23867114347541565",
            "extra": "mean: 335.5823665999992 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[so101]",
            "value": 8.648378314049776,
            "unit": "iter/sec",
            "range": "stddev: 0.04989998981349673",
            "extra": "mean: 115.62861425424046 msec\nrounds: 59"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[kinova]",
            "value": 2.264060230448308,
            "unit": "iter/sec",
            "range": "stddev: 0.43276022937142483",
            "extra": "mean: 441.6843627000105 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[kinova]",
            "value": 6.845591462937389,
            "unit": "iter/sec",
            "range": "stddev: 0.11602172251474356",
            "extra": "mean: 146.07941555000536 msec\nrounds: 20"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[ur5]",
            "value": 0.4228644404370382,
            "unit": "iter/sec",
            "range": "stddev: 5.2844982601961314",
            "extra": "mean: 2.364824053227274 sec\nrounds: 22"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[ur5]",
            "value": 2.2799754006583877,
            "unit": "iter/sec",
            "range": "stddev: 0.5772992870620315",
            "extra": "mean: 438.6012233777744 msec\nrounds: 90"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[franka]",
            "value": 10.038923810170541,
            "unit": "iter/sec",
            "range": "stddev: 0.04625097394404062",
            "extra": "mean: 99.61227108695549 msec\nrounds: 23"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[franka]",
            "value": 35.387243197628024,
            "unit": "iter/sec",
            "range": "stddev: 0.008007490151549725",
            "extra": "mean: 28.258770947916883 msec\nrounds: 96"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[dual]",
            "value": 0.23620707638810376,
            "unit": "iter/sec",
            "range": "stddev: 4.78700490685278",
            "extra": "mean: 4.2335734191000025 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[dual]",
            "value": 17.131653914422895,
            "unit": "iter/sec",
            "range": "stddev: 0.01706371271825094",
            "extra": "mean: 58.371480359996895 msec\nrounds: 25"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[tiago_pro]",
            "value": 0.3429793193108887,
            "unit": "iter/sec",
            "range": "stddev: 4.364596518137476",
            "extra": "mean: 2.9156276885999772 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[tiago_pro]",
            "value": 4.187385018987343,
            "unit": "iter/sec",
            "range": "stddev: 0.1365407728441218",
            "extra": "mean: 238.8125274999993 msec\nrounds: 10"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "4603398+sea-bass@users.noreply.github.com",
            "name": "Sebastian Castro",
            "username": "sea-bass"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "b1f4442b20a5037e959db57007722356b8307145",
          "message": "Iterative interface for OInK (#371)",
          "timestamp": "2026-10-14T22:13:21-04:00",
          "tree_id": "041412f795697c7edfcc3f870668eeb38b6d8ac8",
          "url": "https://github.com/open-planning/roboplan/commit/b1f4442b20a5037e959db57007722356b8307145"
        },
        "date": 1792031198308,
        "tool": "pytest",
        "benches": [
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.Bounded]",
            "value": 193.66622745010724,
            "unit": "iter/sec",
            "range": "stddev: 0.00006346951667716858",
            "extra": "mean: 5.163522897959183 msec\nrounds: 196"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.TimeOptimal]",
            "value": 232.01110613377045,
            "unit": "iter/sec",
            "range": "stddev: 0.00009133596261295548",
            "extra": "mean: 4.310138495798692 msec\nrounds: 238"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[so101]",
            "value": 4918.473333669467,
            "unit": "iter/sec",
            "range": "stddev: 0.000006836728389358372",
            "extra": "mean: 203.3151208027272 usec\nrounds: 5033"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-no_obstacles]",
            "value": 90.83352791333459,
            "unit": "iter/sec",
            "range": "stddev: 0.007986471726548358",
            "extra": "mean: 11.009150728507567 msec\nrounds: 442"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-no_obstacles]",
            "value": 497.9125583010057,
            "unit": "iter/sec",
            "range": "stddev: 0.00009146150925875458",
            "extra": "mean: 2.008384772242408 msec\nrounds: 562"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.Bounded]",
            "value": 111.97385418680234,
            "unit": "iter/sec",
            "range": "stddev: 0.00014996612757076062",
            "extra": "mean: 8.930656243481025 msec\nrounds: 115"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.TimeOptimal]",
            "value": 133.33857942203826,
            "unit": "iter/sec",
            "range": "stddev: 0.0001363933809575142",
            "extra": "mean: 7.499704919120501 msec\nrounds: 136"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[kinova]",
            "value": 2009.4132448710761,
            "unit": "iter/sec",
            "range": "stddev: 0.000014346037956973464",
            "extra": "mean: 497.65771304257527 usec\nrounds: 2070"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-with_obstacles]",
            "value": 534.2045469474598,
            "unit": "iter/sec",
            "range": "stddev: 0.00009723879564299315",
            "extra": "mean: 1.8719421347387974 msec\nrounds: 616"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-with_obstacles]",
            "value": 122.92620249740789,
            "unit": "iter/sec",
            "range": "stddev: 0.006297470173724979",
            "extra": "mean: 8.13496211290743 msec\nrounds: 372"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.Bounded]",
            "value": 78.16368959397289,
            "unit": "iter/sec",
            "range": "stddev: 0.00021559180496272574",
            "extra": "mean: 12.793664234564341 msec\nrounds: 81"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.TimeOptimal]",
            "value": 91.74510298751356,
            "unit": "iter/sec",
            "range": "stddev: 0.00018594823777221865",
            "extra": "mean: 10.899764319149538 msec\nrounds: 94"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[ur5]",
            "value": 2851.339751186717,
            "unit": "iter/sec",
            "range": "stddev: 0.000009980155976231413",
            "extra": "mean: 350.71232727836235 usec\nrounds: 2918"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.Bounded]",
            "value": 12.763047501222282,
            "unit": "iter/sec",
            "range": "stddev: 0.0006629465805182734",
            "extra": "mean: 78.35119315384767 msec\nrounds: 13"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.TimeOptimal]",
            "value": 16.166098268554165,
            "unit": "iter/sec",
            "range": "stddev: 0.0005432112344334086",
            "extra": "mean: 61.85784494117369 msec\nrounds: 17"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[franka]",
            "value": 2374.7268023947977,
            "unit": "iter/sec",
            "range": "stddev: 0.000011364762260066517",
            "extra": "mean: 421.1010710754383 usec\nrounds: 2434"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[dual]",
            "value": 1527.9841607626672,
            "unit": "iter/sec",
            "range": "stddev: 0.00008018149470553291",
            "extra": "mean: 654.4570458772735 usec\nrounds: 1613"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[so101]",
            "value": 4.221729163846242,
            "unit": "iter/sec",
            "range": "stddev: 0.15938408104883026",
            "extra": "mean: 236.8697662000045 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[so101]",
            "value": 9.395594050453635,
            "unit": "iter/sec",
            "range": "stddev: 0.07927844006339174",
            "extra": "mean: 106.43286572728398 msec\nrounds: 11"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[kinova]",
            "value": 2.9266960290998307,
            "unit": "iter/sec",
            "range": "stddev: 0.18548569451765684",
            "extra": "mean: 341.6822211999829 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[kinova]",
            "value": 6.727000112854119,
            "unit": "iter/sec",
            "range": "stddev: 0.07245530007014948",
            "extra": "mean: 148.65467269566045 msec\nrounds: 23"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[ur5]",
            "value": 0.40388347714575995,
            "unit": "iter/sec",
            "range": "stddev: 4.244964227681218",
            "extra": "mean: 2.4759616488076928 sec\nrounds: 26"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[ur5]",
            "value": 6.3512022034094135,
            "unit": "iter/sec",
            "range": "stddev: 0.31604352948547154",
            "extra": "mean: 157.45050589999892 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[franka]",
            "value": 9.472398920658225,
            "unit": "iter/sec",
            "range": "stddev: 0.044161294842676634",
            "extra": "mean: 105.56987816667156 msec\nrounds: 24"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[franka]",
            "value": 36.9642640686636,
            "unit": "iter/sec",
            "range": "stddev: 0.005946374448525706",
            "extra": "mean: 27.053155938460804 msec\nrounds: 65"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[dual]",
            "value": 0.46230130252801427,
            "unit": "iter/sec",
            "range": "stddev: 4.1873806699899765",
            "extra": "mean: 2.163091461200031 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[dual]",
            "value": 17.52911919751357,
            "unit": "iter/sec",
            "range": "stddev: 0.013292740673873926",
            "extra": "mean: 57.04793200002003 msec\nrounds: 22"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[tiago_pro]",
            "value": 0.18016675779567953,
            "unit": "iter/sec",
            "range": "stddev: 5.3883476691538785",
            "extra": "mean: 5.550413473799995 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[tiago_pro]",
            "value": 5.381209720882038,
            "unit": "iter/sec",
            "range": "stddev: 0.08304683108243566",
            "extra": "mean: 185.8318206999911 msec\nrounds: 10"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "andrew.hoelscher@pal-robotics.com",
            "name": "Andrew Hoelscher",
            "username": "andrewhoelscher"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e8cc55fadb4514b66c2b49255f09d786471202be",
          "message": "Bind refreshContext; remove scene argument (#379)",
          "timestamp": "2026-10-15T20:33:09-04:00",
          "tree_id": "e49e53c717175ff23510077a240a8dfce7b3e821",
          "url": "https://github.com/open-planning/roboplan/commit/e8cc55fadb4514b66c2b49255f09d786471202be"
        },
        "date": 1792111703961,
        "tool": "pytest",
        "benches": [
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.Bounded]",
            "value": 184.12568844851347,
            "unit": "iter/sec",
            "range": "stddev: 0.00004614056403206554",
            "extra": "mean: 5.431072700535358 msec\nrounds: 187"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.TimeOptimal]",
            "value": 222.46026965127416,
            "unit": "iter/sec",
            "range": "stddev: 0.000030025389150791166",
            "extra": "mean: 4.495184697778112 msec\nrounds: 225"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[so101]",
            "value": 4899.769973329534,
            "unit": "iter/sec",
            "range": "stddev: 0.000004928696768266133",
            "extra": "mean: 204.09121355557664 usec\nrounds: 4987"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-no_obstacles]",
            "value": 82.89827829055871,
            "unit": "iter/sec",
            "range": "stddev: 0.008878629733397756",
            "extra": "mean: 12.062976706162667 msec\nrounds: 422"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-no_obstacles]",
            "value": 474.8488275150318,
            "unit": "iter/sec",
            "range": "stddev: 0.00009577776632976446",
            "extra": "mean: 2.1059333877545354 msec\nrounds: 539"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.Bounded]",
            "value": 107.68444733007266,
            "unit": "iter/sec",
            "range": "stddev: 0.00010660781991534476",
            "extra": "mean: 9.286392090909988 msec\nrounds: 110"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.TimeOptimal]",
            "value": 128.36948063861865,
            "unit": "iter/sec",
            "range": "stddev: 0.00028226151181216086",
            "extra": "mean: 7.790013600001744 msec\nrounds: 130"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[kinova]",
            "value": 1978.1205050922645,
            "unit": "iter/sec",
            "range": "stddev: 0.000010934894471548567",
            "extra": "mean: 505.53037462869713 usec\nrounds: 2018"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-with_obstacles]",
            "value": 512.7701706698529,
            "unit": "iter/sec",
            "range": "stddev: 0.0001011241329341764",
            "extra": "mean: 1.9501914448214857 msec\nrounds: 589"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-with_obstacles]",
            "value": 111.41814539738913,
            "unit": "iter/sec",
            "range": "stddev: 0.007010453115844669",
            "extra": "mean: 8.975198756300902 msec\nrounds: 357"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.Bounded]",
            "value": 76.22814930343175,
            "unit": "iter/sec",
            "range": "stddev: 0.00021811275616793926",
            "extra": "mean: 13.118513425000344 msec\nrounds: 80"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.TimeOptimal]",
            "value": 89.35454671123179,
            "unit": "iter/sec",
            "range": "stddev: 0.00008604976318285604",
            "extra": "mean: 11.191372311827763 msec\nrounds: 93"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[ur5]",
            "value": 2662.7261482918343,
            "unit": "iter/sec",
            "range": "stddev: 0.000035580163907171664",
            "extra": "mean: 375.5549554510178 usec\nrounds: 2761"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.Bounded]",
            "value": 12.190859793339818,
            "unit": "iter/sec",
            "range": "stddev: 0.0008654441575675209",
            "extra": "mean: 82.02866876923036 msec\nrounds: 13"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.TimeOptimal]",
            "value": 15.600963151202054,
            "unit": "iter/sec",
            "range": "stddev: 0.00045020367056798085",
            "extra": "mean: 64.09860662499867 msec\nrounds: 16"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[franka]",
            "value": 2275.565476571794,
            "unit": "iter/sec",
            "range": "stddev: 0.000007561146969537279",
            "extra": "mean: 439.4512090711313 usec\nrounds: 2315"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[dual]",
            "value": 1474.7648757956458,
            "unit": "iter/sec",
            "range": "stddev: 0.000011538681983389936",
            "extra": "mean: 678.0741909522987 usec\nrounds: 1503"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[so101]",
            "value": 3.0934455646933237,
            "unit": "iter/sec",
            "range": "stddev: 0.3200436362001047",
            "extra": "mean: 323.26413349999825 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[so101]",
            "value": 8.463740350907194,
            "unit": "iter/sec",
            "range": "stddev: 0.0656480479953184",
            "extra": "mean: 118.15107252112405 msec\nrounds: 71"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[kinova]",
            "value": 2.9898791037600967,
            "unit": "iter/sec",
            "range": "stddev: 0.12917028395947563",
            "extra": "mean: 334.4616839999958 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[kinova]",
            "value": 6.454908008367682,
            "unit": "iter/sec",
            "range": "stddev: 0.10091143176095475",
            "extra": "mean: 154.9208755111105 msec\nrounds: 45"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[ur5]",
            "value": 0.2605940478944434,
            "unit": "iter/sec",
            "range": "stddev: 7.116490054638767",
            "extra": "mean: 3.837386187750004 sec\nrounds: 24"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[ur5]",
            "value": 4.066917481266256,
            "unit": "iter/sec",
            "range": "stddev: 0.4161621346016221",
            "extra": "mean: 245.8864741186351 msec\nrounds: 59"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[franka]",
            "value": 9.253818683276076,
            "unit": "iter/sec",
            "range": "stddev: 0.0542979190365643",
            "extra": "mean: 108.06349618749778 msec\nrounds: 32"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[franka]",
            "value": 36.32159883921393,
            "unit": "iter/sec",
            "range": "stddev: 0.007792251427378636",
            "extra": "mean: 27.531827671648884 msec\nrounds: 67"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[dual]",
            "value": 0.19021364220534795,
            "unit": "iter/sec",
            "range": "stddev: 6.024263963052588",
            "extra": "mean: 5.257246475100009 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[dual]",
            "value": 16.659438991278368,
            "unit": "iter/sec",
            "range": "stddev: 0.013973059863965598",
            "extra": "mean: 60.02603091998026 msec\nrounds: 25"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[tiago_pro]",
            "value": 0.2007768876148417,
            "unit": "iter/sec",
            "range": "stddev: 5.002261302796409",
            "extra": "mean: 4.980652961999988 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[tiago_pro]",
            "value": 4.323032299426911,
            "unit": "iter/sec",
            "range": "stddev: 0.07934799865174234",
            "extra": "mean: 231.31911370002172 msec\nrounds: 10"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "andrew.hoelscher@pal-robotics.com",
            "name": "Andrew Hoelscher",
            "username": "andrewhoelscher"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "8a84cd2636fa3d5f1ffc7001e933163111fc32e2",
          "message": "Add joint limits + pullback to PositionLimit (#378)",
          "timestamp": "2026-10-16T07:50:34-04:00",
          "tree_id": "5abd1290a69a9ed33bae72dcaad86fefe9e46da5",
          "url": "https://github.com/open-planning/roboplan/commit/8a84cd2636fa3d5f1ffc7001e933163111fc32e2"
        },
        "date": 1792152540359,
        "tool": "pytest",
        "benches": [
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.Bounded]",
            "value": 139.84141298348501,
            "unit": "iter/sec",
            "range": "stddev: 0.00004586150869870748",
            "extra": "mean: 7.1509574929573825 msec\nrounds: 142"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.TimeOptimal]",
            "value": 169.24341615254204,
            "unit": "iter/sec",
            "range": "stddev: 0.00018052907762199959",
            "extra": "mean: 5.908649345027889 msec\nrounds: 171"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[so101]",
            "value": 3782.1345629041734,
            "unit": "iter/sec",
            "range": "stddev: 0.000009644747192139",
            "extra": "mean: 264.40095754608313 usec\nrounds: 3863"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-no_obstacles]",
            "value": 65.53735379735303,
            "unit": "iter/sec",
            "range": "stddev: 0.011291668063656436",
            "extra": "mean: 15.258473863502077 msec\nrounds: 337"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-no_obstacles]",
            "value": 369.7678290082108,
            "unit": "iter/sec",
            "range": "stddev: 0.00011926972606062263",
            "extra": "mean: 2.7043996842077753 msec\nrounds: 418"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.Bounded]",
            "value": 83.57735150450752,
            "unit": "iter/sec",
            "range": "stddev: 0.00011166561903056784",
            "extra": "mean: 11.964963976467569 msec\nrounds: 85"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.TimeOptimal]",
            "value": 99.11323691137187,
            "unit": "iter/sec",
            "range": "stddev: 0.00005568229827271515",
            "extra": "mean: 10.089469693076524 msec\nrounds: 101"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[kinova]",
            "value": 1531.9365433164262,
            "unit": "iter/sec",
            "range": "stddev: 0.000011109560649784075",
            "extra": "mean: 652.7685525636338 usec\nrounds: 1560"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-with_obstacles]",
            "value": 400.20164838612266,
            "unit": "iter/sec",
            "range": "stddev: 0.00012506536506402463",
            "extra": "mean: 2.498740332611473 msec\nrounds: 460"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-with_obstacles]",
            "value": 87.43863615575638,
            "unit": "iter/sec",
            "range": "stddev: 0.009046210352252629",
            "extra": "mean: 11.436591922804901 msec\nrounds: 285"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.Bounded]",
            "value": 52.09847907674673,
            "unit": "iter/sec",
            "range": "stddev: 0.0001337824624577852",
            "extra": "mean: 19.194418296297886 msec\nrounds: 54"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.TimeOptimal]",
            "value": 62.11034098157249,
            "unit": "iter/sec",
            "range": "stddev: 0.00012976493553335945",
            "extra": "mean: 16.100378523065746 msec\nrounds: 65"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[ur5]",
            "value": 2122.362740643071,
            "unit": "iter/sec",
            "range": "stddev: 0.000010325359921845837",
            "extra": "mean: 471.17299076641456 usec\nrounds: 2166"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.Bounded]",
            "value": 8.979728549250924,
            "unit": "iter/sec",
            "range": "stddev: 0.0011987674840441072",
            "extra": "mean: 111.36194090003073 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.TimeOptimal]",
            "value": 11.064320825711428,
            "unit": "iter/sec",
            "range": "stddev: 0.002319824500734376",
            "extra": "mean: 90.38060408336908 msec\nrounds: 12"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[franka]",
            "value": 1789.073082558167,
            "unit": "iter/sec",
            "range": "stddev: 0.000012548601974597684",
            "extra": "mean: 558.9486588049919 usec\nrounds: 1823"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[dual]",
            "value": 1185.3576825370824,
            "unit": "iter/sec",
            "range": "stddev: 0.00001200113087966611",
            "extra": "mean: 843.6272145802002 usec\nrounds: 1207"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[so101]",
            "value": 2.209161390563515,
            "unit": "iter/sec",
            "range": "stddev: 0.3070158631233963",
            "extra": "mean: 452.66045490000124 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[so101]",
            "value": 6.710414272347055,
            "unit": "iter/sec",
            "range": "stddev: 0.0746170065445738",
            "extra": "mean: 149.02209601587487 msec\nrounds: 63"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[kinova]",
            "value": 1.8337377600092024,
            "unit": "iter/sec",
            "range": "stddev: 0.5121861340507758",
            "extra": "mean: 545.3342467000198 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[kinova]",
            "value": 7.993125815882481,
            "unit": "iter/sec",
            "range": "stddev: 0.042483428059299465",
            "extra": "mean: 125.1075014999742 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[ur5]",
            "value": 0.3250477891716036,
            "unit": "iter/sec",
            "range": "stddev: 7.3509597343679625",
            "extra": "mean: 3.0764707015806425 sec\nrounds: 31"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[ur5]",
            "value": 3.141701682914406,
            "unit": "iter/sec",
            "range": "stddev: 0.45397713179656257",
            "extra": "mean: 318.29883958694256 msec\nrounds: 46"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[franka]",
            "value": 8.55563659500088,
            "unit": "iter/sec",
            "range": "stddev: 0.06310555117477099",
            "extra": "mean: 116.88200976001099 msec\nrounds: 25"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[franka]",
            "value": 27.777490909055693,
            "unit": "iter/sec",
            "range": "stddev: 0.010209332372252997",
            "extra": "mean: 36.00037178570335 msec\nrounds: 42"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[dual]",
            "value": 0.14192665122130774,
            "unit": "iter/sec",
            "range": "stddev: 7.892946143129369",
            "extra": "mean: 7.045893011599981 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[dual]",
            "value": 11.508575305973478,
            "unit": "iter/sec",
            "range": "stddev: 0.029266687652509088",
            "extra": "mean: 86.89172842106305 msec\nrounds: 19"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[tiago_pro]",
            "value": 0.11555741588604386,
            "unit": "iter/sec",
            "range": "stddev: 10.631215364966051",
            "extra": "mean: 8.653706837699996 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[tiago_pro]",
            "value": 3.061865907586966,
            "unit": "iter/sec",
            "range": "stddev: 0.16764860522866076",
            "extra": "mean: 326.59823460005555 msec\nrounds: 10"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "ezra@brooks.cx",
            "name": "Ezra Brooks",
            "username": "EzraBrooks"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "e3b5f67e777eee70c5e315e7579b133cab9d6f74",
          "message": "Don't install into CONDA_PREFIX in pixi build (#382)",
          "timestamp": "2026-10-16T16:51:21-04:00",
          "tree_id": "7e4db8fe5c6447f5e07d3d7346285dd78b13ed82",
          "url": "https://github.com/open-planning/roboplan/commit/e3b5f67e777eee70c5e315e7579b133cab9d6f74"
        },
        "date": 1792185421564,
        "tool": "pytest",
        "benches": [
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.Bounded]",
            "value": 180.0099908116949,
            "unit": "iter/sec",
            "range": "stddev: 0.00004220040235230698",
            "extra": "mean: 5.5552472142842415 msec\nrounds: 182"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.TimeOptimal]",
            "value": 216.6344259410212,
            "unit": "iter/sec",
            "range": "stddev: 0.000030429876162066577",
            "extra": "mean: 4.616071502283992 msec\nrounds: 219"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[so101]",
            "value": 4874.87189139798,
            "unit": "iter/sec",
            "range": "stddev: 0.000008143032204707081",
            "extra": "mean: 205.13359576988336 usec\nrounds: 5012"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-no_obstacles]",
            "value": 83.96302998025818,
            "unit": "iter/sec",
            "range": "stddev: 0.008781915183117418",
            "extra": "mean: 11.910003727058504 msec\nrounds: 425"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-no_obstacles]",
            "value": 473.54614510163134,
            "unit": "iter/sec",
            "range": "stddev: 0.00009391640486180226",
            "extra": "mean: 2.111726619135253 msec\nrounds: 533"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.Bounded]",
            "value": 105.02157531344605,
            "unit": "iter/sec",
            "range": "stddev: 0.00007370070439024999",
            "extra": "mean: 9.52185298130801 msec\nrounds: 107"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.TimeOptimal]",
            "value": 125.56673942249964,
            "unit": "iter/sec",
            "range": "stddev: 0.00004003476898437842",
            "extra": "mean: 7.963892385827256 msec\nrounds: 127"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[kinova]",
            "value": 1905.5842071046886,
            "unit": "iter/sec",
            "range": "stddev: 0.000011705229716168251",
            "extra": "mean: 524.7734507200721 usec\nrounds: 1948"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-with_obstacles]",
            "value": 511.9278776080101,
            "unit": "iter/sec",
            "range": "stddev: 0.00010210703586788613",
            "extra": "mean: 1.953400163852208 msec\nrounds: 592"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-with_obstacles]",
            "value": 110.34524865661984,
            "unit": "iter/sec",
            "range": "stddev: 0.0069459619940811955",
            "extra": "mean: 9.062465418079494 msec\nrounds: 354"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.Bounded]",
            "value": 67.92358187556452,
            "unit": "iter/sec",
            "range": "stddev: 0.00010691313342722001",
            "extra": "mean: 14.722427357143683 msec\nrounds: 70"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.TimeOptimal]",
            "value": 78.9350067835205,
            "unit": "iter/sec",
            "range": "stddev: 0.00010864290994430293",
            "extra": "mean: 12.668650333336931 msec\nrounds: 81"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[ur5]",
            "value": 2772.7244229209823,
            "unit": "iter/sec",
            "range": "stddev: 0.000007388280552207186",
            "extra": "mean: 360.65610838690196 usec\nrounds: 2814"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.Bounded]",
            "value": 11.422771565265272,
            "unit": "iter/sec",
            "range": "stddev: 0.0008127584573419541",
            "extra": "mean: 87.5444277499895 msec\nrounds: 12"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.TimeOptimal]",
            "value": 14.346261425053298,
            "unit": "iter/sec",
            "range": "stddev: 0.000518198728513534",
            "extra": "mean: 69.70457113333168 msec\nrounds: 15"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[franka]",
            "value": 2295.5881665406414,
            "unit": "iter/sec",
            "range": "stddev: 0.000008294995267648136",
            "extra": "mean: 435.61820651260786 usec\nrounds: 2334"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[dual]",
            "value": 1471.6582071805967,
            "unit": "iter/sec",
            "range": "stddev: 0.00000907796797903133",
            "extra": "mean: 679.505604712252 usec\nrounds: 1528"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[so101]",
            "value": 3.2080559951464678,
            "unit": "iter/sec",
            "range": "stddev: 0.27291006749568614",
            "extra": "mean: 311.71525731250324 msec\nrounds: 32"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[so101]",
            "value": 7.835402738586875,
            "unit": "iter/sec",
            "range": "stddev: 0.06590419980868294",
            "extra": "mean: 127.62585834615963 msec\nrounds: 26"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[kinova]",
            "value": 1.3065606871392703,
            "unit": "iter/sec",
            "range": "stddev: 0.9426128639914215",
            "extra": "mean: 765.3681990000109 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[kinova]",
            "value": 7.090820279333382,
            "unit": "iter/sec",
            "range": "stddev: 0.11580087738647257",
            "extra": "mean: 141.02740735293483 msec\nrounds: 17"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[ur5]",
            "value": 0.30015483093429257,
            "unit": "iter/sec",
            "range": "stddev: 5.467861885065701",
            "extra": "mean: 3.3316138770357213 sec\nrounds: 28"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[ur5]",
            "value": 4.0608549664193445,
            "unit": "iter/sec",
            "range": "stddev: 0.4101092388718144",
            "extra": "mean: 246.25356193938376 msec\nrounds: 66"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[franka]",
            "value": 11.056192831029966,
            "unit": "iter/sec",
            "range": "stddev: 0.04796608272233079",
            "extra": "mean: 90.44704766666433 msec\nrounds: 33"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[franka]",
            "value": 35.82183637615266,
            "unit": "iter/sec",
            "range": "stddev: 0.007098480360795073",
            "extra": "mean: 27.915933440690964 msec\nrounds: 59"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[dual]",
            "value": 0.1803345946239169,
            "unit": "iter/sec",
            "range": "stddev: 7.401169483454586",
            "extra": "mean: 5.54524772179999 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[dual]",
            "value": 14.630708232083611,
            "unit": "iter/sec",
            "range": "stddev: 0.014651066141042743",
            "extra": "mean: 68.34939116666305 msec\nrounds: 24"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[tiago_pro]",
            "value": 0.13123551653219975,
            "unit": "iter/sec",
            "range": "stddev: 9.634595415116626",
            "extra": "mean: 7.6198884755000105 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[tiago_pro]",
            "value": 4.059267602924055,
            "unit": "iter/sec",
            "range": "stddev: 0.10897222262735425",
            "extra": "mean: 246.34985860002416 msec\nrounds: 10"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "7966037+eholum@users.noreply.github.com",
            "name": "Erik Holum",
            "username": "eholum"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "f52bf2f93ddf9328d570507356d43720afb83477",
          "message": "Pin tapi to >4 for macos (#383)",
          "timestamp": "2026-10-17T08:21:24-04:00",
          "tree_id": "01e146c9ab7c43c14f17809da34f2b4f3ba99daa",
          "url": "https://github.com/open-planning/roboplan/commit/f52bf2f93ddf9328d570507356d43720afb83477"
        },
        "date": 1792240319614,
        "tool": "pytest",
        "benches": [
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.Bounded]",
            "value": 138.94680145587833,
            "unit": "iter/sec",
            "range": "stddev: 0.00013889467777475857",
            "extra": "mean: 7.196999063829071 msec\nrounds: 141"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.TimeOptimal]",
            "value": 166.08689645784102,
            "unit": "iter/sec",
            "range": "stddev: 0.00006820259998613435",
            "extra": "mean: 6.020944585799018 msec\nrounds: 169"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[so101]",
            "value": 3370.33429350014,
            "unit": "iter/sec",
            "range": "stddev: 0.00004492937046642062",
            "extra": "mean: 296.7064726868639 usec\nrounds: 3588"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-no_obstacles]",
            "value": 66.28482919213984,
            "unit": "iter/sec",
            "range": "stddev: 0.011496820721103187",
            "extra": "mean: 15.086408340908596 msec\nrounds: 352"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-no_obstacles]",
            "value": 400.1186886298215,
            "unit": "iter/sec",
            "range": "stddev: 0.0001108323663054751",
            "extra": "mean: 2.4992584161075557 msec\nrounds: 447"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.Bounded]",
            "value": 80.38656384935207,
            "unit": "iter/sec",
            "range": "stddev: 0.00030331937226146826",
            "extra": "mean: 12.439889853658176 msec\nrounds: 82"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.TimeOptimal]",
            "value": 93.72573756494502,
            "unit": "iter/sec",
            "range": "stddev: 0.0007094970084987034",
            "extra": "mean: 10.669427907217841 msec\nrounds: 97"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[kinova]",
            "value": 1395.346989121477,
            "unit": "iter/sec",
            "range": "stddev: 0.000024703262566697002",
            "extra": "mean: 716.6676158663653 usec\nrounds: 1437"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-with_obstacles]",
            "value": 428.3166128287574,
            "unit": "iter/sec",
            "range": "stddev: 0.00012811338372433717",
            "extra": "mean: 2.33472148884359 msec\nrounds: 493"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-with_obstacles]",
            "value": 88.94863090133593,
            "unit": "iter/sec",
            "range": "stddev: 0.009004112965473253",
            "extra": "mean: 11.242443979932926 msec\nrounds: 299"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.Bounded]",
            "value": 49.93534136456185,
            "unit": "iter/sec",
            "range": "stddev: 0.0006529035747452331",
            "extra": "mean: 20.0258969433957 msec\nrounds: 53"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.TimeOptimal]",
            "value": 58.415279388533314,
            "unit": "iter/sec",
            "range": "stddev: 0.00015480288313051875",
            "extra": "mean: 17.118808819671518 msec\nrounds: 61"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[ur5]",
            "value": 1976.7900907119986,
            "unit": "iter/sec",
            "range": "stddev: 0.000015965192736418727",
            "extra": "mean: 505.8706054317689 usec\nrounds: 2025"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.Bounded]",
            "value": 8.68167900400653,
            "unit": "iter/sec",
            "range": "stddev: 0.001176953675016998",
            "extra": "mean: 115.18509260000371 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.TimeOptimal]",
            "value": 10.733066856327008,
            "unit": "iter/sec",
            "range": "stddev: 0.0013300840355415797",
            "extra": "mean: 93.17001500000092 msec\nrounds: 11"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[franka]",
            "value": 1651.9642866085042,
            "unit": "iter/sec",
            "range": "stddev: 0.000021978896869839756",
            "extra": "mean: 605.3399629195422 usec\nrounds: 1699"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[dual]",
            "value": 1080.1097954566653,
            "unit": "iter/sec",
            "range": "stddev: 0.000025285892846211775",
            "extra": "mean: 925.8318035873424 usec\nrounds: 1115"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[so101]",
            "value": 2.5623812601812577,
            "unit": "iter/sec",
            "range": "stddev: 0.2203756061787336",
            "extra": "mean: 390.26198620000133 msec\nrounds: 15"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[so101]",
            "value": 7.465250396202745,
            "unit": "iter/sec",
            "range": "stddev: 0.07355498444870028",
            "extra": "mean: 133.95397969620112 msec\nrounds: 79"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[kinova]",
            "value": 0.6702241241843615,
            "unit": "iter/sec",
            "range": "stddev: 2.0440279006762294",
            "extra": "mean: 1.4920382061999988 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[kinova]",
            "value": 5.3002437194478,
            "unit": "iter/sec",
            "range": "stddev: 0.11518720966643863",
            "extra": "mean: 188.67056930434583 msec\nrounds: 23"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[ur5]",
            "value": 1.2074752907795838,
            "unit": "iter/sec",
            "range": "stddev: 2.6516539356064746",
            "extra": "mean: 828.174296928568 msec\nrounds: 14"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[ur5]",
            "value": 4.197897962957973,
            "unit": "iter/sec",
            "range": "stddev: 0.40881220553182845",
            "extra": "mean: 238.2144608620663 msec\nrounds: 58"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[franka]",
            "value": 8.668874657937899,
            "unit": "iter/sec",
            "range": "stddev: 0.07029626388354507",
            "extra": "mean: 115.35522653846678 msec\nrounds: 13"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[franka]",
            "value": 28.7213306624731,
            "unit": "iter/sec",
            "range": "stddev: 0.010741365538648762",
            "extra": "mean: 34.81732833871052 msec\nrounds: 62"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[dual]",
            "value": 0.1529644053976675,
            "unit": "iter/sec",
            "range": "stddev: 6.152310369111028",
            "extra": "mean: 6.537468618272737 sec\nrounds: 11"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[dual]",
            "value": 12.524382596967259,
            "unit": "iter/sec",
            "range": "stddev: 0.030307739669159624",
            "extra": "mean: 79.8442551764705 msec\nrounds: 17"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[tiago_pro]",
            "value": 0.14896125831860194,
            "unit": "iter/sec",
            "range": "stddev: 4.706589642757536",
            "extra": "mean: 6.713154892000011 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[tiago_pro]",
            "value": 3.215618436763906,
            "unit": "iter/sec",
            "range": "stddev: 0.10691404354633019",
            "extra": "mean: 310.982170200009 msec\nrounds: 10"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "andrew.hoelscher@pal-robotics.com",
            "name": "Andrew Hoelscher",
            "username": "andrewhoelscher"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "71bf5c1d4940855a51b199f624bd99767a7aaa11",
          "message": "Add bimanual pick-and-place example for TiagoPro (#381)",
          "timestamp": "2026-10-19T08:51:38-04:00",
          "tree_id": "fd7bd8031514054829492fac2a744409cc7527d8",
          "url": "https://github.com/open-planning/roboplan/commit/71bf5c1d4940855a51b199f624bd99767a7aaa11"
        },
        "date": 1792415282152,
        "tool": "pytest",
        "benches": [
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.Bounded]",
            "value": 181.36940830979574,
            "unit": "iter/sec",
            "range": "stddev: 0.000036249719761539186",
            "extra": "mean: 5.513608989074427 msec\nrounds: 183"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.TimeOptimal]",
            "value": 217.43295027767513,
            "unit": "iter/sec",
            "range": "stddev: 0.00003663495594836182",
            "extra": "mean: 4.59911894091001 msec\nrounds: 220"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[so101]",
            "value": 4745.483026192582,
            "unit": "iter/sec",
            "range": "stddev: 0.000027770649459163217",
            "extra": "mean: 210.72670463270512 usec\nrounds: 4943"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-no_obstacles]",
            "value": 83.14628487959594,
            "unit": "iter/sec",
            "range": "stddev: 0.008961576389008269",
            "extra": "mean: 12.02699557109616 msec\nrounds: 429"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-no_obstacles]",
            "value": 476.90235947880325,
            "unit": "iter/sec",
            "range": "stddev: 0.0000926010048916303",
            "extra": "mean: 2.096865280962081 msec\nrounds: 541"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.Bounded]",
            "value": 105.61770890667522,
            "unit": "iter/sec",
            "range": "stddev: 0.00006934584112293833",
            "extra": "mean: 9.46810918691305 msec\nrounds: 107"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.TimeOptimal]",
            "value": 126.2281486056928,
            "unit": "iter/sec",
            "range": "stddev: 0.00004384901742588328",
            "extra": "mean: 7.9221632500035 msec\nrounds: 128"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[kinova]",
            "value": 1951.2150388220161,
            "unit": "iter/sec",
            "range": "stddev: 0.0000072663138909614675",
            "extra": "mean: 512.5011749620986 usec\nrounds: 1989"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-with_obstacles]",
            "value": 515.3388540759485,
            "unit": "iter/sec",
            "range": "stddev: 0.00010151968728456125",
            "extra": "mean: 1.940470803027447 msec\nrounds: 594"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-with_obstacles]",
            "value": 112.79434315991726,
            "unit": "iter/sec",
            "range": "stddev: 0.006935079144783875",
            "extra": "mean: 8.865692835164815 msec\nrounds: 364"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.Bounded]",
            "value": 63.65090774416027,
            "unit": "iter/sec",
            "range": "stddev: 0.00012974496317959212",
            "extra": "mean: 15.710695030767194 msec\nrounds: 65"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.TimeOptimal]",
            "value": 73.93123118315687,
            "unit": "iter/sec",
            "range": "stddev: 0.00012714521080280848",
            "extra": "mean: 13.526083415581228 msec\nrounds: 77"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[ur5]",
            "value": 2765.0664860299808,
            "unit": "iter/sec",
            "range": "stddev: 0.000008170286769320507",
            "extra": "mean: 361.6549565995345 usec\nrounds: 2811"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.Bounded]",
            "value": 11.500380439292616,
            "unit": "iter/sec",
            "range": "stddev: 0.0006974427033783768",
            "extra": "mean: 86.95364516667325 msec\nrounds: 12"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.TimeOptimal]",
            "value": 14.278151665510824,
            "unit": "iter/sec",
            "range": "stddev: 0.0005274179477762217",
            "extra": "mean: 70.03707646666348 msec\nrounds: 15"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[franka]",
            "value": 2302.102170200621,
            "unit": "iter/sec",
            "range": "stddev: 0.000008009870464842748",
            "extra": "mean: 434.3855858981502 usec\nrounds: 2340"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[dual]",
            "value": 1508.037840705024,
            "unit": "iter/sec",
            "range": "stddev: 0.000013349884038170524",
            "extra": "mean: 663.1133337691906 usec\nrounds: 1531"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[so101]",
            "value": 3.565284345019201,
            "unit": "iter/sec",
            "range": "stddev: 0.19095003727166338",
            "extra": "mean: 280.4825374999969 msec\nrounds: 12"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[so101]",
            "value": 9.695290701247126,
            "unit": "iter/sec",
            "range": "stddev: 0.059789821538531955",
            "extra": "mean: 103.1428588181856 msec\nrounds: 11"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[kinova]",
            "value": 1.3301332878510217,
            "unit": "iter/sec",
            "range": "stddev: 0.7715905772307967",
            "extra": "mean: 751.80435609999 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[kinova]",
            "value": 10.096623097875781,
            "unit": "iter/sec",
            "range": "stddev: 0.06645437509411896",
            "extra": "mean: 99.04301570001053 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[ur5]",
            "value": 0.38793731065451803,
            "unit": "iter/sec",
            "range": "stddev: 4.4063347680505425",
            "extra": "mean: 2.5777360736785675 sec\nrounds: 28"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[ur5]",
            "value": 2.7550333577092903,
            "unit": "iter/sec",
            "range": "stddev: 0.5922393392475489",
            "extra": "mean: 362.9720116461543 msec\nrounds: 65"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[franka]",
            "value": 11.659583719191527,
            "unit": "iter/sec",
            "range": "stddev: 0.04476827046958995",
            "extra": "mean: 85.7663553076953 msec\nrounds: 13"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[franka]",
            "value": 36.34560542602842,
            "unit": "iter/sec",
            "range": "stddev: 0.007954619484909501",
            "extra": "mean: 27.513642661290305 msec\nrounds: 62"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[dual]",
            "value": 0.2525939713870518,
            "unit": "iter/sec",
            "range": "stddev: 5.847775877631203",
            "extra": "mean: 3.958922671466659 sec\nrounds: 15"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[dual]",
            "value": 15.388818634448059,
            "unit": "iter/sec",
            "range": "stddev: 0.011440046048037547",
            "extra": "mean: 64.9822461200165 msec\nrounds: 25"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[tiago_pro]",
            "value": 0.16691626869249107,
            "unit": "iter/sec",
            "range": "stddev: 7.479184977308306",
            "extra": "mean: 5.991027764000012 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[tiago_pro]",
            "value": 3.4682239428705697,
            "unit": "iter/sec",
            "range": "stddev: 0.1070338527788674",
            "extra": "mean: 288.3320155999854 msec\nrounds: 10"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "4603398+sea-bass@users.noreply.github.com",
            "name": "Sebastian Castro",
            "username": "sea-bass"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "cb6379e9e681096fe3400c119f39cbb5dc15ebb3",
          "message": "Revert tapi pin and instead re-lock the project (#385)",
          "timestamp": "2026-10-20T09:18:17-04:00",
          "tree_id": "358affb9f65829bbf4318915882f58f696350a39",
          "url": "https://github.com/open-planning/roboplan/commit/cb6379e9e681096fe3400c119f39cbb5dc15ebb3"
        },
        "date": 1792503805458,
        "tool": "pytest",
        "benches": [
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.Bounded]",
            "value": 178.71974171620207,
            "unit": "iter/sec",
            "range": "stddev: 0.00039692148608203406",
            "extra": "mean: 5.595352759562228 msec\nrounds: 183"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.TimeOptimal]",
            "value": 217.33831567725917,
            "unit": "iter/sec",
            "range": "stddev: 0.000025226827396937424",
            "extra": "mean: 4.601121513635772 msec\nrounds: 220"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[so101]",
            "value": 4778.890838040059,
            "unit": "iter/sec",
            "range": "stddev: 0.000006102006266315264",
            "extra": "mean: 209.2535765914512 usec\nrounds: 4870"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-no_obstacles]",
            "value": 84.01373639568368,
            "unit": "iter/sec",
            "range": "stddev: 0.008886312033563784",
            "extra": "mean: 11.9028154549662 msec\nrounds: 433"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-no_obstacles]",
            "value": 472.0941239600133,
            "unit": "iter/sec",
            "range": "stddev: 0.00009043254366806847",
            "extra": "mean: 2.1182216622647494 msec\nrounds: 530"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.Bounded]",
            "value": 106.4961209912222,
            "unit": "iter/sec",
            "range": "stddev: 0.00004515912728360541",
            "extra": "mean: 9.390013370368896 msec\nrounds: 108"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.TimeOptimal]",
            "value": 126.06900123415213,
            "unit": "iter/sec",
            "range": "stddev: 0.00005905997138860593",
            "extra": "mean: 7.932164054688329 msec\nrounds: 128"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[kinova]",
            "value": 1933.742009972987,
            "unit": "iter/sec",
            "range": "stddev: 0.00004192395605727129",
            "extra": "mean: 517.1320656233606 usec\nrounds: 1981"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-with_obstacles]",
            "value": 506.811627794627,
            "unit": "iter/sec",
            "range": "stddev: 0.000118942261430818",
            "extra": "mean: 1.9731196862066187 msec\nrounds: 580"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-with_obstacles]",
            "value": 113.13152498498717,
            "unit": "iter/sec",
            "range": "stddev: 0.006848450025556712",
            "extra": "mean: 8.839269161559544 msec\nrounds: 359"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.Bounded]",
            "value": 65.6449419611499,
            "unit": "iter/sec",
            "range": "stddev: 0.0017270008518149233",
            "extra": "mean: 15.233466130441883 msec\nrounds: 69"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.TimeOptimal]",
            "value": 78.71371763778872,
            "unit": "iter/sec",
            "range": "stddev: 0.0002115103721216734",
            "extra": "mean: 12.70426591463547 msec\nrounds: 82"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[ur5]",
            "value": 2731.3928376678937,
            "unit": "iter/sec",
            "range": "stddev: 0.000006301366957050336",
            "extra": "mean: 366.11357627115103 usec\nrounds: 2773"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.Bounded]",
            "value": 11.515942489406594,
            "unit": "iter/sec",
            "range": "stddev: 0.0005950746577717984",
            "extra": "mean: 86.83614049999733 msec\nrounds: 12"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.TimeOptimal]",
            "value": 14.287094759235917,
            "unit": "iter/sec",
            "range": "stddev: 0.0005304373970810253",
            "extra": "mean: 69.9932363333384 msec\nrounds: 15"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[franka]",
            "value": 2273.1953528075687,
            "unit": "iter/sec",
            "range": "stddev: 0.00000839651172563495",
            "extra": "mean: 439.9093983563375 usec\nrounds: 2312"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[dual]",
            "value": 1498.5641272823975,
            "unit": "iter/sec",
            "range": "stddev: 0.000009842352363282204",
            "extra": "mean: 667.3054437873614 usec\nrounds: 1521"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[so101]",
            "value": 2.7430744764015547,
            "unit": "iter/sec",
            "range": "stddev: 0.2969815909041785",
            "extra": "mean: 364.5544474285763 msec\nrounds: 14"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[so101]",
            "value": 8.089846344783094,
            "unit": "iter/sec",
            "range": "stddev: 0.07661223841402422",
            "extra": "mean: 123.61174210000551 msec\nrounds: 30"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[kinova]",
            "value": 2.4817931780858067,
            "unit": "iter/sec",
            "range": "stddev: 0.38160920754812",
            "extra": "mean: 402.9344624000032 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[kinova]",
            "value": 7.903553377407237,
            "unit": "iter/sec",
            "range": "stddev: 0.0799524238976816",
            "extra": "mean: 126.52536805262271 msec\nrounds: 19"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[ur5]",
            "value": 0.549341698195495,
            "unit": "iter/sec",
            "range": "stddev: 3.8043521236986724",
            "extra": "mean: 1.8203606303414612 sec\nrounds: 41"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[ur5]",
            "value": 3.234229169079569,
            "unit": "iter/sec",
            "range": "stddev: 0.5335566321284081",
            "extra": "mean: 309.1926847857199 msec\nrounds: 28"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[franka]",
            "value": 9.786962985695347,
            "unit": "iter/sec",
            "range": "stddev: 0.06596308169963226",
            "extra": "mean: 102.17674282222207 msec\nrounds: 45"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[franka]",
            "value": 36.4714826113829,
            "unit": "iter/sec",
            "range": "stddev: 0.007812195904914948",
            "extra": "mean: 27.41868244445582 msec\nrounds: 54"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[dual]",
            "value": 0.19510388036691126,
            "unit": "iter/sec",
            "range": "stddev: 5.671624274388343",
            "extra": "mean: 5.1254746862000164 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[dual]",
            "value": 15.82711398849958,
            "unit": "iter/sec",
            "range": "stddev: 0.011191882921318069",
            "extra": "mean: 63.182712952382076 msec\nrounds: 21"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[tiago_pro]",
            "value": 0.3841002271894898,
            "unit": "iter/sec",
            "range": "stddev: 4.176032664792374",
            "extra": "mean: 2.60348713489999 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[tiago_pro]",
            "value": 4.496824491389253,
            "unit": "iter/sec",
            "range": "stddev: 0.08552386069797448",
            "extra": "mean: 222.37914819999105 msec\nrounds: 10"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "4603398+sea-bass@users.noreply.github.com",
            "name": "Sebastian Castro",
            "username": "sea-bass"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "91ce8fc4f42b666d49142549b5d44f6fe31759a8",
          "message": "Fix lighting in MuJoCo example (#389)",
          "timestamp": "2026-10-20T09:47:47-04:00",
          "tree_id": "1ddd876fe96db94fb6e4452236752c38dea876eb",
          "url": "https://github.com/open-planning/roboplan/commit/91ce8fc4f42b666d49142549b5d44f6fe31759a8"
        },
        "date": 1792504915815,
        "tool": "pytest",
        "benches": [
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.Bounded]",
            "value": 140.3894048744788,
            "unit": "iter/sec",
            "range": "stddev: 0.00005768212019482547",
            "extra": "mean: 7.123044654930285 msec\nrounds: 142"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.TimeOptimal]",
            "value": 167.63168497173996,
            "unit": "iter/sec",
            "range": "stddev: 0.00009683423916681931",
            "extra": "mean: 5.9654593352598235 msec\nrounds: 173"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[so101]",
            "value": 3374.0202021856758,
            "unit": "iter/sec",
            "range": "stddev: 0.000015188711201736706",
            "extra": "mean: 296.3823391905609 usec\nrounds: 3756"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-no_obstacles]",
            "value": 67.12143719429812,
            "unit": "iter/sec",
            "range": "stddev: 0.011415895137695775",
            "extra": "mean: 14.898369906849203 msec\nrounds: 365"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-no_obstacles]",
            "value": 395.38448352723486,
            "unit": "iter/sec",
            "range": "stddev: 0.00011647726606736615",
            "extra": "mean: 2.529183722838527 msec\nrounds: 451"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.Bounded]",
            "value": 79.92257795284074,
            "unit": "iter/sec",
            "range": "stddev: 0.00019119797804204256",
            "extra": "mean: 12.51210891357961 msec\nrounds: 81"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.TimeOptimal]",
            "value": 92.76184692459309,
            "unit": "iter/sec",
            "range": "stddev: 0.00017985245137321846",
            "extra": "mean: 10.780294195876767 msec\nrounds: 97"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[kinova]",
            "value": 1382.7355189541597,
            "unit": "iter/sec",
            "range": "stddev: 0.000025399422619986822",
            "extra": "mean: 723.2041025143809 usec\nrounds: 1551"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-with_obstacles]",
            "value": 430.497258920201,
            "unit": "iter/sec",
            "range": "stddev: 0.0001199223129682773",
            "extra": "mean: 2.322895162000009 msec\nrounds: 500"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-with_obstacles]",
            "value": 88.30805587541778,
            "unit": "iter/sec",
            "range": "stddev: 0.009377696123014177",
            "extra": "mean: 11.323995190322936 msec\nrounds: 310"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.Bounded]",
            "value": 49.768054831964115,
            "unit": "iter/sec",
            "range": "stddev: 0.00016230061066788994",
            "extra": "mean: 20.093210461537634 msec\nrounds: 52"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.TimeOptimal]",
            "value": 58.998152279418235,
            "unit": "iter/sec",
            "range": "stddev: 0.00016159599916011863",
            "extra": "mean: 16.949683360657627 msec\nrounds: 61"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[ur5]",
            "value": 1922.587760004386,
            "unit": "iter/sec",
            "range": "stddev: 0.00004567802044217588",
            "extra": "mean: 520.1323033481284 usec\nrounds: 2001"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.Bounded]",
            "value": 8.812045486700388,
            "unit": "iter/sec",
            "range": "stddev: 0.0012432754899796056",
            "extra": "mean: 113.48103019999769 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.TimeOptimal]",
            "value": 10.80105066297296,
            "unit": "iter/sec",
            "range": "stddev: 0.0011964106808257553",
            "extra": "mean: 92.58358572727522 msec\nrounds: 11"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[franka]",
            "value": 1609.5697264506248,
            "unit": "iter/sec",
            "range": "stddev: 0.000018538177346021157",
            "extra": "mean: 621.2840509899315 usec\nrounds: 1667"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[dual]",
            "value": 1066.4860134116109,
            "unit": "iter/sec",
            "range": "stddev: 0.00002301927227598262",
            "extra": "mean: 937.658804170411 usec\nrounds: 1103"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[so101]",
            "value": 2.5532307584129477,
            "unit": "iter/sec",
            "range": "stddev: 0.3249396828832024",
            "extra": "mean: 391.6606427777746 msec\nrounds: 27"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[so101]",
            "value": 7.757311779546748,
            "unit": "iter/sec",
            "range": "stddev: 0.0901487672238775",
            "extra": "mean: 128.9106366250022 msec\nrounds: 24"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[kinova]",
            "value": 1.8429783625657892,
            "unit": "iter/sec",
            "range": "stddev: 0.275746361943671",
            "extra": "mean: 542.5999677000021 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[kinova]",
            "value": 6.093922424583388,
            "unit": "iter/sec",
            "range": "stddev: 0.12442114995091681",
            "extra": "mean: 164.09792089999655 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[ur5]",
            "value": 0.5822780923433257,
            "unit": "iter/sec",
            "range": "stddev: 4.407256541802651",
            "extra": "mean: 1.7173924507027045 sec\nrounds: 37"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[ur5]",
            "value": 4.554738367877145,
            "unit": "iter/sec",
            "range": "stddev: 0.4577806296571095",
            "extra": "mean: 219.55157886842053 msec\nrounds: 38"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[franka]",
            "value": 7.429574522274721,
            "unit": "iter/sec",
            "range": "stddev: 0.07084808257451791",
            "extra": "mean: 134.59720970587009 msec\nrounds: 17"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[franka]",
            "value": 26.834574817046047,
            "unit": "iter/sec",
            "range": "stddev: 0.010606990442591861",
            "extra": "mean: 37.26535660869771 msec\nrounds: 46"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[dual]",
            "value": 0.16600730238406589,
            "unit": "iter/sec",
            "range": "stddev: 6.888590269180045",
            "extra": "mean: 6.023831395600007 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[dual]",
            "value": 13.126582209691954,
            "unit": "iter/sec",
            "range": "stddev: 0.019678760669495164",
            "extra": "mean: 76.18129258823019 msec\nrounds: 17"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[tiago_pro]",
            "value": 0.18983487932344387,
            "unit": "iter/sec",
            "range": "stddev: 4.831160643851017",
            "extra": "mean: 5.267735853199997 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[tiago_pro]",
            "value": 3.144907104727305,
            "unit": "iter/sec",
            "range": "stddev: 0.13733518748861034",
            "extra": "mean: 317.97441600002685 msec\nrounds: 10"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "4603398+sea-bass@users.noreply.github.com",
            "name": "Sebastian Castro",
            "username": "sea-bass"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "d61fc01451a5b29000b49d37740d4b85817d05b5",
          "message": "Update pixi for latest ROS Kilted version (#390)",
          "timestamp": "2026-10-21T09:37:29-04:00",
          "tree_id": "51e861a95fc0dd8191858b9f53f8a8059f63dbb6",
          "url": "https://github.com/open-planning/roboplan/commit/d61fc01451a5b29000b49d37740d4b85817d05b5"
        },
        "date": 1792590623761,
        "tool": "pytest",
        "benches": [
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.Bounded]",
            "value": 235.95849370616583,
            "unit": "iter/sec",
            "range": "stddev: 0.00013845894259942724",
            "extra": "mean: 4.238033496032057 msec\nrounds: 252"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.TimeOptimal]",
            "value": 283.668590110941,
            "unit": "iter/sec",
            "range": "stddev: 0.00009777667814482929",
            "extra": "mean: 3.5252404914090287 msec\nrounds: 291"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[so101]",
            "value": 5671.831017652923,
            "unit": "iter/sec",
            "range": "stddev: 0.000036164658195186696",
            "extra": "mean: 176.3099071336249 usec\nrounds: 6224"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-no_obstacles]",
            "value": 106.58480029291867,
            "unit": "iter/sec",
            "range": "stddev: 0.006991970028355485",
            "extra": "mean: 9.382200813359674 msec\nrounds: 509"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-no_obstacles]",
            "value": 615.1889345520999,
            "unit": "iter/sec",
            "range": "stddev: 0.00007509215996392753",
            "extra": "mean: 1.625516884057853 msec\nrounds: 690"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.Bounded]",
            "value": 136.08679031344118,
            "unit": "iter/sec",
            "range": "stddev: 0.0003028246164065399",
            "extra": "mean: 7.348251786207576 msec\nrounds: 145"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.TimeOptimal]",
            "value": 159.637419239197,
            "unit": "iter/sec",
            "range": "stddev: 0.00021947902394933315",
            "extra": "mean: 6.264195479767957 msec\nrounds: 173"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[kinova]",
            "value": 2470.307793382145,
            "unit": "iter/sec",
            "range": "stddev: 0.000022349365495815226",
            "extra": "mean: 404.8078553931456 usec\nrounds: 2531"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-with_obstacles]",
            "value": 646.465629333427,
            "unit": "iter/sec",
            "range": "stddev: 0.00009399733384305714",
            "extra": "mean: 1.5468726481732733 msec\nrounds: 739"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-with_obstacles]",
            "value": 143.56982225496333,
            "unit": "iter/sec",
            "range": "stddev: 0.00535069686171225",
            "extra": "mean: 6.965252058500957 msec\nrounds: 547"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.Bounded]",
            "value": 91.58866127364405,
            "unit": "iter/sec",
            "range": "stddev: 0.00030317954391667477",
            "extra": "mean: 10.918382102040445 msec\nrounds: 98"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.TimeOptimal]",
            "value": 104.13763849538937,
            "unit": "iter/sec",
            "range": "stddev: 0.0006339361826189767",
            "extra": "mean: 9.602675981982005 msec\nrounds: 111"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[ur5]",
            "value": 3386.174073184652,
            "unit": "iter/sec",
            "range": "stddev: 0.000032542802918486094",
            "extra": "mean: 295.3185448790331 usec\nrounds: 3632"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.Bounded]",
            "value": 14.98774337006372,
            "unit": "iter/sec",
            "range": "stddev: 0.0025518592861964967",
            "extra": "mean: 66.72118512499914 msec\nrounds: 16"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.TimeOptimal]",
            "value": 19.274799314988964,
            "unit": "iter/sec",
            "range": "stddev: 0.0011279603910803416",
            "extra": "mean: 51.881214619046865 msec\nrounds: 21"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[franka]",
            "value": 2837.4083669259244,
            "unit": "iter/sec",
            "range": "stddev: 0.000008406334365501353",
            "extra": "mean: 352.4342888589596 usec\nrounds: 2953"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[dual]",
            "value": 1884.476822604405,
            "unit": "iter/sec",
            "range": "stddev: 0.000012575096070679299",
            "extra": "mean: 530.6512598111815 usec\nrounds: 2013"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[so101]",
            "value": 4.396245235178993,
            "unit": "iter/sec",
            "range": "stddev: 0.17445634883398325",
            "extra": "mean: 227.46683738157864 msec\nrounds: 76"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[so101]",
            "value": 10.241577644737955,
            "unit": "iter/sec",
            "range": "stddev: 0.06143835762188558",
            "extra": "mean: 97.64120672500027 msec\nrounds: 40"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[kinova]",
            "value": 2.2153334037472465,
            "unit": "iter/sec",
            "range": "stddev: 0.23351072146701055",
            "extra": "mean: 451.3993236000033 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[kinova]",
            "value": 9.018227468208776,
            "unit": "iter/sec",
            "range": "stddev: 0.06352836457823942",
            "extra": "mean: 110.88653546666667 msec\nrounds: 30"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[ur5]",
            "value": 0.5068129995288086,
            "unit": "iter/sec",
            "range": "stddev: 3.968260159586798",
            "extra": "mean: 1.9731143457837792 sec\nrounds: 37"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[ur5]",
            "value": 4.617112028367153,
            "unit": "iter/sec",
            "range": "stddev: 0.5245874802156458",
            "extra": "mean: 216.58560456321683 msec\nrounds: 87"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[franka]",
            "value": 13.227546387362224,
            "unit": "iter/sec",
            "range": "stddev: 0.04959839543804605",
            "extra": "mean: 75.59981048000054 msec\nrounds: 25"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[franka]",
            "value": 45.200357826002616,
            "unit": "iter/sec",
            "range": "stddev: 0.005305230331456617",
            "extra": "mean: 22.12371866279177 msec\nrounds: 86"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[dual]",
            "value": 0.13280122358634192,
            "unit": "iter/sec",
            "range": "stddev: 9.51713484088537",
            "extra": "mean: 7.530051101899983 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[dual]",
            "value": 17.56117567009489,
            "unit": "iter/sec",
            "range": "stddev: 0.016410683358795287",
            "extra": "mean: 56.94379572222551 msec\nrounds: 36"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[tiago_pro]",
            "value": 0.13470099878360992,
            "unit": "iter/sec",
            "range": "stddev: 9.37345386508896",
            "extra": "mean: 7.423849927100003 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[tiago_pro]",
            "value": 5.287979260261909,
            "unit": "iter/sec",
            "range": "stddev: 0.10415550123592905",
            "extra": "mean: 189.1081546999999 msec\nrounds: 10"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "4603398+sea-bass@users.noreply.github.com",
            "name": "Sebastian Castro",
            "username": "sea-bass"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "fad7f0793c952eb376e91d6de382eef8fb863e3f",
          "message": "Upgrade to ProxSuite 0.8 (#386)",
          "timestamp": "2026-10-27T20:59:11-05:00",
          "tree_id": "57cd1415504af7a2c1d689a2016316ab5676d55b",
          "url": "https://github.com/open-planning/roboplan/commit/fad7f0793c952eb376e91d6de382eef8fb863e3f"
        },
        "date": 1793153476960,
        "tool": "pytest",
        "benches": [
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.Bounded]",
            "value": 179.28946193294988,
            "unit": "iter/sec",
            "range": "stddev: 0.00004121930150861605",
            "extra": "mean: 5.577572653846085 msec\nrounds: 182"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.TimeOptimal]",
            "value": 211.39137462199744,
            "unit": "iter/sec",
            "range": "stddev: 0.00009814595728352752",
            "extra": "mean: 4.730561981481811 msec\nrounds: 216"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[so101]",
            "value": 4718.215784898253,
            "unit": "iter/sec",
            "range": "stddev: 0.00000779984685481422",
            "extra": "mean: 211.9445242841017 usec\nrounds: 4818"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-no_obstacles]",
            "value": 83.6377533412425,
            "unit": "iter/sec",
            "range": "stddev: 0.008871506619562226",
            "extra": "mean: 11.95632307242872 msec\nrounds: 428"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-no_obstacles]",
            "value": 472.2025637512115,
            "unit": "iter/sec",
            "range": "stddev: 0.00009162029353162053",
            "extra": "mean: 2.1177352195124213 msec\nrounds: 533"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.Bounded]",
            "value": 105.43164107994998,
            "unit": "iter/sec",
            "range": "stddev: 0.0003132893521609546",
            "extra": "mean: 9.484818691588885 msec\nrounds: 107"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.TimeOptimal]",
            "value": 125.7694217036894,
            "unit": "iter/sec",
            "range": "stddev: 0.00003984863520618333",
            "extra": "mean: 7.951058265625033 msec\nrounds: 128"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[kinova]",
            "value": 1950.5134631078263,
            "unit": "iter/sec",
            "range": "stddev: 0.000008687700995093561",
            "extra": "mean: 512.685515334338 usec\nrounds: 1989"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-with_obstacles]",
            "value": 507.29395588839066,
            "unit": "iter/sec",
            "range": "stddev: 0.00011255571647834342",
            "extra": "mean: 1.9712436712335073 msec\nrounds: 584"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-with_obstacles]",
            "value": 110.61761330030872,
            "unit": "iter/sec",
            "range": "stddev: 0.006995098826915621",
            "extra": "mean: 9.040151655461628 msec\nrounds: 357"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.Bounded]",
            "value": 67.62706806050818,
            "unit": "iter/sec",
            "range": "stddev: 0.0007278250280358499",
            "extra": "mean: 14.78697847887279 msec\nrounds: 71"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.TimeOptimal]",
            "value": 79.48191854535594,
            "unit": "iter/sec",
            "range": "stddev: 0.0001246478347226198",
            "extra": "mean: 12.581477879517406 msec\nrounds: 83"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[ur5]",
            "value": 2712.7251089593633,
            "unit": "iter/sec",
            "range": "stddev: 0.000008006141436181144",
            "extra": "mean: 368.6330018096131 usec\nrounds: 2763"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.Bounded]",
            "value": 11.331763289828977,
            "unit": "iter/sec",
            "range": "stddev: 0.0006967851390556733",
            "extra": "mean: 88.24751933333867 msec\nrounds: 12"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.TimeOptimal]",
            "value": 14.164985025612237,
            "unit": "iter/sec",
            "range": "stddev: 0.0005035204759212054",
            "extra": "mean: 70.59661540000661 msec\nrounds: 15"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[franka]",
            "value": 2276.227096128529,
            "unit": "iter/sec",
            "range": "stddev: 0.000007943154418939988",
            "extra": "mean: 439.3234759839333 usec\nrounds: 2311"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[dual]",
            "value": 1499.3990681687499,
            "unit": "iter/sec",
            "range": "stddev: 0.00001087010384397655",
            "extra": "mean: 666.9338545216802 usec\nrounds: 1526"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[so101]",
            "value": 2.0870471392471246,
            "unit": "iter/sec",
            "range": "stddev: 0.32666984186024817",
            "extra": "mean: 479.1458617272714 msec\nrounds: 11"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[so101]",
            "value": 10.069517179353758,
            "unit": "iter/sec",
            "range": "stddev: 0.05277871732208486",
            "extra": "mean: 99.30962748148148 msec\nrounds: 27"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[kinova]",
            "value": 1.8361396500253075,
            "unit": "iter/sec",
            "range": "stddev: 0.5029556129760963",
            "extra": "mean: 544.6208843571441 msec\nrounds: 14"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[kinova]",
            "value": 6.155653727860474,
            "unit": "iter/sec",
            "range": "stddev: 0.07817338524718535",
            "extra": "mean: 162.45228276470496 msec\nrounds: 17"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[ur5]",
            "value": 0.26116458875894055,
            "unit": "iter/sec",
            "range": "stddev: 5.742473105311013",
            "extra": "mean: 3.8290030235416688 sec\nrounds: 24"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[ur5]",
            "value": 7.121900698524208,
            "unit": "iter/sec",
            "range": "stddev: 0.31575248906137465",
            "extra": "mean: 140.41195494444605 msec\nrounds: 54"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[franka]",
            "value": 9.674012292535036,
            "unit": "iter/sec",
            "range": "stddev: 0.05168208647803871",
            "extra": "mean: 103.36972600000222 msec\nrounds: 29"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[franka]",
            "value": 37.33998474255816,
            "unit": "iter/sec",
            "range": "stddev: 0.0070736723590231546",
            "extra": "mean: 26.780942919354022 msec\nrounds: 62"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[dual]",
            "value": 0.15906814616407705,
            "unit": "iter/sec",
            "range": "stddev: 3.8686661131962636",
            "extra": "mean: 6.286613782300014 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[dual]",
            "value": 15.849813758349388,
            "unit": "iter/sec",
            "range": "stddev: 0.015094608111976314",
            "extra": "mean: 63.092223999996115 msec\nrounds: 20"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[tiago_pro]",
            "value": 0.16104978829282607,
            "unit": "iter/sec",
            "range": "stddev: 7.955449066917268",
            "extra": "mean: 6.209259947500004 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[tiago_pro]",
            "value": 4.355407305853565,
            "unit": "iter/sec",
            "range": "stddev: 0.09037573149181521",
            "extra": "mean: 229.5996515999832 msec\nrounds: 10"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "4603398+sea-bass@users.noreply.github.com",
            "name": "Sebastian Castro",
            "username": "sea-bass"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "c0f014508237bfbd5abbf55aff4b8059902c17e0",
          "message": "Depend on libcereal-dev in OInK (#391)",
          "timestamp": "2026-10-27T22:19:49-05:00",
          "tree_id": "e0b2734b292721e03afc5629157480f6ba0f93bf",
          "url": "https://github.com/open-planning/roboplan/commit/c0f014508237bfbd5abbf55aff4b8059902c17e0"
        },
        "date": 1793158540675,
        "tool": "pytest",
        "benches": [
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.Bounded]",
            "value": 138.61312468571145,
            "unit": "iter/sec",
            "range": "stddev: 0.000052399548062147116",
            "extra": "mean: 7.214324056739789 msec\nrounds: 141"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.TimeOptimal]",
            "value": 168.2097382053766,
            "unit": "iter/sec",
            "range": "stddev: 0.00004560511282399468",
            "extra": "mean: 5.944959017646437 msec\nrounds: 170"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[so101]",
            "value": 3651.9522620763696,
            "unit": "iter/sec",
            "range": "stddev: 0.000007595864377360893",
            "extra": "mean: 273.8261423580153 usec\nrounds: 3723"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-no_obstacles]",
            "value": 65.08628397024333,
            "unit": "iter/sec",
            "range": "stddev: 0.011301671080739057",
            "extra": "mean: 15.364220216615655 msec\nrounds: 337"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-no_obstacles]",
            "value": 369.96399914035396,
            "unit": "iter/sec",
            "range": "stddev: 0.00011415072211752133",
            "extra": "mean: 2.7029657002400063 msec\nrounds: 417"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.Bounded]",
            "value": 82.44102601813086,
            "unit": "iter/sec",
            "range": "stddev: 0.00004447044726125009",
            "extra": "mean: 12.129883000001417 msec\nrounds: 84"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.TimeOptimal]",
            "value": 98.07734488868904,
            "unit": "iter/sec",
            "range": "stddev: 0.0003589926394611206",
            "extra": "mean: 10.19603458000347 msec\nrounds: 100"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[kinova]",
            "value": 1500.8003553873189,
            "unit": "iter/sec",
            "range": "stddev: 0.000010311829333723894",
            "extra": "mean: 666.3111428581219 usec\nrounds: 1533"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-with_obstacles]",
            "value": 397.5151735523782,
            "unit": "iter/sec",
            "range": "stddev: 0.00012483404464990453",
            "extra": "mean: 2.515627242762938 msec\nrounds: 449"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-with_obstacles]",
            "value": 84.27379672302293,
            "unit": "iter/sec",
            "range": "stddev: 0.009153545370784647",
            "extra": "mean: 11.866084582454892 msec\nrounds: 285"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.Bounded]",
            "value": 50.55642822973134,
            "unit": "iter/sec",
            "range": "stddev: 0.0001608173002615183",
            "extra": "mean: 19.779878346151 msec\nrounds: 52"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.TimeOptimal]",
            "value": 59.41121084385796,
            "unit": "iter/sec",
            "range": "stddev: 0.00011607725248068963",
            "extra": "mean: 16.831840081969677 msec\nrounds: 61"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[ur5]",
            "value": 2061.2549906028507,
            "unit": "iter/sec",
            "range": "stddev: 0.000023914004823897763",
            "extra": "mean: 485.1413360108019 usec\nrounds: 2116"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.Bounded]",
            "value": 8.936738913745552,
            "unit": "iter/sec",
            "range": "stddev: 0.0010406847702840612",
            "extra": "mean: 111.8976406999991 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.TimeOptimal]",
            "value": 11.141157502777435,
            "unit": "iter/sec",
            "range": "stddev: 0.0009086778886381214",
            "extra": "mean: 89.75728058334198 msec\nrounds: 12"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[franka]",
            "value": 1757.9523435515903,
            "unit": "iter/sec",
            "range": "stddev: 0.000011991085613431643",
            "extra": "mean: 568.8436342817465 usec\nrounds: 1791"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[dual]",
            "value": 1151.564224497118,
            "unit": "iter/sec",
            "range": "stddev: 0.00007660084528449224",
            "extra": "mean: 868.3840455678402 usec\nrounds: 1185"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[so101]",
            "value": 2.6027360237654946,
            "unit": "iter/sec",
            "range": "stddev: 0.31428868597677806",
            "extra": "mean: 384.2110728360594 msec\nrounds: 61"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[so101]",
            "value": 6.826730276500979,
            "unit": "iter/sec",
            "range": "stddev: 0.06065320711185913",
            "extra": "mean: 146.48301009374975 msec\nrounds: 32"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[kinova]",
            "value": 1.2529644108964924,
            "unit": "iter/sec",
            "range": "stddev: 0.6687111785822518",
            "extra": "mean: 798.1072656999913 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[kinova]",
            "value": 4.909374129367636,
            "unit": "iter/sec",
            "range": "stddev: 0.13137093907338204",
            "extra": "mean: 203.69195209997315 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[ur5]",
            "value": 0.2874195604634108,
            "unit": "iter/sec",
            "range": "stddev: 6.175206344491081",
            "extra": "mean: 3.479234323466661 sec\nrounds: 15"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[ur5]",
            "value": 2.886858978280991,
            "unit": "iter/sec",
            "range": "stddev: 0.5443781303638888",
            "extra": "mean: 346.3972461153818 msec\nrounds: 26"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[franka]",
            "value": 8.289112604192866,
            "unit": "iter/sec",
            "range": "stddev: 0.07136603903106631",
            "extra": "mean: 120.64017558335156 msec\nrounds: 12"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[franka]",
            "value": 25.78488474595912,
            "unit": "iter/sec",
            "range": "stddev: 0.012804652382956331",
            "extra": "mean: 38.782411085111214 msec\nrounds: 47"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[dual]",
            "value": 0.2580081192162221,
            "unit": "iter/sec",
            "range": "stddev: 5.741150181175795",
            "extra": "mean: 3.8758470200000033 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[dual]",
            "value": 11.312487279620216,
            "unit": "iter/sec",
            "range": "stddev: 0.019338190390518702",
            "extra": "mean: 88.39788945456141 msec\nrounds: 22"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[tiago_pro]",
            "value": 0.22454668809101241,
            "unit": "iter/sec",
            "range": "stddev: 5.121626014971827",
            "extra": "mean: 4.453416830599986 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[tiago_pro]",
            "value": 2.5812582351008513,
            "unit": "iter/sec",
            "range": "stddev: 0.23968166173336855",
            "extra": "mean: 387.407965000034 msec\nrounds: 10"
          }
        ]
      },
      {
        "commit": {
          "author": {
            "email": "4603398+sea-bass@users.noreply.github.com",
            "name": "Sebastian Castro",
            "username": "sea-bass"
          },
          "committer": {
            "email": "noreply@github.com",
            "name": "GitHub",
            "username": "web-flow"
          },
          "distinct": true,
          "id": "ec4d52795f1a1e620d3383174ab57de2339a6af2",
          "message": "Enable constraint projection in path shortcutting (#387)",
          "timestamp": "2026-10-30T17:27:47-05:00",
          "tree_id": "4f3f762302c5362fa0122da87caafdb03fec2761",
          "url": "https://github.com/open-planning/roboplan/commit/ec4d52795f1a1e620d3383174ab57de2339a6af2"
        },
        "date": 1793400222360,
        "tool": "pytest",
        "benches": [
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.Bounded]",
            "value": 165.08739045941778,
            "unit": "iter/sec",
            "range": "stddev: 0.00015647910816994417",
            "extra": "mean: 6.0573978255827035 msec\nrounds: 172"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[ur5-CartesianSpeedMode.TimeOptimal]",
            "value": 221.07245403456093,
            "unit": "iter/sec",
            "range": "stddev: 0.00010964043096962037",
            "extra": "mean: 4.523403896550888 msec\nrounds: 232"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[so101]",
            "value": 6104.407895033211,
            "unit": "iter/sec",
            "range": "stddev: 0.00002130637243664467",
            "extra": "mean: 163.81605180965047 usec\nrounds: 6466"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-no_obstacles]",
            "value": 106.69693658474401,
            "unit": "iter/sec",
            "range": "stddev: 0.0067900633940862175",
            "extra": "mean: 9.372340312748815 msec\nrounds: 502"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-no_obstacles]",
            "value": 624.1825821937043,
            "unit": "iter/sec",
            "range": "stddev: 0.00012650143881951287",
            "extra": "mean: 1.6020953300002003 msec\nrounds: 700"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.Bounded]",
            "value": 137.31838639375334,
            "unit": "iter/sec",
            "range": "stddev: 0.0007520361728647556",
            "extra": "mean: 7.282345986301878 msec\nrounds: 146"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[franka-CartesianSpeedMode.TimeOptimal]",
            "value": 165.22460130893964,
            "unit": "iter/sec",
            "range": "stddev: 0.00020367662245654092",
            "extra": "mean: 6.052367456648806 msec\nrounds: 173"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[kinova]",
            "value": 2492.909577490582,
            "unit": "iter/sec",
            "range": "stddev: 0.000009241713707868172",
            "extra": "mean: 401.1376942948016 usec\nrounds: 2594"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[dual-with_obstacles]",
            "value": 672.3600394819806,
            "unit": "iter/sec",
            "range": "stddev: 0.00011924148893739645",
            "extra": "mean: 1.4872983837207956 msec\nrounds: 774"
          },
          {
            "name": "benchmarks/test_benchmark_scene.py::test_benchmark_has_collisions[so101-with_obstacles]",
            "value": 143.41427484292595,
            "unit": "iter/sec",
            "range": "stddev: 0.0057157279483255235",
            "extra": "mean: 6.972806584945935 msec\nrounds: 465"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.Bounded]",
            "value": 79.51962691083261,
            "unit": "iter/sec",
            "range": "stddev: 0.0012668146611951648",
            "extra": "mean: 12.57551171764583 msec\nrounds: 85"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[stretch-CartesianSpeedMode.TimeOptimal]",
            "value": 96.74231653497189,
            "unit": "iter/sec",
            "range": "stddev: 0.0006101698840118173",
            "extra": "mean: 10.3367382115406 msec\nrounds: 104"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[ur5]",
            "value": 3343.8827407835156,
            "unit": "iter/sec",
            "range": "stddev: 0.000040140888815042696",
            "extra": "mean: 299.0535486796666 usec\nrounds: 3636"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.Bounded]",
            "value": 15.127747740752872,
            "unit": "iter/sec",
            "range": "stddev: 0.005622673389560943",
            "extra": "mean: 66.10369349999701 msec\nrounds: 16"
          },
          {
            "name": "benchmarks/test_benchmark_cartesian_planning.py::test_benchmark_cartesian_planning[tiago_pro-CartesianSpeedMode.TimeOptimal]",
            "value": 19.177511253629135,
            "unit": "iter/sec",
            "range": "stddev: 0.0018221693163797395",
            "extra": "mean: 52.14440949999499 msec\nrounds: 20"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[franka]",
            "value": 2924.6164984291645,
            "unit": "iter/sec",
            "range": "stddev: 0.000014865040522871182",
            "extra": "mean: 341.92517225321956 usec\nrounds: 3013"
          },
          {
            "name": "benchmarks/test_benchmark_oink.py::test_benchmark_oink_solve[dual]",
            "value": 1915.1612944704025,
            "unit": "iter/sec",
            "range": "stddev: 0.000044623955678258124",
            "extra": "mean: 522.1492324888118 usec\nrounds: 2013"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[so101]",
            "value": 3.648114030121696,
            "unit": "iter/sec",
            "range": "stddev: 0.18190040946016753",
            "extra": "mean: 274.11423868421167 msec\nrounds: 19"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[so101]",
            "value": 11.11958973919789,
            "unit": "iter/sec",
            "range": "stddev: 0.056249838968909824",
            "extra": "mean: 89.93137547825887 msec\nrounds: 23"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[kinova]",
            "value": 2.6084685863444093,
            "unit": "iter/sec",
            "range": "stddev: 0.1936585305196019",
            "extra": "mean: 383.3667023000004 msec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[kinova]",
            "value": 8.74015160935228,
            "unit": "iter/sec",
            "range": "stddev: 0.07667278381109255",
            "extra": "mean: 114.41449126923195 msec\nrounds: 26"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[ur5]",
            "value": 0.6140080721167906,
            "unit": "iter/sec",
            "range": "stddev: 3.631908632965423",
            "extra": "mean: 1.6286430837179444 sec\nrounds: 39"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[ur5]",
            "value": 2.417506092736307,
            "unit": "iter/sec",
            "range": "stddev: 0.5904434710345899",
            "extra": "mean: 413.6494228513518 msec\nrounds: 74"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[franka]",
            "value": 12.885504030842801,
            "unit": "iter/sec",
            "range": "stddev: 0.05212226698730724",
            "extra": "mean: 77.60658780645255 msec\nrounds: 31"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[franka]",
            "value": 47.19147123859779,
            "unit": "iter/sec",
            "range": "stddev: 0.004643023140926473",
            "extra": "mean: 21.190269634613603 msec\nrounds: 104"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[dual]",
            "value": 0.181163059665371,
            "unit": "iter/sec",
            "range": "stddev: 5.59301259955832",
            "extra": "mean: 5.519889109000008 sec\nrounds: 32"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[dual]",
            "value": 19.209472058516493,
            "unit": "iter/sec",
            "range": "stddev: 0.01336038687891288",
            "extra": "mean: 52.057651399984806 msec\nrounds: 35"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt[tiago_pro]",
            "value": 0.1595881649634665,
            "unit": "iter/sec",
            "range": "stddev: 6.938939044500221",
            "extra": "mean: 6.266128821200016 sec\nrounds: 10"
          },
          {
            "name": "benchmarks/test_benchmark_rrt.py::test_benchmark_rrt_connect[tiago_pro]",
            "value": 5.548021604206594,
            "unit": "iter/sec",
            "range": "stddev: 0.06497169219768711",
            "extra": "mean: 180.24443150001161 msec\nrounds: 10"
          }
        ]
      }
    ]
  }
}