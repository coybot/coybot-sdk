# GCOPTER (vendored)

Header-only core of [ZJU-FAST-Lab/GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER) at commit
`e0444f6d47b84f972ced91746b05feb36ce1fd4f`, directory `gcopter/include/gcopter/`, copied unchanged except for
`local.patch`.

Licences:

- `LICENSE`: MIT, Copyright (c) 2021 Zhepei Wang and Fei Gao. Covers the repository, including `lbfgs.hpp`
  (which has no notice of its own).
- Every other header carries its own MIT notice (Zhepei Wang), except:
  - `sdlp.hpp`: Michael E. Hohmeyer (1990) and Zhepei Wang (2021), "Permission is granted to modify and
    re-distribute this code in any manner as long as this notice is preserved."
  - `quickhull.hpp`: public domain (Antti Kuukka).

`local.patch` (the only change): `geo_utils::filterLess::operator()` made `const`. `std::set` needs a
const-callable comparator in C++17, which ROS 2 Humble builds with; upstream compiles as C++14.

`voxel_map.hpp` and `voxel_dilater.hpp` are not used by coybot_avoidance (it has its own map) and are kept only so
the directory matches upstream.
