#pragma once
// SmallIVox uses only these dependencies from the production PCH. No map
// implementation is replaced; this shim keeps the standalone probe ROS-free.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <list>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <ankerl/unordered_dense.h>
