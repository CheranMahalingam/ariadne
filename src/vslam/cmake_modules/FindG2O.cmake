find_path(G2O_INCLUDE_DIR
  NAMES g2o/core/sparse_optimizer.h)
if(G2O_INCLUDE_DIR)
  message(STATUS "Found g2o headers in ${G2O_INCLUDE_DIR}")
endif()

find_library(G2O_CORE_LIB
  NAMES g2o_core)
find_library(G2O_SOLVER_EIGEN_LIB
  NAMES g2o_solver_eigen)
find_library(G2O_STUFF_LIB
  NAMES g2o_stuff)
find_library(G2O_TYPES_SBA_LIB
  NAMES g2o_types_sba)
find_library(G2O_TYPES_SLAM3D_LIB
  NAMES g2o_types_slam3d)

set(G2O_LIBS
  ${G2O_CORE_LIB}
  ${G2O_SOLVER_EIGEN_LIB}
  ${G2O_STUFF_LIB}
  ${G2O_TYPES_SBA_LIB}
  ${G2O_TYPES_SLAM3D_LIB})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(G2O
  REQUIRED_VARS G2O_LIBS G2O_INCLUDE_DIR)
