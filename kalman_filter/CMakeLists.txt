cmake_minimum_required(VERSION 3.8)
project(kalman_filter)

set(CMAKE_BUILD_TYPE "Release")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(kalman_filter INTERFACE)

target_include_directories(kalman_filter
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR})

ament_target_dependencies(kalman_filter
  INTERFACE
    Eigen3)

target_compile_features(kalman_filter INTERFACE c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS kalman_filter
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  set(test_sources
    matrix_test
    filter_base_test
    standard_base_test
    extended_kalman_filter_test)

  foreach(test_src ${test_sources})
    ament_add_gtest(${test_src}
      test/${test_src}.cpp)

    target_include_directories(${test_src} PRIVATE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
      ${EIGEN3_INCLUDE_DIR})

    target_link_libraries(${test_src} gtest_main)
  endforeach()
endif()

ament_package()
