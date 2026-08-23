cmake_minimum_required(VERSION 3.8)
project(drone_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)



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
endif()



include_directories(include)

add_library(drone_lib SHARED
  src/Drone.cpp
  src/movement.cpp
  src/motion_policy.cpp
  src/transformations.cpp
)

ament_target_dependencies(drone_lib
  Eigen3
  px4_msgs
  rclcpp
  tf2
  custom_msgs
)

add_executable(system_health
  src/system_health.cpp
)

ament_target_dependencies(system_health
  rclcpp
  custom_msgs
)

ament_export_targets(drone_lib HAS_LIBRARY_TARGET)

ament_export_dependencies(Eigen3 px4_msgs rclcpp tf2 custom_msgs)
 
# install the include/my_cpp_library directory to the install/include/my_cpp_library
install(
  DIRECTORY include/drone
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)
 
install(
  TARGETS drone_lib
  EXPORT drone_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS system_health
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# O bloco de testes fica AQUI, e nao junto com o lint la em cima: ele precisa
# do alvo `drone_lib`, que so existe a partir do add_library acima. Um
# target_link_libraries contra um alvo ainda nao declarado e erro de CMake.
if(BUILD_TESTING)
  # ---------------------------------------------------------------------------
  # O primeiro teste deste pacote.
  #
  # O drone_lib e a fronteira com o PX4 -- todo comando que chega ao drone passa
  # por aqui -- e nao tinha nenhum. A razao era real: a classe `Drone` e um
  # rclcpp::Node que gira o proprio executor numa thread, e testar qualquer
  # coisa que a toque exigiria subir ROS.
  #
  # A MotionPolicy foi escrita separando a DECISAO da PUBLICACAO justamente
  # para nao herdar esse problema: `decidir()` nao conhece ROS, e e nela que
  # mora a garantia de que a politica axial nunca desloca o drone de lado.
  # ---------------------------------------------------------------------------
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_motion_policy test/test_motion_policy.cpp)
  if(TARGET test_motion_policy)
    target_include_directories(test_motion_policy PRIVATE include)
    target_link_libraries(test_motion_policy drone_lib)
    ament_target_dependencies(test_motion_policy Eigen3 px4_msgs rclcpp tf2 custom_msgs)
  endif()
endif()

ament_package()
