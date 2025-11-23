cmake_minimum_required(VERSION 3.8)
project(solucionador_labirinto)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)

# include directories
include_directories(include)

# add executable
add_executable(rodar_solucao 
  src/main.cpp
  src/algoritmos.cpp
  src/controlador.cpp
)

ament_target_dependencies(rodar_solucao
  rclcpp
  cg_interfaces
)

# install targets
install(TARGETS
  rodar_solucao
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
