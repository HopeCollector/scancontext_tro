cmake_minimum_required(VERSION 3.16)

project(scpp LANGUAGES CXX)

include(cmake/CPM.cmake)

find_package(catkin REQUIRED COMPONENTS expr-utils)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

CPMAddPackage("gh:fmtlib/fmt#7.1.3")
CPMAddPackage(
  NAME CLI11
  VERSION 2.5.0
  GITHUB_REPOSITORY CLIUtils/CLI11
  OPTIONS
    "CLI11_INSTALL ON"
)
CPMAddPackage(
  NAME fkYAML
  VERSION 0.4.2
  GITHUB_REPOSITORY fktn-k/fkYAML
  OPTIONS
    "FK_YAML_INSTALL ON"
)

catkin_package(
  INCLUDE_DIRS cpp/module/Scancontext
  CATKIN_DEPENDS
  DEPENDS EIGEN3 PCL OpenCV Eigen3 expr-utils
)

add_executable(expr expr/main.cc cpp/module/Scancontext/Scancontext.cpp)
target_include_directories(expr PRIVATE cpp/module/Scancontext ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(expr PRIVATE ${PCL_LIBRARIES} ${OpenCV_LIBS} expr-utils::dataloader fmt::fmt fkYAML::fkYAML CLI11::CLI11 Eigen3::Eigen)

install(TARGETS expr DESTINATION lib/${PROJECT_NAME})
