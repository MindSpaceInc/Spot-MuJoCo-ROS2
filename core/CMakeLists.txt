cmake_minimum_required(VERSION 3.8)
project(core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(rcpputils REQUIRED)

set(TARGET_DEPENDENCE
  ament_cmake
  Boost
  Eigen3
  rcpputils
)

include(cmake/deepbreak_cxx_flags.cmake)

file(GLOB_RECURSE source ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${source})
target_link_libraries(${PROJECT_NAME}
    ${Boost_LIBRARIES}
)
target_compile_options(${PROJECT_NAME} PUBLIC ${DEEPBREAK_CXX_FLAGS})   

target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
    
ament_target_dependencies(${PROJECT_NAME} ${TARGET_DEPENDENCE})
ament_export_targets(${PROJECT_NAME}_Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(${TARGET_DEPENDENCE})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
