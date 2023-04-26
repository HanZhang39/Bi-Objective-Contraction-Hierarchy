project (Bi_objective_ch)

cmake_minimum_required (VERSION 2.6)

set(CMAKE_CXX_STANDARD 14)
SET(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
SET (CMAKE_LINKER  "/usr/bin/ld")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -Wall")

include_directories("inc")

file(GLOB SOURCES "src/BiCriteria/*.cpp" "src/Utils/*.cpp")

# Find Boost
find_package( Boost REQUIRED COMPONENTS program_options log thread filesystem system)
include_directories( ${Boost_INCLUDE_DIRS} )

add_executable(boa src/boa_driver.cpp ${SOURCES})
add_executable(contraction src/build_contraction.cpp ${SOURCES})
add_executable(ch_solver src/contraction_solver.cpp ${SOURCES})

target_link_libraries(boa ${Boost_LIBRARIES})
target_link_libraries(contraction ${Boost_LIBRARIES})
target_link_libraries(ch_solver ${Boost_LIBRARIES})
