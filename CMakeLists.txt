cmake_minimum_required(VERSION 3.2)

project(mavlink_listener CXX C)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wno-address-of-packed-member")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Werror  -Wno-address-of-packed-member")

file(GLOB SOURCE_FILES "*.cpp")

add_executable(mavlink_listener ${SOURCE_FILES})

install(TARGETS mavlink_listener RUNTIME DESTINATION bin)