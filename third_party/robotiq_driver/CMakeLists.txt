cmake_minimum_required(VERSION 3.14)
project(robotiq_driver_local LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(serial QUIET)
find_package(zerolancom REQUIRED)

# Collect sources. When serial is not available, exclude default_serial.cpp
file(GLOB_RECURSE ROBOTIQ_SOURCES_ALL
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

set(ROBOTIQ_SOURCES ${ROBOTIQ_SOURCES_ALL})
if(NOT serial_FOUND)
  message(STATUS "serial library not found: building robotiq_driver without serial-backed DefaultSerial (FakeDriver only).")
  list(FILTER ROBOTIQ_SOURCES EXCLUDE REGEX ".*/default_serial.cpp$")
  list(FILTER ROBOTIQ_SOURCES EXCLUDE REGEX ".*/default_serial_factory.cpp$")
endif()

add_library(robotiq_driver SHARED ${ROBOTIQ_SOURCES})

target_include_directories(robotiq_driver
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(robotiq_driver
  PUBLIC zerolancom::zerolancom
)

target_link_libraries(robotiq_driver PUBLIC serial::serial)


# Optionally install if desired
# install(TARGETS robotiq_driver LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
