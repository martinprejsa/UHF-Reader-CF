cmake_minimum_required(VERSION 3.18)

project(chafonlib DESCRIPTION "Interacts with chafon rfid readers"
                  LANGUAGES C)

option(BUILD_TESTS "Build tests" ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 23)
set(CMAKE_INSTALL_RPATH $ORIGIN)

add_library(chafonlib STATIC src/reader.c)

set_target_properties(chafonlib PROPERTIES
                                VERSION 1.0.0
                                SOVERSION 1)

target_include_directories(chafonlib PUBLIC include/)

if(BUILD_TESTS)
  include(CTest)
  add_subdirectory(tests)
endif()