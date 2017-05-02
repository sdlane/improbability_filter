cmake_minimum_required(VERSION 2.8.11)

PROJECT(robocup-ssl)

# Flag for toggling test compilation on or off.
OPTION(test "Build all tests." ON)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/third_party/")

INCLUDE(FindProtobuf)

FIND_PACKAGE(Protobuf REQUIRED)

SET(CMAKE_CXX_FLAGS "-std=c++11 -g")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -rdynamic")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -rdynamic")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -fno-builtin-malloc")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fno-builtin-calloc")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fno-builtin-realloc")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fno-builtin-free -O3 -DNDEBUG")
  SET(BUILD_SPECIFIC_LIBRARIES "tcmalloc")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -rdynamic")
  SET(BUILD_SPECIFIC_LIBRARIES "")
ENDIF()

SET(libs ${PROTOBUF_LIBRARY} rt glog pthread popt ${BUILD_SPECIFIC_LIBRARIES})

FILE(GLOB PROTO_FILES src/proto/*.proto)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

INCLUDE_DIRECTORIES(src/)
INCLUDE_DIRECTORIES(third_party/)
INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIR})

ADD_LIBRARY(protobuf_library ${PROTO_SRCS})
TARGET_LINK_LIBRARIES(protobuf_library ${libs})

ADD_LIBRARY(shared_library
            src/net/netraw.cc
            src/util/helpers.cc
            src/util/pthread_utils.cc
            src/util/timer.cc
            src/util/colorize.cc
            src/util/random.cc)
TARGET_LINK_LIBRARIES(shared_library protobuf_library ${libs})

ADD_LIBRARY(main_library
  src/soccer/main.cc
  src/soccer/sslvisioninputhandler.cc
  src/soccer/kalmanupdate.cc)
TARGET_LINK_LIBRARIES(main_library protobuf_library
                      shared_library ${libs})

ADD_LIBRARY(state_estimation_library
  src/state_estimation/kalman_filter.cc
  src/state_estimation/extended_kalman_filter.cc
  src/state_estimation/improbability_filter.cc)

ADD_EXECUTABLE(soccer src/soccer/main.cc)
TARGET_LINK_LIBRARIES(soccer main_library protobuf_library
                      shared_library state_estimation_library
                      ${libs})