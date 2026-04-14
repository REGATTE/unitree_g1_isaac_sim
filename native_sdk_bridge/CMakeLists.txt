cmake_minimum_required(VERSION 3.16)
project(unitree_g1_native_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(UNITREE_SDK2_ROOT "$ENV{HOME}/unitree_sdk2" CACHE PATH "Path to the unitree_sdk2 checkout/build")
set(UNITREE_SDK2_ARCH "${CMAKE_SYSTEM_PROCESSOR}")

set(UNITREE_SDK2_INCLUDE_DIR "${UNITREE_SDK2_ROOT}/include")
set(UNITREE_SDK2_THIRDPARTY_INCLUDE_DIR "${UNITREE_SDK2_ROOT}/thirdparty/include")
set(UNITREE_SDK2_DDSCXX_INCLUDE_DIR "${UNITREE_SDK2_ROOT}/thirdparty/include/ddscxx")
set(UNITREE_SDK2_LIBRARY "${UNITREE_SDK2_ROOT}/lib/${UNITREE_SDK2_ARCH}/libunitree_sdk2.a")
set(UNITREE_SDK2_DDSC_LIBRARY "${UNITREE_SDK2_ROOT}/thirdparty/lib/${UNITREE_SDK2_ARCH}/libddsc.so")
set(UNITREE_SDK2_DDSCXX_LIBRARY "${UNITREE_SDK2_ROOT}/thirdparty/lib/${UNITREE_SDK2_ARCH}/libddscxx.so")

if(NOT EXISTS "${UNITREE_SDK2_INCLUDE_DIR}")
  message(FATAL_ERROR "unitree_sdk2 headers not found at ${UNITREE_SDK2_INCLUDE_DIR}")
endif()

if(NOT EXISTS "${UNITREE_SDK2_LIBRARY}")
  message(FATAL_ERROR "unitree_sdk2 library not found at ${UNITREE_SDK2_LIBRARY}")
endif()

if(NOT EXISTS "${UNITREE_SDK2_DDSC_LIBRARY}" OR NOT EXISTS "${UNITREE_SDK2_DDSCXX_LIBRARY}")
  message(FATAL_ERROR "unitree_sdk2 DDS thirdparty libraries are missing under ${UNITREE_SDK2_ROOT}/thirdparty/lib/${UNITREE_SDK2_ARCH}")
endif()

find_package(Threads REQUIRED)

add_executable(unitree_g1_native_bridge
  src/main.cpp
  src/bridge_protocol.cpp
  src/lowstate_publisher.cpp
)

target_include_directories(unitree_g1_native_bridge
  PRIVATE
    include
    "${UNITREE_SDK2_INCLUDE_DIR}"
    "${UNITREE_SDK2_THIRDPARTY_INCLUDE_DIR}"
    "${UNITREE_SDK2_DDSCXX_INCLUDE_DIR}"
)

target_link_libraries(unitree_g1_native_bridge
  PRIVATE
    "${UNITREE_SDK2_LIBRARY}"
    "${UNITREE_SDK2_DDSC_LIBRARY}"
    "${UNITREE_SDK2_DDSCXX_LIBRARY}"
    Threads::Threads
)

target_link_options(unitree_g1_native_bridge
  PRIVATE
    "-Wl,--disable-new-dtags"
)

set_target_properties(unitree_g1_native_bridge PROPERTIES
  BUILD_RPATH "${UNITREE_SDK2_ROOT}/thirdparty/lib/${UNITREE_SDK2_ARCH}"
  INSTALL_RPATH "${UNITREE_SDK2_ROOT}/thirdparty/lib/${UNITREE_SDK2_ARCH}"
)
