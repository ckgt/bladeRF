cmake_minimum_required(VERSION 2.8)
project(libbladeRF_test_timestamps C)

include_directories(${libbladeRF_SOURCE_DIR}/include)

set(SRC
    src/main.c
    src/test_rx_gaps.c
    ${BLADERF_HOST_COMMON_SOURCE_DIR}/conversions.c
)

add_executable(libbladeRF_test_timestamps ${SRC})
target_link_libraries(libbladeRF_test_timestamps libbladerf_shared)
