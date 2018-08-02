cmake_minimum_required(VERSION 2.6)
project(protocol)

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
endif()

find_package(Boost 1.55 COMPONENTS system filesystem thread REQUIRED)

add_executable(protocol main.cpp protocol_parser.cpp serial_port.cpp utils.cpp crc32.cpp)

target_link_libraries(protocol ${Boost_LIBRARIES})
install(TARGETS protocol RUNTIME DESTINATION bin)
