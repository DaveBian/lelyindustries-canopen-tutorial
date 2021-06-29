
cmake_minimum_required(VERSION 3.13)

project(canopen-tutorial VERSION 1.0 LANGUAGES CXX)

add_executable(tutorial-master master.cpp)
target_include_directories(tutorial-master PUBLIC "${CMAKE_CURRENT_LIST_DIR}")
target_link_libraries(tutorial-master lely-coapp lely-io2 lely-ev lely-co lely-can lely-util lely-libc rt)

add_executable(401-master 401-master.cpp)
target_include_directories(401-master PUBLIC "${CMAKE_CURRENT_LIST_DIR}")
target_link_libraries(401-master lely-coapp lely-io2 lely-ev lely-co lely-can lely-util lely-libc rt)

add_executable(401-slave 401-slave.cpp)
target_include_directories(401-slave PUBLIC "${CMAKE_CURRENT_LIST_DIR}")
target_link_libraries(401-slave lely-coapp lely-io2 lely-ev lely-co lely-can lely-util lely-libc rt)
