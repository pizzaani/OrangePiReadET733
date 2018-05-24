CMAKE_MINIMUM_REQUIRED (VERSION 3.5)
PROJECT (readET733)

add_executable(readET733 main.cpp)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/build)

# add wiringPi library
target_link_libraries (readET733 wiringPi)
target_link_libraries (readET733 pthread)

