include_directories(${PROJECT_INCLUDE_DIR})

file(GLOB Power_SOURCES ${PROJECT_SOURCE_DIR}/examples/MINLP/Power/*.cpp)
file(GLOB ODO_SOURCES ${PROJECT_SOURCE_DIR}/examples/MINLP/Power/ODO/*.cpp)

add_executable(odo ${ODO_SOURCES} ${Power_SOURCES})

target_link_libraries(odo gravity)
