cmake_minimum_required(VERSION 3.21...3.30)
project(test_range)

find_package(Catch2 CONFIG REQUIRED)


add_executable(${PROJECT_NAME}
    main.cpp
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE Catch2::Catch2
    PRIVATE Catch2::Catch2WithMain
)