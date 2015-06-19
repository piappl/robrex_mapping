find_package(Boost REQUIRED COMPONENTS unit_test_framework)

add_definitions (-DBOOST_TEST_DYN_LINK)

add_executable(surfelmappertest surfel_mapper_test.cpp)
target_link_libraries(surfelmappertest surfelmapper ${Boost_LIBRARIES})


