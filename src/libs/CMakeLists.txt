add_library(node2d node2d.cpp)

add_library(dp_2d dp_2d.cpp)
target_link_libraries(dp_2d node2d)