cmake_minimum_required(VERSION 3.12)
project( cg_labs_2 )
find_package( OpenCV REQUIRED )
add_executable( cg_labs_2 main.cpp )
target_link_libraries( cg_labs_2 ${OpenCV_LIBS} )