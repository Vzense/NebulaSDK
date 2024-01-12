project(MultiConnectionInMultiThread)
add_executable( ${PROJECT_NAME} MultiConnectionInMultiThread.cpp)
target_link_libraries(${PROJECT_NAME} Nebula_api pthread)