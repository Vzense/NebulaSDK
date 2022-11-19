project(TOFFiltersSetGet)
add_executable( ${PROJECT_NAME} TOFFiltersSetGet.cpp)
target_link_libraries(${PROJECT_NAME} Nebula_api)
