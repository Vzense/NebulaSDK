project(ToFFiltersSetGet)
add_executable( ${PROJECT_NAME} ToFFiltersSetGet.cpp)
target_link_libraries(${PROJECT_NAME} Nebula_api DSImgPreProcess)
