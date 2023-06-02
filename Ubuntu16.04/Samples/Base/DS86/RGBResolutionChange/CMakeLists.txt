project(RGBResolutionChange)
add_executable( ${PROJECT_NAME} RGBResolutionChange.cpp)
target_link_libraries(${PROJECT_NAME} Nebula_api DSImgPreProcess)