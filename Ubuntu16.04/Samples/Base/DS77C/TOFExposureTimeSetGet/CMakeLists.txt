project(ToFExposureTimeSetGet)
add_executable( ${PROJECT_NAME} ToFExposureTimeSetGet.cpp)
target_link_libraries(${PROJECT_NAME} Nebula_api DSImgPreProcess)
