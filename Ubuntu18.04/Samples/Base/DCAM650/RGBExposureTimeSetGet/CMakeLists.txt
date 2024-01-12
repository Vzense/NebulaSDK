project(RGBExposureTimeSetGet)
add_executable( ${PROJECT_NAME} RGBExposureTimeSetGet.cpp)
target_link_libraries(${PROJECT_NAME} Nebula_api)