project(TOFExposureTimeSetGet)
add_executable( ${PROJECT_NAME} TOFExposureTimeSetGet.cpp)
target_link_libraries(${PROJECT_NAME} Nebula_api)
